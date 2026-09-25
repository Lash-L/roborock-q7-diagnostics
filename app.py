"""Roborock Q7 & 3irobotix B01 Simple Live Diagnostic Tool (Port 6001).

Connects to the vacuum on 192.168.5.1:6001 to read telemetry, cliff sensors,
bumpers, battery, and provide DIY repair instructions.
Supports Roborock Q7 / Q7 Max (sc05) and 3irobotix B01 ODM platforms.
"""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import http.server
import json
from pathlib import Path
import sys
import threading
import time
from typing import Any, Dict, List, Optional
import urllib.parse
import webbrowser

from diagnostic_engine import evaluate_diagnostics
from q7_protocol import (
    CMD_AUDIO_META,
    CMD_CERT_META,
    CMD_LANGUAGE,
    CMD_MODEL,
    CMD_READ_PASSWORD,
    CMD_SKU,
    PROTOBUF_TAG_METADATA,
    Q7DiagnosticClient,
    TelemetryFrame,
)

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

# Fixed target for Q7 diagnostic port
ROBOT_HOST = "192.168.5.1"
ROBOT_PORT = 6001

CLIENT: Optional[Q7DiagnosticClient] = None
CURRENT_DATA: Dict[str, Any] = {
    "host": ROBOT_HOST,
    "port": ROBOT_PORT,
    "connected": False,
    "started_at": None,
    "frames": [],
    "latest_frame": None,
    "getters": {},
    "discovery": {},
    "services": {},
}
CURRENT_DIAGNOSIS: Optional[Dict[str, Any]] = None
CLIFF_ADC_DATA: Optional[Dict[str, Any]] = None
SSH_PASSWORD: Optional[str] = None

SSE_CLIENTS: List[Any] = []
SSE_LOCK = threading.Lock()


def broadcast_telemetry(frame: TelemetryFrame) -> None:
    """Send live frame to connected browsers."""
    global CURRENT_DATA, CURRENT_DIAGNOSIS
    frame_dict = asdict(frame)

    CURRENT_DATA["latest_frame"] = frame_dict
    CURRENT_DATA["frames"].append(frame_dict)
    if len(CURRENT_DATA["frames"]) > 100:
        CURRENT_DATA["frames"].pop(0)

    diag = evaluate_diagnostics(
        CURRENT_DATA,
        cliff_adc_data=CLIFF_ADC_DATA,
        ssh_password=SSH_PASSWORD,
    )
    CURRENT_DIAGNOSIS = asdict(diag)

    msg = json.dumps({"type": "telemetry", "frame": frame_dict})
    data_bytes = f"data: {msg}\n\n".encode("utf-8")

    with SSE_LOCK:
        dead = []
        for wfile in SSE_CLIENTS:
            try:
                wfile.write(data_bytes)
                wfile.flush()
            except Exception:
                dead.append(wfile)
        for d in dead:
            SSE_CLIENTS.remove(d)


class SimpleDiagnosticHandler(http.server.BaseHTTPRequestHandler):
    """Simple HTTP Handler for Q7 diagnostics."""

    def log_message(self, format: str, *args: Any) -> None:
        pass

    def send_json(self, status: int, data: Any) -> None:
        blob = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(blob)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(blob)

    def do_GET(self) -> None:
        path = urllib.parse.urlparse(self.path).path

        if path in ("/", "/index.html"):
            index_path = STATIC_DIR / "index.html"
            if index_path.exists():
                content = index_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return
            self.send_error(404, "Index file not found")
            return

        if path == "/api/status":
            global CLIENT, CURRENT_DATA, CURRENT_DIAGNOSIS
            is_live = CLIENT is not None and getattr(CLIENT, "_is_streaming", False)
            self.send_json(
                200,
                {
                    "connected": is_live,
                    "host": ROBOT_HOST,
                    "port": ROBOT_PORT,
                    "passive_frame_count": len(CURRENT_DATA.get("frames", [])),
                    "latest_frame": CURRENT_DATA.get("latest_frame"),
                    "has_diagnosis": bool(CURRENT_DIAGNOSIS),
                    "tag_metadata": PROTOBUF_TAG_METADATA,
                },
            )
            return

        if path == "/api/tags":
            self.send_json(200, {"tags": PROTOBUF_TAG_METADATA})
            return

        if path == "/api/telemetry":
            self.send_json(
                200,
                {
                    "data": CURRENT_DATA,
                    "diagnosis": CURRENT_DIAGNOSIS,
                    "cliff_adc": CLIFF_ADC_DATA,
                    "tag_metadata": PROTOBUF_TAG_METADATA,
                },
            )
            return

        if path == "/api/export":
            export_payload = {
                "exported_at": datetime.now(timezone.utc).isoformat(),
                "target": f"{ROBOT_HOST}:{ROBOT_PORT}",
                "data": CURRENT_DATA,
                "diagnosis": CURRENT_DIAGNOSIS,
                "cliff_adc": CLIFF_ADC_DATA,
            }
            blob = json.dumps(export_payload, indent=2).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header(
                "Content-Disposition",
                'attachment; filename="q7_diagnostic_report.json"',
            )
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
            return

        if path == "/api/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            with SSE_LOCK:
                SSE_CLIENTS.append(self.wfile)

            try:
                while True:
                    time.sleep(1)
            except Exception:
                with SSE_LOCK:
                    if self.wfile in SSE_CLIENTS:
                        SSE_CLIENTS.remove(self.wfile)
            return

        self.send_error(404, "Not Found")

    def do_POST(self) -> None:
        path = urllib.parse.urlparse(self.path).path
        global CURRENT_DATA, CURRENT_DIAGNOSIS, CLIFF_ADC_DATA, SSH_PASSWORD, CLIENT

        if path == "/api/connect":
            try:
                if CLIENT:
                    CLIENT.stop_streaming()

                client = Q7DiagnosticClient(host=ROBOT_HOST, port=ROBOT_PORT)
                check = client.check_ports(ports=(ROBOT_PORT,), timeout=2.5)
                if check.get(str(ROBOT_PORT), {}).get("status") != "open":
                    self.send_json(
                        400,
                        {
                            "success": False,
                            "error": f"Cannot reach {ROBOT_HOST}:{ROBOT_PORT}. Ensure your Wi-Fi is connected to the robot's network.",
                        },
                    )
                    return

                CLIENT = client
                CLIENT.add_listener(broadcast_telemetry)
                CLIENT.start_streaming()

                CURRENT_DATA["connected"] = True
                CURRENT_DATA["started_at"] = datetime.now(timezone.utc).isoformat()
                CURRENT_DATA["frames"] = []

                self.send_json(200, {"success": True})
            except Exception as e:
                self.send_json(500, {"success": False, "error": str(e)})
            return

        if path == "/api/disconnect":
            if CLIENT:
                CLIENT.stop_streaming()
                CLIENT = None
            CURRENT_DATA["connected"] = False
            self.send_json(200, {"success": True})
            return

        if path == "/api/read_all_6001":
            client = CLIENT or Q7DiagnosticClient(host=ROBOT_HOST, port=ROBOT_PORT)
            try:
                # 1. Getters
                getters = {}
                for cmd, name in (
                    (CMD_MODEL, "model"),
                    (CMD_SKU, "sku"),
                    (CMD_LANGUAGE, "language"),
                    (CMD_AUDIO_META, "audio_metadata"),
                    (CMD_CERT_META, "certificate_metadata"),
                ):
                    getters[name] = client.query_getter(cmd, timeout=3.0)

                # 2. Stored factory password (0xF7)
                pwd_res = client.query_getter(CMD_READ_PASSWORD, timeout=3.0)
                if pwd_res.get("status") == "reply" and "ssh_password" in pwd_res.get("decoded", {}):
                    SSH_PASSWORD = pwd_res["decoded"]["ssh_password"]
                    getters["password"] = pwd_res

                # 3. Cliff ADC & Thresholds
                CLIFF_ADC_DATA = client.query_cliff_adc(timeout=3.0)

                # 4. UDP Discovery
                discovery = client.query_discovery_8899(timeout=3.0)

                # 5. Service ports
                services = client.check_ports(timeout=2.0)

                CURRENT_DATA.update(
                    getters=getters,
                    discovery=discovery,
                    services=services,
                )

                diag = evaluate_diagnostics(
                    CURRENT_DATA,
                    cliff_adc_data=CLIFF_ADC_DATA,
                    ssh_password=SSH_PASSWORD,
                )
                CURRENT_DIAGNOSIS = asdict(diag)

                self.send_json(
                    200,
                    {
                        "success": True,
                        "data": CURRENT_DATA,
                        "diagnosis": CURRENT_DIAGNOSIS,
                        "cliff_adc": CLIFF_ADC_DATA,
                    },
                )
            except Exception as e:
                self.send_json(500, {"success": False, "error": str(e)})
            return

        if path == "/api/run_cliff_read":
            client = CLIENT or Q7DiagnosticClient(host=ROBOT_HOST, port=ROBOT_PORT)
            try:
                CLIFF_ADC_DATA = client.query_cliff_adc(timeout=3.0)
                if CURRENT_DATA:
                    diag = evaluate_diagnostics(
                        CURRENT_DATA,
                        cliff_adc_data=CLIFF_ADC_DATA,
                        ssh_password=SSH_PASSWORD,
                    )
                    CURRENT_DIAGNOSIS = asdict(diag)
                self.send_json(
                    200,
                    {
                        "success": True,
                        "cliff_adc": CLIFF_ADC_DATA,
                        "diagnosis": CURRENT_DIAGNOSIS,
                    },
                )
            except Exception as e:
                self.send_json(500, {"success": False, "error": str(e)})
            return

        if path == "/api/actuate":
            client = CLIENT or Q7DiagnosticClient(host=ROBOT_HOST, port=ROBOT_PORT)
            content_length = int(self.headers.get("Content-Length", 0))
            body_bytes = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                params = json.loads(body_bytes.decode("utf-8")) if body_bytes else {}
                component = params.get("component", "stop")
                value = int(params.get("value", 50))
                duration_s = float(params.get("duration", 2.0))
                res = client.actuate_motor(component, value, duration_s)
                self.send_json(200, res)
            except Exception as e:
                self.send_json(500, {"success": False, "error": str(e)})
            return

        self.send_error(404, "Not Found")


def run_server(port: int = 8080, max_port_tries: int = 20) -> None:
    """Start local web server with automatic port fallback."""
    http.server.ThreadingHTTPServer.allow_reuse_address = True
    httpd: Optional[http.server.ThreadingHTTPServer] = None
    actual_port = port

    for p in range(port, port + max_port_tries):
        try:
            httpd = http.server.ThreadingHTTPServer(("127.0.0.1", p), SimpleDiagnosticHandler)
            actual_port = p
            break
        except OSError as e:
            if getattr(e, "errno", None) in (10048, 98, 48) or "address already in use" in str(e).lower() or "only one usage" in str(e).lower():
                print(f"[Port Fallback] Port {p} is currently in use, trying {p + 1}...")
                continue
            raise

    if httpd is None:
        try:
            httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), SimpleDiagnosticHandler)
            actual_port = httpd.server_address[1]
            print(f"[Port Fallback] Bound to dynamic ephemeral port {actual_port}")
        except Exception as e:
            print(f"[Error] Failed to bind local server: {e}")
            raise

    url = f"http://127.0.0.1:{actual_port}"
    print(f"Roborock Q7 & 3irobotix B01 Diagnostic Tool: {url}")
    print(f"Server bound on port {actual_port} (auto-fallback active)")
    print(f"Targeting vacuum on {ROBOT_HOST}:{ROBOT_PORT}")

    threading.Timer(0.8, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if CLIENT:
            CLIENT.stop_streaming()
        httpd.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Roborock Q7 & 3irobotix B01 Diagnostic Tool")
    parser.add_argument("--port", type=int, default=8080, help="Web server port (default: 8080)")
    args = parser.parse_args()
    run_server(port=args.port)


if __name__ == "__main__":
    main()

