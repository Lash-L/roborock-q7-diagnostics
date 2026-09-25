"""Roborock Q7 Hardware Diagnostic and Repair Recommendation Engine.

Analyzes telemetry and getter data from port 6001, detects active hardware
faults, computes subsystem health scores, and generates actionable, step-by-step
DIY repair guides for Roborock Q7 owners.
"""

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

# Known reference checksums for Q7 sc05 factory dump
EXPECTED_CHECKSUMS = {
    "device.json": "98f02aaf9ee4d6ec84350eca569061fc",
    "device.key": "b633160bf83720d92ce47eb517433171",
    "device.crt": "eb1339a93eb0ea9feadfd6e7dfeb9feb",
    "device_cfg.json": "7dba8eeeb381bf5ee0857a8d5bf35bd9",
}

# Calibrated factory threshold defaults from vendor storage
DEFAULT_THRESHOLDS = {
    "left": 38,
    "left_front": 20,
    "right_front": 20,
    "right": 58,
    "left_back": 20,
    "right_back": 20,
}


@dataclass
class DiagnosticIssue:
    issue_id: str
    component: str  # 'cliff', 'bumper', 'dustbin', 'battery', 'lidar', 'system'
    severity: str  # 'critical', 'warning', 'info'
    title: str
    description: str
    technical_detail: str
    repair_steps: List[str]
    part_recommendations: Optional[str] = None


@dataclass
class SubsystemHealth:
    name: str
    score: int  # 0 to 100
    status: str  # 'nominal', 'warning', 'error', 'unknown'
    summary: str


@dataclass
class DiagnosticReport:
    overall_score: int  # 0 to 100
    overall_status: str  # 'nominal', 'attention_required', 'fault_detected'
    model: str
    sku: str
    serial: str
    firmware_version: str
    mcu_version: str
    subsystems: Dict[str, SubsystemHealth]
    issues: List[DiagnosticIssue]
    ssh_password: Optional[str] = None
    root_access_guide: Optional[str] = None


def evaluate_diagnostics(data: Dict[str, Any], cliff_adc_data: Optional[Dict[str, Any]] = None, ssh_password: Optional[str] = None) -> DiagnosticReport:
    """Evaluates all diagnostic data and produces a comprehensive report."""
    issues: List[DiagnosticIssue] = []
    subsystems: Dict[str, SubsystemHealth] = {}

    latest_frame = data.get("latest_frame") or {}
    getters = data.get("getters") or {}
    discovery = data.get("discovery") or {}

    model = (
        getters.get("model", {}).get("decoded", {}).get("value")
        or "roborock.vacuum.sc05 (Q7)"
    )
    sku = getters.get("sku", {}).get("decoded", {}).get("value") or "S031APRO"
    serial = discovery.get("serial") or "RCO4HP53522856"
    fw_ver = discovery.get("versions", {}).get("software") or "03.01.74"
    mcu_ver = discovery.get("versions", {}).get("mcu") or "1.0.0_21070308"

    # Password resolution
    if not ssh_password:
        ssh_password = (
            getters.get("password", {}).get("decoded", {}).get("ssh_password")
            or getters.get("0xf7", {}).get("decoded", {}).get("ssh_password")
        )

    # 1. EVALUATE CLIFF SENSORS (Ground Detection)
    cliff_data = latest_frame.get("cliff", {})
    wheel_data = latest_frame.get("wheel", {})
    ahrs_data = latest_frame.get("ahrs", {})
    cliff_score = 100
    cliff_status = "nominal"
    cliff_issues = []

    # Check passive triggers
    triggered_sensors = []
    if cliff_data.get("left_triggered"):
        triggered_sensors.append("Left")
    if cliff_data.get("left_front_triggered"):
        triggered_sensors.append("Left-Front")
    if cliff_data.get("right_front_triggered"):
        triggered_sensors.append("Right-Front")
    if cliff_data.get("right_triggered"):
        triggered_sensors.append("Right")

    # Orientation & wheel suspension status
    is_suspended = bool(wheel_data.get("left_wheel_up") or wheel_data.get("right_wheel_up"))
    accel_z = ahrs_data.get("accel_z")
    is_inverted = bool(accel_z is not None and accel_z < -500)

    # Check active ADC readings if available
    adc_readings = {}
    thresholds = {}
    if cliff_adc_data and "data" in cliff_adc_data:
        adc_readings = cliff_adc_data["data"].get("adc", {})
        thresholds = cliff_adc_data["data"].get("threshold", {})
        active_cliff_names = ("left", "left_front", "right_front", "right")
        zero_adc_count = sum(1 for name in active_cliff_names if adc_readings.get(name) == 0)

        # Check if passive telemetry confirms robot is flat on floor (all triggers clear & wheels compressed)
        passive_is_flat_and_clear = (
            not is_suspended
            and not is_inverted
            and not any(cliff_data.get(f"{k}_triggered", False) for k in ("left", "left_front", "right_front", "right"))
            and any(latest_frame.get("cliff", {}).values())
        )

        for name in active_cliff_names:
            if name not in adc_readings:
                continue
            adc_val = adc_readings[name]
            thresh = thresholds.get(name, DEFAULT_THRESHOLDS.get(name, 25))

            if adc_val == 0:
                if is_suspended or is_inverted or zero_adc_count >= 2:
                    # Inverted or elevated: 0 ADC is the normal optical reflection failure into open air
                    label = name.replace("_", " ").title()
                    if label not in triggered_sensors:
                        triggered_sensors.append(label)
                elif passive_is_flat_and_clear:
                    # Stale cached ADC read from earlier upside-down session; live telemetry shows clear floor
                    pass
                else:
                    # Isolated 0 ADC reading while vacuum is confirmed flat on ground
                    issues.append(
                        DiagnosticIssue(
                            issue_id=f"cliff_dead_{name}",
                            component="cliff",
                            severity="critical",
                            title=f"Cliff Sensor Fault ({name.replace('_', ' ').title()})",
                            description=f"Sensor reads 0 ADC while on floor. Emitter IR diode is unpowered or harness connector loose.",
                            technical_detail=f"ADC={adc_val}, Threshold={thresh}. An isolated 0 reading on flat ground indicates open circuit.",
                            repair_steps=[
                                "Unscrew the bottom base plate to access the front cliff harness.",
                                "Check for a pinched or severed wire on the 4-pin connector leading to the sensor module.",
                                f"Inspect the {name} cliff optical assembly for liquid damage or hair wrapped around the diode.",
                                "Replace the cliff sensor assembly if wiring is intact but ADC remains 0.",
                            ],
                            part_recommendations="Roborock Q7 / Q7 Max Cliff Sensor Harness (Part # 9.01.0772 / SC05 Cliff Module)",
                        )
                    )
                    cliff_score -= 30
            elif adc_val <= thresh:
                label = name.replace("_", " ").title()
                if label not in triggered_sensors:
                    triggered_sensors.append(label)

    if is_inverted or is_suspended:
        issues.append(
            DiagnosticIssue(
                issue_id="vacuum_elevated_inverted",
                component="cliff",
                severity="info",
                title="Vacuum Inverted or Elevated (Optical Cliff Drop Detected)",
                description="The robot is upside down or raised in the air. Cliff sensors point into open air (reading 0 ADC), and wheel suspension microswitches are open. MCU safety interlock prevents motors from spinning.",
                technical_detail=f"Inverted={is_inverted}, Suspended={is_suspended}. Normal optical behavior: IR light does not reflect back from open space.",
                repair_steps=[
                    "Place the vacuum flat on level flooring so the suspension wheels are compressed.",
                    "Click 'Read Cliff ADC' once resting on the floor to refresh calibrated ground reflections.",
                    "Active motor testing requires either flat placement or manually depressing the wheel drop switches.",
                ],
            )
        )
        cliff_status = "warning"
        cliff_score = max(cliff_score, 70)
    elif triggered_sensors:
        cliff_score -= 25 * len(triggered_sensors)
        cliff_score = max(cliff_score, 20)
        cliff_status = "warning" if cliff_score > 40 else "error"
        issues.append(
            DiagnosticIssue(
                issue_id="cliff_triggered_general",
                component="cliff",
                severity="critical" if len(triggered_sensors) > 1 else "warning",
                title="Cliff Drop Sensor Triggered (Error 4 Risk)",
                description=f"Ground detection triggered on: {', '.join(triggered_sensors)}. Robot will refuse to move forward or will back away unexpectedly.",
                technical_detail="Infrared light reflection is below calibrated threshold. Often caused by dust buildup, dark/black carpet, or dirty lens.",
                repair_steps=[
                    "Wipe the 4 clear plastic window apertures on the front lower rim using a soft microfiber cloth with a drop of 90%+ isopropyl alcohol.",
                    "If the robot is sitting on a black or high-contrast rug, test moving it to a light-colored solid floor. Black surfaces absorb IR light, causing false cliff triggers.",
                    "Verify the robot suspension wheels are not collapsed, which alters the focal distance to the floor.",
                    "If the issue persists on light flooring, check the sensor ADC calibration via diagnostic mode.",
                ],
                part_recommendations="Roborock Q7 Cliff Sensor Module (Left & Right pairs)",
            )
        )

    subsystems["cliff"] = SubsystemHealth(
        name="Cliff & Ground Sensors",
        score=max(0, cliff_score),
        status=cliff_status,
        summary="All 4 sensors clear" if not triggered_sensors else f"Triggered on {len(triggered_sensors)} sensor(s)",
    )

    # 2. EVALUATE BUMPERS (Front & LiDAR)
    bumper_data = latest_frame.get("bumper", {})
    bumper_score = 100
    bumper_status = "nominal"
    stuck_bumpers = []

    if bumper_data.get("left_pressed"):
        stuck_bumpers.append("Left Front Bumper")
    if bumper_data.get("right_pressed"):
        stuck_bumpers.append("Right Front Bumper")
    if bumper_data.get("lidar_pressed"):
        stuck_bumpers.append("LiDAR LDS Turret Bumper")

    if stuck_bumpers:
        bumper_score = 30
        bumper_status = "error"
        issues.append(
            DiagnosticIssue(
                issue_id="bumper_stuck",
                component="bumper",
                severity="critical",
                title="Bumper Sensor Stuck / Depressed (Error 2)",
                description=f"Sensors report active collision on: {', '.join(stuck_bumpers)}. Robot will back away repeatedly or spin in place.",
                technical_detail="Microswitch circuit is closed. Normal state when resting is open.",
                repair_steps=[
                    "Gently tap along the full perimeter of the bumper to check if it pops outward smoothly.",
                    "Inspect the space between the bumper skirt and the chassis for trapped debris (pet kibble, toys, dirt).",
                    "If the bumper does not rebound, a return spring has popped out or the plastic guide rail is misaligned.",
                    "Remove the bottom front edge screws, unclip the bumper cover, and inspect the internal microswitches. Clean microswitch plungers with electrical contact cleaner.",
                    "If LiDAR turret bumper is stuck: verify the circular orange/black laser dome moves freely and clicks when pressed down.",
                ],
                part_recommendations="Roborock Q7 Bumper Spring Set / Omron Bumper Microswitches (D2F series)",
            )
        )

    subsystems["bumper"] = SubsystemHealth(
        name="Collision & Bumpers",
        score=bumper_score,
        status=bumper_status,
        summary="Clear and responsive" if not stuck_bumpers else f"Stuck: {', '.join(stuck_bumpers)}",
    )

    # 3. EVALUATE DUSTBIN (Dustbox)
    dustbox_present = latest_frame.get("dustbox_present", True)
    dustbox_score = 100 if dustbox_present else 40
    dustbox_status = "nominal" if dustbox_present else "warning"

    if not dustbox_present:
        issues.append(
            DiagnosticIssue(
                issue_id="dustbin_missing",
                component="dustbin",
                severity="warning",
                title="Dustbin Not Detected (Error 10)",
                description="The internal reed switch / Hall effect sensor does not detect the dustbin magnetic marker.",
                technical_detail="Tag 1 reported 0 (dustbox absent).",
                repair_steps=[
                    "Ensure the dustbin is inserted and clicked firmly into place.",
                    "Examine the bottom-left outer corner of the clear dustbin for the small rectangular silver magnet. If it fell out, insert a 5x2mm neodymium magnet.",
                    "Check the chassis receptacle for dried liquid or dust preventing full seating.",
                ],
                part_recommendations="Roborock Q7 Replacement Dustbin with Filter & Magnet",
            )
        )

    subsystems["dustbin"] = SubsystemHealth(
        name="Dustbin & Filter",
        score=dustbox_score,
        status=dustbox_status,
        summary="Installed and detected" if dustbox_present else "Missing / Magnet not sensed",
    )

    # 4. EVALUATE BATTERY & POWER
    battery_data = latest_frame.get("battery", {})
    bat_score = 100
    bat_status = "nominal"
    volt_raw = battery_data.get("voltage_raw")
    curr_raw = battery_data.get("current_raw")
    est_volt = battery_data.get("estimated_voltage_v")

    if est_volt is not None:
        if est_volt < 12.0:
            bat_score = 30
            bat_status = "error"
            issues.append(
                DiagnosticIssue(
                    issue_id="battery_critically_low",
                    component="battery",
                    severity="critical",
                    title="Battery Voltage Depleted (< 12V)",
                    description=f"Pack voltage reads {est_volt}V. Roborock 4S Li-ion battery is critically low or cells are degraded.",
                    technical_detail=f"Reported raw voltage {volt_raw} (~{est_volt}V). Nominal full charge is 14.4V-16.8V.",
                    repair_steps=[
                        "Place the vacuum directly against the dock charging pins manually and verify the dock LED dims or pulses.",
                        "Clean the two silver contact pads underneath the vacuum using rubbing alcohol to remove oxidation.",
                        "Inspect dock spring-loaded charging blades for corrosion or bent pins.",
                        "If battery fails to charge above 12.5V after 4 hours on dock, replace the battery pack.",
                    ],
                    part_recommendations="Roborock 14.4V 5200mAh Li-ion Battery Pack (BRR-2P4S-5200S)",
                )
            )
        elif est_volt < 13.5:
            bat_score = 70
            bat_status = "warning"

    bat_pct = battery_data.get("percentage")
    bat_summary = f"{est_volt}V"
    if bat_pct is not None:
        bat_summary += f" ({bat_pct}%)"
    if battery_data.get("is_charging"):
        bat_summary += " [Charging]"

    subsystems["battery"] = SubsystemHealth(
        name="Battery & Power",
        score=bat_score,
        status=bat_status,
        summary=bat_summary if est_volt else "Status OK",
    )

    # 5. EVALUATE MOTORS & DRIVETRAIN (Tags 6, 7, 8, 9, 10)
    brush_data = latest_frame.get("brush", {})
    wheel_data = latest_frame.get("wheel", {})
    blower_data = latest_frame.get("blower", {})
    motor_score = 100
    motor_status = "nominal"
    stalled_motors = []

    if brush_data.get("rolling_stalled"):
        stalled_motors.append("Main Roller Brush")
        issues.append(
            DiagnosticIssue(
                issue_id="main_brush_stalled",
                component="brush",
                severity="critical",
                title="Main Roller Brush Stalled (Error 2)",
                description="Main brush roller motor has seized or is jammed by hair, threads, or rug tassels.",
                technical_detail="Tag 7 reported rolling_stalled=True.",
                repair_steps=[
                    "Remove the orange main brush cover plate by squeezing the two release latches.",
                    "Lift out the red rubber roller and use the cleaning tool or scissors to cut hair wrapped around the end bearings.",
                    "Pull off both plastic end-caps of the roller to clear hair accumulated inside the brass bushings.",
                    "Spin the square drive gear inside the cavity by hand to ensure the gearbox turns freely.",
                ],
                part_recommendations="Roborock Q7 All-Rubber Roller Brush & End Bearing Caps",
            )
        )
    if brush_data.get("side_stalled"):
        stalled_motors.append("Side Brush")
        issues.append(
            DiagnosticIssue(
                issue_id="side_brush_stalled",
                component="brush",
                severity="warning",
                title="Side Brush Motor Stalled",
                description="Side brush motor is overloaded or wrapped with hair.",
                technical_detail="Tag 6 reported side_stalled=True.",
                repair_steps=[
                    "Unscrew the center Phillips screw on the 5-arm silicone side brush.",
                    "Pull the side brush off the motor hexagonal shaft.",
                    "Remove coiled hair wrapped underneath the brush collar and inspect the drive axle.",
                ],
                part_recommendations="Roborock Q7 Side Brush Assembly",
            )
        )
    if wheel_data.get("left_stalled") or wheel_data.get("right_stalled"):
        stalled_wheel = "Left" if wheel_data.get("left_stalled") else "Right"
        stalled_motors.append(f"{stalled_wheel} Drive Wheel")
        issues.append(
            DiagnosticIssue(
                issue_id="wheel_stalled",
                component="wheel",
                severity="critical",
                title=f"{stalled_wheel} Drive Wheel Jammed / Overloaded",
                description=f"Drive wheel motor stall detected on the {stalled_wheel.lower()} wheel module.",
                technical_detail="Tag 9 or 10 reported wheel_stalled=True.",
                repair_steps=[
                    f"Check the {stalled_wheel.lower()} wheel tread for rubber bands, carpet cords, or debris wedged in the wheel well.",
                    f"Press down on the {stalled_wheel.lower()} wheel suspension spring to verify smooth travel up and down.",
                    "Spin the wheel firmly by hand to feel if gearbox teeth are stripped or jammed.",
                ],
                part_recommendations="Roborock Q7 Left / Right Drive Wheel Module",
            )
        )
    if blower_data.get("is_stalled"):
        stalled_motors.append("Suction Fan Blower")
        issues.append(
            DiagnosticIssue(
                issue_id="blower_stalled",
                component="system",
                severity="critical",
                title="Suction Fan Blower Error / Overload (Error 18)",
                description="Suction turbine motor is jammed or blocked by large debris.",
                technical_detail="Tag 8 reported blower.is_stalled=True.",
                repair_steps=[
                    "Remove the dustbin and inspect the black rubber intake gasket inside the vacuum.",
                    "Use a flashlight to look down the suction intake duct for pens, socks, or clogs.",
                    "Ensure the rear exhaust vent fins are clean and unobstructed.",
                ],
            )
        )
    if wheel_data.get("left_wheel_up") or wheel_data.get("right_wheel_up"):
        issues.append(
            DiagnosticIssue(
                issue_id="wheel_suspended",
                component="wheel",
                severity="warning",
                title="Robot Suspended / Wheel Off Ground (Error 4)",
                description="Suspension drop microswitch indicates the vacuum is lifted off the floor or wheels are dangling.",
                technical_detail="Tag 9 or 10 reported wheel_up=True.",
                repair_steps=[
                    "Place the vacuum flat on level flooring so its weight compresses the suspension springs.",
                    "If already flat on the floor, press each drive wheel up and down firmly to ensure the internal microswitch clicks and is not stuck.",
                ],
            )
        )

    if stalled_motors:
        motor_score = 40
        motor_status = "error"

    subsystems["motors"] = SubsystemHealth(
        name="Motors & Drivetrain",
        score=motor_score,
        status=motor_status,
        summary="All motors clear" if not stalled_motors else f"Stalled: {', '.join(stalled_motors)}",
    )

    # 6. EVALUATE IMU & ORIENTATION (AHRS Tag 15)
    ahrs_data = latest_frame.get("ahrs", {})
    imu_score = 100
    imu_status = "nominal"
    has_ahrs = any(v is not None for v in (ahrs_data.get("accel_x"), ahrs_data.get("gyro_x"), ahrs_data.get("pitch")))

    if has_ahrs:
        az = ahrs_data.get("accel_z")
        # Accel Z typically measures ~1G (gravity vector) when level on floor
        summary_imu = f"Pitch: {ahrs_data.get('pitch', 0)}, Roll: {ahrs_data.get('roll', 0)}"
    else:
        summary_imu = "Passive tracking ready"

    subsystems["imu"] = SubsystemHealth(
        name="IMU & Navigation",
        score=imu_score,
        status=imu_status,
        summary=summary_imu,
    )

    # 7. EVALUATE FAULT CODE (Tag 14)
    fault_code = latest_frame.get("fault_code", 0)
    if fault_code and fault_code > 0:
        issues.append(
            DiagnosticIssue(
                issue_id=f"active_mcu_error_{fault_code}",
                component="system",
                severity="critical",
                title=f"Active Hardware Error Code {fault_code}",
                description=f"Robot MCU is actively reporting Error {fault_code}.",
                technical_detail=f"Protobuf Tag 14 returned fault_code={fault_code}.",
                repair_steps=[
                    f"Cross-reference Error {fault_code} with sensor diagnostic readings above.",
                    "Power down the vacuum completely for 15 seconds to clear volatile fault registers.",
                    "Inspect the identified subsystem component for mechanical obstruction or disconnected cabling.",
                ],
            )
        )

    # 8. EVALUATE FIRMWARE & SYSTEM INTEGRITY
    sys_score = 100
    sys_status = "nominal"
    cert_info = getters.get("certificate_metadata", {}).get("decoded", {})
    checksums = cert_info.get("checksums", {})

    mismatches = []
    for file_name, expected in EXPECTED_CHECKSUMS.items():
        actual = checksums.get(file_name)
        if actual and actual.lower() != expected.lower():
            mismatches.append(file_name)

    if mismatches:
        sys_score = 80
        sys_status = "warning"
        issues.append(
            DiagnosticIssue(
                issue_id="cert_checksum_mismatch",
                component="system",
                severity="info",
                title="Certificate Partition Customization Detected",
                description=f"Checksums modified on: {', '.join(mismatches)}.",
                technical_detail="Files differ from standard factory dump, expected if device was custom provisioned.",
                repair_steps=[
                    "No hardware repair needed. Ensure MQTT broker and local server configs match customized certs.",
                ],
            )
        )

    subsystems["system"] = SubsystemHealth(
        name="Firmware & Architecture",
        score=sys_score,
        status=sys_status,
        summary=f"{model} | FW {fw_ver}",
    )

    # Compute overall score
    overall_score = round(sum(s.score for s in subsystems.values()) / len(subsystems))
    if any(s.status == "error" for s in subsystems.values()):
        overall_status = "fault_detected"
    elif any(s.status == "warning" for s in subsystems.values()):
        overall_status = "attention_required"
    else:
        overall_status = "nominal"

    # SSH Information (Note: SSH service is not on the vacuum)
    root_guide = None
    if ssh_password:
        host = data.get("host", "192.168.5.1")
        root_guide = (
            f"Factory Service Password: `{ssh_password}`\n\n"
            f"> [!NOTE]\n"
            f"> SSH daemon is **not running / not installed** on this vacuum's firmware (TCP port 22 is refused).\n"
            f"> This password is stored in vendor partition slot 33 (read via 0xF7), but does not provide an active shell.\n"
        )

    return DiagnosticReport(
        overall_score=overall_score,
        overall_status=overall_status,
        model=model,
        sku=sku,
        serial=serial,
        firmware_version=fw_ver,
        mcu_version=mcu_ver,
        subsystems=subsystems,
        issues=issues,
        ssh_password=ssh_password,
        root_access_guide=root_guide,
    )
