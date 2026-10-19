"""Check production ordering and the host handshake before telemetry starts."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_all_sensor_dependent_setup_occurs_after_host_wait():
    source = (ROOT / "firmware/BaseLink/BaseLink.ino").read_text()
    setup = source[source.index("void setup() {"):source.index("void loop() {")]
    waiting = setup.index("waitForHostStartup();")
    for call in ("leds_begin();", "imu.begin()", "steppers.begin()", "delay(STARTUP_SETTLE_MS)",
                 "imu.calibrateGyro()", "btgamepad_begin();", "payload_begin();"):
        assert setup.index(call) > waiting, call
    assert setup.index("params_dump();") < waiting
    assert setup.index("pollSerial();") < setup.index("startupGate.complete();")


def test_wait_loop_services_ack_and_discovery_without_auto_enabling():
    source = (ROOT / "firmware/BaseLink/BaseLink.ino").read_text()
    waiting = source[source.index("void waitForHostStartup()"):source.index("void setup()")]
    assert "pollSerial();" in waiting
    assert "mayInitialize" in waiting
    assert "requestEnable" not in waiting and "imu." not in waiting
    parser = source[source.index("void handleLine"):source.index("void pollSerial")]
    assert parser.index('"BR,1"') < parser.index("if (at_handleLine(line))")
    assert parser.index("if (!startupGate.initialized)") < parser.index("if (at_handleLine(line))")
