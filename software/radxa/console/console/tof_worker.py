"""VL53L5CX ULD in an isolated process; JSON lines, no serial/motor access."""
import ctypes as C
import fcntl
import json
import os
import signal
import sys
import time


def emit(**frame):
    print(json.dumps(frame), flush=True)


def main():
    vendor = os.environ.get("BOT_TOF_VENDOR")
    if vendor:
        sys.path.insert(0, vendor)
    import vl53l5cx_ctypes as sdk
    from smbus2 import SMBus, i2c_msg

    bus_number = int(os.environ.get("BOT_TOF_BUS", "7"))
    address = int(os.environ.get("BOT_TOF_ADDRESS", "0x29"), 0)
    hz = int(os.environ.get("BOT_TOF_HZ", "10"))
    integration_ms = int(os.environ.get("BOT_TOF_INTEGRATION_MS", "20"))
    if not 1 <= hz <= 15:
        raise ValueError("8x8 ranging requires 1..15 Hz")
    if not 2 <= integration_ms <= 1000 or 4 * integration_ms >= 1000 / hz:
        raise ValueError("8x8 integration budget must fit four subframes in the ranging period")
    os.nice(10)
    # Do not allow multiple console workers to upload firmware concurrently.
    lock = os.open(f"/tmp/2pybot-tof-{bus_number}-{address}.lock", os.O_CREAT | os.O_RDWR, 0o600)
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    bus = SMBus(bus_number)
    lib = sdk._VL53
    fault = []

    @sdk._I2C_RD_FUNC
    def read(addr, reg, output, length):
        try:
            if fault:
                return 255
            for offset in range(0, length, 256):
                count = min(256, length - offset)
                w = i2c_msg.write(addr, [(reg + offset) >> 8, (reg + offset) & 255])
                r = i2c_msg.read(addr, count)
                bus.i2c_rdwr(w, r)
                for index, value in enumerate(r):
                    output[offset + index] = value
            return 0
        except Exception as exc:  # noqa: BLE001 -- exceptions must not escape a C callback
            fault.append(f"read addr={addr:#x} reg={reg:#x} len={length}: {exc}")
            return 255

    @sdk._I2C_WR_FUNC
    def write(addr, reg, data, length):
        try:
            if fault:
                return 255
            for offset in range(0, length, 256):
                payload = [(reg + offset) >> 8, (reg + offset) & 255]
                payload += [data[i] for i in range(offset, min(offset + 256, length))]
                bus.i2c_rdwr(i2c_msg.write(addr, payload))
            return 0
        except Exception as exc:  # noqa: BLE001 -- exceptions must not escape a C callback
            fault.append(f"write addr={addr:#x} reg={reg:#x} len={length}: {exc}")
            return 255

    @sdk._SLEEP_FUNC
    def sleep(ms):
        time.sleep(ms / 1000)
        return 0

    # ctypes defaults truncate pointers on aarch64; all signatures are explicit.
    lib.get_configuration.argtypes = [C.c_uint8, sdk._I2C_RD_FUNC, sdk._I2C_WR_FUNC, sdk._SLEEP_FUNC]
    lib.get_configuration.restype = C.c_void_p
    lib.cleanup_configuration.argtypes = [C.c_void_p]
    lib.cleanup_configuration.restype = None
    signatures = {
        "is_alive": [C.POINTER(C.c_uint8)], "init": [],
        "set_resolution": [C.c_uint8], "set_ranging_frequency_hz": [C.c_uint8],
        "set_target_order": [C.c_uint8], "start_ranging": [], "stop_ranging": [],
        "set_ranging_mode": [C.c_uint8], "set_integration_time_ms": [C.c_uint32],
        "check_data_ready": [C.POINTER(C.c_uint8)],
        "get_ranging_data": [C.POINTER(sdk.VL53L5CX_ResultsData)],
    }
    for name, args in signatures.items():
        fn = getattr(lib, "vl53l5cx_" + name)
        fn.argtypes = [C.c_void_p] + args
        fn.restype = C.c_uint8
    cfg = lib.get_configuration(address << 1, read, write, sleep)
    if not cfg:
        raise RuntimeError("ULD allocation failed")

    def call(name, *args):
        code = getattr(lib, "vl53l5cx_" + name)(cfg, *args)
        if fault or code:
            raise RuntimeError(f"{name}: ULD={code}; I2C={fault[-1] if fault else 'OK'}")

    stopping = [False]

    def terminate(signum, frame):
        # Exceptions raised inside ctypes callbacks are swallowed by ctypes.
        stopping[0] = True

    signal.signal(signal.SIGTERM, terminate)
    signal.alarm(45)
    started = False
    try:
        alive = C.c_uint8()
        call("is_alive", C.byref(alive))
        if not alive.value:
            raise RuntimeError(f"No VL53L5CX at /dev/i2c-{bus_number} {address:#x}")
        emit(state="initializing", message="ID verified; uploading sensor firmware")
        call("init")
        call("set_resolution", 64)
        call("set_ranging_frequency_hz", hz)
        call("set_ranging_mode", 3)  # autonomous mode uses the explicit integration budget
        call("set_integration_time_ms", integration_ms)
        call("set_target_order", 1)
        call("start_ranging")
        started = True
        last = time.monotonic()
        sequence = 0
        while not stopping[0]:
            signal.alarm(6)
            ready = C.c_uint8()
            call("check_data_ready", C.byref(ready))
            if ready.value:
                result = sdk.VL53L5CX_ResultsData()
                read_start = time.monotonic_ns()
                call("get_ranging_data", C.byref(result))
                read_end = time.monotonic_ns()
                sequence += 1
                last = time.monotonic()
                emit(state="ranging", distance_mm=list(result.distance_mm[0]),
                     status=list(result.target_status[0]),
                     sigma_mm=list(result.range_sigma_mm[0]),
                     targets=list(result.nb_target_detected), sequence=sequence,
                     capture_monotonic_ns=(read_start + read_end) // 2,
                     read_duration_ms=round((read_end-read_start)/1e6, 3),
                     timestamp_source="host_read_midpoint",
                     timestamp_uncertainty_ms=round(500/hz+(read_end-read_start)/2e6, 2),
                     ambient_per_spad=list(result.ambient_per_spad),
                     signal_per_spad=list(result.signal_per_spad[0]),
                     reflectance=list(result.reflectance[0]))
            elif time.monotonic() - last > 3:
                raise RuntimeError("No fresh depth frame for 3 seconds")
            time.sleep(0.01)
    finally:
        signal.alarm(2)
        if started and not fault:
            lib.vl53l5cx_stop_ranging(cfg)
        lib.cleanup_configuration(cfg)
        bus.close()
        os.close(lock)
        signal.alarm(0)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001 -- report isolated worker failure to the supervisor
        emit(state="error", message=str(exc))
        sys.exit(1)
