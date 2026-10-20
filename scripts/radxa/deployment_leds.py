#!/usr/bin/env python3
"""Independent status animation service; ESP32 serial-owner API or separately wired SPI ring."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "software/radxa/console"))
from console.deployment_status import indicator, led_frame


class Esp32Backend:
    def __init__(self, base):
        self.base = base
        self.sent_at = 0
        self.available = False
        self.last_phase=None

    def show(self, state, frame, now):
        if now - self.sent_at < .4 or (state['phase_code']==0 and self.last_phase==0):
            return
        self.sent_at = now
        payload = json.dumps({"phase": state["phase_code"], "progress": state["progress"], "ttl_ms": 1500}).encode()
        request = urllib.request.Request(self.base + "/api/internal/deployment/led", data=payload,
                                         headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=.3) as response:
            self.available = bool(json.load(response).get("sent"))
            if self.available:self.last_phase=state['phase_code']


class SpiBackend:
    def __init__(self, bus, device):
        import spidev
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 2400000
        self.spi.mode = 0
        self.available = True

    @staticmethod
    def encode(frame):
        # At 2.4 MHz, 100 and 110 encode WS2812 0/1. GRB wire order.
        output = bytearray(24)
        for red, green, blue in frame:
            for value in (green, red, blue):
                word = 0
                for bit in range(7, -1, -1):
                    word = (word << 3) | (0b110 if value & (1 << bit) else 0b100)
                output.extend(word.to_bytes(3, "big"))
        output.extend(bytes(24))
        return list(output)

    def show(self, state, frame, now):
        self.spi.writebytes2(self.encode(frame))


def normal_frame(status,seconds,count=16,brightness=.35):
    """Return to measured robot personality when the Radxa owns the physical ring."""
    tel=status.get('telemetry',{});debug=status.get('debug',{});hub=tel.get('hub',{})
    if not tel.get('stale',True) and (hub.get('fault_l') or hub.get('fault_r')):
        return led_frame('failed',0,seconds,count,brightness)
    frame=[];brightness=max(0,min(.6,brightness))
    driving=abs(debug.get('fwd',0))>.03 or abs(debug.get('str',0))>.05
    center=math.atan2(debug.get('str',0),debug.get('fwd',0))*count/(2*math.pi)
    for i in range(count):
        if not tel.get('stale',True) and tel.get('balancing'):
            base=.08+.06*(1+math.sin(seconds*1.6))/2
            distance=min((i-center)%count,(center-i)%count)
            beam=.65*math.exp(-distance*distance/5) if driving else 0
            color=(base*.15+beam*.15,base*.55+beam*.75,base*.85+beam)
            if driving and debug.get('fwd',0)<-.03:color=(base+beam,base*.4+beam*.25,base*.02)
        else:
            distance=(i-seconds*.6)%count
            ember=.02+.12*math.exp(-distance)
            color=(ember,ember*.35,0)
        frame.append(tuple(round(255*brightness*max(0,min(1,v))) for v in color))
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("/etc/2pybot-deploy.json"))
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    led = config.get("led", {})
    backend = SpiBackend(led.get("bus", 0), led.get("device", 0)) if led.get("backend") == "spi" else Esp32Backend(config.get("api", "http://127.0.0.1:8080"))
    last_error = ""; previous = None;telemetry={};last_poll=0;was_balancing=False;fall_until=0
    while True:
        now = time.time()
        state = indicator(Path(config.get("state_dir", "/var/lib/2pybot-deploy")),
                          Path(config.get("run_dir", "/run/2pybot-deploy")), "/var/lib/2pybot/firmware", now)
        try:
            if isinstance(backend,SpiBackend) and now-last_poll>=.1:
                try:
                    with urllib.request.urlopen(config.get('api','http://127.0.0.1:8080')+'/api/status',timeout=.25) as response:telemetry=json.load(response)
                except Exception:telemetry={}
                last_poll=now
                tel=telemetry.get('telemetry',{})
                if not tel.get('stale',True) and was_balancing and not tel.get('balancing') and abs(telemetry.get('debug',{}).get('pF',0))>45:
                    fall_until=now+1.4
                was_balancing=bool(tel.get('balancing'))
            if isinstance(backend,SpiBackend) and state['phase']=='idle':
                frame=normal_frame(telemetry,now,led.get('count',16),led.get('brightness',.35))
            else:frame=led_frame(state["phase"], state["progress"], now, led.get("count", 16), led.get("brightness", .35))
            hub=telemetry.get('telemetry',{}).get('hub',{})
            if isinstance(backend,SpiBackend) and not telemetry.get('telemetry',{}).get('stale',True) and (hub.get('fault_l') or hub.get('fault_r')):
                frame=led_frame('failed',0,now,led.get('count',16),led.get('brightness',.35))
            if isinstance(backend,SpiBackend) and now<fall_until:frame=led_frame('failed',0,now,led.get('count',16),led.get('brightness',.35))
            backend.show(state,frame,now)
            last_error = ""
        except Exception as error:
            backend.available = False
            if str(error) != last_error:
                print("LED output unavailable: " + str(error), flush=True)
            last_error = str(error)
        visible = (state["phase"], backend.available)
        if visible != previous:
            print(json.dumps({"phase": state["phase"], "backend": led.get("backend", "esp32"), "available": backend.available}), flush=True)
            previous = visible
        time.sleep(1 / 30)


if __name__ == "__main__":
    main()
