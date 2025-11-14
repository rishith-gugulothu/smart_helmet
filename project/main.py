"""
smart_helmet.py

Smart Helmet - Python implementation (Raspberry Pi oriented)

Features:
 - Fall/crash detection using MPU6050 accelerometer/gyro (I2C)
 - GPS location reading from a serial GPS module (e.g., Neo-6M)
 - SMS alert via GSM modem (e.g., SIM800) or HTTP webhook if GSM not available
 - LED + buzzer alerts
 - Simulation mode for running on a non-RPi environment (for demo)

Usage:
    python smart_helmet.py            # default: auto-detect Raspberry Pi, hardware mode if available
    python smart_helmet.py --sim      # run simulation mode (no hardware required)

Dependencies (hardware mode):
 - Raspbian / Raspberry Pi
 - python3
 - pip install pyserial smbus2 requests
 - RPi.GPIO (preinstalled on RPi OS)
 - MPU6050 hardware connected on I2C bus (address 0x68)
 - GPS module on serial (e.g., /dev/ttyS0 or /dev/ttyAMA0)
 - GSM modem on serial (e.g., /dev/ttyUSB0)

This file is intentionally single-file for submission/demo purposes.
"""

import time
import threading
import argparse
import math
import sys

# Optional imports that may not exist in simulation mode
try:
    import serial
except Exception:
    serial = None
try:
    import requests
except Exception:
    requests = None
try:
    from smbus2 import SMBus
except Exception:
    SMBus = None
try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None

# ----------------------
# Configuration section
# ----------------------
CONFIG = {
    "SIMULATION": False,
    "MPU6050_I2C_ADDR": 0x68,
    "I2C_BUS": 1,
    "FALL_ACCEL_THRESHOLD_G": 3.0,  # Gs (change per your sensitivity)
    "FALL_DETECTION_WINDOW": 0.6,   # seconds to consider crash event
    "GPS_SERIAL_PORT": "/dev/ttyS0", # change as per connection
    "GPS_BAUD": 9600,
    "GSM_SERIAL_PORT": "/dev/ttyUSB0", # or /dev/ttyAMA0; used only if GSM attached
    "GSM_BAUD": 115200,
    "ALERT_PHONE_NUMBER": "+911234567890", # example; change for real SMS
    "HTTP_ALERT_WEBHOOK": None, # e.g., "https://maker.ifttt.com/trigger/helmet_alert/with/key/XXXX" (optional fallback)
    "LED_PIN": 17,    # BCM pin for LED
    "BUZZER_PIN": 27, # BCM pin for buzzer
    "ALERT_DURATION": 10, # seconds to sound buzzer/LED on alert
}

# ----------------------
# Utility functions
# ----------------------
def log(msg):
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp}] {msg}")

# ----------------------
# Hardware Abstractions
# ----------------------
class LEDBuzzer:
    """Simple LED + Buzzer control (or simulation)"""
    def __init__(self, config):
        self.led_pin = config["LED_PIN"]
        self.buzzer_pin = config["BUZZER_PIN"]
        self.sim = config["SIMULATION"]
        if not self.sim:
            if GPIO is None:
                raise RuntimeError("RPi.GPIO library not found but simulation disabled.")
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.led_pin, GPIO.OUT)
            GPIO.setup(self.buzzer_pin, GPIO.OUT)

    def alert_on(self):
        if self.sim:
            log("SIM: LED ON, BUZZER ON")
        else:
            GPIO.output(self.led_pin, GPIO.HIGH)
            GPIO.output(self.buzzer_pin, GPIO.HIGH)

    def alert_off(self):
        if self.sim:
            log("SIM: LED OFF, BUZZER OFF")
        else:
            GPIO.output(self.led_pin, GPIO.LOW)
            GPIO.output(self.buzzer_pin, GPIO.LOW)

    def cleanup(self):
        if not self.sim:
            GPIO.cleanup()

# ----------------------
# MPU6050 interface
# ----------------------
class MPU6050:
    """
    Basic MPU6050 accelerometer reader
    Uses smbus2.SMBus in hardware mode; otherwise simulation
    """
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    ACCEL_SCALE_MODIFIER = 16384.0  # for +/-2g

    def __init__(self, config):
        self.sim = config["SIMULATION"]
        self.addr = config["MPU6050_I2C_ADDR"]
        self.i2c_bus_num = config["I2C_BUS"]
        if not self.sim:
            if SMBus is None:
                raise RuntimeError("smbus2 not available; cannot use MPU6050 in hardware mode.")
            self.bus = SMBus(self.i2c_bus_num)
            # wake up sensor
            self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0)
        else:
            log("SIM: MPU6050 in simulation mode")

    def _read_i2c_word(self, reg):
        # read two bytes and combine
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def read_accel(self):
        """Return (ax_g, ay_g, az_g) in G units"""
        if self.sim:
            # return fake data (small noise). For demo, produce normal standing and occasionally large spike
            t = time.time()
            # simulate a sudden spike every 20s for demo
            if int(t) % 20 == 0:
                return (0.1, 0.1, 4.5)  # big spike on z -> crash
            return (0.02, 0.01, 0.98)  # near 1g normal
        else:
            ax = self._read_i2c_word(self.ACCEL_XOUT_H) / self.ACCEL_SCALE_MODIFIER
            ay = self._read_i2c_word(self.ACCEL_XOUT_H + 2) / self.ACCEL_SCALE_MODIFIER
            az = self._read_i2c_word(self.ACCEL_XOUT_H + 4) / self.ACCEL_SCALE_MODIFIER
            return (ax, ay, az)

# ----------------------
# GPS Reader
# ----------------------
class GPSReader:
    """Reads NMEA from GPS module, returns (lat, lon) as decimal degrees or None"""
    def __init__(self, config):
        self.sim = config["SIMULATION"]
        self.port = config["GPS_SERIAL_PORT"]
        self.baud = config["GPS_BAUD"]
        self.ser = None
        if not self.sim:
            if serial is None:
                raise RuntimeError("pyserial not installed; cannot use GPS in hardware mode.")
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                log(f"GPS serial opened at {self.port}@{self.baud}")
            except Exception as e:
                log(f"Failed to open GPS serial: {e}")
                self.ser = None
        else:
            log("SIM: GPS in simulation mode")

    @staticmethod
    def _nmea_to_decimal(coord_str, hemi):
        # coord_str like "3723.2475" -> degrees + minutes. Format: ddmm.mmmm (lat), dddmm.mmmm (lon)
        try:
            if not coord_str or coord_str == "":
                return None
            dot = coord_str.find(".")
            if dot == -1:
                return None
            degrees_len = 2 if len(coord_str) <= 9 else 3  # rough heuristic
            degrees = float(coord_str[:degrees_len])
            minutes = float(coord_str[degrees_len:])
            dec = degrees + minutes / 60.0
            if hemi in ['S', 'W']:
                dec = -dec
            return dec
        except Exception:
            return None

    def get_location(self, timeout=3.0):
        if self.sim:
            # sample simulated location
            return (17.445, 78.349)  # example: Hyderabad
        if self.ser is None:
            return None
        end = time.time() + timeout
        while time.time() < end:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            if line.startswith("$GPRMC") or line.startswith("$GPGGA"):
                parts = line.split(",")
                # For GPRMC: parts[3] lat, parts[4] N/S, parts[5] lon, parts[6] E/W
                try:
                    if line.startswith("$GPRMC") and len(parts) > 6 and parts[3] and parts[5]:
                        lat = self._nmea_to_decimal(parts[3], parts[4])
                        lon = self._nmea_to_decimal(parts[5], parts[6])
                        if lat is not None and lon is not None:
                            return (lat, lon)
                    if line.startswith("$GPGGA") and len(parts) > 5 and parts[2] and parts[4]:
                        lat = self._nmea_to_decimal(parts[2], parts[3])
                        lon = self._nmea_to_decimal(parts[4], parts[5])
                        if lat is not None and lon is not None:
                            return (lat, lon)
                except Exception:
                    continue
        return None

# ----------------------
# GSM / Alert sender
# ----------------------
class AlertSender:
    """
    Send SMS via GSM module (AT commands) OR fallback to HTTP webhook (IFTTT or custom)
    """
    def __init__(self, config):
        self.sim = config["SIMULATION"]
        self.port = config["GSM_SERIAL_PORT"]
        self.baud = config["GSM_BAUD"]
        self.phone = config["ALERT_PHONE_NUMBER"]
        self.webhook = config["HTTP_ALERT_WEBHOOK"]
        self.ser = None
        if not self.sim:
            if serial is None:
                log("pyserial not available, GSM disabled.")
            else:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=1)
                    log(f"GSM serial opened at {self.port}@{self.baud}")
                    time.sleep(1)
                    # basic AT test
                    self._send_at("AT")
                except Exception as e:
                    log(f"Failed to open GSM serial: {e}")
                    self.ser = None
        else:
            log("SIM: AlertSender in simulation mode")

    def _send_at(self, cmd, wait=0.2):
        if self.ser:
            self.ser.write((cmd + "\r\n").encode())
            time.sleep(wait)
            resp = self.ser.read(self.ser.in_waiting or 1).decode(errors='ignore')
            return resp
        return None

    def send_sms(self, message):
        if self.sim:
            log(f"SIM: Would send SMS to {self.phone}: {message}")
            if self.webhook and requests:
                try:
                    requests.post(self.webhook, json={"value1": message}, timeout=3)
                    log("SIM: Also sent webhook alert")
                except Exception as e:
                    log(f"SIM: webhook failed: {e}")
            return True
        # try GSM AT commands
        if self.ser:
            try:
                self._send_at("AT+CMGF=1", wait=0.5)  # text mode
                self._send_at(f'AT+CMGS="{self.phone}"', wait=0.5)
                self.ser.write(message.encode() + b"\x1A")  # ctrl+z to send
                time.sleep(2)
                log("SMS sent (attempted via GSM)")
                return True
            except Exception as e:
                log(f"GSM SMS send error: {e}")
        # fallback to webhook if configured
        if self.webhook and requests:
            try:
                requests.post(self.webhook, json={"value1": message}, timeout=3)
                log("Webhook alert sent")
                return True
            except Exception as e:
                log(f"Webhook failed: {e}")
        log("Failed to send alert (no GSM/webhook available)")
        return False

# ----------------------
# Main smart helmet logic
# ----------------------
class SmartHelmet:
    def __init__(self, config):
        self.config = config
        self.led_buzzer = LEDBuzzer(config)
        self.mpu = MPU6050(config)
        self.gps = GPSReader(config)
        self.alert_sender = AlertSender(config)
        self._accel_history = []  # list of (timestamp, magnitude_g)
        self._running = False

    @staticmethod
    def magnitude_g(ax, ay, az):
        return math.sqrt(ax*ax + ay*ay + az*az)

    def _record_accel(self, mag):
        now = time.time()
        self._accel_history.append((now, mag))
        # keep only last few seconds
        cutoff = now - 2.0
        self._accel_history = [(t, m) for (t, m) in self._accel_history if t >= cutoff]

    def check_for_fall(self):
        ax, ay, az = self.mpu.read_accel()
        mag = self.magnitude_g(ax, ay, az)
        self._record_accel(mag)
        # Condition: magnitude exceeds threshold
        if mag >= self.config["FALL_ACCEL_THRESHOLD_G"]:
            log(f"High accel detected: {mag:.2f} g")
            # confirm sustained or sharp change within window
            now = time.time()
            recent = [m for (t, m) in self._accel_history if t >= now - self.config["FALL_DETECTION_WINDOW"]]
            if recent and max(recent) >= self.config["FALL_ACCEL_THRESHOLD_G"]:
                return True
        return False

    def do_alert(self, location=None):
        # turn on visible/audible alert
        log("ALERT: Triggering LED/Buzzer and sending notification")
        self.led_buzzer.alert_on()
        # prepare message
        if location:
            lat, lon = location
            msg = f"SmartHelmet ALERT: possible crash detected at https://maps.google.com/?q={lat},{lon}"
        else:
            msg = "SmartHelmet ALERT: possible crash detected. Location unavailable."
        # send alert in background
        t = threading.Thread(target=self.alert_sender.send_sms, args=(msg,))
        t.daemon = True
        t.start()
        # keep alarm for configured duration
        time.sleep(self.config["ALERT_DURATION"])
        self.led_buzzer.alert_off()

    def run(self):
        log("SmartHelmet started")
        self._running = True
        try:
            while self._running:
                try:
                    # sample accel
                    if self.check_for_fall():
                        log("Fall/Crash detected!")
                        # read GPS for location (may take a few seconds)
                        loc = self.gps.get_location(timeout=4.0)
                        if loc:
                            log(f"Location obtained: {loc[0]:.6f}, {loc[1]:.6f}")
                        else:
                            log("Location not available")
                        self.do_alert(location=loc)
                        # after alert, we sleep a bit to avoid repeated triggers
                        time.sleep(5)
                    else:
                        # not detected: small sleep and continue
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    log("User requested interrupt, stopping")
                    break
                except Exception as e:
                    log(f"Runtime error in main loop: {e}")
                    # avoid busy-looping on errors
                    time.sleep(0.5)
        finally:
            self.shutdown()

    def shutdown(self):
        log("Shutting down SmartHelmet")
        self.led_buzzer.cleanup()
        self._running = False

# ----------------------
# CLI / Demo wrapper
# ----------------------
def main():
    parser = argparse.ArgumentParser(description="Smart Helmet Python demo")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode (no hardware)")
    args = parser.parse_args()

    config = CONFIG.copy()
    if args.sim:
        config["SIMULATION"] = True

    # If running on non-RPi environment and RPi.GPIO not present, default to simulation
    if GPIO is None and not config["SIMULATION"]:
        log("RPi.GPIO not found; forcing simulation mode.")
        config["SIMULATION"] = True

    try:
        helmet = SmartHelmet(config)
        helmet.run()
    except Exception as e:
        log(f"Fatal error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
