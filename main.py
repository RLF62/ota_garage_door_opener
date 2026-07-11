from machine import Pin, UART, Timer, I2C, ADC
import machine
import time
import utime
import ujson
import os
import ubinascii
import BME280
import adafruit_simplemath

# ----------------------------
# Firmware version / UART updater
# ----------------------------
FW_VERSION = "1.0.8-lidar-filter-vent-status"
UPDATE_MODE = False
_update_expected_size = 0
_update_expected_checksum = ""
_update_received_size = 0
_update_checksum_sum = 0
_update_seq_expected = 0
UPDATE_NEW_FILE = "main.py.new"
UPDATE_BAK_FILE = "main.py.bak"
UPDATE_TARGET_FILE = "main.py"


def update_checksum_bytes(data):
    """Small checksum that works on MicroPython without extra libraries."""
    total = 0
    try:
        for b in data:
            total = (total + int(b)) & 0xFFFFFFFF
    except Exception:
        pass
    return total


def send_update_status(status, **extra):
    try:
        payload = {"update": status}
        for k, v in extra.items():
            payload[k] = v
        uart.write(ujson.dumps(payload) + "\n")
    except Exception:
        pass


def send_fw_version():
    try:
        uart.write(ujson.dumps({"fw_version": FW_VERSION}) + "\n")
    except Exception:
        pass


def update_start(msg):
    global UPDATE_MODE, _update_expected_size, _update_expected_checksum
    global _update_received_size, _update_checksum_sum, _update_seq_expected
    global abort_motion, pending_command, stop_command

    try:
        size = int(msg.get("size", 0))
        checksum = str(msg.get("checksum", "")).strip().lower()
        filename = str(msg.get("filename", UPDATE_TARGET_FILE)).strip()

        if filename != UPDATE_TARGET_FILE or size <= 0 or not checksum:
            send_update_status("failed", reason="bad_start")
            return

        # Put the controller in a safe state before receiving code.
        UPDATE_MODE = True
        abort_motion = True
        pending_command = None
        stop_command = False
        MOTOR_MOVE.value(0)
        LIGHT_ON_OFF.value(0)

        try:
            if UPDATE_NEW_FILE in os.listdir():
                os.remove(UPDATE_NEW_FILE)
        except Exception:
            pass

        with open(UPDATE_NEW_FILE, "wb") as f:
            pass

        _update_expected_size = size
        _update_expected_checksum = checksum
        _update_received_size = 0
        _update_checksum_sum = 0
        _update_seq_expected = 0
        send_update_status("ready", size=size, version=FW_VERSION)
    except Exception as e:
        UPDATE_MODE = False
        send_update_status("failed", reason="start_exception", detail=str(e))


def update_chunk(msg):
    global _update_received_size, _update_checksum_sum, _update_seq_expected

    if not UPDATE_MODE:
        send_update_status("failed", reason="not_in_update_mode")
        return

    try:
        seq = int(msg.get("seq", -1))
        if seq != _update_seq_expected:
            send_update_status("failed", reason="bad_seq", expected=_update_seq_expected, got=seq)
            return

        data_b64 = msg.get("data", "")
        chunk = ubinascii.a2b_base64(data_b64)
        if not chunk:
            send_update_status("failed", reason="empty_chunk", seq=seq)
            return

        with open(UPDATE_NEW_FILE, "ab") as f:
            f.write(chunk)

        _update_received_size += len(chunk)
        _update_checksum_sum = (_update_checksum_sum + update_checksum_bytes(chunk)) & 0xFFFFFFFF
        _update_seq_expected += 1
        send_update_status("chunk_ok", seq=seq, received=_update_received_size)
    except Exception as e:
        send_update_status("failed", reason="chunk_exception", detail=str(e))


def update_end(msg=None):
    global UPDATE_MODE

    if not UPDATE_MODE:
        send_update_status("failed", reason="not_in_update_mode")
        return

    try:
        actual_checksum = "%08x" % (_update_checksum_sum & 0xFFFFFFFF)

        if _update_received_size != _update_expected_size:
            UPDATE_MODE = False
            send_update_status("failed", reason="size_mismatch", expected=_update_expected_size, got=_update_received_size)
            return

        if actual_checksum.lower() != _update_expected_checksum.lower():
            UPDATE_MODE = False
            send_update_status("failed", reason="checksum_mismatch", expected=_update_expected_checksum, got=actual_checksum)
            return

        try:
            if UPDATE_BAK_FILE in os.listdir():
                os.remove(UPDATE_BAK_FILE)
        except Exception:
            pass

        try:
            if UPDATE_TARGET_FILE in os.listdir():
                os.rename(UPDATE_TARGET_FILE, UPDATE_BAK_FILE)
        except Exception:
            pass

        os.rename(UPDATE_NEW_FILE, UPDATE_TARGET_FILE)
        send_update_status("success", size=_update_received_size, checksum=actual_checksum)
        time.sleep_ms(500)
        machine.soft_reset()
    except Exception as e:
        UPDATE_MODE = False
        send_update_status("failed", reason="end_exception", detail=str(e))


def update_cancel(reason="cancelled"):
    global UPDATE_MODE
    UPDATE_MODE = False
    try:
        if UPDATE_NEW_FILE in os.listdir():
            os.remove(UPDATE_NEW_FILE)
    except Exception:
        pass
    send_update_status("cancelled", reason=reason)

# ----------------------------
# Garmin LIDAR-Lite v4 driver
# ----------------------------
# main_nonblocking_motor.py
# ----------------------------
class LidarLiteV4:
    """
    Garmin LIDAR-Lite v4 I2C driver for MicroPython.
    Address: 0x62

    Sequence:
      1) Write 0x04 to reg 0x00 (acquire)
      2) Poll reg 0x01 bit0 until 0 (not busy)
      3) Read 2 bytes at 0x10
    """
    def __init__(self, i2c, addr=0x62):
        self.i2c = i2c
        self.addr = addr
        self.i2c_error_count = 0
        self._configured = False

    def _write_reg(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read_u8(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _read_bytes(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def configure_long_range(self):
        """
        Boost range by increasing acquisition effort/sensitivity.
        Tune 0x04 and 0x1C if needed.
        """
        try:
            self._write_reg(0x02, 0x80)  # baseline
            self._write_reg(0x04, 0x08)  # acquisition count
            self._write_reg(0x1C, 0x00)  # sensitivity
            self._configured = True
        except Exception:
            self.i2c_error_count += 1
            self._configured = False

    def read_cm(self, retries=5, settle_ms=8, busy_timeout_ms=200, debug=False):
        """
        Returns distance in centimeters, or None on failure.
        More forgiving timing helps prevent every-other-read failures.
        """
        if not self._configured:
            self.configure_long_range()
            time.sleep_ms(50)

        for _ in range(retries):
            try:
                # Trigger measurement
                self._write_reg(0x00, 0x04)

                # Wait until not busy (status reg 0x01 bit0 clears)
                t0 = time.ticks_ms()
                while True:
                    status = self._read_u8(0x01)
                    if (status & 0x01) == 0:
                        break
                    if time.ticks_diff(time.ticks_ms(), t0) > busy_timeout_ms:
                        raise OSError("LIDAR busy timeout")
                    time.sleep_ms(settle_ms)

                # Small settle after busy clears
                if settle_ms > 0:
                    time.sleep_ms(settle_ms)

                # Read two bytes at 0x10
                b = self._read_bytes(0x10, 2)
                b0, b1 = b[0], b[1]

                # Try both byte orders
                cm_a = (b0 << 8) | b1
                cm_b = (b1 << 8) | b0

                if debug:
                    print("status:", status, "raw:", b0, b1, "cm_a:", cm_a, "cm_b:", cm_b)

                if 5 <= cm_a <= 1000:
                    return cm_a
                if 5 <= cm_b <= 1000:
                    return cm_b

            except Exception as e:
                self.i2c_error_count += 1
                if debug:
                    print("LIDAR err:", e)
                time.sleep_ms(10)

        return None


# ----------------------------
# Pins
# ----------------------------
LIGHT_PIN = 10
OPEN_PIN = 11
CLOSE_PIN = 12
VENT_PIN = 13
STOP_PIN = 14

LIGHT_ON_OFF = Pin(22, Pin.OUT)
LIGHT_ON_OFF.value(0)

MOTOR_MOVE = Pin(18, Pin.OUT)
MOTOR_MOVE.value(0)

# Relay pulse timing.
# This is now non-blocking, so the Pico can keep reading UART/LIDAR while the button is held.
button_hold_time = 1.0
button_hold_ms = int(button_hold_time * 1000)

DEBOUNCE_MS = 200

_motor_pulse_active = False
_motor_pulse_until_ms = 0

_light_pulse_active = False
_light_pulse_until_ms = 0


# ----------------------------
# Watchdog tuning (single source of truth)
# ----------------------------
# Heartbeat-only reset: Pi must miss HB long enough N times in a row.
HB_TIMEOUT_MS = 15000           # count a "miss" if hb age > 15s
HB_MISSES_TO_RESET = 8          # ~2 minutes of continuous misses triggers reset

# Net is status-only (DO NOT reset from net)
NET_TIMEOUT_MS = 180000         # used only for diagnostics/logging

hb_miss_count = 0


# ----------------------------
# PI POWER CONTROL (AO3407A + 2N2222)
# ----------------------------
PI_PWR_PIN = 2
pi_pwr = Pin(PI_PWR_PIN, Pin.OUT)
pi_pwr.value(1)  # Pi ON by default

PI_BOOT_GRACE_MS = 180000       # ignore watchdog checks for 3 minutes after power on
PI_RESET_COOLDOWN_MS = 300000   # don't reset again within 5 minutes
PI_POWER_OFF_MS = 2500          # cut power for 2.5s

_last_hb_ms = utime.ticks_ms()
_last_net_ms = utime.ticks_ms()
_last_pi_power_on_ms = utime.ticks_ms()
_last_pi_reset_ms = 0


# ----------------------------
# Control flags
# ----------------------------
debounce_flags = {}
debounce_timers = {}
for name in ['open', 'close', 'vent', 'light', 'stop']:
    debounce_flags[name] = False
    debounce_timers[name] = Timer()

stop_command = False
abort_motion = False
pending_command = None
vent_status = 0


# ----------------------------
# UART and I2C (shared bus)
# ----------------------------
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), rxbuf=8192)

# Buffered UART receive prevents partial JSON lines during firmware updates.
_uart_rx_buffer = b""
MAX_UART_BUFFER = 16384

# I2C pins (RP2040)
I2C_ID = 0
SCL_PIN_NUM = 9
SDA_PIN_NUM = 8
I2C_FREQ = 100000

i2c = I2C(I2C_ID, scl=Pin(SCL_PIN_NUM), sda=Pin(SDA_PIN_NUM), freq=I2C_FREQ)


# ----------------------------
# Debug helper (USB console + UART)
# ----------------------------
def dbg(msg):
    try:
        print("[DBG]", msg)
    except:
        pass
    try:
        uart.write(ujson.dumps({"dbg": msg}) + "\n")
    except:
        pass


def send_event(event):
    """Send event messages to the Pi Zero for logging in serial_reader.py."""
    try:
        uart.write(ujson.dumps({
            "event": event,
            "ms": utime.ticks_ms()
        }) + "\n")
    except:
        pass


# Let the Pi side serial reader start, then announce Pico boot.
time.sleep_ms(500)
send_event("pico_boot")


# ----------------------------
# Ignore boot glitches
# ----------------------------
BOOT_IGNORE_MS = 4000
_boot_ms = utime.ticks_ms()


def stable_low(pin, ms=40):
    t0 = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), t0) < ms:
        if pin.value() != 0:
            return False
        time.sleep_ms(2)
    return True


# ----------------------------
# LIDAR Health / Recovery Settings
# ----------------------------
LIDAR_STALE_MS = 1500
LIDAR_RECOVER_COOLDOWN_MS = 800
LIDAR_MAX_RECOVERS = 4

_last_lidar_good_ms = utime.ticks_ms()
_last_lidar_value_in = None
_lidar_recover_attempts = 0
_last_recover_ms = 0


# ----------------------------
# I2C bus clear helper (for stuck SDA/SCL)
# ----------------------------
def i2c_bus_clear(scl_pin_num=SCL_PIN_NUM, sda_pin_num=SDA_PIN_NUM, pulses=9):
    try:
        scl = Pin(scl_pin_num, Pin.OUT)
        sda = Pin(sda_pin_num, Pin.IN, Pin.PULL_UP)

        scl.value(1)
        time.sleep_us(5)

        for _ in range(pulses):
            scl.value(0)
            time.sleep_us(5)
            scl.value(1)
            time.sleep_us(5)

        # STOP: SDA low then high while SCL high
        sda = Pin(sda_pin_num, Pin.OUT)
        sda.value(0)
        time.sleep_us(5)
        scl.value(1)
        time.sleep_us(5)
        sda = Pin(sda_pin_num, Pin.IN, Pin.PULL_UP)
        time.sleep_us(5)

        return True
    except Exception as e:
        dbg("i2c_bus_clear err: " + str(e))
        return False


def rebuild_i2c_and_lidar():
    global i2c, lidar
    try:
        try:
            i2c.deinit()
        except:
            pass

        time.sleep_ms(50)
        i2c_bus_clear()

        time.sleep_ms(50)
        i2c = I2C(I2C_ID, scl=Pin(SCL_PIN_NUM), sda=Pin(SDA_PIN_NUM), freq=I2C_FREQ)
        lidar = LidarLiteV4(i2c=i2c, addr=0x62)
        lidar.configure_long_range()
        time.sleep_ms(100)
        return True
    except Exception as e:
        dbg("rebuild_i2c_and_lidar err: " + str(e))
        return False


def lidar_health_check():
    """
    Recovery-only watchdog (NO Pico self-reset).
    Also skips recovery during PI boot grace to avoid chasing noise during Pi power cycling.
    """
    global _lidar_recover_attempts, _last_recover_ms

    now = utime.ticks_ms()

    # Skip any LIDAR recovery while the Pi is in its boot grace period
    if utime.ticks_diff(now, _last_pi_power_on_ms) < PI_BOOT_GRACE_MS:
        return

    stale = utime.ticks_diff(now, _last_lidar_good_ms)

    if stale < LIDAR_STALE_MS:
        _lidar_recover_attempts = 0
        return

    if utime.ticks_diff(now, _last_recover_ms) < LIDAR_RECOVER_COOLDOWN_MS:
        return

    _last_recover_ms = now
    _lidar_recover_attempts += 1
    dbg("LIDAR stale " + str(stale) + "ms -> recover attempt " + str(_lidar_recover_attempts))

    # 1) light touch
    try:
        lidar.configure_long_range()
    except:
        pass

    # 2) heavier touch
    if _lidar_recover_attempts >= 2:
        rebuild_i2c_and_lidar()

    # 3) keep trying rebuilds, but NEVER reset the Pico
    if _lidar_recover_attempts >= LIDAR_MAX_RECOVERS:
        dbg("LIDAR still stale; continuing rebuild attempts (no Pico reset)")
        _lidar_recover_attempts = 0


# ----------------------------
# Garmin LIDAR-Lite v4 init
# ----------------------------
lidar = LidarLiteV4(i2c=i2c, addr=0x62)
lidar.configure_long_range()

# Give the LIDAR time to settle after configuration.
time.sleep_ms(1000)

# Test LIDAR 5 times on startup.
for i in range(5):
    cm = lidar.read_cm(retries=5, settle_ms=8, busy_timeout_ms=200)
    print("LIDAR cm:", cm)
    time.sleep_ms(500)


# ----------------------------
# BME280 + light sensor
# ----------------------------
scan = i2c.scan()
bme_addr = 0x77 if 0x77 in scan else (0x76 if 0x76 in scan else None)

if bme_addr is None:
    bme = None
    print("BME280 not found on I2C scan:", [hex(x) for x in scan])
else:
    bme = BME280.BME280(i2c=i2c, addr=bme_addr)
    print("BME280 found at", hex(bme_addr))

light_sensor = ADC(0)  # GP26


# ----------------------------
# Distance thresholds (inches)
# ----------------------------
DOOR_CLOSED_IN = 108
DOOR_OPEN_IN = 11
DOOR_VENT_IN = 75

# Clear a remembered vent state once the measured door position moves away
# from the configured vent location. This also handles movement from a vehicle
# remote, where the Pico never receives an OPEN or CLOSE command.
VENT_STATUS_EXIT_DEADBAND_IN = 4.0

LIGHT_LEVEL_ON = 30000
MAX_TIMEOUT = 30


# ----------------------------
# Globals
# ----------------------------
_last_good_distance_in = None
mapped = 0.0

# LIDAR sanity filter. The physical door target should remain close to the
# configured open/closed range. Readings outside this envelope are discarded.
LIDAR_MIN_VALID_IN = 5.0
LIDAR_MAX_VALID_IN = 140.0

# A single reading cannot legitimately jump this far between samples. Large
# changes must repeat closely before they are accepted, allowing genuine door
# movement/reacquisition while rejecting isolated values such as 202 inches.
LIDAR_MAX_SINGLE_JUMP_IN = 18.0
LIDAR_JUMP_CONFIRM_TOLERANCE_IN = 4.0
LIDAR_JUMP_CONFIRM_COUNT = 3
_lidar_jump_candidate_in = None
_lidar_jump_candidate_count = 0

LOOP_SLEEP_S = 0.05

# Environmental period: 60 seconds
ENV_PERIOD_S = 60.0
_last_env_ts_ms = 0


# ----------------------------
# Debounce / actions
# ----------------------------
def clear_flag(name):
    debounce_flags[name] = False


def service_pulses():
    """
    Turns relay outputs off when their non-blocking hold time has expired.
    Call this often from loops and the main loop.
    """
    global _motor_pulse_active, _light_pulse_active

    now = utime.ticks_ms()

    if _motor_pulse_active and utime.ticks_diff(now, _motor_pulse_until_ms) >= 0:
        MOTOR_MOVE.value(0)
        _motor_pulse_active = False

    if _light_pulse_active and utime.ticks_diff(now, _light_pulse_until_ms) >= 0:
        LIGHT_ON_OFF.value(0)
        _light_pulse_active = False


def motor_pulse(force=False):
    """
    Starts a garage button pulse without blocking.
    force=True allows STOP to pulse even when abort_motion is set.
    """
    global _motor_pulse_active, _motor_pulse_until_ms

    if abort_motion and not force:
        return False

    MOTOR_MOVE.value(1)
    _motor_pulse_active = True
    _motor_pulse_until_ms = utime.ticks_add(utime.ticks_ms(), button_hold_ms)
    return True


def light_pulse():
    """
    Starts a light button pulse without blocking.
    """
    global _light_pulse_active, _light_pulse_until_ms

    LIGHT_ON_OFF.value(1)
    _light_pulse_active = True
    _light_pulse_until_ms = utime.ticks_add(utime.ticks_ms(), button_hold_ms)
    return True


def wait_ms_with_service(ms):
    """
    Delay helper that keeps UART, relay timers, and STOP responsive.
    """
    end_ms = utime.ticks_add(utime.ticks_ms(), ms)
    while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
        service_pulses()
        check_uart()
        if abort_motion:
            break
        time.sleep_ms(10)


def wait_pulse_done_with_service():
    """
    Wait until the motor pulse finishes while keeping UART/STOP responsive.
    """
    while _motor_pulse_active:
        service_pulses()
        check_uart()
        if abort_motion:
            break
        time.sleep_ms(10)


def pulse_motor_for_stop():
    """
    STOP uses the same wall-button/motor trigger line.
    Non-blocking pulse is forced even if abort_motion is already true.
    """
    motor_pulse(force=True)


def stop_start_trigger():
    global stop_command, abort_motion, pending_command
    send_event("wall_stop")
    stop_command = True
    abort_motion = True
    pending_command = None


def enqueue_command(cmd):
    global pending_command, abort_motion

    if utime.ticks_diff(utime.ticks_ms(), _boot_ms) < BOOT_IGNORE_MS:
        return

    send_event("wall_" + cmd)
    abort_motion = False
    pending_command = cmd


def debounce_handler(name, action_func):
    def handler(pin):
        if debounce_flags[name]:
            return

        if utime.ticks_diff(utime.ticks_ms(), _boot_ms) < BOOT_IGNORE_MS:
            return

        if not stable_low(pin, 40):
            return

        debounce_flags[name] = True
        action_func()
        debounce_timers[name].init(
            mode=Timer.ONE_SHOT,
            period=DEBOUNCE_MS,
            callback=lambda t: clear_flag(name)
        )
    return handler


def light_turn_on_off():
    send_event("wall_light")
    light_pulse()


def safe_motor(wait_for_done=True):
    """
    Start a non-blocking motor pulse.
    By default this waits only for the pulse to finish while still servicing UART/LIDAR-safe tasks.
    """
    if not motor_pulse(force=False):
        return False

    if wait_for_done:
        wait_pulse_done_with_service()

    return True


# ----------------------------
# UART send helpers
# ----------------------------
def send_position(mapped_pos, actual_distance):
    """
    FAST: sent every position update, includes light info for HTML bulb.
    """
    try:
        light_value = light_sensor.read_u16()
        light_detected = 'on' if light_value >= LIGHT_LEVEL_ON else 'off'

        data = ujson.dumps({
            'position_percent': round(mapped_pos, 1),
            'position_in': round(actual_distance, 1),
            'light': light_detected,
            'light_value': int(light_value),
        })
        uart.write(data + '\n')
    except:
        pass


def send_vent_status(vent):
    try:
        data = ujson.dumps({'vent_status': vent})
        uart.write(data + '\n')
    except:
        pass


def _as_float_strip_units(x):
    # Accepts numbers or strings like "24.3C" or "51.2%"
    s = str(x).strip()
    s = s.replace("C", "").replace("c", "").replace("%", "")
    return float(s)


def send_environmental_data():
    """
    SLOW: sent once per minute, temp/humidity only (no light here).
    """
    if bme is None:
        return
    try:
        temp_c = _as_float_strip_units(bme.temperature)
        temp_f = (temp_c * 9 / 5) + 32

        humidity = _as_float_strip_units(bme.humidity)
        humidity_int = int(humidity)

        data = ujson.dumps({
            'temperature_f': round(temp_f, 1),
            'humidity': humidity_int
        })
        uart.write(data + '\n')
    except:
        pass


# ----------------------------
# Position read (returns last good instead of None)
# ----------------------------
def get_position(sample_count=3, delay=0.001, settle_ms=8):
    global mapped, _last_good_distance_in, vent_status
    global _last_lidar_good_ms, _last_lidar_value_in
    global _lidar_jump_candidate_in, _lidar_jump_candidate_count

    valid_readings = []
    for _ in range(sample_count):
        distance_cm = lidar.read_cm(retries=5, settle_ms=settle_ms, busy_timeout_ms=200)
        if distance_cm is None:
            time.sleep(delay)
            continue

        distance_in = distance_cm / 2.54

        # Reject impossible garage-door measurements before averaging. This
        # blocks the repeatable bogus ~202-inch reading from reaching motion,
        # vent, UART, or HTML position logic.
        if LIDAR_MIN_VALID_IN <= distance_in <= LIDAR_MAX_VALID_IN:
            valid_readings.append(distance_in)

        time.sleep(delay)

    if valid_readings:
        # Median is more resistant than an average to one bad sample.
        valid_readings.sort()
        count = len(valid_readings)
        if count & 1:
            measured_in = valid_readings[count // 2]
        else:
            measured_in = (valid_readings[(count // 2) - 1] + valid_readings[count // 2]) / 2.0

        accepted_in = measured_in

        if _last_good_distance_in is not None:
            jump = abs(measured_in - _last_good_distance_in)

            if jump > LIDAR_MAX_SINGLE_JUMP_IN:
                # Do not accept a large discontinuity until several successive
                # calls report approximately the same new distance.
                if (_lidar_jump_candidate_in is not None and
                        abs(measured_in - _lidar_jump_candidate_in) <= LIDAR_JUMP_CONFIRM_TOLERANCE_IN):
                    _lidar_jump_candidate_count += 1
                    _lidar_jump_candidate_in = (
                        (_lidar_jump_candidate_in * (_lidar_jump_candidate_count - 1)) + measured_in
                    ) / _lidar_jump_candidate_count
                else:
                    _lidar_jump_candidate_in = measured_in
                    _lidar_jump_candidate_count = 1

                if _lidar_jump_candidate_count < LIDAR_JUMP_CONFIRM_COUNT:
                    return _last_good_distance_in

                accepted_in = _lidar_jump_candidate_in
                _lidar_jump_candidate_in = None
                _lidar_jump_candidate_count = 0
            else:
                _lidar_jump_candidate_in = None
                _lidar_jump_candidate_count = 0

        m = adafruit_simplemath.map_range(accepted_in, DOOR_OPEN_IN, DOOR_CLOSED_IN, 0, 100)
        if m < 0:
            m = 0.0
        elif m > 100:
            m = 100.0

        mapped = float(m)
        _last_good_distance_in = float(accepted_in)

        _last_lidar_good_ms = utime.ticks_ms()
        _last_lidar_value_in = float(accepted_in)

        send_position(mapped, accepted_in)

        # vent_status used to remain latched at 1 until an OPEN/CLOSE command
        # arrived from the Pi. A vehicle remote bypasses the Pi, so the HTML
        # could continue to say VENTED after the door had opened or closed.
        # Clear that remembered state as soon as a confirmed LIDAR position is
        # safely outside the vent zone. Do not automatically set it here, which
        # avoids briefly reporting VENTED while a moving door passes the target.
        if vent_status == 1 and abs(accepted_in - DOOR_VENT_IN) > VENT_STATUS_EXIT_DEADBAND_IN:
            vent_status = 0
            send_vent_status(vent_status)

        return accepted_in

    if _last_good_distance_in is not None:
        return _last_good_distance_in

    return None


# ----------------------------
# UART command handling
# ----------------------------
def handle_command(cmd):
    """
    Handles commands from the Pi Zero/web app.
    Accepts: open, close, vent, stop, light
    """
    global stop_command, abort_motion, pending_command

    if cmd is None:
        return

    try:
        cmd = str(cmd).strip().lower()
    except:
        return

    if cmd == "":
        return

    if UPDATE_MODE:
        send_update_status("busy", reason="update_mode")
        return

    if cmd == "stop":
        send_event("app_stop")
        stop_command = True
        abort_motion = True
        pending_command = None

    elif cmd in ("open", "close", "vent"):
        send_event("app_" + cmd)
        abort_motion = False
        pending_command = cmd

    elif cmd == "light":
        send_event("app_light")
        light_turn_on_off()


# ----------------------------
# UART config + heartbeat updates from Pi Zero
# ----------------------------
def _process_uart_line(line_str):
    global DOOR_VENT_IN, DOOR_OPEN_IN, DOOR_CLOSED_IN, LIGHT_LEVEL_ON
    global _last_hb_ms, _last_net_ms

    if not line_str:
        return

    # First try JSON.
    try:
        msg = ujson.loads(line_str)

        # Firmware/version/update commands MUST be handled before the normal
        # UPDATE_MODE blocking logic, or chunks get rejected as "busy".
        if 'cmd' in msg:
            raw_cmd = str(msg.get('cmd', '')).strip().lower()
            if raw_cmd == 'fw_version':
                send_fw_version()
                return
            elif raw_cmd == 'update_start':
                update_start(msg)
                return
            elif raw_cmd == 'update_chunk':
                update_chunk(msg)
                return
            elif raw_cmd == 'update_end':
                update_end(msg)
                return
            elif raw_cmd == 'update_cancel':
                update_cancel("zero_cancel")
                return

        # During update mode, only update commands and heartbeat/net are allowed.
        if UPDATE_MODE:
            if 'hb' in msg:
                _last_hb_ms = utime.ticks_ms()
            if 'net' in msg:
                _last_net_ms = utime.ticks_ms()
            return

        # Heartbeat: {"hb":1}
        if 'hb' in msg:
            _last_hb_ms = utime.ticks_ms()

        # Network OK: {"net":1}  (status-only)
        if 'net' in msg:
            _last_net_ms = utime.ticks_ms()

        # Web/app commands.
        if 'cmd' in msg:
            handle_command(msg.get('cmd'))
        if 'command' in msg:
            handle_command(msg.get('command'))
        if 'action' in msg:
            handle_command(msg.get('action'))

        # Existing config updates
        if 'vent_distance' in msg:
            DOOR_VENT_IN = int(msg['vent_distance'])
        if 'min_distance' in msg:
            DOOR_OPEN_IN = int(msg['min_distance'])
        if 'max_distance' in msg:
            DOOR_CLOSED_IN = int(msg['max_distance'])
        if 'light_level_on' in msg:
            LIGHT_LEVEL_ON = int(msg['light_level_on'])

    except Exception:
        # With buffered UART, parse errors should be rare. During update mode,
        # ignore bad lines instead of replying bad_json, because that can cause
        # the Zero to wait on the wrong response while the Pico is still alive.
        if UPDATE_MODE:
            return

        # Also support plain text commands like STOP, OPEN, CLOSE, VENT, LIGHT.
        handle_command(line_str)


def check_uart():
    global _uart_rx_buffer

    # Read raw bytes and only process complete newline-terminated lines.
    # uart.readline() can return partial data on MicroPython when bytes arrive
    # slowly, which was causing bad_json during firmware transfer.
    try:
        while uart.any():
            chunk = uart.read()
            if not chunk:
                break
            _uart_rx_buffer += chunk

            # Prevent a damaged/no-newline stream from eating all RAM.
            if len(_uart_rx_buffer) > MAX_UART_BUFFER:
                _uart_rx_buffer = b""
                if UPDATE_MODE:
                    send_update_status("failed", reason="rx_buffer_overflow")
                break

            while b"\n" in _uart_rx_buffer:
                line, _uart_rx_buffer = _uart_rx_buffer.split(b"\n", 1)
                try:
                    line_str = line.decode().strip()
                except Exception:
                    line_str = ""
                _process_uart_line(line_str)
    except Exception:
        pass

# ----------------------------
# Movement control (robust comparisons)
# ----------------------------
def start_move(action):
    global vent_status, abort_motion
    send_event("motion_" + action)
    abort_motion = False
    deadband = 1.0

    current_in = get_position(sample_count=2, delay=0.001, settle_ms=8)
    if current_in is None:
        return

    def read_in():
        # Fast single sample for motion tracking.
        return get_position(sample_count=1, delay=0.001, settle_ms=5)

    if action == 'open':
        vent_status = 0
        send_vent_status(vent_status)

        if mapped <= 0:
            return

        safe_motor()

        # Short delay so HTML simulation starts sooner.
        wait_ms_with_service(250)
        check_uart()
        if abort_motion:
            return

        p = read_in()
        if p is None:
            return

        # If distance went the wrong way, pulse again to reverse/stop/restart depending opener state.
        if p >= current_in:
            safe_motor()
            wait_ms_with_service(250)
            check_uart()
            if abort_motion:
                return
            safe_motor()

    elif action == 'close':
        vent_status = 0
        send_vent_status(vent_status)

        if mapped >= 100:
            return

        safe_motor()

        # Short delay so HTML simulation starts sooner.
        wait_ms_with_service(250)
        check_uart()
        if abort_motion:
            return

        p = read_in()
        if p is None:
            return

        # If distance went the wrong way, pulse again to reverse/stop/restart depending opener state.
        if p <= current_in:
            safe_motor()
            wait_ms_with_service(250)
            check_uart()
            if abort_motion:
                return
            safe_motor()

    elif action == 'vent':
        vent_status = 1

        if abs(current_in - DOOR_VENT_IN) <= deadband:
            send_vent_status(vent_status)
            return

        start_time = time.time()
        p = read_in()
        if p is None:
            return

        if p < DOOR_VENT_IN:
            safe_motor()
            wait_ms_with_service(250)
            check_uart()
            if abort_motion:
                return

            while (time.time() - start_time) <= MAX_TIMEOUT and not abort_motion:
                check_uart()
                if abort_motion:
                    break

                p = read_in()
                if p is None:
                    time.sleep(0.02)
                    continue

                if p >= (DOOR_VENT_IN - deadband):
                    break

                time.sleep(0.02)

            if not abort_motion:
                safe_motor()
                send_vent_status(vent_status)

        elif p > DOOR_VENT_IN:
            safe_motor()
            wait_ms_with_service(250)
            check_uart()
            if abort_motion:
                return

            while (time.time() - start_time) <= MAX_TIMEOUT and not abort_motion:
                check_uart()
                if abort_motion:
                    break

                p = read_in()
                if p is None:
                    time.sleep(0.02)
                    continue

                if p <= (DOOR_VENT_IN + deadband):
                    break

                time.sleep(0.02)

            if not abort_motion:
                safe_motor()
                send_vent_status(vent_status)


# ----------------------------
# Interrupt bindings
# ----------------------------
Pin(STOP_PIN, Pin.IN, Pin.PULL_UP).irq(trigger=Pin.IRQ_FALLING, handler=debounce_handler('stop', stop_start_trigger))
Pin(OPEN_PIN, Pin.IN, Pin.PULL_UP).irq(trigger=Pin.IRQ_FALLING, handler=debounce_handler('open', lambda: enqueue_command('open')))
Pin(CLOSE_PIN, Pin.IN, Pin.PULL_UP).irq(trigger=Pin.IRQ_FALLING, handler=debounce_handler('close', lambda: enqueue_command('close')))
Pin(VENT_PIN, Pin.IN, Pin.PULL_UP).irq(trigger=Pin.IRQ_FALLING, handler=debounce_handler('vent', lambda: enqueue_command('vent')))
Pin(LIGHT_PIN, Pin.IN, Pin.PULL_UP).irq(trigger=Pin.IRQ_FALLING, handler=debounce_handler('light', light_turn_on_off))


# ----------------------------
# Pi power-cycle helpers
# ----------------------------
def power_cycle_pi(reason="no_hb"):
    global _last_pi_reset_ms, _last_pi_power_on_ms, _last_hb_ms, _last_net_ms

    dbg("PI RESET: " + reason)

    _last_pi_reset_ms = utime.ticks_ms()

    # OFF
    pi_pwr.value(0)
    time.sleep_ms(PI_POWER_OFF_MS)

    # ON
    pi_pwr.value(1)
    _last_pi_power_on_ms = utime.ticks_ms()

    # reset timers so we don't immediately reset again
    _last_hb_ms = utime.ticks_ms()
    _last_net_ms = utime.ticks_ms()


def pi_heartbeat_watchdog():
    """
    Heartbeat-only watchdog. NET is tracked for info but NEVER triggers reset.
    """
    global hb_miss_count
    now = utime.ticks_ms()

    if UPDATE_MODE:
        hb_miss_count = 0
        return

    # boot grace
    if utime.ticks_diff(now, _last_pi_power_on_ms) < PI_BOOT_GRACE_MS:
        hb_miss_count = 0
        return

    # cooldown
    if utime.ticks_diff(now, _last_pi_reset_ms) < PI_RESET_COOLDOWN_MS:
        hb_miss_count = 0
        return

    # never reset during motion / commands
    if pending_command is not None or stop_command:
        hb_miss_count = 0
        return

    hb_age = utime.ticks_diff(now, _last_hb_ms)

    # Heartbeat miss counting
    if hb_age > HB_TIMEOUT_MS:
        hb_miss_count += 1
    else:
        hb_miss_count = 0

    if hb_miss_count >= HB_MISSES_TO_RESET:
        dbg("WATCHDOG HB TRIP hb_age_ms=" + str(hb_age) +
            " hb_miss=" + str(hb_miss_count))
        power_cycle_pi("HB misses=" + str(hb_miss_count) + " age_ms=" + str(hb_age))
        hb_miss_count = 0
        return


# ----------------------------
# Main loop
# ----------------------------
# Motor and light relay pulses are non-blocking.
# service_pulses() must run every loop.
while True:
    check_uart()
    service_pulses()
    pi_heartbeat_watchdog()

    if UPDATE_MODE:
        time.sleep_ms(20)
        continue

    if stop_command:
        send_event("motion_stop")
        pulse_motor_for_stop()
        wait_pulse_done_with_service()
        stop_command = False
        abort_motion = False

    if pending_command:
        cmd = pending_command
        pending_command = None
        start_move(cmd)

    # Position updates for HTML simulation and status.
    get_position(sample_count=2, delay=0.001, settle_ms=8)

    # Recovery-only LIDAR watchdog (no Pico reset)
    lidar_health_check()

    # 60s environmental updates (temp/humidity only)
    now_ms = utime.ticks_ms()
    if utime.ticks_diff(now_ms, _last_env_ts_ms) >= int(ENV_PERIOD_S * 1000):
        _last_env_ts_ms = now_ms
        send_environmental_data()

    time.sleep(LOOP_SLEEP_S)
