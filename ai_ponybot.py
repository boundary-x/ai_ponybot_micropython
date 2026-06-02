from microbit import i2c, sleep
from machine import time_pulse_us
import math
import utime
import ustruct

PCA9685_ADDRESS = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L = 0xFA
ALL_LED_ON_H = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

RESTART = 0x80
SLEEP = 0x10
ALLCALL = 0x01
OUTDRV = 0x04
RESET = 0x00

class _PWMController:

    def __init__(self, i2c, address=PCA9685_ADDRESS):
        self._i2c = i2c
        self.address = address
        i2c.write(self.address, bytearray([MODE1, RESET]))
        self.set_all_pwm(0, 0)
        i2c.write(self.address, bytearray([MODE2, OUTDRV]))
        i2c.write(self.address, bytearray([MODE1, ALLCALL]))
        sleep(5)

        self._i2c.write(self.address, bytearray([MODE1]))
        mode1 = ustruct.unpack('<B', self._i2c.read(self.address, 1))[0]
        self._i2c.write(self.address, bytearray([MODE1, mode1 & ~SLEEP]))
        sleep(5)

    def set_pwm_frequency(self, freq_hz):
        prescaleval = 25000000.0 / 4096 / freq_hz - 1.0
        prescale = int(math.floor(prescaleval + 0.5))

        self._i2c.write(self.address, bytearray([MODE1]))
        oldmode = ustruct.unpack('<B', self._i2c.read(self.address, 1))[0]
        newmode = (oldmode & 0x7F) | 0x10
        self._i2c.write(self.address, bytearray([MODE1, newmode]))
        self._i2c.write(self.address, bytearray([PRESCALE, prescale]))
        self._i2c.write(self.address, bytearray([MODE1, oldmode]))
        sleep(5)
        self._i2c.write(self.address, bytearray([MODE1, oldmode | RESTART]))

    def set_pwm_duty_cycle(self, channel, on, off):
        i2c.write(self.address, bytearray([LED0_ON_L + 4 * channel, on & 0xFF]))
        i2c.write(self.address, bytearray([LED0_ON_H + 4 * channel, on >> 8]))
        i2c.write(self.address, bytearray([LED0_OFF_L + 4 * channel, off & 0xFF]))
        i2c.write(self.address, bytearray([LED0_OFF_H + 4 * channel, off >> 8]))

    def set_all_pwm(self, on, off):
        i2c.write(self.address, bytearray([ALL_LED_ON_L, on & 0xFF]))
        i2c.write(self.address, bytearray([ALL_LED_ON_H, on >> 8]))
        i2c.write(self.address, bytearray([ALL_LED_OFF_L, off & 0xFF]))
        i2c.write(self.address, bytearray([ALL_LED_OFF_H, off >> 8]))

    def set_duty(self, channel, value):
        if not 0 <= value <= 4095:
            raise ValueError("듀티 값은 0~4095 범위여야 합니다.")
        if value == 0:
            self.set_pwm_duty_cycle(channel, 0, 4096)
        elif value == 4095:
            self.set_pwm_duty_cycle(channel, 4096, 0)
        else:
            self.set_pwm_duty_cycle(channel, 0, value)

class PonyMotor:

    def __init__(self, i2c, motor_channels=None, pwm_freq=50):
        self.pwm = _PWMController(i2c)
        self.pwm.set_pwm_frequency(pwm_freq)

        if motor_channels is None:
            self.motor_channels = {
                1: (7, 6),
                2: (5, 4),
                3: (2, 3),
                4: (0, 1)
            }
        else:
            self.motor_channels = motor_channels

    def move(self, motor_num, speed_percent):
        if motor_num not in self.motor_channels:
            raise ValueError("정의되지 않은 모터 번호: {}".format(motor_num))

        speed_percent = max(-100, min(100, speed_percent))
        pwm_value = int(abs(speed_percent) * 40.95)

        ch1, ch2 = self.motor_channels[motor_num]

        if speed_percent > 0:
            self.pwm.set_duty(ch1, pwm_value)
            self.pwm.set_duty(ch2, 0)
        elif speed_percent < 0:
            self.pwm.set_duty(ch1, 0)
            self.pwm.set_duty(ch2, pwm_value)
        else:
            self.pwm.set_duty(ch1, 0)
            self.pwm.set_duty(ch2, 0)

    def drive(self, direction, speed=0):
        speed = max(0, min(100, speed))
        if direction == "forward":
            self.move(1, speed)
            self.move(2, speed)
            self.move(3, speed)
            self.move(4, speed)
        elif direction == "backward":
            self.move(1, -speed)
            self.move(2, -speed)
            self.move(3, -speed)
            self.move(4, -speed)
        elif direction == "left":
            self.move(1, speed)
            self.move(2, speed)
            self.move(3, -speed)
            self.move(4, -speed)
        elif direction == "right":
            self.move(1, -speed)
            self.move(2, -speed)
            self.move(3, speed)
            self.move(4, speed)
        elif direction == "stop":
            for i in range(1, 5):
                self.move(i, 0)
        else:
            raise ValueError("direction must be one of: 'forward', 'backward', 'left', 'right', 'stop'")

    def mecanum(self, direction_code, speed=0):
        speed = max(0, min(100, speed))

        if direction_code == 7:
            self.move(1, speed); self.move(2, 0); self.move(3, speed); self.move(4, 0)
        elif direction_code == 9:
            self.move(1, 0); self.move(2, speed); self.move(3, 0); self.move(4, speed)
        elif direction_code == 4:
            self.move(1, speed); self.move(2, -speed); self.move(3, speed); self.move(4, -speed)
        elif direction_code == 6:
            self.move(1, -speed); self.move(2, speed); self.move(3, -speed); self.move(4, speed)
        elif direction_code == 1:
            self.move(1, 0); self.move(2, -speed); self.move(3, 0); self.move(4, -speed)
        elif direction_code == 3:
            self.move(1, -speed); self.move(2, 0); self.move(3, -speed); self.move(4, 0)
        elif direction_code == 8:
            self.move(1, speed); self.move(2, speed); self.move(3, speed); self.move(4, speed)
        elif direction_code == 2:
            self.move(1, -speed); self.move(2, -speed); self.move(3, -speed); self.move(4, -speed)
        elif direction_code == 5:
            for i in range(1, 5):
                self.move(i, 0)
        else:
            raise ValueError("direction_code는 1~9 중 하나여야 합니다.")

class PonyServo:

    def __init__(self, pwm, min_us=600, max_us=2400, degrees=180, pwm_freq=50):
        self.pwm = pwm
        self.degrees = degrees
        self._period_us = 1000000 // pwm_freq
        self.min_duty = self._us_to_duty(min_us)
        self.max_duty = self._us_to_duty(max_us)

    def _us_to_duty(self, us):
        return int(4095 * us / self._period_us)

    def set_angle(self, servo_num, angle):
        if not 1 <= servo_num <= 8:
            raise ValueError("서보 번호는 1~8 사이여야 합니다.")
        angle = max(0, min(self.degrees, angle))
        duty_range = self.max_duty - self.min_duty
        duty = int(self.min_duty + duty_range * angle / self.degrees)
        channel = servo_num + 7
        self.pwm.set_duty(channel, duty)

    def release(self, servo_num):
        if not 1 <= servo_num <= 8:
            raise ValueError("서보 번호는 1~8 사이여야 합니다.")
        channel = servo_num + 7
        self.pwm.set_duty(channel, 0)

class PonySonar:

    def __init__(self, timeout_us=30000):
        self.timeout = timeout_us

    def measure(self, trig_pin, echo_pin):
        trig_pin.write_digital(0)
        utime.sleep_us(2)
        trig_pin.write_digital(1)
        utime.sleep_us(10)
        trig_pin.write_digital(0)

        try:
            duration = time_pulse_us(echo_pin, 1, self.timeout)
        except OSError:
            return -1

        distance = int(duration * 0.017)
        if distance < 2 or distance > 400:
            return -1
        return distance

_FONT_A = bytes([
    0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x07,0x00,0x07,0x00,0x00,
    0x14,0x7F,0x14,0x7F,0x14, 0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62,
    0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00, 0x00,0x1C,0x22,0x41,0x00,
    0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
    0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00,
    0x20,0x10,0x08,0x04,0x02, 0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00,
    0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31, 0x18,0x14,0x12,0x7F,0x10,
    0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00,
    0x00,0x56,0x36,0x00,0x00, 0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14,
    0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06, 0x32,0x49,0x79,0x41,0x3E,
    0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
    0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01,
    0x3E,0x41,0x49,0x49,0x7A, 0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00,
    0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41, 0x7F,0x40,0x40,0x40,0x40,
    0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
    0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46,
    0x46,0x49,0x49,0x49,0x31, 0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F,
    0x1F,0x20,0x40,0x20,0x1F, 0x7F,0x20,0x18,0x20,0x7F, 0x63,0x14,0x08,0x14,0x63,
    0x03,0x04,0x78,0x04,0x03, 0x61,0x51,0x49,0x45,0x43,
])

_FONT_B = bytes([
    0x20,0x54,0x54,0x54,0x78, 0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20,
    0x38,0x44,0x44,0x48,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02,
    0x0C,0x52,0x52,0x52,0x3E, 0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00,
    0x20,0x40,0x44,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00, 0x00,0x41,0x7F,0x40,0x00,
    0x7C,0x04,0x18,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
    0x7C,0x14,0x14,0x14,0x08, 0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08,
    0x48,0x54,0x54,0x54,0x20, 0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C,
    0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C, 0x44,0x28,0x10,0x28,0x44,
    0x0C,0x50,0x50,0x50,0x3C, 0x44,0x64,0x54,0x4C,0x44,
])

_FONT_BLANK = bytes(5)

class PonyOLED:

    def __init__(self, i2c, addr=0x3C):
        self.i2c = i2c
        self.addr = addr
        self.width = 128
        self.height = 64
        self.pages = self.height // 8
        self.buffer = bytearray(1 + self.width * self.pages)
        self.buffer[0] = 0x40

        i2c.init(freq=400000)
        self.init()

    def send_cmd(self, cmd):
        self.i2c.write(self.addr, bytearray([0x00, cmd]))

    def init(self):
        self.i2c.write(self.addr, bytearray([
            0x00,
            0xAE,
            0xA4,
            0xD5, 0xF0,
            0xA8, 0x3F,
            0xD3, 0x00,
            0x40,
            0x8D, 0x14,
            0x20, 0x00,
            0x21, 0x00, 0x7F,
            0x22, 0x00, 0x07,
            0xA1,
            0xC8,
            0xDA, 0x12,
            0x81, 0xCF,
            0xD9, 0xF1,
            0xDB, 0x40,
            0xA6,
            0xD6, 0x00,
            0xAF,
        ]))
        self.clear()

    def clear(self):
        for i in range(1, len(self.buffer)):
            self.buffer[i] = 0
        self.show()

    def show(self):
        self.i2c.write(self.addr, self.buffer)

    def invert(self, invert=True):
        self.send_cmd(0xA7 if invert else 0xA6)

    def power(self, on=True):
        self.send_cmd(0xAF if on else 0xAE)

    def draw_pixel(self, x, y, color=1):
        if not (0 <= x < self.width and 0 <= y < self.height):
            return
        page = y >> 3
        shift = y & 7
        index = 1 + x + page * self.width
        if color:
            self.buffer[index] |= (1 << shift)
        else:
            self.buffer[index] &= ~(1 << shift)

    def draw_hline(self, x, y, length, color=1):
        if not (0 <= y < self.height):
            return
        page = y >> 3
        shift = y & 7
        bit = 1 << shift
        x_end = min(x + length, self.width)
        x_start = max(x, 0)
        base = 1 + page * self.width
        if color:
            for xi in range(x_start, x_end):
                self.buffer[base + xi] |= bit
        else:
            mask = ~bit & 0xFF
            for xi in range(x_start, x_end):
                self.buffer[base + xi] &= mask

    def draw_vline(self, x, y, length, color=1):
        for i in range(length):
            self.draw_pixel(x, y + i, color)

    def draw_rect(self, x1, y1, x2, y2, color=1):
        w = x2 - x1 + 1
        self.draw_hline(x1, y1, w, color)
        self.draw_hline(x1, y2, w, color)
        self.draw_vline(x1, y1, y2 - y1 + 1, color)
        self.draw_vline(x2, y1, y2 - y1 + 1, color)

    def draw_char(self, x, y, char, color=1):
        code = ord(char) if isinstance(char, str) else char
        if 32 <= code <= 90:
            idx = (code - 32) * 5
            glyph = _FONT_A[idx:idx + 5]
        elif 97 <= code <= 122:
            idx = (code - 97) * 5
            glyph = _FONT_B[idx:idx + 5]
        else:
            glyph = _FONT_BLANK
        for col in range(5):
            line = glyph[col]
            for row in range(8):
                if (line >> row) & 1:
                    self.draw_pixel(x + col, y + row, color)
                elif not color:
                    self.draw_pixel(x + col, y + row, 0)

    def draw_text(self, x, y, text, color=1):
        text = str(text)
        for i, char in enumerate(text):
            self.draw_char(x + i * 6, y, char, color)

    def write_line(self, line_num, text, color=1):
        if 0 <= line_num < 8:
            self.draw_text(0, line_num * 8, str(text), color)

class PonyColor:
    def __init__(self, i2c, version=2, address=0x29):
        self.i2c = i2c
        self.address = address
        self.is_setup = False
        self.set_profile(version)

    def _write_byte(self, reg, value):
        self.i2c.write(self.address, bytes([0x80 | reg, value]))

    def _read_word(self, reg):
        self.i2c.write(self.address, bytes([0x80 | reg]))
        data = self.i2c.read(self.address, 2)
        return data[1] << 8 | data[0]

    def _read_raw_data(self):
        self.setup()
        return [self._read_word(0x14 + i * 2) for i in range(4)]

    def _read_all(self):
        raw = self._read_raw_data()
        c = raw[0]
        if c == 0:
            return 0, 0, 0, 0
        r_raw, g_raw, b_raw = raw[1], raw[2], raw[3]
        r = max(0, min(255, int((r_raw - self.r_min) / max(1, self.r_max - self.r_min) * 255)))
        g = max(0, min(255, int((g_raw - self.g_min) / max(1, self.g_max - self.g_min) * 255)))
        b = max(0, min(255, int((b_raw - self.b_min) / max(1, self.b_max - self.b_min) * 255)))
        return c, r, g, b

    def setup(self):
        if self.is_setup: return
        self.is_setup = True
        self._write_byte(0x00, 0x03)
        self._write_byte(0x01, 0xC0)
        self._write_byte(0x0F, 0x03)

    def set_profile(self, version):
        if version == 1 or version == "V1":
            self.r_min, self.g_min, self.b_min = 1883, 1866, 1371
            self.r_max, self.g_max, self.b_max = 18837, 20336, 14248
        else:
            self.r_min, self.g_min, self.b_min = 1513, 1006, 800
            self.r_max, self.g_max, self.b_max = 12590, 9557, 7167

    def light(self):
        return self._read_raw_data()[0]

    def rgb(self):
        _, r, g, b = self._read_all()
        return [r, g, b]

    def is_color(self, target, threshold=40):
        c, r, g, b = self._read_all()
        if c == 0 or (r + g + b) < 60: return False
        if target == "red":      return r > g and r > b and r > g * 1.5
        elif target == "yellow": return r > b and g > b and r > g and r <= g * 1.5
        elif target == "green":  return g > r and g >= b and g > r * 1.4
        elif target == "blue":   return b > r and b > g
        return False

    def is_in_range(self, min_r, max_r, min_g, max_g, min_b, max_b):
        _, r, g, b = self._read_all()
        return (min_r <= r <= max_r) and (min_g <= g <= max_g) and (min_b <= b <= max_b)
