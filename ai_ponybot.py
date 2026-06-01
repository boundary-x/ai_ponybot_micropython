# ai_ponybot.py

from microbit import i2c, sleep
from machine import time_pulse_us
import math
import utime
import ustruct


# PCA9685 레지스터 상수
PCA9685_ADDRESS = 0x40
MODE1 = 0x00
MODE2 = 0x0
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L = 0xFA
ALL_LED_ON_H = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

# 비트 플래그
RESTART = 0x80
SLEEP = 0x10
ALLCALL = 0x01
OUTDRV = 0x04
RESET = 0x00

class _PWMController:
    """PCA9685 PWM 제어 클래스 (내부용)."""

    def __init__(self, i2c, address=PCA9685_ADDRESS):
        self.address = address
        i2c.write(self.address, bytearray([MODE1, RESET]))
        self.set_all_pwm(0, 0)
        i2c.write(self.address, bytearray([MODE2, OUTDRV]))
        i2c.write(self.address, bytearray([MODE1, ALLCALL]))
        sleep(5)

        # 슬립 해제
        i2c.write(self.address, bytearray([MODE1]))
        mode1 = ustruct.unpack('<B', i2c.read(self.address, 1))[0]
        i2c.write(self.address, bytearray([MODE1, mode1 & ~SLEEP]))
        sleep(5)

    def set_pwm_frequency(self, freq_hz):
        """PWM 주파수 설정"""
        prescaleval = 25000000.0 / 4096 / freq_hz - 1.0
        prescale = int(math.floor(prescaleval + 0.5))

        i2c.write(self.address, bytearray([MODE1]))
        oldmode = ustruct.unpack('<B', i2c.read(self.address, 1))[0]
        newmode = (oldmode & 0x7F) | 0x10
        i2c.write(self.address, bytearray([MODE1, newmode]))
        i2c.write(self.address, bytearray([PRESCALE, prescale]))
        i2c.write(self.address, bytearray([MODE1, oldmode]))
        sleep(5)
        i2c.write(self.address, bytearray([MODE1, oldmode | RESTART]))

    def set_pwm_duty_cycle(self, channel, on, off):
        """PWM 채널 설정"""
        i2c.write(self.address, bytearray([LED0_ON_L + 4 * channel, on & 0xFF]))
        i2c.write(self.address, bytearray([LED0_ON_H + 4 * channel, on >> 8]))
        i2c.write(self.address, bytearray([LED0_OFF_L + 4 * channel, off & 0xFF]))
        i2c.write(self.address, bytearray([LED0_OFF_H + 4 * channel, off >> 8]))

    def set_all_pwm(self, on, off):
        """전체 채널 PWM 설정"""
        i2c.write(self.address, bytearray([ALL_LED_ON_L, on & 0xFF]))
        i2c.write(self.address, bytearray([ALL_LED_ON_H, on >> 8]))
        i2c.write(self.address, bytearray([ALL_LED_OFF_L, off & 0xFF]))
        i2c.write(self.address, bytearray([ALL_LED_OFF_H, off >> 8]))

    def set_duty(self, channel, value):
        """듀티 설정 (0~4095)"""
        if not 0 <= value <= 4095:
            raise ValueError("듀티 값은 0~4095 범위여야 합니다.")
        if value == 0:
            self.set_pwm_duty_cycle(channel, 0, 4096)
        elif value == 4095:
            self.set_pwm_duty_cycle(channel, 4096, 0)
        else:
            self.set_pwm_duty_cycle(channel, 0, value)

class PonyMotor:
    """포니봇의 DC 모터 제어 클래스."""

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
        """
        모터를 이동시킵니다.
        모터 번호: 1~4
        속도 비율: -100 ~ 100
        """
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
        """기본 방향 주행 함수.
        direction: "forward", "backward", "left", "right", "stop"
        speed: 0~100
        """
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
        """
        메카넘휠 특수 방향 이동.
        direction_code:
          7=좌상단, 9=우상단, 4=좌로, 6=우로,
          1=좌하단, 3=우하단,
          8=직진, 2=후진, 5=정지
        speed: 0~100 (5번 정지 시 생략 가능)
        """
        speed = max(0, min(100, speed))
    
        if direction_code == 7:  # 좌상단 대각선
            self.move(1, speed)
            self.move(2, 0)
            self.move(3, speed)
            self.move(4, 0)
        elif direction_code == 9:  # 우상단 대각선
            self.move(1, 0)
            self.move(2, speed)
            self.move(3, 0)
            self.move(4, speed)
        elif direction_code == 4:  # 좌로 수평 이동
            self.move(1, speed)
            self.move(2, -speed)
            self.move(3, speed)
            self.move(4, -speed)
        elif direction_code == 6:  # 우로 수평 이동
            self.move(1, -speed)
            self.move(2, speed)
            self.move(3, -speed)
            self.move(4, speed)
        elif direction_code == 1:  # 좌하단 대각선
            self.move(1, 0)
            self.move(2, -speed)
            self.move(3, 0)
            self.move(4, -speed)
        elif direction_code == 3:  # 우하단 대각선
            self.move(1, -speed)
            self.move(2, 0)
            self.move(3, -speed)
            self.move(4, 0)
        elif direction_code == 8:  # 직진
            self.move(1, speed)
            self.move(2, speed)
            self.move(3, speed)
            self.move(4, speed)
        elif direction_code == 2:  # 후진
            self.move(1, -speed)
            self.move(2, -speed)
            self.move(3, -speed)
            self.move(4, -speed)
        elif direction_code == 5:  # 정지
            for i in range(1, 5):
                self.move(i, 0)
        else:
            raise ValueError("direction_code는 1, 2, 3, 4, 5, 6, 7, 8, 9 중 하나여야 합니다.")
            

class PonyServo:
    """포니봇 서보모터 제어 클래스 (PCA9685 기반)."""

    def __init__(self, pwm, min_us=600, max_us=2400, degrees=180):
        self.pwm = pwm
        self.degrees = degrees
        self.min_duty = self._us_to_duty(min_us)
        self.max_duty = self._us_to_duty(max_us)

    def _us_to_duty(self, us):
        return int(4095 * us / 20000)  # 50Hz 기준 20ms 주기

    def set_angle(self, servo_num, angle):
        """
        서보 번호: 1~8 (S1~S8)
        각도: 0~180도
        """
        if not 1 <= servo_num <= 8:
            raise ValueError("서보 번호는 1~8 사이여야 합니다.")
        angle = max(0, min(self.degrees, angle))
        duty_range = self.max_duty - self.min_duty
        duty = int(self.min_duty + duty_range * angle / self.degrees)
        channel = servo_num + 7  # S1~S8 → 채널 8~15
        self.pwm.set_duty(channel, duty)

    def release(self, servo_num):
        """서보 해제 (신호 제거)"""
        if not 1 <= servo_num <= 8:
            raise ValueError("서보 번호는 1~8 사이여야 합니다.")
        channel = servo_num + 7
        self.pwm.set_duty(channel, 0)

class PonySonar:
    """초음파 센서 거리 측정 클래스 (정수 cm 반환)"""

    def __init__(self, timeout_us=30000):
        self.timeout = timeout_us

    def measure(self, trig_pin, echo_pin):
        """
        trig_pin: 송신 핀 (예: pin13)
        echo_pin: 수신 핀 (예: pin14)
        반환값: 거리(cm, 정수형), 실패 시 -1
        """
        trig_pin.write_digital(0)
        utime.sleep_us(2)
        trig_pin.write_digital(1)
        utime.sleep_us(10)
        trig_pin.write_digital(0)

        try:
            duration = time_pulse_us(echo_pin, 1, self.timeout)
        except OSError:
            return -1  # 타임아웃

        distance = duration * 0.017  # 거리 계산 (cm)
        distance = int(distance)     # 소수점 제거 → 정수 cm

        if distance < 2 or distance > 400:
            return -1

        return distance


class PonyOLED:
    """128x64 OLED 디스플레이 제어 클래스 (SSD1306, I2C)"""

    def __init__(self, i2c, addr=0x3C):
        self.i2c = i2c
        self.addr = addr
        self.width = 128
        self.height = 64
        self.pages = self.height // 8
        self.buffer = bytearray(1 + self.width * self.pages)
        self.buffer[0] = 0x40  # 첫 바이트는 데이터 전송을 의미하는 0x40
        self.cursor_x = 0
        self.cursor_y = 0
        self.init()

    def send_cmd(self, cmd):
        self.i2c.write(self.addr, b'\x00' + bytes([cmd]))

    def init(self):
        cmds = [
            0xAE,       # Display OFF
            0xA4,       # Resume to RAM content display
            0xD5, 0xF0, # Set display clock divide ratio
            0xA8, 0x3F, # Set multiplex ratio
            0xD3, 0x00, # Display offset
            0x40,       # Set start line
            0x8D, 0x14, # Enable charge pump
            0x20, 0x00, # Memory addressing mode
            0x21, 0, 127, # Column address range
            0x22, 0, 7,   # Page address range
            0xA1,       # Segment remap
            0xC8,       # COM output scan direction
            0xDA, 0x12, # COM pins config
            0x81, 0xCF, # Contrast
            0xD9, 0xF1, # Precharge period
            0xDB, 0x40, # VCOMH deselect level
            0xA6,       # Normal display
            0xD6, 0x00, # Zoom OFF
            0xAF        # Display ON
        ]
        for cmd in cmds:
            self.send_cmd(cmd)
        self.clear()

    def clear(self):
        for i in range(1, len(self.buffer)):
            self.buffer[i] = 0
        self.cursor_x = 0
        self.cursor_y = 0
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
        page = y // 8
        shift = y % 8
        index = 1 + x + page * self.width
        if color:
            self.buffer[index] |= (1 << shift)
        else:
            self.buffer[index] &= ~(1 << shift)

    def draw_hline(self, x, y, length, color=1):
        for i in range(length):
            self.draw_pixel(x + i, y, color)

    def draw_vline(self, x, y, length, color=1):
        for i in range(length):
            self.draw_pixel(x, y + i, color)

    def draw_rect(self, x1, y1, x2, y2, color=1):
        self.draw_hline(x1, y1, x2 - x1 + 1, color)
        self.draw_hline(x1, y2, x2 - x1 + 1, color)
        self.draw_vline(x1, y1, y2 - y1 + 1, color)
        self.draw_vline(x2, y1, y2 - y1 + 1, color)

    def draw_char(self, x, y, char, color=1):
        if not 32 <= ord(char) < 127:
            char = '?'
        idx = (ord(char) - 32) * 5
        if idx + 4 >= len(FONT_5X7):
            return
        for col in range(5):
            line = FONT_5X7[idx + col]
            for row in range(8):
                pixel_on = (line >> row) & 0x01
                self.draw_pixel(x + col, y + row, pixel_on if color else not pixel_on)
        for row in range(8):
            self.draw_pixel(x + 5, y + row, 0 if color else 1)

    def draw_text(self, x, y, text, color=1):
        text = str(text)  # 문자열 외 숫자, 실수, 불리언 출력 대응
        for i, char in enumerate(text):
            self.draw_char(x + i * 6, y, char, color)

    def write_line(self, line_num, text, color=1):
        if 0 <= line_num < 8:
            self.draw_text(0, line_num * 8, str(text), color)

FONT_5X7 = bytes([
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
    0x03,0x04,0x78,0x04,0x03, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
    0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04,
    0x40,0x40,0x40,0x40,0x40, 0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78,
    0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20, 0x38,0x44,0x44,0x48,0x7F,
    0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x0C,0x52,0x52,0x52,0x3E,
    0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00,
    0x7F,0x10,0x28,0x44,0x00, 0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78,
    0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38, 0x7C,0x14,0x14,0x14,0x08,
    0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
    0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C,
    0x3C,0x40,0x30,0x40,0x3C, 0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C,
    0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00, 0x00,0x00,0x7F,0x00,0x00,
    0x00,0x41,0x36,0x08,0x00, 0x10,0x08,0x08,0x10,0x08, 0x78,0x46,0x41,0x46,0x78
])

class PonyColor:
    """TCS34725 색상 센서 제어 클래스"""
    def __init__(self, i2c, version=1, address=0x29):
        self.i2c = i2c
        self.address = address
        self.is_setup = False
        
        # 1: V1(검은색 모듈), 2: V2(보라색 모듈) 프로파일 매핑
        self.set_profile(version)

    def _write_byte(self, reg, value):
        self.i2c.write(self.address, bytes([0x80 | reg, value]))

    def _read_word(self, reg):
        self.i2c.write(self.address, bytes([0x80 | reg]))
        data = self.i2c.read(self.address, 2)
        return data[1] << 8 | data[0]

    def _read_raw_data(self):
        """[C, R, G, B] 반환"""
        self.setup()
        data = []
        for offset in range(4):
            data.append(self._read_word(0x14 + offset * 2))
        return data

    def setup(self):
        if self.is_setup:
            return
        self.is_setup = True
        self._write_byte(0x00, 0x03)  # ENABLE: PON | AEN
        self._write_byte(0x01, 0xC0)  # ATIME: 154ms 고정
        self._write_byte(0x0F, 0x03)  # CONTROL: 60x 고이득 레지스터 (0x80 | 0x0F = 0x8F 자동안정화)

    def set_profile(self, version):
        """하드웨어 사양별 실측 미니멈/맥시멈 데이터 테이블 갱신"""
        if version == 1 or version == "V1":
            self.r_min, self.g_min, self.b_min = 1883, 1866, 1371
            self.r_max, self.g_max, self.b_max = 18837, 20336, 14248
        else:
            self.r_min, self.g_min, self.b_min = 1513, 1006, 800
            self.r_max, self.g_max, self.b_max = 12590, 9557, 7167

    def light(self):
        """Clear 채널 밝기 값"""
        return self._read_raw_data()[0]

    def rgb(self):
        """실측 기반 2-Point 정규화 8비트 표준 RGB (0~255)"""
        data = self._read_raw_data()
        _, r_raw, g_raw, b_raw = data
        
        r_255 = max(0, min(255, int((r_raw - self.r_min) / max(1, (self.r_max - self.r_min)) * 255)))
        g_255 = max(0, min(255, int((g_raw - self.g_min) / max(1, (self.g_max - self.g_min)) * 255)))
        b_255 = max(0, min(255, int((b_raw - self.b_min) / max(1, (self.b_max - self.b_min)) * 255)))
        
        return [r_255, g_255, b_255]

    def is_color(self, target, threshold=40):
        """
        [최적화 알고리즘] 상대적 RGB 우세도 기반 고정밀 색상 판별
        target: "red", "green", "blue", "yellow"
        """
        r, g, b = self.rgb()
        
        if (r + g + b) < 60:
            return False

        if target == "red":
            return (r > g and r > b and r > g * 1.5)
        elif target == "yellow":
            return (r > b and g > b and r > g and r <= g * 1.5)
        elif target == "green":
            return (g > r and g >= b and g > r * 1.4)
        elif target == "blue":
            return (b > r and b > g)
        else:
            return False

    def is_in_range(self, min_r, max_r, min_g, max_g, min_b, max_b):
        r, g, b = self.rgb()
        return (min_r <= r <= max_r) and (min_g <= g <= max_g) and (min_b <= b <= max_b)
