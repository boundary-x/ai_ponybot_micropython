# ai_ponybot_v2.py
# Core Driver Module for AI Ponybot
# Features: PCA9685 PWM, Motor, Servo, Sonar, SSD1306 OLED, TCS34725 Color

from microbit import i2c, sleep
from machine import time_pulse_us
import math
import utime
import ustruct

# --- Register Constants ---
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

RESTART = 0x80
SLEEP = 0x10
ALLCALL = 0x01
OUTDRV = 0x04
RESET = 0x00

class _PWMController:
    # PCA9685 하드웨어 통신용 내부 드라이버 클래스
    def __init__(self, i2c, address=PCA9685_ADDRESS):
        self.address = address
        i2c.write(self.address, bytearray([MODE1, RESET]))
        self.set_all_pwm(0, 0)
        i2c.write(self.address, bytearray([MODE2, OUTDRV]))
        i2c.write(self.address, bytearray([MODE1, ALLCALL]))
        sleep(5)
        i2c.write(self.address, bytearray([MODE1]))
        mode1 = ustruct.unpack('<B', i2c.read(self.address, 1))[0]
        i2c.write(self.address, bytearray([MODE1, mode1 & ~SLEEP]))
        sleep(5)

    def set_pwm_frequency(self, freq_hz):
        prescaleval = 25000000.0 / 4096 / freq_hz - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        i2c.write(self.address, bytearray([MODE1]))
        oldmode = ustruct.unpack('<B', i2c.read(self.address, 1))[0]
        i2c.write(self.address, bytearray([MODE1, (oldmode & 0x7F) | 0x10]))
        i2c.write(self.address, bytearray([PRESCALE, prescale]))
        i2c.write(self.address, bytearray([MODE1, oldmode]))
        sleep(5)
        i2c.write(self.address, bytearray([MODE1, oldmode | RESTART]))

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
        if value == 0:
            self.set_pwm_duty_cycle(channel, 0, 4096)
        elif value == 4095:
            self.set_pwm_duty_cycle(channel, 4096, 0)
        else:
            self.set_pwm_duty_cycle(channel, 0, value)

class PonyMotor:
    # 포니봇 DC 모터 및 메카넘휠 이동 제어 클래스
    def __init__(self, i2c, motor_channels=None, pwm_freq=50):
        self.pwm = _PWMController(i2c)
        self.pwm.set_pwm_frequency(pwm_freq)
        self.motor_channels = motor_channels if motor_channels else {1: (7, 6), 2: (5, 4), 3: (2, 3), 4: (0, 1)}

    def move(self, motor_num, speed_percent):
        # 개별 모터 속도 설정 (motor_num: 1~4 / speed_percent: -100 ~ 100)
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
        # 기본 4륜 동기화 주행 (direction: 'forward', 'backward', 'left', 'right', 'stop')
        speed = max(0, min(100, speed))
        if direction == "forward":
            self.move(1, speed); self.move(2, speed); self.move(3, speed); self.move(4, speed)
        elif direction == "backward":
            self.move(1, -speed); self.move(2, -speed); self.move(3, -speed); self.move(4, -speed)
        elif direction == "left":
            self.move(1, speed); self.move(2, speed); self.move(3, -speed); self.move(4, -speed)
        elif direction == "right":
            self.move(1, -speed); self.move(2, -speed); self.move(3, speed); self.move(4, speed)
        elif direction == "stop":
            for i in range(1, 5): self.move(i, 0)
        else:
            raise ValueError("존재하지 않는 주행 방향 제어 문자열입니다.")

    def mecanum(self, direction_code, speed=0):
        # 메카넘휠 전방위 이동 제어 (direction_code: 1~9 키패드 레이아웃 매핑)
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
            for i in range(1, 5): self.move(i, 0)
        else:
            raise ValueError("direction_code는 1부터 9 사이의 정수여야 합니다.")

class PonyServo:
    # 로봇 관절용 서보모터 제어 클래스 (S1~S8 패드 구동 지원)
    def __init__(self, pwm, min_us=600, max_us=2400, degrees=180):
        self.pwm = pwm
        self.degrees = degrees
        self.min_duty = int(4095 * min_us / 20000)
        self.max_duty = int(4095 * max_us / 20000)

    def set_angle(self, servo_num, angle):
        # 지정 채널 서보 목표 각도 이동 (servo_num: 1~8 / angle: 0~180)
        if not 1 <= servo_num <= 8:
            raise ValueError("서보 번호는 1~8 사이여야 합니다.")
        angle = max(0, min(self.degrees, angle))
        duty_range = self.max_duty - self.min_duty
        duty = int(self.min_duty + duty_range * angle / self.degrees)
        self.pwm.set_duty(servo_num + 7, duty)

    def release(self, servo_num):
        # 서보모터 펄스 시그널 완전 차단 (토크 해제)
        if not 1 <= servo_num <= 8:
            raise ValueError("서보 번호는 1~8 사이여야 합니다.")
        self.pwm.set_duty(servo_num + 7, 0)

class PonySonar:
    # 초음파 활용 거리 탐지 센서 제어 클래스
    def __init__(self, timeout_us=30000):
        self.timeout = timeout_us

    def measure(self, trig_pin, echo_pin):
        # 실시간 초음파 왕복 타임아웃 측정 (정수 단위 cm 반환 / 센서 실패 시 -1)
        trig_pin.write_digital(0); utime.sleep_us(2)
        trig_pin.write_digital(1); utime.sleep_us(10)
        trig_pin.write_digital(0)
        try:
            duration = time_pulse_us(echo_pin, 1, self.timeout)
        except OSError:
            return -1
        distance = int(duration * 0.017)
        return distance if 2 <= distance <= 400 else -1

class PonyOLED:
    # SSD1306 그래픽 모듈 및 I2C 통신 제어 클래스
    def __init__(self, i2c, addr=0x3C):
        self.i2c = i2c
        self.addr = addr
        self.width = 128
        self.height = 64
        self.buffer = bytearray(1 + 128 * 8)
        self.buffer[0] = 0x40
        self.init()

    def send_cmd(self, cmd):
        self.i2c.write(self.addr, b'\x00' + bytes([cmd]))

    def init(self):
        cmds = [0xAE, 0xA4, 0xD5, 0xF0, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00, 0x21, 0, 127, 0x22, 0, 7, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA6, 0xD6, 0x00, 0xAF]
        for cmd in cmds: self.send_cmd(cmd)
        self.clear()

    def clear(self):
        # 로컬 그래픽 렌더링 프레임 버퍼 완전 비우기
        for i in range(1, len(self.buffer)): self.buffer[i] = 0
        self.show()

    def show(self):
        # 렌더링이 완료된 가상 프레임 버퍼 데이터를 물리 패널에 일괄 드로잉
        self.i2c.write(self.addr, self.buffer)

    def draw_pixel(self, x, y, color=1):
        if not (0 <= x < 128 and 0 <= y < 64): return
        index = 1 + x + (y // 8) * 128
        if color: self.buffer[index] |= (1 << (y % 8))
        else: self.buffer[index] &= ~(1 << (y % 8))

    def draw_char(self, x, y, char, color=1):
        idx = (max(32, min(ord(char), 126)) - 32) * 5
        for col in range(5):
            line = FONT_5X7[idx + col]
            for row in range(8):
                self.draw_pixel(x + col, y + row, (line >> row) & 0x01 if color else not ((line >> row) & 0x01))
        for row in range(8): self.draw_pixel(x + 5, y + row, 0 if color else 1)

    def draw_text(self, x, y, text, color=1):
        # 도트 단위 지정 좌표 문자열 출력 (x: 0~122 / y: 0~56)
        for i, char in enumerate(str(text)): self.draw_char(x + i * 6, y, char, color)

    def write_line(self, line_num, text, color=1):
        # 행 배치 단위 지정 문자열 가속 출력 (line_num: 0~7행 배치 단위)
        if 0 <= line_num < 8: self.draw_text(0, line_num * 8, text, color)

class PonyColor:
    # TCS34725 고정밀 광학 컬러센서 정규화 제어 클래스
    def __init__(self, i2c, version=1, address=0x29):
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

    def setup(self):
        if self.is_setup: return
        self.is_setup = True
        self._write_byte(0x00, 0x03)
        self._write_byte(0x01, 0xC0)
        self._write_byte(0x0F, 0x03)

    def set_profile(self, version):
        # 보유 사양 버전에 맞는 팩토리 실측 임계 데이터 매핑 (1=V1검은색 / 2=V2보라색 조명오염 보정)
        if version == 1 or version == "V1":
            self.r_min, self.g_min, self.b_min = 1883, 1866, 1371
            self.r_max, self.g_max, self.b_max = 18837, 20336, 14248
        else:
            self.r_min, self.g_min, self.b_min = 1513, 1006, 800
            self.r_max, self.g_max, self.b_max = 12590, 9557, 7167

    def light(self): return self._read_raw_data()[0]

    def rgb(self):
        # 조명 노이즈를 연산 제거한 표준 8비트 RGB (0~255) 스케일 반환
        _, r_raw, g_raw, b_raw = self._read_raw_data()
        r_255 = max(0, min(255, int((r_raw - self.r_min) / max(1, (self.r_max - self.r_min)) * 255)))
        g_255 = max(0, min(255, int((g_raw - self.g_min) / max(1, (self.g_max - self.g_min)) * 255)))
        b_255 = max(0, min(255, int((b_raw - self.b_min) / max(1, (self.b_max - self.b_min)) * 255)))
        return [r_255, g_255, b_255]

    def is_color(self, target, threshold=40):
        # 상대적 RGB 우세도 분별 수식을 통한 고정밀 색상 판정 ('red', 'green', 'blue', 'yellow')
        r, g, b = self.rgb()
        if (r + g + b) < 60: return False
        if target == "red": return (r > g and r > b and r > g * 1.5)
        elif target == "yellow": return (r > b and g > b and r > g and r <= g * 1.5)
        elif target == "green": return (g > r and g >= b and g > r * 1.4)
        elif target == "blue": return (b > r and b > g)
        return False

    def is_in_range(self, min_r, max_r, min_g, max_g, min_b, max_b):
        r, g, b = self.rgb()
        return (min_r <= r <= max_r) and (min_g <= g <= max_g) and (min_b <= b <= max_b)

# --- 5x7 Font ASCII Data Map (ASCII 32 ~ 126 Perfect Match) ---
FONT_HEX = "000000000000005f00000700070000147f147f14242a7f2a12231308646236495522500005030000001c2241000041221c0014083e081408083e080800503000000808080808006060000020100804023e5149453e00427f400042615149462141454b311814127f1027454545393c4a49493001710905033649494936064949291e003636000000563600000814224100141414141400412214080201510906324979413e7e1111117e7f494949363e414141227f4141221c7f494949417f090909013e4149497a7f0808087f00417f41002040413f017f081422417f404040407f020c027f7f0408107f3e4141413e7f090909063e4151215e7f09192946464949493101017f01013f4040403f1f2040201f7f2018207f631408146303047804036151494543007f41410002040810200041417f0004020102044040404040000102040020545454787f484444383844444420384444487f3854545418087e0901020c5252523e7f0804047800447d40002040443d007f1028440000417f40007c041804787c0804047838444444387c14141408081414187c7c080404084854545420043f4440203c4040207c1c2040201c3c4030403c44281028440c5050503c4464544c44000836410000007f0000004136080010080810087846414678"

# 로컬 고속 디코더 파이프라인 컴프리헨션 가동 (ubinascii 종속성 제거 완료)
FONT_5X7 = bytes(int(FONT_HEX[i:i+2], 16) for i in range(0, len(FONT_HEX), 2))
