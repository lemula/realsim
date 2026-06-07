import socket
import json
import time
import logging
import os
import re
from datetime import datetime
from pathlib import Path

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError:
        SMBus = None


# PCA9685寄存器地址
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09

# 通道定义
CHANNEL_STEERING = 0    # 转向舵机
CHANNEL_THROTTLE = 1    # 油门电调
CHANNEL_LIFT_DIR = 2    # 升降方向舵机
CHANNEL_LIFT_SPEED = 3  # 升降电调
CHANNEL_DIFF_LOCK = 4   # 差速锁舵机

# USB-I2C / PCA9685配置
DEFAULT_I2C_ADDRESS = 0x40
DEFAULT_PWM_FREQUENCY = 50
I2C_BUS_ENV = "TRUCK_I2C_BUS"
I2C_ADDRESS_ENV = "TRUCK_I2C_ADDRESS"
USB_I2C_KEYWORDS = (
    "usb",
    "ch341",
    "cp2112",
    "mcp2221",
    "ft232",
    "ftdi",
    "i2c-tiny",
    "tiny-usb",
)

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def clamp(value, lower, upper):
    return max(lower, min(upper, float(value)))


def read_text(path):
    try:
        return path.read_text(errors="ignore").strip()
    except OSError:
        return ""


def parse_i2c_bus(value):
    if isinstance(value, int):
        return value

    text = str(value).strip()
    match = re.fullmatch(r"(?:/dev/)?i2c-(\d+)", text)
    if match:
        return int(match.group(1))
    return int(text, 10)


def parse_i2c_address(value):
    return int(str(value).strip(), 0)


def available_i2c_buses():
    buses = []
    for dev in sorted(Path("/dev").glob("i2c-*")):
        try:
            buses.append(parse_i2c_bus(dev.name))
        except ValueError:
            continue
    return buses


def i2c_bus_description(bus_num):
    base = Path(f"/sys/class/i2c-dev/i2c-{bus_num}")
    parts = [
        read_text(base / "name"),
        read_text(base / "device" / "name"),
        read_text(base / "device" / "modalias"),
        read_text(base / "device" / "uevent"),
    ]
    return " ".join(part for part in parts if part)


def detect_usb_i2c_bus():
    buses = available_i2c_buses()
    usb_buses = []

    for bus_num in buses:
        description = i2c_bus_description(bus_num).lower()
        if any(keyword in description for keyword in USB_I2C_KEYWORDS):
            usb_buses.append(bus_num)

    if usb_buses:
        if len(usb_buses) > 1:
            logger.warning(
                "检测到多个USB-I2C总线: %s，默认使用 i2c-%d；可用 %s 手动指定",
                usb_buses,
                usb_buses[0],
                I2C_BUS_ENV,
            )
        return usb_buses[0]

    pca_buses = detect_pca9685_buses(buses, resolve_i2c_address())
    if pca_buses:
        if len(pca_buses) > 1:
            logger.warning(
                "多个I2C总线在地址0x%02X有响应: %s，默认使用 i2c-%d；可用 %s 手动指定",
                resolve_i2c_address(),
                pca_buses,
                pca_buses[0],
                I2C_BUS_ENV,
            )
        else:
            logger.info("在 i2c-%d 地址0x%02X探测到PCA9685", pca_buses[0], resolve_i2c_address())
        return pca_buses[0]

    available = ", ".join(f"i2c-{bus}" for bus in buses) or "无"
    raise RuntimeError(
        "没有自动识别到USB-I2C适配器。"
        f"当前可见I2C总线: {available}。"
        f"请先运行 `i2cdetect -l` 找到USB-I2C对应的总线号，"
        f"然后设置 `export {I2C_BUS_ENV}=总线号`，例如 `export {I2C_BUS_ENV}=1`。"
        "如果设备只显示为 /dev/ttyUSB0，则它当前是USB串口模式，不能被smbus当作I2C总线使用。"
    )


def detect_pca9685_buses(buses, address):
    """在可见I2C总线上探测PCA9685默认地址，作为USB描述识别失败后的兜底"""
    if SMBus is None:
        return []

    found = []
    for bus_num in buses:
        bus = None
        try:
            bus = SMBus(bus_num)
            bus.read_byte_data(address, PCA9685_MODE1)
            found.append(bus_num)
        except OSError:
            pass
        finally:
            if bus is not None:
                close = getattr(bus, "close", None)
                if close is not None:
                    close()

    return found


def resolve_i2c_bus(bus_num=None):
    if bus_num is not None:
        return parse_i2c_bus(bus_num)

    env_bus = os.getenv(I2C_BUS_ENV)
    if env_bus:
        return parse_i2c_bus(env_bus)

    return detect_usb_i2c_bus()


def resolve_i2c_address(address=None):
    if address is not None:
        return parse_i2c_address(address)

    env_address = os.getenv(I2C_ADDRESS_ENV)
    if env_address:
        return parse_i2c_address(env_address)

    return DEFAULT_I2C_ADDRESS


class PCA9685:
    """通过USB-I2C适配器访问PCA9685，由PCA9685输出PWM波形"""

    def __init__(self, bus_num=None, address=None, frequency=DEFAULT_PWM_FREQUENCY):
        if SMBus is None:
            raise RuntimeError("缺少 smbus/smbus2，请先安装: python3 -m pip install smbus2")

        self.bus_num = resolve_i2c_bus(bus_num)
        self.address = resolve_i2c_address(address)
        self.frequency = int(frequency)
        self.pwm_range = 4096
        self.bus = SMBus(self.bus_num)

        description = i2c_bus_description(self.bus_num) or "未知适配器"
        logger.info(
            "已连接PCA9685: /dev/i2c-%d (%s), address=0x%02X, frequency=%dHz",
            self.bus_num,
            description,
            self.address,
            self.frequency,
        )

        self._reset()
        self._set_pwm_frequency(self.frequency)

    def _reset(self):
        self.bus.write_byte_data(self.address, PCA9685_MODE1, 0x00)
        self.bus.write_byte_data(self.address, PCA9685_MODE2, 0x04)
        time.sleep(0.005)

    def _set_pwm_frequency(self, frequency):
        prescale_value = 25000000.0 / self.pwm_range / float(frequency) - 1.0
        prescale = int(round(prescale_value))

        old_mode = self.bus.read_byte_data(self.address, PCA9685_MODE1)
        new_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, PCA9685_MODE1, new_mode)
        self.bus.write_byte_data(self.address, PCA9685_PRESCALE, prescale)
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode | 0x80)

    def pulse_us_to_ticks(self, pulse_us):
        ticks = int(round(float(pulse_us) * self.frequency * self.pwm_range / 1000000.0))
        return int(clamp(ticks, 0, self.pwm_range - 1))

    def set_pwm(self, channel, on, off):
        channel = int(channel)
        self.bus.write_byte_data(self.address, PCA9685_LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, PCA9685_LED0_ON_H + 4 * channel, on >> 8)
        self.bus.write_byte_data(self.address, PCA9685_LED0_OFF_L + 4 * channel, off & 0xFF)
        self.bus.write_byte_data(self.address, PCA9685_LED0_OFF_H + 4 * channel, off >> 8)

    def set_pwm_us(self, channel, pulse_us):
        ticks = self.pulse_us_to_ticks(pulse_us)
        self.set_pwm(channel, 0, ticks)
        logger.debug("PCA9685通道 %d 脉宽 %.1fus -> ticks=%d", channel, pulse_us, ticks)

    def close(self):
        close = getattr(self.bus, "close", None)
        if close is not None:
            close()


class ServoController:
    """舵机和电调控制器，通过USB-I2C写PCA9685寄存器"""

    def __init__(self):
        self.pca = PCA9685()

        # 舵机参数，单位: 微秒
        self.servo_min_us = 500
        self.servo_max_us = 2500

        # 电调参数，单位: 微秒
        self.esc_min_us = 500
        self.esc_mid_us = 1500
        self.esc_max_us = 2500

        self._initialize_escs()

    def _initialize_escs(self):
        """初始化电调和舵机"""
        logger.info("开始初始化电调和舵机...")

        self.set_motor_speed(CHANNEL_THROTTLE, -100)
        time.sleep(1)
        self.set_motor_speed(CHANNEL_THROTTLE, 100)
        time.sleep(0.5)
        self.set_motor_speed(CHANNEL_THROTTLE, 0)
        time.sleep(0.5)

        self.set_motor_speed(CHANNEL_LIFT_SPEED, -100)
        time.sleep(1)
        self.set_motor_speed(CHANNEL_LIFT_SPEED, 100)
        time.sleep(0.5)
        self.set_motor_speed(CHANNEL_LIFT_SPEED, 0)
        time.sleep(0.5)

        self.set_servo_angle(CHANNEL_STEERING, 90)
        self.set_servo_angle(CHANNEL_LIFT_DIR, 90)
        self.set_servo_angle(CHANNEL_DIFF_LOCK, 90)
        logger.info("电调和舵机初始化完成")

    def angle_to_pulse_us(self, angle):
        angle = clamp(angle, 0, 180)
        return int(round(self.servo_min_us + (self.servo_max_us - self.servo_min_us) * angle / 180))

    def speed_to_pulse_us(self, speed):
        speed = clamp(speed, -100, 100)
        if speed >= 0:
            return int(round(self.esc_mid_us + (self.esc_max_us - self.esc_mid_us) * speed / 100))
        return int(round(self.esc_mid_us + (self.esc_mid_us - self.esc_min_us) * speed / 100))

    def set_servo_angle(self, channel, angle):
        pulse_us = self.angle_to_pulse_us(angle)
        self.pca.set_pwm_us(channel, pulse_us)
        logger.debug("舵机通道 %d 角度 %.2f -> %dus", channel, angle, pulse_us)

    def set_motor_speed(self, channel, speed):
        pulse_us = self.speed_to_pulse_us(speed)
        self.pca.set_pwm_us(channel, pulse_us)
        logger.debug("电机通道 %d 速度 %.2f%% -> %dus", channel, speed, pulse_us)

    def close(self):
        self.pca.close()


class JetsonReceiver:
    def __init__(self, host='192.168.0.85', port=8888):
        self.host = host
        self.port = port
        self.last_print_time = 0  # 记录上次打印时间
        self.controller = ServoController()  # 初始化控制器
        self.setup_socket()
        
    def setup_socket(self):
        """创建并配置UDP socket"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.server_socket.bind((self.host, self.port))
            logger.info(f"服务已启动，监听 {self.host}:{self.port}")
        except Exception as e:
            logger.error(f"Socket绑定失败: {e}")
            raise
    
    def handle_handshake(self, address):
        """处理客户端握手请求"""
        response = json.dumps({"status": "ready", "message": "connected"})
        self.server_socket.sendto(response.encode('utf-8'), address)
        logger.info(f"已与客户端 {address} 建立连接")

    def normalize_steering(self, steering):
        """兼容方向盘相对角(-45~45)和舵机绝对角(0~180)两种输入"""
        steering = float(steering)
        if -45 <= steering <= 45:
            return 90 + steering
        return steering

    def parse_control_data(self, data):
        """兼容旧方向盘包和键盘遥控包"""
        if "steer_angle" in data or "speed_percent" in data:
            steer_angle = float(data.get("steer_angle", 0))
            speed_percent = float(data.get("speed_percent", 0))
            if data.get("stop", False):
                speed_percent = 0

            return {
                "steering": 90 + clamp(steer_angle, -45, 45),
                "throttle": max(0, speed_percent),
                "brake": max(0, -speed_percent),
                "up": int(data.get("up", 0)),
                "down": int(data.get("down", 0)),
                "lock": int(data.get("lock", 0)),
            }

        required_fields = ("steering", "throttle", "brake", "up", "down", "lock")
        missing_fields = [field for field in required_fields if field not in data]
        if missing_fields:
            raise KeyError(", ".join(missing_fields))

        return {
            "steering": self.normalize_steering(data["steering"]),
            "throttle": float(data["throttle"]),
            "brake": float(data["brake"]),
            "up": int(data["up"]),
            "down": int(data["down"]),
            "lock": int(data["lock"]),
        }
    
    def process_data(self, data, address):
        """处理接收到的数据并控制设备"""
        try:
            control = self.parse_control_data(data)
        except KeyError as e:
            logger.error(f"数据缺少必要字段: {e}")
            return

        steering = control["steering"]
        throttle = control["throttle"]
        brake = control["brake"]
        up = control["up"]
        down = control["down"]
        lock = control["lock"]
        
        # 执行控制逻辑
        # 1. 转向控制
        self.controller.set_servo_angle(CHANNEL_STEERING, steering)
        
        # 2. 油门和刹车控制 
        if brake > 0:
            self.controller.set_motor_speed(CHANNEL_THROTTLE, -brake)  
        else:
            self.controller.set_motor_speed(CHANNEL_THROTTLE, throttle)
        
        # 3. 升降控制
        if up == 1:
            self.controller.set_servo_angle(CHANNEL_LIFT_DIR, 60)  # 上升方向
            self.controller.set_motor_speed(CHANNEL_LIFT_SPEED, 70)  # 上升速度
        elif down == 1:
            self.controller.set_servo_angle(CHANNEL_LIFT_DIR, 150)  # 下降方向
            self.controller.set_motor_speed(CHANNEL_LIFT_SPEED, 70)  # 下降速度
        else:
            self.controller.set_servo_angle(CHANNEL_LIFT_DIR, 90)  # 停止方向
            self.controller.set_motor_speed(CHANNEL_LIFT_SPEED, 0)  # 停止升降
        
        # 4. 差速锁控制
        if lock == 1:
            self.controller.set_servo_angle(CHANNEL_DIFF_LOCK, 30)  # 锁定状态
        else:
            self.controller.set_servo_angle(CHANNEL_DIFF_LOCK, 90)  # 解锁状态
        
        # 每隔1秒打印一次数据
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            try:
                print("\n" + "="*40)
                print(f"[{datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S')}]")
                print(f"来自客户端: {address}")
                print(f"方向盘角度: {steering}°")
                print(f"油门开度: {throttle}%")
                print(f"刹车开度: {brake}%")
                print(f"上升按钮: {'按下' if up == 1 else '未按'}")
                print(f"下降按钮: {'按下' if down == 1 else '未按'}")
                print(f"差速锁: {'锁定' if lock == 1 else '解锁'}")
                print("="*40)
                
                self.last_print_time = current_time
            except Exception as e:
                logger.error(f"打印数据时出错: {e}")
        
    def run(self):
        """主循环：接收并处理数据"""
        


        try:
            while True:
                # 接收数据
                data_bytes, address = self.server_socket.recvfrom(1024)
                
                try:
                    # 解析JSON数据
                    data = json.loads(data_bytes.decode('utf-8'))
                    
                    # 处理握手请求
                    if data.get("type") == "handshake":
                        self.handle_handshake(address)
                    else:
                        # 处理控制数据
                        self.process_data(data, address)
                        
                except json.JSONDecodeError:
                    logger.error(f"无效的JSON数据: {data_bytes}")
                except Exception as e:
                    logger.error(f"数据处理错误: {e}")
                    
        except KeyboardInterrupt:
            logger.info("\n程序已停止")
        finally:
            # 关闭时复位所有设备
            self.controller.set_servo_angle(CHANNEL_STEERING, 90)
            self.controller.set_servo_angle(CHANNEL_LIFT_DIR, 90)
            self.controller.set_servo_angle(CHANNEL_DIFF_LOCK, 90)
            self.controller.set_motor_speed(CHANNEL_THROTTLE, 0)
            self.controller.set_motor_speed(CHANNEL_LIFT_SPEED, 0)
            self.controller.close()
            
            self.server_socket.close()
            logger.info("Socket已关闭，所有设备已复位")


if __name__ == "__main__":
    receiver = JetsonReceiver(host='192.168.2.32', port=8888)
    #receiver = JetsonReceiver(host='192.168.0.85', port=8888)
    receiver.run()
