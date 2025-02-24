import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import time
import board
import busio
import adafruit_ahtx0
import adafruit_ens160
import smbus

# I2C の設定
def setting_i2c():
    i2c = busio.I2C(board.SCL, board.SDA)
    return i2c

# AHT21・ENS160 センサーの初期化
def init_sensor():
    i2c = setting_i2c()
    sensor_aht21 = adafruit_ahtx0.AHTx0(i2c)  # 温湿度センサー
    sensor_ENS160 = adafruit_ens160.ENS160(i2c)  # CO₂センサー
    return sensor_aht21, sensor_ENS160

# 土壌センサーの初期化
def init_soil_sensor():
    i2c_bus = smbus.SMBus(1)  # I2Cバス1を使用
    i2c_addr = 0x44  # 土壌センサーのI2Cアドレス
    i2c_bus.write_byte_data(i2c_addr, 0x21, 0x30)  # 初期化コマンド
    time.sleep(0.5)  # 安定化待ち
    return i2c_bus, i2c_addr

# 土壌センサーのデータ取得
def read_soil_sensor(i2c_bus, i2c_addr):
    def tempChanger(msb, lsb):
        mlsb = ((msb << 8) | lsb)
        return (-45 + 175 * int(str(mlsb), 10) / (pow(2, 16) - 1))

    def humidChanger(msb, lsb):
        mlsb = ((msb << 8) | lsb)
        return (100 * int(str(mlsb), 10) / (pow(2, 16) - 1))

    i2c_bus.write_byte_data(i2c_addr, 0xE0, 0x00)  # 測定コマンド送信
    data = i2c_bus.read_i2c_block_data(i2c_addr, 0x00, 6)  # データ取得
    temperature = tempChanger(data[0], data[1])  # 土壌温度
    humidity = humidChanger(data[3], data[4])  # 土壌湿度
    return temperature, humidity

# ROS2 パブリッシャーノード
class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.sensor_aht21, self.sensor_ENS160 = init_sensor()  # AHT21 & ENS160 初期化
        self.i2c_bus, self.i2c_addr = init_soil_sensor()  # 土壌センサー初期化
        self.tmr = self.create_timer(1.0, self.timer_callback)  # 1秒ごとにデータ送信

    def timer_callback(self):
        msg = String()

        # AHT21（温湿度） & ENS160（CO₂） のデータ取得
        temperature = self.sensor_aht21.temperature
        humidity = self.sensor_aht21.relative_humidity
        co2 = self.sensor_ENS160.eCO2

        # 土壌センサーのデータ取得
        soil_temp, soil_humid = read_soil_sensor(self.i2c_bus, self.i2c_addr)

        # メッセージ生成
        msg.data = (
            f"Air Temp: {temperature:.2f} °C, Humidity: {humidity:.2f} %, CO₂: {co2} ppm, "
            f"Soil Temp: {soil_temp:.2f} °C, Soil Humidity: {soil_humid:.2f} %"
        )

        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
