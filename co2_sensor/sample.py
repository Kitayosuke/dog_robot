import time
import board
import busio
import adafruit_ahtx0
import adafruit_ens160

# I2Cの設定
def setting_i2c():
    i2c = busio.I2C(board.SCL, board.SDA)
    return i2c

# センサの初期化
def init_sensor():
    i2c = setting_i2c()
    sensor_aht21 = adafruit_ahtx0.AHTx0(i2c)
    sensor_ENS160 = adafruit_ens160.ENS160(i2c)
    return sensor_aht21, sensor_ENS160

def main():
    try:
        sensor_aht21, sensor_ENS160 = init_sensor()
        while True:
            # 温度と湿度の取得
            temperature = sensor_aht21.temperature  # 温度（Celsius）
            humidity = sensor_aht21.relative_humidity  # 湿度（%）
            co2 = sensor_ENS160.eCO2  # 二酸化炭素 (ppm)

            # 取得したデータの表示
            print(f"Temperature: {temperature:.2f} °C, Humidity: {humidity:.2f} %, CO₂: {co2} ppm")

            # 1秒待機
            time.sleep(1)

    except Exception as e:
        print(f"エラーが発生しました: {e}")

if __name__ == "__main__":
    main()