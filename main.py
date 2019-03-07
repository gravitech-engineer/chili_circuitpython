import busio
import time
import board
import HTS221

i2c = busio.I2C(board.SCL,board.SDA)
sensor = HTS221.HTS221_I2C(i2c)

while True:
    humid = sensor.readHumi()
    temp = sensor.readTemp()
    #print(humid)
    #print(temp)
    print("Humidity : {0:5.2f} ,Temperature : {1:5.2f}".format(humid,temp))
    time.sleep(1)