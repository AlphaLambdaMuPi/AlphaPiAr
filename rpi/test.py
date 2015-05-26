import serial
import json

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)

while True:
    res = ser.readline().decode().strip('\n').strip('\r')
    try:
        res = json.loads(res)
    except:
        print('JSON decode failed : {}'.format(res))
        continue
    print('ok', res)
    ser.write('alpha beta\n'.encode())

