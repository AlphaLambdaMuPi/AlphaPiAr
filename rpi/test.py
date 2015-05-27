import serial
import json
import random
import time
import sys
from multiprocessing import Process, Lock, Value, Array

def serial_reading(arr):
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)
    cnt = 0
    while True:
        res = ser.readline()
        res = res.decode().strip('\n').strip('\r')
        try:
            res = json.loads(res)
        except KeyboardInterrupt:
            break
        except:
            # print('JSON decode failed : {}'.format(res))
            continue

        if cnt % 50 == 0:
            print(res)

        if 'accel' not in res:
            continue

        cnt += 1

        motors = list(arr)
        j = json.dumps({'motor': motors, 'time': time.time()}, separators=(',',':')) + '\n'
        ser.write(j.encode())
        ser.flush()

def reading_from_stdin(arr):
    while True:
        inp = input()
        mt = int(inp)
        for i in range(4):
            arr[i] = mt
        print("-- Set Motor", mt)

if __name__ == "__main__":
    arr = Array('i', [0]*4)
    p = Process(target=serial_reading, args=(arr,))
    p.start()
    try:
        reading_from_stdin(arr)
    except KeyboardInterrupt:
        print("stoping..., please press Ctrl-C again")
        p.join()
    finally:
        print('exit.')




