import serial
import json
import random
import time
import sys
import struct
from multiprocessing import Process, Lock, Value, Array

def decode_sensor(b):
    res = []
    ret = {}
    for i in range(11):
        res.append(struct.unpack('f', b[4*i:4*(i+1)]))
    
    ret['accel'] = res[0:3]
    ret['gyro'] = res[3:6]
    ret['mag'] = res[6:9]
    ret['temperature'] = res[9]
    ret['pressure'] = res[10]
    ret['time'] = time.time()

    return ret    

def serial_reading(arr):
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)
    cnt = 0

    def setup():
        while True:
            ser.write('S'.encode())
            try:
                ret = ser.read(1)
                ret = ret.decode()
                if ret == 's':
                    break
                elif ret == '':
                    raise ValueError
                else:
                    print('Communication with Arduino failed !! (received {})'.format(ret))
            except:
                print('Waiting for Arduino...')
                pass

        print('Connected with Arduino.')

    def read_sensor():
        b = 'R'.encode()
        ser.write(b)
        ser.flush()

        res = ser.read(4*11) # ax, ay, az, gx, gy, gz, mx, my, mz, temp, pres
        return decode_sensor(res)

    def write_motor():
        b = 'M'.encode()
        ser.write(b)
        motors = list(arr)
        st = struct.pack('hhhh', *motors)
        ser.write(st)
        ser.flush()
        res = ser.read(1)
        res = res.decode()
        if res != 'm':
            raise ValueError('Write motor to Arduino failed !!!')


    setup()
    while True:
        t = time.time()
        cnt += 1

        zz = read_sensor()
        print(zz)

        write_motor()

        print(time.time() - t)
        continue
        try:
            res = json.loads(res)
        except KeyboardInterrupt:
            break
        except:
            # print('JSON decode failed : {}'.format(res))
            continue

        # if 'accel' not in res:
            # continue

        print(json.dumps(res))

        cnt += 1

        j = json.dumps({'motor': motors, 'time': time.time()}, separators=(',',':')) + '\n'
        ser.write(j.encode())
        ser.flush()

def reading_from_stdin(arr):
    while True:
        inp = input().split()
        mt = [int(x) for x in inp]
        for i in range(4):
            arr[i] = mt[i]
        # print("-- Set Motor", mt)

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

