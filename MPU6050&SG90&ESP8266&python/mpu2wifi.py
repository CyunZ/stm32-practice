from socket import *
import serial #导入模块
import math
import struct
import time

portx="COM4"
bps=115200
timex=5
ser=serial.Serial(portx,bps,timeout=timex)
print("串口详情参数：", ser)
print(ser.port)#获取到当前打开的串口名
print(ser.baudrate)#获取波特率



client = socket(AF_INET, SOCK_STREAM)
host = "192.168.1.170"
port = 8080
client.connect((host, port))
while True:
    time.sleep(0.5)
    ba = bytearray()
    if ser.in_waiting:
        byteArr=ser.read(ser.in_waiting)
        if len(byteArr)>=6:
            if byteArr[0] == 0x99 and byteArr[1] == 0x11 :
                ba.append(byteArr[2])
                ba.append(byteArr[3])
                ba.append(byteArr[4])
                ba.append(byteArr[5])
                ptich = struct.unpack("!f",ba)[0]

                ptich_int = int(ptich) + 90 
                wifidata = str(ptich_int) + "OK" # OK是自定义的帧尾
                print(wifidata)
                client.send(wifidata.encode("utf-8"))
    
client.close()
ser.close()#关闭串口