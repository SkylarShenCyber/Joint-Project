#  -*- coding: utf-8 -*
import serial
import time
import struct
import avoid_listener

# 初始化串口
serialHandle = serial.Serial(
    "/dev/ttyUSB0", 115200, bytesize=8, stopbits=1, parity='N', timeout=0.5)

# 校验和


def checksum(buf):
    sum = ~(buf) % 256
    return sum

# 小车底盘控制


def car_control(x_move,y_move,x_speed,y_speed,angle,angle_speed):
    # 直行速度设置
    upper_eight = hex(y_speed / 256)
    lower_eight = hex(y_speed % 256)
    buf1 = 0x04 + 0x21 + upper_eight + lower_eight
    package6 = struct.pack("BBBBBBBBB", 0xfc, 0xfd, 0x04, 0x21,
                           upper_eight, lower_eight, checksum(buf1), 0xfe, 0xff)
    serialHandle.write(package6)
    time.sleep(0.5)

    # 直行指定距离
    y_distance = hex(y_move)
    buf2 = 0x03 + 0x28 + y_distance
    package7 = struct.pack("BBBBBBBB", 0xfc, 0xfd, 0x03,
                           0x28, y_distance, checksum(buf2), 0xfe, 0xff)
    serialHandle.write(package7)
    time.sleep(0.5)

    # 平移指定距离+速度
    x_distance = hex(abs(x_move))
    x_speed = hex(x_speed)
    if (x_move >= 0):
        param1 = 0x00
        buf3 = 0x05 + 0x24 + 0x00 + x_distance+x_speed
    elif (x_move < 0):
        param1 = 0x01
        buf3 = 0x05 + 0x24 + 0x01 + x_distance+x_speed
    package8 = struct.pack("BBBBBBBBB", 0xfc, 0xfd, 0x05, 0x24,
                           param1, x_speed, x_distance, checksum(buf3), 0xfe, 0xff)
    serialHandle.write(package8)
    time.sleep(0.5)

    # 旋转指定角度
    angel = hex(abs(angel))
    if (angel >= 0):
        param2 = 0x00
        buf4 = 0x04 + 0x23 + 0x00 + angel
    elif (angel <= 0):
        param2 = 0x01
        buf4 = 0x04 + 0x23 + 0x01 + angel
    package9 = struct.pack("BBBBBBBBB", 0xfc, 0xfd, 0x04,
                           0x23, param2, angel, checksum(buf4), 0xfe, 0xff)
    serialHandle.write(package9)
    time.sleep(0.5)


if __name__ == "__main__":

    # 机器人初始化
    if (avoid_listener.command == 1):
        package1 = struct.pack("BBBBBBBB", 0xfc, 0xfd,
                               0x03, 0x01, 0x02, 0xf9, 0xfe, 0xff)
        serialHandle.write(package1)
        time.sleep(0.5)
        package2 = struct.pack("BBBBBBBB", 0xfc, 0xfd,
                               0x03, 0x01, 0x03, 0xf8, 0xfe, 0xff)
        serialHandle.write(package2)
        time.sleep(0.5)

    # 底盘初始化
    if(avoid_listener.command_under_start == 1):
        package_under_start = struct.pack(
            "BBBBBBB", 0xfc, 0xfd, 0x02, 0x33, checksum(0x02+0x31))
        serialHandle.write(package_under_start)
        time.sleep(0.5)

    car_control(avoid_listener.x_move,avoid_listener.y_move,avoid_listener.x_speed,
    avoid_listener.y_speed,avoid_listener.angle,avoid_listener.angle_speed)  # 先
    
    # stop_command
    package_stop = struct.pack(
        "BBBBBBB", 0xfc, 0xfd, 0x02, 0x32, checksum(0x02 + 0x32), 0xfe, 0xff)
    serialHandle.write(package_stop)
    time.sleep(0.5)

    
    
