import serial
import asyncio
from struct import *

async def main():
    task1 = asyncio.create_task(
        send_command0(0.5))

    task2 = asyncio.create_task(
        rev_command0(0.3))

    await task1
    await task2

async def send_command0(delay):
    global ser
    global packet
    while True:
        result = ser.write(packet)
        await asyncio.sleep(delay)  # 阻塞直到协程sleep(2)返回结果
        
async def rev_command0(delay):
    global ser
    while True:
        first_byte = ser.read().hex().upper()
        #print(first_byte)
        if '55' == first_byte:
            second_byte = ser.read().hex().upper()
            if 'AA' == second_byte:
                protocol_version = ser.read().hex().upper()
                if '00' == protocol_version:
                    command0 = ser.read().hex().upper()
                    if 'C0' == command0:
                        #print('bingo!')
                        #
                        #
                        length = int.from_bytes(pack('cc', ser.read(), ser.read()), byteorder='big', signed=False)
                        #print(length)
                        if 8 == length:
                            avg = int.from_bytes(pack('c', ser.read()), byteorder='big', signed=False)
                            light_ad = int.from_bytes(pack('c', ser.read()), byteorder='big', signed=False)
                            SUM0 = int.from_bytes(pack('cc', ser.read(), ser.read()), byteorder='big', signed=False)
                            SUM2 = int.from_bytes(pack('cc', ser.read(), ser.read()), byteorder='big', signed=False)
                            TH = int.from_bytes(pack('cc', ser.read(), ser.read()), byteorder='big', signed=False)
                            crc = int.from_bytes(pack('c', ser.read()), byteorder='big', signed=False)
                            site = {"avg": avg*16, 
                                    "light_ad": light_ad,
                                    "SUM0": SUM0*256,
                                    "SUM2": SUM2*256,
                                    "TH": TH*256,
                                    }
                            print("平均值：{avg}，光敏值：{light_ad}，SUM0值：{SUM0}，SUM1值：{SUM2}, TH值：{TH}".format(**site))
        await asyncio.sleep(delay)
        
ser=serial.Serial("com6", 9600, timeout=0.5)
print(ser.port)

ser.close()
ser.open()
# 55 AA 00 C0 00 00 BF

packet = bytearray()
packet.append(0x55)
packet.append(0xAA)
packet.append(0x00)
packet.append(0xC0)
packet.append(0x00)
packet.append(0x00)
packet.append(0xBF)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    ser.close()
    print('Bye-Bye!!!')


