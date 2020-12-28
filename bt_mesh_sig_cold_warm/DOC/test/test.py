import serial
import asyncio
from struct import *
import csv
import time

async def main_co():
    global packet1
    global ser
    
    task1 = asyncio.create_task(
        send_command0(0.5))

    task2 = asyncio.create_task(
        rev_command0(0.01))

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
                                    "diff": SUM2*256 - SUM0*256,
                                    }
                            with open('result.csv', 'a', newline='') as csvfile:
                                fieldnames = ['avg', 'light_ad', 'SUM0', 'SUM2', 'TH', 'diff']
                                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                                writer.writerow(site)
                            print("平均值：{avg}，光敏值：{light_ad}，SUM0值：{SUM0}，SUM1值：{SUM2}，TH值：{TH}，差值：{diff}".format(**site))
        await asyncio.sleep(delay)

def main():
    print('this message is from main function')
    global ser
    global packet
    global packet1
    global packet2
    
    ser=serial.Serial("com6", 9600, timeout=0.5)
    print(ser.port)

    ser.close()
    ser.open()
    # 55 AA 00 C0 00 00 BF      //trigger the command0
    packet = bytearray()
    packet.append(0x55)
    packet.append(0xAA)
    packet.append(0x00)
    packet.append(0xC0)
    packet.append(0x00)
    packet.append(0x00)
    packet.append(0xBF)

    # 31 80 25 00 00            //open the usb-uart function of the hclink
    packet1 = bytearray()
    packet1.append(0x31)
    packet1.append(0x80)
    packet1.append(0x25)
    packet1.append(0x00)
    packet1.append(0x00)

    # 32 53 54 4F 50            //close the usb-uart function of the hclink
    packet2 = bytearray()
    packet2.append(0x32)
    packet2.append(0x53)
    packet2.append(0x54)
    packet2.append(0x4F)
    packet2.append(0x50)
    
    ser.set_buffer_size(rx_size = 1024, tx_size = 1024)
    ser.write(packet1)
    ser.flush()    

    try:
        asyncio.run(main_co())
    except KeyboardInterrupt:
        ser.write(packet2)
        ser.close()
        print('Bye-Bye!!!')

if __name__ == '__main__':
    main()
