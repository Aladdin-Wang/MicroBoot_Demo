import PikaStdLib
import time
import ym
#字节
#data = bytes([0x01, 0x02, 0x03, 0x04])
#字符串
data = b'boot\r\n'
#uart = serial.uart(115200)
#uart.write_bytes(data,len(data))

rs485 = serial.rs485(115200)
rs485.write_bytes(data,len(data))
time.sleep_ms(200)
rs485.write_bytes(data,len(data))

ymodem = ym.ymodem("485",115200)
ymodem.send("app.bin")

