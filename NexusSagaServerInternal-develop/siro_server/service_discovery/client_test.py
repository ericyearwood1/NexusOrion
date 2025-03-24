import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 7452))

while True:
    data, addr = s.recvfrom(1024)
    print(data, addr)
    time.sleep(1)
