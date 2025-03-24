import logging
from time import sleep
from socket import socket, AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_BROADCAST


logger = logging.getLogger('service_discovery')
logger.setLevel(logging.DEBUG)


# hack for better ip
def get_my_ip():
    s = socket(AF_INET, SOCK_DGRAM)
    s.settimeout(0)
    s.connect(("8.8.8.8", 80))
    my_ip = s.getsockname()[0]
    s.close()
    return my_ip


def main(port=7400):
    s = socket(AF_INET, SOCK_DGRAM)
    my_ip = get_my_ip()
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    data = f'{my_ip}'.encode('utf-8')
    logger.info(f'Broadcasting service discovery to port {port} with data {data}')
    while True:
        s.sendto(data, ('255.255.255.255', port))
        sleep(0.5)
        

if __name__ == '__main__':
    main()
