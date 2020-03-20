#!/usr/bin/python3

import sys
import serial
import socket
import queue
import threading
import time
from binascii import hexlify, unhexlify

UDP_IP = "quic.tech"
UDP_PORT = 4433
UDP_PAIR = (UDP_IP, UDP_PORT)


def run_thread(q, sock):
    while True:
        try:
            data, server_addr = sock.recvfrom(1500)
            print("Net << ({}) {}".format(len(data), data.hex()))
            q.put(data)
        except socket.timeout:
            print("Net << timeout")
            time.sleep(0.5)


def main():
    q = queue.SimpleQueue()
    port = sys.argv[1]
    print("Opening serial port {}".format(port))
    port = serial.Serial(port, 115200, timeout=0.5, write_timeout=10, rtscts=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    port.read()
    port.write("\rquiche SERIAL\r".encode("utf-8"))

    t = threading.Thread(target=run_thread, args=(q, sock), daemon=True)
    print("Starting thread...")
    t.start()
    print("Main loop...")
    while True:
        line = port.readline().decode("utf-8").strip()
        if line:
            print("Port << {!r}".format(line))
            if line.startswith("TX"):
                data_len = int(line[2:6], 16)
                data = unhexlify(line[6:])
                if data_len != len(data):
                    raise Exception("Bad data length {} != {}".format(
                        data_len, len(data)))
                print("Net >> {}".format(data.hex()))
                sock.sendto(data, UDP_PAIR)
            elif line == "?":
                try:
                    print("Queue pend...")
                    data = q.get(block=True, timeout=0.1)
                    print("Queue gave {} byte packet".format(len(data)))
                    msg = "{:04x}{}".format(len(data), data.hex())
                    print("Port << {!r}".format(msg))
                    port.write(msg.encode("ascii"))
                except queue.Empty:
                    pass


if __name__ == '__main__':
    main()
