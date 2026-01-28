import socket
import struct
import time 
class LidraLiveRawDataFront:
    def __init__(self, ip='195.37.48.222', port=2111):
        self.ip = ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(2.0)
        self.socket.bind((self.ip, self.port))
        self.running = False
        self.buffer_size = 4096
        self.data = []
        self.running = False
    def start(self):
        self.running = True
        print(f"Listening for data on {self.ip}:{self.port}")
        while self.running:
            try:
                packet, addr = self.socket.recvfrom(self.buffer_size)
                if packet:
                    self.process_packet(packet)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving data: {e}")
    def stop(self):
        self.running = False
        self.socket.close()
        print("Stopped listening for data.")