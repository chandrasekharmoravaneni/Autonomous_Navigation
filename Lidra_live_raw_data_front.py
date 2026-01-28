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
    def process_packet(self, packet):
        header_format = '>HBBI'
        header_size = struct.calcsize(header_format)
        if len(packet) < header_size:
            print("Received packet is too small to contain header.")
            return
        header = struct.unpack(header_format, packet[:header_size])
        packet_type = header[1]
        if packet_type == 0x01:
            self.parse_lidar_data(packet[header_size:])
        else:
            print(f"Unknown packet type: {packet_type}")