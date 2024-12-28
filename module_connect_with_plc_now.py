from socket import*
import time

from socket import socket, AF_INET, SOCK_STREAM

class ConnectPLC:
    def __init__(self, ip_address="192.168.3.39", port=2500):
        self.ip_address = ip_address
        self.port = port
        self.timeout_sec=0.5

    def send_M_bit(self, address):
        # this is frame to read m100 from PLC to python
        frame = {
            'header': [0x50, 0, 0, 0xff, 0xff, 3, 0],
            'length': [0x0c, 0],
            'timer': [0x20, 0],
            'command': [1, 4],
            'sub_command': [1, 0],
            'start_addr': [address, 0, 0],
            'device': [0x90],
            'points': [8, 0],
            'b_data': [True]
        }
        dummy = []
        for field in list(frame.values()):
            dummy += field
        dummy[7] = len(dummy) - 9
        return dummy

    def send_D_bit(self, address, value):
        # Sử dụng 2 byte để biểu diễn giá trị lớn hơn 256 nhưng nhỏ hơn 65535
        lower_byte = value & 0xFF  # Lấy 8 bit thấp nhất (byte 1)
        higher_byte = (value >> 8) & 0xFF  # Lấy 8 bit cao (byte 2)

        # Khung dữ liệu gửi đi
        frame = {
            'header': [0x50, 0, 0, 0xff, 0xff, 3, 0],
            'length': [0x10, 0],  # Độ dài sẽ cập nhật sau
            'timer': [0x20, 0],
            'command': [1, 0x14],
            'sub_command': [0, 0],
            'start_addr': [address & 0xFF, (address >> 8) & 0xFF, (address >> 16) & 0xFF],  # Địa chỉ 3 byte
            'device': [0xa8],
            'points': [1, 0],  # Gửi 1 từ (2 byte)
            'w_data': [lower_byte, higher_byte]  # 2 byte của giá trị cần gửi
        }

        # Kết hợp các phần của frame thành một khung dữ liệu hoàn chỉnh
        dummy = []
        for field in list(frame.values()):
            dummy += field

        # Cập nhật độ dài khung dữ liệu
        frame_length = len(dummy) - 9
        dummy[7] = frame_length & 0xFF
        dummy[8] = (frame_length >> 8) & 0xFF

        return dummy
    def connect_and_send(self):
        for i in range(20):
            try:
                s = socket(AF_INET, SOCK_STREAM)
                s.settimeout(self.timeout_sec)
                s.connect((self.ip_address, self.port))
                # s.close()
                print("PLC connection successful")
                break
            except Exception as e:
                print(e)
                time.sleep(1)
            finally:
                s.close()

        

# # Truyền thông dữ liệu như mong muốn 
# plc = ConnectPLC()
# # Kết nối và gửi dữ liệu
# plc.connect_and_send()
