#pyuic5 -x gui.ui -o gui.py
#pyuic5 -x gui_main_adjust.ui -o gui_main.py
# pyrcc5 -o resource_rc.py resource.qrc
# "C:\Users\pronics\AppData\Local\Programs\Python\Python310\python.exe"
#  C:\Users\pronics\AppData\Local\Programs\Python\Python312\python.exe
# "main.py"
# "module_connect_with_plc_now.py"

# "C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\main_verAI_ver2.py"
import sys
import cv2
import os
import numpy as np
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QEvent , QDateTime, QTimer , QMutex
from PyQt5.QtGui import QImage, QPixmap, QTransform, QPainter, QPainterPath, QColor
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QPushButton, QFileDialog, QLabel

from gui import Ui_MainWindow
from pypylon import pylon
from multiprocessing import Process, Value, Queue
import time
import threading
import pyscreenshot as ImageGrab
import serial
from datetime import datetime
import imutils
import shutil
import natsort
from Robot_Guide_Template_Binh_Tan_to_modify import modlue1
import multiprocessing
# import resource_rc
from socket import*
from socket import socket, AF_INET, SOCK_STREAM
from module_connect_with_plc_now import ConnectPLC
from PIL import Image, ImageDraw
import copy
import math
from datetime import datetime
from ultralytics import YOLO
import gc
import traceback
import torch

# "C:\Users\pronics\AppData\Local\Programs\Python\Python312\python.exe"




class ConnectPLC_LLL:
    def __init__(self, ip_address="192.168.3.39", port=2500):
        self.ip_address = ip_address
        self.port = port

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

class MainWindow(QMainWindow):
    def __init__(self, pic_cam1=None, pic_cam2=None, pic_cam3=None, stage_cam1=None, stage_cam2=None, stage_cam3=None, mode_re=None):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        ### Led display HMi and check PLC
        self.time_start_led=time.time()

        ### Load AI

        # self.model= YOLO(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\best.pt')
        self.model= YOLO(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\best_new_cam2_ver3.pt')
        img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam2\image.png')
        results= self.model.predict(img)


        self.model_cam1= YOLO(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\best_super_cam1_38.pt')
        img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam1\initial.png')
        results= self.model_cam1.predict(img)


        ## Vitral        
        self.ui.space_screen_cam1_1.mouseMoveEvent=self.mouse_move_event
        #### Setup parameter for cam2
        self.type_blister_now=1 # 2 opposite director

        self.color_red=(0, 0, 255)
        self.color_orange=(0, 165, 255)

        # Variables for FPS calculation
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()


        ###### Phần cam1 thực thi luôn ở phần Display timer

        ###### phần cam2
        self.ui.but_thread_process_cam2.hide()
        self.ui.but_cut_cam2.hide()
        self.ui.but_poly1_cam2.hide()
        self.ui.but_poly2_cam2.hide()
        self.ui.but_rec_cam2.hide()
        self.ui.but_undo_cam2.hide()
        self.ui.display_emptycell_cam2_2.hide()
        self.ui.display_value_number_cam2.hide()
        self.ui.display_value_size_cam2.hide()
        self.ui.value_size_cam2.hide()

        self.ui.display_emptycell_cam2.hide()
        self.ui.display_ratio_poly1.hide()
        self.ui.display_ratio_poly2.hide()
        self.ui.display_value_blister_thread_cam2.hide()
        self.ui.display_value_satisfaction_rate_cam2.hide()
        self.ui.space_screen_cam2_2.hide()
        self.ui.value_blister_thread_cam2.hide()
        self.ui.value_satisfaction_rate_cam2.hide()
        # Fuction button

        ######### Phần cam3
        self.ui.thread_process_place_cam3.hide()

        self.value_blister_thread_cam3=0
        self.value_satisfaction_rate_cam3=0

        self.point_back_direction_cam3=[]

        self.mode_show_pirority_cam3=True

        self.read_data_thread_file_cam3()
        self.start_cam3()

        # Function button
        self.ui.but_pirority_cam3.clicked.connect(self.show_pirority_cam3)
        self.ui.but_thread_process_cam3.clicked.connect(self.show_thread_process_cam3)

        self.ui.value_direction_thread_cam3_2.valueChanged.connect(self.update_value_cam3)
        self.ui.value_satisfaction_rate_cam3_2.valueChanged.connect(self.update_value_cam3)

        # Fuction button
        self.ui.but_cut_cam3_4.clicked.connect(self.cut_cam3)
        self.ui.but_undo_cam3_2.clicked.connect(self.undo_cam3)

        # phần chạy plc 
        self.plc =ConnectPLC_LLL()
        self.s=socket(AF_INET, SOCK_STREAM)
        self.s.connect((self.plc.ip_address, self.plc.port))
        print('OKOKOKOKOK')




        ### Luồng chạy 3 camera 
        # self.stage=Value("i",0)
        self.pic_cam1=pic_cam1
        self.pic_cam2=pic_cam2
        self.pic_cam3=pic_cam3

        self.trigger_cam1= stage_cam1
        self.trigger_cam2= stage_cam2
        self.trigger_cam3= stage_cam3

        self.mode_re=mode_re
        
        # self.trigger_cam1=Value("i",0)
        # self.trigger_cam2=Value("i",0)
        # self.trigger_cam3=Value("i",0)
       

    #     self.camera_process = Process(
    #     target=camera_Basler_multi,
    #     args=(
    #         self.pic_cam1,
    #         self.pic_cam2,
    #         self.pic_cam3,
    #     ),
    #     daemon=True
    # )
    #     self.camera_process.start()
        

        self.time_re= 0
        self.old_value_cam2=0

        # Timer to show image
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.display_on_gui)
        self.timer.start()

        # Tạo thread

        # thread1 = threading.Thread(target=self.display_on_gui_with_cam2)
        # thread1.start()
    
    def mouse_move_event(self,event):
        global_position = event.globalPos()
                    
        # Chuyển đổi vị trí global thành vị trí cục bộ của label
        local_position = self.ui.space_screen_cam1_1.mapFromGlobal(global_position)

        x=y=0
        x= local_position.x()
        y= local_position.y()

        img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\image copy.jpg')

        # frame_info = modlue1.FrameCalibrate(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\Robot_Guide_Template_Binh_Tan_to_modify\calib_camera.json")
        # img=frame_info.undistortImage(img)
        self.image_height,self.image_width,_=img.shape
        

        initial_size = self.ui.space_screen_cam1_1.size()
        self.initial_width = initial_size.width()
        self.initial_height = initial_size.height()
    
        calibration_coefficien_x =self.image_width/self.initial_width
        calibration_coefficien_Y =self.image_height/self.initial_height# hệ số calib
        x=int(x*calibration_coefficien_x)
        y=int(y*calibration_coefficien_Y)
        # coordinate = frame_info.getCoordinate((x,y))
        s =y
        v =x
        k=f"Real X:{s:.1f}, Y:{v:.1f}"
        print(k)
        cv2.putText(img,k,( x+20,y+40), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255, 0, 0),4)
        
        resized_img2 = cv2.resize(img, (self.initial_width,self.initial_height), interpolation=cv2.INTER_LINEAR)
        img_height, img_width, img_channel = resized_img2.shape
        q_image = QImage(resized_img2.data, img_width, img_height, img_width * img_channel, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.ui.space_screen_cam1_1.setPixmap(pixmap) 

    ################
    
    def closeEvent(self, event):
        # Ngắt kết nối camera khi thoát chương trình
        self.s.close()

        print("Ngắt kết nối camera thoát chương trình")    
        event.accept()

    def show_pirority_cam3(self):
        self.ui.priority_place_cam3.show()
        self.ui.thread_process_place_cam3.hide()

        self.mode_show_pirority_cam3=True

    def show_thread_process_cam3(self):
        self.ui.priority_place_cam3.hide()
        self.ui.thread_process_place_cam3.show()
        self.mode_show_pirority_cam3=False
    
    def cut_cam3(self):
        if self.point_end_x_cam3!=None:
            if len(self.point_back_direction_cam3)<=0:
                self.point_back_direction_cam3.append(( (self.point_start_x_cam3,self.point_start_y_cam3),(self.point_end_x_cam3,self.point_end_y_cam3)))
                # print(self.point_back_priority)
                img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_drawed.png',1)

                
                for i, point in enumerate(self.point_back_direction_cam3):
                    # Vẽ hình chữ nhật
                    cv2.rectangle(img,(point[0][0],point[0][1]),(point[1][0],point[1][1]),(0, 165, 255),1) #bgr
                    # Thêm chữ "St:" và số thứ tự
                    cv2.putText(img, "Direction", (point[0][0], point[0][1] - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 1)

                cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_drawed.png',img)

                ####
                data_backup_write=open(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\point_back_direction.txt","w")
                data_all_write = ""
                for point in self.point_back_direction_cam3:
                    data_all_write += f"{point[0][0]} {point[0][1]} {point[1][0]} {point[1][1]}\n"
                data_backup_write.write(data_all_write)
                data_backup_write.close()

        self.point_start_x_cam3=None
        self.point_start_y_cam3=None

        self.point_end_x_cam3=None
        self.point_end_y_cam3=None

    
    def undo_cam3(self):
        if len(self.point_back_direction_cam3) !=0:
            self.point_back_direction_cam3.pop()

            ####
            data_backup_write=open(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\point_back_direction.txt","w")
            data_all_write = ""
            for point in self.point_back_direction_cam3:
                data_all_write += f"{point[0][0]} {point[0][1]} {point[1][0]} {point[1][1]}\n"
            data_backup_write.write(data_all_write)
            data_backup_write.close()
            #####
            self.start_cam3()
            img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_drawed.png',1)
            cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_to_draw.png',img)


        self.point_start_x_cam3=None
        self.point_start_y_cam3=None

        self.point_end_x_cam3=None
        self.point_end_y_cam3=None

    def mousePressEvent(self, event):
        if self.ui.tabWidget.currentIndex() == 2:
            if self.mode_show_pirority_cam3==True:
                
                # Tính toán vị trí của sự kiện chuột trên toàn bộ nền
                global_position = event.globalPos()
                
                # Chuyển đổi vị trí global thành vị trí cục bộ của label
                local_position = self.ui.space_screen_cam3_4.mapFromGlobal(global_position)

                x=y=0
                # x=event.x()-124 ### location x
                # y=event.y()-48  ### location y
                x= local_position.x()
                y= local_position.y()
                print(f'Tọa độ lần lượt là Start {x} và {y} \n')
                if (x>=0 and x<=511) and (y>=0 and y<=571) and len(self.point_back_direction_cam3)<=0: ### suitable with W and H
                    # print('OKKKKKKKKKk cam3')
                    self.point_start_x_cam3=x
                    self.point_start_y_cam3=y
    
    def mouseMoveEvent(self, event):
        if self.mode_show_pirority_cam3==True:
            global_position = event.globalPos()
                
            # Chuyển đổi vị trí global thành vị trí cục bộ của label
            local_position = self.ui.space_screen_cam2_1.mapFromGlobal(global_position)

            x=y=0
            # x=event.x()-124 ### location x
            # y=event.y()-48  ### location y
            x= local_position.x()
            y= local_position.y()
           
            if (x>=0 and x<=511) and (y>=0 and y<=571) and  len(self.point_back_direction_cam3)<=0:
                # print(f'Tọa độ lần lượt là Start {self.point_start_x} và {self.point_start_y} \n')
                img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_drawed.png',1)

                cv2.rectangle(img,(self.point_start_x_cam3,self.point_start_y_cam3),(x,y),((255, 0, 255)),1) ### bgr
                cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_to_draw.png',img)

    def mouseReleaseEvent(self, event):
        if self.mode_show_pirority_cam3==True:
            global_position = event.globalPos()
                
            # Chuyển đổi vị trí global thành vị trí cục bộ của label
            local_position = self.ui.space_screen_cam2_1.mapFromGlobal(global_position)

            x=y=0
            # x=event.x()-124 ### location x
            # y=event.y()-48  ### location y
            x= local_position.x()
            y= local_position.y()
            
            if (x>=0 and x<=511) and (y>=0 and y<=571) and  len(self.point_back_direction_cam3)<=0:
                self.point_end_x_cam3=x
                self.point_end_y_cam3=y
                # print('hahaha')
                img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_drawed.png',1)

                cv2.rectangle(img,(self.point_start_x_cam3,self.point_start_y_cam3),(self.point_end_x_cam3,self.point_end_y_cam3),((255, 0, 255)),2) ### bgr
                cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_to_draw.png',img)


    
    def read_data_thread_file_cam3(self):
        #################### Thread blister
        file_read_value_adjust= open(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\data_thread_cam3.txt","r")
        file_read_value_adjust=file_read_value_adjust.readlines()
        # Set value wwith hozier
        self.ui.value_direction_thread_cam3_2.setValue(int(file_read_value_adjust[0]))
        self.ui.value_satisfaction_rate_cam3_2.setValue(int(file_read_value_adjust[1]))


        # Upgrade data alongwith Gui
        self.value_blister_thread_cam3= self.ui.value_direction_thread_cam3_2.value()
        self.ui.display_value_direction_thread_cam3_2.setText('Direction thread: '+str(self.value_blister_thread_cam3))

        self.value_satisfaction_rate_cam3= self.ui.value_satisfaction_rate_cam3_2.value()
        self.ui.display_value_satisfaction_rate_cam3_2.setText('Satisfaction rate: '+str(self.value_satisfaction_rate_cam3))
    
    def read_points_from_file_cam2(self,filename):
        points = []
        with open(filename, "r") as file:
            lines = file.readlines()

            if not lines:
                return points

            for line in lines:
                if line.strip():
                    x1, y1 = map(int, line.strip().split())
                    points.append(((x1, y1)))
        return points

    def read_points_from_file_cam3(self,filename):
        points = []
        with open(filename, "r") as file:
            lines = file.readlines()

            if not lines:
                return points

            for line in lines:
                if line.strip():
                    x1, y1, x2, y2 = map(int, line.strip().split())
                    points.append(((x1, y1), (x2, y2)))
        return points


    def update_value_cam3(self):
        self.value_blister_thread_cam3= self.ui.value_direction_thread_cam3_2.value()
        self.ui.display_value_direction_thread_cam3_2.setText('Direction thread: '+str(self.value_blister_thread_cam3))

        self.value_satisfaction_rate_cam3= self.ui.value_satisfaction_rate_cam3_2.value()
        self.ui.display_value_satisfaction_rate_cam3_2.setText('Satisfaction rate: '+str(self.value_satisfaction_rate_cam3))

        self.update_data_cam3()

    def update_data_cam2(self):
        ########## Blister thread
        self.value_blister_thread_cam2= self.ui.value_blister_thread_cam2.value()
        self.value_satisfaction_rate_cam2= self.ui.value_satisfaction_rate_cam2.value()

        data_backup_write=open(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\data_thread_cam2.txt","w")
        data_all_write = [str(self.value_blister_thread_cam2) +'\n' +str(self.value_satisfaction_rate_cam2)]
        
        data_backup_write.writelines(data_all_write)
        data_backup_write.close()

        ########## Size
        self.value_size_cam2= self.ui.value_size_cam2.value()

        data_backup_write=open(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\data_size.txt","w")
        data_all_write = [str(self.value_size_cam2)]
        
        data_backup_write.writelines(data_all_write)
        data_backup_write.close()

    def update_data_cam3(self):
        ########## Blister thread
        self.value_blister_thread_cam3= self.ui.value_direction_thread_cam3_2.value()
        self.value_satisfaction_rate_cam3= self.ui.value_satisfaction_rate_cam3_2.value()

        data_backup_write=open(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\data_thread_cam3.txt","w")
        data_all_write = [str(self.value_blister_thread_cam3) +'\n' +str(self.value_satisfaction_rate_cam3)]
        
        data_backup_write.writelines(data_all_write)
        data_backup_write.close()   
    
    def start_cam3(self):
        
        img=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image.png',1)

        self.point_back_direction_cam3=self.read_points_from_file_cam3(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\point_back_direction.txt")
        # print(len(self.point_back_priority))
        for i, point in enumerate(self.point_back_direction_cam3):
            # Vẽ hình chữ nhật
            cv2.rectangle(img,(point[0][0],point[0][1]),(point[1][0],point[1][1]),(0, 165, 255),2) #bgr
            # Thêm chữ "St:" và số thứ tự
            cv2.putText(img, "Direction", (point[0][0], point[0][1] - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 1)

        cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_drawed.png',img)

        self.point_start_x_cam3=None
        self.point_start_y_cam3=None

        self.point_end_x_cam3=None
        self.point_end_y_cam3=None
    
    def extract_rectangles_cam3(self):
        img_original=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image.png',1)
        # if img_original is not None:

        img_original=cv2.cvtColor(img_original,cv2.COLOR_RGB2BGR)

        self.point_back_direction_cam3=self.read_points_from_file_cam3(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_txt\point_back_direction.txt")
        
        # print(self.point_back_direction)
        rectangles = []
        for ((x, y), (m, n)) in self.point_back_direction_cam3:
            rectangle = img_original[y:n, x:m]
            rectangles.append((rectangle, x, y, m, n))
        return rectangles
    
    ###### Code riêng cho cam 1
    def clearTableData(self):
        self.row=-1
        self.count =0 
        self.ui.table_cam1.clearContents()
        row_count = self.ui.table_cam1.rowCount()
        # Xóa tất cả các dòng nếu có ít nhất một dòng
        if row_count > 0:
            self.ui.table_cam1.setRowCount(0)

    def Update_table(self,datas):

        current_time = datetime.now()
        # In ra ngày, tháng, năm, giờ, phút và giây
        date_time = f"{current_time.strftime('%Y-%m-%d %H:%M:%S')}"

        for data in datas:

            self.row=self.row+1
            self.count=self.count+1
            self.ui.table_cam1.setRowCount(self.row + 1)  # Cập nhật số lượng hàng nếu cần
            
            self.ui.table_cam1.setItem(self.row, 0, QtWidgets.QTableWidgetItem(str(self.count)))
            self.ui.table_cam1.setItem(self.row, 1, QtWidgets.QTableWidgetItem(str(data[0])))
            self.ui.table_cam1.setItem(self.row, 2, QtWidgets.QTableWidgetItem(str(data[1])))
            self.ui.table_cam1.setItem(self.row, 3, QtWidgets.QTableWidgetItem(str(data[2])))
            
    def send_data(self,address,value):
        try:
            print(f"Move {value} into D{address}")
            self.s.sendall((bytes(self.plc.send_D_bit(address, value))))
            time.sleep(0.043) 
        except Exception as e:
            print(f"Error in send_data: {e}")
            pass
            # for i in range(1000):
            #     try:
            #         self.plc =ConnectPLC_LLL()
            #         self.s=socket(AF_INET, SOCK_STREAM)
            #         self.s.connect((self.plc.ip_address, self.plc.port))
            #         print("PLC connection successful")
            #         break
            #     except Exception as e:
            #         print(e)
            #         time.sleep(1)



    def optimal_rotation(self,alpha, beta):
        # Tính góc chênh lệch
        delta_theta = beta - alpha
        
        # Tối ưu hóa quãng đường quay
        if delta_theta > 180:
            delta_theta -= 360  # Quay ngược chiều kim đồng hồ (CCW)
        elif delta_theta < -180:
            delta_theta += 360  # Quay theo chiều kim đồng hồ (CW)
        
        # Xác định hướng quay
        if delta_theta > 0:
            direction = "CW"  # Chiều kim đồng hồ
        else:
            direction = "CCW"  # Ngược chiều kim đồng hồ
        
        return abs(delta_theta), direction
    
    def calculate_angle(self, rect_points):
        dx = rect_points[2][0] - rect_points[1][0]
        dy = rect_points[2][1] - rect_points[1][1]
        angle = math.atan2(dy, dx)
        return angle

    def calculate_center(self, rect_points):
        # Tính tọa độ tâm của hình chữ nhật
        x_coords = [p[0] for p in rect_points]
        y_coords = [p[1] for p in rect_points]
        
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))
    
        return center_x, center_y

    def display_on_gui(self):
        try:        
            # if self.stage.value ==1:
            if ( not self.pic_cam1.empty() or not self.pic_cam2.empty() or not self.pic_cam3.empty()) and self.mode_re.value ==1:
            # if self.mode_re.value ==1:
                self.time_re=time.time()


            if (  self.pic_cam1.empty() and  self.pic_cam2.empty() and self.pic_cam3.empty()) and (time.time() - self.time_re) >70 and self.mode_re.value ==1 :
            # if (  self.pic_cam1.empty() and  self.pic_cam2.empty() and self.pic_cam3.empty()) and (time.time() - self.time_re) >60 and self.mode_re.value ==1 :
            # if  (time.time() - self.time_re) >10 and self.mode_re.value ==1 :
                self.s.close()
                gc.collect()
                torch.cuda.empty_cache()

                time.sleep(0.5)
                self.plc =ConnectPLC_LLL()
                self.s=socket(AF_INET, SOCK_STREAM)
                self.s.connect((self.plc.ip_address, self.plc.port))
                print("PLC connection AGAIN successfulllllllllllllllllllllllllllllllllllllll")
                self.time_re=time.time()
            


                

            if (time.time()-self.time_start_led>2.5) and self.pic_cam1.empty() and self.pic_cam2.empty() and self.pic_cam3.empty() :
                self.send_data(466,1)
                self.time_start_led=time.time()
                gc.collect()
                torch.cuda.empty_cache()
            x_central=0
            y_central=0
            theta_central=0

            ###### Tính toán hiệu năng 
            # Increment frame count
            self.frame_count += 1
            elapsed_time = time.time() - self.start_time
            if elapsed_time > 1.0:
                self.fps = self.frame_count / elapsed_time
                # print(f"FPS: {self.fps}")
                # print(f"Time_re: {self.time_re}")
                # print(f"Mode re: {self.mode_re.value}")
                # print(f"Time_cam1: {not self.pic_cam1.empty()}")
                self.frame_count = 0
                self.start_time = time.time()


            
            ## Cập nhập hình ảnh 
            if not self.pic_cam3.empty():
            # if self.trigger_cam3.get()==1:
                self.start_cam3()
                # self.trigger_cam3.value=0


                ######### Cam 1
            path_file_cam1_AI=r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\data_AI\cam1'
            if not self.pic_cam1.empty():
                # img_original=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam1\image.png',1)
                img_original=self.pic_cam1.get()
                # if img_original is not None and self.trigger_cam1.get()==1:
                if img_original is not None :
                # if img_original is not None :
                    data=[]
                    data1=[]
                    data2=[]
                    number_part=0
                    # self.home_cam1=[545.0, 736.0, 12.407418527400745]
                    self.home_cam1=[565.5, 759.0, 9.509005342328448]
                    # Gọi phương thức 
                    processed_image = modlue1.Trigger_image_customer_ver2(img_original)
                    
                    # file_name=datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f") +'.png'
                    # link=os.path.join(path_file_cam1_AI,file_name)
                    # cv2.imwrite(link,processed_image)
                    cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam1\initial.png',processed_image)

                    processed_image=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam1\initial.png')
                    results= self.model_cam1.predict(processed_image)

                    number_part=len(results[0].obb.cls)

                    for i in range(number_part):
                        if results[0].obb[i].conf>0.8:
                            id= results[0].obb[i].cls
                            
                            datas=results[0].obb[i].xyxyxyxy
                            datas=datas.cpu().numpy()

                            for value in datas:
                                points = np.array(value, dtype=np.int32).reshape((-1, 1, 2))

                                if results[0].names[id.item()]=='product':
                                    cv2.polylines(processed_image, [points], isClosed=True, color=(255, 0, 0), thickness=3)
                                    angle=self.calculate_angle(value)
                                    degree = math.degrees(angle)
                                    
                                    if degree>-90:
                                        degree=degree-180
                                    angle= math.radians(degree)
                                    # print(degree)
                                    
                                    a,b=self.calculate_center(value)
                                    cv2.circle(processed_image, (a,b), 5, (255, 0, 0) , 3) 
                                    

                                    L=150
                                    ax= int(a + L*np.cos(angle))
                                    bx= int(b + L*np.sin(angle))

                                    cv2.arrowedLine(processed_image, (a,b), (ax,bx), (255,0,0),2,2)

                                    degree=90-degree

                                    print(f'Lan luot gia tri a:{a} b:{b} theta:{degree}')
                                    if not data1:
                                        data1.append([b,a,degree])
                                    else:
                                        data2.append([b,a,degree])

                                    # data.append([b,a,degree])
                    if data1 and not data2:
                        data= data1
                    elif not data1 and data2:
                        data= data2
                    elif data1 and data2:
                        if data1[0]>data2[0]:
                            data=data + data1 +data2
                        else:
                            data=data + data2 +data1


                    
                    
                    self.ui.display_number_items_cam1.setText('Number of visible items: '+ str(len(data)))
                    # Handle data
                    self.clearTableData()
                    self.Update_table(data)                
                    # anpha=1
                    # beta=1
                    
                    # self.L=307.1742176680849
                    # self.L=302.65863609023285+15.0
                    self.L=302.65863609023285

                    k_constant_anpha=0
                    k_constant_beta=0
                    angle_move=0
                    delta_l=0

                    print(data)

                    if len(data)==0:
                        print('gia tri khong co')
                        self.send_data(476,1)

                    else:
                        for i in range(len(data)):
                            if i==0:
                                angle_radiant=0
                                if data[i][0]<385:
                                    print(123)
                                    angle = data[i][2]+90
                                    angle_radiant = math.radians(angle)
                                    # k=int(self.ui.lineEdit.text())
                                    k=20 #18 10 7
                                    delta_l=-6
                                else:
                                    print(456)
                                    angle = data[i][2]-90
                                    angle_radiant = math.radians(angle)
                                    k=5 #12
                                    delta_l=0
                                
                                angle_move=data[i][2]+180

                                x_new_move = data[i][0] + k * math.cos(math.radians(angle_move))
                                y_new_move = data[i][1] + k  * math.sin(math.radians(angle_move))
                                cv2.circle(processed_image,(int(y_new_move),int(x_new_move)), 5, (255, 175, 64), 3)
                                
                                # x_new = data[i][0]+  self.L * math.cos(angle_radiant)
                                # y_new = data[i][1]+  self.L * math.sin(angle_radiant)

                                x_new = x_new_move+  (self.L-delta_l) * math.cos(angle_radiant)
                                y_new = y_new_move+  (self.L-delta_l) * math.sin(angle_radiant)
                                
                            
                                cv2.circle(processed_image,(int(y_new),int(x_new)), 5, (255, 255, 0), 3)

                                #### Tinh toan gia tri xung

                                x_xung= int( int((x_new-self.home_cam1[0])*2/10*(-2000/150))/(1))
                                y_xung= int( int((y_new-self.home_cam1[1])*2/10*(-15000/150))/(1))

                                # distance= math.sqrt( ((x_new - data[i][0])*2/10)**2 + ((y_new-data[i][1])*2/10)**2 )
                                # print(f"Vi tri lan luot la TH1 : ({x_new},{y_new})  ({data[i][0]},{data[i][1]}) with distance: {distance}")
                    #             # print(f"Angle:{angle}")
                    #             # print((f" Value now: {angle+180}"))
                                theta,rotate= self.optimal_rotation(angle+180,self.home_cam1[2])
                                if rotate=='CW':
                                    theta_xung=int(theta*2000/360)
                                else:
                                    theta_xung=int(theta*-2000/360)
                                
                                if theta_xung<0:
                                    theta_xung=theta_xung+ 2000

                                print(f"gia tri 1 : {x_xung} , {y_xung}, {theta_xung}")

                                self.send_data(302,1)
                                self.send_data(272,x_xung)
                                self.send_data(270,y_xung)
                                self.send_data(274,theta_xung)
                                if len(data)==1:
                                    self.send_data(476,1)

                            elif i==1:
                                angle_radiant=0
                                if data[i][0]<385:
                                    print(123)
                                    angle = data[i][2]+90
                                    angle_radiant = math.radians(angle)
                                    # k=int(self.ui.lineEdit.text())
                                    k=20 #18 10 7
                                    delta_l=-6
                                else:
                                    print(456)
                                    angle = data[i][2]-90
                                    angle_radiant = math.radians(angle)
                                    k=5 #12
                                    delta_l=0
                                
                                angle_move=data[i][2]+180

                                x_new_move = data[i][0] + k * math.cos(math.radians(angle_move))
                                y_new_move = data[i][1] + k  * math.sin(math.radians(angle_move))
                                cv2.circle(processed_image,(int(y_new_move),int(x_new_move)), 5, (255, 175, 64), 3)
                                
                                # x_new = data[i][0]+  self.L * math.cos(angle_radiant)
                                # y_new = data[i][1]+  self.L * math.sin(angle_radiant)

                                x_new = x_new_move+  (self.L-delta_l) * math.cos(angle_radiant)
                                y_new = y_new_move+  (self.L-delta_l) * math.sin(angle_radiant)
                                
                            
                                cv2.circle(processed_image,(int(y_new),int(x_new)), 5, (255, 255, 0), 3)

                                #### Tinh toan gia tri xung

                                x_xung= int( int((x_new-self.home_cam1[0])*2/10*(-2000/150))/(1))
                                y_xung= int( int((y_new-self.home_cam1[1])*2/10*(-15000/150))/(1))

                    #             distance= math.sqrt( ((x_new - data[i][0])/10)**2 + ((y_new-data[i][1])/10)**2 )
                    #             print(f"Vi tri lan luot la TH1 : ({x_new},{y_new})  ({data[i][0]},{data[i][1]}) with distance: {distance}")
                    #             # print(f"Angle:{angle}")
                    #             # print((f" Value now: {angle+180}"))
                                theta,rotate= self.optimal_rotation(angle+180,self.home_cam1[2])
                                if rotate=='CW':
                                    theta_xung=int(theta*2000/360)
                                else:
                                    theta_xung=int(theta*-2000/360)
                                
                                if theta_xung<0:
                                    theta_xung=theta_xung+ 2000

                                print(f"gia tri 2 : {x_xung} , {y_xung}, {theta_xung}")
                    
                                self.send_data(310,1)
                                self.send_data(278,x_xung)
                                self.send_data(276,y_xung)
                                self.send_data(280,theta_xung)
                                if len(data)==2:
                                    self.send_data(476,1)
                    data=[]
                    
                    

                    ##########
                    cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\image copy.jpg',processed_image)

                    initial_size = self.ui.space_screen_cam1_1.size()
                    self.initial_width = initial_size.width()
                    self.initial_height = initial_size.height()

                    resized_img2 = cv2.resize(processed_image, (self.initial_width,self.initial_height), interpolation=cv2.INTER_LINEAR)
                    img_height, img_width, img_channel = resized_img2.shape
                    q_image = QImage(resized_img2.data, img_width, img_height, img_width * img_channel, QImage.Format.Format_RGB888)
                    pixmap = QPixmap.fromImage(q_image)
                    self.ui.space_screen_cam1_1.setPixmap(pixmap) 
                    # self.trigger_cam1.value=0


            #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            
            ######### Cam 2
            
            # img_original=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam2\image.png')
            if not self.pic_cam2.empty():
                img_original= self.pic_cam2.get()
                # img_original=cv2.resize(img_original.copy(),(511,571))
                # if img_original is not None and self.trigger_cam2.get()==1:
                if img_original is not None :
                    count_ok=0
                    count_ng=0
                    number_part=0
                    data_cam2_send=0
                    print('1111111111111111111111111111')

                    results= self.model.predict(img_original)

                    number_part= len(results[0].obb.cls)

                    for i in range(number_part):
                        if results[0].obb[i].conf>0.78:
                            id= results[0].obb[i].cls[0]
                            # print(id)
                            if results[0].names[id.item()]=='ok':
                                count_ok+=1
                            elif results[0].names[id.item()]=='ng':
                                count_ng+=1
                    
                    for i in range(number_part):
                        if results[0].obb[i].conf>0.78:
                            id= results[0].obb[i].cls
                            
                            datas=results[0].obb[i].xyxyxyxy
                            datas=datas.cpu().numpy()

                            for data in datas:
                                points = np.array(data, dtype=np.int32).reshape((-1, 1, 2))
                                if results[0].names[id.item()]=='ok':
                                    # Vẽ OBB lên ảnh (màu xanh lá cây và đường viền dày 2 pixel)
                                    cv2.polylines(img_original, [points], isClosed=True, color=(255, 0, 0), thickness=2)
                                    
                                elif results[0].names[id.item()]=='ng':
                                    # Vẽ OBB lên ảnh (màu đỏ và đường viền dày 2 pixel)
                                    cv2.polylines(img_original, [points], isClosed=True, color=(0, 0, 255), thickness=2)
                    
                    cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam2\iamge_handled_AI.png',img_original)
                    
                    ###Send Data
                    # if count_ng!=0:
                    #     self.old_value_cam2=0
                    #     self.send_data(464,1)
                    #     print('Tha hang sai hoan toan')
                    # else:
                    #     data_cam2_send=count_ok+1
                    #     if data_cam2_send != self.old_value_cam2:
                    #         self.send_data(460,data_cam2_send)
                    #         self.send_data(478,1)
                    #         self.old_value_cam2=data_cam2_send
                    #         print(f"Gia tri o tiep theo la{data_cam2_send}")
                    #     else:
                    #         if self.old_value_cam2!=2:
                    #             print(f'Ok Reset lai nhe vi tung lap {self.old_value_cam2}')
                    #             self.old_value_cam2=0
                    #             self.send_data(464,1)
                    #         else:
                    #             self.send_data(460,data_cam2_send)
                    #             self.send_data(478,1)
                    #             self.old_value_cam2=data_cam2_send

                    if count_ng>1:
                        self.old_value_cam2=0
                        self.send_data(464,1)
                        print('Tha hang sai hoan toan')
                    else:
                        data_cam2_send=count_ok+1+count_ng
                        if data_cam2_send != self.old_value_cam2:
                            self.send_data(460,data_cam2_send)
                            self.send_data(478,1)
                            self.old_value_cam2=data_cam2_send
                            print(f"Gia tri o tiep theo la{data_cam2_send}")
                        else:
                            if self.old_value_cam2!=2:
                                print(f'Ok Reset lai nhe vi tung lap {self.old_value_cam2}')
                                self.old_value_cam2=0
                                self.send_data(464,1)
                            else:
                                self.send_data(460,data_cam2_send)
                                self.send_data(478,1)
                                self.old_value_cam2=data_cam2_send
                    # self.send_data(460,self.count_cc)
                    # self.send_data(478,1)
                    # self.count_cc=self.count_cc+1
                    # if self.count_cc>8:
                    #     self.count_cc=2
                    # self.send_data(478,1)
                            
                    self.ui.space_screen_cam2_1.setPixmap(QtGui.QPixmap(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam2\iamge_handled_AI.png"))

                    # self.trigger_cam2.value=0


            # #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                
            
            ######## Cam 3 xx
            data_cam3=0
            if not self.pic_cam3.empty():
                img_original= self.pic_cam3.get()

                # img_original=cv2.imread(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image.png',1)
                # img_original= self.pic_cam3.get()
                if img_original is not None :

                    if self.point_start_x_cam3!=None:
                        self.ui.space_screen_cam3_4.setPixmap(QtGui.QPixmap(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_to_draw.png"))
                    else:
                        self.ui.space_screen_cam3_4.setPixmap(QtGui.QPixmap(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image_drawed.png"))

                    self.ui.display_value_direction_cam3_2.setText('Number: '+str(len(self.point_back_direction_cam3)))

                    if len(self.point_back_direction_cam3)==1:
                        self.ui.display_value_direction_cam3_2.setStyleSheet('background-color: green')
                    else:
                        self.ui.display_value_direction_cam3_2.setStyleSheet('background-color: white')
                    
                    # Tạo ảnh trắng (ảnh xám) với cùng kích thước ảnh gốc
                    composite_image = np.ones((img_original.shape[0], img_original.shape[1]), dtype=np.uint8) * 0

                    # Trích xuất các vùng hình chữ nhật từ ảnh
                    rectangles = self.extract_rectangles_cam3()

                    count_pixel_white=0
                    ratio_white=0

                    for rectangle, x, y, m, n in rectangles:
                    # Áp dụng phân ngưỡng cho mỗi rectangle
                        # Chuyển đổi màu từ BGR (mặc định của OpenCV) sang RGB cho PyQt5 hiển thị
                        rectangle = cv2.cvtColor(rectangle, cv2.COLOR_BGR2GRAY)

                        # Áp dụng GaussianBlur để giảm nhiễu
                        img_blur = cv2.GaussianBlur(rectangle, (5, 5), 1.4)

                        # Áp dụng phương pháp Canny để tìm các cạnh
                        # edges = cv2.Canny(img_blur, threshold1=20, threshold2=200)
                        edges = cv2.Canny(img_blur, threshold1=self.value_blister_thread_cam3, threshold2=255)

                        
                        # Tạo kernel để giãn nở
                        kernel = np.ones((7,7), np.uint8)

                        # Áp dụng giãn nở
                        edges = cv2.dilate(edges, kernel, iterations=1)
                        count_pixel_white= cv2.countNonZero(edges)
                        # Bước 4: Tính phần trăm pixel trắng
                        ratio_white = (count_pixel_white / edges.size) * 100


                        if len(self.point_back_direction_cam3)==1:
                            self.ui.display_value_ratio_now_cam3.setText('Ratio now: '+str(ratio_white))
                            if ratio_white > int(self.ui.value_satisfaction_rate_cam3_2.value()):
                                self.ui.display_detected_letter_cam3_2.setStyleSheet('background-color: green')
                                data_cam3=1
                            else:
                                self.ui.display_detected_letter_cam3_2.setStyleSheet('background-color: white')
                                data_cam3=2
                            
                        # Gán rectangle đã phân ngưỡng vào composite_image
                        composite_image[y:n, x:m] = edges
                    
                    for i, point in enumerate(self.point_back_direction_cam3):
                        # Thêm chữ "St:" và số thứ tự
                        cv2.putText(composite_image, "Direction", (point[0][0], point[0][1] - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255), 1)
                    
                    h, w = composite_image.shape  # Composite image chỉ có hai chiều (h, w) vì là ảnh xám
                    q = QImage(composite_image.data, w, h, w, QImage.Format_Grayscale8)
                    self.ui.space_screen_cam3_3.setPixmap(QPixmap.fromImage(q))
                    # print(data_cam3)

                    # if self.trigger_cam3.get()==1:
                    # self.send_data(462,1)
                    self.send_data(462,data_cam3)


                    # self.trigger_cam3.value=0

                
                
            # self.stage.value=0
        except Exception as e:
            # Ghi lỗi vào file
            with open(r"C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\error_log\error_log_thread_processing_send_Data.txt", "a") as log_file:
                current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                log_file.write(f"[{current_time}] Error: {str(e)}\n")
                log_file.write(traceback.format_exc())  # Ghi chi tiết lỗi
                log_file.write("\n")
            # print("Đã ghi lỗi vào file error_log.txt")

def capture_camera(camera, img_queue, resize=None, stop_event= None, mode_save=False):
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    
    while not stop_event.is_set():
        grabResult = camera.RetrieveResult(0, pylon.TimeoutHandling_Return)
        try:
            if grabResult.GrabSucceeded():
                image = converter.Convert(grabResult)
                img = image.GetArray()
                if resize:
                    img = cv2.resize(img, resize, interpolation=cv2.INTER_CUBIC)
                if mode_save:
                    cv2.imwrite(r'C:\Users\pronics\Desktop\sort_blister_hn_ver2\program\cam3\image.png',img)
                img_queue.put(img)
            grabResult.Release()
        except Exception as e:
            ERORRRRRRRR = "LOL"

# def camera_Basler_multi(stage,trigger_cam1,trigger_cam2,trigger_cam3):
def camera_Basler_multi(pic_cam1, pic_cam2, pic_cam3, mode_re):
    id_cam1=0
    cam1=0
    id_cam2=0
    cam2=0
    id_cam3=0
    cam3=0

    maxCamerasToUse = 10
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()

    if len(devices) == 0:
        raise pylon.RuntimeException("No camera present.")
    cameras = pylon.InstantCameraArray(min(len(devices), maxCamerasToUse))

    for i, camera in enumerate(cameras):
        # count=count+1
        
        if i ==0: 
            id_cam2=0
            cam2=camera
        elif i==1:
            id_cam3=1
            cam3=camera
        else:
            id_cam1=2
            cam1= camera


        camera.Attach(tlFactory.CreateDevice(devices[i]))
        ###
        while True:
            try:
                camera.Open()
                break
            except pylon.RuntimeException as e:
                print(f"Error: {e}. Retrying in 5 seconds...")
                time.sleep(5)

        print("DeviceClass: ", camera.GetDeviceInfo().GetDeviceClass())
        print("DeviceFactory: ", camera.GetDeviceInfo().GetDeviceFactory())
        print("ModelName: ", camera.GetDeviceInfo().GetModelName())
        camera.MaxNumBuffer = 100
        camera.TriggerSelector.SetValue('FrameStart') #  Continuous
        camera.TriggerSource.SetValue('Line1')
        camera.TriggerActivation.SetValue('RisingEdge')  #Falling Edge   RisingEdge  99995  309540

    cam1.TriggerMode.SetValue('On')
    cam2.TriggerMode.SetValue('On')
    cam3.TriggerMode.SetValue('On')
    cam3.ExposureTimeAbs.SetValue(5005)

    cam_mapping = {2: cam1, 0: cam2, 1: cam3}
    cameras.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    stop_event = threading.Event()

    if mode_re.value==0:
        mode_re.value = 1


    t1 = threading.Thread(target=capture_camera, args=(cam_mapping[2], pic_cam1, None, stop_event, False), daemon=True)
    t2 = threading.Thread(target=capture_camera, args=(cam_mapping[0], pic_cam2, (576, 432), stop_event, False), daemon=True)
    t3 = threading.Thread(target=capture_camera, args=(cam_mapping[1], pic_cam3, (576, 432), stop_event, True), daemon=True)
    
    t1.start()
    t2.start()
    t3.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping all threads...")
        stop_event.set()
    
    t1.join()
    t2.join()
    t3.join()
    
   
    cameras.StopGrabbing()
    cameras.Close()
    print('Ket thuc chuong trinh')

# Khi muốn dừng camera
def stop_cameras(stop_event):
    stop_event.set()

def main_program(pic_cam1, pic_cam2, pic_cam3, stage_cam1, stage_cam2, stage_cam3,mode_re):
    app = QApplication(sys.argv)
    main_win = MainWindow(pic_cam1=pic_cam1, pic_cam2=pic_cam2, pic_cam3=pic_cam3, stage_cam1=stage_cam1, stage_cam2=stage_cam2, stage_cam3=stage_cam3, mode_re=mode_re)
    main_win.show()
    sys.exit(app.exec())
def inital():
    multiprocessing.freeze_support()
    print('Scaning this PC')
    connect= ConnectPLC()
    connect.connect_and_send()
    # main_program()()

    ###############
    queue_pic_cam1= Queue()
    queue_pic_cam2= Queue()
    queue_pic_cam3= Queue()
    # stage_cam1=Value("i",0)
    # stage_cam2=Value("i",0)
    # stage_cam3=Value("i",0)
    stage_cam1=Queue()
    stage_cam2=Queue()
    stage_cam3=Queue()
    mode_re=Value("i",0)


    process1 = Process(target = main_program, args=(queue_pic_cam1, queue_pic_cam2, queue_pic_cam3, stage_cam1, stage_cam2, stage_cam3, mode_re,))
    process1.start()

    process2 = Process(target = camera_Basler_multi, args=(queue_pic_cam1, queue_pic_cam2, queue_pic_cam3, mode_re, ))
    process2.start()

    

if __name__ == "__main__":
    inital()
    
   