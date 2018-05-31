#coding=utf-8
import threading
import SocketServer
import cv2
import numpy as np
import math
import socket
import time

# distance data measured by ultrasonic sensor
sensor_data = " "
prediction = " "

class DistanceToCamera(object):       ###这一部分是利用图像，而非超声波，计算检测到的物体（如stop标识）与相机间距离，原理是单目视觉

    def __init__(self):
        # camera params
        self.alpha = 8.0 * math.pi / 180
        self.v0 = 231.00885300695285         ###这里是棋盘格校准后得到的数据，这些数据与镜头有关
        self.ay = 717.27500143296152

    def calculate(self, v, h, x_shift, image):
        # compute and return the distance from the target point to the camera
        d = h / math.tan(self.alpha + math.atan((v - self.v0) / self.ay))
        if d > 0:
            cv2.putText(image, "%.1fcm" % d,
                (image.shape[1] - x_shift, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return d


class ObjectDetection(object):        ###这一部分用于检测并标记图像中出现的红绿灯和stop

    def __init__(self):
        self.red_light = False
        self.green_light = False
        self.yellow_light = False

    def detect(self, cascade_classifier, gray_image, image):

        # y camera coordinate of the target point 'P'
        v = 0

        # minimum value to proceed traffic light state validation
        threshold = 150     
        
        # detection
        cascade_obj = cascade_classifier.detectMultiScale(
            gray_image,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )

        # draw a rectangle around the objects
        for (x_pos, y_pos, width, height) in cascade_obj:
            cv2.rectangle(image, (x_pos+5, y_pos+5), (x_pos+width-5, y_pos+height-5), (255, 255, 255), 2)
            v = y_pos + height - 5
            #print(x_pos+5, y_pos+5, x_pos+width-5, y_pos+height-5, width, height)

            # stop sign
            if width/height == 1:
                cv2.putText(image, 'STOP', (x_pos, y_pos-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # traffic lights
            else:
                roi = gray_image[y_pos+10:y_pos + height-10, x_pos+10:x_pos + width-10]
                mask = cv2.GaussianBlur(roi, (25, 25), 0)
                (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(mask)
                
                # check if light is on
                if maxVal - minVal > threshold:
                    cv2.circle(roi, maxLoc, 5, (255, 0, 0), 2)
                    
                    # Red light
                    if 1.0/8*(height-30) < maxLoc[1] < 4.0/8*(height-30):
                        cv2.putText(image, 'Red', (x_pos+5, y_pos-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        self.red_light = True
                    
                    # Green light
                    elif 5.5/8*(height-30) < maxLoc[1] < height-30:
                        cv2.putText(image, 'Green', (x_pos+5, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        self.green_light = True
    
                    # yellow light
                    #elif 4.0/8*(height-30) < maxLoc[1] < 5.5/8*(height-30):
                    #    cv2.putText(image, 'Yellow', (x_pos+5, y_pos - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    #    self.yellow_light = True
        return v


class SensorDataHandler(SocketServer.BaseRequestHandler):        ###这一部分用于接收超声波数据，得到的是正前方的障碍物距离

    data = " "

    def handle(self):
        global sensor_data
        try:
            while self.data:
                self.data = self.request.recv(1024)
                sensor_data = round(float(self.data), 1)
                print sensor_data
        finally:
            print "Connection closed on thread 2"



class VideoStreamHandler(SocketServer.StreamRequestHandler):   ###获取图像，根据图像预测识别路径，得出识别结果

    global prediction
    # h1: stop sign
    h1 = 15.5 - 10  # cm
    # h2: traffic light
    h2 = 15.5 - 10

    obj_detection = ObjectDetection()

    # cascade classifiers
    stop_cascade = cv2.CascadeClassifier('cascade_xml/stop_sign.xml')
    light_cascade = cv2.CascadeClassifier('cascade_xml/traffic_light.xml')

    d_to_camera = DistanceToCamera()
    d_stop_sign = 25
    d_light = 25

    stop_start = 0              # start time when stop at the stop sign
    stop_finish = 0
    stop_time = 0
    drive_time_after_stop = 0

    def handle(self):

        global sensor_data
        global prediction
        stream_bytes = ' '
        stop_flag = False
        stop_sign_active = True

        # stream video frames one by one
        try:

            while True:
 
                stream_bytes += self.rfile.read(1024)
                first = stream_bytes.find('\xff\xd8')
                last = stream_bytes.find('\xff\xd9')
                if first != -1 and last != -1:
                    jpg = stream_bytes[first:last+2]
                    stream_bytes = stream_bytes[last+2:]
                    gray = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), 0)
                    image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), -1)

                    global prediction
                    up_min_location=np.argmin(np.sum(gray[150:170, :],axis=0))    ###寻黑线
                    low_min_location=np.argmin(np.sum(gray[210:230, :],axis=0))    ###寻黑线
                    tmp = up_min_location - low_min_location                        ###寻黑线
                    # up_max_location=np.argmax(np.sum(gray[150:170, :],axis=0))    ###寻白线
                    # low_max_location=np.argmax(np.sum(gray[210:230, :],axis=0))    ###寻白线
                    # tmp = up_max_location - low_max_location                       ###寻白线
                    print ("tmp=",tmp)
                    if -40<tmp<40:
                        prediction = 0 #直行
                    elif -220<tmp<-40:
                        prediction = 1 #左转
                    elif 40<tmp<220:
                        prediction = 2 #右转
                    else:
                        prediction = 5 #后退

                    
                    # object detection
                    v_param1 = self.obj_detection.detect(self.stop_cascade, gray, image)
                    v_param2 = self.obj_detection.detect(self.light_cascade, gray, image)

                    # distance measurement
                    if v_param1 > 0 or v_param2 > 0:
                        d1 = self.d_to_camera.calculate(v_param1, self.h1, 300, image)
                        d2 = self.d_to_camera.calculate(v_param2, self.h2, 100, image)
                        self.d_stop_sign = d1
                        self.d_light = d2

                    cv2.imshow('image', image)

                    
                    # stop conditions
                    if sensor_data is not None and sensor_data < 35:
                        print("Stop, obstacle in front")
                        prediction = 6

                        
                    elif 0 < self.d_stop_sign < 35 and stop_sign_active:
                        print("Stop sign ahead")
                        prediction = 6

                        # stop for 5 seconds
                        if stop_flag is False:
                            self.stop_start = cv2.getTickCount()
                            stop_flag = True
                        self.stop_finish = cv2.getTickCount()

                        self.stop_time = (self.stop_finish - self.stop_start)/cv2.getTickFrequency()
                        print "Stop time: %.2fs" % self.stop_time

                        # 5 seconds later, continue driving
                        if self.stop_time > 5:
                            print("Waited for 5 seconds")
                            stop_flag = False
                            stop_sign_active = False

                    elif 0 < self.d_light < 30:
                        #print("Traffic light ahead")
                        if self.obj_detection.red_light:
                            print("Red light")
                            #self.rc_car.stop()
                            prediction = 6
                            #self.gpio.handle.stop()
                        elif self.obj_detection.green_light:
                            print("Green light")
                            pass
                        elif self.obj_detection.yellow_light:
                            print("Yellow light flashing")
                            pass
                        
                        self.d_light = 30
                        self.obj_detection.red_light = False
                        self.obj_detection.green_light = False
                        self.obj_detection.yellow_light = False

                    else:
                        time.sleep(0.05)
                        self.stop_start = cv2.getTickCount()
                        self.d_stop_sign = 35

                        if stop_sign_active is False:
                            self.drive_time_after_stop = (self.stop_start - self.stop_finish)/cv2.getTickFrequency()
                            if self.drive_time_after_stop > 5:
                                stop_sign_active = True

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        prediction = 6

            cv2.destroyAllWindows()


        finally:
            print "Connection closed on thread 1"

class GPIOhandler(SocketServer.BaseRequestHandler):    ###这一部分用于运动控制，功能是发出上一步得到的识别结果，使小车接收到指令后执行相应的子程序，做出转弯前进等动作
    data = " " 
    senddata = " "
    def handle(self):

        try:
            while True :
                global prediction
                self.data =prediction
                if self.data==0:
                    senddata = "runO"
                elif self.data==1:
                    senddata = "leftO"
                elif self.data==2:
                    senddata = "rightO"
                elif self.data==5:
                    senddata = "backO"
                else:
                    senddata = 'brakeO'

                self.request.send(bytes(senddata))

                print "prediction=",senddata.split('O')[0]

        finally:
            print "Connection closed on thread 3"



class ThreadServer(object):           ###多线程，在电脑和小车之间同时建立三个连接，三个线程动作同步进行。1.摄像头拍摄画面：小车→电脑  2.超声波数据：小车→电脑  3.运动指令：电脑→小车

    def server_thread(host, port):
        server = SocketServer.TCPServer((host, port), VideoStreamHandler)
        server.serve_forever()

    def server_thread2(host, port):
        server = SocketServer.TCPServer((host, port), SensorDataHandler)
        server.serve_forever()

    def server_thread3(host,port):
        server = SocketServer.TCPServer((host, port), GPIOhandler)
        server.serve_forever()

    GPIO_thread = threading.Thread(target=server_thread3, args=('192.168.43.61', 8004))
    GPIO_thread.start()

    distance_thread = threading.Thread(target=server_thread2, args=('192.168.43.61', 8002))
    distance_thread.start()

    video_thread = threading.Thread(target=server_thread('192.168.43.61', 8000))
    video_thread.start()

if __name__ == '__main__':
    try:
        ThreadServer()

    except KeyboardInterrupt:
        print "end"    

