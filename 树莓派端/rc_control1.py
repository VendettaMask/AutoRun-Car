#coding=utf-8
import socket
from socket import *
import RPi.GPIO as GPIO
import time


IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13
GPIO.setmode(GPIO.BCM)    #设置GPIO口为BCM编码方式
GPIO.setwarnings(False)   #忽略警告信息    
t1 = 0.03


class rpiGPIOHelper(object):
    
    def __init__(self):
        global pwm_ENA
        global pwm_ENB
        global delaytime
        GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
        #设置pwm引脚和频率为2000hz
        pwm_ENA = GPIO.PWM(ENA, 2000)
        pwm_ENB = GPIO.PWM(ENB, 2000)
        pwm_ENA.start(0)
        pwm_ENB.start(0)       

    #小车右转
    def right(self,delaytime):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        print "pi car right."
        time.sleep(delaytime)

    #小车左转   
    def left(self,delaytime):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        print "pi car left."
        time.sleep(delaytime)

    #小车前进
    def run(self,delaytime):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        print "pi car forwarding."
        time.sleep(delaytime)

    #小车后退
    def back(self,delaytime):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        print "pi car backward"
        time.sleep(delaytime)   

       
    #小车停止   
    def brake(self,delaytime):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(30)
        pwm_ENB.ChangeDutyCycle(30)
        time.sleep(delaytime)


# ============socket================ #
client_socket = socket(AF_INET, SOCK_STREAM)
client_socket.connect(('192.168.43.61', 8004))
# ============socket================ #


try:
    gpio_helper = rpiGPIOHelper()
    while True:
        pre_data = client_socket.recv(1024)
        print "pre_data=====================",pre_data
        data = pre_data.split('O')[0]
        
        if not data: continue

        if data=="run":
            gpio_helper.run(0.03)
        elif data=="left":
            gpio_helper.left(0.03)
        elif data=="right":
            gpio_helper.right(0.03)
        elif data=="back":
            gpio_helper.back(0.03)
        else:
            data = "brake"
            gpio_helper.brake(0.03)
            print 555555555555555555555555555555555555555           

        client_socket.sendall(data + " had recvied!")

except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()
