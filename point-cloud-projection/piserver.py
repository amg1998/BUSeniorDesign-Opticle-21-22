import socket
import RPi.GPIO as GPIO

#setup PI
GPIO.setmode(GPIO.BOARD)
#motor1
GPIO.setup(8,GPIO.OUT)
pwm = GPIO.PWM(14, 100)
pwm.start(0)

HOST_IP =  ''
PORT = ''
BUFFER_SIZE = 20

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')

s.bind((HOST_IP, PORT))
s.listen(True)

conn, addr = s.accept()
print('Connection address:', addr)
while True:
    data = conn.recv(BUFFER_SIZE)
    if data == '1':
        pwm.ChangeDutyCycle(100)
    else:
        pwm.ChangeDutyCycle(0)
