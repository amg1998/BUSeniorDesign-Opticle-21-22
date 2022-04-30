import socket
import RPi.GPIO as GPIO

#setup PI
GPIO.setmode(GPIO.BOARD)
#motor1
GPIO.setup(8,GPIO.OUT)
pwm = GPIO.PWM(8, 100)
pwm.start(0)

HOST_IP = '172.20.10.11'
PORT = 2000
BUFFER_SIZE = 20

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')

try:
    s.bind((HOST_IP, PORT))
except socket.error:
    print('Bind failed')

s.listen(1)
print('Socket awaiting messages')
conn, addr = s.accept()
print('Connection address:', addr)
while True:
    data = conn.recv(BUFFER_SIZE)
    if data == bytes('1','utf-8'):
        pwm.ChangeDutyCycle(100)
        print('Vibrating')
    else:
        pwm.ChangeDutyCycle(0)
        print('Not Vibrating')