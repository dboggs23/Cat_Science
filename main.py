import RPi.GPIO as GPIO
from picamera import PiCamera
import time
from time import sleep
from random import randint
import cv2
import os
import wiringpi

GPIO.setmode(GPIO.BOARD)
Ultrasonic_trigger_pin = 7 #Trigger
Ultrasonic_echo_pin = 11 #Echo pin

GPIO.setup(Ultrasonic_trigger_pin, GPIO.OUT)
GPIO.setup(Ultrasonic_echo_pin, GPIO.IN)

# use 'GPIO naming'
wiringpi.wiringPiSetupGpio()

# set #18 to be a PWM output
wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)

# set the PWM mode to milliseconds stype
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

#load face cascade
faceCascade = cv2.CascadeClassifier("/home/pi/haarcascade_frontalface_default.xml")



# divide down clock
wiringpi.pwmSetClock(192)
wiringpi.pwmSetRange(2000)

while True:
    print('running')
    
    GPIO.output(Ultrasonic_trigger_pin, GPIO.HIGH)

    time.sleep(0.00001)

    GPIO.output(Ultrasonic_trigger_pin, GPIO.LOW)
    
    time.sleep(2)

    GPIO.output(Ultrasonic_trigger_pin, GPIO.HIGH)

    time.sleep(0.00001)

    GPIO.output(Ultrasonic_trigger_pin, GPIO.LOW)

    while GPIO.input(Ultrasonic_echo_pin)==0:
        pulse_start_time = time.time()
    while GPIO.input(Ultrasonic_echo_pin)==1:
        pulse_end_time = time.time()

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * 17150, 2)
    
    print ("Distance:",distance,"cm")
    
    if distance < 100:
        #generate random number to concat on the end of file name
        numberID = str(randint(1000, 100000))
        photoID = '/home/pi/Pictures/Cat_Faces/cat_face_{}.jpg'.format(numberID)
        
        camera = PiCamera()
        camera.rotation = 180
        camera.resolution = (1024, 768)
        camera.start_preview()
        sleep(2)
        camera.capture(photoID)
        camera.stop_preview()
        camera.close()
        image = cv2.imread(photoID)

        # Convert into grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Look for faces in the image using the loaded cascade file
        faces = faceCascade.detectMultiScale(gray, 1.2, 5)
        
        if len(faces) > 0:
            for (x,y,w,h) in faces:
                # Create rectangle around faces
                cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
                
            # Create the resizeable window
            cv2.namedWindow('Kitty', cv2.WINDOW_NORMAL)

            # Display the image
            cv2.imshow('Kitty', image)
            
            delay_period = 0.01
            
            for pulse in range(50, 250, 1):
                    wiringpi.pwmWrite(18, pulse)
                    time.sleep(delay_period)
            for pulse in range(250, 50, -1):
                    wiringpi.pwmWrite(18, pulse)
                    time.sleep(delay_period)
                    
            sleep(30)
            #close window and delete picture
            
    sleep(5)
    
GPIO.cleanup()
            
            
        
        
    

