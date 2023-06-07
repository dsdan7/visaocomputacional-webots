
import cv2
import numpy as np
from controller import Supervisor

robot = Supervisor()
timestep = 32
fruit = -1
laranja = 0
limao = 0
counter = 0
state = 0


target_positions = [-1.570796, -1.87972, -2.139774, -2.363176, -1.50971]


speed = 2


hand_motors = []
hand_motors.append(robot.getDevice('finger_1_joint_1'))
hand_motors.append(robot.getDevice('finger_2_joint_1'))
hand_motors.append(robot.getDevice('finger_middle_joint_1'))

ur_motors = []
ur_motors.append(robot.getDevice('shoulder_pan_joint'))
ur_motors.append(robot.getDevice('shoulder_lift_joint'))
ur_motors.append(robot.getDevice('elbow_joint'))
ur_motors.append(robot.getDevice('wrist_1_joint'))
ur_motors.append(robot.getDevice('wrist_2_joint'))


for i in range(5):
    ur_motors[i].setVelocity(speed)


distance_sensor = robot.getDevice('distance sensor')
distance_sensor.enable(timestep)


position_sensor = robot.getDevice('wrist_1_joint_sensor')
position_sensor.enable(timestep)


camera = robot.getDevice('camera')
camera.enable(timestep)


display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0xFFFFFF)
display.setFont('Impact', 16, True)





def resetDisplay():
    display.setAlpha(0.0)
    display.fillRectangle(0, 0, 200, 150)
    display.setAlpha(1.0)

def printDisplay(x, y, w, h, name):
    resetDisplay()
    display.drawRectangle(x, y, w, h)
    display.drawText(name, x - 2, y - 20)

def findFruit():
    min = []
    max = []
    cnts = []
    mask = []
    model = -1
   
    fname = ['Laranja', 'Limao']

    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    roi = img[0:150, 35:165]
    imHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    
    min.append(np.array([10, 135, 135], np.uint8))
    max.append(np.array([32, 255, 255], np.uint8))
    
   
    min.append(np.array([30, 50, 50], np.uint8))
    max.append(np.array([90, 255, 255], np.uint8))
    
   
    Kernel = np.ones((5, 5), np.uint8)
    
    for i in range(2):
        mask.append(cv2.inRange(imHSV, min[i], max[i]))
        mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_CLOSE, Kernel)
        mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_OPEN, Kernel)
        cnts.append(cv2.findContours(mask[i], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0])
        for c in cnts[i]:
            x, y, w, h = cv2.boundingRect(c)
            if w > 80:
                model = i
                printDisplay(x + 35, y, w, h, fname[i])
    return model


while robot.step(timestep) != -1:
    
    if counter <= 0:
        if state == 0: 
            fruit = findFruit()
            if distance_sensor.getValue() < 500:
                state = 1 
                if fruit == 0:
                    laranja += 1
                elif fruit == 1:
                    limao += 1
                counter = 8
                for i in range(3):
                    hand_motors[i].setPosition(0.52)

        elif state == 1: 
            for i in range(fruit, 5):
                ur_motors[i].setPosition(target_positions[i])
            state = 2 

        elif state == 2:
            if position_sensor.getValue() < -2.3:
                counter = 8
                state = 3 
                resetDisplay()
                for i in range(3):
                    hand_motors[i].setPosition(hand_motors[i].getMinPosition())

        elif state == 3: 
            for i in range(fruit, 5):
                ur_motors[i].setPosition(0.0)
            state = 4 # 

        elif state == 4: # 
            if position_sensor.getValue() > -0.1:
                state = 0 # 
            
    else:
        counter -= 1

    strP = f'Limao: {limao:3d}    Laranjas: {laranja:3d}'
    robot.setLabel(1, strP, 0.3, 0.96, 0.06,0xFFFFFF, 0, 'Lucida Console')

    pass

