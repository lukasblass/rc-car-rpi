#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO    
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class DirectionController:
  def __init__(self):
    self.current_dir = 0

    self.in_1 = 24
    self.in_2 = 23
    self.en = 25
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(self.in_1,GPIO.OUT)
    GPIO.setup(self.in_2,GPIO.OUT)
    GPIO.setup(self.en,GPIO.OUT)

    GPIO.output(self.in_1,GPIO.LOW)
    GPIO.output(self.in_2,GPIO.LOW)
    self.p = GPIO.PWM(self.en, 1000)
    
    self.p.start(100)
    self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback, queue_size=30)
    
    self.pub = rospy.Publisher("controllerchat", String, queue_size=30)

  def setMotorFromSteering(self, d):
    if d == self.current_dir:
      return
    elif d == 1:
      # left
      GPIO.output(self.in_1,GPIO.LOW)
      GPIO.output(self.in_2,GPIO.HIGH)
    elif d == -1:
      # right
      GPIO.output(self.in_1,GPIO.HIGH)
      GPIO.output(self.in_2,GPIO.LOW)
    else:
      # straight
      GPIO.output(self.in_1,GPIO.LOW)
      GPIO.output(self.in_2,GPIO.LOW)
    self.current_dir = d

  def callback(self, twist):
    steering = 0
    if twist.angular.z > 0:
      steering = -1
    elif twist.angular.z < 0:
      steering = 1
    self.setMotorFromSteering(steering)


class SpeedController:
  def __init__(self):
    self.current_vel = 0

    self.in_1 = 16
    self.in_2 = 20
    self.en = 21
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(self.in_1,GPIO.OUT)
    GPIO.setup(self.in_2,GPIO.OUT)
    GPIO.setup(self.en,GPIO.OUT)

    GPIO.output(self.in_1,GPIO.LOW)
    GPIO.output(self.in_2,GPIO.LOW)
    self.p = GPIO.PWM(self.en, 1000)
    
    self.p.start(100)
    self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback, queue_size=30)

  def setMotorFromVelocity(self, vel):
    if vel == self.current_vel:
      return
    elif vel == 1:
      # forward
      GPIO.output(self.in_1,GPIO.HIGH)
      GPIO.output(self.in_2,GPIO.LOW)
    elif vel == -1:
      # backward
      GPIO.output(self.in_1,GPIO.LOW)
      GPIO.output(self.in_2,GPIO.HIGH)
    else:
      # idle
      GPIO.output(self.in_1,GPIO.LOW)
      GPIO.output(self.in_2,GPIO.LOW)
    self.current_vel = vel

  def callback(self, twist):
    velocity = 0
    if twist.linear.x > 0:
      velocity = 1
    elif twist.linear.x < 0:
      velocity = -1
    self.setMotorFromVelocity(velocity)


    
if __name__ == '__main__':
  rospy.init_node('listener', anonymous=True)
  dir_ctrl = DirectionController()
  speed_ctrl = SpeedController()
  rospy.spin()