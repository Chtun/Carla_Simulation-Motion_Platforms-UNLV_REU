import sys

import pygame

from pygame.locals import *


import pyglet

# get a list of all low-level input devices:
devices = pyglet.input.get_devices()

print("\nDevices: ")
for device in devices:
    print(device.name)


# get a list of all joysticks:
joysticks_pyglet = pyglet.input.get_joysticks()

print("\nJoysticks: ")
for joystick in joysticks_pyglet:
    print(joystick.device.name)
    for control in joystick.device.get_controls():
        print(str(control))
print()

pedals = joysticks_pyglet[0]
pedals.open()
print(pedals.device.name)


pygame.init()
pygame.display.set_caption('game base')
screen = pygame.display.set_mode((500, 500), 0, 32)
clock = pygame.time.Clock()

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
print(str(joysticks))
for joystick in joysticks:
    print(joystick.get_name())
    print(joystick.get_numaxes())
    print(joystick.get_numbuttons())
    print()

steering_wheel = joysticks[0]

my_square = pygame.Rect(50, 50, 50, 50)
my_square_color = 0
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
motion = [0, 0]

wheel_axis_val = 0.0
right_pedal_axis_val = 0.0
middle_pedal_axis_val = 0.0
left_pedal_axis_val = 0.0

while True:

    screen.fill((0, 0, 0))

    pygame.draw.rect(screen, colors[my_square_color], my_square)
    if abs(motion[0]) < 0.1:
        motion[0] = 0
    if abs(motion[1]) < 0.1:
        motion[1] = 0
    my_square.x += motion[0] * 10
    my_square.y += motion[1] * 10


    # Pygame -> Axis 0: wheel
    # Pyglet -> x: rightmost pedal
    # Pyglet -> y: middle pedal
    # Pyglet -> z: leftmost pedal


    if (wheel_axis_val != steering_wheel.get_axis(0) or right_pedal_axis_val != pedals.x_control.value or  middle_pedal_axis_val != pedals.y_control.value or  left_pedal_axis_val != pedals.z_control.value):
        # Steering wheel is through pygame, pedals is through pyglet
        print("Wheel axis value: " + str(steering_wheel.get_axis(0)))
        print("Rightmost pedal axis value: " + str(pedals.x_control.value))
        print("Middle pedal axis value: " + str(pedals.y_control.value))
        print("Leftmost pedal axis value: " + str(pedals.z_control.value))
        wheel_axis_val = steering_wheel.get_axis(0)
        right_pedal_axis_val = pedals.x_control.value
        middle_pedal_axis_val = pedals.y_control.value
        left_pedal_axis_val = pedals.z_control.value

    for event in pygame.event.get():
        print(event)
        # if event.type == JOYBUTTONDOWN:
        #     print(event)
        #     if event.button == 0:
        #         my_square_color = (my_square_color + 1) % len(colors)
        # if event.type == JOYBUTTONUP:
        #     print(event)
        # if event.type == JOYAXISMOTION:
        #     print(event)
        #     if event.axis < 2:
        #         motion[event.axis] = event.value
        # if event.type == JOYHATMOTION:
        #     print(event)
        if event.type == JOYDEVICEADDED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            for joystick in joysticks:
                print("Device added: " + joystick.get_name())
        if event.type == JOYDEVICEREMOVED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                pygame.quit()
                sys.exit()

    pygame.display.update()
    clock.tick(60)
