import sys

import pygame

from pygame.locals import *
pygame.init()
pygame.display.set_caption('game base')
screen = pygame.display.set_mode((500, 500), 0, 32)
clock = pygame.time.Clock()

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for joystick in joysticks:
    print(joystick.get_name())
    print(joystick.get_numaxes())
    print(joystick.get_numbuttons())

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


    # Axis 0: wheel
    # Axis 1: rightmost pedal
    # Axis 2: middle pedal
    # Axis 4: leftmost pedal

    # **Assumption that Logitech G27 joystick is the first one in the list**

    if (wheel_axis_val != joysticks[0].get_axis(0) or right_pedal_axis_val != joysticks[0].get_axis(1) or  middle_pedal_axis_val != joysticks[0].get_axis(2) or  left_pedal_axis_val != joysticks[0].get_axis(4)):
        print("Wheel axis value: " + str(joysticks[0].get_axis(0)))
        print("Rightmost pedal axis value: " + str(joysticks[0].get_axis(1)))
        print("Middle pedal axis value: " + str(joysticks[0].get_axis(2)))
        print("Leftmost pedal axis value: " + str(joysticks[0].get_axis(4)))
        wheel_axis_val = joysticks[0].get_axis(0)
        right_pedal_axis_val = joysticks[0].get_axis(1)
        middle_pedal_axis_val = joysticks[0].get_axis(2)
        left_pedal_axis_val = joysticks[0].get_axis(4)

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
                print(joystick.get_name())
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
