#!/usr/bin/env python

# Created by Chitsein Htun

"""
Visualization of Trajectory Predictions of Agents within the Carla World
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import math

try:
    import pygame
    from pygame import *
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# Predefined some colors
BLUE  = (0, 0, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
LIGHT_GREY = (180, 180, 180)
MEDIUM_GREY = (128, 128, 128)
DARK_GREY = (64, 64, 64)

# Preset values
FPS = 60
FramePerSec = pygame.time.Clock()


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================



def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- Surrounding Objects ---------------------------------------------------------------------
# ==============================================================================


class StaticObject(pygame.sprite.Sprite):
    def __init__(self, global_position_x, global_position_y, width, height, rotation, args):
        super().__init__()
        self.global_position_x = global_position_x
        self.global_position_y = global_position_y
        self.local_position_x = 0
        self.local_position_y = 0
        self.rect = pygame.Rect((0, 0), (width, height))
        self.SCREEN_WIDTH = args.width
        self.SCREEN_HEIGHT = args.height
        self.surface = pygame.Surface((width, height))
        self.surface.set_colorkey(BLACK)
        self.rotation = rotation

    def update(self, ego_position_x, ego_position_y, args):
        self.local_position_x = int((self.global_position_x - ego_position_x) * args.scale_multiplier + self.SCREEN_WIDTH/2)
        self.local_position_y = int((self.global_position_y - ego_position_y) * args.scale_multiplier + self.SCREEN_HEIGHT/2)
            

    def draw(self, ego_rotation):
        self.surface.fill(DARK_GREY)
        rotated_surface = pygame.transform.rotate(self.surface, ego_rotation - self.rotation)
        return rotated_surface

class DynamicObject(pygame.sprite.Sprite):
    def __init__(self, global_position_x, global_position_y, width, height, rotation, args):
        super().__init__()
        self.global_position_x = int(global_position_x)
        self.global_position_y = int(global_position_y)
        self.local_position_x = 0
        self.local_position_y = 0
        self.rotation = rotation
        self.width = width
        self.height = height
        self.rect = pygame.Rect((0,0), (int(width), int(height)))
        self.trajectory = (2, 60)
        self.trajectory_width = args.trajectory_thickness
        self.SCREEN_WIDTH = args.width
        self.SCREEN_HEIGHT = args.height
        self.surface = pygame.Surface((int(width), int(height)))
        self.surface.set_colorkey(BLACK)
        self.traj_surface = pygame.Surface((args.width, args.height))
        self.traj_surface.set_colorkey(BLACK)
        

    def update(self, ego_position_x, ego_position_y, args):
        self.local_position_x = int((self.global_position_x - ego_position_x) * args.scale_multiplier + self.SCREEN_WIDTH/2 - self.width/4)
        self.local_position_y = int((self.global_position_y - ego_position_y) * args.scale_multiplier + self.SCREEN_HEIGHT/2 - self.height/4)
            

    def draw(self, ego_rotation):
        # Car Body
        self.surface.fill(GREEN)
        rotated_surface = pygame.transform.rotate(self.surface, ego_rotation - self.rotation)
        return rotated_surface

    def draw_trajectory(self):
        # Predicted Trajectory
        if (self.trajectory[0] != 0 or self.trajectory[1] != 0):
            pygame.draw.line(self.traj_surface, WHITE, (self.local_position_x + self.width/4, self.local_position_y + self.height), (self.local_position_x + self.width/4 + self.trajectory[0], self.local_position_y + self.height + self.trajectory[1]), self.trajectory_width)
        return self.traj_surface



# ==============================================================================
# -- Ego ---------------------------------------------------------------------
# ==============================================================================


class Ego(pygame.sprite.Sprite):
    def __init__(self, width, height, args):
        super().__init__()
        self.rect = pygame.Rect((0, 0), (int(width), int(height)))
        self.width = width
        self.height = height
        self.local_position_x = int(args.width/2 - self.width/2)
        self.local_position_y = int(args.height/2 - self.height/2)
        self.trajectory = (0, 0)
        self.trajectory_width = args.trajectory_thickness
        self.surface = pygame.Surface((int(width), int(height)))
        self.surface.set_colorkey(BLACK)
        self.traj_surface = pygame.Surface((args.width, args.height))
        self.traj_surface.set_colorkey(BLACK)

    def update(self):
         self.trajectory = (-5, -80)

    def draw(self):
        # Car Body
        self.surface.fill(RED)
        return self.surface

    def draw_trajectory(self):
        # Predicted Trajectory
        if (self.trajectory[0] != 0 or self.trajectory[1] != 0):
            pygame.draw.line(self.traj_surface, WHITE, (self.local_position_x + self.width/2, self.local_position_y + self.height/2), (self.local_position_x + self.width/2 + self.trajectory[0], self.local_position_y + self.height/2 + self.trajectory[1]), self.trajectory_width)
        return self.traj_surface

        

class RoadLines(pygame.sprite.Sprite):
    def __init__(self, line_width, world, args):
        super().__init__()
        self.local_position_x = 0
        self.local_position_y = 0
        self.line_width = line_width
        self.surface = pygame.Surface((args.width, args.height))
        self.surface.set_colorkey(BLACK)
        self.world = world
        self.all_waypoints = args.map.generate_waypoints(1)

    def update(self):
        self.surface.set_colorkey(BLACK)
        

    def draw(self, args):
        ego_waypoint = args.map.get_waypoint(self.world.get_actor(args.ego_vehicle_id).get_location())
        for waypoint in self.all_waypoints:
            position = ((int((waypoint.transform.location.x - ego_waypoint.transform.location.x) * args.scale_multiplier + args.width/2)), int(((waypoint.transform.location.y - ego_waypoint.transform.location.y) * args.scale_multiplier + args.height/2)))
            pygame.draw.circle(self.surface, LIGHT_GREY, position, self.line_width)
        
        print("Points generated.")

        return self.surface
        



# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()

        display = pygame.display.set_mode((args.width, args.height))
        display.fill((0,0,0))
        pygame.display.flip()

        try:
            args.map = sim_world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()
        
        default_position = list((20, 20))
        default_rotation = 0
        args.ego_vehicle_id = None
        found_ego = False
        vehicles = list()
        try:
            print("Actors:")
            for actor in sim_world.get_actors():
                print(actor.type_id)
            print()

            for actor in sim_world.get_actors():
                if (actor.type_id == "vehicle.lincoln.mkz_2017" and found_ego == False):
                    args.ego_vehicle_id = actor.id
                    found_ego = True
                elif "vehicle" in actor.type_id:
                    if (actor.bounding_box.extent.y <= 0 or actor.bounding_box.extent.x <= 0):
                        continue
                    vehicles.append(DynamicObject(actor.get_location().x,
                                                  actor.get_location().y,
                                                  int(actor.bounding_box.extent.x * 2 * args.scale_multiplier),
                                                  int(actor.bounding_box.extent.y * 2 * args.scale_multiplier),
                                                  actor.get_transform().rotation.yaw,
                                                  args))
                    

            if args.ego_vehicle_id == None:
                print("Couldn't find the Lincoln MKZ 2017 vehicle. Ego vehicle position will be set to a traffic light cam position.")
        except:
            print("Failed to get actors. Ego vehicle position will be set to a traffic light cam position.")

        if args.ego_vehicle_id == None:
                ego_position = default_position
                ego_rotation = default_rotation
        else:
            ego_position = list((int(sim_world.get_actor(args.ego_vehicle_id).get_location().x), int(sim_world.get_actor(args.ego_vehicle_id).get_location().y)))
            ego_rotation = sim_world.get_actor(args.ego_vehicle_id).bounding_box.rotation.yaw # Birds eye view rotation
            ego_vehicle = Ego(int(sim_world.get_actor(args.ego_vehicle_id).bounding_box.extent.y * 2 * args.scale_multiplier),
                              int(sim_world.get_actor(args.ego_vehicle_id).bounding_box.extent.x * 2 * args.scale_multiplier),
                              args)

        print("Ego vehicle id: " + str(args.ego_vehicle_id))
        print("Ego rotation: " + str(ego_rotation))
        print("Ego location: " + str(ego_position))
        print("Ego vehicle size:\nx: " + str(sim_world.get_actor(args.ego_vehicle_id).bounding_box.extent.y * 2 * args.scale_multiplier) + " y: " + str(sim_world.get_actor(args.ego_vehicle_id).bounding_box.extent.x * 2 * args.scale_multiplier))


        

        road_lines = RoadLines(4, sim_world, args)
        final_road_lines = road_lines.draw(args)
        

        clock = pygame.time.Clock()
        time = pygame.time

        while True:     
            for event in pygame.event.get():              
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()

            # Update ego position based on Carla
            

            ego_vehicle.update()
            road_lines.update()
            for vehicle in vehicles:
                vehicle.update(ego_position[0], ego_position[1], args)
            
            display.fill(BLACK)

            rotated_ego = ego_vehicle.draw()
            ego_trajectory = ego_vehicle.draw_trajectory()
            rotated_vehicles = list()
            vehicle_trajectories = list()
            for vehicle in vehicles:
                rotated_vehicles.append(vehicle.draw(ego_rotation))
                vehicle_trajectories.append(vehicle.draw_trajectory())



            display.blit(final_road_lines, (road_lines.local_position_x, road_lines.local_position_y))
            for i in range(len(rotated_vehicles)):
                display.blit(rotated_vehicles[i], (vehicles[i].local_position_x, vehicles[i].local_position_y))
                display.blit(vehicle_trajectories[i], (0, 0))
            display.blit(rotated_ego, (ego_vehicle.local_position_x, ego_vehicle.local_position_y))
            display.blit(ego_trajectory, (0, 0))
            
            
        
            pygame.display.update()
            FramePerSec.tick(FPS)

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        pygame.quit()
   




# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():

    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--traj_thickness',
        metavar='trajectory_thickness',
        default='10',
        help='trajectory thickness (default: 10)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]
    args.trajectory_thickness = int(args.traj_thickness)

    args.scale_multiplier = 20

    print()



    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    
    pygame.init()
    pygame.display.set_caption('game base')
    pygame.display.set_mode((500, 500), 0, 32)
    clock = pygame.time.Clock()

    try:
        game_loop(args)


    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()