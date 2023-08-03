#!/usr/bin/env python

# Modified by Chitsein Htun
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

# Additional features added include data communication for motion simulator and joystick input control.

"""
Welcome to CARLA manual control.

*** KNOWN RESOLUTIONS THAT WORK*** -> 1024x720 works well and was previous default, 2880 x ANY, 2880-64a x 600. New default is 3840x720. 5760x1080 also works (Eleetus full res).

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit


Use the Platform's Steering Wheel and Pedals to control the vehicle

Buttons:
Press the top left red button on the steering wheel to change the car's gear to reverse
Press the top right red button on the steering wheel to toggle the platform input usage
Press bottom right red button to toggle the right mirror view
Press bottom right left button to toggle the left mirror view
Press middle right red button to toggle the reverse view
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import subprocess
from subprocess import Popen, CREATE_NEW_CONSOLE


# For accessing functions from DLL
import ctypes
from ctypes import *

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
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
    from pygame.locals import JOYBUTTONDOWN
    from pygame.locals import JOYBUTTONUP
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = [world.get_blueprint_library().find("vehicle.lincoln.mkz_2017")]
    return bps
    # bps = world.get_blueprint_library().filter(filter)

    # if generation.lower() == "all":
    #     return bps

    # # If the filter returns only one bp, we assume that this one needed
    # # and therefore, we ignore the generation
    # if len(bps) == 1:
    #     return bps

    # try:
    #     int_generation = int(generation)
    #     # Check if generation is in available generations
    #     if int_generation in [1, 2]:
    #         bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
    #         return bps
    #     else:
    #         print("   Warning! Actor Generation is not valid. No actor will be spawned.")
    #         return []
    # except:
    #     print("   Warning! Actor Generation is not valid. No actor will be spawned.")
    #     return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(get_actor_blueprints(self.world, self._actor_filter, self._actor_generation))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot, joystick, args):
        self._autopilot_enabled = start_in_autopilot
        self._ackermann_enabled = False
        self._ackermann_reverse = 1
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # Joystick additions
        self.joystick = joystick
        self.wheel_axis_val = 0
        self.right_pedal_axis_val = 1
        self.middle_pedal_axis_val = 1
        self.left_pedal_axis_val = 1
        self.wheel_sensitivity = float(args.wheel_sensitivity)
        self.joystick_in_use = True


    # Axis 0: wheel
    # Axis 1: rightmost pedal
    # Axis 2: middle pedal
    # Axis 4: leftmost pedal
    def get_joystick_input(self):
        if (self.wheel_axis_val != self.joystick.get_axis(0) or self.right_pedal_axis_val != self.joystick.get_axis(1) or  self.middle_pedal_axis_val != self.joystick.get_axis(2) or  self.left_pedal_axis_val != self.joystick.get_axis(4)):
            self.wheel_axis_val = self.joystick.get_axis(0)

            # ***Issues with getting pedals working, so commenting out these pedal axis vals for now***

            # self.right_pedal_axis_val = self.joystick.get_axis(1)
            # self.middle_pedal_axis_val = self.joystick.get_axis(2)
            # self.left_pedal_axis_val = self.joystick.get_axis(4)


    def _parse_joystick_input(self):
        
        # Temporary fix while pedals are broken
        # Top right red button on wheel controls acceleration
        # Top left white button on wheel controls brakes

        if self.joystick.get_button(3):
            self._control.throttle = min(self._control.throttle + 0.01, 1.00)
        else:
            self._control.throttle = 0

        if self.joystick.get_button(7):
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        # ***Issues with getting pedals working, so commenting out these pedal axis vals for now***

        # if self.right_pedal_axis_val < 0.99:
        #     self._control.throttle = (1 - self.right_pedal_axis_val)/2
        # else:
        #     self._control.throttle = 0
        
        # if self.middle_pedal_axis_val < 0.75:
        #     self._control.brake = (1 - self.middle_pedal_axis_val)/2
        # else:
        #     self._control.brake = 0

        self._steer_cache = self.wheel_sensitivity * self.wheel_axis_val
        self._control.steer = self._steer_cache


    def parse_events(self, client, world, clock, sync_mode):
        self.get_joystick_input()
        self._parse_joystick_input()
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_f:
                        # Toggle ackermann controller
                        self._ackermann_enabled = not self._ackermann_enabled
                        world.hud.show_ackermann_info(self._ackermann_enabled)
                        world.hud.notification("Ackermann Controller %s" %
                                               ("Enabled" if self._ackermann_enabled else "Disabled"))
                    if event.key == K_q:
                        if not self._ackermann_enabled:
                            self._control.gear = 1 if self._control.reverse else -1
                        else:
                            self._ackermann_reverse *= -1
                            # Reset ackermann control
                            self._ackermann_control = carla.VehicleAckermannControl()
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker

            elif event.type == JOYBUTTONUP:
                # Press bottom left blue button on steering wheel (button event #11) in order to put the car in reverse
                if event.button == 11:
                    self._control.gear = 1 if self._control.reverse else -1
                # Press top middle yellow button on steering wheel (button event #21) in order to toggle joystick usage
                elif event.button == 21:
                    self.joystick_in_use = False if self.joystick_in_use else True
                # Hold the right wing behind wheel (button event #4) in order to see right mirror
                elif event.button == 4:
                    print("Right mirror view toggled")
                    right_mirror_index = 2
                    if world.camera_manager.transform_index != right_mirror_index:
                        world.camera_manager.set_camera(right_mirror_index)
                    else:
                        world.camera_manager.set_camera(0)
                # Hold the left wing behind wheel (button event #5) in order to see left mirror
                elif event.button == 5:
                    print("Left mirror view toggled")
                    left_mirror_index = 3
                    if world.camera_manager.transform_index != left_mirror_index:
                        world.camera_manager.set_camera(left_mirror_index)
                    else:
                        world.camera_manager.set_camera(0)
                # Press bottom right grey button (button event #2) in order to see reverse view
                elif event.button == 2:
                    print("Reverse view toggled")
                    reverse_view = 4
                    if world.camera_manager.transform_index != reverse_view:
                        world.camera_manager.set_camera(reverse_view)
                    else:
                        world.camera_manager.set_camera(0)
            



        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                # Apply control
                if not self._ackermann_enabled:
                    world.player.apply_control(self._control)
                else:
                    world.player.apply_ackermann_control(self._ackermann_control)
                    # Update control to the last one applied by the ackermann controller.
                    self._control = world.player.get_control()
                    # Update hud with the newest ackermann control
                    world.hud.update_ackermann_control(self._ackermann_control)

            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
                world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            if not self._ackermann_enabled:
                self._control.throttle = min(self._control.throttle + 0.01, 1.00)
            else:
                self._ackermann_control.speed += round(milliseconds * 0.005, 2) * self._ackermann_reverse
        elif self.joystick_in_use != True:
            if not self._ackermann_enabled:
                self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            if not self._ackermann_enabled:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._ackermann_control.speed -= min(abs(self._ackermann_control.speed), round(milliseconds * 0.005, 2)) * self._ackermann_reverse
                self._ackermann_control.speed = max(0, abs(self._ackermann_control.speed)) * self._ackermann_reverse
        elif self.joystick_in_use != True:
            if not self._ackermann_enabled:
                self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if self.joystick_in_use != True:
            if keys[K_LEFT] or keys[K_a]:
                if self._steer_cache > 0:
                    self._steer_cache = 0
                else:
                    self._steer_cache -= steer_increment
            elif keys[K_RIGHT] or keys[K_d]:
                if self._steer_cache < 0:
                    self._steer_cache = 0
                else:
                    self._steer_cache += steer_increment
            self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
            if not self._ackermann_enabled:
                self._control.steer = round(self._steer_cache, 1)
                self._control.hand_brake = keys[K_SPACE]
            else:
                self._ackermann_control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                # Driver's seat - index 0
                (carla.Transform(carla.Location(x=-0.15*bound_x, y=-0.25*bound_y, z=1.0*bound_z)), Attachment.Rigid),
                # In front of car - index 1
                (carla.Transform(carla.Location(x=+1.0*bound_x, y=-0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                # Right Mirror - index 2
                (carla.Transform(carla.Location(x=0.4*bound_x, y=+1.0*bound_y, z=1.0*bound_z), carla.Rotation(yaw=180)), Attachment.Rigid),
                # Left Mirror - index 3
                (carla.Transform(carla.Location(x=0.4*bound_x, y=-1.0*bound_y, z=1.0*bound_z), carla.Rotation(yaw=180)), Attachment.Rigid),
                # Reverse View - index 4
                (carla.Transform(carla.Location(x=-0.6*bound_x, y=0.0*bound_y, z=1.0*bound_z), carla.Rotation(yaw=180)), Attachment.Rigid),
                # Other views
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
            ['sensor.camera.normals', cc.Raw, 'Camera Normals', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)
    
    def set_camera(self, index):
        if index < len(self._camera_transforms):
            self.transform_index = index
            self.set_sensor(self.index, notify=False, force_respawn=True)
        else:
            print("Index out of bounds for camera")

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- Motion Platform -------------------------------------------------------------
# ==============================================================================


class MotionPlatform(object):
    def __init__(self, args):
        # Load Eleetus Blue Tiger DLL into memory
        self.blue_tiger_api = ctypes.WinDLL(args.dll_path)

        self.blue_tiger_init = self.blue_tiger_api['BTInit']
        self.blue_tiger_init.argtypes = (
            ctypes.c_char_p, # Company name (null-terminated character string of up to 40 characters)
            ctypes.c_char_p, # Software product name (null-terminated character string of up to 40 characters)
            ctypes.c_char_p # Software version (null-terminated character string of up to 40 characters)
        )
        self.blue_tiger_init.restype = ctypes.c_ushort

        company = "Chitsein Htun - UNLV Smart Cities REU"
        product = "Manual Control - Eleetus"
        version = "v1.0"

        company_char_p = company.encode('utf-8')
        product_char_p = product.encode('utf-8')
        version_char_p = version.encode('utf-8')

        return_code = self.blue_tiger_init(ctypes.c_char_p(company_char_p), ctypes.c_char_p(product_char_p), ctypes.c_char_p(version_char_p))

        while return_code != 0 and args.no_motion != True:
            print("return code error: " + str(return_code))
            decision = input("There was an error with initializing the Blue Tiger Motion Platform. Would you like to continue without motion? (Y/N)")
            if decision.upper() == "Y":
                args.no_motion = True
                return
            elif decision.upper() == "N":
                return_code = self.blue_tiger_init(ctypes.c_char_p(company_char_p), ctypes.c_char_p(product_char_p), ctypes.c_char_p(version_char_p))

        if return_code == 0:
            print("Blue Tiger Initialization was successful.")

        # For calculating angular acceleration later on
        self.last_tick = None
        self.prev_rot_vel = None

        # Sensitivity parameters
        self.distance_sensitivity = float(args.distance_sensitivity)
        self.rotational_sensitivity = float(args.rotational_sensitivity)

        # Function parameters
        self.function_call_hertz = float(args.function_call_hertz)

        # ***DLL Function Definition Set Up***

        # Pitch Roll Data
        self.blue_tiger_pitch_roll_data = self.blue_tiger_api['BTPitchRollData']
        self.blue_tiger_pitch_roll_data.argtypes = (
            ctypes.c_float, # pitch
            ctypes.c_float # roll
        )
        self.blue_tiger_pitch_roll_data.restype = ctypes.c_ushort

        # Rotation Vector Data
        self.blue_tiger_pitch_rotation_vector_data = self.blue_tiger_api['BTRotationVectorData']
        self.blue_tiger_pitch_rotation_vector_data.argtypes = (
            ctypes.c_float, # xRotation
            ctypes.c_float, # yRotation
            ctypes.c_float, # zRotation
            ctypes.c_float # aRotation
        )
        self.blue_tiger_pitch_rotation_vector_data.restype = ctypes.c_ushort

        # Acceleration Data
        self.blue_tiger_acceleration_data = self.blue_tiger_api['BTAccelerationData']
        self.blue_tiger_acceleration_data.argtypes = (
            ctypes.c_float, # xAccel
            ctypes.c_float, # yAccel
            ctypes.c_float, # zAccel
            ctypes.c_float, # xRotAccel
            ctypes.c_float, # yRotAccel
            ctypes.c_float, # zRotAccel
            ctypes.c_float, # xForward
            ctypes.c_float, # yForward
            ctypes.c_float, # zForward
            ctypes.c_float, # xRight
            ctypes.c_float, # yRight
            ctypes.c_float # zRight
        )
        self.blue_tiger_acceleration_data.restype = ctypes.c_ushort

        # Pause
        self.blue_tiger_pause = self.blue_tiger_api['BTPause']
        self.blue_tiger_pause.argtypes = None
        self.blue_tiger_pause.restype = ctypes.c_ushort

        # Resume
        self.blue_tiger_resume = self.blue_tiger_api['BTResume']
        self.blue_tiger_resume.argtypes = None
        self.blue_tiger_resume.restype = ctypes.c_ushort

        # Shutdown
        self.blue_tiger_shutdown = self.blue_tiger_api['BTShutdown']
        self.blue_tiger_shutdown.argtypes = None
        self.blue_tiger_shutdown.restype = ctypes.c_ushort

        # Status
        self.blue_tiger_status = self.blue_tiger_api["BTStatus"]
        self.blue_tiger_status.argtypes = None
        self.blue_tiger_status.restype = ctypes.c_ushort


        self.last_function_call_time = None

        # More functions can be included from the API documentation folder. These are only the few functions that may be used within this program.

    def status(self):
        return self.blue_tiger_status()

    def pause(self):
        return_code = self.blue_tiger_pause()
        if return_code != 0:
            print("Failed to pause Eleetus Blue Tiger Motion Platform")

    def resume(self):
        return_code = self.blue_tiger_resume()
        if return_code != 0:
            print("Failed to resume Eleetus Blue Tiger Motion Platform")

    def shutdown(self):
        return_code = self.blue_tiger_shutdown()
        if return_code != 0:
            print("Failed to shutdown Eleetus Blue Tiger Motion Platform")
    
    def update_motion_platform(self, rotation, accel_vector, rot_velocity_vector, local_forward_vector, local_right_vector, time):

        # Important Information about world axis for Eleetus Blue Tiger Motion Platform API:
        # Forward vector -> A vector that defines the local negative z-axis in world coordinates.
        # Right vector   -> A vector that defines the local positive x-axis in world coordinates.
        # Up vector      -> A vector that defines the local positive y-axis in world coordinates.

        # In Carla, positive x is forward vector, positive y is right vector, and positive z is up vector

        # Angles are in radians -> Carla angles are in degrees, so must convert.
        pitch_raw = rotation.pitch * self.rotational_sensitivity * math.pi / 180
        yaw_raw = rotation.yaw * self.rotational_sensitivity * math.pi / 180
        roll_raw = rotation.roll * self.rotational_sensitivity * math.pi / 180

        if (pitch_raw > math.pi / 4):
            pitch_raw = math.pi / 4
        if (pitch_raw < -1 * math.pi / 4):
            pitch_raw = -1 * math.pi / 4
        if (yaw_raw > math.pi / 4):
            yaw_raw = math.pi / 4
        if (yaw_raw < -1 * math.pi / 4):
            yaw_raw = -1 * math.pi / 4
        if (roll_raw > math.pi / 4):
            roll_raw = math.pi / 4
        if (roll_raw < -1 * math.pi / 4):
            roll_raw = -1 * math.pi / 4 

        pitch = ctypes.c_float(pitch_raw)
        yaw = ctypes.c_float(yaw_raw)
        roll = ctypes.c_float(roll_raw)


        # Refer to API Documentation for more details
        
        # If first pass through, set the values
        if (self.last_function_call_time == None):
            self.last_function_call_time = time.get_ticks()
        if (self.last_tick == None):
            self.last_tick = time.get_ticks()
        if (self.prev_rot_vel == None):
            self.prev_rot_vel = rot_velocity_vector

        delta_t = time.get_ticks() - self.last_tick
        
        # Calculate rotational acceleration vector
        if delta_t > 0:
            xRotAccel_float = (rot_velocity_vector.x - self.prev_rot_vel.x) / delta_t
            yRotAccel_float = (rot_velocity_vector.y - self.prev_rot_vel.y) / delta_t
            zRotAccel_float = (rot_velocity_vector.z - self.prev_rot_vel.z) / delta_t
        else:
            xRotAccel_float = 0.0
            yRotAccel_float = 0.0
            zRotAccel_float = 0.0

        # Keep in mind that carla coordinate system is translated to BT coordinate system
        xAccel = ctypes.c_float(accel_vector.y * self.distance_sensitivity)
        yAccel = ctypes.c_float(accel_vector.z * self.distance_sensitivity)
        zAccel = ctypes.c_float(-1 * accel_vector.x * self.distance_sensitivity)
        xRotAccel = ctypes.c_float(yRotAccel_float * self.rotational_sensitivity)
        yRotAccel = ctypes.c_float(zRotAccel_float * self.rotational_sensitivity)
        zRotAccel = ctypes.c_float(-1 * xRotAccel_float * self.rotational_sensitivity)
        xForward = ctypes.c_float(local_forward_vector.y)
        yForward = ctypes.c_float(local_forward_vector.z)
        zForward = ctypes.c_float(-1 * local_forward_vector.x)
        xRight = ctypes.c_float(local_right_vector.y)
        yRight = ctypes.c_float(local_right_vector.z)
        zRight = ctypes.c_float(-1 * local_right_vector.x)


        # Call motion function every time period
        if time.get_ticks() - self.last_function_call_time > (1000 / self.function_call_hertz):
            status = self.blue_tiger_status()
            # print("pitch: " + str(pitch))
            # print("roll: " + str(roll))
            # print("Status code: " + str(status))
            # print("xAccel: " + str(xAccel))
            # print("yAccel: " + str(yAccel))
            # print("zAccel: " + str(zAccel))
            # print("xRotAccel: " + str(xRotAccel))
            # print("yRotAccel: " + str(yRotAccel))
            # print("zRotAccel: " + str(zRotAccel))
            # print("xForward: " + str(xForward))
            # print("yForward: " + str(yForward))
            # print("zForward: " + str(zForward))
            # print("xRight: " + str(xRight))
            # print("yRight: " + str(yRight))
            # print("zRight: " + str(zRight))
            self.last_function_call_time = time.get_ticks()

            # Using pitch roll data
            # return_code = self.blue_tiger_pitch_roll_data(pitch, roll)

            # Using acceleration data
            return_code = self.blue_tiger_acceleration_data(xAccel, yAccel, zAccel, xRotAccel, yRotAccel, zRotAccel, xForward, yForward, zForward, xRight, yRight, zRight)

            if return_code != 0:
                print("Failed to move Eleetus Blue Tiger Motion Platform")
        

        # Store the last time and rotational velocity values
        self.last_tick = time.get_ticks()
        self.prev_rot_vel = rot_velocity_vector

        



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
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        # for joystick in joysticks:
        #     print(joystick.get_name())

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        # **Assumption that Logitech G27 joystick is the first one in the list**
        controller = KeyboardControl(world, args.autopilot, joysticks[0], args)
        motion_platform = MotionPlatform(args)
        if args.no_motion:
            motion_platform = None
        


        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        time = pygame.time
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock, args.sync):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            if args.no_motion == False:
                motion_platform.update_motion_platform(
                world.player.get_transform().rotation,
                world.player.get_acceleration(),
                world.player.get_angular_velocity(),
                world.player.get_transform().get_forward_vector(),
                world.player.get_transform().get_right_vector(),
                time
                )

    finally:

        try:
            motion_platform.shutdown()
            print("motion platform shutdown successfully.")
        except:
            print("motion wasn't activated.")

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


import csv
import os


    


   




# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():

    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--config_path',
        metavar='config_path',
        default="null",
        type=str,
        help='config_path set automatically to simcraft_config.txt in current working directory')
    argparser.add_argument(
        '--dll_path',
        metavar='dll_path',
        default="null",
        type=str,
        help='dll_path set automatically to BTApi_x64.dll in current working directory')
    argparser.add_argument(
        '--no_motion',
        action='store_true',
        help='start driving simulation without motion platform capabilities')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='3840x720',
        help='window resolution (default: 3840x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    print()

    # If not specified by user, configuration file is defined as config_eleetus.txt within the cwd
    if args.config_path == "null":
        args.config_path = os.getcwd() + "\\config_eleetus.txt"

    try:
        config = open(args.config_path, 'r').readlines()

        for var in config:
            var = var.strip("\n")
            input_sense_var_index = var.find("wheel_sensitivity=")
            distance_sense_var_index = var.find("distance_sensitivity=")
            rotation_sense_var_index = var.find("rotational_sensitivity=")
            function_call_hertz_index = var.find("function_call_hertz=")
            dll_path_index = var.find("dll_path=")
            
            if dll_path_index != -1:
                args.dll_path = var[dll_path_index + len("dll_path="):]
                print("Using platform motion .dll path from configuration file: " + args.dll_path)

            if input_sense_var_index != -1:
                args.wheel_sensitivity = var[input_sense_var_index + len("wheel_sensitivity="):]
                print("Using wheel sensitivity from configuration file: " + args.wheel_sensitivity)
                try:
                    float(args.wheel_sensitivity)
                except:
                    print("Wheel sensitivity is not configured correctly. Make sure that it is only a float value.")
                    args.wheel_sensitivity = 0.25
                if float(args.wheel_sensitivity) <= 0:
                    print("Wheel sensitivity value is not valid. Please choose one that is greater than 0. Setting wheel sensitivity to default (0.25)")
                    args.wheel_sensitivity = 0.25
            else:
                args.wheel_sensitivity = 0.25

            if distance_sense_var_index != -1:
                args.distance_sensitivity = var[distance_sense_var_index + len("distance_sensitivity="):]
                print("Using distance sensitivity from configuration file: " + args.distance_sensitivity)
                try:
                    float(args.distance_sensitivity)
                except:
                    print("Distance sensitivity is not configured correctly. Make sure that it is only a float value.")
                    args.distance_sensitivity = 0.5
                if float(args.distance_sensitivity) <= 0:
                    print("Distance sensitivity value is not valid. Please choose one that is greater than 0. Setting distance sensitivity to default (0.5)")
                    args.distance_sensitivity = 0.5
                if float(args.distance_sensitivity) > 5:
                    print("Warning! Distance sensitivity value is set too high! Please choose one that is less than 5. Setting distance sensitivity to default (0.5)")
                    args.distance_sensitivity = 0.5
            else:
                args.distance_sensitivity = 0.5

            if rotation_sense_var_index != -1:
                args.rotational_sensitivity = var[rotation_sense_var_index + len("rotational_sensitivity="):]
                print("Using rotational sensitivity from configuration file: " + args.rotational_sensitivity)
                try:
                    float(args.rotational_sensitivity)
                except:
                    print("Rotational sensitivity is not configured correctly. Make sure that it is only a float value.")
                    args.rotational_sensitivity = 75.0
                if float(args.rotational_sensitivity) <= 0:
                    print("Rotational sensitivity value is not valid. Please choose one that is greater than 0. Setting rotational sensitivity to default (75.0)")
                    args.rotational_sensitivity = 75.0
                if float(args.distance_sensitivity) > 100:
                    print("Warning! Rotational sensitivity value is set too high! Please choose one that is less than 100. Setting distance sensitivity to default (75.0)")
                    args.distance_sensitivity = 75.0
            else:
                args.rotational_sensitivity = 75.0

            if function_call_hertz_index != -1:
                args.function_call_hertz = var[function_call_hertz_index + len("function_call_hertz="):]
                print("Using function call hertz from configuration file: " + args.function_call_hertz)
                try:
                    float(args.function_call_hertz)
                except:
                    print("Function call hertz is not configured correctly. Make sure that it is only a float value.")
                    args.function_call_hertz = 20.0
                if float(args.function_call_hertz) <= 0:
                    print("Function call hertz value is not valid. Please choose one that is greater than 0. Setting function call hertz to default (20.0)")
                    args.function_call_hertz = 20.0
            else:
                args.function_call_hertz = 20.0

    except:
        print("Configuration file not found. Continuing with default values.")

    print()

    # If not specified by user or config file, platform motion .dll path is defined as BTApi_x64.dll within the cwd
    if args.dll_path == "null":
        args.dll_path = os.getcwd() + "\\BTApi_x64.dll"



    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    
    pygame.init()
    pygame.display.set_caption('game base')
    screen = pygame.display.set_mode((500, 500), 0, 32)
    clock = pygame.time.Clock()

    try:
        game_loop(args)


    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
