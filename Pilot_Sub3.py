#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control with steering wheel Logitech G29.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    
    # agents 폴더 경로를 PYTHONPATH에 추가
    current_dir = os.path.dirname(os.path.abspath(__file__))
    carla_path = os.path.join(os.path.dirname(current_dir), 'carla')

    # CARLA agents 경로 찾기
    possible_paths = [
        carla_path,  # 현재 디렉토리 기준
        '/home/somin/CARLA_0.9.14/PythonAPI/carla',  # 절대 경로
        os.path.join(os.path.expanduser('~'), 'CARLA_0.9.14', 'PythonAPI', 'carla'),
    ]

    agents_found = False
    for path in possible_paths:
        agents_check = os.path.join(path, 'agents')
        if os.path.exists(agents_check):
            sys.path.insert(0, path)
            print(f"agents 경로 찾음: {agents_check}")
            agents_found = True
            break

    if not agents_found:
        print("WARNING: agents 폴더를 찾을 수 없습니다.")
        print("다음 경로들을 확인했습니다:")
        for path in possible_paths:
            print(f"  - {os.path.join(path, 'agents')}")

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
import csv, pathlib, datetime
from collections import deque

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

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
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    from agents.navigation.basic_agent import BasicAgent
    print("GlobalRoutePlanner가 성공적으로 import되었습니다.")
except ImportError as e:
    print(f"Import 오류: {e}")
    print("현재 Python 경로:")
    for path in sys.path:
        print(f"  - {path}")

    # 직접 경로 지정해서 다시 시도
    carla_agents_path = '/home/somin/CARLA_0.9.14/PythonAPI/carla'
    if carla_agents_path not in sys.path:
        sys.path.insert(0, carla_agents_path)
        try:
            from agents.navigation.global_route_planner import GlobalRoutePlanner
            from agents.navigation.basic_agent import BasicAgent
            print("agents를 직접 경로로 import했습니다 (GRP/BasicAgent).")
        except ImportError:
            raise RuntimeError('agents 모듈을 찾을 수 없습니다. CARLA 설치를 확인하세요.')
    else:
        raise RuntimeError('agents 모듈을 찾을 수 없습니다.')

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


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, actor_filter):
        self.world = carla_world
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.controller = None  # will be set from game_loop

        # 내비게이션 관련 변수 추가
        self.route_planner = None
        self.route = []
        self.current_route_index = 0
        self.destination_set = False
        self.nav_direction = "Continue"

        # 내비게이션 HUD 표시용 변수
        self.nav_distance = 0.0
        self.nav_waypoint_info = "0/0"
        self.route_points = []   # 경로 시각화용 포인트
                
        # 지정된 좌표
        self.spawn_location = carla.Location(x=335.1, y=5.3, z=1.5)
        self.destination_location = carla.Location(x=394.5, y=240.2, z=0.5)
        
        # 멀티-스톱 경로용 변수
        self._primary_destination = self.destination_location  # 기존 목적지 백업
        self._via3_location = carla.Location(x=153.9, y=33.8, z=0.5)  # 비가 그치는 지점(via3)

        self.restart()
        self.world.on_tick(hud.on_world_tick)

        # Initial weather setting (no rain)
        clear_weather = carla.WeatherParameters.ClearNoon
        clear_weather.precipitation = 0.0
        clear_weather.precipitation_deposits = 0.0
        clear_weather.cloudiness = 20.0
        self.world.set_weather(clear_weather)

        self.bicycle = None
        # Bicycle spawn location (fixed coordinates) - changed to via point location
        self.bicycle_spawn_location = carla.Location(x=396.4, y=250.5, z=0.5)
        
        # Bicycle distance parameters (adjustable)
        self.bicycle_spawn_trigger_distance = 45.0  # Spawn bicycle when player comes within this distance
        self.bicycle_start_distance = 25.0          # Start bicycle movement when player comes within this distance
        
        # Initialize other vehicles
        self.other_vehicles = []
        self.stop_signs = []
        
        self.truck = None
        self.truck_agent = None
        self.truck_stage = 0  # 0: via[0] → 1: via[1] → 2: final → 3: done

        self.truck_vias = [
            carla.Location(x=335.2, y=66.8,  z=0.5),   # via1
            carla.Location(x=317.4, y=130.9, z=0.5),   # via2
            carla.Location(x=153.9, y=33.8,  z=0.5),   # via3 (여기서 비 멈춤)
            carla.Location(x=109.9, y=58.0,  z=0.5),   # via4
        ]
        self.truck_final_location = carla.Location(x=335.1, y=350.0, z=0.5)
        
        # Traffic light timing (seconds) - change here to apply globally
        self.light_green  = 8.0
        self.light_yellow = 2.0
        self.light_red    = 2.0   # reduced from 4.0 to 2.0 (shorter)
        
        self._tl_last = {}  # {light.id: (state, changed_at)}
        
        # Spawn truck ahead of vehicle at the same time
        self.spawn_truck_ahead()
        
        # Initialize traffic light control (shorter timing)
        self.setup_traffic_lights()

        # --- Stop task tracking state variables ---
        self._stop_task_start_time = None          # simulation timestamp when trigger first occurred
        self._traffic_prev_state_num = -1          # previous traffic light state (0/1/2/-1)
        self._saw_green_since_last_red = False     # whether Green was seen since last Red
        self._front_vehicle_prev_speed = {}        # {actor_id: prev_speed_mps}
        self._front_stop_event_t_sim = None        # 'occurrence time' of front vehicle sudden stop (simulation ts single point)
        self._lead_vehicle_id = None               # current lead vehicle id (if exists)

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = self.world.get_blueprint_library().find("vehicle.audi.tt")
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            self.destroy()
        # Spawn at fixed location
        fixed_spawn = carla.Transform(
            carla.Location(x=335.1, y=5.3, z=1.5),
            carla.Rotation(yaw=90.0)
        )
        self.player = self.world.try_spawn_actor(blueprint, fixed_spawn)


        if self.player is None:
            print("Failed to spawn at fixed coordinates → trying random location")
            spawn_points = self.world.get_map().get_spawn_points()
            for sp in spawn_points:
                self.player = self.world.try_spawn_actor(blueprint, sp)
                if self.player is not None:
                    break

        if self.player is None:
            raise RuntimeError("Failed to spawn vehicle at all locations")

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self, self.player)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        # (Optional) lighten coasting feel a bit
        try:
            phys = self.player.get_physics_control()
            phys.drag_coefficient = max(0.05, phys.drag_coefficient * 0.55)
            self.player.apply_physics_control(phys)
            print(f"[PHYS] Drag coefficient tuned for smoother coasting: {phys.drag_coefficient:.3f}")
        except Exception:
            pass
        # ▶ 스폰 직후 Drive(1단)로 세팅해 '기어>=1' 조건을 즉시 만족
        try:
            self.player.apply_control(carla.VehicleControl(
                throttle=0.0, brake=0.0, hand_brake=False, gear=1
            ))
            print("[INIT] Set initial gear=1 (Drive) for creep.")
        except Exception:
            pass

        self.setup_navigation()

        # Acutal Vehicle Status Logging
        import csv, pathlib, datetime

        log_dir = pathlib.Path('/home/somin/CARLA_0.9.14/Logs')
        log_dir.mkdir(parents=True, exist_ok=True)
        csv_fp = log_dir / f'[sub3]_vehicle_data_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv'
        self.csv_file = csv_fp.open('w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # CSV 헤더 추가
        self.csv_writer.writerow([
            'frame_id', 'simulation_timestamp', 'real_timestamp', 'steer', 'throttle', 'brake',
            'speed',
            'velocity_x', 'velocity_y', 'velocity_z',          
            'acceleration_x', 'acceleration_y', 'acceleration_z', 
            'weather_rain',                                      
            'light_state',
            'traffic_light_state',   # 0=Green,1=Red,2=Yellow, -1=Unknown/NoTL
            'dist_to_tl_stop_wp_m',  # meters to current TL stop waypoint (NaN if none)
            'waypoint_x', 'waypoint_y',                         
            'loc_x', 'loc_y', 'loc_z', 'yaw', 'pitch', 'roll',
            'lane_id', 'lane_type', 'is_junction', 'speed_limit',
            'collision_other_id', 'collision_impulse', 'lane_markings',
            'player_stopped', 'wiper_level', 'bicycle_spawned_time',
            'lead_vehicle_id', 'lead_type_id', 'lead_role_name',
            'lead_loc_x', 'lead_loc_y', 'lead_loc_z',
            'lead_vel_x', 'lead_vel_y', 'lead_vel_z', 'lead_speed_ms',
            'front_vehicle_stop',      # 앞차가 "멈춘 그 순간"에만 1
            'stop_task_start_time',    # 트리거 발생 시각(이후에도 유지)
            'lane_invaded', 'lane_inv_types', 'lane_inv_colors', 'lane_inv_time'
        ])

        # --- (CAN·이벤트 상태 저장용 변수 초기화) ----------------------
        self.can_state = {'steer':0,'throttle':0,'brake':0,
                        'speed':0}

        self.collision_state = {'other_id':-1,'impulse':0.0}
        self.lane_state = {'markings': ''}
        
        # --- Lane invasion logging setup ---
        self._lane_inv_queue = deque()  # sensor → logger event queue
        self._lane_inv_latch = {"tsec": 0.0, "types": [], "colors": [], "until": 0.0}
        self._lane_inv_latch_window = float(os.getenv("LANE_INV_LATCH", "0.5"))  # seconds
        
        # Initialize bicycle spawn flag
        if hasattr(self, 'bicycle_spawned'):
            delattr(self, 'bicycle_spawned')

        # 15 Hz resampler state (performance optimization)
        self._sample_dt = 1.0/15.0  # 15 Hz (reduced from 50Hz)
        self._next_log_t = None
        self._prev_state = None  # last packed state
        self._log_buffer = []    # buffered rows for batch write (written once at exit)
        self._raw_logging = False  # set True to also dump raw per-tick rows
        self._last_flush = time.time()

    def spawn_bicycle(self):
        """Spawn bicycle snapped to road"""
        bicycle_bp = self.world.get_blueprint_library().find('vehicle.bh.crossbike')
        
        if not bicycle_bp:
            print("Bicycle blueprint not found")
            return False
        
        # Set to red color
        if bicycle_bp.has_attribute('color'):
            bicycle_bp.set_attribute('color', '255,0,0')
        
        # Snap to road
        wp = self.world.get_map().get_waypoint(self.bicycle_spawn_location, project_to_road=True)
        loc = wp.transform.location + carla.Location(z=0.15)   # Slightly raise
        yaw = wp.transform.rotation.yaw                         # Match lane direction
        tf = carla.Transform(loc, carla.Rotation(yaw=yaw))
        
        self.bicycle = self.world.try_spawn_actor(bicycle_bp, tf)
        
        if self.bicycle:
            print(f"Bicycle spawned successfully! Location: ({loc.x:.1f}, {loc.y:.1f})")

            try:
                # Initially turn off physics and fix for a few frames → prevent falling
                self.bicycle.set_simulate_physics(False)
                self._bicycle_arm_time = time.time() + 0.2  # Turn on physics after 0.2 seconds

                # Initialize autonomous driving - completely straight driving
                self.bicycle_control = carla.VehicleControl()
                self.bicycle_control.throttle = 0.6
                self.bicycle_control.steer = 0.0  # Fix steering angle to 0 for completely straight driving
                self.bicycle_control.hand_brake = False  # Release hand brake

                # Set fall timer (20 seconds after spawn)
                self.fall_timer = time.time() + 20

                return True
            except RuntimeError as e:
                print(f"Bicycle setup failed: {e}")
                if self.bicycle.is_alive:
                    self.bicycle.destroy()
                self.bicycle = None
                return False
        else:
            print("Bicycle spawn failed!")
            return False

    def spawn_truck_ahead(self):
        """Spawn truck ahead of vehicle"""
        try:
            # Truck blueprint
            truck_bp = self.world.get_blueprint_library().find('vehicle.carlamotors.carlacola')
            if truck_bp.has_attribute('color'):
                truck_bp.set_attribute('color', '0,0,255')  # Blue color
            
            # Vehicle spawn location (335.1, 5.3) with y value 10 larger
            truck_location = carla.Location(x=335.1, y=15.3, z=0.5)
            truck_transform = carla.Transform(truck_location, carla.Rotation(yaw=90))
            truck = self.world.try_spawn_actor(truck_bp, truck_transform)
            
            if truck:
                self.truck = truck
                self.truck.set_simulate_physics(True)
                self.truck.apply_control(carla.VehicleControl(hand_brake=False, brake=0.0, throttle=0.0, gear=1))
                self.truck.set_autopilot(False)

                self.truck_agent = BasicAgent(self.truck, target_speed=20)  # km/h
                self.truck_agent.set_destination(self.truck_vias[0], self.truck.get_location())
                self.truck_stage = 0
                self.other_vehicles.append(truck)
                print(f"[TRUCK] Via movement started: {self.truck.get_location()} -> {self.truck_vias[0]}")
            else:
                print("Truck spawn failed")

        except Exception as e:
            print(f"Truck spawn failed: {e}")



    def setup_traffic_lights(self):
        """Setup traffic light control (shorter timing)"""
        try:
            # Find all traffic lights and set initial configuration
            traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
            
            for light in traffic_lights:
                light.set_state(carla.TrafficLightState.Green)
                light.set_green_time(self.light_green)
                light.set_yellow_time(self.light_yellow)
                light.set_red_time(self.light_red)
                
            print(f"[TRAFFIC] {len(list(traffic_lights))} traffic lights configured (R:{self.light_red}s, Y:{self.light_yellow}s, G:{self.light_green}s)")
            
            # Initialize each light
            now = time.time()
            for light in traffic_lights:
                self._tl_last[light.id] = (light.get_state(), now)
                # Unfreeze if stuck
                if light.is_frozen():
                    light.freeze(False)
            
        except Exception as e:
            print(f"Traffic light setup failed: {e}")

    def update_traffic_lights(self):
        """Force update traffic light timing (performance optimization)"""
        if not hasattr(self, '_light_update_counter'):
            self._light_update_counter = 0
        
        self._light_update_counter += 1

        # Update only every 5 frames (performance optimization)
        if self._light_update_counter % 5 == 0:
            traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
            for light in traffic_lights:
                if light.is_frozen():
                    light.freeze(False)  # Unfreeze if stuck
                light.set_green_time(self.light_green)
                light.set_yellow_time(self.light_yellow)
                light.set_red_time(self.light_red)

        # Force skip logic every 2 seconds (increased from 1 second)
        if self._light_update_counter % 120 == 0:  # Changed from 60 to 120
            now = time.time()
            traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
            for light in traffic_lights:
                st = light.get_state()
                lid = light.id
                prev = self._tl_last.get(lid)
                if not prev or prev[0] != st:
                    self._tl_last[lid] = (st, now)
                    continue

                # Force transition to green for entire group if red light exceeds target red_time
                if st == carla.TrafficLightState.Red and (now - prev[1]) > (self.light_red + 0.05):
                    group = light.get_group_traffic_lights()
                    for l in (group if group else [light]):
                        l.set_state(carla.TrafficLightState.Green)
                        self._tl_last[l.id] = (carla.TrafficLightState.Green, now)

    def spawn_other_vehicles(self):
        """Spawn other vehicles at specific locations"""
        try:
            # Vehicle 2: Fast vehicle passing in adjacent lane (avoiding bicycle spawn point)
            vehicle2_bp = self.world.get_blueprint_library().find('vehicle.audi.a2')
            if vehicle2_bp.has_attribute('color'):
                vehicle2_bp.set_attribute('color', '255,255,0')  # Yellow color
            
            # Completely avoid bicycle spawn point (395.3, 282.9)
            vehicle2_location = carla.Location(x=398.0, y=450.0, z=0.5)  # Set larger y value to completely avoid
            vehicle2_transform = carla.Transform(vehicle2_location, carla.Rotation(yaw=-90))
            vehicle2 = self.world.try_spawn_actor(vehicle2_bp, vehicle2_transform)
            
            if vehicle2:
                vehicle2.set_autopilot(True)
                # Set speed for fast movement (avoiding bicycle spawn point)
                vehicle2.set_target_velocity(carla.Vector3D(0, -15, 0))  # 15m/s = ~54km/h
                self.other_vehicles.append(vehicle2)
                print(f"Vehicle 2 spawned successfully: {vehicle2_location} (avoiding bicycle spawn point)")

        except Exception as e:
            print(f"Vehicle spawn failed: {e}")

    def spawn_stop_signs(self):
        """Spawn stop signs"""
        try:
            # Stop sign 1: near bicycle
            stop_sign1_bp = self.world.get_blueprint_library().find('static.prop.stopsign')
            stop_sign1_location = carla.Location(x=395.3, y=285.0, z=0.5)
            stop_sign1_transform = carla.Transform(stop_sign1_location, carla.Rotation(yaw=-90))
            stop_sign1 = self.world.try_spawn_actor(stop_sign1_bp, stop_sign1_transform)
            
            if stop_sign1:
                self.stop_signs.append(stop_sign1)
                print(f"Stop sign 1 spawned successfully: {stop_sign1_location}")

            # Stop sign 2: ahead of vehicle
            stop_sign2_bp = self.world.get_blueprint_library().find('static.prop.stopsign')
            stop_sign2_location = carla.Location(x=395.3, y=315.0, z=0.5)
            stop_sign2_transform = carla.Transform(stop_sign2_location, carla.Rotation(yaw=-90))
            stop_sign2 = self.world.try_spawn_actor(stop_sign2_bp, stop_sign2_transform)
            
            if stop_sign2:
                self.stop_signs.append(stop_sign2)
                print(f"Stop sign 2 spawned successfully: {stop_sign2_location}")

        except Exception as e:
            print(f"Stop sign spawn failed: {e}")

    def change_weather_to_rain(self):
        """Change weather to rain"""
        rain_weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=60.0,
            precipitation_deposits=70.0,
            wind_intensity=30.0,
            sun_azimuth_angle=70.0,
            sun_altitude_angle=30.0
        )
        self.world.set_weather(rain_weather)
        print("[WEATHER] Rain has started!")

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def check_highway_proximity(self):
        """고속도로 근처인지 확인하는 함수"""
        player_location = self.player.get_location()
        
        # Town04의 고속도로 좌표 (예시)
        highway_zones = [
            {"center": carla.Location(x=200, y=-150, z=0), "radius": 50},
            {"center": carla.Location(x=350, y=-200, z=0), "radius": 60},
            {"center": carla.Location(x=450, y=-100, z=0), "radius": 70}
        ]
        
        for zone in highway_zones:
            distance = player_location.distance(zone["center"])
            if distance <= zone["radius"]:
                return True
        return False

    def auto_weather_change(self):
        """위치 기반 자동 날씨 변경"""
        if not hasattr(self, '_current_weather_type'):
            self._current_weather_type = "clear"
            self._weather_change_cooldown = 0
        
        # 쿨다운 감소
        if self._weather_change_cooldown > 0:
            self._weather_change_cooldown -= 1
            return
        
        is_near_highway = self.check_highway_proximity()
        
        # 고속도로 근처에서 비 시작
        if is_near_highway and self._current_weather_type != "rain":
            # 비 날씨 설정
            rain_weather = carla.WeatherParameters(
                cloudiness=80.0,
                precipitation=60.0,
                precipitation_deposits=70.0,
                wind_intensity=50.0,
                sun_azimuth_angle=70.0,
                sun_altitude_angle=30.0
            )
            self.world.set_weather(rain_weather)
            self._current_weather_type = "rain"
            self._weather_change_cooldown = 300  # 5초 쿨다운 (60fps 기준)
            self.hud.notification("🌧️ It begins to rain near the highway!", seconds=3.0)
            
        # 고속도로에서 멀어지면 맑은 날씨로 복귀
        elif not is_near_highway and self._current_weather_type != "clear":
            clear_weather = carla.WeatherParameters.ClearNoon
            self.world.set_weather(clear_weather)
            self._current_weather_type = "clear"
            self._weather_change_cooldown = 300
            self.hud.notification("☀️ Rain", seconds=3.0)

    def tick(self, clock):
        self.hud.tick(self, clock)
        now_wall = time.time()   # ← 맨 위쪽으로 이동(트럭 블록보다 위)
        
        # Navigation HUD update
        if hasattr(self, 'destination_set') and self.destination_set:
            self.update_navigation_hud()
            
        # Continuous traffic light update (every frame)
        self.update_traffic_lights()

        # Scenario management
        player_location = self.player.get_location()
        
        # 1. Initial vehicle and stop sign spawn (when vehicle reaches specific location)
        if not hasattr(self, 'scenario_initialized'):
            init_trigger = carla.Location(x=394.0, y=300.0, z=0.0)
            if player_location.distance(init_trigger) <= 10.0:
                print("[SCENARIO] Initialization: spawning other vehicles and stop signs")
                self.spawn_other_vehicles()
                self.spawn_stop_signs()
                self.scenario_initialized = True
        
        # 2. Start rain weather (when vehicle approaches bicycle area)
        if not hasattr(self, 'rain_started') and hasattr(self, 'scenario_initialized'):
            rain_trigger = carla.Location(x=394.0, y=290.0, z=0.0)
            if player_location.distance(rain_trigger) <= 5.0:
                print("[WEATHER] Starting rain weather")
                self.change_weather_to_rain()
                self.rain_started = True
        
        # 3. Bicycle spawn (when vehicle reaches specific coordinates)
        if not self.bicycle or not self.bicycle.is_alive:
            # Bicycle spawn trigger coordinates (location vehicle will reach)
            trigger_location = carla.Location(x=394.0, y=250.5, z=0.0)
            trigger_distance = self.bicycle_spawn_trigger_distance  # Use parameter
            
            # Calculate distance between vehicle and trigger coordinates
            distance_to_trigger = player_location.distance(trigger_location)
            
            # Spawn bicycle if close enough
            if distance_to_trigger <= trigger_distance:
                if not hasattr(self, 'bicycle_spawned'):
                    print(f"[BICYCLE] Spawning bicycle! Vehicle reached trigger coordinates! Distance: {distance_to_trigger:.1f}m")
                    self.spawn_bicycle()
                    self.bicycle_spawned = True
                    self._bicycle_spawned_time_wall = time.time()  # record wall-clock at spawn
        
        # Stop rain near via3 (153.9, 33.8) – once
        if not hasattr(self, '_rain_stopped_at_via3'):
            stop_loc = carla.Location(x=153.9, y=33.8, z=0.0)
            if self.player.get_location().distance(stop_loc) < 12.0:
                clear = carla.WeatherParameters.ClearNoon
                clear.precipitation = 0.0
                clear.precipitation_deposits = 0.0
                self.world.set_weather(clear)
                self._rain_stopped_at_via3 = True
                self.hud.notification("[WEATHER] Rain stopped", seconds=2.5)
        
        # Truck waypoint following via BasicAgent
        if self.truck and self.truck.is_alive and self.truck_agent:
            try:
                control = self.truck_agent.run_step()   # ← 한 번만
                self.truck.apply_control(control)

                # 현재 목표지 선택
                if self.truck_stage < len(self.truck_vias):
                    dest = self.truck_vias[self.truck_stage]
                else:
                    dest = self.truck_final_location

                if self.truck.get_location().distance(dest) < 3.0:
                    if self.truck_stage < len(self.truck_vias) - 1:
                        # 다음 경유지로
                        self.truck_stage += 1
                        self.truck_agent.set_destination(self.truck_vias[self.truck_stage])
                        self.truck_agent.set_target_speed(20)  # Maintain
                        print(f"[TRUCK] Reached via{self.truck_stage} → proceeding to next via: {self.truck_vias[self.truck_stage]}")
                    elif self.truck_stage == len(self.truck_vias) - 1:
                        # To final destination
                        self.truck_stage += 1
                        self.truck_agent.set_destination(self.truck_final_location)
                        self.truck_agent.set_target_speed(20)  # Maintain
                        print(f"[TRUCK] Reached last via → proceeding to final destination: {self.truck_final_location}")
                    else:
                        # Reached final destination → stop
                        self.truck.apply_control(carla.VehicleControl(throttle=0.0, brake=0.6))
                        self.truck_stage = 3
                        print("[TRUCK] Detour/route completed → stopping")
                
                # Safety mechanism: give one "kick" if speed is almost 0 for more than 2 seconds
                if not hasattr(self, '_truck_kick_ts'):
                    self._truck_kick_ts = now_wall
                elif now_wall - self._truck_kick_ts > 2.0 and self.truck.get_velocity().length() < 0.1:
                    self.truck.apply_control(carla.VehicleControl(throttle=0.25, brake=0.0, hand_brake=False))
                    self._truck_kick_ts = float('inf')
                    
            except Exception as e:
                print(f"[ERROR] Truck BasicAgent control error: {e}")
        
        # Check if player is alive
        if not self.player or not self.player.is_alive:
            return
            
        transform = self.player.get_transform()

        control = self.player.get_control()
        velocity = self.player.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        # ---------------------------------------------------------------------------
        snap      = self.world.get_snapshot()
        frame_id  = snap.frame
        simulation_timestamp = snap.timestamp.elapsed_seconds
        
        # If bicycle just spawned this run and sim-time not set yet → set once
        if hasattr(self, 'bicycle_spawned') and getattr(self, '_bicycle_spawned_time_sim', None) is None:
            self._bicycle_spawned_time_sim = simulation_timestamp
        
        # Add real time (fixed interval for 100Hz sampling)
        now_wall = time.time()
        real_timestamp = now_wall

        t         = self.player.get_transform()
        vel       = self.player.get_velocity()
        speed_ms  = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        rot       = t.rotation

        # 가속도
        accel = self.player.get_acceleration()
        
        # Weather (precipitation)
        weather = self.world.get_weather()
        rain = weather.precipitation  # 0-100
        
        # Vehicle light state (vehicle itself)
        light_state = int(self.player.get_light_state())

        # ===== Traffic light state (intersection control) and Stop task trigger detection =====
        def _tl_state_num_of(actor):
            try:
                st = actor.get_state()
            except Exception:
                return -1
            if st == carla.TrafficLightState.Green:  return 0
            if st == carla.TrafficLightState.Red:    return 1
            if st == carla.TrafficLightState.Yellow: return 2
            return -1

        tl_actor = None
        try:
            tl_actor = self.player.get_traffic_light()
        except Exception:
            tl_actor = None
        traffic_light_state_num = _tl_state_num_of(tl_actor) if tl_actor else -1

        # When Red occurs after seeing Green (typically Green→Yellow→Red path), record Red transition as Stop task start
        if traffic_light_state_num == 0:
            self._saw_green_since_last_red = True
        if traffic_light_state_num == 1:  # Frame when Red occurred
            if self._traffic_prev_state_num != 1:
                # If Green was seen before (regardless of Yellow), recognize Red transition as trigger
                if self._saw_green_since_last_red and self._stop_task_start_time is None:
                    self._stop_task_start_time = simulation_timestamp
            # Entered Red phase, prepare for next cycle
            self._saw_green_since_last_red = False
        self._traffic_prev_state_num = traffic_light_state_num

        # ===== Lead vehicle sudden stop detection (front_vehicle_stop=1 only for one moment) =====
        front_stop_now = 0
        # Lead vehicle detection: within 40m forward view & forward direction (dot>0) (performance optimization)
        ego_loc = t.location
        ego_fwd = t.get_forward_vector()
        lead = None
        
        # Detect lead vehicle only every 2 frames (performance optimization)
        if not hasattr(self, '_lead_detection_counter'):
            self._lead_detection_counter = 0
        self._lead_detection_counter += 1
        
        if self._lead_detection_counter % 2 == 0:  # Execute only every 2 frames
            try:
                vehicles = self.world.get_actors().filter('vehicle.*')
                best_dist = 1e9
                for v in vehicles:
                    if v.id == self.player.id: 
                        continue
                    loc = v.get_location()
                    rel = loc - ego_loc
                    # Forward direction & distance limit
                    if (ego_fwd.x*rel.x + ego_fwd.y*rel.y + ego_fwd.z*rel.z) <= 0:
                        continue
                    d = math.sqrt(rel.x*rel.x + rel.y*rel.y + rel.z*rel.z)
                    if d < 40.0 and d < best_dist:
                        best_dist = d
                        lead = v
            except Exception:
                lead = None
        else:
            # Use cached lead vehicle information
            lead = getattr(self, '_cached_lead', None)
        
        # Cache lead vehicle information
        self._cached_lead = lead

        # Initialize lead vehicle information (default: none)
        lead_id, lead_type_id, lead_role_name = -1, '', ''
        lead_loc, lead_vel3, lead_spd = (float('nan'),)*3, (float('nan'),)*3, float('nan')
        
        if lead is not None and lead.is_alive:
            self._lead_vehicle_id = lead.id
            vel_lead = lead.get_velocity()
            spd_lead = math.sqrt(vel_lead.x**2 + vel_lead.y**2 + vel_lead.z**2)  # m/s
            loc_lead = lead.get_location()
            
            # Store lead vehicle information
            lead_id = lead.id
            lead_type_id = getattr(lead, "type_id", "")
            lead_role_name = lead.attributes.get("role_name", "")
            lead_loc = (loc_lead.x, loc_lead.y, loc_lead.z)
            lead_vel3 = (vel_lead.x, vel_lead.y, vel_lead.z)
            lead_spd = spd_lead
            
            prev = self._front_vehicle_prev_speed.get(lead.id, spd_lead)
            # Definition of "sudden stop": previous >2 m/s, now <0.3 m/s sudden drop
            if prev > 2.0 and spd_lead < 0.3:
                front_stop_now = 1
                # Remember this moment as simulation ts (to set 1 only on one line among 50Hz rows)
                self._front_stop_event_t_sim = simulation_timestamp
                # Set stop task start time here if still empty
                if self._stop_task_start_time is None:
                    self._stop_task_start_time = simulation_timestamp
            self._front_vehicle_prev_speed[lead.id] = spd_lead
        else:
            self._lead_vehicle_id = None
        
        # Nearest waypoint
        wp = self.world.get_map().get_waypoint(t.location, project_to_road=True)
        lane_loc = wp.transform.location if wp else carla.Location(0, 0, 0)

        # --- distance to traffic light's stop waypoint (meters)
        stop_wp_xy = (float('nan'), float('nan'))
        dist_to_stop_wp = float('nan')
        
        # 성능 최적화: fallback 탐색은 10프레임마다만 실행
        if not hasattr(self, '_tl_guess_counter'):
            self._tl_guess_counter = 0
            self._cached_tl_guess = (float('nan'), None, None)
        
        self._tl_guess_counter += 1
        should_update_guess = (self._tl_guess_counter % 10 == 0)  # 10프레임마다 갱신
        
        # 1) 먼저 '현재 차량에 실제로 영향주는' 신호등으로 시도
        if tl_actor:
            try:
                stops = tl_actor.get_stop_waypoints()
                if stops:
                    candidates = stops
                    if wp:
                        same_lane = [sw for sw in stops if sw.road_id == wp.road_id and sw.lane_id == wp.lane_id]
                        if same_lane:
                            candidates = same_lane
                    cur_loc = t.location
                    best_wp = min(candidates, key=lambda sw: cur_loc.distance(sw.transform.location))
                    sw_loc = best_wp.transform.location
                    stop_wp_xy = (sw_loc.x, sw_loc.y)
                    dist_to_stop_wp = cur_loc.distance(sw_loc)
            except Exception:
                pass
        
        # 2) fallback: 아직 트리거에 안 들어와 tl_actor가 없거나 거리 못 구했으면
        if not math.isfinite(dist_to_stop_wp):
            if should_update_guess:
                # 10프레임마다 새로운 추정 계산
                self._cached_tl_guess = self._nearest_tl_stop_ahead(
                    t.location, t.get_forward_vector(), wp, max_dist=120.0, lateral_tol=4.0
                )
            
            # 캐시된 추정값 사용
            guess_d, guess_loc, guess_light = self._cached_tl_guess
            if math.isfinite(guess_d) and guess_loc is not None:
                stop_wp_xy = (guess_loc.x, guess_loc.y)
                dist_to_stop_wp = guess_d

        lane_id   = wp.lane_id if wp else 0
        lane_type = str(wp.lane_type) if wp else ''
        is_junc   = int(wp.is_junction) if wp else 0
        speed_lim = self.player.get_speed_limit()

        # Use direct control state instead of CAN Bus sensor
        can_data = {
            'steer': control.steer,
            'throttle': control.throttle,
            'brake': control.brake,
            'speed': speed_ms
        }

        # Collision/lane state (get from world object)
        collision_data = getattr(self.world, 'collision_state', {'other_id': -1, 'impulse': 0.0})
        lane_data = getattr(self.world, 'lane_state', {'markings': ''})

        # Bicycle state tracking
        bicycle_spawned = 1 if hasattr(self, 'bicycle_spawned') and self.bicycle_spawned else 0
        bicycle_fell = 1 if hasattr(self.bicycle, 'fell') and self.bicycle.fell else 0
        
        # Player stopped state (when speed is less than 1km/h)
        player_stopped = 1 if can_data['speed'] < 0.28 else 0  # 1km/h = 0.28 m/s
        
        # Wiper state (get from controller)
        wiper_level = getattr(self.controller, '_wiper_level', 0)

        state_cur = self._pack_state(
            simulation_timestamp, t, vel, accel, speed_ms, rot, wp,
            lane_id, lane_type, is_junc, speed_lim, control, rain, light_state,
            traffic_light_state_num,
            collision_data, lane_data, bicycle_spawned, bicycle_fell,
            player_stopped, wiper_level
        )
        state_cur['stop_wp_xy'] = stop_wp_xy
        state_cur['dist_to_stop_wp_m'] = dist_to_stop_wp

        # attach lead features (nearest-hold; no need to interpolate)
        state_cur['lead_id'] = lead_id
        state_cur['lead_type_id'] = lead_type_id
        state_cur['lead_role_name'] = lead_role_name
        state_cur['lead_loc'] = lead_loc
        state_cur['lead_vel'] = lead_vel3
        state_cur['lead_speed_ms'] = lead_spd

        # Initialize resampler anchors
        if self._next_log_t is None:
            self._next_log_t = simulation_timestamp
            self._prev_state = state_cur

        # Produce uniform 15 Hz samples between prev->cur (performance optimization)
        t0 = self._prev_state['t']; t1 = state_cur['t']
        if t1 > t0 + 1e-9:
            while self._next_log_t <= t1 + 1e-12:
                alpha = 0.0 if t1 == t0 else max(0.0, min(1.0, (self._next_log_t - t0) / (t1 - t0)))

                # Interpolate kinematics / pose
                loc  = self._lerp_tuple(self._prev_state['loc'],   state_cur['loc'],   alpha)
                ypr  = self._lerp_tuple(self._prev_state['ypr'],   state_cur['ypr'],   alpha)
                vel3 = self._lerp_tuple(self._prev_state['vel'],   state_cur['vel'],   alpha)
                acc3 = self._lerp_tuple(self._prev_state['acc'],   state_cur['acc'],   alpha)
                spd  = self._lerp(self._prev_state['speed_ms'],    state_cur['speed_ms'], alpha)
                wp2  = self._lerp_tuple(self._prev_state['wp_xy'], state_cur['wp_xy'], alpha)

                steer, thr, brk = state_cur['ctrl']

                # distance to TL stop waypoint at this interpolated pose
                stop_dist = state_cur.get('dist_to_stop_wp_m', float('nan'))
                if 'stop_wp_xy' in state_cur and not any(math.isnan(v) for v in state_cur['stop_wp_xy']):
                    try:
                        dx = loc[0] - state_cur['stop_wp_xy'][0]
                        dy = loc[1] - state_cur['stop_wp_xy'][1]
                        stop_dist = math.hypot(dx, dy)
                    except Exception:
                        pass

                # Front vehicle sudden stop flag is set to 1 only on one sample that matches 'that moment' simulation time
                fv_stop = 0
                if self._front_stop_event_t_sim is not None:
                    if abs(self._next_log_t - self._front_stop_event_t_sim) <= (self._sample_dt * 0.5):
                        fv_stop = 1

                # --- Lane invasion data processing ---
                csv_lane_invaded = 0
                csv_lane_types   = 'nan'   # <- 빈 문자열 대신 'nan'
                csv_lane_colors  = 'nan'
                csv_lane_time    = 'nan'

                # Prefer queue event (one-off), then fallback to latch window
                now_sim_t = self.world.get_snapshot().timestamp.elapsed_seconds
                q = getattr(self, "_lane_inv_queue", None)

                if q and len(q) > 0:
                    tsec, types, colors = q.popleft()
                    csv_lane_invaded = 1
                    csv_lane_types   = "|".join(types)  if types  else 'nan'
                    csv_lane_colors  = "|".join(colors) if colors else 'nan'
                    csv_lane_time    = f"{tsec:.3f}"
                else:
                    latch = getattr(self, "_lane_inv_latch", None)
                    if latch and now_sim_t <= latch.get("until", 0.0):
                        csv_lane_invaded = 1
                        csv_lane_types   = "|".join(latch.get("types", []))  or 'nan'
                        csv_lane_colors  = "|".join(latch.get("colors", [])) or 'nan'
                        csv_lane_time    = f"{latch.get('tsec', float('nan')):.3f}"
                    else:
                        # 이벤트가 전혀 없는 샘플은 명시적으로 NaN 유지
                        csv_lane_invaded = 0
                        csv_lane_types   = 'nan'
                        csv_lane_colors  = 'nan'
                        csv_lane_time    = 'nan'

                row = [
                    frame_id,                    # keep frame_id of current snapshot
                    self._next_log_t,            # uniform 15 Hz simulation timestamp
                    time.time(),                 # wall-clock
                    steer, thr, brk,
                    spd,
                    *vel3,
                    *acc3,
                    state_cur['rain'],
                    state_cur['light_state'],
                    state_cur['tl_state_num'],
                    stop_dist,
                    *wp2,
                    *loc, *ypr,
                    state_cur['lane_id'], state_cur['lane_type'], state_cur['is_junc'],
                    state_cur['speed_lim'],
                    state_cur['collision'][0], state_cur['collision'][1],
                    state_cur['lane_mark'],
                    state_cur['player_stopped'], state_cur['wiper_level'],
                    (getattr(self, '_bicycle_spawned_time_sim', '') if hasattr(self, 'bicycle_spawned') else ''),
                    state_cur['lead_id'], state_cur['lead_type_id'], state_cur['lead_role_name'],
                    *state_cur['lead_loc'], *state_cur['lead_vel'], state_cur['lead_speed_ms'],
                    fv_stop,
                    (self._stop_task_start_time if self._stop_task_start_time is not None else ''),
                    csv_lane_invaded, csv_lane_types, csv_lane_colors, csv_lane_time
                ]
                self._log_buffer.append(row)
                self._next_log_t += self._sample_dt

        # 주기적 플러시(0.5초)
        now_wall = time.time()
        if self._log_buffer and (now_wall - self._last_flush > 0.5):
            self.csv_writer.writerows(self._log_buffer)
            self._log_buffer.clear()
            self._last_flush = now_wall

        self._prev_state = state_cur

        # (Optional) keep the old per-tick raw line: disabled by default
        if self._raw_logging:
            self.csv_writer.writerow([
                frame_id, simulation_timestamp, real_timestamp,
                can_data['steer'], can_data['throttle'], can_data['brake'],
                can_data['speed'],
                vel.x, vel.y, vel.z,
                accel.x, accel.y, accel.z,
                rain,
                light_state,
                traffic_light_state_num,
                dist_to_stop_wp,
                lane_loc.x, lane_loc.y,
                t.location.x, t.location.y, t.location.z,
                rot.yaw, rot.pitch, rot.roll,
                lane_id, lane_type, is_junc,
                speed_lim,
                collision_data['other_id'], collision_data['impulse'],
                lane_data['markings'],
                bicycle_spawned, bicycle_fell, player_stopped, wiper_level,
                front_stop_now,
                (self._stop_task_start_time if self._stop_task_start_time is not None else '')
            ])

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

        if self.bicycle and self.bicycle.is_alive:
            # spawn 후 0.2초 지나면 물리 켜기
            if not getattr(self, '_bicycle_armed', False):
                if time.time() >= getattr(self, '_bicycle_arm_time', 0):
                    self.bicycle.set_simulate_physics(True)
                    self._bicycle_armed = True
            
            # 플레이어가 지정된 거리 이내일 때부터 달리기 시작
            if getattr(self, '_bicycle_armed', False):
                player_location = self.player.get_location()
                bicycle_location = self.bicycle.get_location()
                distance_to_player = player_location.distance(bicycle_location)
                
                if distance_to_player < self.bicycle_start_distance:
                    # 자동 주행 제어
                    if hasattr(self, 'bicycle_control'):
                        try:
                            self.bicycle.apply_control(self.bicycle_control)
                        except RuntimeError:
                            print("[경고] 자전거 접근 불가")
                            self.bicycle = None

            # 넘어짐 타이머 확인
            if hasattr(self, 'fall_timer') and time.time() > self.fall_timer:
                if not hasattr(self.bicycle, 'fell'):
                    try:
                        transform = self.bicycle.get_transform()
                        fallen_rot = carla.Rotation(
                            pitch=0,
                            yaw=transform.rotation.yaw,
                            roll=45
                        )
                        fallen_transform = carla.Transform(transform.location, fallen_rot)
                        self.bicycle.set_simulate_physics(False)
                        self.bicycle.set_transform(fallen_transform)

                        self.bicycle.fell = True
                        self.recover_timer = time.time() + 15  # 15초 뒤 물리 다시 켜기
                        print("자전거가 넘어졌습니다!")
                    except RuntimeError:
                        print("[경고] 자전거 회전 실패")
                        self.bicycle = None
                    finally:
                        delattr(self, 'fall_timer')

            # 복구 타이머도 여기에
            if hasattr(self, 'recover_timer') and time.time() > self.recover_timer:
                if self.bicycle and self.bicycle.is_alive:
                    try:
                        self.bicycle.set_simulate_physics(True)
                        print("🔄 자전거 물리 엔진 복구됨")
                    except RuntimeError:
                        print("[경고] 자전거 복구 실패")
                        self.bicycle = None
                delattr(self, 'recover_timer')

    def _set_multi_stop_route(self, stops):
        """
        stops: [경유지1, 경유지2, ... , 최종목적지]
        현재 위치 → 각 stop 으로 이어지는 도로 경로를 연결해서 self.route_points에 저장
        """
        if not self.route_planner:
            self.route_planner = GlobalRoutePlanner(self.world.get_map(), 2.0)

        self.route_points = []
        self.route = []  # 전체 경로를 저장 (HUD 인덱스 계산용)

        cur = self.player.get_location()
        for k, dest in enumerate(stops):
            seg = self.route_planner.trace_route(cur, dest)
            # 시각화용 포인트 연결
            self.route_points += [wp.transform.location for (wp, _) in seg]
            # 전체 경로를 self.route에 누적 저장
            self.route += seg
            cur = dest

        # HUD update settings
        self.destination_set = True
        self.current_route_index = 0
        self.destination_location = stops[-1]  # Final destination changed to via3
        self.hud.notification(f'Route set (including via points): {len(self.route_points)} points')

        # ▶ Draw route visualization "once and long" (long life_time)
        self._draw_route_debug_once()

    def _draw_route_debug_once(self):
        """Draw route debug lines once with long life time"""
        try:
            if not self.route_points:
                return
            col = carla.Color(18, 42, 66)  # Dark color
            thickness = 0.03
            life = 120.0   # Maintain for 120 seconds (adjust longer if needed)
            z_off = 0.08
            skip = 2       # Dotted line effect
            for i in range(0, len(self.route_points) - 1, skip):
                p0 = self.route_points[i]   + carla.Location(z=z_off)
                p1 = self.route_points[i+1] + carla.Location(z=z_off)
                self.world.debug.draw_line(p0, p1, thickness=thickness, color=col, life_time=life)
            self._last_route_dbg_drawn_at = time.time()
        except:
            pass

    def setup_navigation(self):
        """내비게이션 경로 설정"""
        try:
            # 좌표가 유효한지 확인
            spawn_waypoint = self.world.get_map().get_waypoint(self.spawn_location)
            dest_waypoint = self.world.get_map().get_waypoint(self.destination_location)
            
            if not spawn_waypoint:
                self.hud.notification(f'시작점이 유효하지 않습니다: {self.spawn_location}')
                return
            
            if not dest_waypoint:
                self.hud.notification(f'목적지가 유효하지 않습니다: {self.destination_location}')
                return
            
            # Global Route Planner 초기화
            self.route_planner = GlobalRoutePlanner(self.world.get_map(), 2.0)
            
            # ★ 변경 포인트: 모든 경유지를 포함한 멀티-스톱 경로 설정
            stops = [
                carla.Location(x=334.7, y=307.6, z=0.5),
                carla.Location(x=396.4, y=250.5, z=0.5),
                carla.Location(x=396.7, y=19.8, z=0.5),
                carla.Location(x=172.9, y=-2.1, z=0.5),
                carla.Location(x=154.0, y=36.9, z=0.5)
            ]
            self._set_multi_stop_route(stops)
            
            # 시작 포인트 라벨(선택)
            if self.route_points:
                self.world.debug.draw_string(self.route_points[0] + carla.Location(z=0.5),
                                             "ROUTE_START", draw_shadow=False,
                                             color=carla.Color(255,255,0), life_time=2.0)
        except Exception as e:
            self.hud.notification(f'내비게이션 설정 오류: {str(e)}')
            print(f"상세 오류: {e}")

    def update_navigation_hud(self):
        """내비게이션 HUD 정보 업데이트"""
        if not self.destination_set or not self.route:
            return
            
        current_location = self.player.get_location()
        current_rotation = self.player.get_transform().rotation
        
        # 목적지까지의 거리 계산
        distance_to_destination = current_location.distance(self.destination_location)
        
        # 방향 지시 계산 (더 먼 거리에서 미리 계산)
        direction = "Continue"
        look_ahead_index = min(self.current_route_index + 2, len(self.route) - 1)  # 2개 앞 웨이포인트 확인

        if look_ahead_index > self.current_route_index:
            current_wp = self.route[self.current_route_index][0]
            ahead_wp = self.route[look_ahead_index][0]
            
            # 현재 차량 방향과 앞쪽 웨이포인트 방향 비교
            vehicle_yaw = current_rotation.yaw
            ahead_yaw = ahead_wp.transform.rotation.yaw
            # 각도 차이 계산
            angle_diff = ahead_yaw - vehicle_yaw
            # 각도를 -180~180 범위로 정규화
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
                
            if angle_diff > 30:
                direction = "Turn Right"
            elif angle_diff < -30:
                direction = "Turn Left"
            elif abs(angle_diff) > 10:
                direction = "Slight Right" if angle_diff > 0 else "Slight Left"
        
        # 다음 웨이포인트까지의 거리
        if self.current_route_index < len(self.route):
            next_waypoint = self.route[self.current_route_index][0]
            distance_to_next = current_location.distance(next_waypoint.transform.location)
            
            # 웨이포인트에 가까워지면 다음 웨이포인트로 이동
            if distance_to_next < 8.0 and self.current_route_index < len(self.route) - 1:
                self.current_route_index += 1

        # HUD에 저장할 내비게이션 정보 업데이트
        self.nav_distance = distance_to_destination
        self.nav_waypoint_info = f'{self.current_route_index + 1}/{len(self.route)}'
        self.nav_direction = direction

        # ▶ Don't draw every frame. Redraw periodically only if needed (e.g., 30 second intervals)
        if getattr(self, '_last_route_dbg_drawn_at', 0) + 30.0 < time.time():
            self._draw_route_debug_once()

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor]
        for sensor in sensors:
            if sensor is not None and sensor.is_alive:
                try:
                    sensor.stop()
                    sensor.destroy()
                except RuntimeError:
                    pass
        
        # Lane invasion sensor cleanup
        if hasattr(self, "lane_invasion_sensor") and self.lane_invasion_sensor:
            self.lane_invasion_sensor.destroy()
        if self.player is not None and self.player.is_alive:
            try:
                self.player.destroy()
            except RuntimeError:
                pass
        # Save all log data at once on exit (performance improvement by removing real-time IO)
        if hasattr(self, '_log_buffer') and self._log_buffer:
            try:
                print(f"[LOG] Saving {len(self._log_buffer)} rows to file...")
                self.csv_writer.writerows(self._log_buffer)
                print("[LOG] Save completed successfully")
            except Exception as e:
                print(f"[ERROR] Log save failed: {e}")
        # CSV 파일 닫기
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            
        # 보행자 정리
        if hasattr(self, 'walkers'):
            for walker, controller in self.walkers:
                if controller and controller.is_alive:
                    try:
                        controller.stop()
                        controller.destroy()
                    except RuntimeError:
                        pass
                if walker and walker.is_alive:
                    try:
                        walker.destroy()
                    except RuntimeError:
                        pass

        # 자전거 정리
        if self.bicycle is not None and self.bicycle.is_alive:
            try:
                self.bicycle.destroy()
            except RuntimeError:
                pass
            self.bicycle = None

        # 다른 차량들 정리
        for vehicle in self.other_vehicles:
            if vehicle and vehicle.is_alive:
                try:
                    vehicle.destroy()
                except RuntimeError:
                    pass
        self.other_vehicles.clear()

        # 정지 표지판들 정리
        for stop_sign in self.stop_signs:
            if stop_sign and stop_sign.is_alive:
                try:
                    stop_sign.destroy()
                except RuntimeError:
                    pass
        self.stop_signs.clear()

    def get_weather_presets(self):
        """다양한 날씨 프리셋"""
        return {
            "clear": carla.WeatherParameters.ClearNoon,
            "light_rain": carla.WeatherParameters(
                cloudiness=60.0,
                precipitation=30.0,
                precipitation_deposits=40.0,
                wind_intensity=30.0
            ),
            "heavy_rain": carla.WeatherParameters(
                cloudiness=90.0,
                precipitation=80.0,
                precipitation_deposits=90.0,
                wind_intensity=70.0,
                sun_azimuth_angle=70.0,
                sun_altitude_angle=10.0  # 어두운 하늘
            ),
            "storm": carla.WeatherParameters(
                cloudiness=100.0,
                precipitation=100.0,
                precipitation_deposits=100.0,
                wind_intensity=100.0,
                sun_altitude_angle=5.0
            )
        }
    def smooth_weather_transition(self, target_weather_name):
        """부드러운 날씨 전환"""
        if not hasattr(self, '_weather_transition'):
            self._weather_transition = {
                'active': False,
                'current': None,
                'target': None,
                'progress': 0.0
            }
        
        weather_presets = self.get_weather_presets()
        current_weather = self.world.get_weather()
        target_weather = weather_presets[target_weather_name]
        
        # 전환 시작
        if not self._weather_transition['active']:
            self._weather_transition = {
                'active': True,
                'current': current_weather,
                'target': target_weather,
                'progress': 0.0
            }
        
        # 점진적 변화 (5초에 걸쳐 변화)
        self._weather_transition['progress'] += 0.01  # 초당 0.6씩 증가
        
        if self._weather_transition['progress'] >= 1.0:
            # 전환 완료
            self.world.set_weather(target_weather)
            self._weather_transition['active'] = False
            return True
        
        # 중간값 계산
        progress = self._weather_transition['progress']
        current = self._weather_transition['current']
        target = self._weather_transition['target']
        
        interpolated = carla.WeatherParameters(
            cloudiness=current.cloudiness + (target.cloudiness - current.cloudiness) * progress,
            precipitation=current.precipitation + (target.precipitation - current.precipitation) * progress,
            precipitation_deposits=current.precipitation_deposits + (target.precipitation_deposits - current.precipitation_deposits) * progress,
            wind_intensity=current.wind_intensity + (target.wind_intensity - current.wind_intensity) * progress
        )
        
        self.world.set_weather(interpolated)
        return False
    
    def _pack_state(self, simulation_timestamp, t, vel, accel, speed_ms, rot, wp,
                    lane_id, lane_type, is_junc, speed_lim, control, rain,
                    light_state, tl_state_num, collision_data, lane_data, bicycle_spawned,
                    bicycle_fell, player_stopped, wiper_level):
        # Collects all fields required for interpolation and logging
        lane_xy = (wp.transform.location.x, wp.transform.location.y) if wp else (0.0, 0.0)
        return {
            't': simulation_timestamp,
            'loc': (t.location.x, t.location.y, t.location.z),
            'ypr': (rot.yaw, rot.pitch, rot.roll),
            'vel': (vel.x, vel.y, vel.z),
            'acc': (accel.x, accel.y, accel.z),
            'speed_ms': speed_ms,
            'wp_xy': lane_xy,
            'lane_id': lane_id,
            'lane_type': lane_type,
            'is_junc': is_junc,
            'speed_lim': speed_lim,
            'ctrl': (control.steer, control.throttle, control.brake),
            'rain': rain,
            'light_state': light_state,
            'tl_state_num': tl_state_num,
            'collision': (collision_data['other_id'], collision_data['impulse']),
            'lane_mark': lane_data['markings'],
            'bicycle_spawned': bicycle_spawned,
            'bicycle_fell': bicycle_fell,
            'player_stopped': player_stopped,
            'wiper_level': wiper_level
        }

    def _lerp(self, a, b, alpha):
        return a + (b - a) * alpha

    def _lerp_tuple(self, a, b, alpha):
        return tuple(self._lerp(x, y, alpha) for x, y in zip(a, b))
    
    def is_on_highway(self):
        """현재 고속도로에 있는지 정확히 감지"""
        player_location = self.player.get_location()
        waypoint = self.world.get_map().get_waypoint(player_location)
        
        if waypoint:
            # 고속도로 특성 확인
            road_id = waypoint.road_id
            lane_type = waypoint.lane_type
            
            # Town04에서 고속도로로 간주할 도로 ID들
            highway_road_ids = [36, 37, 38, 39, 40]  # 실제 맵에 맞게 조정 필요
            
            # 도로 폭이 넓거나 속도 제한이 높은 경우
            if (road_id in highway_road_ids or 
                waypoint.lane_width > 4.0 or
                lane_type == carla.LaneType.Driving):
                return True
        
        return False

    def _nearest_tl_stop_ahead(self, ego_loc, ego_fwd, wp, max_dist=120.0, lateral_tol=4.0):
        """
        ego_loc: carla.Location (차량 위치)
        ego_fwd: carla.Vector3D (차량 전방 벡터)
        wp:     현재 차량이 있는 차선의 Waypoint (없으면 None)
        return: (best_dist, best_sw_loc, best_light) or (nan, None, None)
        """
        try:
            tls = self.world.get_actors().filter('traffic.traffic_light')
        except Exception:
            return float('nan'), None, None

        # 전방 판단/횡오프셋 계산을 위한 전방 단위벡터(2D)
        f = ego_fwd
        f2 = math.hypot(f.x, f.y) or 1.0
        fx, fy = f.x / f2, f.y / f2

        best_d = float('inf')
        best_sw_loc = None
        best_light = None

        for tl in tls:
            try:
                for sw in tl.get_stop_waypoints():
                    sw_loc = sw.transform.location
                    relx = sw_loc.x - ego_loc.x
                    rely = sw_loc.y - ego_loc.y
                    # (1) 전방 필터: 전방 내적 > 0
                    if (fx * relx + fy * rely) <= 0:
                        continue
                    # (2) 횡방향 오프셋(전방 벡터에 수직 성분) 제한
                    lateral = abs(fx * rely - fy * relx)
                    if lateral > lateral_tol:
                        continue
                    # (3) 거리 제한
                    d = math.hypot(relx, rely)
                    if d <= max_dist and d < best_d:
                        # 같은 도로 우선(선택) — 엄격히 하고 싶으면 가중치 주기
                        if wp and sw.road_id != wp.road_id:
                            # road_id 다르면 약간 페널티를 주고 계속 비교
                            d *= 1.05
                        best_d = d
                        best_sw_loc = sw_loc
                        best_light = tl
            except Exception:
                continue

        if best_sw_loc is None:
            return float('nan'), None, None
        return best_d, best_sw_loc, best_light


# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot):
        # Keep a reference to world for speed feedback (coast controller)
        self._world = world
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
            # ▶ 시작부터 Drive 모드 보장
            self._control.manual_gear_shift = False   # 자동 변속 유지(기어 필드는 로직 게이트용)
            self._control.gear = 1
            self._control.reverse = False
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))

        self._left_blinker_on = False
        self._right_blinker_on = False
        self._left_button_prev = False
        self._right_button_prev = False
        
        # 와이퍼 상태 초기화
        self._wiper_level = 0  # 0: OFF, 1: LOW, 2: MEDIUM, 3: HIGH
        self._l3_button_prev = False

        # ── 페달 데드존(노이즈 무시)
        self._throttle_deadzone = 0.02
        self._brake_deadzone    = 0.08   # G29 캘리브가 약간 올라오는 경우 대비

        # --- Coast/Hold (quasi-cruise) state ---
        self._prev_throttle_raw = 0.0
        self._coast_hold_active = False
        self._coast_target_kph = 0.0
        self._coast_until_wall = 0.0
        # Tunables
        self._coast_min_throttle = 0.15
        self._coast_kp = 0.05
        self._coast_max_throttle = 0.40
        self._coast_hold_seconds = 50.0

        # --- Creep (idle crawl like real AT car) ---
        self._creep_enabled = True
        self._creep_target_kph = 8.0       # ← 원하면 4~10으로 조절
        self._creep_min_throttle = 0.08    # ↑ 정지 마찰/구름저항을 확실히 극복
        self._creep_kp = 0.05              # ↑ 목표보다 느리면 더 민첩하게 받쳐줌
        self._creep_max_throttle = 0.30
        self._creep_deadband_kph = 0.05
        # 정지 탈출 킥(anti-stiction): 완전 정지 근처에서 잠깐 힘줘 출발
        self._creep_kick_throttle = 0.18   # 18% 이상 주입
        self._creep_kick_kph = 0.5         # 0.5 km/h 미만일 때 킥


    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                elif event.button == 1:
                    world.hud.toggle_info()
                elif event.button == 2:
                    world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 23:
                    world.camera_manager.next_sensor()

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
        # 🎮 여기서부터는 매 프레임마다 버튼 상태 체크 (깜빡이 토글용)
        jsButtons = [float(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons())]

        # 🔺 왼쪽 깜빡이: 버튼 5
        if jsButtons[5]:
            if not hasattr(self, '_left_button_prev') or not self._left_button_prev:
                self._left_blinker_on = not getattr(self, '_left_blinker_on', False)
                self._right_blinker_on = False
                if self._left_blinker_on:
                    world.player.set_light_state(carla.VehicleLightState.LeftBlinker)
                else:
                    world.player.set_light_state(carla.VehicleLightState.NONE)
            self._left_button_prev = True
        else:
            self._left_button_prev = False

        # 🔺 오른쪽 깜빡이: 버튼 4
        if jsButtons[4]:
            if not hasattr(self, '_right_button_prev') or not self._right_button_prev:
                self._right_blinker_on = not getattr(self, '_right_blinker_on', False)
                self._left_blinker_on = False
                if self._right_blinker_on:
                    world.player.set_light_state(carla.VehicleLightState.RightBlinker)
                else:
                    world.player.set_light_state(carla.VehicleLightState.NONE)
            self._right_button_prev = True
        else:
            self._right_button_prev = False

        # 🔄 와이퍼 제어: L3 버튼 (11번)
        if jsButtons[11]:  # L3 버튼
            if not self._l3_button_prev:  # 한 번 눌렀을 때만 토글
                self._wiper_level = 0 if self._wiper_level == 1 else 1  # 0↔1 토글
                
                # 와이퍼 단계에 따른 메시지 출력
                wiper_messages = {
                    0: "Wiper: OFF",
                    1: "Wiper: ON"
                }
                
                message = wiper_messages[self._wiper_level]
                world.hud.notification(message, seconds=2.0)
                print(f"🔄 {message}")
                
            self._l3_button_prev = True
        else:
            self._l3_button_prev = False

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

        # === 자율주행 R2 버튼 제어 ===
        jsButtons = [float(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons())]

        if not hasattr(self, '_r2_was_pressed'):
            self._r2_was_pressed = False

        if jsButtons[6]:  # R2 눌렸을 때
            if not self._r2_was_pressed:  # 한 번 눌렀을 때만 토글
                self._autopilot_enabled = not self._autopilot_enabled
                world.player.set_autopilot(self._autopilot_enabled)
                msg = "Autopilot ON" if self._autopilot_enabled else "Autopilot OFF"
                world.hud.notification(msg)
            self._r2_was_pressed = True
        else:
            self._r2_was_pressed = False

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 0.5  # 0.55
        steerCmd = K1 * jsInputs[self._steer_idx]

        K2 = 1.4  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        # 기어 변속: 위쪽(14) → Drive / 아래쪽(15) → Reverse
        if jsButtons[14]:  # 위쪽 Paddle → Drive
            self._control.gear = 1
            self._control.reverse = False
        elif jsButtons[15]:  # 아래쪽 Paddle → Reverse
            self._control.gear = -1
            self._control.reverse = True

        # ---- Steering applies directly
        self._control.steer = steerCmd

        # ---- Coast/Hold logic (keeps speed when you release the pedal)
        raw_throttle = throttleCmd
        raw_brake = brakeCmd
        thr_off = (raw_throttle < self._throttle_deadzone)
        brk_off = (raw_brake    < self._brake_deadzone)

        # Detect throttle release (pressed -> released)
        just_released = (self._prev_throttle_raw > 0.05 and thr_off)

        now_wall = time.time()
        try:
            v = self._world.player.get_velocity()
            speed_kph = 3.6 * math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            # 전/후 진행 부호가 있는 "전방 속도" (후진 롤백 보정용)
            fwd = self._world.player.get_transform().get_forward_vector()
            spd_signed = (v.x*fwd.x + v.y*fwd.y + v.z*fwd.z)  # m/s (전방 +, 후방 -)
            spd_signed_kph = 3.6 * spd_signed
        except Exception:
            speed_kph = 0.0
            spd_signed_kph = 0.0

        if just_released and brk_off:
            # Hold current speed (clamped) or at least 20 km/h (creep preference)
            target = max(20.0, min(speed_kph, 60.0))
            self._coast_target_kph = target
            self._coast_hold_active = True
            self._coast_until_wall = now_wall + self._coast_hold_seconds

        # Cancel coast when user intervenes or timer ends
        if (not thr_off) or (not brk_off) or (now_wall > self._coast_until_wall):
            self._coast_hold_active = False

        inj = 0.0
        if self._coast_hold_active:
            err = self._coast_target_kph - speed_kph  # km/h
            if err > 0.3:  # only add when dropping below target
                inj = self._coast_min_throttle + self._coast_kp * err
                inj = max(0.0, min(self._coast_max_throttle, inj))

        # --- Creep injection (only when no pedal & no brake & gear >= 1) ---
        creep_inj = 0.0
        if (self._creep_enabled
            and thr_off and brk_off
            and not self._control.reverse
            and getattr(self._control, "gear", 0) >= 1):
            # 목표보다 느리거나(정지 포함) 후방으로 밀리면(언덕 롤백) 앞으로 밀어줌
            err_c = self._creep_target_kph - spd_signed_kph  # 전방 기준 오차
            if err_c > self._creep_deadband_kph:
                creep_inj = self._creep_min_throttle + self._creep_kp * err_c
                creep_inj = max(0.0, min(self._creep_max_throttle, creep_inj))
            # 정지 탈출 킥: 아주 느리면 잠깐 크게 밀어 초기 가속 확보
            if speed_kph < self._creep_kick_kph:
                creep_inj = max(creep_inj, self._creep_kick_throttle)

        # Final brake/throttle with both injections
        self._control.brake = raw_brake
        self._control.throttle = max(raw_throttle, inj, creep_inj)

        # Remember for next frame
        self._prev_throttle_raw = raw_throttle

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_walker_keys(self, keys, milliseconds):
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
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
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
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        
        # 깜빡이 상태 변수
        self._blinker_left  = False
        self._blinker_right = False
        self._blink_visible = True
        self._last_blink_toggle_ms = 0

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
            
        # ─ Read player's blinker state
        try:
            ls = world.player.get_light_state()
            self._blinker_left  = bool(ls & carla.VehicleLightState.LeftBlinker)
            self._blinker_right = bool(ls & carla.VehicleLightState.RightBlinker)
        except Exception:
            self._blinker_left = self._blinker_right = False

        # Blink toggle (every 500ms)
        now_ms = pygame.time.get_ticks()
        if now_ms - self._last_blink_toggle_ms > 500:
            self._blink_visible = not self._blink_visible
            self._last_blink_toggle_ms = now_ms
            
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.audi.tt')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.world.get_map().name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']           

        if hasattr(world, 'current_route_index') and world.current_route_index < len(world.route):
            self._info_text.append(f'Direction: {world.nav_direction}')
            self._info_text.append(f'Nav: {world.nav_waypoint_info} | Distance: {world.nav_distance:.1f} m')

        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
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
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        
        # 깜빡이 표시
        if self._blinker_left and self._blinker_right:
            text = 'Indicator: HAZARD ' + ('◀▶' if self._blink_visible else '  ')
        elif self._blinker_left:
            text = 'Indicator: LEFT  ' + ('◀◀' if self._blink_visible else '  ')
        elif self._blinker_right:
            text = 'Indicator: RIGHT ' + ('▶▶' if self._blink_visible else '  ')
        else:
            text = 'Indicator: OFF'
        self._info_text.append(text)
        
        # 와이퍼 레벨 표시
        if hasattr(world, 'controller'):
            lvl = getattr(world.controller, "_wiper_level", 0)
            self._info_text.append(f'Wiper: {"ON" if lvl else "OFF"}')

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
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
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

        world = self._parent.get_world()
        # world 에 아직 속성이 없다면 한 번만 만들어 둠
        if not hasattr(world, 'collision_state'):
            world.collision_state = {'other_id': -1, 'impulse': 0.0}
            world.collision_state['other_id'] = event.other_actor.id
            world.collision_state['impulse']  = intensity


        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    """Spawns sensor.other.lane_invasion on the parent and logs events to world's queue."""
    def __init__(self, world_ctx, parent_actor):
        # world_ctx may be a wrapper (with .world) or the carla.World itself
        self._ctx = world_ctx
        self._world = getattr(world_ctx, "world", world_ctx)
        self._parent = parent_actor
        self.sensor = None

        bp = self._world.get_blueprint_library().find('sensor.other.lane_invasion')
        # Spawn sensor slightly above the roof to avoid collisions with geometry
        transform = carla.Transform(carla.Location(z=2.0))
        self.sensor = self._world.spawn_actor(bp, transform, attach_to=self._parent)

        # Start listening
        self.sensor.listen(self._on_invasion)

    def destroy(self):
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()
            self.sensor = None

    def _on_invasion(self, event: carla.LaneInvasionEvent):
        """Push (tsec, types, colors) into the world's queue and update latch."""
        # --- HUD popup (same as sub2) ---
        try:
            lane_types_set = set(x.type for x in event.crossed_lane_markings)
            # enum 이름을 예쁘게 뽑아 "solid", "broken" 등으로
            text = [getattr(t, 'name', str(t)) for t in lane_types_set]
            hud = getattr(self, 'hud', None) or getattr(getattr(self, '_ctx', None), 'hud', None)
            if hud:
                hud.notification('Crossed lane : ' + ' and '.join([s.lower() for s in text]))
        except Exception:
            pass
        
        # Resolve the logging world/context
        ctx = self._ctx
        world = getattr(ctx, "world", None) or self._world

        # Collect marking types/colors safely
        types = []
        colors = []
        for m in event.crossed_lane_markings:
            # Marking type name (fallback to str)
            tname = getattr(m.type, "name", str(m.type))
            types.append(tname)
            # Marking color name if available
            cname = getattr(getattr(m, "color", None), "name", "")
            if cname:
                colors.append(cname)

        # Simulation time (seconds)
        try:
            tsec = world.get_snapshot().timestamp.elapsed_seconds
        except Exception:
            tsec = 0.0

        # Push to queue
        q = getattr(ctx, "_lane_inv_queue", None)
        if q is not None:
            q.append((tsec, types, colors))

        # Update latch so periodic logger won't miss brief events
        latch = getattr(ctx, "_lane_inv_latch", None)
        latch_window = getattr(ctx, "_lane_inv_latch_window", 0.5)
        if latch is not None:
            ctx._lane_inv_latch = {
                "tsec": tsec,
                "types": types,
                "colors": colors,
                "until": tsec + float(latch_window)
            }

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
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
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        # ▶ Quality up a bit from 1280x720: 1536x864 (16:9, +44% pixels)
        INTERNAL_W = 1536
        INTERNAL_H = 864
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(INTERNAL_W))
                bp.set_attribute('image_size_y', str(INTERNAL_H))
                bp.set_attribute('sensor_tick', '0.033')  # keep 30fps first; if heavy → 0.04~0.05
                # (Note) Using ColorConverter.Raw so no additional post-processing cost
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
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
            # ▶ Scale up internal 1536x864 image to full screen size for blit
            scaled = pygame.transform.smoothscale(self.surface, self.hud.dim)
            # If performance is heavy, use: pygame.transform.scale(self.surface, self.hud.dim)  # Faster but slightly rougher edges
            display.blit(scaled, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data) # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
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
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    pygame.joystick.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        client.load_world("Town01")

        display = pygame.display.set_mode(
            (0,0), pygame.FULLSCREEN)
        args.width, args.height = pygame.display.get_surface().get_size()

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args.filter)
        
        controller = DualControl(world, args.autopilot)
        world.controller = controller   # <-- 연결!

        clock = pygame.time.Clock()

        # === NPC Vehicle Generation ===
        vehicles_list = []
        bp_library = world.world.get_blueprint_library()
        all_bps = bp_library.filter('vehicle.*')

        # List of large vehicle IDs to exclude
        exclude = [
            'vehicle.carlamotors.carlacola',      # Cola truck (used only in spawn_truck_ahead)
            'vehicle.carlamotors.european_truck', # European truck
            'vehicle.tesla.cybertruck',           # Cybertruck
            'vehicle.carlamotors.firetruck',      # Fire truck
            'vehicle.ford.ambulance',             # Ambulance
            'vehicle.mercedes.sprinter',          # Sprinter
            'vehicle.volkswagen.t2',              # T2 bus
            'vehicle.carlamotors.firetruck',      # Fire truck
            'vehicle.carlamotors.truck',          # Regular truck
            'vehicle.carlamotors.box_truck',      # Box truck
            'vehicle.carlamotors.delivery_truck', # Delivery truck
            'vehicle.carlamotors.flatbed_truck',  # Flatbed truck
            'vehicle.carlamotors.garbage_truck',  # Garbage truck
            'vehicle.carlamotors.tanker_truck',   # Tanker truck
            'vehicle.carlamotors.utility_truck',  # Utility truck
            'vehicle.carlamotors.van',            # Van
            'vehicle.carlamotors.box_van',        # Box van
            'vehicle.carlamotors.delivery_van',   # Delivery van
            'vehicle.carlamotors.utility_van',    # Utility van
            'vehicle.carlamotors.work_van',       # Work van
            'vehicle.carlamotors.ambulance',      # Ambulance
            'vehicle.carlamotors.police',         # Police car
            'vehicle.carlamotors.police_van',     # Police van
            'vehicle.carlamotors.fire_truck',     # Fire truck
            'vehicle.carlamotors.emergency_truck' # Emergency vehicle
        ]

        # Use only remaining ones after excluding the list (passenger cars and small vehicles only)
        blueprints = [bp for bp in all_bps if bp.id not in exclude]
        
        spawn_points = world.world.get_map().get_spawn_points()
        num_vehicles = 10  # Reduced from 20 to 10 (triple monitor performance optimization)

        random.shuffle(spawn_points)
        for i in range(num_vehicles):
            if i >= len(spawn_points):
                break
            bp = random.choice(blueprints)
            vehicle = world.world.try_spawn_actor(bp, spawn_points[i])
            if vehicle is not None:
                try:
                    vehicle.set_autopilot(True)  # TrafficManager 제어 받음
                    vehicles_list.append(vehicle)
                except RuntimeError:
                    print(f"NPC vehicle {i} autopilot setup failed")
                    if vehicle.is_alive:
                        vehicle.destroy()

        print(f" Spawned {len(vehicles_list)} NPC vehicles (passenger cars and small vehicles only).")

        while True:
            # Lower to 50Hz for performance optimization (considering triple monitor environment)
            clock.tick_busy_loop(50)  # Reduced from 60Hz to 50Hz for additional optimization
            if controller.parse_events(world, clock):
                return
            
            # 🌧️ Automatic weather change check
            try:
                world.auto_weather_change()
            except Exception as e:
                print(f"[ERROR] Weather change error: {e}")
            
            try:
                world.tick(clock)
                world.render(display)
                pygame.display.flip()
            except Exception as e:
                print(f"[ERROR] Game loop error: {e}")
                break

    finally:

        # Clean up vehicles
        if 'vehicles_list' in locals():
            for v in vehicles_list:
                if v.is_alive:
                    try:
                        v.destroy()
                    except RuntimeError:
                        pass

        if world is not None:
            world.destroy()
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\n📊 KeyboardInterrupt detected - saving log data before exit...')
        if 'world' in locals() and world is not None:
            world.destroy()
        print('✅ Safely terminated. Bye!')


if __name__ == '__main__':

    main()
