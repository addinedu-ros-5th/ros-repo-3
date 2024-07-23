import math
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped, TaskResult
from geometry_msgs.msg import Quaternion
import time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

class AStarPlanner:
    def __init__(self, resolution, rr, padding):
        self.resolution = resolution
        self.rr = rr
        self.padding = padding
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.map_resolution = 1
        self.map_origin = (0, 0)
        ox, oy = self.load_map()
        self.calc_obstacle_map(ox, oy)
        print("Map loading done!")

    def load_map(self):
        print("Loading map start!")
        map_yaml_file = os.path.join(get_package_share_directory('minibot_navigation2'), 'maps', 'map.yaml')
        map_yaml_data = yaml.full_load(open(map_yaml_file))
        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin']
        map_pgm_file = os.path.join(get_package_share_directory('minibot_navigation2'), 'maps', map_yaml_data['image'])
        
        with open(map_pgm_file, 'rb') as pgmf:
            pgm_data = pgmf.readlines()
            pgm_data = [line for line in pgm_data if not line.startswith(b'#')]
            map_width, map_height = map(int, pgm_data[1].split())
            map_data = np.array(list(map(int, pgm_data[3])))
            map_data[map_data <= 210] = 0
            map_data[map_data > 210] = 100
            map_data = map_data.reshape((map_height, map_width))
            map_data = np.flip(map_data, axis=0)
        
        ox, oy = [], []
        padded_map_data = map_data.copy()
        for i in range(map_height):
            for j in range(map_width):
                if map_data[i][j] == 0:
                    ox.append(j)
                    oy.append(i)
                    for dx in range(-self.padding, self.padding + 1):
                        for dy in range(-self.padding, self.padding + 1):
                            if 0 <= j + dx < map_width and 0 <= i + dy < map_height and padded_map_data[i + dy][j + dx] != 0:
                                padded_map_data[i + dy][j + dx] = 0
                                ox.append(j + dx)
                                oy.append(i + dy)
        return ox, oy

    class Node:
        def __init__(self, x, y, cost, parent_index, vector=None):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
            self.vector = vector

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx_real, sy_real, gx_real, gy_real):
        sx = (sx_real - self.map_origin[0]) / self.map_resolution
        sy = (sy_real - self.map_origin[1]) / self.map_resolution
        gx = (gx_real - self.map_origin[0]) / self.map_resolution
        gy = (gy_real - self.map_origin[1]) / self.map_resolution
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        is_starting = True
        
        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_manhattan(goal_node, open_set[o]))
            current = open_set[c_id]
            
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.vector = current.vector
                break
            
            del open_set[c_id]
            closed_set[c_id] = current
            
            for i, _ in enumerate(self.motion):
                is_turned = 0
                before_vector = (0, 0)
                now_vector = (self.motion[i][0], self.motion[i][1])
                
                if closed_set.get(current.parent_index):
                    before_node = closed_set[current.parent_index]
                    before_vector = (current.x - before_node.x, current.y - before_node.y)
                    is_turned = before_vector != now_vector
                
                node = self.Node(current.x + self.motion[i][0], current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * (1 + is_turned), c_id, now_vector)
                
                n_id = self.calc_grid_index(node)
                
                if (not is_starting) and (not self.verify_node(node)):
                    continue
                
                if n_id in closed_set:
                    continue
                
                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
                
                is_starting = False
        
        rx, ry, tpx, tpy, tvec_x, tvec_y = self.calc_final_path(goal_node, closed_set)
        
        for i in range(len(tpx)):
            tpx[i] = (tpx[i] * self.map_resolution) + self.map_origin[0]
            tpy[i] = (tpy[i] * self.map_resolution) + self.map_origin[1]
        
        print(tpx, tpy, tvec_x, tvec_y)
        return tpx, tpy

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        tpx, tpy = [], []
        tvec_x, tvec_y = [], []
        parent_index = goal_node.parent_index
        now_node = goal_node
        before_vector = (0, 0)
        now_vector = (0, 0)
        
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            is_turned = 0
            now_vector = (n.x - now_node.x, n.y - now_node.y)
            is_turned = now_vector != before_vector
            
            if is_turned:
                tpx.append(now_node.x)
                tpy.append(now_node.y)
                tvec_x.append(now_node.vector[0])
                tvec_y.append(now_node.vector[1])
            
            parent_index = n.parent_index
            now_node = n
            before_vector = now_vector
        
        return rx, ry, tpx[::-1], tpy[::-1], tvec_x[::-1], tvec_y[::-1]

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    @staticmethod
    def calc_manhattan(n1, n2):
        d = abs(n1.x - n2.x) + abs(n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (self.calc_xy_index(node.x, self.min_x) - self.min_x) * self.x_width + (self.calc_xy_index(node.y, self.min_y) - self.min_y)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)
        
        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False
        
        if self.obstacle_map[node.x][node.y]:
            return False
        
        return True

    def calc_obstacle_map(self, ox, oy):
        print("Calc Obstacle...")
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        #print(f'Setting obstacle at ({ix}, {iy}) due to proximity to obstacle point ({iox}, {ioy})')
                        break

        # Optional: Visualize or log the obstacle map to verify correctness
        for row in self.obstacle_map:
            print(row)

    @staticmethod
    def get_motion_model():
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
        ]
        return motion

'''def main():
    print("A* Path Planning with ROS2")
    rclpy.init()
    node = rclpy.create_node('a_star_path_planning')
    
    threshold_distance = 0.02
    
    
    # 목적지 좌표 설정
    targets = {
        '집품 1-1': {'x': 1.185, 'y': 1.456, 'z': -0.003, 'w': 0.999},
        '집품 1-2': {'x': 1.15, 'y': 1.186, 'z': 0.027, 'w': 0.999},
        '집품 2-1': {'x': 1.325, 'y': 0.709, 'z': 0.046, 'w': 0.999},
        '집품 2-2': {'x': 1.250, 'y': 0.438, 'z': -0.011, 'w': 0.999},
        '집품 3-1': {'x': 0.305, 'y': 1.366, 'z': 0.997, 'w': 0.073},
        '집품 3-2': {'x': 0.427, 'y': 1.265, 'z': 0.998, 'w': 0.061},
        '집품 4-1': {'x': 0.216, 'y': 0.746, 'z': 0.999, 'w': 0.045},
        '집품 4-2': {'x': 0.288, 'y': 0.473, 'z': 0.999, 'w': 0.046},
        '포장소': {'x': 0.719, 'y': -0.586, 'z': -0.665, 'w': 0.747},
        '출고소 1': {'x': 1.328, 'y': -1.581, 'z': -0.668, 'w': 0.743},
        '출고소 2': {'x': 0.132, 'y': -1.761, 'z': -0.656, 'w': 0.754},
        '충전소': {'x': 0.000, 'y': 0.000, 'z': 0.000, 'w': 0.999}
    }
    

    for target_name, target_pose in targets.items():
        print(f"Moving to {target_name} at position ({target_pose['x']}, {target_pose['y']})...")
        
        # AStarPlanner 인스턴스 생성
        a_star = AStarPlanner(resolution=1, rr=1, padding=3)
        
        # 시작 좌표와 목표 좌표 설정
        sx_real, sy_real = 0.0, 0.0  # 시작 지점 좌표
        gx_real, gy_real = target_pose['x'], target_pose['y']  # 목표 지점 좌표
        
        # 경로 계획
        tpx, tpy = a_star.planning(sx_real, sy_real, gx_real, gy_real)
        
        # ROS2 네비게이터 인스턴스 생성
        nav = BasicNavigator()
        nav.waitUntilNav2Active()
        
        # 경로 생성
        path = []
        for x, y in zip(tpx, tpy):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = nav.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = Quaternion(w=1.0)
            path.append(pose)
        
        # 경로 따라가기
        nav.followWaypoints(path)
        
        # 경로 완료까지 대기
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:

                # 현재 경유지와 목표 지점 간의 거리 계산
                current_waypoint_index = feedback.current_waypoint
                if current_waypoint_index < len(path):
                    current_pose = path[current_waypoint_index].pose.position
                    remaining_distance = math.hypot(
                        target_pose['x'] - current_pose.x,
                        target_pose['y'] - current_pose.y
                    )
                    print(f"Distance remaining: {remaining_distance:.2f} meters")
                    
                    if remaining_distance <= threshold_distance:
                        print("Arrived at the Target")
                        break
        
        # 최종 결과 확인
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Reached {target_name} successfully!")
        else:
            print(f"Failed to reach {target_name}.")
        
        time.sleep(0.5)  # 1초 대기
    
    # 종료
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''
