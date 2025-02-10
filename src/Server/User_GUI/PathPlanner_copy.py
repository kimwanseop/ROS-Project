import heapq
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation
from scipy.interpolate import interp1d
from scipy.interpolate import splprep, splev

# 맵에서 상하좌우로 이동할 수 있는 방향을 나타내는 리스트

class PathPlanning():
    def __init__(self, map_path='./Images/asap_map_resized.pgm', middle_path='./Images/asap_map_middleLine.png'):
        self.MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        self.map = cv2.imread(map_path, cv2.COLOR_BGR2GRAY)
        self.middle = cv2.imread(middle_path, cv2.IMREAD_GRAYSCALE)
        self.grid = None 
        self.background = None
        crop_size = 16
        self.map = self.map[crop_size+2:-crop_size-2, crop_size:-crop_size]
        self.middle = self.middle[crop_size+2:-crop_size-2, crop_size:-crop_size]
        self.start, self.goal = None, None 

        self.min1_take = None
        self.min2_take = None
        self.one_way = None
        self.exc_sig = None
        # self.call_sig = None

        # wp number
        wp_num = {
            0 : (110, 237),
            1 : (88, 261),
            2 : (88, 275),
            3 : (58, 196),
            4 : (45, 196),
            5 : (44, 140),
            6 : (31, 140),
            7 : (37, 42),
            8 : (107, 28),
            9 : (178, 42),
            10 : (187, 140),
            11 : (170, 140),
            12 : (187, 196),
            13 : (171, 196),
            14 : (142, 276),
            15 : (142, 261),
            16 : (73, 173),
            17 : (73, 158),
            18 : (155, 173),
            19 : (155, 158),
            20 : (85,96),
            21 : (85, 46),
            22 : (150, 97),
            23 : (121, 237),
            24 : (44,95),
            25 : (150, 46),
            # 일방 통행 좌표용
            30 : (88, 107),
            31 : (37, 56),
            
            # 조건문 좌표용
            32 : (90, 0)
        }

        for key, value in wp_num.items():
            wp_num[key] = (int(value[0] - crop_size), int(value[1]) - crop_size-2)

        self.wp_num = wp_num

        # Waypoint 그래프
        self.waypoint_graph = {
            wp_num[0]: [wp_num[1], wp_num[14]],  # 0번 : 1, 14
            wp_num[1]: [wp_num[3]],  # 1번 : 3
            wp_num[2]: [wp_num[14], wp_num[23]],  # 2번 : 14, 23
            wp_num[3]: [wp_num[16], wp_num[20], wp_num[21]],  # 3번 : 16, 20
            wp_num[4]: [wp_num[2]],  # 4번 : 2
            wp_num[5]: [wp_num[16]],  # 5번 : 16
            wp_num[6]: [wp_num[4]],  # 6번 : 4
            wp_num[7]: [wp_num[6], wp_num[24]],  # 7번 : 6, 24
            wp_num[8]: [wp_num[7]],  # 8번 : 7
            wp_num[9]: [wp_num[8]],  # 9번 : 8
            wp_num[10]: [wp_num[9]],  # 10번 : 9
            wp_num[11]: [wp_num[13], wp_num[19]],  # 11번 : 13, 19
            wp_num[12]: [wp_num[10], wp_num[19]],  # 12번 : 10, 19
            wp_num[13]: [wp_num[15]],  # 13번 : 15
            wp_num[14]: [wp_num[12]],  # 14번 : 12
            wp_num[15]: [wp_num[1], wp_num[23]],  # 15번 : 1, 23
            wp_num[16]: [wp_num[18]],  # 16번 : 18
            wp_num[17]: [wp_num[4], wp_num[20], wp_num[21]],  # 17번 : 4, 20, 21
            wp_num[18]: [wp_num[10], wp_num[13]],  # 18번 : 10, 13
            wp_num[19]: [wp_num[17]],  # 19번 : 17
            wp_num[20]: [wp_num[21], wp_num[22]],  # 20번 : 21, 22
            wp_num[21]: [wp_num[11]],  # 21번 : 11
            wp_num[22]: [wp_num[11]],  # 22번 : 11

            wp_num[23]: [],  # 23번 : 도착 시 행하는 명령어만 따로 만들기

            wp_num[24]: [wp_num[5], wp_num[6]],  # 24번 : 5, 6  
        }

    # 좌표 범위 늘리기
    def expand_coordinates(self, coord, buffer_size):
        x, y = coord
        expanded_area = [(x + dx, y + dy) 
                        for dx in range(-buffer_size, buffer_size + 1) 
                        for dy in range(-buffer_size, buffer_size + 1)]
        return expanded_area

    def expand_multiple_coordinates(self, coord_list, buffer_size):
        expanded_set = set()  # 중복 제거
        for coord in coord_list:
            expanded_set.update(self.expand_coordinates(coord, buffer_size))
        return list(expanded_set)

    # pgm 지도를 이진화 형태로 변환
    def pixel_based_map(self, img):
        img_height, img_width = img.shape 
        map_cost = np.ones((img_height, img_width), dtype=int)

        for i in range(img_height):
            for j in range(img_width):
                if img[i, j] > 210:  
                    map_cost[i, j] = 0 
                else:  
                    map_cost[i, j] = 1 

        return map_cost


    # 가장 적은 비용을 가진 두 wp 가져오기
    def find_two_smallest(self, cost_list):
        min1, min2 = float('inf'), float('inf')  

        for value in cost_list:
            if value < min1:
                min2 = min1  
                min1 = value  
            elif value < min2: 
                min2 = value 

        return min1, min2, cost_list.index(min1), cost_list.index(min2)


    # 장애물 근처에 가중치 맵 생성하기
    def create_weight_map(self, grid, middle, buffer_size, penalty):
        background = grid  # 이진화 된 배열 형태의 전체 map
        overlay = middle  # 이진화 된 배열 형태의 중앙선 map

        background[200:228, 86:89] = 1

        x, y = 18, 19  # 중앙선 map 을 삽입할 위치

        background_result = background.astype('uint8')
        overlay = overlay.astype('uint8')

        h, w = overlay.shape[:2]
        background_roi = background_result[y:y+h, x:x+w]

        result_roi = background_roi - overlay

        background_result[y:y+h, x:x+w] = result_roi
        self.background = background_result 
        # plt.imshow(background_result, cmap='gray')
    
        obstacle = background_result == 1
        # 장애물 주변에 가중치를 부여
        dilated_obstacle = binary_dilation(obstacle, structure=np.ones((buffer_size, buffer_size)))
        weight_map = np.zeros_like(background_result, dtype=float)
        weight_map[dilated_obstacle] = penalty
        return weight_map, background_result

    # 휴리스틱 함수 : 유클리드 거리 계산
    def euclidean_distance(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    def a_star(self, start, goal, waypoint_graph=None, grid=None, weight_map=None):
        # waypoint 기반 전역 경로 pathplanning
        if grid is None and weight_map is None:
            # print('waypoint 기반 전역 경로 pathplanning')
            """ A* 알고리즘을 사용하여 start에서 goal까지 최단 경로 찾기 """
            open_set = []  # 우선순위 큐 (힙)
            heapq.heappush(open_set, (0, start))  # (f_score, node)
            
            came_from = {}  # 경로 추적을 위한 딕셔너리
            g_score = {node: float('inf') for node in waypoint_graph}  # 시작점에서의 거리
            g_score[start] = 0
            f_score = {node: float('inf') for node in waypoint_graph}  # 휴리스틱 거리 포함 점수
            f_score[start] = self.euclidean_distance(start, goal)
            
            while open_set:
                _, current = heapq.heappop(open_set)  # f_score가 가장 낮은 노드 선택
                
                if current == goal:
                    # 최단 경로를 재구성하여 반환
                    path = []
                    while current in came_from:
                        path.append(current)
                        current = came_from[current]
                    path.append(start)
                    return path[::-1]  # 역순으로 반환
                
                for neighbor in waypoint_graph[current]:
                    tentative_g_score = g_score[current] + self.euclidean_distance(current, neighbor)
                    
                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.euclidean_distance(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
            
            return None  # 경로가 없는 경우
        # 두 waypoint 사이 pathplanning
        else:
            # print('waypoint 사이의 거리 pathplanning')
            open_set = []
            heapq.heappush(open_set, (0 + self.euclidean_distance(start, goal), 0, start))
            g_costs = {start: 0}

            came_from = {}

            while open_set:
                _, current_cost, current = heapq.heappop(open_set)

                if current == goal:
                    path = []
                    while current in came_from:
                        path.append(current)
                        current = came_from[current]
                    path.append(start)
                    path.reverse()

                    # goal까지의 총 비용 출력
                    # print(f"Total cost from {start} to {goal}: {g_costs[goal]}")
                    return path, g_costs[goal]
                
                for move in self.MOVES:
                    neighbor = (current[0] + move[0], current[1] + move[1])
                    if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and grid[neighbor] == 0:
                        tentative_g_cost = current_cost + 1 + weight_map[neighbor]  # 가중치 추가
                        if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                            g_costs[neighbor] = tentative_g_cost
                            f_cost = tentative_g_cost + self.euclidean_distance(neighbor, goal)
                            # f_cost = tentative_g_cost + manhattan_distance(neighbor, goal)
                            heapq.heappush(open_set, (f_cost, tentative_g_cost, neighbor))
                            came_from[neighbor] = current
            return None

    # def valid_start_goal(self, coord):
    #     if (20 < coord[0] < 73) and (133 < coord[1] < 195):
    #         # print('호출 및 주정차 금지 구역 입니다.')
    #         return False
    #     else:
    #         # print('유효한 좌표')
    #         return True

    def find_start_goal(self, coord, waypoint_graph, grid, weight_map, flag=None):
        # global min1_take, min2_take, one_way, exc_sig
        cost_list = []
        for station in waypoint_graph:
            _, g_cost = self.a_star((coord[1], coord[0]), (station[1], station[0]), waypoint_graph, grid, weight_map)
            cost_list.append(g_cost)
        
        min1, min2, min1_idx, min2_idx = self.find_two_smallest(cost_list)

        # start 좌표를 줬을 때
        if flag is None:
            if (self.wp_num[30][0] < self.start[1] < self.wp_num[30][1] and self.wp_num[20][0] < self.start[0] < self.wp_num[22][0]):  # 일방 통행 구역에 start 가 있으면
                print('start 가 일방통행 구역 안에 있으므로 따로 처리')
                new_coord = self.wp_num[22]
                self.one_way = 1
                return new_coord
            elif (self.wp_num[31][0] < self.start[1] < self.wp_num[31][1] and self.wp_num[21][0] < self.start[0] < self.wp_num[25][0]):  # 다른 일방 통행 구역에 start 가 있으면
                print('start 가 일방통행 구역 안에 있으므로 따로 처리')
                new_coord = self.wp_num[25]
                self.one_way = 1
                return new_coord
            else:
                print(f"최소 비용 wp from start : {min1_idx}번, 비용 : {min1}")
                print(f"두 번째로 작은 비용 wp from start : {min2_idx}번, 비용 : {min2}")

                # if (min1_idx == 20 and min2_idx == 21 and start[1] < 90) or (min1_idx == 5 and min2_idx == 20):
                if (min1_idx == 20 and min2_idx == 21 and self.start[1] < self.wp_num[32][0]):   
                    new_coord, idx = self.wp_num[min2_idx], min2_idx
                    self.min2_take = 1
                    print('min2 선택!(start)')
                    return new_coord
                else:
                    # new_coord, idx = wp_num[min1_idx], min1_idx
                    # min1_take = 1
                    # print('min1 선택! (start)')
                    # if (min1_idx == 20 and min2_idx == 5) or (min1_idx == 5 and min2_idx == 20) or (min1_idx == 5 and min2_idx == 17) or (min1_idx == 17 and min2_idx == 5):
                    if (min1_idx == 20 and min2_idx == 17) or (min1_idx == 17 and min2_idx == 20) or (min1_idx == 5 and min2_idx == 17) or (min1_idx == 17 and min2_idx == 5):
                        self.exc_sig = 1
                        new_coord = self.wp_num[20]
                        print('(5번, 20번), (5번, 17번) 예외처리')
                        return new_coord
                    else:
                        new_coord, idx = self.wp_num[min1_idx], min1_idx
                        self.min1_take = 1
                        print('min1 선택! (start)')
                        return new_coord
        
        # goal 좌표를 줬을 때
        else:
            print(f"최소 비용 wp from goal : {min1_idx}번, 비용 : {min1}")
            print(f"최소 비용 wp from goal : {min2_idx}번, 비용 : {min2}")

            if min1_idx == 5 and min2_idx == 20:
                new_coord, idx = self.wp_num[min2_idx], min2_idx
                print('min2 선택 (goal)')
                return new_coord
            else:
                new_coord, idx = self.wp_num[min1_idx], min1_idx
                print('min1 선택 (goal)')
                return new_coord
        # 주 지도
        
    # return_sig 은 GUI 로부터 받아오는 값 (사용자가 차량을 반납할 때)
    # call_sig 도 GUI 로부터 받아오는 값 (최초 호출 시 발생하는 신호로 생각)
    def generate_waypoint(self, start_point, goal_point, call_sig=None, return_sig=None):
        # 지도 변환 pgm -> 이진 배열 (0: 도로, 1: 장애물)
        grid = self.pixel_based_map(self.map)
        self.grid = grid 
        
        # 최초 호출, 반납 여부
        # return_sig 는 GUI 로부터 받아야 함.
        # call_sig = None  # 사용자가 최초 호출 시 해당 시그널 1 을 받아오기
        # return_sig = None

        # 테스트 실행
        start = start_point  # 시작 좌표
        goal = goal_point  # 목표 좌표
        
        self.start = start 
        self.goal = goal

        if return_sig is not None:
            self.goal = self.wp_num[23]  # 목표 좌표
        # 장애물 주변 가중치 추가
        weight_map, background_result = self.create_weight_map(grid, self.middle, buffer_size=10, penalty=7)
        
        # 시작점, 출발점 보간
        new_start = self.find_start_goal(start, self.waypoint_graph, grid, weight_map)
        new_goal = self.find_start_goal(goal, self.waypoint_graph, grid, weight_map, flag=1)

        if (new_start != new_goal or self.exc_sig is not None):
            wp_path = self.a_star(new_start, new_goal, self.waypoint_graph)

            s = wp_path[-2]
            g = wp_path[-1]

            path_result, _ = self.a_star((s[1], s[0]), (g[1], g[0]), self.waypoint_graph, grid, weight_map)

            expended_list = self.expand_multiple_coordinates(path_result, buffer_size=3)

            if (goal[1], goal[0]) in expended_list:
                wp_path[-1] = goal
            else:
                wp_path.append(goal)

            # if new_goal != self.goal:
            #     wp_path[-1] = self.goal
            
            if wp_path:
                linked_path = []
                if len(wp_path) == 1 or self.one_way is not None:
                    wp_path.insert(0, start)
                
                if self.one_way is not None:
                    for_cnt = len(wp_path) - 1
                else:
                    for_cnt = len(wp_path)

                for i in range(for_cnt):
                    if ((i == 0) and (wp_path[i] != start)):
                        if len(wp_path) == 1:
                            wp_start = start
                            wp_goal = wp_path[i]
                        else:
                            if call_sig is not None:
                                if self.exc_sig is not None:
                                    wp_path[0] = start
                                else:
                                    wp_path.insert(0, start)

                            elif (self.min1_take is None and self.min2_take is not None):
                                wp_path.insert(0, new_start)
                            elif (self.min2_take is None and self.min1_take is not None):
                                if self.exc_sig is not None:
                                    wp_path.insert(0, start)
                                else:
                                    wp_path[0] = start

                            wp_start = start
                            wp_goal = wp_path[i + 1]

                    else:
                        try:
                            wp_start = wp_path[i]
                            wp_goal = wp_path[i + 1]
                        except:
                            pass

                    part_path, _ = (self.a_star((wp_start[1], wp_start[0]), (wp_goal[1], wp_goal[0]), self.waypoint_graph, grid, weight_map))
                    real_path = np.array(part_path)

                    sample_cnt = 15

                    if len(real_path) // sample_cnt >= 1 and len(real_path) > sample_cnt:
                        real_path = real_path[::sample_cnt]
                    else:
                        pass

                    if (len(wp_path) != 1) and (tuple(real_path[-1]) != (wp_goal[1], wp_goal[0])):
                        real_path[-1] = (wp_goal[1], wp_goal[0])

                    linked_path.append(real_path)
                    self.init_signal()  # 모든 시그널 초기화
                # 15개씩 샘플링 된 모든 경로 좌표
                return np.concatenate(linked_path), wp_path
            else:
                print('No path found')
        else:
            print('현재 목적지 근처에 있습니다.')
            # 경로 생성 바로 종료하도록 하는 신호
            return None, None


                
    def draw_path(self, real_path):
        # 시작점, 목표점
        plt.imshow(self.grid, cmap='gray')
        plt.imshow(self.background, cmap='gray')

        plt.scatter(self.start[0], self.start[1], color='blue', marker='o', label='Start', s=30)
        plt.scatter(self.goal[0], self.goal[1], color='red', marker='x', label='Goal', s=30)

        path_x = real_path[:, 0]
        path_y = real_path[:, 1]


        m = len(path_x)  # 데이터 개수
        k = (5, m - 1)  # 최소한 m > k 조건을 만족하도록 설정

        # b - spline 보간법
        try:
            tck, u = splprep([path_x, path_y], s=5, k=k)  # s 가 클수록 부드러운 곡선
            u_fine = np.linspace(0, 1, 300)  # 값이 클수록 세밀한 보간
            smooth_x, smooth_y = splev(u_fine, tck)

            plt.plot(smooth_y, smooth_x, 'green', linewidth=2, label='Path')
        except:
            plt.plot(real_path[:, 1], real_path[:, 0], color='green', linewidth=2, label="Path")    

        
    def init_signal(self):
        self.min1_take = None
        self.min2_take = None
        self.one_way = None
        self.exc_sig = None