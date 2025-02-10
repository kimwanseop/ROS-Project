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
    def __init__(self, map_path='Images/asap_map_resized.pgm', middle_path='Images/asap_map_middleLine.png'):
        self.MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        self.map = cv2.imread(map_path, cv2.COLOR_BGR2GRAY)
        self.middle = cv2.imread(middle_path, cv2.IMREAD_GRAYSCALE)
        self.grid = self.pixel_based_map(self.map)

        self.background = None
        crop_size = 16
        self.map = self.map[crop_size+2:-crop_size-2, crop_size:-crop_size]
        self.middle = self.middle[crop_size+2:-crop_size-2, crop_size:-crop_size]
        self.start, self.goal = None, None 

        self.min1_take = None
        self.min2_take = None
        self.one_way = None
        self.exc_sig = None

        # wp number
        wp_num = {
            0 : (108, 237),
            1 : (88, 261),
            2 : (88, 277),
            3 : (60, 196),
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
            14 : (142, 277),
            15 : (142, 261),
            16 : (73, 173),
            17 : (73, 158),
            18 : (155, 173),
            19 : (155, 158),
            20 : (85,96),
            21 : (85, 45),
            22 : (150, 97),
            23 : (121, 237),
            24 : (44,95),
            25 : (150, 45),
            26 : (108, 216),
            # 일방 통행 좌표용
            30 : (88, 107),
            31 : (37, 56)
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
            # wp_num[20]: [wp_num[22]],  # 20번 : 21, 22
            wp_num[21]: [wp_num[25]],  # 21번 : 25
            wp_num[22]: [wp_num[11]],  # 22번 : 11

            wp_num[23]: [],  # 23번 : 도착 시 행하는 명령어만 따로 만들기

            wp_num[24]: [wp_num[5], wp_num[6]],  # 24번 : 5, 6  
            wp_num[25]: [wp_num[11]], # 25번 : 11
            wp_num[26]: [wp_num[0]]  # 26번 : 0
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
    def create_weight_map(self, middle, buffer_size, penalty):
        background = self.grid  # 이진화 된 배열 형태의 전체 map
        overlay = middle  # 이진화 된 배열 형태의 중앙선 map
        
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
        print('here1', weight_map)
        weight_map[dilated_obstacle] = penalty
        print('here2', weight_map)
        return weight_map, background_result

    # 휴리스틱 함수 : 유클리드 거리 계산
    def euclidean_distance(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    def a_star(self, start, goal, weight_map=None):
        # waypoint 기반 전역 경로 pathplanning
        if self.grid is None and weight_map is None:
            # print('waypoint 기반 전역 경로 pathplanning')
            """ A* 알고리즘을 사용하여 start에서 goal까지 최단 경로 찾기 """
            open_set = []  # 우선순위 큐 (힙)
            heapq.heappush(open_set, (0, start))  # (f_score, node)
            
            came_from = {}  # 경로 추적을 위한 딕셔너리
            g_score = {node: float('inf') for node in self.waypoint_graph}  # 시작점에서의 거리
            g_score[start] = 0
            f_score = {node: float('inf') for node in self.waypoint_graph}  # 휴리스틱 거리 포함 점수
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
                
                for neighbor in self.waypoint_graph[current]:
                    tentative_g_score = g_score[current] + self.euclidean_distance(current, neighbor)
                    
                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.euclidean_distance(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
            
            return None, None  # 경로가 없는 경우
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
                    if 0 <= neighbor[0] < self.grid.shape[0] and 0 <= neighbor[1] < self.grid.shape[1] and self.grid[neighbor] == 0:
                        tentative_g_cost = current_cost + 1 + weight_map[neighbor]  # 가중치 추가
                        if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                            g_costs[neighbor] = tentative_g_cost
                            f_cost = tentative_g_cost + self.euclidean_distance(neighbor, goal)
                            # f_cost = tentative_g_cost + manhattan_distance(neighbor, goal)
                            heapq.heappush(open_set, (f_cost, tentative_g_cost, neighbor))
                            came_from[neighbor] = current
            return None, None


    def find_start_goal(self, coord, weight_map, flag=None):
        global min1_take, min2_take, one_way, exc_sig
        cost_list = []
        print('here4', coord, self.waypoint_graph, weight_map, flag)
        for station in self.waypoint_graph:
            print('here5', (coord[1], coord[0]), (station[1], station[0]), weight_map)
            _, g_cost = self.a_star((coord[1], coord[0]), (station[1], station[0]), weight_map)
            cost_list.append(g_cost)
        
        min1, min2, min1_idx, min2_idx = self.find_two_smallest(cost_list)

        # start 좌표를 줬을 때
        if flag is None:
            if (self.wp_num[30][0] < self.start[1] < self.wp_num[30][1] and self.wp_num[20][0] < self.start[0] < self.wp_num[22][0]):  # 일방 통행 구역에 start 가 있으면
                new_coord = self.wp_num[22]
                self.one_way = 1
                return new_coord
            elif (self.wp_num[31][0] < self.start[1] < self.wp_num[31][1] and self.wp_num[21][0] < self.start[0] < self.wp_num[25][0]):  # 다른 일방 통행 구역에 start 가 있으면
                new_coord = self.wp_num[25]
                self.one_way = 1
                return new_coord
            else:

                # if (min1_idx == 20 and min2_idx == 21 and start[1] < 90) or (min1_idx == 5 and min2_idx == 20):
                if (min1_idx == 20 and min2_idx == 21 and self.start[1] < 90):   
                    new_coord, idx = self.wp_num[min2_idx], min2_idx
                    min2_take = 1
                    return new_coord
                else:
                    # new_coord, idx = wp_num[min1_idx], min1_idx
                    # min1_take = 1
                    # print('min1 선택! (start)')
                    # if (min1_idx == 20 and min2_idx == 5) or (min1_idx == 5 and min2_idx == 20) or (min1_idx == 5 and min2_idx == 17) or (min1_idx == 17 and min2_idx == 5):
                    if (min1_idx == 20 and min2_idx == 17) or (min1_idx == 17 and min2_idx == 20) or (min1_idx == 5 and min2_idx == 17) or (min1_idx == 17 and min2_idx == 5):
                        self.exc_sig = 1
                        new_coord = self.wp_num[20]
                        
                        return new_coord
                    else:
                        new_coord, idx = self.wp_num[min1_idx], min1_idx
                        min1_take = 1
                        return new_coord
        
        # goal 좌표를 줬을 때
        else:

            if (min1_idx == 5 and min2_idx == 20 or (min1_idx == 20 and min2_idx == 21)):
                new_coord, idx = self.wp_num[min2_idx], min2_idx
                return new_coord
            else:
                new_coord, idx = self.wp_num[min1_idx], min1_idx
                return new_coord
        
    def generate_waypoint(self, start_point, goal_point, is_renting=None, pp_cnt=None):
        self.start = start_point
        if start_point and goal_point:
            # goal = wp_num[22]
            if is_renting is False:
                goal = self.wp_num[23]  # 목표 좌표

            # 장애물 주변 가중치 추가
            weight_map, background_result = self.create_weight_map(self.middle, buffer_size=10, penalty=7)
            print('here3', weight_map)
            # 시작점, 출발점 보간
            if pp_cnt:
                new_start = self.find_start_goal(self.start, weight_map)
            else:
                new_start = self.wp_num[26]
            
            new_goal = self.find_start_goal(goal, weight_map, flag=1)


            # 시작점, 목표점
            plt.scatter(self.start[0], self.start[1], color='blue', marker='o', label='Start', s=30)
            plt.scatter(goal[0], goal[1], color='red', marker='x', label='Goal', s=30)

            if (new_start != new_goal or exc_sig is not None):

                first_wp_path = self.a_star(new_start, new_goal)


                # 도착점 구하기
                s = first_wp_path[-2]
                g = first_wp_path[-1]

                path_result, _ = self.a_star((s[1], s[0]), (g[1], g[0]), weight_map=weight_map)

                expended_list = self.expand_multiple_coordinates(path_result, buffer_size=4)

                expended_goal = self.expand_coordinates((goal[1], goal[0]), buffer_size=4)

                if any(goal in expended_list for goal in expended_goal):
                    first_wp_path[-1] = goal
                else:
                    first_wp_path.append(goal)
                    
                wp_path = first_wp_path

                # 전역 경로 사이 두 waypoint 간의 pathplanning 실행
                if wp_path:
                    linked_path = []
                    if len(wp_path) == 1 or one_way is not None:
                        wp_path.insert(0, self.start)

                    if one_way is not None:
                        for_cnt = len(wp_path) - 1
                    else:
                        for_cnt = len(wp_path)

                    real_wp_path = []

                    for i in range(for_cnt):
                        # start 좌표가 waypoint_graph 에 명시된 좌표가 아니면
                        # 우선 현재 좌표에서 모든 wp 까지의 비용을 구한 후 가장 최소 비용을 가지는 wp 를 start 좌표로 고정
                        if ((i == 0) and (wp_path[i] != self.start)):
                            if len(wp_path) == 1:
                                wp_start = self.start
                                wp_goal = wp_path[i]
                            else:
                                # 최초 차고지로부터 호출 시 무조건 start = (89, 216) 부근
                                # if is_renting is False:  # 최초 한 번 호출 시
                                if pp_cnt is False:  # 최초 한번 호출 시 
                                    if exc_sig is not None:
                                        wp_path[0] = self.start
                                        # pp_cnt = True
                                    else:
                                        # wp_path[0] = start
                                        wp_path.insert(0, self.start)

                                else:
                                    if (min1_take is None and min2_take is not None):
                                        wp_path.insert(0, new_start)  # 이게 min2_take 일 때
                                    # elif (min2_take is None and min1_take == 1) or (call_sig is not None):
                                    elif (min2_take is None and min1_take is not None):
                                        if exc_sig is not None or len(wp_path) <= 2:
                                            wp_path.insert(0, self.start)
                                        else:
                                            wp_path[0] = self.start
                    
                                wp_start = self.start
                                wp_goal = wp_path[i + 1]
                            
                            # print(f'최종 wp_path = {wp_path}')
                            
                        else:
                            try:
                                wp_start = wp_path[i]
                                wp_goal = wp_path[i + 1]
                            except:
                                pass
                        
                        # print(f'최종 wp_path = {wp_path}')

                        part_path, _ = (self.a_star((wp_start[1], wp_start[0]), (wp_goal[1], wp_goal[0]), weight_map=weight_map))
                        real_path = np.array(part_path)


                        sample_cnt = 15
                        
                        # 즉 , 샘플링 하고 나서 1개 이상 나와야 함.
                        if len(real_path) // sample_cnt >= 1 and len(real_path) > sample_cnt:
                            real_path = real_path[::sample_cnt] 
                        else:
                            pass

                        # 경로 마지막 좌표가 waypoint 좌표가 아니면 waypoint 좌표로 맞춰주기
                        if (len(wp_path) != 1) and (tuple(real_path[-1]) != (wp_goal[1], wp_goal[0])):
                            real_path[-1] = (wp_goal[1], wp_goal[0])
                            # real_path.append((wp_goal[1], wp_goal[0]))
                        
                        linked_path.append(real_path)

                        real_wp_path.append(real_path[0])
                        real_wp_path.append(real_path[-1])

                        
                        path_x = real_path[:, 0]
                        path_y = real_path[:, 1]

                        m = len(path_x)  # 데이터 개수
                        k = (5, m - 1)  # 최소한 m > k 조건을 만족하도록 설정

                        # b - spline 보간법
                        try:
                            tck, u = splprep([path_x, path_y], s=5, k=k)  # s 가 클수록 부드러운 곡선
                            u_fine = np.linspace(0, 1, 300)  # 값이 클수록 세밀한 보간
                        except:
                            pass
                    linked_path = np.concatenate(linked_path)
                    _, idx = np.unique(linked_path, axis=0, return_index=True)
                    final_path = linked_path[np.sort(idx)]
                    print(final_path)
                    return final_path
                else:
                    print('No path found')
            else:
                print('현재 목적지 근처에 있습니다.')

            # 시그널 초기화
            print('모든 시그널 초기화')
            
            min1_take = min2_take = one_way = exc_sig = for_cnt = None
        else:
            print('호출 및 주정차 금지 구역입니다.')

        
    def init_signal(self):
        self.min1_take = None
        self.min2_take = None
        self.one_way = None
        self.exc_sig = None