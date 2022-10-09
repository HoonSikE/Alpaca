        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True:

                # global_obj,local_obj
                result = self.calc_vaild_obj([self.current_postion.x,self.current_postion.y,self.vehicle_yaw],self.object_data)
                
                global_npc_info = result[0] 
                local_npc_info = result[1] 
                global_ped_info = result[2] 
                local_ped_info = result[3] 
                global_obs_info = result[4] 
                local_obs_info = result[5] 


############ 필요한 설정
                # 글로벌 데이터 받기
                rospy.Subscriber("/global_data", global_data, self.global_data_callback)

                def global_data_callback(self, msg):
                    self.global_data = msg


                # 1. 링크셋, 노드셋 
                current_path = os.path.dirname(os.path.realpath(__file__))
                sys.path.append(current_path)
                from lib.mgeo.class_defs import *

                load_path = os.path.normpath(os.path.join(current_path, '../../Sangam_Mgeo'))
                mgeo_planner_map = MGeo.create_instance_from_json(load_path)
                self.node_set = mgeo_planner_map.node_set
                self.link_set = mgeo_planner_map.link_set
                self.nodes=node_set.nodes
                self.links=link_set.lines

                # 2. 현재 신호등 정보 받기
                from morai_msgs.msg import GetTrafficLightStatus
                rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.call_back1)

                self.possible_link_direction = [] # 지금 지나갈 수 있는 링크 속성 (직진, 좌회전, 우회전)

                def call_back1(self, msg):
                    # 신호등 상태
                    status = msg.trafficLightStatus
                    # 지나갈수 있는 속성값 정의
                    if status == 1: # red
                        self.possible_link_direction == []
                    elif status == 4: # yellow
                        self.possible_link_direction == []
                    elif status == 5: # red, yellow
                        self.possible_link_direction == []
                    elif status == 16: # green
                        self.possible_link_direction == ['straight', 'right_unprotected']
                    elif status == 20: # green, yellow
                        self.possible_link_direction == []
                    elif status == 31: # green, left
                        self.possible_link_direction == ['straight', 'left', 'right_unprotected']


############ 사용 데이터 : data =  {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}
                # 1. 차량 전방, 가장 가까운 노드 찾기
                    # 차가 있는 현재 링크 찾기
                current_link = None
                dis = float('inf')

                flag = 0
                for link_idx in self.global_data.links_idx:
                    for x, y, _ in self.links[link_idx].points:
                        temp = ((self.current_postion.x - x)**2 + (self.current_postion.y - y)**2)**0.5
                        if temp < dis:
                            dis = temp
                            current_link = link_idx
                        if temp < 5: # 특정 링크의 한 point와 거리가 5 미만으로 가까우면, 그 링크위에 차가 있다고 봄
                            flag = 1
                            break
                    if flag:
                        break
                
                forward_shortest_node = self.links[current_link].get_to_node() # 차량 전방, 가장 가까운 노드

                # 2. 1번에서 찾은 노드가 정지선 == ture 이면 현재 신호와 차량의 향후 진행 링크 비교한다.
                if forward_shortest_node.is_on_stop_line() == True:
                    # 차의 다음 진행 링크가 직진, 좌회전, 우회전인지 찾기
                    next_link_direction = None
                    for link_idx in self.global_data.links_idx:
                        if self.links[link_idx].get_from_node() == forward_shortest_node:
                            next_link_direction = self.links[link_idx].related_signal
                            break
                    # 2-1. 맞는 신호라서 통과 가능 >> npc 에 장애물 추가하지 않고 그냥 패스
                    if next_link_direction in self.possible_link_direction:
                        pass
                    # 2-2. 다른 신호라서 통과 불가 >> npc 에 장애물 추가
                    else:
                        local_npc_info.append([1, forward_shortest_node.point.x, forward_shortest_node.point.y, 0])