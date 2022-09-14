#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo_dijkstra_path_1 은 Mgeo 데이터를 이용하여 시작 Node 와 목적지 Node 를 지정하여 Dijkstra 알고리즘을 적용하는 예제 입니다.
# 사용자가 직접 지정한 시작 Node 와 목적지 Node 사이 최단 경로 계산하여 global Path(전역경로) 를 생성 합니다.

# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. 시작 Node 와 종료 Node 정의
# 3. weight 값 계산
# 4. Dijkstra Path 초기화 로직
# 5. Dijkstra 핵심 코드
# 6. node path 생성
# 7. link path 생성
# 8. Result 판별
# 9. point path 생성
# 10. dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
# 11. dijkstra 이용해 만든 Global Path 정보 Publish

#TODO: (0) 필수 학습 지식
'''
# dijkstra 알고리즘은 그래프 구조에서 노드 간 최단 경로를 찾는 알고리즘 입니다.
# 시작 노드부터 다른 모든 노드까지의 최단 경로를 탐색합니다.
# 다양한 서비스에서 실제로 사용 되며 인공 위성에도 사용되는 방식 입니다.
# 전체 동작 과정은 다음과 같습니다.
#
# 1. 시작 노드 지정
# 2. 시작 노드를 기준으로 다른 노드와의 비용을 저장(경로 탐색 알고리즘에서는 비용이란 경로의 크기를 의미)
# 3. 방문하지 않은 노드들 중 가장 적은 비용의 노드를 방문
# 4. 방문한 노드와 인접한 노드들을 조사해서 새로 조사된 최단 거리가 기존 발견된 최단거리 보다 작으면 정보를 갱신
#   [   새로 조사된 최단 거리 : 시작 노드에서 방문 노드 까지의 거리 비용 + 방문 노드에서 인접 노드까지의 거리 비용    ]
#   [   기존 발견된 최단 거리 : 시작 노드에서 인접 노드까지의 거리 비용                                       ]
# 5. 3 ~ 4 과정을 반복 
# 

'''
class dijkstra_path_pub :
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)

        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        # NodeSet() 에는 Node들이 
        # self.nodes[node_obj.idx] = node_obj 코드로 추가된다.
        # 여기서 node_obj는 Node() 이고 self.nodes 는 Dict()이다.
        # node_obj.idx는 'A219BS010380'와 같은 값으로 알고있다.
        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        # 즉 self.nodes 는 Node().idx를 키로 하고 Node()를 값으로 하는 Dict()이다.
        self.nodes=node_set.nodes
        self.links=link_set.lines

        # for key, val in self.nodes.items():
        #     print(key, val)

        # node는 A119BS010을 기본으로 가지고 있다.
        # (u'A119BS010717', <class_defs.node.Node object at 0x7faf15ee3ed0>)
        # (u'A119BS010169', <class_defs.node.Node object at 0x7faf1674e5d0>)
        # (u'A119BS010168', <class_defs.node.Node object at 0x7faf1674e590>)
        # (u'ND000000', <class_defs.node.Node object at 0x7faf15eed210>)

        # for key, val in self.nodes.items():
        #     print(val.get_from_nodes(), val.get_to_nodes())

        # get_from_nodes()와 get_to_nodes()는.. 차선의 방향마다 노드가 있어서 지정이 확실하게 된것 같다.
        # 예시로 가져온 아래의 두 노드의 get_from_nodes()와 get_to_nodes()은... 
        # 교차로 같은 것인가? get_to_nodes()가 같다.
        # ([<class_defs.node.Node object at 0x7f36ed69bd50>, <class_defs.node.Node object at 0x7f36ece25e10>], [<class_defs.node.Node object at 0x7f36ed6901d0>, <class_defs.node.Node object at 0x7f36ed69bd90>])
        # ([<class_defs.node.Node object at 0x7f36ed69bb90>, <class_defs.node.Node object at 0x7f36ece25dd0>], [<class_defs.node.Node object at 0x7f36ed69bd90>, <class_defs.node.Node object at 0x7f36ed6901d0>])
        
        # for key, val in self.links.items():
        #     print(key, val)

        # link는 A219BS010을 기본으로 가지고 있다.
        # (u'A219BS010424-A219BS010425', <class_defs.link.Link object at 0x7f4844431050>)
        # (u'A219BS010115-A219BS010114', <class_defs.link.Link object at 0x7f4844485850>)
        # (u'A219BS010419-A219BS010418', <class_defs.link.Link object at 0x7f484449bed0>)
        # (u'A219BS010124', <class_defs.link.Link object at 0x7f48444aec10>)

        self.global_planner=Dijkstra(self.nodes,self.links)

        #TODO: (2) 시작 Node 와 종료 Node 정의

        # Dijkstra Path 를 만들기 위한 출발 Node 와 도착 Node의 Idx 를 지정합니다.
        # MGeo 데이터는 Json 파일로 Idx 정보를 확인 할 수 있지만 시뮬레이터를 통해 직접 확인 가능합니다.
        # F8 키보드 입력 또는 시뮬레이터에서 화면 좌측 상단에 View --> MGeo Viewer 를 클릭합니다.
        # MGeo Viewer 기능을 이용하여 맵에있는 MGeo 정보를 확인 할 수 있으며 시각화 까지 가능합니다.
        # 해당 기능을 이용하여 원하는 시작 위치와 종료 위치의 Node 이름을 알아낸 뒤 아래 변수에 입력하세요.
        
        # self.start_node = 'A119BS010184'
        # self.end_node = 'A119BS010148'

        self.start_node = 'A119BS010176'
        self.end_node = 'A119BS010327'

        # 왜 있는..?
        # self.global_path_msg = Path()
        # self.global_path_msg.header.frame_id = '/map'

        self.global_path_msg = self.calc_dijkstra_path_node(self.start_node, self.end_node)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (11) dijkstra 이용해 만든 Global Path 정보 Publish
            # dijkstra 이용해 만든 Global Path 메세지 를 전송하는 publisher 를 만든다.
            self.global_path_pub.publish(self.global_path_msg)
            
            rate.sleep()

    def calc_dijkstra_path_node(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)

        #TODO: (10) dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'
        
        # dijkstra 경로 데이터 중 Point 정보를 이용하여 Path 데이터를 만들어 줍니다.
        # => result에 대한 true false는 안..사용하나보다.
        point_paths = path["point_path"]
        for point_path in point_paths:
            read_pose = PoseStamped()
            read_pose.pose.position.x = point_path[0]
            read_pose.pose.position.y = point_path[1]
            read_pose.pose.position.z = .0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        # Node() 참고
        self.nodes = nodes
        # Link() 참고
        self.links = links
        #TODO: (3) weight 값 계산
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        #TODO: (3) weight 값 계산
        '''
        # weight 값 계산은 각 Node 에서 인접 한 다른 Node 까지의 비용을 계산합니다.
        # 계산된 weight 값 은 각 노드간 이동시 발생하는 비용(거리)을 가지고 있기 때문에
        # Dijkstra 탐색에서 중요하게 사용 됩니다.
        # weight 값은 딕셔너리 형태로 사용 합니다.
        # 이중 중첩된 딕셔너리 형태로 사용하며 
        # Key 값으로 Node의 Idx Value 값으로 다른 노드 까지의 비용을 가지도록 합니다.
        # 아래 코드 중 self.find_shortest_link_leading_to_node 를 완성하여 
        # Dijkstra 알고리즘 계산을 위한 Node와 Node 사이의 최단 거리를 계산합니다.

        '''
        
        # 초기 설정
        weight = dict() # 노드의 idx로 다른 노드까지의 거리를 저장하는 변수인데..
        # 아래 초기화하는 것을 보면 알겠지만 N*N의 크기를 자랑한다. 아무리봐도 희소행렬인데;;

        # https://wikidocs.net/16 에 따르면
        # Dict().items는 Key, Value 쌍을 얻는 함수
        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 다른 노드로 진행하는 모든 weight
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            # 전체 weight matrix에 추가
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 현재 노드로는 cost = 0
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
                weight[from_node_id][to_node.idx] = min_cost           

        return weight

    def find_shortest_link_leading_to_node(self, from_node, to_node):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        #TODO: (3) weight 값 계산
        
        # 최단거리 Link 인 shortest_link 변수와
        # shortest_link 의 min_cost 를 계산 합니다.

        # 찾아보기 귀찮아서 Node.find_shortest_link_leading_to_node 를 그대로 복붙하였다.
        # 해석하면.. from_node에서 Link를 통해서 직접적으로 연결된 to_node로 가는 최단 거리의 link와 cost..
        
        # 왜 쓰는거지..? 이해를 잘못했나? 아니면 직접 코딩하라는 큰 그림인가..?? @,.@
        # 애초에.. 두 node가 다른 두 link로 연결되는 경우도 있..나? (왜 있을 것 같지..)
        # 음.. 나중에 해당 함수를 안 사용하고 `find_shortest_path`에 직접 따로 초기화를 할 것 같다.

        to_links = []
        for link in from_node.get_to_links():
            # 이렇게 링크 다 찾아볼려면 차라리 from_node에서 get_to_nodes에 있는 모든 노드까지의 최단을 
            # get_to_nodes.idx를 키로 하여 Dict()로 저장한 다음에 반환하고 
            # 함수명을 find_shortest_link_leading_to_node's'로 이름을 바꾸는게 더 좋지 않나..?
            # 나도 잘 모르겠다.. ^ㅠ^ 이유가 있긋지~
            if link.to_node is to_node:
                to_links.append(link)


        if len(to_links) == 0:
            raise BaseException('[ERROR] Error @ Dijkstra.find_shortest_path : Internal data error. There is no link from node (id={}) to node (id={})'.format(self.idx, to_node.idx))

        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost
        
    def find_nearest_node_idx(self, distance, s):      
        """방문하지 않은 node 중 가장 작은 가중치(cost)를 가지는 node의 idx 반환"""  
        idx_list = self.nodes.keys()
        min_value = float('inf')
        min_idx = idx_list[-1] # 왜 굳이 가장 마지막 node의 idx를 넣었을까? 그냥 초기화인가?

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx): 
        #TODO: (4) Dijkstra Path 초기화 로직
        # s 초기화         >> s = [False] * len(self.nodes)
        # from_node 초기화 >> from_node = [start_node_idx] * len(self.nodes)
        '''
        # Dijkstra 경로 탐색을 위한 초기화 로직 입니다.
        # 변수 s와 from_node 는 딕셔너리 형태로 크기를 MGeo의 Node 의 개수로 설정합니다. 
        # Dijkstra 알고리즘으로 탐색 한 Node 는 변수 s 에 True 로 탐색하지 않은 변수는 False 로 합니다.
        # from_node 의 Key 값은 Node 의 Idx로
        # from_node 의 Value 값은 Key 값의 Node Idx 에서 가장 비용이 작은(가장 가까운) Node Idx로 합니다.
        # from_node 통해 각 Node 에서 가장 가까운 Node 찾고
        # 이를 연결해 시작 노드부터 도착 노드 까지의 최단 경로를 탐색합니다. 

        '''

        # 파이썬의 딕셔너리는.. 개수 설정이 있나..? 왜 설명에서 개수를 설정하라고 했을까..?
        s = dict()  # Node 탐색 여부
        from_node = dict()  # 한 노드의 idx를 키, 다른 노드의 idx를 값으로 하는 딕셔너리
        # 시작 노드부터 도착 노드까지의 최단 '경로'를 찾기 위해 사용한다.

        # self.nodes.keys() 는 맵에 존재하는 모든 node들의 idx를 의미
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx

        s[start_node_idx] = True
        distance = copy.deepcopy(self.weight[start_node_idx])    # start_node_idx부터 각 노드까지의 거리 가중치
        
        #TODO: (5) Dijkstra 핵심 코드

        # 맞나..? 우선순위 큐를 사용하지 않는 것은 도로의 노드가 너무 많아서 그렇다고 해도..
        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s) # 방문하지 않은 노드 중 가장 가중치가 적은 node의 idx
            s[selected_node_idx] = True # 위의 노드를 방문처리
            
            # enumerate는 왜..?
            # 노드별로 있는 get_to_nodes, get_from_nodes 함수는 왜 안 사용하지..?
            # 아.. 나중에 싹다 고치자..

            # 위에서 선택한 노드에서 연결된 방문하지 않은 노드를 탐색하여 거리가 더 가까우면 distance와 from_node 갱신
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    # 아무래도 inf + a 를 하면 생기는 오버플로우 처리를 파이썬이 자동으로 해주는 모양이다.
                    distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        # from_node에 탐색하는 노드의 idx를 키로하고 현재 노드의 idx를 값으로 저장한다.
                        from_node[to_node_idx] = selected_node_idx

        #TODO: (6) node path 생성

        # 시작 노드부터 도착 노드까지의 최단경로 탐색, 위에서 언급한대로 from_node를 사용한다.
        # node, link, point path 모두 동일한 방식이다.
        # 시작 노드로부터는 못 구하고 무조건 도착노드부터 탐색하여 찾은 다음 reverse를 해주어야 한다.
        # 이유는 다익스트라 알고리즘을 잘 알고있다면 이해할 것이다.

        # 어.. 그런데 왜.. path를 따로따로 구한거지..? node 구할때 link 구하고 동시에 point path 구하는 것도 가능한데..?
        # 그리고 node 속성을 전혀 사용하지 않는 이유가.. 있나?
        # 생각하지 말자 ^^!

        tracking_idx = end_node_idx
        node_path = [end_node_idx]
        
        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)     

        node_path.reverse()

        #TODO: (7) link path 생성
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
            link_path.append(shortest_link.idx)

        #TODO: (8) Result 판별
        if len(link_path) == 0:
            return False, {'node_path': node_path, 'link_path':link_path, 'point_path':[]}

        #TODO: (9) point path 생성
        point_path = []        
        for link_id in link_path:
            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])

        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}

if __name__ == '__main__':
    
    dijkstra_path_pub = dijkstra_path_pub()