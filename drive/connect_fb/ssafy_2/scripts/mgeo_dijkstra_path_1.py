#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json
import heapq

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
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'map'


        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines

        self.global_planner=Dijkstra(self.nodes, self.links)

        #TODO: (2) 시작 Node 와 종료 Node 정의
        self.start_node = u'A119BS010332'
        self.end_node = u'A119BS010272'

        
        

        points_list = self.calc_dijkstra_path_node(self.start_node, self.end_node)
        
        for point in points_list:
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(point[0])
            read_pose.pose.position.y = float(point[1])
            read_pose.pose.orientation.w = 1

            self.global_path_msg.poses.append(read_pose)


        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #TODO: (11) dijkstra 이용해 만든 Global Path 정보 Publish
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()


    def calc_dijkstra_path_node(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)

        #TODO: (10) dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = [] if not result else path
        '''
        # dijkstra 경로 데이터 중 Point 정보를 이용하여 Path 데이터를 만들어 줍니다.
        '''

        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
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
        weight = dict() 

        for from_node_id, from_node in self.nodes.items():
            weigh_from_this_node = dict()
            for node in from_node.get_to_nodes():
                weigh_from_this_node[node.idx] = float('inf')
            weight[from_node_id] = weigh_from_this_node
        
        for from_node_idx in weight.keys():
            for next_node_idx in weight[from_node_idx].keys():
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node_idx, next_node_idx)
                weight[from_node_idx][next_node_idx] = [min_cost, shortest_link]

        return weight

    def find_shortest_link_leading_to_node(self, from_node_idx, to_node_idx):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        #TODO: (3) weight 값 계산
        shortest_link = None
        min_cost = float('inf')

        for key, link in self.links.items():
            if link.from_node.idx == from_node_idx and link.to_node.idx == to_node_idx:
                if len(link.points) < min_cost:
                    min_cost = len(link.points)
                    shortest_link = link.idx

        return shortest_link, min_cost
        
    def find_nearest_node_idx(self, distance, s):        
        idx_list = self.nodes.keys()
        min_value = float('inf')
        min_idx = idx_list[-1]

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx): 
        

        shortest_ways = {}
        distances = {}
        for node_id in self.weight.keys():
            shortest_ways[node_id] = []
            distances[node_id] = float('inf')

        # start point
        distances[start_node_idx] = 0   
        queue = []


        heapq.heappush( queue, [distances[start_node_idx], start_node_idx] )
        while queue:
            distance, start_idx = heapq.heappop(queue)

            if distances[start_idx] < distance:
                continue

            for next_node in self.weight[start_idx]:
                contents = self.weight[start_idx][next_node]
                if distance + contents[0] < distances[next_node]:
                    distances[next_node] = distance + contents[0]
                    shortest_ways[next_node] = shortest_ways[start_idx] + [contents[1]]

                    heapq.heappush(queue, [distances[next_node], next_node])

        # print(distances[end_node_idx])
        # print(shortest_ways[end_node_idx])


        #TODO: (8) Result 판별
        if len(shortest_ways[end_node_idx]) == 0:
            return False, []

        #TODO: (9) point path 생성
        point_path = []        
        for link_id in shortest_ways[end_node_idx]:
            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])

        return True, point_path

if __name__ == '__main__':
    
    dijkstra_path_pub = dijkstra_path_pub()