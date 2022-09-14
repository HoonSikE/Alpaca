#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage

# gps_parser 는 GPS의 위경도 데이터를 UTM 좌표로 변환하는 예제

class LL2UTMConverter:
    def __init__(self, zone=52) :
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        # 초기화
        self.x, self.y = None, None

        # (1) 변환 하고자 하는 좌표계를 선언
        # 클래스명 위에 커서를 두고 ctrl 클릭으로 파일까지 들어가서 메뉴얼에 예시 코드까지 확인이 가능하다.
        # utm은 가로 세로로 구역(zone)을 나누는데 우리나라는 52S구역에 속해있고 zone에 52를 넣어주어야 정확한 값이 나온다.
        # 참고한 곳은 https://proj.org/operations/projections/utm.html
        self.proj_UTM = Proj(proj='utm',zone=52,ellps='WGS84')

    # (2) 시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성
    def navsat_callback(self, gps_msg):
        # GPS 센서에서 수신되는 위도 경도 데이터를 확인
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude

        self.convertLL2UTM()

        utm_msg = Float32MultiArray()

        # (4) 위도 경도 데이터와 변환한 UTM 좌표를 터미널 창에 출력 하여 확인
        utm_msg.data = [self.x, self.y]
        os.system('clear')
        print(' lat : ', self.lat)
        print(' lon : ', self.lon)
        print('x=%9.3f y=%11.3f' % (self.x, self.y))

    # (3) 위도 경도 데이터를 UTM 좌표로 변환
    def convertLL2UTM(self):
        # proj_UTM(경도 데이터, 위도 데이터)
        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0]
        self.y = xy_zone[1]

if __name__ == '__main__':

    rospy.init_node('gps_parser', anonymous=True)

    gps_parser = LL2UTMConverter()

    rospy.spin()