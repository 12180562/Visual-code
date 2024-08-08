#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.Inha_VelocityObstacle import VO_module
from functions.Inha_VectorFieldHistogram import VFH_module
from functions.Inha_DataProcess import Inha_dataProcess, UKF
from udp_col_msg.msg import col, vis_info, cri_info, VO_info, static_OB_info, static_OB_array_info
from udp_msgs.msg import frm_info, group_wpts_info, wpt_idx_os, group_boundary_info
from ctrl_msgs.msg import ctrl_output_pknu
from math import sqrt, atan2, cos, sin, isnan
from numpy import rad2deg

import csv
import numpy as np
import rospy
import time
import rospkg

class data_inNout:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(self):
        # Subscriber = input
        rospy.Subscriber('/frm_info', frm_info, self.OP_callback) 
        rospy.Subscriber('/waypoint_info', group_wpts_info, self.wp_callback)
        rospy.Subscriber('/static_OB_info', static_OB_info, self.static_OB_callback)
        rospy.Subscriber('/static_OB_array', static_OB_array_info, self.static_OB_array_callback)
        rospy.Subscriber('/wpts_idx_os_kriso', wpt_idx_os, self.wp_idx_callback)
        # rospy.Subscriber('/ctrl_info_pknu', ctrl_output_pknu, self.wp_idx_callback)

        ############################ for connect with KRISO format ##################################

        # rospy.Subscriber('/Unavailiable_Area_info', group_boundary_info, self.static_unavailable_callback)
        # rospy.Subscriber('/Availiable_Area_info', group_boundary_info, self.static_available_callback)

        ############################ for connect with KRISO format ##################################

        self.WP_pub = rospy.Publisher('/vessel1_info', col, queue_size=10)
        self.cri_pub = rospy.Publisher('/cri1_info', cri_info, queue_size=10)
        self.VO_pub = rospy.Publisher('/VO1_info', VO_info, queue_size=10)
        self.Vis_pub = rospy.Publisher('/vis1_info', vis_info, queue_size=10)
        self.ship_ID = []
        self.waypoint_idx = 0
        self.len_waypoint_info = 0
        self.waypoint_dict = dict()
        self.ts_spd_dict = dict()
        self.r_deg = []
        self.static_OB_array_origin = np.array([0,0])
        self.shape_origin = np.array([])
        self.os_position_origin = np.array([])
        self.Drift_angle = 0
        # self.ship1_index = rospy.get_param('ship1_index')
        # self.index = rospy.get_param('index')

        self.TS_WP_index = []
        ############################ for connect with KRISO format ##################################

        # self.static_unavailable_info =[]
        # self.static_available_info =[]

        ############################ for connect with KRISO format ##################################
        self.static_obstacle_info = []
        self.static_point_info = []

        self.target_heading_list = []

    def wp_callback(self, wp):
        ''' subscribe `/waypoint_info`

        Example:
            OS_wpts_x = self.waypoint_dict['2000'].wpts_x
        '''
        self.len_waypoint_info = len(wp.group_wpts_info)
        wp_dic = dict()
        for i in range(self.len_waypoint_info):
            shipID = wp.group_wpts_info[i].shipID
            wp_dic['{}'.format(shipID)] = wp.group_wpts_info[i]
            ## 위 처럼 표현할 경우, dictionary 안의 message 변수명을 알고 있어야 호출이 가능함!
            ## 따라서, 새로운 임의의 key Value로 바꾸서 저장하고 싶다면 아래와 같이 새로운 dictionary를 만들어도 됨. (이중 dictionary 구조)
            # wp_dic2 = dict()
            # wp_dic2['waypoint_x'] = wp.group_wpts_info[i].wpts_x
            # wp_dic2['waypoint_y'] = wp.group_wpts_info[i].wpts_y
            
            # wp_dic[f'{shipID}'] = wp_dic2

        self.waypoint_dict = wp_dic
        
    def wp_idx_callback(self, idx):
        self.waypoint_idx = idx.m_idxWptOS
        # self.waypoint_idx = idx.i_way[self.ship1_index]
        

    def OP_callback(self, operation):
        ''' subscribe `/frm_info` 
        
        params : 
            `frm_info` 변수명은 입출력관계도 KRISO 참조

        Note :
            `psi`값이 [-2pi, 2pi]값으로 들어오므로, 편의상 강제로 [0, 2pi]로 변경
        '''
        self.ship_ID = list(operation.m_nShipID)

        self.Pos_X  = operation.m_fltPos_X
        self.Pos_Y  = operation.m_fltPos_Y
        self.Vel_U  = operation.m_fltVel_U
        self.Drift_angle = operation.m_fltDriftangle 

        self.delta_deg = operation.m_fltRudderAngleFeedSTBD # deg.
        self.r_deg = operation.m_fltFOGang_yawz

        raw_psi = np.asanyarray(operation.m_fltHeading)
        self.Heading = raw_psi % 360
                

    def static_OB_callback(self, static_OB):
        self.static_obstacle_info = static_OB.data
        self.static_point_info = static_OB.point

        ############################ for connect with KRISO format ##################################

    def static_unavailable_callback(self, static_OB):
        self.len_static_obstacle_info = len(static_OB.group_boundary_info)
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
            static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
            
        static_ob_info = []
        
        for k in range(len(static_ob_list_x)):
            for l in range(len(static_ob_list_x[k])):
                if l == 0:
                    pass
                else:
                    static_ob_info.append(static_ob_list_x[k][l-1])
                    static_ob_info.append(static_ob_list_y[k][l-1])
                    static_ob_info.append(static_ob_list_x[k][l])
                    static_ob_info.append(static_ob_list_y[k][l])
                    
        self.static_unavailable_info = static_ob_info
        
    def static_available_callback(self, static_OB):
        self.len_static_obstacle_info = len(static_OB.group_boundary_info)
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
            static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
        
        static_ob_info = []
        
        for k in range(len(static_ob_list_x)):
            for l in range(len(static_ob_list_x[k])):
                if l == 0:
                    pass
                else:
                    static_ob_info.append(static_ob_list_x[k][l-1])
                    static_ob_info.append(static_ob_list_y[k][l-1])
                    static_ob_info.append(static_ob_list_x[k][l])
                    static_ob_info.append(static_ob_list_y[k][l])
                    
        self.static_available_info = static_ob_info

        ############################ for connect with KRISO format ##################################

    def static_OB_array_callback(self, static_array):
        self.static_OB_array_origin = np.array(static_array.data)
        self.static_OB_array_origin = self.static_OB_array_origin.reshape(static_array.shape[0],static_array.shape[1])
        self.shape_origin = static_array.shape
        self.os_position_origin = np.array(static_array.origin_point)

    def path_out_publish(self, pub_list):
        ''' publish `/path_out_inha`
        
        Note :
            pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
        '''
        inha = col()
        inha.nship_ID = pub_list[0]
        inha.modifyWayPoint = pub_list[1]
        inha.numOfWayPoint  = pub_list[2]
        inha.latOfWayPoint = pub_list[3]
        inha.longOfWayPoint = pub_list[4]
        inha.speedOfWayPoint = pub_list[5]
        inha.ETAOfWayPoint = round(pub_list[6], 3)
        inha.EDAOfWayPoint = round(pub_list[7], 3)
        inha.error = pub_list[8]
        inha.errorCode = pub_list[9]
        inha.targetSpeed = round(pub_list[10], 3)
        inha.targetCourse = round(pub_list[11], 3)
        
        self.WP_pub.publish(inha)


    def vis_out(self, pub_list):
        vis = vis_info()
        vis.nship_ID = pub_list[0]
        vis.collision_cone = pub_list[1]
        vis.v_opt = pub_list[2]
        vis.local_goal = pub_list[3]

        self.Vis_pub.publish(vis)

    def cri_out(self, pub_list):
        cri = cri_info()
        cri.DCPA = pub_list[0]
        cri.TCPA = pub_list[1]
        cri.UDCPA = pub_list[2]
        cri.UTCPA = pub_list[3]
        cri.UD = pub_list[4]
        cri.UB = pub_list[5]
        cri.UK = pub_list[6]
        cri.CRI = pub_list[7]
        cri.Rf = pub_list[8]
        cri.Ra = pub_list[9]
        cri.Rs = pub_list[10]
        cri.Rp = pub_list[11]
        cri.encounter_classification = pub_list[12]

        self.cri_pub.publish(cri)

    def vo_out(self, pub_list):
        vo = VO_info()
        vo.V_opt = pub_list[0]
        vo.Collision_cone = pub_list[1]

        self.VO_pub.publish(vo)

def main():  
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()  
    package_path = rospack.get_path('kass_inha')
    VO_operate = rospy.get_param("shipInfo_all/ship1_info/include_inha_modules")

    update_rate = rospy.get_param("update_rate")
    dt =  rospy.get_param("mmg_dt")

    timestr = time.strftime("%Y%m%d-%H%M%S")
    path = "/home/phl/Documents/" + timestr + ".csv"
    # header = ['ShipID', 'Pos_X', 'Pos_Y', 'wp_x', 'wp_y', 'Vel_U', 'Vx', 'Vy', 'Heading', 'desired_heading', 'encounter', 'encounterMMSI']
    header = ['RD','RC', 'K', 'DCPA','TCPA', 'UDCPA', 'UTCPA', 'UD', 'UB', 'UK', 'CRI', 'Rf', 'Ra', 'Rs', 'Rp', 'ENC', 'V_opt', 'pub_collision_cone', 'VO_operate']
    file = open(path, 'a', newline='')
    writer = csv.writer(file)
    writer.writerow(header)

    node_Name = "vessel_node1"
    rospy.init_node("{}".format(node_Name), anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    OS_ID = rospy.get_param("shipInfo_all/ship1_info/ship_ID")
    TS_ID = []
    desired_spd_list = []
    pub_collision_cone = []
    V_opt = []

    # 자선의 정보
    OS_scale = rospy.get_param("shipInfo_all/ship1_info/ship_scale")
    target_speed = rospy.get_param("shipInfo_all/ship1_info/target_speed")  * 0.5144 / sqrt(OS_scale)
    ship_L = rospy.get_param("shipInfo_all/ship1_info/ship_L") ## 향후 이부분은, 1) ship domain과 2) AIS data의 선박의 길이 부분으로 나누어 고려 및 받아야 함!
    static_obstacle_key = rospy.get_param("shipInfo_all/ship1_info/static_obstacle_key")

    # # !----- 설문조사에서는 충돌회피 시점은 12m 어선 기준으로 일반적으론 HO 3nm/ CS & OT 2nm을 기준으로 하고 있으며, 최소 안전 이격거리는 0.5~ 1nm으로 조사됨
    # # 다만, 2m급 모형선 테스트에서는 협소한 부분이 있으므로 스케일 다운(1/200)을 시켜서, "회피시점: 0.0015nm(27.78m) / 최소 안전 이격거리는 9.26m"가 되게끔 할 예정
    # # 참고 논문: https://www.koreascience.or.kr/article/JAKO201427542599696.pdf 

    data = data_inNout()
    
    t = 0
    waypointIndex = 0
    targetspdIndex = 0    

    ukf_instances = {}
    first_loop = True  # 첫 번째 루프 실행 여부를 추적하는 변수

    encounter = None
    encounterMMSI = []

    last_publish_time = rospy.Time.now()  # 마지막으로 발행한 시간을 초기화
    delay = rospy.get_param('ais_delay')
    publish_interval = rospy.Duration(delay)  # 발행 주기를 5초로 설정
    first_publish = True
    latest_ship_info = {}
    predicted_state = [0,0]
    Pre_X = 0
    Pre_Y = 0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()  # 현재 시간을 계속 추적
        # data.static_obstacle_info = data.static_unavailable_info + data.static_available_info
        
        if len(data.ship_ID) == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/frm_info` topic subscription in {}=========".format(node_Name))
            rate.sleep()
            continue

        if data.len_waypoint_info == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/waypoint_info` topic subscription in {} =========".format(node_Name))
            rate.sleep()
            continue

        startTime = time.time()

        inha = Inha_dataProcess(
            data.ship_ID,
            data.Pos_X, 
            data.Pos_Y, 
            data.Vel_U, 
            data.Heading, 
            data.waypoint_dict,
            Pre_X,
            Pre_Y,
            data.r_deg,
            data.Drift_angle,
            np.array(data.Vel_U) * np.cos(np.array(data.Drift_angle)),
            np.array(data.Vel_U) * np.sin(np.array(data.Drift_angle)),
            )                       # inha_module의 data 송신을 위해 필요한 함수들이 정의됨

        ## <======== 서울대학교 전역경로를 위한 waypoint 수신 및 Local path의 goal로 처리
        wpts_x_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_x)
        wpts_y_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_y)
        # Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]   
        Local_goal = [wpts_x_os[data.waypoint_idx], wpts_y_os[data.waypoint_idx]]          # kriso
        # Local_goal = [wpts_x_os[int(data.waypoint_idx)], wpts_y_os[int(data.waypoint_idx)]]          # 부경대
        ## <========= `/frm_info`를 통해 들어온 자선 타선의 데이터 전처리
        ship_list, ship_ID = inha.ship_list_container(OS_ID)

        # if first_loop:
        #     for ship_id in data.ship_ID:
        #         # 각 선박 ID에 대해 독립적인 UKF 인스턴스 생성 및 저장
        #         ukf_instances[ship_id] = UKF()

        #     first_loop = False  # 첫 번째 루프가 실행된 후에는 이 조건을 더 이상 만족시키지 않음

        # for ship_id, ship_info in ship_list.items():

        #     # 해당 선박의 UKF 인스턴스를 사용하여 업데이트

        #     if ship_id == OS_ID:
        #         ship_list[OS_ID].update({
        #             'Pos_X' : ship_info['Ori_X'],
        #             'Pos_Y' : ship_info['Ori_Y'],
        #             })
                
        #     else:
        #         if (current_time - last_publish_time >= publish_interval) or first_publish or (ship_info == None):
        #             # frm_info_publish 메소드를 호출하여 상태 정보를 발행
        #             # 최신 상태 정보 업데이트
        #             latest_ship_info = ship_list

        #             # 딜레이 안함
        #             ship_list[ship_id].update({
        #                 'Ship_ID' : latest_ship_info[ship_id]['Ship_ID'],
        #                 'Ori_X' : latest_ship_info[ship_id]['Ori_X'],
        #                 'Ori_Y' : latest_ship_info[ship_id]['Ori_Y'],
        #                 'Vel_U' : latest_ship_info[ship_id]['Vel_U'],
        #                 'Heading' : latest_ship_info[ship_id]['Heading'],
        #                 'Pos_X' : latest_ship_info[ship_id]['Ori_X'],
        #                 'Pos_Y' : latest_ship_info[ship_id]['Ori_Y'],
        #                 })
                    
        #             last_publish_time = current_time  # 마지막 발행 시간을 현재 시간으로 업데이트
        #             first_publish = False  # 첫 번째 발행이 끝났으니 플래그를 False로 설정

        #         else:
        #             # 발행 주기 사이에는 마지막으로 업데이트된 상태 정보를 유지하며 발행
        #             predicted_state = ukf_instances[ship_id].update_ukf(latest_ship_info[ship_id]['Ori_X'], latest_ship_info[ship_id]['Ori_Y'], latest_ship_info[ship_id]['Vel_U'], latest_ship_info[ship_id]['Heading'])
        #             Pre_X = predicted_state[0]
        #             Pre_Y = predicted_state[1]

        #             # 예측 진행
        #             ship_list[ship_id].update({
        #                 'Ship_ID' : latest_ship_info[ship_id]['Ship_ID'],
        #                 'Ori_X' : latest_ship_info[ship_id]['Ori_X'],
        #                 'Ori_Y' : latest_ship_info[ship_id]['Ori_Y'],
        #                 'Vel_U' : latest_ship_info[ship_id]['Vel_U'],
        #                 'Heading' : latest_ship_info[ship_id]['Heading'],
        #                 'Pos_X' : Pre_X,
        #                 'Pos_Y' : Pre_Y,
        #                 })

        #             # # 예측 안함
        #             # ship_list[ship_id].update({
        #             #     'Ship_ID' : latest_ship_info[ship_id]['Ship_ID'],
        #             #     'Ori_X' : latest_ship_info[ship_id]['Ori_X'],
        #             #     'Ori_Y' : latest_ship_info[ship_id]['Ori_Y'],
        #             #     'Vel_U' : latest_ship_info[ship_id]['Vel_U'],
        #             #     'Heading' : latest_ship_info[ship_id]['Heading'],
        #             #     'Pos_X' : latest_ship_info[ship_id]['Ori_X'],
        #             #     'Pos_Y' : latest_ship_info[ship_id]['Ori_Y'],
        #             #     })

        # print(ship_list)
        # print("예측됨 X: {}, Y: {}".format(ship_list[OS_ID]['next_X'], ship_list[OS_ID]['next_Y']))

        OS_list, TS_list = inha.classify_OS_TS(ship_list, ship_ID, OS_ID)

        TS_ID = ship_ID[:]  ## 리스트 복사
        TS_ID.remove(OS_ID)
        
        # TS_ID = TS_list.key()
        # TODO : why do this?

        OS_Vx, OS_Vy = inha.U_to_vector_V(OS_list['Vel_U'], OS_list['Heading'])

        OS_list['V_x'] = OS_Vx
        OS_list['V_y'] = OS_Vy

        _, local_goal_EDA = inha.eta_eda_assumption(Local_goal, OS_list, target_speed)

        # <=========== VO 기반 충돌회피를 위한 경로 생성
        # !--- 1) `Local goal`으로 향하기 위한 속도벡터 계산

        Local_PP = VO_module()
        VFH_pp = VFH_module(OS_list)

        V_des = Local_PP.vectorV_to_goal(OS_list, Local_goal, target_speed)

        '''
            NOTE: 
                `OS_list` and `TS_list`:
                    {
                        'Ship_ID': [], 
                        'Pos_X' : [],  
                        'Pos_Y' : [],   
                        'Vel_U' : [],   
                        'Heading_deg' : [], 
                        'V_x' : [], 
                        'V_y' : [], 
                        'radius' : []
                    }
        '''

        TS_list = inha.TS_info_supplement(
            OS_list, 
            TS_list,
            )
        
        TS_RD_temp = []
        TS_RC_temp = []
        TS_K_temp = []
        TS_DCPA_temp = []
        TS_TCPA_temp = []
        TS_UDCPA_temp = []
        TS_UTCPA_temp = []
        TS_UD_temp = []
        TS_UB_temp = []
        TS_UK_temp = []
        TS_CRI_temp = []
        TS_Rf_temp = []
        TS_Ra_temp = []
        TS_Rs_temp = []
        TS_Rp_temp = []
        TS_ENC_temp = []

        encounterMMSI = []

        TS_list_copy = {}
        TS_ID_copy = []

        for ts_ID in TS_ID:
            
            temp_RD = TS_list[ts_ID]['RD']
            TS_RD_temp.append(temp_RD)
            
            temp_RC = TS_list[ts_ID]['RC']
            TS_RC_temp.append(temp_RC)

            temp_K = TS_list[ts_ID]['K']
            TS_K_temp.append(temp_K)

            temp_DCPA = TS_list[ts_ID]['DCPA']
            TS_DCPA_temp.append(temp_DCPA)

            temp_TCPA = TS_list[ts_ID]['TCPA']
            TS_TCPA_temp.append(temp_TCPA)

            temp_UDCPA = TS_list[ts_ID]['UDCPA']
            TS_UDCPA_temp.append(temp_UDCPA)
            
            temp_UTCPA = TS_list[ts_ID]['UTCPA']
            TS_UTCPA_temp.append(temp_UTCPA)

            temp_UD = TS_list[ts_ID]['UD']
            TS_UD_temp.append(temp_UD)

            temp_UB = TS_list[ts_ID]['UB']
            TS_UB_temp.append(temp_UB)

            temp_UK = TS_list[ts_ID]['UK']
            TS_UK_temp.append(temp_UK)

            temp_cri = TS_list[ts_ID]['CRI']
            TS_CRI_temp.append(temp_cri)

            temp_Rf = TS_list[ts_ID]['Rf']
            TS_Rf_temp.append(temp_Rf)

            temp_Ra = TS_list[ts_ID]['Ra']
            TS_Ra_temp.append(temp_Ra)

            temp_Rs = TS_list[ts_ID]['Rs']
            TS_Rs_temp.append(temp_Rs)

            temp_Rp = TS_list[ts_ID]['Rp']
            TS_Rp_temp.append(temp_Rp)

            temp_enc = TS_list[ts_ID]['status']
            TS_ENC_temp.append(temp_enc)

            distance = sqrt((OS_list["Pos_X"]-TS_list[ts_ID]["Pos_X"])**2+(OS_list["Pos_Y"]-TS_list[ts_ID]["Pos_Y"])**2)

            if distance <= rospy.get_param("detecting_distance"):
                encounter = True
                encounterMMSI.append(ts_ID)
                TS_list_copy[ts_ID] = TS_list[ts_ID]
                TS_ID_copy.append(ts_ID)

        TS_ID = TS_ID_copy
        TS_list = TS_list_copy

        print(TS_ID)
        # print(TS_list)
        
        if len(encounterMMSI) == 0:
            encounter = False
            encounterMMSI = []
            
            
        # VO_operate = False

        # for rd, enc, ub, dcpa, rc, cri in zip(TS_RD_temp, TS_ENC_temp, TS_UB_temp, TS_DCPA_temp, TS_RC_temp, TS_CRI_temp):
        #     VO_operate_list = []
        #     if cri > 0.579:
        #         VO_operate = True
        #         VO_operate_list.append(VO_operate)
        #         # print(VO_operate_list)
             
        #     # if rd < 50:   
        #     if rd < 34.5:
        #         VO_operate = True
        #         VO_operate_list.append(VO_operate)
        #         print(VO_operate_list)
        #         # print("RD 규칙 적용")
                
        #     # if dcpa < 140:    
        #     if dcpa <= 60.31:
        #         VO_operate = True
        #         # print(VO_operate)

        #     # if enc== "safe":
        #     #     VO_operate = False
        #         # print("ENC 규칙 적용")
            
        # print("VO_operate : ", VO_operate)

        # NOTE: `VO_update()` takes the majority of the computation time
        # TODO: Reduce the computation time of `VO_update()`
        # V_opt, VO_BA_all = Local_PP.VO_update(OS_list, TS_list_sort, static_OB, V_des, v_min)

        ############################ for connect with KRISO format ##################################

        # data.static_obstacle_info = data.static_available_info + data.static_unavailable_info

        ############################ for connect with KRISO format ##################################

        reachableVel_global_all, vector_histogram = VFH_pp.VFH_RV_generate(
            OS_list,
            data.static_obstacle_info,
            data.static_point_info,
            data.static_OB_array_origin,
            data.os_position_origin,
            static_obstacle_key
            )

        V_selected, pub_collision_cone= Local_PP.VO_update(
            OS_list, 
            TS_list, 
            V_des, 
            reachableVel_global_all,
            )
        
        V_opt = V_selected

        # TODO: Reduce the computation time for this part (~timeChckpt4_vesselNode)
        desired_spd_list = []
        desired_heading_list = []

        # NOTE: Only one step ahead
        wp = inha.waypoint_generator(OS_list, V_selected, dt)
        wp_x = wp[0]
        wp_y = wp[1]

        if VO_operate:
            pass
        else:
            V_selected = V_des

        eta, eda = inha.eta_eda_assumption(wp, OS_list, target_speed)            
        temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
        desired_spd_list.append(temp_spd)
        desired_heading_list.append(temp_heading_deg)

        desired_spd = desired_spd_list[0]
        desired_heading = desired_heading_list[0]

        if t%10 ==0:
            pass

        t += 1

        if len(data.target_heading_list) != rospy.get_param('filter_length'):
            data.target_heading_list.append(desired_heading)
        
        else:
            del data.target_heading_list[0]

        sum_of_heading = 0
        real_target_heading = 0
        for i in data.target_heading_list:
            sum_of_heading = sum_of_heading + i

        if len(data.target_heading_list) >= 2:
            if data.target_heading_list[len(data.target_heading_list)-1]*data.target_heading_list[len(data.target_heading_list)-2] < 0:
                data.target_heading_list = [data.target_heading_list[-1]]
                real_target_heading = desired_heading
            else:
                real_target_heading = sum_of_heading/len(data.target_heading_list)

        

        # # < =========  인하대 모듈에서 나온 데이터를 최종적으로 송신하는 부분
        # OS_pub_list = [int(OS_ID), False, waypointIndex, [wp_x], [wp_y],desired_spd, eta, eda, 0.5, 0.0, False, [], desired_spd, desired_heading, isNeedCA, ""]
        OS_pub_list = [
            int(OS_ID), 
            False,
            # waypointIndex, # 인하대
            # int(data.waypoint_idx), # 부경대 i_way
            data.waypoint_idx, # kriso
            [wp_x], 
            [wp_y],  
            desired_spd_list, 
            eta, 
            eda, 
            False, 
            0, 
            desired_spd, 
            # desired_heading,
            real_target_heading,
            ]

        vis_pub_list = [
            int(OS_ID), 
            pub_collision_cone,
            V_opt,
            Local_goal,
            vector_histogram
        ]

        cri_pub_list = [
            TS_DCPA_temp,
            TS_TCPA_temp,
            TS_UDCPA_temp,
            TS_UTCPA_temp,
            TS_UD_temp,
            TS_UB_temp,
            TS_UK_temp,
            TS_CRI_temp,
            TS_Rf_temp,
            TS_Ra_temp,
            TS_Rs_temp,
            TS_Rp_temp,
            TS_ENC_temp,
        ]

        vo_pub_list = [
            V_selected,
            pub_collision_cone
        ]

        ship_dic2list = list(OS_list.values())

        savedata_list = [
            TS_RD_temp,
            TS_RC_temp,
            TS_K_temp,
            TS_DCPA_temp,
            TS_TCPA_temp,
            TS_UDCPA_temp,
            TS_UTCPA_temp,
            TS_UD_temp,
            TS_UB_temp,
            TS_UK_temp,
            TS_CRI_temp,
            TS_Rf_temp,
            TS_Ra_temp,
            TS_Rs_temp,
            TS_Rp_temp,
            TS_ENC_temp,
            V_selected,
            pub_collision_cone,
            VO_operate,
        ]
        # print(f"encounter: ", encounter)
        # print(f"encounterMMSI: ",encounterMMSI)

        writer.writerow(savedata_list)

        data.path_out_publish(OS_pub_list)
        data.vis_out(vis_pub_list)
        data.cri_out(cri_pub_list)
        data.vo_out(vo_pub_list)

        if local_goal_EDA < 2 * (ship_L/OS_scale) :
        # 만약 `reach criterion`와 거리 비교를 통해 waypoint 도달하였다면, 
        # 앞서 정의한 `waypint 도달 유무 확인용 flag`를 `True`로 바꾸어 `while`문 종료
            # waypointIndex = (waypointIndex + 1) % len(wpts_x_os)  #인하대
            targetspdIndex = waypointIndex
            # if waypointIndex == len(wpts_x_os) - 1:
            data.waypoint_idx = (data.waypoint_idx + 1) % len(wpts_x_os)  # kriso 
            # data.waypoint_idx = (int(data.waypoint_idx) + 1) % len(wpts_x_os) # 부경대

            # targetspdIndex = data.waypoint_idx
            # waypointIndex = (waypointIndex + 1) % len(wpts_x_os)
            # targetspdIndex = waypointIndex

                # rospy.signal_shutdown("종료")
        rate.sleep()
        
        print("Loop end time: ", time.time() - startTime)
        print("================ Node 1 loop end ================\n")

    file.close()

    rospy.spin()

if __name__ == '__main__':
    main()