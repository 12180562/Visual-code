from numpy import deg2rad, rad2deg
from math import *
from numpy import rad2deg

import numpy as np
import pandas as pd


import csv
import ast
import json


class CRI:
    def __init__(self, L, B, Xo, Yo, Xt, Yt, Co, Ct, Vo, Vt, ship_scale):
        self.ship_scale = ship_scale  
        self.L = L / self.ship_scale    #타선의 길이 [m] from pram
        self.B = B / self.ship_scale     #타선의 폭 [m]
        self.Xo = Xo    #자선 x좌표  [m]
        self.Yo = Yo    #자선 y좌표  [m] 
        self.Xt = Xt    #타선 x좌표  [m]
        self.Yt = Yt    #타선 y좌표  [m]
        self.Co = Co    #자선 Heading angle [rad]
        self.Ct = Ct    #타선 Heading angle [rad]
        self.Vo = Vo    #자선 속도   [knots]
        self.Vt = Vt    #타선 속도   [knots]
        self.ratio = self.ship_scale #1852/110  #1 해리는 1852m
        # self.ratio = (12*self.L) / self.ship_scale #1852/110  #1 해리는 1852m

    def RD(self):
        '''Relative Distance, 자선과 타선 사이의 상대 거리'''
        result = sqrt(((self.Xt - self.Xo) ** 2) + ((self.Yt - self.Yo) ** 2)) + 0.0001
        # print(result)
        return result

    def TB(self):
        '''True Bearing, 자선의 위치 기준 타선의 절대 방위, rad'''
        Xot = self.Xt - self.Xo
        Yot = self.Yt - self.Yo
        result = atan2(Yot, Xot) % (2*pi)
        return result

    def RB(self):
        '''Relative Bearing, 자선의 Heading angle에 대한 타선의 방위, rad'''
        if self.TB() - self.Co >= 0:
            result = self.TB() - self.Co
        else:
            result = self.TB() - self.Co + (2 * pi)
        return result
    
    def HAD(self):
        '''Heading angle difference, rad'''
        result = self.Ct - self.Co
        if result < 0 :
            result += 2*pi
        return result

    def Vox(self):
        '''자선 x방향 속도'''
        result = self.Vo * cos(self.Co)
        return result

    def Voy(self):
        '''자선 y방향 속도'''
        result = self.Vo * sin(self.Co)
        return result

    def Vtx(self):
        '''타선 x방향 속도'''
        result = self.Vt * cos(self.Ct)
        return result

    def Vty(self):
        '''타선 y방향 속도'''
        result = self.Vt * sin(self.Ct)
        return result

    def Vrx(self):
        result = self.Vtx() - self.Vox()
        return result

    def Vry(self):
        result = self.Vty() - self.Voy()
        return result

    def RV(self):
        '''Relative Velocity, 자선에 대한 타선의 상대속도'''
        result = sqrt(pow(self.Vrx(), 2) + pow(self.Vry(), 2)) + 0.001
        return result

    def RC(self):
        '''Relative speed heading direction, 상대속도(RV)의 방향'''
        result = atan2(self.Vry(), self.Vrx()) % (2*pi)
        return result

    def tcpa(self):
        result = (self.RD() * cos(self.RC() - self.TB() - pi))/self.RV()
        return result

    def dcpa(self):
        result = sqrt(pow(self.RD(), 2) + pow((self.tcpa() * self.RV()), 2))
        return result

    def d1(self):
        '''Safe approaching distance'''
        # RB = np.rad2deg(self.RB())
        # if 0 <= RB < 112.5:
        #     result = self.ratio * (1.1 - 0.2 * (self.RB()/pi))
        # elif 112.5 <= RB < 180:
        #     result = self.ratio * (1.0 - 0.4 * (self.RB()/pi))
        # elif 180 <= RB < 247.5:
        #     result = self.ratio * (1.0 - 0.4 * ((2 * pi - self.RB())/pi))
        # else:
        #     result = self.ratio * (1.1 - 0.2 * ((2 * pi - self.RB())/pi))
        # result = self.ratio * (1.1 - 0.2 * (self.RB()/pi))
        # print(self.RB())
        result = self.L * 10 
        return result

    def d2(self):
        '''Safe passing distance'''
        result = 2 * self.d1()
        return result

    def UDCPA(self):
        '''#d1, d2의 범위에 따른 DCPA의 계수'''
        if abs(self.dcpa()) <= self.d1():
            result = 1
        elif self.d2() < abs(self.dcpa()):
            result = 0
        else:
            result = 0.5 - 0.5 * sin((pi/(self.d2() - self.d1())) * (abs(self.dcpa()) - (self.d1() + self.d2())/2))
        return result

    def D1(self):
        '''Distance of action'''
        result = 12 * self.L
        return result

    def D2(self):
        '''Distance of last action'''
        result = self.ratio * (1.7 * cos(self.RB() - np.deg2rad(19))) + sqrt(4.4 + 2.89 * pow(cos(self.RB() - np.deg2rad(19)), 2))
        return result

    def UD(self):
        '''D1, D2의 범위에 따른 Relative distance의 계수'''
        if self.RD() <= self.D1():
            result = 1
        elif self.D2() < self.RD():
            result = 0
        else:
            result = pow((self.D2() - self.RD())/(self.D2() - self.D1()), 2)
        return result

    def t1(self):
        '''Collision time'''
        D1 = self.D1()
        if abs(self.dcpa()) <= D1:
            result = sqrt(pow(D1, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D1 - abs(self.dcpa())) / self.RV()
        return result

    def t2(self):
        '''Avoidance time'''
        # D2 = 12 * self.ratio  # 원래 이렇게 되어 있었음
        # D2 = 12 * 12  # 동훈 논문
        D2 = self.D2()  # 다른 논문
        if abs(self.dcpa()) <= D2:
            result = sqrt(pow(D2, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D2 - abs(self.dcpa())) / self.RV()
        return result

    def UTCPA(self):
        '''t1, t2의 범위에 따른 TCPA의 계수'''
        if self.tcpa() < 0:
            result = 0
        else:
            if self.tcpa() <= self.t1():
                result = 1
            elif self.t2() < self.tcpa():
                result = 0
            else:
                result = pow(((self.t2() - abs(self.tcpa()))/(self.t2() - self.t1())), 2)
        return result

    def UB(self):
        '''Relative bearing에 대한 계수 UB'''
        result = 0.5 * (cos(self.RB() - np.deg2rad(19)) + sqrt((440/289) + pow(cos(self.RB() - np.deg2rad(19)), 2))) - (5/17)
        return result

    def K(self):
        '''Speed factor'''
        if self.Vt == 0 or self.Vo == 0:
            result = 0.001
        else:
            result = self.Vt / self.Vo
        return result

    def sinC(self):
        '''Collision angle, UK의 계산에 사용'''
        result = abs(sin(abs(self.Ct - self.Co)))
        return result

    def UK(self):
        '''Speed factor에 대한 계수 UK'''
        result = 1 / (1 + (2 / (self.K() * sqrt(pow(self.K(), 2) + 1 + (2 * self.K() * self.sinC())))))
        return result

    def CRI(self):
        '''충돌위험도지수, UDCPA, UTCPA, UD, UB, UK 5개의 파라미터에 가중치를 곱하여 계산'''
        result = 0.4 * self.UDCPA() + 0.367 * self.UTCPA() + 0.133 * self.UD() + 0.067 * self.UB() + 0.033 * self.UK() #원래 값
        # result = 0.4457 * self.UDCPA() + 0.2258 * self.UTCPA() + 0.1408 * self.UD() + 0.1321 * self.UB() + 0.0556 * self.UK() #원준 수정 값
        return round(result, 3)

    def encounter_classification(self):
        HAD = np.rad2deg(self.HAD())
        RB = np.rad2deg(self.RB())

        if 0 <= RB <= 22.5 or 337.5 <= RB <= 360:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"


        elif 22.5 < RB <= 90:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Safe"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"


        elif 90 < RB <= 112.5:
            if 67.5 <= HAD < 202.5:
                return "Safe"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"


        elif 247.5 <= RB < 270:
            if 157.5 <= HAD <= 292.5:
                return "Safe"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            else:
                return "Overtaking"


        elif 270 <= RB < 337.5:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            elif 202.5 < HAD <= 292.5:
                return "Safe"
            else:
                return "Overtaking"

        else:
            if 0 <= HAD <= 67.5 or 292.5 <= HAD <= 360:
                if self.Vt > self.Vo:
                    return "Overtaking"
                else:
                    return "Safe"
            else:
                return "Safe"

    def CoE(self):
        '''Coefficients of encounter situations'''
        if self.encounter_classification() == "Head-on":
            s = abs(2 - (self.Vo - self.Vt)/self.Vo)
            t = 0.2
        elif self.encounter_classification() == "Starboard crossing" or self.encounter_classification() == "Port crossing":
            s = 2 - self.HAD()/pi
            t = self.HAD()/pi
        elif self.encounter_classification() == "Overtaking":
            s = 1
            t = 0.2
        else:
            s = abs(1 + (self.Vo - self.Vt)/self.Vo)
            t = abs(0.5 + (self.Vo - self.Vt)/self.Vo)
        return s, t

    def ship_domain(self):
        if self.Vo == 0.0: 
            self.Vo = 0.1

        KAD = pow(10, (0.3591 * log10(self.Vo) + 0.0952))  ## 논문에서 보면 지수함수를 사용
        KDT = pow(10, (0.5411 * log10(self.Vo) - 0.0795))  ## 논문에서 보면 지수함수를 사용 -> 
        AD = self.L * KAD
        DT = self.L * KDT

        s, t = self.CoE()

        R_fore = self.L + (0.67 * (1 + s) * sqrt(pow(AD,2) + pow(DT/2,2)))
        R_aft = self.L + (0.67 * sqrt(pow(AD,2) + pow(DT/2,2)))
        R_stbd = self.B + DT * (1 + t)
        R_port = self.B + (0.75 * DT * (1 + t))

        return R_fore, R_aft, R_stbd, R_port

    def Rf(self):
        SD = self.ship_domain()
        result = SD[0]
        return result

    def Ra(self):
        SD = self.ship_domain()
        result = SD[1]
        return result

    def Rs(self):
        SD = self.ship_domain()
        result = SD[2]
        return result

    def Rp(self):
        SD = self.ship_domain()
        result = SD[3]
        return result

    #Ship domain distance
    def SD_dist(self):
        RB = np.rad2deg(self.RB())
        Rf, Ra, Rs, Rp = self.Rf(), self.Ra(), self.Rs(), self.Rp()
        if 0 <= RB < 90:
            result = sqrt(pow(Rf,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Rf,2)/pow(Rs,2))))
        elif 90 <= RB < 180:
            result = sqrt(pow(Ra,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Ra,2)/pow(Rs,2))))
        elif 180 <= RB < 270:
            result = sqrt(pow(Ra,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Ra,2)/pow(Rp,2))))
        else:
            result = sqrt(pow(Rf,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Rf,2)/pow(Rp,2))))

        return result

class Inha_dataProcess:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(
        self,
        ship_ID, 
        Pos_X, 
        Pos_Y,
        Vel_U, 
        Heading, 
        # waypoint_dict, 
        # Pre_X,
        # Pre_Y,
        parameter,
        ):

        self.ship_ID = ship_ID
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Vel_U = Vel_U
        self.Heading = Heading
        # self.waypoint_dict = waypoint_dict
        # self.r_deg = r_deg
        # self.Drift_angle = Drift_angle
        # self.u_body = u_body,
        # self.v_body = v_body
        self.parameter = parameter

        self.ship_dic = {}
        

    def ship_list_container(self, OS_ID):

        for i in range(len(self.ship_ID)):
            index_ship = self.ship_ID[i]
            if index_ship == OS_ID:
                self.ship_dic[OS_ID] = {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Pos_X' : self.Pos_X[i],
                    'Pos_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    }
            else:
                self.ship_dic[index_ship]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Pos_X' : self.Pos_X[i],
                    'Pos_Y' : self.Pos_Y[i],                            
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    }
        return self.ship_dic, self.ship_ID
                
    
    def classify_OS_TS(self, ship_dic, ship_ID, OS_ID):
        if len(ship_ID) == 1:
            OS_list = ship_dic[OS_ID]
            TS_list = None

        else:
            for i in range(len(ship_ID)):
                if ship_ID[i] == OS_ID:
                    OS_list = ship_dic[OS_ID]
            TS_list = ship_dic.copy()
            del(TS_list[OS_ID])
        return OS_list, TS_list

    def CRI_cal(self, OS, TS):
        cri = CRI(
            L = self.parameter['ship_L'],
            B = self.parameter['ship_B'],
            Xo = OS['Pos_X'],
            Yo = OS['Pos_Y'],
            Xt = TS['Pos_X'],
            Yt = TS['Pos_Y'],
            Co = deg2rad(OS['Heading']),
            Ct = deg2rad(TS['Heading']),
            Vo = OS['Vel_U'],
            Vt = TS['Vel_U'],
            ship_scale = self.parameter['ship_scale'],            
        )
        RD = cri.RD()
        RC = cri.RC()
        TB = rad2deg(cri.TB())
        RB = rad2deg(cri.RB())

        Vox = cri.Vox()
        Voy = cri.Voy()
        Vtx = cri.Vtx()
        Vty = cri.Vty()

        DCPA = cri.dcpa()
        TCPA = cri.tcpa()
        
        K = cri.K()

        UDCPA = cri.UDCPA()
        UTCPA = cri.UTCPA()
        UD = cri.UD()
        UB = cri.UB()
        UK = cri.UK()

        enc = cri.encounter_classification()

        Rf = cri.Rf()
        Ra = cri.Ra()
        Rs = cri.Rs()
        Rp = cri.Rp()
        SD_dist = cri.SD_dist()
        # rb, lb = cri.SD_dist_new()

        cri_value = cri.CRI()

        return RD, RC, K, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value
    
    def U_to_vector_V(self, U, deg):
        psi = deg2rad(deg) ##강제로 xy좌표에서 NED좌표로 변환
        
        V_x = U * cos(psi)
        V_y = U * sin(psi)

        return V_x, V_y

    def waypoint_generator(self, OS, V_selected, dt):
        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])

        wp = OS_X + V_selected * dt

        wp_x = wp[0]
        wp_y = wp[1]

        return wp_x, wp_y
    
    def eta_eda_assumption(self, WP, OS, target_speed):

        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])

        distance = np.linalg.norm(WP - OS_X)

        eta = distance/ target_speed
        eda = distance
        
        return eta, eda

    def desired_value_assumption(self, V_des):
        U_des = sqrt(V_des[0]**2 + V_des[1]**2)
        target_head = rad2deg(atan2(V_des[1], V_des[0])) ## NED 좌표계 기준으로 목적지를 바라보는 방향각

        desired_heading = target_head        
        desired_spd = U_des

        return desired_spd, desired_heading


    def TS_info_supplement(self, OS_list, TS_list):
        if TS_list == None:
            TS_list = None
        else:
            TS_ID = TS_list.keys()
            for ts_ID in TS_ID:
                # RD, RC, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value,rb,lb = self.CRI_cal(OS_list, TS_list[ts_ID])
                RD, RC, TB, RB, K, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value = self.CRI_cal(OS_list, TS_list[ts_ID])

                TS_list[ts_ID]['RD'] = RD 
                TS_list[ts_ID]['RC'] = RC
                TS_list[ts_ID]['TB'] = TB  
                TS_list[ts_ID]['RB'] = RB
                TS_list[ts_ID]['K'] = K

                TS_list[ts_ID]['V_x'] = Vtx
                TS_list[ts_ID]['V_y'] = Vty

                TS_list[ts_ID]['DCPA'] = DCPA
                TS_list[ts_ID]['TCPA'] = TCPA

                TS_list[ts_ID]['UDCPA'] = UDCPA
                TS_list[ts_ID]['UTCPA'] = UTCPA
                TS_list[ts_ID]['UD'] = UD
                TS_list[ts_ID]['UB'] = UB
                TS_list[ts_ID]['UK'] = UK

                TS_list[ts_ID]['status'] = enc

                TS_list[ts_ID]['Rf'] = Rf
                TS_list[ts_ID]['Ra'] = Ra
                TS_list[ts_ID]['Rs'] = Rs
                TS_list[ts_ID]['Rp'] = Rp
                TS_list[ts_ID]['mapped_radius'] = 120
                # TS_list[ts_ID]["right_boundary"] = rb
                # TS_list[ts_ID]["left_boundary"] = lb

                TS_list[ts_ID]['CRI'] = cri_value

                # print(enc)

        return TS_list
    

class VO_module:
    def __init__(self, parameter):
        # NOTE: It is not clear what min and max of speed could be.
        self.min_targetSpeed = parameter['min_targetSpeed']
        self.max_targetSpeed = parameter['max_targetSpeed']
        self.num_targetSpeedCandidates = parameter['num_targetSpeedCandidates']

        # NOTE: It is not clear what min and max of heading angle could be.
        self.min_targetHeading_deg_local = parameter['min_targetHeading_deg_local']
        self.max_targetHeading_deg_local = parameter['max_targetHeading_deg_local']
        self.num_targetHeadingCandidates = parameter['num_targetHeadingCandidates']

        self.weight_alpha = parameter['weight_focusObs']
        self.weight_aggresiveness = parameter['weight_agressivness']
        self.cri_param = parameter['cri_param']
        self.time_horizon = parameter['timeHorizon']
        self.delta_t = parameter['delta_t']

        self.rule = parameter['Portside_rule']
        self.errorCode = None
        self.error = False
        
        
    def __is_all_vels_collidable(self, vel_all_annotated, shipID_all):
        for vel_annotated in vel_all_annotated:
            isVelCollidable = False
            for shipID in shipID_all:
                if (vel_annotated[shipID] == 'inCollisionCone'):
                   isVelCollidable = True
                   break
            if not isVelCollidable:
                return False

        return True

    def __is_all_vels_avoidable(self, vel_all_annotated, shipID_all):
        for vel_annotated in vel_all_annotated:
            for shipID in shipID_all:
                if (vel_annotated[shipID] == 'inCollisionCone'):
                   return False

        return True

    
    def __remove_annotation(self, vel_annotated_all):
        vels = []
        
        for vel_annotated in vel_annotated_all:
            vels.append(vel_annotated['vel'])
            
        return vels

    def __annotate_vels(self, reachableVel_global_all, RVOdata_all, TS):
        # Make the `reachableVel_global_all` dictionary
        reachableVel_global_all_annotated = []

        for reachableVel_global in reachableVel_global_all:
            
            reachableVel_global_annotated = {'vel': reachableVel_global}

            for RVOdata in RVOdata_all:
                '''
                `RVOdata` structure:
                    RVOdata = {
                        "TS_ID",
                        "LOSdist", 
                        "mapped_radius", 
                        "vA",
                        "vB",
                        "boundLineAngle_left_rad_global", 
                        "boundLineAngle_right_rad_global", 
                        "collisionConeShifted_local",
                        "CRI"
                        }
                '''
                # NOTE: vA2B_RVO is the relative velocity from the agent A to B
                #       on the RVO configuration space, not the VO configuration space
                vA2B_RVO = reachableVel_global - RVOdata['collisionConeTranslated']

                angle_vA2B_RVO_rad_global = atan2(
                    vA2B_RVO[1],
                    vA2B_RVO[0],
                    )
                
                if self.__is_in_left(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'],
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    LOSangle_rad_global=0.5*(RVOdata['boundLineAngle_left_rad_global']+RVOdata['boundLineAngle_right_rad_global']),
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inLeft'
                
                elif self.__is_in_right(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'],
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    LOSangle_rad_global=0.5*(RVOdata['boundLineAngle_left_rad_global']+RVOdata['boundLineAngle_right_rad_global']),
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inRight'
                
                elif self.__is_within_time_horizon(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'], 
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    velVecNorm=np.linalg.norm(vA2B_RVO),
                    shortestRelativeDist=RVOdata['LOSdist']-RVOdata['mapped_radius'],
                    timeHorizon=RVOdata['CRI']*self.cri_param,
                    # timeHorizon=self.time_horizon
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inTimeHorizon'
                
                elif self.__is_in_collision_cone(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'], 
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    velVecNorm=np.linalg.norm(vA2B_RVO),
                    shortestRelativeDist=RVOdata['LOSdist']-RVOdata['mapped_radius'],
                    timeHorizon=RVOdata['CRI']*self.cri_param,
                    # timeHorizon=self.time_horizon
                    ):
                    
                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inCollisionCone'

                else:
                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inCollisionCone'

            reachableVel_global_all_annotated.append(reachableVel_global_annotated)

        return reachableVel_global_all_annotated

    def __take_vels(self, vel_all_annotated, annotation, shipID_all):
        vel_hasAnnotation_all_annotated = []
        for vel_annotated in vel_all_annotated:
            hasAnnotation = True
            for shipID in shipID_all:
                if vel_annotated[shipID] not in annotation:
                    hasAnnotation = False
                    break
            if hasAnnotation:
                vel_hasAnnotation_all_annotated.append(vel_annotated)

        return vel_hasAnnotation_all_annotated

    def __is_in_between(self, theta_given, theta_left, theta_right):
        if abs(theta_right - theta_left) <= pi:
            if theta_left < 0 and theta_right < 0:
                if theta_given > 0:
                    theta_given -= 2*pi
            elif theta_left > 0 and theta_right > 0:
                if theta_given < 0:
                    theta_given += 2*pi
            if theta_right <= theta_given <= theta_left:    
                return True
            else :
                return False
        else:
            if (theta_left < 0) and (theta_right > 0):
                ## 각도 보정 
                theta_left += 2*pi
                if theta_given <0:
                    theta_given += 2*pi

                if theta_right <= theta_given <= theta_left:
                    return True
                else :
                    return False
            
            if (theta_left > 0) and (theta_right <0):
                theta_right += 2*pi            
                if theta_given < 0:
                    theta_given += 2*pi
                    
                if theta_left <= theta_given <= theta_right:
                    return True
                else:
                    return False

    def __is_in_left(
        self,  
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        LOSangle_rad_global,
        ):
        
        velVecAngle_translated_rad_global = velVecAngle_rad_global - LOSangle_rad_global
        while velVecAngle_translated_rad_global < 0:
            velVecAngle_translated_rad_global += 2*pi
        velVecAngle_translated_rad_global %= (2*pi)

        if self.__is_in_between(
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ):
            return False
        elif (0 < velVecAngle_translated_rad_global <= pi):
            return True
        elif (pi < velVecAngle_translated_rad_global <= 2*pi):
            return False

    def __is_in_right(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        LOSangle_rad_global,
        ):
        
        velVecAngle_translated_rad_global = velVecAngle_rad_global - LOSangle_rad_global
        while velVecAngle_translated_rad_global < 0:
            velVecAngle_translated_rad_global += 2*pi
        velVecAngle_translated_rad_global %= (2*pi)

        if self.__is_in_between(
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global, 
            boundLineAngle_right_rad_global,
            ):
            return False
        elif (0 < velVecAngle_translated_rad_global <= pi):
            return False
        elif (pi < velVecAngle_translated_rad_global <= 2*pi):
            return True

    def __is_within_time_horizon(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        velVecNorm,
        shortestRelativeDist, 
        timeHorizon,
        ):
        if self.__is_in_between(
            velVecAngle_rad_global,
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ) and (velVecNorm <= shortestRelativeDist / timeHorizon):
            return True
        else:
            return False

    def __is_in_collision_cone(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        velVecNorm,
        shortestRelativeDist, 
        timeHorizon,
        ):
        if self.__is_in_between(
            velVecAngle_rad_global,
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ) and (velVecNorm > shortestRelativeDist / timeHorizon):
            return True
        else:
            return False

    def __generate_vel_candidates(self, targetSpeed_all, targetHeading_rad_global_all, OS, static_obstacle_info, static_point_info):
        reachableVelX_global_all = np.zeros(self.num_targetSpeedCandidates * self.num_targetHeadingCandidates)
        reachableVelY_global_all = np.zeros(self.num_targetSpeedCandidates * self.num_targetHeadingCandidates)
        
        idx = int(0)
        for targetHeading_rad_global in targetHeading_rad_global_all:
            for targetSpeed in targetSpeed_all:

                reachableVelX_global_all[idx] = targetSpeed * cos(targetHeading_rad_global)
                reachableVelY_global_all[idx] = targetSpeed * sin(targetHeading_rad_global)

                idx += 1

        reachableVelX_global_all = np.reshape(reachableVelX_global_all, newshape=(-1, 1))
        reachableVelY_global_all = np.reshape(reachableVelY_global_all, newshape=(-1, 1))

        reachableVel_global_all = np.concatenate(
            (reachableVelX_global_all, reachableVelY_global_all),
             axis=-1,
             )
        

        reachableVel_global_all_after_obstacle = self.__delete_vector_inside_obstacle(reachableVel_global_all, OS, static_obstacle_info,static_point_info)
        

        return reachableVel_global_all_after_obstacle


    def __delete_vector_inside_obstacle(self, reachableVel_global_all, OS, static_obstacle_info, static_point_info):

        pA = np.array([OS['Pos_X'], OS['Pos_Y']])

        reachableVel_global_all_copy = np.copy(reachableVel_global_all)
        static_OB_data = static_obstacle_info
        static_point_data = static_point_info
        
        pA = np.array([OS['Pos_X'], OS['Pos_Y']])
        delta_t = self.delta_t # constant
        detecting_radious = 100

        #initial number for while
        obstacle_number = 0
        point_number = 0

        # obstacle_radious
        radious = 5

        while (obstacle_number) != len(static_OB_data):
            obstacle_point_x = [static_OB_data[obstacle_number],static_OB_data[obstacle_number+2]] 
            obstacle_point_y = [static_OB_data[obstacle_number+1],static_OB_data[obstacle_number+3]] 

            if obstacle_point_x[0] > obstacle_point_x[1]:
                obstacle_point_x.reverse()
                obstacle_point_y.reverse()

            if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                slope = 9999

            elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                slope =-9999

            else: 
                slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0])
                
            reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t 
            result = []

            for i in range(len(reachableVel_global_all_after_delta_t)): 
                after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]
                if (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) < 0:
                    vector_slope = 9999

                elif (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) > 0:
                    vector_slope = -9999
                else:
                    vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                if self.get_crosspt(slope, 
                        vector_slope,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        after_delta_t_x, 
                        after_delta_t_y):

                    component = reachableVel_global_all[i]
                    component_list = component.tolist()
                    result.append(component_list)
                else:
                    pass

            reachableVel_global_all = np.array(result)

            obstacle_number = obstacle_number+4

        while point_number != len(static_point_data):
        
            point_x = static_point_data[point_number]
            point_y = static_point_data[point_number+1]

            reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t 
            result = []

            for i in range(len(reachableVel_global_all_after_delta_t)-1): 
                after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]

                if (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) < 0:
                    vector_slope = 9999

                elif (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) > 0:
                    vector_slope = -9999
                else:
                    vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                if self.get_crosspt_circle(vector_slope,
                        point_x, 
                        point_y, 
                        radious, 
                        pA[0], 
                        pA[1], 
                        after_delta_t_x, 
                        after_delta_t_y ):
                    
                    component = reachableVel_global_all[i]
                    component_list = component.tolist()
                    result.append(component_list)

                else:
                    pass

            reachableVel_global_all = np.array(result)

            point_number = point_number+2

        if len(reachableVel_global_all) == 0:
            reachableVel_global_all = reachableVel_global_all = np.array([reachableVel_global_all_copy[-1,:]])
            print("------ All of vector candidates have collision risk with static obstacle ------")
            self.errorCode = 310
            self.error = True
        else:
            pass

        return reachableVel_global_all
    
    def if_all_vector_collidable(self, OS, effective_static_OB, detecting_radious, reachableVel_global_all_copy):
        space_number = 20
        pA = [OS['Pos_X'], OS['Pos_Y']]
        vector_radian_plus = 0
        vector_radian_minus = 0
        vector_space_radian = pi/space_number
        detecting_vector_list = []
        detecting_vector_list_in_right = []
        detecting_vector_list_in_left = []
        for i in range(space_number):
            vector_radian_plus = vector_radian_plus + vector_space_radian * i
            vector_radian_minus = vector_radian_minus - vector_space_radian * i
            vector_point_plus = [(pA[0]+detecting_radious*cos(vector_radian_plus)),(pA[0]+detecting_radious*sin(vector_radian_plus))]
            vector_point_minus = [(pA[0]+detecting_radious*cos(vector_radian_minus)),(pA[0]+detecting_radious*sin(vector_radian_minus))]
            vector_slope_plus = tan(vector_radian_plus)
            vector_slope_minus = tan(vector_radian_minus)
            obstacle_number = 0

            while (obstacle_number) != len(effective_static_OB):
                obstacle_point_x = [effective_static_OB[obstacle_number],effective_static_OB[obstacle_number+2]] 
                obstacle_point_y = [effective_static_OB[obstacle_number+1],effective_static_OB[obstacle_number+3]] 

                if obstacle_point_x[0] > obstacle_point_x[1]:
                    obstacle_point_x.reverse()
                    obstacle_point_y.reverse()

                if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                    slope = 9999

                elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                    slope =-9999

                else: 
                    slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0])

                if self.get_crosspt(slope, 
                        vector_slope_plus,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        vector_point_plus[0], 
                        vector_point_plus[1]):

                    detecting_vector_list_in_right.append(vector_radian_plus)

                elif self.get_crosspt(slope, 
                        vector_slope_minus,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        vector_point_minus[0], 
                        vector_point_minus[1]):

                    detecting_vector_list_in_left.append(vector_radian_minus)
                else:
                    pass

            if len(detecting_vector_list) == 0:
                reachableVel_global_all = np.array([reachableVel_global_all_copy[182,:]])

            else:
                if len(detecting_vector_list_in_right) >= len(detecting_vector_list_in_left):
                    reachableVel_global_all = np.array([reachableVel_global_all_copy[182,:]])
                    # select vector that is in right
                else:
                    reachableVel_global_all = np.array([reachableVel_global_all_copy[123,:]])
                    # select vector that is in left

            return reachableVel_global_all

    def get_crosspt(self, slope, vector_slope, start_x, start_y,end_x, end_y, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y):

        x_point = [start_x, end_x]
        y_point = [start_y, end_y]

        if (slope) == (vector_slope): 
            return True

        else:
            cross_x = (start_x * slope - start_y - OS_pos_x * vector_slope + OS_pos_y) / (slope - vector_slope)
            cross_y = slope * (cross_x - start_x) + start_y

            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and OS_pos_y <= cross_y <= after_delta_t_y:

                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and OS_pos_y <= cross_y <= after_delta_t_y:

                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and  after_delta_t_y <= cross_y <= OS_pos_y:

                        return False
                    else:
                        return True
                else:
                    return True

            else:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and after_delta_t_y <= cross_y <= OS_pos_y:

                        return False
                    else:
                        return True
                else:
                    return True
                
    def get_crosspt_circle(self, vector_slope, point_x, point_y, radious, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y ):

        point = [point_x,point_y]
        r = radious

        vector_point_x = [OS_pos_x,after_delta_t_x]
        vector_point_y = [OS_pos_y,after_delta_t_y]

        slope = vector_slope

        a = slope
        b = -1
        c = -slope*vector_point_x[0]+vector_point_y[0]

        xp = point[0]
        yp = point[1]

        A = 1+(a**2/b**2)
        B = (-2*xp)+(2*a*c/b**2)+(2*a*yp/b)
        C = (xp**2)+(c**2/b**2)+(2*c*yp/b)+(yp**2)-(r**2)

        discriminant = (B**2)-(4*A*C)

        if discriminant >= 0:
            cross_x_1 = (-B+sqrt(discriminant))/(2*A)
            cross_y_1 = slope*(cross_x_1-vector_point_x[0])+vector_point_y[0]


            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if OS_pos_x <= cross_x_1 <= after_delta_t_x and OS_pos_y <= cross_y_1 <= after_delta_t_y:
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if after_delta_t_x <= cross_x_1 <= OS_pos_x and OS_pos_y <= cross_y_1 <= after_delta_t_y:
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if OS_pos_x <= cross_x_1 <= after_delta_t_x and  after_delta_t_y <= cross_y_1 <= OS_pos_y:
                        return False
                    else:
                        return True
                else:
                    return True

            else:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if after_delta_t_x <= cross_x_1 <= OS_pos_x and after_delta_t_y <= cross_y_1 <= OS_pos_y:
                        return False
                    else:
                        return True
                else:
                    return True

        else:
            return True


    def __select_vel_inside_RVOs(self, reachableCollisionVel_global_all, RVOdata_all, V_des):
        # Compute the minimum time to collision(tc) for every velocity candidates
        velCandidates_dict = dict()

        for reachableCollisionVel_global in reachableCollisionVel_global_all:
            
            velCandidates_dict[tuple(reachableCollisionVel_global)] = dict()
            tc_min = 99999.9 # initialize the time to collision to a large enough

            # Compute the minimum time to collision(tc) for a velocity candidate
            for RVOdata in RVOdata_all:

                vA2B_RVO = reachableCollisionVel_global - RVOdata['collisionConeTranslated']

                angle_vA2B_RVO_rad_global = atan2(
                    vA2B_RVO[1],
                    vA2B_RVO[0],
                    )

                # Consider only collidable agent.
                # NOTE: tc will maintain a large initial value if no collision 
                #       for the corresponding agent.
                if self.__is_in_between(
                RVOdata['boundLineAngle_right_rad_global'],
                angle_vA2B_RVO_rad_global, 
                RVOdata['boundLineAngle_left_rad_global'],
                ):

                    angle_at_pA = abs(angle_vA2B_RVO_rad_global - 0.5 * (RVOdata['boundLineAngle_right_rad_global'] + RVOdata['boundLineAngle_left_rad_global']))   
                    angle_at_pA %= (2*pi)
                    if angle_at_pA > pi:
                        angle_at_pA = abs(angle_at_pA - (2*pi))                

                    if (abs(RVOdata['LOSdist'] * sin(angle_at_pA)) > RVOdata['mapped_radius']):
                        angle_at_collisionPoint = 0.0    
                    else:
                        angle_at_collisionPoint = asin(
                            abs(RVOdata['LOSdist'] * sin(angle_at_pA)) / RVOdata['mapped_radius']
                            )

                    dist_collision = abs(RVOdata['LOSdist'] * cos(angle_at_pA)) - abs(RVOdata['mapped_radius'] * cos(angle_at_collisionPoint))

                    if dist_collision < 0: dist_collision = 0
                    tc = dist_collision / np.linalg.norm([vA2B_RVO])
                    
                    # NOTE: Avoid zero division for the penalty calculation
                    if tc < 0.00001: tc = 0.00001

                    if tc < tc_min: tc_min = tc

            # Store the minimum time to collision for each velocity candidate
            velCandidates_dict[tuple(reachableCollisionVel_global)]['tc_min'] = tc_min

            # Comput and store the penalty for each velocity candidate
            velCandidates_dict[tuple(reachableCollisionVel_global)]['penalty'] = self.weight_aggresiveness / tc_min + np.linalg.norm([V_des - reachableCollisionVel_global])
        # Take the velocity that has the minimum penalty
        vA_post = min(velCandidates_dict, key=lambda k : velCandidates_dict[k]['penalty'])
        return vA_post

    def __choose_velocity(self, V_des, RVOdata_all, OS, TS, static_obstacle_info, static_point_info): 
        # Generate target speed candidates
        # NOTE: We generate the target velocity candidates manually, not deriving from mmg and feasible acc.
        targetSpeed_all = np.linspace(
            start=self.min_targetSpeed, 
            stop=self.max_targetSpeed, 
            num=self.num_targetSpeedCandidates,
            )

        TS_ID = TS.keys()
        for ts_ID in TS_ID:
            status = TS[ts_ID]['status']

        # Generate target heading angle candidates
        min_targetHeading_rad_local = np.deg2rad(self.min_targetHeading_deg_local)
        max_targetHeading_rad_local = np.deg2rad(self.max_targetHeading_deg_local)
        heading_rad = np.deg2rad(OS['Heading'])

        min_targetHeading_rad_global = heading_rad + min_targetHeading_rad_local
        max_targetHeading_rad_global = heading_rad + max_targetHeading_rad_local

        targetHeading_rad_global_all = np.linspace(
            start=min_targetHeading_rad_global, 
            stop=max_targetHeading_rad_global, 
            num=self.num_targetHeadingCandidates,
            )

        # Generate target velocity vector candidates
        reachableVel_global_all = self.__generate_vel_candidates(
            targetSpeed_all, 
            targetHeading_rad_global_all,
            OS,
            static_obstacle_info,
            static_point_info
            )
        
        # Annotate the velocities - 'in time horizon', 'in left', 'in right', 'in collision cone'
        reachableVel_all_annotated = self.__annotate_vels(
            reachableVel_global_all, 
            RVOdata_all, 
            TS,
            )

    

        isAllVelsCollidable = self.__is_all_vels_collidable(
            vel_all_annotated=reachableVel_all_annotated, 
            shipID_all=TS.keys(),
            )

        isAllVelsAvoidable = self.__is_all_vels_avoidable(
            vel_all_annotated=reachableVel_all_annotated, 
            shipID_all=TS.keys(),
            )

        # When no avoidance velocities
        if isAllVelsCollidable:
            velCandidates = self.__remove_annotation(reachableVel_all_annotated)
            # NOTE: `_select_vel_inside_RVOs()` returns a velocity where 
            #       all the reachable velocities are inside the collision cones
            vA_post = self.__select_vel_inside_RVOs(
                velCandidates, 
                RVOdata_all, 
                V_des,
                )

        # When no collision velocities
        elif isAllVelsAvoidable:
            velCandidates = self.__remove_annotation(reachableVel_all_annotated)
            vA_post = min(
                velCandidates,
                key= lambda v: np.linalg.norm(v - V_des),
                )
            

        # When partially have avoidance velocities
        else:
            avoidanceVel_all_annotated = self.__take_vels(
                vel_all_annotated=reachableVel_all_annotated,
                annotation=['inLeft', 'inRight', 'inTimeHorizon'],
                shipID_all=TS.keys(),
                )

            avoidanceAllRightVel_all_annotated = self.__take_vels(  
                vel_all_annotated=reachableVel_all_annotated,       
                annotation=['inLeft'],                              
                shipID_all=TS.keys(),
                )


            if avoidanceAllRightVel_all_annotated:
                velCandidates = self.__remove_annotation(avoidanceAllRightVel_all_annotated)
            else:
                velCandidates = self.__remove_annotation(avoidanceVel_all_annotated)
            vA_post = min(
                velCandidates,
                key= lambda v: np.linalg.norm(v - V_des),
                )


        return vA_post 

    def __extract_RVO_data(self, OS, TS):

        vA = np.array([OS['V_x'], OS['V_y']])
        pA = np.array([OS['Pos_X'], OS['Pos_Y']])

        RVOdata_all = []
        pub_collision_cone = []
        TS_ID = TS.keys()

        for ts_ID in TS_ID:

            vB = np.array([TS[ts_ID]['V_x'], TS[ts_ID]['V_y']])
            pB = np.array([TS[ts_ID]['Pos_X'], TS[ts_ID]['Pos_Y']])

            CRI = TS[ts_ID]['CRI']
            status = TS[ts_ID]['status']

            RVOapexPos_global = pA + (1 - self.weight_alpha) * vA + self.weight_alpha * vB
            LOSdist = np.linalg.norm([pA - pB]) 
            LOSangle_rad = atan2(pB[1] - pA[1], pB[0] - pA[0])
            
            boundLineAngle_left_rad_global = LOSangle_rad + atan2(TS[ts_ID]['mapped_radius'],LOSdist)
            boundLineAngle_right_rad_global = LOSangle_rad - atan2(TS[ts_ID]['mapped_radius'],LOSdist)
            
            collisionConeTranslated = (1 - self.weight_alpha) * vA + self.weight_alpha * vB
            
            RVOdata = {
                "TS_ID": ts_ID,
                "LOSdist": LOSdist,
                "mapped_radius": TS[ts_ID]['mapped_radius'],
                "vA": vA,
                "vB": vB,
                "boundLineAngle_left_rad_global" : boundLineAngle_left_rad_global,
                "boundLineAngle_right_rad_global" : boundLineAngle_right_rad_global,
                "collisionConeTranslated": collisionConeTranslated,
                "CRI": CRI,
                }
            RVOdata_all.append(RVOdata)
            bound_left_view = [
                cos(boundLineAngle_left_rad_global)* int(LOSdist)/2,
                sin(boundLineAngle_left_rad_global)* int(LOSdist)/2,
                ]
            bound_right_view = [
                cos(boundLineAngle_right_rad_global)* int(LOSdist)/2,
                sin(boundLineAngle_right_rad_global)* int(LOSdist)/2,
                ]

            pub_collision_cone.append(RVOapexPos_global[0])
            pub_collision_cone.append(RVOapexPos_global[1])
            pub_collision_cone.append(bound_left_view[0] + RVOapexPos_global[0])
            pub_collision_cone.append(bound_left_view[1] + RVOapexPos_global[1])
            pub_collision_cone.append(bound_right_view[0] + RVOapexPos_global[0])
            pub_collision_cone.append(bound_right_view[1] + RVOapexPos_global[1])
            

        return RVOdata_all, pub_collision_cone

    def VO_update(self, OS_original, TS_original, V_des, static_obstacle_info, static_point_info):   

        RVOdata_all, pub_collision_cone = self.__extract_RVO_data(
            OS_original,
            TS_original,
            )

        V_opt = self.__choose_velocity(V_des, RVOdata_all, OS_original, TS_original,static_obstacle_info, static_point_info)

        return V_opt, pub_collision_cone

    def vectorV_to_goal(self, OS, goal, V_max):
        Pos_x = OS['Pos_X']
        Pos_y = OS['Pos_Y']
        dif_x = [goal[0] - Pos_x, goal[1] - Pos_y]
        norm = np.linalg.norm(dif_x)
        V_des = [dif_x[k] * V_max / norm for k in range(2)]

        return V_des
    
def convert_to_list(val):
    try:
        if isinstance(val, str):
            # 'nan'을 'None'으로 변환
            val = val.replace('nan', 'None')
            val = ast.literal_eval(val)
            # None을 np.nan으로 변환
            return [np.nan if v is None else v for v in val]
        return val
    except (ValueError, SyntaxError):
        return val
    
# 문자열을 튜플로 변환하는 함수 정의
def convert_to_tuple(val):
    try:
        if isinstance(val, str):
            # 'nan'을 np.nan으로 변환
            val = val.replace('nan', 'None')
            val = ast.literal_eval(val)
            # None을 np.nan으로 변환
            return tuple(np.nan if v is None else v for v in val)
        return val
    except (ValueError, SyntaxError):
        return val



with open('./parameter.json', 'r') as file:
    parameter = json.load(file)

df = pd.read_csv("C:/Users/wonjun/Desktop/Generate_data/YOO/inha_scenario(crossing_wpchange)_2024-07-31-12-25-40-frm_info_pre.csv")

# 문자열을 리스트로 변환하는 함수 정의

df['.m_nShipID'] = df['.m_nShipID'].apply(convert_to_list)

columns_to_convert = ['.m_fltHeading', '.m_fltVel_U', '.m_fltPos_X', '.m_fltPos_Y']
for col in columns_to_convert:
    df[col] = df[col].apply(convert_to_tuple)






for index, row in df.iterrows():
    ship_ID = row['.m_nShipID']
    Pos_X = row['.m_fltPos_X']
    Pos_Y = row['.m_fltPos_Y']
    Vel_U = row['.m_fltVel_U']
    Heading = row['.m_fltHeading']
    parameter = parameter
    dataprocess = Inha_dataProcess(ship_ID, Pos_X, Pos_Y, Vel_U, Heading, parameter)
    ship_dic, ID = dataprocess.ship_list_container(2000)
    OS_list, TS_list = dataprocess.classify_OS_TS(ship_dic, ID, 2000)

    OS_Vx, OS_Vy = dataprocess.U_to_vector_V(OS_list['Vel_U'], OS_list['Heading'])
    TS_list = dataprocess.TS_info_supplement(OS_list, TS_list)
    # df.at[index, 'TS_list'] = str(TS_list)
    # df.to_csv("C:/Users/SMY/Desktop/연구실/11. 데이터/06. 2024 내수면 시험 데이터/rosbag_to_csv/202405291434/generate_data.csv")

    TS_ID = ID[:]
    TS_ID.remove(2000)
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
        TS_RD_temp.append(TS_list[ts_ID]['RD'])
        TS_RC_temp.append(TS_list[ts_ID]['RC'])
        TS_K_temp.append(TS_list[ts_ID]['K'])
        TS_DCPA_temp.append(TS_list[ts_ID]['DCPA'])
        TS_TCPA_temp.append(TS_list[ts_ID]['TCPA'])
        TS_UDCPA_temp.append(TS_list[ts_ID]['UDCPA'])
        TS_UTCPA_temp.append(TS_list[ts_ID]['UTCPA'])
        TS_UD_temp.append(TS_list[ts_ID]['UD'])
        TS_UB_temp.append(TS_list[ts_ID]['UB'])
        TS_UK_temp.append(TS_list[ts_ID]['UK'])
        TS_CRI_temp.append(TS_list[ts_ID]['CRI'])
        TS_Rf_temp.append(TS_list[ts_ID]['Rf'])
        TS_Ra_temp.append(TS_list[ts_ID]['Ra'])
        TS_Rs_temp.append(TS_list[ts_ID]['Rs'])
        TS_Rp_temp.append(TS_list[ts_ID]['Rp'])
        TS_ENC_temp.append(TS_list[ts_ID]['status'])

    df.at[index, 'RD'] = str(TS_RD_temp)
    df.at[index, 'RC'] = str(TS_RC_temp)
    df.at[index, 'K'] = str(TS_K_temp)
    df.at[index, 'DCPA'] = str(TS_DCPA_temp)
    df.at[index, 'TCPA'] = str(TS_TCPA_temp)
    df.at[index, 'UDCPA'] = str(TS_UDCPA_temp)
    df.at[index, 'UTCPA'] = str(TS_UTCPA_temp)
    df.at[index, 'UD'] = str(TS_UD_temp)
    df.at[index, 'UB'] = str(TS_UB_temp)
    df.at[index, 'UK'] = str(TS_UK_temp)
    df.at[index, 'CRI'] = str(TS_CRI_temp)
    df.at[index, 'Rf'] = str(TS_Rf_temp)
    df.at[index, 'Ra'] = str(TS_Ra_temp)
    df.at[index, 'Rs'] = str(TS_Rs_temp)
    df.at[index, 'Rp'] = str(TS_Rp_temp)
    df.at[index, 'ENC'] = str(TS_ENC_temp)

df.columns = ['time', '.m_nShipID', '.m_fltHeading', '.m_fltVel_U', '.m_fltPos_X', '.m_fltPos_Y', '.RD','.RC', '.K', '.DCPA', '.TCPA', '.UDCPA', 'UTCPA', 
    '.UD', '.UB', '.UK', '.CRI', '.Rf', '.Ra', '.Rs', '.Rp', '.encounter_classification']



df.to_csv("C:/Users/wonjun/Desktop/Generate_data/YOO/inha_scenario(crossing_wpchange)_2024-07-31-12-25-40-frm_info_final.csv", index=False)

