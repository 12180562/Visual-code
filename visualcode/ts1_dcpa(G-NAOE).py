import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.pyplot as pyplot
import numpy as np
import pandas as pd
import re
from math import *
import glob

class Visualization:
    def __init__(self, ship_ID, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt1, Xt1_list, Yt1, Yt1_list,
    Ct1, Rf1, Ra1, Rs1, Rp1, Vx, Vy, Gx, Gy, 
    C1x, C1y, C2x, C2y, C3x, C3y, DCPA1, TCPA1, cri1, Heading, time):

        self.ship_ID = ship_ID
        self.L = L
        self.B = B
        self.Xo = Xo
        self.Xo_list = Xo_list
        self.Yo = Yo
        self.Yo_list = Yo_list
        self.Co = np.deg2rad(Co)

        self.Xt1 = Xt1
        self.Xt1_list = Xt1_list
        self.Yt1 = Yt1
        self.Yt1_list = Yt1_list

        self.Ct1 = np.deg2rad(Ct1)

        self.Rf1 = round(Rf1, 4)
        self.Ra1 = round(Ra1, 4)
        self.Rs1 = round(Rs1, 4)
        self.Rp1 = round(Rp1, 4)

        self.Vx = Vx *3
        self.Vy = Vy *3
        self.Gx = Gx
        self.Gy = Gy

        self.C1x = C1x
        self.C1y = C1y
        self.C2x = C2x
        self.C2y = C2y
        self.C3x = C3x
        self.C3y = C3y

        self.DCPA1 = DCPA1

        self.TCPA1 = TCPA1

        self.cri1 = cri1

        self.Heading = Heading
        self.time = time

    def transform(self, x, y, theta, Xs, Ys):
        x_trans = (cos(theta) * x + sin(theta) * y) + Xs
        y_trans = (-sin(theta) * x + cos(theta) * y) + Ys

        return x_trans, y_trans

    def ship_domain(self, Rf, Ra, Rs, Rp):
        x1 = np.linspace(0, Rs-0.0001, 50)
        y1 = ((Rf**2)-(Rf**2/Rs**2)*(x1**2)) ** 0.5
        y2 = -((Ra**2)-(Ra**2/Rs**2)*(x1**2)) ** 0.5


        x2 = np.linspace(-Rp+0.0001, 0, 50)
        y3 = ((Rf**2)-(Rf**2/Rp**2)*x2**2) ** 0.5
        y4 = -((Ra**2)-(Ra**2/Rp**2)*x2**2) ** 0.5

        return x1, x2, y1, y2, y3, y4

    def create_ship_domain(self, Rf, Ra, Rs, Rp):
        SD_points = self.ship_domain(Rf, Ra, Rs, Rp)
        Q1 = self.transform(SD_points[0], SD_points[2], self.Co, self.Xo, self.Yo)
        Q2 = self.transform(SD_points[0], SD_points[3], self.Co, self.Xo, self.Yo)
        Q3 = self.transform(SD_points[1], SD_points[4], self.Co, self.Xo, self.Yo)
        Q4 = self.transform(SD_points[1], SD_points[5], self.Co, self.Xo, self.Yo)

        return Q1, Q2, Q3, Q4

    def create_coord_OS(self, Xo, Yo, Co):
        OS_boundary1 = self.transform(0, 2.25, Co, Xo, Yo)
        OS_boundary2 = self.transform(0.45, -2.25, Co, Xo, Yo)
        OS_boundary3 = self.transform(-0.45, -2.25, Co, Xo, Yo)
        
        return OS_boundary1, OS_boundary2, OS_boundary3

    def create_coord_TS(self, Xt, Yt, Ct):
        TS_boundary1 = self.transform(0, 2.25, Ct, Xt, Yt)
        TS_boundary2 = self.transform(0.45, -2.25, Ct, Xt, Yt)
        TS_boundary3 = self.transform(-0.45, -2.25, Ct, Xt, Yt)

        return TS_boundary1, TS_boundary2, TS_boundary3

    def ploting(self, name = None):

        f,axes = plt.subplots(1, 2)
        f.set_size_inches((12, 6))

        axes[0].set_xticks(np.arange(-250, 250, 50))
        # axes[0].tick_params(axis='x', labelrotation=45)
        axes[0].set_yticks(np.arange(-100, 400, 50))
        axes[0].set_aspect('equal')
        axes[0].grid(linestyle='-.')
        axes[0].set_xlabel('X axis (m)', labelpad=1, fontsize=10)
        axes[0].set_ylabel('Y axis (m)', labelpad=1, fontsize=10)
        axes[0].axis([-200, 200, -50, 350])
        # axes[0,0].arrow(self.Xo, self.Yo, self.Gx - self.Xo, self.Gy - self.Yo, head_width=2, head_length=2, color='y')
        axes[0].text(self.Xo + 1, self.Yo + 1, 'OS')
        # TODO for all TS
        axes[0].text(self.Xt1 + 1, self.Yt1 + 1, 'TS')

        Xo_traj = self.Xo_list
        Yo_traj = self.Yo_list
        Xt1_traj = self.Xt1_list
        Yt1_traj = self.Yt1_list

        axes[0].plot(Xo_traj, Yo_traj, 'g')  # own ship
        # for x, y in zip(Xo_traj, Yo_traj):  # own ship
        #     axes[0,0].scatter(x, y, c='g', s=1)  # Change to 'scatter' for black dots
        
        # axes[0, 0].plot(Xt1_traj, Yt1_traj, 'r')
        for x, y in zip(Xt1_traj, Yt1_traj):    # target ship
            axes[0].scatter(x, y, c='r', s=1)  # Change to 'scatter' for black dots

        time_x = list(range(1, self.time+1, 1))
        dcpa1_ya = self.DCPA1

        tcpa1_ya = self.TCPA1

        dcpa1_yb = []

        tcpa1_yb = []

        for i in range(len(time_x)-len(dcpa1_ya)):
            dcpa1_yb.append(0)

        dcpa1_y = dcpa1_ya + dcpa1_yb

        for i in range(len(time_x)-len(tcpa1_ya)):
            tcpa1_yb.append(0)

        tcpa1_y = tcpa1_ya + tcpa1_yb

        axes[1].plot(time_x, dcpa1_y, 'r', label="DCPA")
        axes[1].grid(linestyle='-.')
        axes[1].set_xlabel('Time (s)', labelpad=1, fontsize=10)
        axes[1].set_ylabel('DCPA', labelpad=1, fontsize=10)

        axes[1].axis([0, self.time, 0, max(dcpa1_y)]) # 로그 스케일 범위 설정

        # 두 번째 y축 생성 및 데이터 플롯
        ax2 = axes[1].twinx()
        ax2.plot(time_x, tcpa1_y, 'b', label="TCPA")
        ax2.axis([0, self.time, 0, max(tcpa1_y)])
        ax2.set_ylabel('TCPA', labelpad=1, fontsize=10)  # 두 번째 y축 레이블 설정

        lines, labels = axes[1].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1].legend(lines + lines2, labels + labels2, bbox_to_anchor=(0.5, 0.9), loc='center', fontsize='large')

        if name:
            pyplot.savefig(name, dpi = 150)

        plt.tight_layout()
        plt.cla()
        plt.close() 



frm_info = pd.read_csv('./csv/frm.csv')
wp_info = pd.read_csv('./csv/wp.csv')
cri_info = pd.read_csv('./csv/cri.csv')
vo_info = pd.read_csv('./csv/vo.csv')

time = len(cri_info)
dcpa1_list = []
dcpa2_list = []
dcpa3_list = []
tcpa1_list = [] 
tcpa2_list = []
tcpa3_list = []
cri1_list = []
cri2_list = []
cri3_list = []
Heading_list = []
Xo_list = []
Yo_list = []
Xt1_list = []
Yt1_list = []
Xt2_list = []
Yt2_list = []
Xt3_list = []
Yt3_list = []

def char_to_num(str):
    bracket = str.strip("("")")
    comma = bracket.split(',')
    filtered_comma = [num for num in comma if num]  # 빈 문자열 필터링
    result = list(map(float, filtered_comma))

    return result

def char_to_str(str):
    result = re.findall(r'\w+\s?-?\w+', str)

    return result

def flt(input):
    str = re.findall(r'-?\d+.\d+', input)
    result = float(str[0])
    
    return result

data_len = min(len(frm_info), len(wp_info), len(cri_info), len(vo_info))

for i in range(data_len):
    goal_point = re.findall(r'-?\d+.\d+', wp_info.loc[i]['.group_wpts_info'])
    goal_Xo = float(goal_point[1])
    goal_Yo = float(goal_point[3])

    ship_ID = char_to_num(frm_info.loc[i]['.m_nShipID'])
    OS_ship_ID = ship_ID[0]
    TS1_ship_ID = ship_ID[1]

    PosX = char_to_num(frm_info.loc[i]['.m_fltPos_X'])
    PosY = char_to_num(frm_info.loc[i]['.m_fltPos_Y'])

    Xo = PosY[0]
    Yo = PosX[0]

    Xt1 = PosY[1]
    Yt1 = PosX[1]

    Xo_list.append(Xo)
    Yo_list.append(Yo)
    Xt1_list.append(Xt1)
    Yt1_list.append(Yt1)

    heading = char_to_num(frm_info.loc[i]['.m_fltHeading'])
    Co = heading[0]
    Ct1 = heading[1]

    V_opt = char_to_num(vo_info.loc[i]['.V_opt'])
    Vx = V_opt[1]
    Vy = V_opt[0]

    enc_all = char_to_str(cri_info.loc[i]['.encounter_classification'])
    enc1 = enc_all[0]

    dcpa_all = char_to_num(cri_info.loc[i]['.DCPA'])
    dcpa1 = round(dcpa_all[0], 1)

    dcpa1_list.append(dcpa1)

    tcpa_all = char_to_num(cri_info.loc[i]['.TCPA'])
    tcpa1 = abs(round(tcpa_all[0], 1))

    tcpa1_list.append(tcpa1)

    cri = char_to_num(cri_info.loc[i]['.CRI'])
    cri1 = cri[0]

    cri1_list.append(cri1)

    Heading = heading[0] 
    if 0 <= Heading <= 180:
        Heading = Heading
    else:
        Heading = Heading - 360
    Heading_list.append(Heading)


    rf = char_to_num(cri_info.loc[i]['.Rf'])
    rf1 = rf[0]

    ra = char_to_num(cri_info.loc[i]['.Ra'])
    ra1 = ra[0]

    rs = char_to_num(cri_info.loc[i]['.Rs'])
    rs1 = rs[0]

    rp = char_to_num(cri_info.loc[i]['.Rp'])
    rp1 = rp[0]

    collision_cone = char_to_num(vo_info.loc[i]['.Collision_cone'])
    C1x = collision_cone[1]
    C1y = collision_cone[0]
    C2x = collision_cone[3]
    C2y = collision_cone[2]
    C3x = collision_cone[5]
    C3y = collision_cone[4]

    # ship ID, L, B, Xo, Yo, Xt, Yt, Co, Ct, Rf, Ra, Rs, Rp, Vx, Vy, Gx, Gy, C1x, C1y, C2x, C2y, C3x, C3y, enc, cri
    aa = Visualization(OS_ship_ID, 2, 0.6, Xo, Xo_list, Yo, Yo_list, Co, Xt1, Xt1_list, Yt1, Yt1_list,
                       Ct1, rf1, ra1, rs1, rp1, Vx, Vy, goal_Xo, goal_Yo, C1x, C1y, C2x, C2y, C3x, C3y,
                       dcpa1_list, tcpa1_list,  cri1_list, Heading_list, time)
    
    print(round(i/len(frm_info)*100,2),"%")
    Save_image = aa.ploting(name='fig/snap%s.png'%str(i))