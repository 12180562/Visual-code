import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.pyplot as pyplot
import numpy as np
import pandas as pd
import re
import gc
from math import *

class Visualization:
    def __init__(self, ship_ID, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt1, Xt1_list, Yt1, Yt1_list, 
    Ct1, Rf1, Ra1, Rs1, Rp1, enc1, time):
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

        self.enc1 = enc1

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
        OS_boundary1 = self.transform(0, 1.2, Co, Xo, Yo)
        OS_boundary2 = self.transform(0.3, -1.2, Co, Xo, Yo)
        OS_boundary3 = self.transform(-0.3, -1.2, Co, Xo, Yo)

        return OS_boundary1, OS_boundary2, OS_boundary3

    def create_coord_TS(self, Xt, Yt, Ct):
        TS_boundary1 = self.transform(0, 1.2, Ct, Xt, Yt)
        TS_boundary2 = self.transform(0.3, -1.2, Ct, Xt, Yt)
        TS_boundary3 = self.transform(-0.3, -1.2, Ct, Xt, Yt)

        return TS_boundary1, TS_boundary2, TS_boundary3

    def ploting(self, name = None):
        # colliscion cone

        OS_boundary1, OS_boundary2, OS_boundary3 = self.create_coord_OS(self.Xo, self.Yo, self.Co)
        Q1, Q2, Q3, Q4 = self.create_ship_domain(self.Rf1, self.Ra1, self.Rs1, self.Rp1)

        # TODO for all TS
        TS_boundary1, TS_boundary2, TS_boundary3 = self.create_coord_TS(self.Xt1, self.Yt1, self.Ct1)

        f,axes = plt.subplots(1, 2, figsize=(12,6))
        # f.set_size_inches((12, 12))

        axes[0].set_xticks(np.arange(100, 300, 25))
        axes[0].tick_params(axis='x', labelrotation=45)
        axes[0].set_yticks(np.arange(-50, 350, 25))
        axes[0].set_aspect('equal')
        axes[0].grid(linestyle='-.')
        axes[0].set_xlabel('X axis (m)', labelpad=15)
        axes[0].set_ylabel('Y axis (m)', labelpad=15)
        axes[0].axis([100, 300, -50, 350,])
        # axes[0,0].arrow(self.Xo, self.Yo, self.Gx - self.Xo, self.Gy - self.Yo, head_width=2, head_length=2, color='y')
        axes[0].text(self.Xo + 1, self.Yo + 1, 'OS')
        # TODO for all TS
        axes[0].text(self.Xt1 + 1, self.Yt1 + 1, 'TS1')

        Xo_traj = self.Xo_list
        Yo_traj = self.Yo_list
        Xt1_traj = self.Xt1_list
        Yt1_traj = self.Yt1_list

        axes[0].plot(Xo_traj, Yo_traj, 'g')
        axes[0].plot(Xt1_traj, Yt1_traj, 'r')

        #TODO for all ship
        axes[1].set_xticks(np.arange(-350, 350, 10))
        axes[1].set_yticks(np.arange(-350, 350, 10))
        axes[1].set_aspect('equal')
        axes[1].grid(linestyle='-.')
        axes[1].set_xlabel('X axis (m)', labelpad=15)
        axes[1].set_ylabel('Y axis (m)', labelpad=15)
        # TODO auto axis
        axes[1].axis([self.Xo-50, self.Xo+50, self.Yo-50, self.Yo+50])

        axes[1].add_patch(
            patches.Polygon(
                (OS_boundary1, OS_boundary2, OS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Green'
            )
        )

        # TODO for all TS
        axes[1].add_patch(
            patches.Polygon(
                (TS_boundary1, TS_boundary2, TS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Red'
            )
        )

        # TODO for all TS
        # axe[0,1].plot(Q1[0], Q1[1], c='r', label = 'Ship domain for TS1')
        axes[1].plot(Q1[0], Q1[1], c='r')
        axes[1].plot(Q2[0], Q2[1], c='r')
        axes[1].plot(Q3[0], Q3[1], c='r')
        axes[1].plot(Q4[0], Q4[1], c='r')

        if name:
            pyplot.savefig(name, dpi = 300)

        plt.tight_layout()
        plt.cla()
        plt.close() 
        gc.collect()

data = pd.read_csv('C:/Users/wonjun/Desktop/Generate_data/YOO/inha_scenario(crossing_wpchange)_2024-07-31-12-25-40-frm_info_final.csv')

time = len(data)
cri1_list = []
Heading_list = []
Xo_list = []
Yo_list = []
Xt1_list = []
Yt1_list = []

def char_to_num(str):
    numbers = re.findall(r'-?\d+\.\d+|-?\d+', str)

    # 각 숫자를 float로 변환하여 리스트 반환
    result = list(map(float, numbers))
    return result

def char_to_str(str):
    result = re.findall(r'\w+\s?-?\w+', str)

    return result

def flt(input):
    str = re.findall(r'-?\d+.\d+', input)
    result = float(str[0])
    
    return result

data_len = len(data)

for i in range(1,1805):
    ship_ID = char_to_num(data.loc[i]['.m_nShipID'])
    OS_ship_ID = ship_ID[0]
    TS1_ship_ID = ship_ID[1]

    PosX = char_to_num(data.loc[i]['.m_fltPos_X'])
    PosY = char_to_num(data.loc[i]['.m_fltPos_Y'])

    Xo = PosY[0]
    Yo = PosX[0]

    Xt1 = PosY[2]
    Yt1 = PosX[2]
    
    Xo_list.append(Xo)
    Yo_list.append(Yo)
    Xt1_list.append(Xt1)
    Yt1_list.append(Yt1)

    heading = char_to_num(data.loc[i]['.m_fltHeading'])
    Co = heading[0]
    Ct1 = heading[1]

    enc_all = char_to_str(data.loc[i]['.encounter_classification'])
    enc1 = enc_all[0] 

    Heading = heading[0] 
    if 0 <= Heading <= 180:
         Heading = Heading
    else:
         Heading = Heading - 360
    Heading_list.append(Heading)

    rf = flt(data.loc[i]['.Rf'])
    rf1 = rf
    ra = flt(data.loc[i]['.Ra'])
    ra1 = ra
    rs = flt(data.loc[i]['.Rs'])
    rs1 = rs
    rp = flt(data.loc[i]['.Rp'])
    rp1 = rp

aa = Visualization(OS_ship_ID, 2.5, 0.6, Xo, Xo_list, Yo, Yo_list, Co, Xt1, Xt1_list, Yt1, Yt1_list,
Ct1, rf1, ra1, rs1, rp1, enc1, time)
print(round(i/len(data)*100,2),"%")
Save_image = aa.ploting(name='C:/Users/wonjun/Desktop/Generate_data/YOO/fig/snap%s.png'%str(i))