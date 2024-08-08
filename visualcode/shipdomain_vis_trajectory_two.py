import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.pyplot as pyplot
import numpy as np
import pandas as pd
import re
from math import *

class Visualization:
    def __init__(self, ship_ID, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list, Xt4, Xt4_list, Yt4, Yt4_list,
    Ct1, Ct2, Rf1, Ra1, Rs1, Rp1, Rf2, Ra2, Rs2, Rp2, Vx, Vy, Gx, Gy, 
    C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y, enc1, enc2, cri1, cri2, Heading, time):
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

        self.Xt2 = Xt2
        self.Xt2_list = Xt2_list
        self.Yt2 = Yt2
        self.Yt2_list = Yt2_list
        
        self.Xt3 = Xt3
        self.Xt3_list = Xt3_list
        self.Yt3 = Yt3
        self.Yt3_list = Yt3_list
        
        self.Xt4 = Xt4
        self.Xt4_list = Xt4_list
        self.Yt4 = Yt4
        self.Yt4_list = Yt4_list

        self.Ct1 = np.deg2rad(Ct1)
        self.Ct2 = np.deg2rad(Ct2)

        self.Rf1 = round(Rf1, 4)
        self.Ra1 = round(Ra1, 4)
        self.Rs1 = round(Rs1, 4)
        self.Rp1 = round(Rp1, 4)

        self.Rf2 = round(Rf2, 4)
        self.Ra2 = round(Ra2, 4)
        self.Rs2 = round(Rs2, 4)
        self.Rp2 = round(Rp2, 4)

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
        self.C4x = C4x
        self.C4y = C4y
        self.C5x = C5x
        self.C5y = C5y
        self.C6x = C6x
        self.C6y = C6y

        self.enc1 = enc1
        self.enc2 = enc2
        self.cri1 = cri1
        self.cri2 = cri2
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
        OS_boundary1 = self.transform(0, 1, Co, Xo, Yo)
        OS_boundary2 = self.transform(0.6, -1, Co, Xo, Yo)
        OS_boundary3 = self.transform(-0.6, -1, Co, Xo, Yo)

        return OS_boundary1, OS_boundary2, OS_boundary3

    def create_coord_TS(self, Xt, Yt, Ct):
        TS_boundary1 = self.transform(0, 1, Ct, Xt, Yt)
        TS_boundary2 = self.transform(0.6, -1, Ct, Xt, Yt)
        TS_boundary3 = self.transform(-0.6, -1, Ct, Xt, Yt)

        return TS_boundary1, TS_boundary2, TS_boundary3

    def ploting(self, name = None):
        # colliscion cone
        cc1 = [[self.C1x, self.C2x], [self.C1x, self.C3x]]
        cc2 = [[self.C1y, self.C2y], [self.C1y, self.C3y]]
        cc3 = [[self.C4x, self.C5x], [self.C4x, self.C6x]]
        cc4 = [[self.C4y, self.C5y], [self.C4y, self.C6y]]

        OS_boundary1, OS_boundary2, OS_boundary3 = self.create_coord_OS(self.Xo, self.Yo, self.Co)
        Q1, Q2, Q3, Q4 = self.create_ship_domain(self.Rf1, self.Ra1, self.Rs1, self.Rp1)
        Q5, Q6, Q7, Q8 = self.create_ship_domain(self.Rf2, self.Ra2, self.Rs2, self.Rp2)

        # TODO for all TS
        TS_boundary1, TS_boundary2, TS_boundary3 = self.create_coord_TS(self.Xt1, self.Yt1, self.Ct1)
        TS_boundary4, TS_boundary5, TS_boundary6 = self.create_coord_TS(self.Xt2, self.Yt2, self.Ct2)

        f,axes = plt.subplots(1, 2, figsize=(12,6))
        # f.set_size_inches((12, 12))

        axes[0].set_xticks(np.arange(-300, 300, 50))
        axes[0].tick_params(axis='x', labelrotation=45)
        axes[0].set_yticks(np.arange(-100, 500, 50))
        axes[0].set_aspect('equal')
        axes[0].grid(linestyle='-.')
        axes[0].set_xlabel('X axis (m)', labelpad=15)
        axes[0].set_ylabel('Y axis (m)', labelpad=15)
        axes[0].axis([-300, 300, -100, 500,])
        # axes[0,0].arrow(self.Xo, self.Yo, self.Gx - self.Xo, self.Gy - self.Yo, head_width=2, head_length=2, color='y')
        axes[0].text(self.Xo + 1, self.Yo + 1, 'OS')
        # TODO for all TS
        axes[0].text(self.Xt1 + 1, self.Yt1 + 1, 'TS1')
        axes[0].text(self.Xt2 + 1, self.Yt2 + 1, 'TS2')

        Xo_traj = self.Xo_list
        Yo_traj = self.Yo_list
        Xt1_traj = self.Xt1_list
        Yt1_traj = self.Yt1_list
        Xt2_traj = self.Xt2_list
        Yt2_traj = self.Yt2_list
        Xt3_traj = self.Xt3_list
        Yt3_traj = self.Yt3_list
        Xt4_traj = self.Xt4_list
        Yt4_traj = self.Yt4_list
        
        axes[0].plot(Xo_traj, Yo_traj, 'g')
        axes[0].plot(Xt1_traj, Yt1_traj, 'r')
        axes[0].plot(Xt2_traj, Yt2_traj, 'b')
        # for x, y in zip(Xt3_traj, Yt3_traj):
        #     axes[0].scatter(y, x, c='k', s=0.5)  # Change to 'scatter' for black dots
        # for x, y in zip(Xt4_traj, Yt4_traj):
        #     axes[0].scatter(y, x, c='k', s=0.5)  # Change to 'scatter' for black dots




        # axes[0,1].set_xticks(np.arange(-350, 350, 5))
        # axes[0,1].set_yticks(np.arange(-350, 350, 5))
        # axes[0,1].set_aspect('equal')
        # axes[0,1].grid(linestyle='-.')
        # axes[0,1].set_xlabel('X axis (m)', labelpad=15)
        # axes[0,1].set_ylabel('Y axis (m)', labelpad=15)
        # # TODO auto axis
        # axes[0,1].axis([self.Xo-20, self.Xo+20, self.Yo-20, self.Yo+20])

        # ##full=+600

        # axes[0,1].add_patch(
        #     patches.Polygon(
        #         (OS_boundary1, OS_boundary2, OS_boundary3),
        #         closed=True,
        #         edgecolor='black',
        #         facecolor='Green'
        #     )
        # )

        # # TODO for all TS
        # axes[0,1].add_patch(
        #     patches.Polygon(
        #         (TS_boundary1, TS_boundary2, TS_boundary3),
        #         closed=True,
        #         edgecolor='black',
        #         facecolor='Red'
        #     )
        # )

        # axes[0,1].add_patch(
        #     patches.Polygon(
        #         (TS_boundary4, TS_boundary5, TS_boundary6),
        #         closed=True,
        #         edgecolor='black',
        #         facecolor='Blue'
        #     )
        # )

        # # TODO for all TS
        # # axes[0,1].plot(Q1[0], Q1[1], c='r', label = 'Ship domain for TS1')
        # axes[0,1].plot(Q1[0], Q1[1], c='r')
        # axes[0,1].plot(Q2[0], Q2[1], c='r')
        # axes[0,1].plot(Q3[0], Q3[1], c='r')
        # axes[0,1].plot(Q4[0], Q4[1], c='r')
        # axes[0,1].plot(Q5[0], Q5[1], c='b')
        # axes[0,1].plot(Q6[0], Q6[1], c='b')
        # axes[0,1].plot(Q7[0], Q7[1], c='b')
        # axes[0,1].plot(Q8[0], Q8[1], c='b')

        # # TODO for all TS
        # for i in range(len(cc1)):
        #     axes[0,1].plot(cc1[i], cc2[i], color='k')
        # for i in range(len(cc3)):
        #     axes[0,1].plot(cc3[i], cc4[i], color='k')

        # axes[0,1].fill([self.C1x, self.C2x, self.C3x], [self.C1y, self.C2y, self.C3y], color='r', alpha=0.5)
        # axes[0,1].fill([self.C4x, self.C5x, self.C6x], [self.C4y, self.C5y, self.C6y], color='b', alpha=0.5)

        # # axes[0,1].arrow(self.Xo, self.Yo, self.Vx, self.Vy, head_width=1, head_length=1, color='r')






        # axes[1,0].axis([-10, 10, -10, 10])
        # axes[1,0].set_aspect('equal')
        # axes[1,0].set_xlabel('Encounter', labelpad=15)
        # axes[1,0].add_patch(
        #     patches.Polygon(
        #         ([0,1], [-0.6, -1], [0.6,-1]),
        #         closed=True,
        #         edgecolor='black',
        #         facecolor='Green'
        #     )
        # )

        # # TODO just one ship among all ship
        # axes[1,0].arrow(0, 0, 10, tan(np.deg2rad(67.5))*10, color='k')
        # axes[1,0].arrow(0, 0, -10, tan(np.deg2rad(67.5))*10, color='k')
        # axes[1,0].arrow(0, 0, 10, -tan(np.deg2rad(22.5))*10, color='k')
        # axes[1,0].arrow(0, 0, -10, -tan(np.deg2rad(22.5))*10, color='k')

        # if self.enc1 == 'Head-on' or self.enc1 == 'Overtaking':
        #         axes[1,0].fill([0, -10, 10], [0, tan(np.deg2rad(67.5))*10, tan(np.deg2rad(67.5))*10], color='r', alpha=0.7)
        # if self.enc1 == 'Port crossing':
        #         axes[1,0].fill([0, -10, -10], [0, tan(np.deg2rad(67.5))*10, -tan(np.deg2rad(22.5))*10], color='r', alpha=0.7)
        # if self.enc1 == 'Starboard crossing':
        #         axes[1,0].fill([0, 10, 10], [0, tan(np.deg2rad(67.5))*10, -tan(np.deg2rad(22.5))*10], color='r', alpha=0.7)
        # if self.enc1 == 'Overtaken':
        #         axes[1,0].fill([0, -10, 10], [0, -tan(np.deg2rad(22.5))*10, -tan(np.deg2rad(22.5))*10], color='r', alpha=0.7)
        #         axes[1,0].fill([-10, 10, 10, -10], [-tan(np.deg2rad(22.5))*10, -tan(np.deg2rad(22.5))*10, -10, -10], color='r', alpha=0.7)

        # if self.enc2 == 'Head-on' or self.enc2 == 'Overtaking':
        #         axes[1,0].fill([0, -10, 10], [0, tan(np.deg2rad(67.5))*10, tan(np.deg2rad(67.5))*10], color='b', alpha=0.7)
        # if self.enc2 == 'Port crossing':
        #         axes[1,0].fill([0, -10, -10], [0, tan(np.deg2rad(67.5))*10, -tan(np.deg2rad(22.5))*10], color='b', alpha=0.7)
        # if self.enc2 == 'Starboard crossing':
        #         axes[1,0].fill([0, 10, 10], [0, tan(np.deg2rad(67.5))*10, -tan(np.deg2rad(22.5))*10], color='b', alpha=0.7)
        # if self.enc2 == 'Overtaken':
        #         axes[1,0].fill([0, -10, 10], [0, -tan(np.deg2rad(22.5))*10, -tan(np.deg2rad(22.5))*10], color='b', alpha=0.7)
        #         axes[1,0].fill([-10, 10, 10, -10], [-tan(np.deg2rad(22.5))*10, -tan(np.deg2rad(22.5))*10, -10, -10], color='b', alpha=0.7)




        # # TODO for all ship
        time_x = list(range(1, self.time+1, 1))
        cri1_ya = self.cri1
        cri2_ya = self.cri2
        cri1_yb = []
        cri2_yb = []
        Heading_ya = self.Heading
        Heading_yb = []

        for i in range(len(time_x)-len(cri1_ya)):
            cri1_yb.append(0)
        for i in range(len(time_x)-len(cri2_ya)):
            cri2_yb.append(0)

        cri1_y = cri1_ya + cri1_yb
        cri2_y = cri2_ya + cri2_yb
        

        for i in range(len(time_x)-len(Heading_ya)):
             Heading_yb.append(0)
        Heading_y = Heading_ya + Heading_yb



        axes[1].plot(time_x, cri1_y, 'r--', label='CRI1')
        axes[1].fill_between(time_x[0:], cri1_y[0:], color='r', alpha=0.1)
        axes[1].plot(time_x, cri2_y, 'b--', label='CRI2')
        axes[1].fill_between(time_x[0:], cri2_y[0:], color='b', alpha=0.1)

        # axes[1,1].axhline(y = 0.66, xmin = 0, xmax = 1, color='r')
        axes[1].set_xlabel('Time (s)', labelpad=15)
        axes[1].set_ylabel('CRI', labelpad=15)
        axes[1].axis([0, self.time, 0, 1])

        

        ax2 = axes[1].twinx()
        ax2.plot(time_x, Heading_y, 'g-', label='Heading')
        ax2.fill_between(time_x[0:], Heading_y[0:], color='g', alpha=0.05)
        ax2.set_ylim(-180, 180)
        ax2.set_ylabel('Heading', labelpad=15)
        ax2.yaxis.tick_right()

        lines, labels = axes[1].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1].legend(lines + lines2, labels + labels2, loc='best')
        # axes[1].legend(loc='best')





        if name:
            pyplot.savefig(name, dpi = 600)

        plt.tight_layout()
        plt.cla()
        plt.close() 



frm_info = pd.read_csv('./csv/frm.csv')
wp_info = pd.read_csv('./csv/wp.csv')
cri_info = pd.read_csv('./csv/cri.csv')
vo_info = pd.read_csv('./csv/vo.csv')
ais_info = pd.read_csv('./csv/ais.csv')

time = len(cri_info)
cri1_list = []
cri2_list = []
Heading_list = []
Xo_list = []
Yo_list = []
Xt1_list = []
Yt1_list = []
Xt2_list = []
Yt2_list = []
Xt3_list = []
Yt3_list = []
Xt4_list = []
Yt4_list = []

def char_to_num(str):
    bracket = str.strip("("")")
    comma = bracket.split(',')
    result = list(map(float, comma))

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
    # goal_Xt1 = float(goal_point[3])
    # goal_Yt1 = float(goal_point[3])
    # goal_Xt2 = float(goal_point[3])
    # goal_Yt2 = float(goal_point[3])

    ship_ID = char_to_num(frm_info.loc[i]['.m_nShipID'])
    OS_ship_ID = ship_ID[0]
    TS1_ship_ID = ship_ID[1]
    TS2_ship_ID = ship_ID[2]

    PosX = char_to_num(frm_info.loc[i]['.m_fltPos_X'])
    PosY = char_to_num(frm_info.loc[i]['.m_fltPos_Y'])
    
    aisX = char_to_num(ais_info.loc[i]['.m_fltPos_X'])
    aisY = char_to_num(ais_info.loc[i]['.m_fltPos_Y'])

    Xo = PosY[0]
    Yo = PosX[0]

    Xt1 = PosY[1]
    Yt1 = PosX[1]

    Xt2 = PosY[2]
    Yt2 = PosX[2]
    
    Xt3 = aisX[1]
    Yt3 = aisY[1]

    Xt4 = aisX[2]
    Yt4 = aisY[2]

    Xo_list.append(Xo)
    Yo_list.append(Yo)
    Xt1_list.append(Xt1)
    Yt1_list.append(Yt1)
    Xt2_list.append(Xt2)
    Yt2_list.append(Yt2)
    Xt3_list.append(Xt3)
    Yt3_list.append(Yt3)
    Xt4_list.append(Xt4)
    Yt4_list.append(Yt4)

    heading = char_to_num(frm_info.loc[i]['.m_fltHeading'])
    Co = heading[0]
    Ct1 = heading[1]
    Ct2 = heading[2]

    V_opt = char_to_num(vo_info.loc[i]['.V_opt'])
    Vx = V_opt[1]
    Vy = V_opt[0]

    enc_all = char_to_str(cri_info.loc[i]['.encounter_classification'])
    enc1 = enc_all[0]
    enc2 = enc_all[1]

    cri = char_to_num(cri_info.loc[i]['.CRI'])
    cri1 = cri[0]
    cri2 = cri[1]
    cri1_list.append(cri1)
    cri2_list.append(cri2)

    Heading = heading[0]

    
    if 0 < Heading <= 180:
            Heading = Heading
    else:
         Heading = Heading - 360

    Heading_list.append(Heading)


    rf = char_to_num(cri_info.loc[i]['.Rf'])
    rf1 = rf[0]
    rf2 = rf[1]
    ra = char_to_num(cri_info.loc[i]['.Ra'])
    ra1 = ra[0]
    ra2 = ra[1]
    rs = char_to_num(cri_info.loc[i]['.Rs'])
    rs1 = rs[0]
    rs2 = rs[1]
    rp = char_to_num(cri_info.loc[i]['.Rp'])
    rp1 = rp[0]
    rp2 = rp[1]

    collision_cone = char_to_num(vo_info.loc[i]['.Collision_cone'])
    C1x = collision_cone[1]
    C1y = collision_cone[0]
    C2x = collision_cone[3]
    C2y = collision_cone[2]
    C3x = collision_cone[5]
    C3y = collision_cone[4]
    C4x = collision_cone[7]
    C4y = collision_cone[6]
    C5x = collision_cone[9]
    C5y = collision_cone[8]
    C6x = collision_cone[11]
    C6y = collision_cone[10]

    # ship ID, L, B, Xo, Yo, Xt, Yt, Co, Ct, Rf, Ra, Rs, Rp, Vx, Vy, Gx, Gy, C1x, C1y, C2x, C2y, C3x, C3y, enc, cri
    aa = Visualization(OS_ship_ID, 2, 0.6, Xo, Xo_list, Yo, Yo_list, Co, Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list, Xt4, Xt4_list, Yt4, Yt4_list,
    Ct1, Ct2, rf1, ra1, rs1, rp1, rf2, ra2, rs2, rp2, Vx, Vy, goal_Xo, goal_Yo, C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y,
    enc1, enc2, cri1_list, cri2_list, Heading_list, time)

    print(i)
    Save_image = aa.ploting(name='fig/snap%s.png'%str(i))