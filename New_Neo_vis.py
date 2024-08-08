import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd
import re
from math import *
import glob

class Visualization:
    def __init__(self, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list,
    Ct1, Ct2, Ct3, Rf1, Ra1, Rs1, Rp1, Rf2, Ra2, Rs2, Rp2, Rf3, Ra3, Rs3, Rp3, Vx, Vy,
    C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y, C7x, C7y, C8x, C8y, C9x, C9y, 
    DCPA1, DCPA2, DCPA3, TCPA1, TCPA2, TCPA3, cri1, cri2, cri3, Heading, time):
        
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

        self.Ct1 = np.deg2rad(Ct1)
        self.Ct2 = np.deg2rad(Ct2)
        self.Ct3 = np.deg2rad(Ct3)

        self.Rf1 = round(Rf1, 4)
        self.Ra1 = round(Ra1, 4)
        self.Rs1 = round(Rs1, 4)
        self.Rp1 = round(Rp1, 4)

        self.Rf2 = round(Rf2, 4)
        self.Ra2 = round(Ra2, 4)
        self.Rs2 = round(Rs2, 4)
        self.Rp2 = round(Rp2, 4)

        self.Rf3 = round(Rf3, 4)
        self.Ra3 = round(Ra3, 4)
        self.Rs3 = round(Rs3, 4)
        self.Rp3 = round(Rp3, 4)

        self.Vx = Vx *3
        self.Vy = Vy *3

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
        self.C7x = C7x
        self.C7y = C7y
        self.C8x = C8x
        self.C8y = C8y 
        self.C9x = C9x
        self.C9y = C9y

        self.DCPA1 = DCPA1
        self.DCPA2 = DCPA2
        self.DCPA3 = DCPA3
        self.TCPA1 = TCPA1
        self.TCPA2 = TCPA2
        self.TCPA3 = TCPA3

        self.cri1 = cri1
        self.cri2 = cri2
        self.cri3 = cri3
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

    def create_coord_ship(self, X, Y, C):
        ship_boundary1 = self.transform(0, self.L/2, C, X, Y)
        ship_boundary2 = self.transform(self.B/2, self.L/10, C, X, Y)
        ship_boundary3 = self.transform(self.B/2, -self.L/2, C, X, Y)
        ship_boundary4 = self.transform(-self.B/2, -self.L/2, C, X, Y)
        ship_boundary5 = self.transform(-self.B/2, self.L/10, C, X, Y)
        return ship_boundary1, ship_boundary2, ship_boundary3, ship_boundary4, ship_boundary5

    def highlight_points(self, axes, traj_x, traj_y, color):        
        highlight_interval = 100
        for i in range(0, len(traj_x), highlight_interval):
            axes[0, 0].scatter(traj_x[i], traj_y[i], c=color, s=10)

    def ploting(self, name = None):
        f,axes = plt.subplots(2, 2)
        f.set_size_inches((12, 12))

###################### 2사분면 그림 ######################

        axes[0,0].set_xticks(np.arange(-250, 250, 50))
        axes[0,0].tick_params(axis='x', labelrotation=45)
        axes[0,0].set_yticks(np.arange(-100, 400, 50))
        axes[0,0].set_aspect('equal')
        axes[0,0].grid(linestyle='-.')
        axes[0,0].set_xlabel('X axis (m)', labelpad=1, fontsize=10)
        axes[0,0].set_ylabel('Y axis (m)', labelpad=1, fontsize=10)
        axes[0,0].axis([-200, 200, -50, 350])
        
        axes[0,0].arrow(self.Xo, self.Yo, self.Vx, self.Vy, head_width=5, head_length=20, color='k')

        axes[0,0].text(self.Xo + 1, self.Yo + 1, 'OS')
        axes[0,0].text(self.Xt1 + 1, self.Yt1 + 1, 'TS1')
        axes[0,0].text(self.Xt2 + 1, self.Yt2 + 1, 'TS2')
        axes[0,0].text(self.Xt3 + 1, self.Yt3 + 1, 'TS3')

        Xo_traj = self.Xo_list
        Yo_traj = self.Yo_list
        Xt1_traj = self.Xt1_list
        Yt1_traj = self.Yt1_list
        Xt2_traj = self.Xt2_list
        Yt2_traj = self.Yt2_list
        Xt3_traj = self.Xt3_list
        Yt3_traj = self.Yt3_list

        axes[0, 0].plot(Xo_traj, Yo_traj, 'g', label='OS')
        
        for x, y in zip(Xt1_traj, Yt1_traj):
            axes[0, 0].scatter(x, y, c='r', s=1)
        for x, y in zip(Xt2_traj, Yt2_traj):
            axes[0, 0].scatter(x, y, c='b', s=1)
        for x, y in zip(Xt3_traj, Yt3_traj):
            axes[0, 0].scatter(x, y, c='y', s=1)

        self.highlight_points(axes, Xo_traj, Yo_traj, 'g')
        self.highlight_points(axes, Xt1_traj, Yt1_traj, 'r')
        self.highlight_points(axes, Xt2_traj, Yt2_traj, 'b')
        self.highlight_points(axes, Xt3_traj, Yt3_traj, 'y')
        
        axes[0, 0].plot([], [], 'r-', label='TS1')
        axes[0, 0].plot([], [], 'b-', label='TS2')
        axes[0, 0].plot([], [], 'y-', label='TS3')
        
        axes[0, 0].legend(loc='best', fontsize='large')

###################### 1사분면 그림 ######################

        # modify
        width = 40
        height = 40

        cc1 = [[self.C1x, self.C2x], [self.C1x, self.C3x]]
        cc2 = [[self.C1y, self.C2y], [self.C1y, self.C3y]]
        cc3 = [[self.C4x, self.C5x], [self.C4x, self.C6x]]
        cc4 = [[self.C4y, self.C5y], [self.C4y, self.C6y]]
        cc5 = [[self.C7x, self.C8x], [self.C7x, self.C9x]]
        cc6 = [[self.C7y, self.C8y], [self.C7y, self.C9y]]

        OS_boundary1, OS_boundary2, OS_boundary3, OS_boundary4, OS_boundary5 = self.create_coord_ship(self.Xo, self.Yo, self.Co)
        Q1, Q2, Q3, Q4 = self.create_ship_domain(self.Rf1, self.Ra1, self.Rs1, self.Rp1)
        Q5, Q6, Q7, Q8 = self.create_ship_domain(self.Rf2, self.Ra2, self.Rs2, self.Rp2)
        Q9, Q10, Q11, Q12 = self.create_ship_domain(self.Rf3, self.Ra3, self.Rs3, self.Rp3)

        TS_boundary1, TS_boundary2, TS_boundary3, TS_boundary4, TS_boundary5 = self.create_coord_ship(self.Xt1, self.Yt1, self.Ct1)
        TS_boundary6, TS_boundary7, TS_boundary8, TS_boundary9, TS_boundary10 = self.create_coord_ship(self.Xt2, self.Yt2, self.Ct2)
        TS_boundary11, TS_boundary12, TS_boundary13, TS_boundary14, TS_boundary15 = self.create_coord_ship(self.Xt3, self.Yt3, self.Ct3)
        
        axes[0,1].set_xticks(np.arange(-600, 600, 5))
        axes[0,1].tick_params(axis='x', labelrotation=45)
        axes[0,1].set_yticks(np.arange(-600, 600, 5))
        axes[0,1].set_aspect('equal')
        axes[0,1].grid(linestyle='-.')
        axes[0,1].set_xlabel('X axis (m)', labelpad=1, fontsize=10)
        axes[0,1].set_ylabel('Y axis (m)', labelpad=1, fontsize=10)
        axes[0, 1].axis([self.Xo - width / 2, self.Xo + width / 2, self.Yo - (height*(1/5)), self.Yo + (height*(4/5))])
        
        axes[0,1].add_patch(
            patches.Polygon(
                (OS_boundary1, OS_boundary2, OS_boundary3, OS_boundary4, OS_boundary5),
                closed=True,
                edgecolor='black',
                facecolor='Green'
            )
        )

        axes[0,1].add_patch(
            patches.Polygon(
                (TS_boundary1, TS_boundary2, TS_boundary3, TS_boundary4, TS_boundary5),
                closed=True,
                edgecolor='black',
                facecolor='Red'
            )
        )

        axes[0,1].add_patch(
            patches.Polygon(
                (TS_boundary6, TS_boundary7, TS_boundary8, TS_boundary9, TS_boundary10),
                closed=True,
                edgecolor='black',
                facecolor='Blue'
            )
        )

        axes[0,1].add_patch(
            patches.Polygon(
                (TS_boundary11, TS_boundary12, TS_boundary13, TS_boundary14, TS_boundary15),
                closed=True,
                edgecolor='black',
                facecolor='y'
            )
        )
        
        axes[0,1].plot(Q1[0], Q1[1], c='r')
        axes[0,1].plot(Q2[0], Q2[1], c='r')
        axes[0,1].plot(Q3[0], Q3[1], c='r')
        axes[0,1].plot(Q4[0], Q4[1], c='r')
        axes[0,1].plot(Q5[0], Q5[1], c='b')
        axes[0,1].plot(Q6[0], Q6[1], c='b')
        axes[0,1].plot(Q7[0], Q7[1], c='b')
        axes[0,1].plot(Q8[0], Q8[1], c='b')
        axes[0,1].plot(Q9[0], Q9[1], c='y')
        axes[0,1].plot(Q10[0], Q10[1], c='y')
        axes[0,1].plot(Q11[0], Q11[1], c='y')
        axes[0,1].plot(Q12[0], Q12[1], c='y')

        for cc in [cc1, cc2, cc3, cc4, cc5, cc6]:
            for i in range(len(cc[0])):
                axes[0, 1].plot(cc[0][i], cc[1][i], color='k')

        axes[0,1].fill([self.C1x, self.C2x, self.C3x], [self.C1y, self.C2y, self.C3y], color='r', alpha=0.7)
        axes[0,1].fill([self.C4x, self.C5x, self.C6x], [self.C4y, self.C5y, self.C6y], color='b', alpha=0.5)
        axes[0,1].fill([self.C7x, self.C8x, self.C9x], [self.C7y, self.C8y, self.C9y], color='y', alpha=0.5)


        axes[0,1].arrow(self.Xo, self.Yo, self.Vx, self.Vy, head_width=1, head_length=1, color='y')


###################### 3사분면 그림 ######################        

        time_x = list(range(1, self.time+1, 1))
        dcpa1_ya, dcpa2_ya, dcpa3_ya = self.DCPA1, self.DCPA2, self.DCPA3
        tcpa1_ya, tcpa2_ya, tcpa3_ya = self.TCPA1, self.TCPA2, self.TCPA3

        dcpa1_y = np.pad(dcpa1_ya, (0, len(time_x) - len(dcpa1_ya)), 'constant')
        dcpa2_y = np.pad(dcpa2_ya, (0, len(time_x) - len(dcpa2_ya)), 'constant')
        dcpa3_y = np.pad(dcpa3_ya, (0, len(time_x) - len(dcpa3_ya)), 'constant')
        tcpa1_y = np.pad(tcpa1_ya, (0, len(time_x) - len(tcpa1_ya)), 'constant')
        tcpa2_y = np.pad(tcpa2_ya, (0, len(time_x) - len(tcpa2_ya)), 'constant')
        tcpa3_y = np.pad(tcpa3_ya, (0, len(time_x) - len(tcpa3_ya)), 'constant')

        axes[1,0].plot(time_x, dcpa1_y, 'r', label="DCPA1")
        axes[1,0].plot(time_x, dcpa2_y, 'b', label="DCPA2")
        axes[1,0].plot(time_x, dcpa3_y, 'y', label="DCPA3")
        # axes[1,0].grid(linestyle='-.')
        axes[1,0].set_xlabel('Time (s)', labelpad=1, fontsize=10)
        axes[1,0].set_ylabel('DCPA', labelpad=1, fontsize=10)
        axes[1,0].axis([0, self.time, 0, max(max(dcpa1_y), max(dcpa2_y), max(dcpa3_y))])

        ax2 = axes[1,0].twinx()
        ax2.plot(time_x, tcpa1_y, 'r--', label="TCPA1")
        ax2.plot(time_x, tcpa2_y, 'b--', label="TCPA2")
        ax2.plot(time_x, tcpa3_y, 'y--', label="TCPA3")
        ax2.axis([0, self.time, min(min(tcpa1_y), min(tcpa2_y), min(tcpa3_y)), max(max(tcpa1_y), max(tcpa2_y), max(tcpa3_y))])

        lines, labels = axes[1,0].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,0].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')

###################### 4사분면 그림 ######################  

        cri1_ya, cri2_ya, cri3_ya = self.cri1, self.cri2, self.cri3
        Heading_ya = self.Heading

        cri1_y = np.pad(cri1_ya, (0, len(time_x) - len(cri1_ya)), 'constant')
        cri2_y = np.pad(cri2_ya, (0, len(time_x) - len(cri2_ya)), 'constant')
        cri3_y = np.pad(cri3_ya, (0, len(time_x) - len(cri3_ya)), 'constant')
        Heading_y = np.pad(Heading_ya, (0, len(time_x) - len(Heading_ya)), 'constant')

        axes[1,1].plot(time_x, cri1_y, 'r--', label='CRI1')
        axes[1,1].fill_between(time_x, cri1_y, color='r', alpha=0)
        axes[1,1].plot(time_x, cri2_y, 'b--', label='CRI2')
        axes[1,1].fill_between(time_x, cri2_y, color='b', alpha=0)
        axes[1,1].plot(time_x, cri3_y, 'y--', label='CRI3')
        axes[1,1].fill_between(time_x, cri3_y, color='y', alpha=0)

        axes[1,1].set_xlabel('Time (s)', labelpad=1, fontsize=10)
        axes[1,1].set_ylabel('CRI', labelpad=1, fontsize=10)
        axes[1,1].axis([0, self.time, 0, 1])

        ax2 = axes[1,1].twinx()
        ax2.plot(time_x, Heading_y, 'g-', label='Heading', )
        ax2.fill_between(time_x, Heading_y, color='g', alpha=0)
        ax2.set_ylim(-180, 180)
        ax2.set_ylabel('Heading(deg)', labelpad=1, fontsize=10, rotation=270)
        ax2.yaxis.tick_right()

        lines, labels = axes[1,1].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,1].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')

        if name:
            plt.savefig(name, dpi=150)
        plt.tight_layout()
        plt.cla()
        plt.close()

# modify
ship_L = 163.55
ship_B = 27.4
scale = 70

directory_path = './csv'

file_patterns = {
    'frm': 'frm',
    'wp': 'waypoint',
    'cri': 'cri',
    'vo': 'VO'
}

frm_info = None
wp_info = None
cri_info = None
vo_info = None

for key, filename in file_patterns.items():
    file_path = f'{directory_path}/*{filename}*.csv'
    matching_files = glob.glob(file_path)
    
    if matching_files:
        file_to_read = matching_files[0]

        df = pd.read_csv(file_to_read)
        if key == 'frm':
            frm_info = df
        elif key == 'wp':
            wp_info = df
        elif key == 'cri':
            cri_info = df
        elif key == 'vo':
            vo_info = df
        
        print(f"Successfully read {file_path}")

    else:
        print(f"No file found at: {file_path}")
        sys.exit("Exit the program.")

folder_name = "fig"

if not os.path.exists(folder_name):
    os.makedirs(folder_name)
    print(f"Folder '{folder_name}' has been created.")
else:
    print(f"Folder '{folder_name}' already exists.")

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
    filtered_comma = [num for num in comma if num]
    result = list(map(float, filtered_comma))

    return result

data_len = min(len(frm_info), len(wp_info), len(cri_info), len(vo_info))

for i in range(0, data_len):
    PosX = char_to_num(frm_info.loc[i]['.m_fltPos_X'])
    PosY = char_to_num(frm_info.loc[i]['.m_fltPos_Y'])

    Xo = PosY[0]
    Yo = PosX[0]
    Xt1 = PosY[1]
    Yt1 = PosX[1]
    Xt2 = PosY[2]
    Yt2 = PosX[2]
    Xt3 = PosY[3]
    Yt3 = PosX[3]

    Xo_list.append(Xo)
    Yo_list.append(Yo)
    Xt1_list.append(Xt1)
    Yt1_list.append(Yt1)
    Xt2_list.append(Xt2)
    Yt2_list.append(Yt2)
    Xt3_list.append(Xt3)
    Yt3_list.append(Yt3)

    heading = char_to_num(frm_info.loc[i]['.m_fltHeading'])
    Co = heading[0]
    Ct1 = heading[1]
    Ct2 = heading[2]
    Ct3 = heading[3]

    V_opt = char_to_num(vo_info.loc[i]['.V_opt'])
    Vx = V_opt[1]
    Vy = V_opt[0]

    dcpa_all = char_to_num(cri_info.loc[i]['.DCPA'])
    dcpa1 = round(dcpa_all[0], 1)
    dcpa2 = round(dcpa_all[1], 1)
    dcpa3 = round(dcpa_all[2], 1)
    dcpa1_list.append(dcpa1)
    dcpa2_list.append(dcpa2)
    dcpa3_list.append(dcpa3)

    tcpa_all = char_to_num(cri_info.loc[i]['.TCPA'])
    tcpa1 = abs(round(tcpa_all[0], 1))
    tcpa2 = abs(round(tcpa_all[1], 1))
    tcpa3 = abs(round(tcpa_all[2], 1))
    tcpa1_list.append(tcpa1)
    tcpa2_list.append(tcpa2)
    tcpa3_list.append(tcpa3)

    cri = char_to_num(cri_info.loc[i]['.CRI'])
    cri1 = cri[0]
    cri2 = cri[1]
    cri3 = cri[2]
    cri1_list.append(cri1)
    cri2_list.append(cri2)
    cri3_list.append(cri3)

    Heading = heading[0] 
    if 0 <= Heading <= 180:
        Heading = Heading
    else:
        Heading = Heading - 360
    Heading_list.append(Heading)

    rf = char_to_num(cri_info.loc[i]['.Rf'])
    rf1 = rf[0]
    rf2 = rf[1]
    rf3 = rf[2]
    ra = char_to_num(cri_info.loc[i]['.Ra'])
    ra1 = ra[0]
    ra2 = ra[1]
    ra3 = ra[2]
    rs = char_to_num(cri_info.loc[i]['.Rs'])
    rs1 = rs[0]
    rs2 = rs[1]
    rs3 = rs[2]
    rp = char_to_num(cri_info.loc[i]['.Rp'])
    rp1 = rp[0]
    rp2 = rp[1]
    rp3 = rp[2]

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
    C7x = collision_cone[13]
    C7y = collision_cone[12]
    C8x = collision_cone[15]
    C8y = collision_cone[14]
    C9x = collision_cone[17]
    C9y = collision_cone[16]

    aa = Visualization(ship_L/scale, ship_B/scale, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list,
    Ct1, Ct2, Ct3, rf1, ra1, rs1, rp1, rf2, ra2, rs2, rp2, rf3, ra3, rs3, rp3,  Vx, Vy,
    C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y, C7x, C7y, C8x, C8y, C9x, C9y, 
    dcpa1_list, dcpa2_list, dcpa3_list, tcpa1_list, tcpa2_list, tcpa3_list, cri1_list, cri2_list, cri3_list, Heading_list, time)
    
    print(i)
    print(round(i/len(frm_info)*100,2),"%")
    
    Save_image = aa.ploting(name=f'{folder_name}/snap{str(i)}.png')