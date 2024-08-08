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
    # def __init__(self, ship_ID, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    # Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list,
    # Ct1, Ct2, Ct3, Rf1, Ra1, Rs1, Rp1, Rf2, Ra2, Rs2, Rp2, Rf3, Ra3, Rs3, Rp3, Vx, Vy, Gx, Gy, 
    # C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y, C7x, C7y, C8x, C8y, C9x, C9y, DCPA1, DCPA2, DCPA3, TCPA1, TCPA2, TCPA3, cri1, cri2, cri3, Heading, time):
        
    def __init__(self, ship_ID, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list,
    Ct1, Ct2, Ct3, Rf1, Ra1, Rs1, Rp1, Rf2, Ra2, Rs2, Rp2, Rf3, Ra3, Rs3, Rp3, Vx, Vy, 
    C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y, C7x, C7y, C8x, C8y, C9x, C9y, DCPA1, DCPA2, DCPA3, TCPA1, TCPA2, TCPA3, cri1, cri2, cri3, Heading, time):
        self.ship_ID = ship_ID
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
        # self.Gx = Gx
        # self.Gy = Gy

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

        # self.enc1 = enc1
        # self.enc2 = enc2
        # self.enc3 = enc3
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

        ship_shape = [
              (0, 1),
              (-0.5, 0.5),
              (-0.5, -1),
              (0.5, -1),
              (0.5, 0.5),
              ]
        # colliscion cone
        cc1 = [[self.C1x, self.C2x], [self.C1x, self.C3x]]
        cc2 = [[self.C1y, self.C2y], [self.C1y, self.C3y]]
        cc3 = [[self.C4x, self.C5x], [self.C4x, self.C6x]]
        cc4 = [[self.C4y, self.C5y], [self.C4y, self.C6y]]
        cc5 = [[self.C7x, self.C8x], [self.C7x, self.C9x]]
        cc6 = [[self.C7y, self.C8y], [self.C7y, self.C9y]]
        


        OS_boundary1, OS_boundary2, OS_boundary3 = self.create_coord_OS(self.Xo, self.Yo, self.Co)
        Q1, Q2, Q3, Q4 = self.create_ship_domain(self.Rf1, self.Ra1, self.Rs1, self.Rp1)
        Q5, Q6, Q7, Q8 = self.create_ship_domain(self.Rf2, self.Ra2, self.Rs2, self.Rp2)
        Q9, Q10, Q11, Q12 = self.create_ship_domain(self.Rf3, self.Ra3, self.Rs3, self.Rp3)

        # TODO for all TS
        TS_boundary1, TS_boundary2, TS_boundary3 = self.create_coord_TS(self.Xt1, self.Yt1, self.Ct1)
        TS_boundary4, TS_boundary5, TS_boundary6 = self.create_coord_TS(self.Xt2, self.Yt2, self.Ct2)
        TS_boundary7, TS_boundary8, TS_boundary9 = self.create_coord_TS(self.Xt3, self.Yt3, self.Ct3)

        f,axes = plt.subplots(2, 2)
        f.set_size_inches((12, 12))

        axes[0,0].set_xticks(np.arange(-320, 320, 50))
        axes[0,0].tick_params(axis='x', labelrotation=45)
        axes[0,0].set_yticks(np.arange(-320, 320, 50))
        axes[0,0].set_aspect('equal')
        axes[0,0].grid(linestyle='-.')
        axes[0,0].set_xlabel('X axis (m)', labelpad=1, fontsize=16)
        axes[0,0].set_ylabel('Y axis (m)', labelpad=1, fontsize=16)
        axes[0,0].axis([-320, 320, -320, 320])
        # axes[0,0].arrow(self.Xo, self.Yo, self.Gx - self.Xo, self.Gy - self.Yo, head_width=2, head_length=2, color='y')
        axes[0,0].text(self.Xo + 1, self.Yo + 1, 'OS')
        # TODO for all TS
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
        

        axes[0, 0].plot(Xo_traj, Yo_traj, 'g')
        axes[0, 0].plot(Xt1_traj, Yt1_traj, 'r')
        axes[0, 0].plot(Xt2_traj, Yt2_traj, 'b')
        axes[0, 0].plot(Xt3_traj, Yt3_traj, 'y')






        axes[0,1].set_xticks(np.arange(-600, 600, 10))
        axes[0,1].set_yticks(np.arange(-600, 600, 10))
        axes[0,1].set_aspect('equal')
        axes[0,1].grid(linestyle='-.')
        axes[0,1].set_xlabel('X axis (m)', labelpad=1, fontsize=16)
        axes[0,1].set_ylabel('Y axis (m)', labelpad=1, fontsize=16)
        # TODO auto axis
        # axes[0,1].axis([self.Xo-45, self.Xo+45, self.Yo-25, self.Yo+65])
        axes[0,1].axis([self.Xo-25, self.Xo+25, self.Yo-25, self.Yo+25])


        ##full=+600

        axes[0,1].add_patch(
            patches.Polygon(
                (OS_boundary1, OS_boundary2, OS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Green'
            )
        )

        # TODO for all TS
        axes[0,1].add_patch(
            patches.Polygon(
                (TS_boundary1, TS_boundary2, TS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Red'
            )
        )

        axes[0,1].add_patch(
            patches.Polygon(
                (TS_boundary4, TS_boundary5, TS_boundary6),
                closed=True,
                edgecolor='black',
                facecolor='Blue'
            )
        )

        axes[0,1].add_patch(
            patches.Polygon(
                (TS_boundary7, TS_boundary8, TS_boundary9),
                closed=True,
                edgecolor='black',
                facecolor='y'
            )
        )

        # TODO for all TS
        # axes[0,1].plot(Q1[0], Q1[1], c='r', label = 'Ship domain for TS1')
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

        # TODO for all TS
        for i in range(len(cc1)):
            axes[0,1].plot(cc1[i], cc2[i], color='k')
        for i in range(len(cc3)):
            axes[0,1].plot(cc3[i], cc4[i], color='k')
        for i in range(len(cc5)):
            axes[0,1].plot(cc5[i], cc6[i], color='k')

        axes[0,1].fill([self.C1x, self.C2x, self.C3x], [self.C1y, self.C2y, self.C3y], color='r', alpha=0.7)
        axes[0,1].fill([self.C4x, self.C5x, self.C6x], [self.C4y, self.C5y, self.C6y], color='b', alpha=0.5)
        axes[0,1].fill([self.C7x, self.C8x, self.C9x], [self.C7y, self.C8y, self.C9y], color='y', alpha=0.5)


        axes[0,1].arrow(self.Xo, self.Yo, self.Vx, self.Vy, head_width=1, head_length=1, color='y')

        
        
        
        time_x = list(range(1, self.time+1, 1))
        dcpa1_ya = self.DCPA1
        dcpa2_ya = self.DCPA2
        dcpa3_ya = self.DCPA3
        tcpa1_ya = self.TCPA1
        tcpa2_ya = self.TCPA2
        tcpa3_ya = self.TCPA3

        dcpa1_yb = []
        dcpa2_yb = []
        dcpa3_yb = []
        tcpa1_yb = []
        tcpa2_yb = []
        tcpa3_yb = []

        for i in range(len(time_x)-len(dcpa1_ya)):
            dcpa1_yb.append(0)
        for i in range(len(time_x)-len(dcpa2_ya)):
            dcpa2_yb.append(0)
        for i in range(len(time_x)-len(dcpa3_ya)):
            dcpa3_yb.append(0)

        dcpa1_y = dcpa1_ya + dcpa1_yb
        dcpa2_y = dcpa2_ya + dcpa2_yb
        dcpa3_y = dcpa3_ya + dcpa3_yb

        for i in range(len(time_x)-len(tcpa1_ya)):
            tcpa1_yb.append(0)
        for i in range(len(time_x)-len(tcpa2_ya)):
            tcpa2_yb.append(0)
        for i in range(len(time_x)-len(tcpa3_ya)):
            tcpa3_yb.append(0)

        tcpa1_y = tcpa1_ya + tcpa1_yb
        tcpa2_y = tcpa2_ya + tcpa2_yb
        tcpa3_y = tcpa3_ya + tcpa3_yb

        axes[1,0].plot(time_x, dcpa1_y, 'r', label="DCPA1")
        axes[1,0].plot(time_x, dcpa2_y, 'b', label="DCPA2")
        axes[1,0].plot(time_x, dcpa3_y, 'y', label="DCPA3")
        axes[1,0].grid(linestyle='-.')
        axes[1,0].set_xlabel('Time (s)', labelpad=1, fontsize=16)
        axes[1,0].set_ylabel('DCPA', labelpad=1, fontsize=16)

        axes[1,0].axis([0, self.time, 0, max(max(dcpa1_y), max(dcpa2_y), max(dcpa3_y))]) # 로그 스케일 범위 설정

        ax2 = axes[1,0].twinx()
        ax2.plot(time_x, tcpa1_y, 'r--', label="TCPA1")
        ax2.plot(time_x, tcpa2_y, 'b--', label="TCPA2")
        ax2.plot(time_x, tcpa3_y, 'y--', label="TCPA3")
        # ax2.set_ylabel('TCPA', labelpad=1)
        # ax2.yaxis.label.set_label_coords(1.05, 0.5)
        ax2.axis([0, self.time, min(min(tcpa1_y), min(tcpa2_y), min(tcpa3_y)), max(max(tcpa1_y), max(tcpa2_y), max(tcpa3_y))])
        # ax2.tick_params(axis='y')

        lines, labels = axes[1,0].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,0].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')




        ###################### 조우 상황 그림 ######################
       
        ''' 
        # axes[1,0].axis([-10, 10, -10, 10])
        # axes[1,0].set_aspect('equal')
        # axes[1,0].set_xlabel('Encounter', labelpad=15)
        # axes[1,0].add_patch(
        #     patches.Polygon(
        #         ship_shape,
        #         closed=True,
        #         edgecolor='black',
        #         facecolor='Green'
        #     )
        # )

        # def draw_encounter_wedge(ax, theta1, theta2, color):
        #       wedge = patches.Wedge((0,0), 10, theta1, theta2, facecolor=color, alpha=0.7)
        #       ax.add_patch(wedge)

        # # TODO just one ship among all ship
        # axes[1,0].arrow(0, 0, 10, tan(np.deg2rad(67.5))*10, color='k')
        # axes[1,0].arrow(0, 0, -10, tan(np.deg2rad(67.5))*10, color='k')
        # axes[1,0].arrow(0, 0, 10, -tan(np.deg2rad(22.5))*10, color='k')
        # axes[1,0].arrow(0, 0, -10, -tan(np.deg2rad(22.5))*10, color='k')

        # if self.enc1 == 'Head-on' or self.enc1 == 'Overtaking':
        #         axes[1,0].fill([0, -10, 10], [0, tan(np.deg2rad(67.5))*10, tan(np.deg2rad(67.5))*10], color='r', alpha=0.7)
        #         # draw_encounter_wedge(axes[1,0], tan(np.deg2rad(67.5))*10, tan(np.deg2rad(67.5))*10, 'r')
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

        # if self.enc3 == 'Head-on' or self.enc3 == 'Overtaking':
        #         axes[1,0].fill([0, -10, 10], [0, tan(np.deg2rad(67.5))*10, tan(np.deg2rad(67.5))*10], color='y', alpha=0.7)
        # if self.enc3 == 'Port crossing':
        #         axes[1,0].fill([0, -10, -10], [0, tan(np.deg2rad(67.5))*10, -tan(np.deg2rad(22.5))*10], color='y', alpha=0.7)
        # if self.enc3 == 'Starboard crossing':
        #         axes[1,0].fill([0, 10, 10], [0, tan(np.deg2rad(67.5))*10, -tan(np.deg2rad(22.5))*10], color='y', alpha=0.7)
        # if self.enc3 == 'Overtaken':
        #         axes[1,0].fill([0, -10, 10], [0, -tan(np.deg2rad(22.5))*10, -tan(np.deg2rad(22.5))*10], color='y', alpha=0.7)
        #         axes[1,0].fill([-10, 10, 10, -10], [-tan(np.deg2rad(22.5))*10, -tan(np.deg2rad(22.5))*10, -10, -10], color='y', alpha=0.7)
        '''



    ###################### DCPA, TCPA 그림 ######################
    
        # TODO for all ship
        time_x = list(range(1, self.time+1, 1))
        cri1_ya = self.cri1
        cri2_ya = self.cri2
        cri3_ya = self.cri3
        cri1_yb = []
        cri2_yb = []
        cri3_yb = []

        Heading_ya = self.Heading
        Heading_yb = []

        for i in range(len(time_x)-len(cri1_ya)):
            cri1_yb.append(0)
        for i in range(len(time_x)-len(cri2_ya)):
            cri2_yb.append(0)
        for i in range(len(time_x)-len(cri3_ya)):
            cri3_yb.append(0)

        cri1_y = cri1_ya + cri1_yb
        cri2_y = cri2_ya + cri2_yb
        cri3_y = cri3_ya + cri3_yb

        for i in range(len(time_x)-len(Heading_ya)):
             Heading_yb.append(0)
        Heading_y = Heading_ya + Heading_yb

        axes[1,1].plot(time_x, cri1_y, 'r--', label='CRI1')
        axes[1,1].fill_between(time_x[0:], cri1_y[0:], color='r', alpha=0.1)

        axes[1,1].plot(time_x, cri2_y, 'b--', label='CRI2')
        axes[1,1].fill_between(time_x[0:], cri2_y[0:], color='b', alpha=0.1)

        axes[1,1].plot(time_x, cri3_y, 'y--', label='CRI3')
        axes[1,1].fill_between(time_x[0:], cri3_y[0:], color='y', alpha=0.1)

        # axes[1,1].axhline(y = 0.66, xmin = 0, xmax = 1, color='r')
        axes[1,1].set_xlabel('Time (s)', labelpad=1, fontsize=16)
        axes[1,1].set_ylabel('CRI', labelpad=1, fontsize=15)
        axes[1,1].axis([0, self.time, 0, 1])

        ax2 = axes[1,1].twinx()
        ax2.plot(time_x, Heading_y, 'g-', label='Heading', )
        ax2.fill_between(time_x[0:], Heading_y[0:], color='g', alpha=0.05)
        ax2.set_ylim(-180, 180)
        ax2.set_ylabel('Heading(deg)', labelpad=1, fontsize=16, rotation=270)
        ax2.yaxis.tick_right()

        lines, labels = axes[1,1].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,1].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')





        if name:
            pyplot.savefig(name, dpi = 150)

        plt.tight_layout()
        plt.cla()
        plt.close() 



frm_info = pd.read_csv('./csv/frm.csv')
# wp_info = pd.read_csv('./csv/wp.csv')
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
    result = list(map(float, comma))

    return result

def char_to_str(str):
    result = re.findall(r'\w+\s?-?\w+', str)

    return result

def flt(input):
    str = re.findall(r'-?\d+.\d+', input)
    result = float(str[0])
    
    return result

data_len = min(len(frm_info), len(cri_info), len(vo_info))

for i in range(data_len):
    # goal_point = re.findall(r'-?\d+.\d+', wp_info.loc[i]['.group_wpts_info'])
    # goal_Xo = float(goal_point[1])
    # goal_Yo = float(goal_point[3])
    # goal_Xt1 = float(goal_point[3])
    # goal_Yt1 = float(goal_point[3])
    # goal_Xt2 = float(goal_point[3])
    # goal_Yt2 = float(goal_point[3])

    ship_ID = char_to_num(frm_info.loc[i]['.m_nShipID'])
    OS_ship_ID = ship_ID[0]
    TS1_ship_ID = ship_ID[1]
    TS2_ship_ID = ship_ID[2]
    TS3_ship_ID = ship_ID[3]

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

    enc_all = char_to_str(cri_info.loc[i]['.encounter_classification'])
    enc1 = enc_all[0]
    enc2 = enc_all[1]
    enc3 = enc_all[2]

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
    # ship ID, L, B, Xo, Yo, Xt, Yt, Co, Ct, Rf, Ra, Rs, Rp, Vx, Vy, Gx, Gy, C1x, C1y, C2x, C2y, C3x, C3y, enc, cri
    # aa = Visualization(OS_ship_ID, 2.3, 0.4, Xo, Xo_list, Yo, Yo_list, Co, Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list,
    # Ct1, Ct2, Ct3, rf1, ra1, rs1, rp1, rf2, ra2, rs2, rp2, rf3, ra3, rs3, rp3,  Vx, Vy, goal_Xo, goal_Yo, C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y, 
    # C7x, C7y, C8x, C8y, C9x, C9y, dcpa1_list, dcpa2_list, dcpa3_list, tcpa1_list, tcpa2_list, tcpa3_list, cri1_list, cri2_list, cri3_list, Heading_list, time)

    aa = Visualization(OS_ship_ID, 2.3, 0.4, Xo, Xo_list, Yo, Yo_list, Co, Xt1, Xt1_list, Yt1, Yt1_list, Xt2, Xt2_list, Yt2, Yt2_list, Xt3, Xt3_list, Yt3, Yt3_list,
    Ct1, Ct2, Ct3, rf1, ra1, rs1, rp1, rf2, ra2, rs2, rp2, rf3, ra3, rs3, rp3,  Vx, Vy, C1x, C1y, C2x, C2y, C3x, C3y, C4x, C4y, C5x, C5y, C6x, C6y, 
    C7x, C7y, C8x, C8y, C9x, C9y, dcpa1_list, dcpa2_list, dcpa3_list, tcpa1_list, tcpa2_list, tcpa3_list, cri1_list, cri2_list, cri3_list, Heading_list, time)

    print(i)
    Save_image = aa.ploting(name='fig/snap%s.png'%str(i))