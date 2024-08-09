import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd
from math import *
import glob
import time
import gc

class Visualization:
    def __init__(self, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt_list, Yt_list, Ct_list, rf, ra, rs, rp, Vx, Vy,
    collision_points, dcpa_list, tcpa_list, cri_list, Heading_list, graph_time, number_of_ts):

        self.number_of_ts = number_of_ts
        
        self.L = L
        self.B = B
        self.Xo = Xo
        self.Xo_list = Xo_list
        self.Yo = Yo
        self.Yo_list = Yo_list
        self.Co = np.deg2rad(Co)
        self.Vx = Vx
        self.Vy = Vy

        self.Xt1 = Xt_list[0][-1]
        self.Xt1_list = Xt_list[0]
        self.Yt1 = Yt_list[0][-1]
        self.Yt1_list = Yt_list[0]
        self.Ct1 = np.deg2rad(Ct_list[0])
        self.Rf1 = round(rf[0], 4)
        self.Ra1 = round(ra[0], 4)
        self.Rs1 = round(rs[0], 4)
        self.Rp1 = round(rp[0], 4)
        self.C1x, self.C1y = collision_points[0]
        self.C2x, self.C2y = collision_points[1]
        self.C3x, self.C3y = collision_points[2]
        self.DCPA1 = dcpa_list[0]
        self.TCPA1 = tcpa_list[0]
        self.cri1 = cri_list[0]

        if self.number_of_ts >= 2:
            self.Xt2 = Xt_list[1][-1]
            self.Xt2_list = Xt_list[1]
            self.Yt2 = Yt_list[1][-1]
            self.Yt2_list = Yt_list[1]
            self.Ct2 = np.deg2rad(Ct_list[1])
            self.Rf2 = round(rf[1], 4)
            self.Ra2 = round(ra[1], 4)
            self.Rs2 = round(rs[1], 4)
            self.Rp2 = round(rp[1], 4)
            self.C4x, self.C4y = collision_points[3]
            self.C5x, self.C5y = collision_points[4]
            self.C6x, self.C6y = collision_points[5]
            self.DCPA2 = dcpa_list[1]
            self.TCPA2 = tcpa_list[1]
            self.cri2 = cri_list[1]
            
        if self.number_of_ts >= 3:
            self.Xt3 = Xt_list[2][-1]
            self.Xt3_list = Xt_list[2]
            self.Yt3 = Yt_list[2][-1]
            self.Yt3_list = Yt_list[2] 
            self.Ct3 = np.deg2rad(Ct_list[2])
            self.Rf3 = round(rf[2], 4)
            self.Ra3 = round(ra[2], 4)
            self.Rs3 = round(rs[2], 4)
            self.Rp3 = round(rp[2], 4)
            self.C7x, self.C7y = collision_points[6]
            self.C8x, self.C8y = collision_points[7]
            self.C9x, self.C9y = collision_points[8]
            self.DCPA3 = dcpa_list[2]
            self.TCPA3 = tcpa_list[2]
            self.cri3 = cri_list[2]
            
        if self.number_of_ts >= 4:            
            self.Xt4 = Xt_list[3][-1]
            self.Xt4_list = Xt_list[3]
            self.Yt4 = Yt_list[3][-1]
            self.Yt4_list = Yt_list[3] 
            self.Ct4 = np.deg2rad(Ct_list[3])
            self.Rf4 = round(rf[3], 4)
            self.Ra4 = round(ra[3], 4)
            self.Rs4 = round(rs[3], 4)
            self.Rp4 = round(rp[3], 4)
            self.C10x, self.C10y = collision_points[9]
            self.C11x, self.C11y = collision_points[10]
            self.C12x, self.C12y = collision_points[11]
            self.DCPA4 = dcpa_list[3]
            self.TCPA4 = tcpa_list[3]
            self.cri4 = cri_list[3]
            
        self.Heading = Heading_list
        self.graph_time = graph_time

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

        axes[0,0].set_xticks(np.arange(-250, 250, 50))  # 원하는 범위 쁠마 50씩 해주기
        axes[0,0].tick_params(axis='x', labelrotation=45)
        axes[0,0].set_yticks(np.arange(-100, 400, 50))  # 원하는 범위 쁠마 50씩 해주기
        axes[0,0].set_aspect('equal')
        axes[0,0].grid(linestyle='-.')
        axes[0,0].set_xlabel('X axis (m)', labelpad=1, fontsize=10)
        axes[0,0].set_ylabel('Y axis (m)', labelpad=1, fontsize=10)
        axes[0,0].axis([-200, 200, -50, 350])   # 원하는 범위 그대로 쓰기
        
        axes[0,0].arrow(self.Xo, self.Yo, self.Vx, self.Vy, head_width=5, head_length=15, color='k')

        axes[0,0].text(self.Xo + 1, self.Yo + 1, 'OS')
        Xo_traj = self.Xo_list
        Yo_traj = self.Yo_list
        axes[0, 0].plot(Xo_traj, Yo_traj, 'g', label='OS')
        self.highlight_points(axes, Xo_traj, Yo_traj, 'g')

        axes[0,0].text(self.Xt1 + 1, self.Yt1 + 1, 'TS1')
        Xt1_traj = self.Xt1_list
        Yt1_traj = self.Yt1_list
        for x, y in zip(Xt1_traj, Yt1_traj):
            axes[0, 0].scatter(x, y, c='r', s=1)
        self.highlight_points(axes, Xt1_traj, Yt1_traj, 'r')
        axes[0, 0].plot([], [], 'r-', label='TS1')

        if self.number_of_ts >= 2:  
            axes[0,0].text(self.Xt2 + 1, self.Yt2 + 1, 'TS2')
            Xt2_traj = self.Xt2_list
            Yt2_traj = self.Yt2_list
            for x, y in zip(Xt2_traj, Yt2_traj):
                axes[0, 0].scatter(x, y, c='b', s=1)
            self.highlight_points(axes, Xt2_traj, Yt2_traj, 'b')
            axes[0, 0].plot([], [], 'b-', label='TS2')

        if self.number_of_ts >= 3:              
            axes[0,0].text(self.Xt3 + 1, self.Yt3 + 1, 'TS3')
            Xt3_traj = self.Xt3_list
            Yt3_traj = self.Yt3_list
            for x, y in zip(Xt3_traj, Yt3_traj):
                axes[0, 0].scatter(x, y, c='y', s=1)
            self.highlight_points(axes, Xt3_traj, Yt3_traj, 'y')
            axes[0, 0].plot([], [], 'y-', label='TS3')

        if self.number_of_ts >= 4:              
            axes[0,0].text(self.Xt4 + 1, self.Yt4 + 1, 'TS4')
            Xt4_traj = self.Xt4_list
            Yt4_traj = self.Yt4_list
            for x, y in zip(Xt4_traj, Yt4_traj):
                axes[0, 0].scatter(x, y, c='m', s=1)
            self.highlight_points(axes, Xt4_traj, Yt4_traj, 'm')
            axes[0, 0].plot([], [], 'm-', label='TS4')

        axes[0, 0].legend(loc='best', fontsize='large')

###################### 1사분면 그림 ######################

        # can modify
        width = 40
        height = 40
        
        axes[0,1].set_xticks(np.arange(-600, 600, 5))
        axes[0,1].tick_params(axis='x', labelrotation=45)
        axes[0,1].set_yticks(np.arange(-600, 600, 5))
        axes[0,1].set_aspect('equal')
        axes[0,1].grid(linestyle='-.')
        axes[0,1].set_xlabel('X axis (m)', labelpad=1, fontsize=10)
        axes[0,1].set_ylabel('Y axis (m)', labelpad=1, fontsize=10)
        axes[0, 1].axis([self.Xo - width / 2, self.Xo + width / 2, self.Yo - (height*(1/5)), self.Yo + (height*(4/5))])

        OS_boundary1, OS_boundary2, OS_boundary3, OS_boundary4, OS_boundary5 = self.create_coord_ship(self.Xo, self.Yo, self.Co)
        axes[0,1].add_patch(
            patches.Polygon(
                (OS_boundary1, OS_boundary2, OS_boundary3, OS_boundary4, OS_boundary5),
                closed=True,
                edgecolor='black',
                facecolor='Green'
            )
        )
        axes[0,1].arrow(self.Xo, self.Yo, self.Vx, self.Vy, head_width=1, head_length=2, color='k')
        
        Q1, Q2, Q3, Q4 = self.create_ship_domain(self.Rf1, self.Ra1, self.Rs1, self.Rp1)
        cc1 = [[self.C1x, self.C2x], [self.C1x, self.C3x]]
        cc2 = [[self.C1y, self.C2y], [self.C1y, self.C3y]]
        TS_boundary1, TS_boundary2, TS_boundary3, TS_boundary4, TS_boundary5 = self.create_coord_ship(self.Xt1, self.Yt1, self.Ct1)
        axes[0,1].add_patch(
            patches.Polygon(
                (TS_boundary1, TS_boundary2, TS_boundary3, TS_boundary4, TS_boundary5),
                closed=True,
                edgecolor='black',
                facecolor='Red'
            )
        )
        axes[0,1].plot(Q1[0], Q1[1], c='r')
        axes[0,1].plot(Q2[0], Q2[1], c='r')
        axes[0,1].plot(Q3[0], Q3[1], c='r')
        axes[0,1].plot(Q4[0], Q4[1], c='r')
        axes[0,1].fill([self.C1x, self.C2x, self.C3x], [self.C1y, self.C2y, self.C3y], color='r', alpha=0.5)
        
        for cc in [cc1, cc2]:
            for i in range(len(cc[0])):
                axes[0, 1].plot(cc[0][i], cc[1][i], color='k')
                
        if self.number_of_ts >= 2:  
            Q5, Q6, Q7, Q8 = self.create_ship_domain(self.Rf2, self.Ra2, self.Rs2, self.Rp2)
            cc3 = [[self.C4x, self.C5x], [self.C4x, self.C6x]]
            cc4 = [[self.C4y, self.C5y], [self.C4y, self.C6y]]
            TS_boundary6, TS_boundary7, TS_boundary8, TS_boundary9, TS_boundary10 = self.create_coord_ship(self.Xt2, self.Yt2, self.Ct2)
            axes[0,1].add_patch(
                patches.Polygon(
                    (TS_boundary6, TS_boundary7, TS_boundary8, TS_boundary9, TS_boundary10),
                    closed=True,
                    edgecolor='black',
                    facecolor='Blue'
                )
            )
            axes[0,1].plot(Q5[0], Q5[1], c='b')
            axes[0,1].plot(Q6[0], Q6[1], c='b')
            axes[0,1].plot(Q7[0], Q7[1], c='b')
            axes[0,1].plot(Q8[0], Q8[1], c='b')
            axes[0,1].fill([self.C4x, self.C5x, self.C6x], [self.C4y, self.C5y, self.C6y], color='b', alpha=0.5)
            
            for cc in [cc1, cc2, cc3, cc4]:
                for i in range(len(cc[0])):
                    axes[0, 1].plot(cc[0][i], cc[1][i], color='k')
                    
        if self.number_of_ts >= 3: 
            Q9, Q10, Q11, Q12 = self.create_ship_domain(self.Rf3, self.Ra3, self.Rs3, self.Rp3) 
            cc5 = [[self.C7x, self.C8x], [self.C7x, self.C9x]]
            cc6 = [[self.C7y, self.C8y], [self.C7y, self.C9y]]
            TS_boundary11, TS_boundary12, TS_boundary13, TS_boundary14, TS_boundary15 = self.create_coord_ship(self.Xt3, self.Yt3, self.Ct3)
            axes[0,1].add_patch(
                patches.Polygon(
                    (TS_boundary11, TS_boundary12, TS_boundary13, TS_boundary14, TS_boundary15),
                    closed=True,
                    edgecolor='black',
                    facecolor='y'
                )
            )
            axes[0,1].plot(Q9[0], Q9[1], c='y')
            axes[0,1].plot(Q10[0], Q10[1], c='y')
            axes[0,1].plot(Q11[0], Q11[1], c='y')
            axes[0,1].plot(Q12[0], Q12[1], c='y')
            axes[0,1].fill([self.C7x, self.C8x, self.C9x], [self.C7y, self.C8y, self.C9y], color='y', alpha=0.5)
            
            for cc in [cc1, cc2, cc3, cc4, cc5, cc6]:
                for i in range(len(cc[0])):
                    axes[0, 1].plot(cc[0][i], cc[1][i], color='k')
                    
        if self.number_of_ts >= 4:  
            Q13, Q14, Q15, Q16 = self.create_ship_domain(self.Rf4, self.Ra4, self.Rs4, self.Rp4)
            cc7 = [[self.C10x, self.C11x], [self.C10x, self.C12x]]
            cc8 = [[self.C10y, self.C11y], [self.C10y, self.C12y]]
            TS_boundary16, TS_boundary17, TS_boundary18, TS_boundary19, TS_boundary20 = self.create_coord_ship(self.Xt4, self.Yt4, self.Ct4)
            axes[0,1].add_patch(
                patches.Polygon(
                    (TS_boundary16, TS_boundary17, TS_boundary18, TS_boundary19, TS_boundary20),
                    closed=True,
                    edgecolor='black',
                    facecolor='m'
                )
            )
            axes[0,1].plot(Q13[0], Q13[1], c='m')
            axes[0,1].plot(Q14[0], Q14[1], c='m')
            axes[0,1].plot(Q15[0], Q15[1], c='m')
            axes[0,1].plot(Q16[0], Q16[1], c='m')
            axes[0,1].fill([self.C10x, self.C11x, self.C12x], [self.C10y, self.C11y, self.C12y], color='m', alpha=0.5)
        
            for cc in [cc1, cc2, cc3, cc4, cc5, cc6, cc7, cc8]:
                for i in range(len(cc[0])):
                    axes[0, 1].plot(cc[0][i], cc[1][i], color='k')

###################### 3사분면 그림 ######################        

        time_x = list(range(1, self.graph_time+1, 1))
        axes[1,0].grid(linestyle='-.')
        axes[1,0].set_xlabel('Time (s)', labelpad=1, fontsize=10)
        axes[1,0].set_ylabel('DCPA', labelpad=1, fontsize=10)

        ax2 = axes[1,0].twinx()        
        
        dcpa1_ya = self.DCPA1
        tcpa1_ya = self.TCPA1
        dcpa1_yb = []
        tcpa1_yb = []
        for i in range(len(time_x)-len(dcpa1_ya)):
            dcpa1_yb.append(0)
        for i in range(len(time_x)-len(tcpa1_ya)):
            tcpa1_yb.append(0)
        dcpa1_y = dcpa1_ya + dcpa1_yb
        tcpa1_y = tcpa1_ya + tcpa1_yb
        axes[1,0].plot(time_x, dcpa1_y, 'r', label="DCPA1")
        axes[1,0].axis([0, self.graph_time, 0, max(dcpa1_y)])
        ax2.plot(time_x, tcpa1_y, 'r--', label="TCPA1")
        ax2.axis([0, self.graph_time, min(tcpa1_y), max(tcpa1_y)])
        
        if self.number_of_ts >= 2:
            dcpa2_ya = self.DCPA2
            tcpa2_ya = self.TCPA2
            dcpa2_yb = []
            tcpa2_yb = []
            for i in range(len(time_x)-len(dcpa2_ya)):
                dcpa2_yb.append(0)
            for i in range(len(time_x)-len(tcpa2_ya)):
                tcpa2_yb.append(0)
            dcpa2_y = dcpa2_ya + dcpa2_yb
            tcpa2_y = tcpa2_ya + tcpa2_yb
            axes[1,0].plot(time_x, dcpa2_y, 'b', label="DCPA2")
            axes[1,0].axis([0, self.graph_time, 0, max(max(dcpa1_y), max(dcpa2_y))])
            ax2.plot(time_x, tcpa2_y, 'b--', label="TCPA2")
            ax2.axis([0, self.graph_time, min(min(tcpa1_y), min(tcpa2_y)), max(max(tcpa1_y), max(tcpa2_y))])
            
        if self.number_of_ts >= 3:            
            dcpa3_ya = self.DCPA3
            tcpa3_ya = self.TCPA3
            dcpa3_yb = []
            tcpa3_yb = []
            for i in range(len(time_x)-len(dcpa3_ya)):
                dcpa3_yb.append(0)
            for i in range(len(time_x)-len(tcpa3_ya)):
                tcpa3_yb.append(0)
            dcpa3_y = dcpa3_ya + dcpa3_yb
            tcpa3_y = tcpa3_ya + tcpa3_yb
            axes[1,0].plot(time_x, dcpa3_y, 'y', label="DCPA3")
            axes[1,0].axis([0, self.graph_time, 0, max(max(dcpa1_y), max(dcpa2_y), max(dcpa3_y))])
            ax2.plot(time_x, tcpa3_y, 'y--', label="TCPA3")
            ax2.axis([0, self.graph_time, min(min(tcpa1_y), min(tcpa2_y), min(tcpa3_y)), max(max(tcpa1_y), max(tcpa2_y), max(tcpa3_y))])
            
        if self.number_of_ts >= 4:            
            dcpa4_ya = self.DCPA4
            tcpa4_ya = self.TCPA4
            dcpa4_yb = []
            tcpa4_yb = []
            for i in range(len(time_x)-len(dcpa4_ya)):
                dcpa4_yb.append(0)
            for i in range(len(time_x)-len(tcpa4_ya)):
                tcpa4_yb.append(0)
            dcpa4_y = dcpa4_ya + dcpa4_yb
            tcpa4_y = tcpa4_ya + tcpa4_yb
            axes[1,0].plot(time_x, dcpa4_y, 'm', label="DCPA4")
            axes[1,0].axis([0, self.graph_time, 0, max(max(dcpa1_y), max(dcpa2_y), max(dcpa3_y), max(dcpa4_y))])
            ax2.plot(time_x, tcpa4_y, 'm--', label="TCPA4")
            ax2.axis([0, self.graph_time, min(min(tcpa1_y), min(tcpa2_y), min(tcpa3_y), min(tcpa4_y)), max(max(tcpa1_y), max(tcpa2_y), max(tcpa3_y), max(tcpa4_y))])



        lines, labels = axes[1,0].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,0].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')

###################### 4사분면 그림 ######################  

        Heading_ya = self.Heading
        Heading_yb = []

        for i in range(len(time_x)-len(Heading_ya)):
            Heading_yb.append(0)
        Heading_y = Heading_ya + Heading_yb

        ax2 = axes[1,1].twinx()
        ax2.plot(time_x, Heading_y, 'g-', label='Heading', )
        ax2.fill_between(time_x, Heading_y, color='g', alpha=0)
        ax2.set_ylim(-180, 180)
        ax2.set_ylabel('Heading(deg)', labelpad=1, fontsize=10, rotation=270)
        ax2.yaxis.tick_right()

        cri1_ya = self.cri1
        cri1_y = np.pad(cri1_ya, (0, len(time_x) - len(cri1_ya)), 'constant')
        axes[1,1].plot(time_x, cri1_y, 'r--', label='CRI1')
        axes[1,1].fill_between(time_x, cri1_y, color='r', alpha=0.1)

        if self.number_of_ts >= 2:        
            cri2_ya = self.cri2
            cri2_y = np.pad(cri2_ya, (0, len(time_x) - len(cri2_ya)), 'constant')
            axes[1,1].plot(time_x, cri2_y, 'b--', label='CRI2')
            axes[1,1].fill_between(time_x, cri2_y, color='b', alpha=0.1)

        if self.number_of_ts >= 3:              
            cri3_ya = self.cri3
            cri3_y = np.pad(cri3_ya, (0, len(time_x) - len(cri3_ya)), 'constant')
            axes[1,1].plot(time_x, cri3_y, 'y--', label='CRI3')
            axes[1,1].fill_between(time_x, cri3_y, color='y', alpha=0.1)

        if self.number_of_ts >= 4:            
            cri4_ya = self.cri4
            cri4_y = np.pad(cri4_ya, (0, len(time_x) - len(cri4_ya)), 'constant')
            axes[1,1].plot(time_x, cri4_y, 'm--', label='CRI4')
            axes[1,1].fill_between(time_x, cri4_y, color='m', alpha=0.1)
        
        axes[1,1].set_xlabel('Time (s)', labelpad=1, fontsize=10)
        axes[1,1].set_ylabel('CRI', labelpad=1, fontsize=10)
        axes[1,1].axis([0, self.graph_time, 0, 1])
        
        lines, labels = axes[1,1].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,1].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')

        if name:
            plt.savefig(name, dpi=150)
        plt.tight_layout()
        plt.cla()
        plt.close()
        gc.collect()

def char_to_num(str):
    bracket = str.strip("("")")
    comma = bracket.split(',')
    filtered_comma = [num for num in comma if num]
    result = list(map(float, filtered_comma))

    return result

start_time = time.time()

# can modify
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

graph_time = len(cri_info)
dcpa_list = [[] for _ in range(5)]
tcpa_list = [[] for _ in range(5)]
cri_list = [[] for _ in range(5)]
Heading_list = []
Xo_list = []
Yo_list = []
Xt_list = [[] for _ in range(5)]
Yt_list = [[] for _ in range(5)]

data_len = min(len(frm_info), len(wp_info), len(cri_info), len(vo_info))
number_of_ts = len(char_to_num(frm_info.loc[2]['.m_nShipID'])) - 1
print(f"{number_of_ts} ts")
for i in range(0, data_len):
    PosX = char_to_num(frm_info.loc[i]['.m_fltPos_X'])
    PosY = char_to_num(frm_info.loc[i]['.m_fltPos_Y'])
    
    Xo = PosY[0]
    Yo = PosX[0]
    Xo_list.append(Xo)
    Yo_list.append(Yo)

    for j in range(1, min(5, len(PosX))):
        Xt_list[j-1].append(PosY[j])
        Yt_list[j-1].append(PosX[j])
    
    heading = char_to_num(frm_info.loc[i]['.m_fltHeading'])
    Heading = heading[0]
    if 0 <= Heading <= 180:
        Heading = Heading
    else:
        Heading = Heading - 360

    Heading_list.append(Heading)

    V_opt = char_to_num(vo_info.loc[i]['.V_opt'])
    Vx = V_opt[1]
    Vy = V_opt[0]

    dcpa_all = char_to_num(cri_info.loc[i]['.DCPA'])
    for j in range(min(3, len(dcpa_all))):
        dcpa_list[j].append(round(dcpa_all[j], 1))
    
    tcpa_all = char_to_num(cri_info.loc[i]['.TCPA'])
    for j in range(min(3, len(tcpa_all))):
        tcpa_list[j].append(abs(round(tcpa_all[j], 1)))
    
    cri = char_to_num(cri_info.loc[i]['.CRI'])
    for j in range(min(3, len(cri))):
        cri_list[j].append(cri[j])
    rf = char_to_num(cri_info.loc[i]['.Rf'])
    ra = char_to_num(cri_info.loc[i]['.Ra'])
    rs = char_to_num(cri_info.loc[i]['.Rs'])
    rp = char_to_num(cri_info.loc[i]['.Rp'])
    
    collision_cone = char_to_num(vo_info.loc[i]['.Collision_cone'])
    collision_points = [(collision_cone[2*j+1], collision_cone[2*j]) for j in range(len(collision_cone)//2)]

    aa = Visualization(ship_L/scale, ship_B/scale, Xo, Xo_list, Yo, Yo_list, Heading, 
    Xt_list, Yt_list, heading[1:], rf, ra, rs, rp, Vx, Vy,
    collision_points, dcpa_list, tcpa_list, cri_list, Heading_list, graph_time, number_of_ts)
    
    Save_image = aa.ploting(name=f'{folder_name}/snap{str(i)}.png')
    
    end_time = time.time()
    execution_time = end_time - start_time
    execution_time_print = time.strftime('%H:%M:%S', time.gmtime(execution_time))
    
    print(f"time:       {execution_time_print} s")
    print(f"NO.:        {i}")
    print(f"Progress:   {round(i/len(frm_info)*100,2)} %")
    print("\n")