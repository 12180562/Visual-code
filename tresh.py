import sys, os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import re
from math import *
import glob
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import Image

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

    def highlight_points(axes, traj_x, traj_y, color):        
        highlight_interval = 1000
        for i in range(0, len(traj_x), highlight_interval):
            axes[0, 0].scatter(traj_x[i], traj_y[i], c=color, s=10)

    def ploting(self, name = None):
        f,axes = plt.subplots(2, 2)
        f.set_size_inches((12, 12))

        # 설정을 반복문 밖으로 이동
        common_settings = {
            'xticks': np.arange(-250, 250, 50),
            'yticks': np.arange(-100, 400, 50),
            'aspect': 'equal',
            'grid': {'linestyle': '-.'},
            'xlabel': {'label': 'X axis (m)', 'labelpad': 1, 'fontsize': 10},
            'ylabel': {'label': 'Y axis (m)', 'labelpad': 1, 'fontsize': 10},
            'axis': [-200, 200, -50, 350]
        }

        for ax in axes.flatten():
            ax.set_xticks(common_settings['xticks'])
            ax.set_yticks(common_settings['yticks'])
            ax.set_aspect(common_settings['aspect'])
            ax.grid(**common_settings['grid'])
            ax.set_xlabel(**common_settings['xlabel'])
            ax.set_ylabel(**common_settings['ylabel'])
            ax.axis(common_settings['axis'])

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

        # 벡터화된 방식으로 변경
        axes[0, 0].scatter(Xt1_traj, Yt1_traj, c='r', s=1)
        axes[0, 0].scatter(Xt2_traj, Yt2_traj, c='b', s=1)
        axes[0, 0].scatter(Xt3_traj, Yt3_traj, c='y', s=1)

        highlight_interval = 1000
        def highlight_points(ax, traj_x, traj_y, color):
            ax.scatter(traj_x[::highlight_interval], traj_y[::highlight_interval], c=color, s=10)

        highlight_points(axes[0, 0], Xo_traj, Yo_traj, 'g')
        highlight_points(axes[0, 0], Xt1_traj, Yt1_traj, 'r')
        highlight_points(axes[0, 0], Xt2_traj, Yt2_traj, 'b')
        highlight_points(axes[0, 0], Xt3_traj, Yt3_traj, 'y')

        axes[0, 0].plot([], [], 'r-', label='TS1')
        axes[0, 0].plot([], [], 'b-', label='TS2')
        axes[0, 0].plot([], [], 'y-', label='TS3')
        axes[0, 0].legend(loc='best', fontsize='large')

        width = 100
        height = 100

        # colliscion cone
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
        
        axes[0, 1].set_xticks(np.arange(-600, 600, 5))
        axes[0, 1].tick_params(axis='x', labelrotation=45)
        axes[0, 1].set_yticks(np.arange(-600, 600, 5))
        axes[0, 1].set_aspect('equal')
        axes[0, 1].grid(linestyle='-.')
        axes[0, 1].set_xlabel('X axis (m)', labelpad=1, fontsize=10)
        axes[0, 1].set_ylabel('Y axis (m)', labelpad=1, fontsize=10)
        
        axes[0, 1].axis([self.Xo - width / 2, self.Xo + width / 2, self.Yo - (height*(1/4)), self.Yo + (height*(3/4))])
        
        axes[0, 1].add_patch(
            patches.Polygon(
                (OS_boundary1, OS_boundary2, OS_boundary3, OS_boundary4, OS_boundary5),
                closed=True,
                edgecolor='black',
                facecolor='Green'
            )
        )

        axes[0, 1].add_patch(
            patches.Polygon(
                (TS_boundary1, TS_boundary2, TS_boundary3, TS_boundary4, TS_boundary5),
                closed=True,
                edgecolor='black',
                facecolor='Red'
            )
        )

        axes[0, 1].add_patch(
            patches.Polygon(
                (TS_boundary6, TS_boundary7, TS_boundary8, TS_boundary9, TS_boundary10),
                closed=True,
                edgecolor='black',
                facecolor='Blue'
            )
        )

        axes[0, 1].add_patch(
            patches.Polygon(
                (TS_boundary11, TS_boundary12, TS_boundary13, TS_boundary14, TS_boundary15),
                closed=True,
                edgecolor='black',
                facecolor='y'
            )
        )

        for q in [Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11, Q12]:
            axes[0, 1].plot(q[0], q[1], c='r')

        for cc in [cc1, cc2, cc3, cc4, cc5, cc6]:
            for i in range(len(cc[0])):
                axes[0, 1].plot(cc[0][i], cc[1][i], color='k')

        axes[0, 1].fill([self.C1x, self.C2x, self.C3x], [self.C1y, self.C2y, self.C3y], color='r', alpha=0.7)
        axes[0, 1].fill([self.C4x, self.C5x, self.C6x], [self.C4y, self.C5y, self.C6y], color='b', alpha=0.5)
        axes[0, 1].fill([self.C7x, self.C8x, self.C9x], [self.C7y, self.C8y, self.C9y], color='y', alpha=0.5)

        axes[0, 1].arrow(self.Xo, self.Yo, self.Vx, self.Vy, head_width=1, head_length=1, color='y')

        # 3사분면 그림
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
        axes[1,0].grid(linestyle='-.')
        axes[1,0].set_xlabel('Time (s)', labelpad=1, fontsize=16)
        axes[1,0].set_ylabel('DCPA', labelpad=1, fontsize=16)
        axes[1,0].axis([0, self.time, 0, max(max(dcpa1_y), max(dcpa2_y), max(dcpa3_y))])

        ax2 = axes[1,0].twinx()
        ax2.plot(time_x, tcpa1_y, 'r--', label="TCPA1")
        ax2.plot(time_x, tcpa2_y, 'b--', label="TCPA2")
        ax2.plot(time_x, tcpa3_y, 'y--', label="TCPA3")
        ax2.axis([0, self.time, min(min(tcpa1_y), min(tcpa2_y), min(tcpa3_y)), max(max(tcpa1_y), max(tcpa2_y), max(tcpa3_y))])

        lines, labels = axes[1,0].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,0].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')

        # 4사분면 그림
        cri1_ya, cri2_ya, cri3_ya = self.cri1, self.cri2, self.cri3
        Heading_ya = self.Heading

        cri1_y = np.pad(cri1_ya, (0, len(time_x) - len(cri1_ya)), 'constant')
        cri2_y = np.pad(cri2_ya, (0, len(time_x) - len(cri2_ya)), 'constant')
        cri3_y = np.pad(cri3_ya, (0, len(time_x) - len(cri3_ya)), 'constant')
        Heading_y = np.pad(Heading_ya, (0, len(time_x) - len(Heading_ya)), 'constant')

        axes[1,1].plot(time_x, cri1_y, 'r--', label='CRI1')
        axes[1,1].fill_between(time_x, cri1_y, color='r', alpha=0.1)
        axes[1,1].plot(time_x, cri2_y, 'b--', label='CRI2')
        axes[1,1].fill_between(time_x, cri2_y, color='b', alpha=0.1)
        axes[1,1].plot(time_x, cri3_y, 'y--', label='CRI3')
        axes[1,1].fill_between(time_x, cri3_y, color='y', alpha=0.1)

        axes[1,1].set_xlabel('Time (s)', labelpad=1, fontsize=16)
        axes[1,1].set_ylabel('CRI', labelpad=1, fontsize=15)
        axes[1,1].axis([0, self.time, 0, 1])

        ax2 = axes[1,1].twinx()
        ax2.plot(time_x, Heading_y, 'g-', label='Heading', )
        ax2.fill_between(time_x, Heading_y, color='g', alpha=0.05)
        ax2.set_ylim(-180, 180)
        ax2.set_ylabel('Heading(deg)', labelpad=1, fontsize=16, rotation=270)
        ax2.yaxis.tick_right()

        lines, labels = axes[1,1].get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        axes[1,1].legend(lines + lines2, labels + labels2, loc='best', fontsize='large')

        if name:
            plt.savefig(name, dpi=600)
        plt.tight_layout()
        plt.cla()
        plt.close()

# 데이터 처리 및 시각화 실행
# (생략된 데이터 처리 부분 포함)
