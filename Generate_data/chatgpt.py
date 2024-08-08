import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from math import cos, sin, tan

class Visualization:
    def __init__(self, ship_ID, L, B, Xo, Xo_list, Yo, Yo_list, Co, 
    Xt1, Xt1_list, Yt1, Yt1_list, 
    Ct1, Rf1, Ra1, Rs1, Rp1, Vx, Vy, 
    C1x, C1y, C2x, C2y, C3x, C3y, enc1,  cri1, Heading, time):
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

        self.Vx = Vx * 3
        self.Vy = Vy * 3

        self.C1x = C1x
        self.C1y = C1y
        self.C2x = C2x
        self.C2y = C2y
        self.C3x = C3x
        self.C3y = C3y

        self.enc1 = enc1
        self.cri1 = cri1
        self.Heading = Heading
        self.time = time

    def transform(self, x, y, theta, Xs, Ys):
        x_trans = (cos(theta) * x + sin(theta) * y) + Xs
        y_trans = (-sin(theta) * x + cos(theta) * y) + Ys

        return x_trans, y_trans

    def ship_domain(self, Rf, Ra, Rs, Rp):
        x1 = np.linspace(0, Rs - 0.0001, 50)
        y1 = ((Rf ** 2) - (Rf ** 2 / Rs ** 2) * (x1 ** 2)) ** 0.5
        y2 = -((Ra ** 2) - (Ra ** 2 / Rs ** 2) * (x1 ** 2)) ** 0.5

        x2 = np.linspace(-Rp + 0.0001, 0, 50)
        y3 = ((Rf ** 2) - (Rf ** 2 / Rp ** 2) * x2 ** 2) ** 0.5
        y4 = -((Ra ** 2) - (Ra ** 2 / Rp ** 2) * x2 ** 2) ** 0.5

        return x1, x2, y1, y2, y3, y4

    def create_ship_domain(self, Rf, Ra, Rs, Rp):
        SD_points = self.ship_domain(Rf, Ra, Rs, Rp)
        Q1 = self.transform(SD_points[0], SD_points[2], self.Co, self.Xo, self.Yo)
        Q2 = self.transform(SD_points[0], SD_points[3], self.Co, self.Xo, self.Yo)
        Q3 = self.transform(SD_points[1], SD_points[4], self.Co, self.Yo)
        Q4 = self.transform(SD_points[1], SD_points[5], self.Co, self.Yo)

        return Q1, Q2, Q3, Q4

    def create_coord_OS(self, Xo, Yo, Co):
        OS_boundary1 = self.transform(0, 1.2, Co, Xo, Yo)
        OS_boundary2 = self.transform(0.3, -1.2, Co, Xo, Yo)
        OS_boundary3 = self.transform(-0.3, -1.2, Co, Xo, Yo)

        return OS_boundary1, OS_boundary2, OS_boundary3

    def create_coord_TS(self, Xt, Yt, Ct):
        TS_boundary1 = self.transform(0, 1.2, Ct, Xt, Yt)
        TS_boundary2 = self.transform(0.3, 1.2, Ct, Xt, Yt)
        TS_boundary3 = self.transform(-0.3, -1.2, Ct, Xt, Yt)

        return TS_boundary1, TS_boundary2, TS_boundary3

    def ploting(self, name=None):
        # collision cone
        if self.C1x and self.C1y and self.C2x and self.C2y and self.C3x and self.C3y:
            cc1 = [[self.C1x, self.C2x], [self.C1x, self.C3x]]
            cc2 = [[self.C1y, self.C2y], [self.C1y, self.C3y]]

        OS_boundary1, OS_boundary2, OS_boundary3 = self.create_coord_OS(self.Xo, self.Yo, self.Co)
        Q1, Q2, Q3, Q4 = self.create_ship_domain(self.Rf1, self.Ra1, self.Rs1, self.Rp1)

        # TODO for all TS
        TS_boundary1, TS_boundary2, TS_boundary3 = self.create_coord_TS(self.Xt1, self.Yt1, self.Ct1)

        f, axes = plt.subplots(2, 2)
        f.set_size_inches((12, 12))

        axes[0, 0].set_xticks(np.arange(-600, 600, 50))
        axes[0, 0].tick_params(axis='x', labelrotation=45)
        axes[0, 0].set_yticks(np.arange(-600, 600, 50))
        axes[0, 0].set_aspect('equal')
        axes[0, 0].grid(linestyle='-.')
        axes[0, 0].set_xlabel('X axis (m)', labelpad=15)
        axes[0, 0].set_ylabel('Y axis (m)', labelpad=15)
        axes[0, 0].axis([-600, 600, -600, 600])
        # axes[0,0].arrow(self.Xo, self.Yo, self.Gx - self.Xo, self.Gy - self.Yo, head_width=2, head_length=2, color='y')
        axes[0, 0].text(self.Xo + 1, self.Yo + 1, 'OS')
        # TODO for all TS
        axes[0, 0].text(self.Xt1 + 1, self.Yt1 + 1, 'TS1')

        Xo_traj = self.Xo_list
        Yo_traj = self.Yo_list
        Xt1_traj = self.Xt1_list
        Yt1_traj = self.Yt1_list

        axes[0, 0].plot(Xo_traj, Yo_traj, 'g')
        axes[0, 0].plot(Xt1_traj, Yt1_traj, 'r')

        axes[0, 1].set_xticks(np.arange(-800, 800, 5))
        axes[0, 1].set_yticks(np.arange(-800, 800, 5))
        axes[0, 1].set_aspect('equal')
        axes[0, 1].grid(linestyle='-.')
        axes[0, 1].set_xlabel('X axis (m)', labelpad=15)
        axes[0, 1].set_ylabel('Y axis (m)', labelpad=15)
        # TODO auto axis
        axes[0, 1].axis([self.Xo - 15, self.Xo + 15, self.Yo - 15, self.Yo + 15])

        axes[0, 1].add_patch(
            patches.Polygon(
                (OS_boundary1, OS_boundary2, OS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Green'
            )
        )

        # TODO for all TS
        axes[0, 1].add_patch(
            patches.Polygon(
                (TS_boundary1, TS_boundary2, TS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Red'
            )
        )

        # TODO for all TS
        axes[0, 1].plot(Q1[0], Q1[1], c='r')
        axes[0, 1].plot(Q2[0], Q2[1], c='r')
        axes[0, 1].plot(Q3[0], Q3[1], c='r')
        axes[0, 1].plot(Q4[0], Q4[1], c='r')

        if self.C1x and self.C1y and self.C2x and self.C2y and self.C3x and self.C3y:
            axes[0, 1].plot(cc1[0], cc2[0], 'purple')
            axes[0, 1].plot(cc1[1], cc2[1], 'purple')

        axes[1, 0].set_xticks(np.arange(0, 60, 10))
        axes[1, 0].set_yticks(np.arange(-0.5, 1.5, 0.5))
        axes[1, 0].grid(linestyle='-.')
        axes[1, 0].set_xlabel('time (s)', labelpad=15)
        axes[1, 0].set_ylabel('E(t)', labelpad=15)
        axes[1, 0].axis([0, 60, -0.5, 1.5])

        time = self.time
        axes[1, 0].plot(time, self.enc1, 'blue')
        axes[1, 0].plot(time, self.cri1, 'red')

        axes[1, 1].set_xticks(np.arange(-200, 200, 25))
        axes[1, 1].tick_params(axis='x', labelrotation=45)
        axes[1, 1].set_yticks(np.arange(-200, 200, 25))
        axes[1, 1].set_aspect('equal')
        axes[1, 1].grid(linestyle='-.')
        axes[1, 1].set_xlabel('X axis (m)', labelpad=15)
        axes[1, 1].set_ylabel('Y axis (m)', labelpad=15)
        axes[1, 1].axis([-200, 200, -200, 200])

        axes[1, 1].add_patch(
            patches.Polygon(
                (OS_boundary1, OS_boundary2, OS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Green'
            )
        )

        # TODO for all TS
        axes[1, 1].add_patch(
            patches.Polygon(
                (TS_boundary1, TS_boundary2, TS_boundary3),
                closed=True,
                edgecolor='black',
                facecolor='Red'
            )
        )

        # TODO for all TS
        axes[1, 1].plot(Q1[0], Q1[1], c='r')
        axes[1, 1].plot(Q2[0], Q2[1], c='r')
        axes[1, 1].plot(Q3[0], Q3[1], c='r')
        axes[1, 1].plot(Q4[0], Q4[1], c='r')

        if self.C1x and self.C1y and self.C2x and self.C2y and self.C3x and self.C3y:
            axes[1, 1].plot(cc1[0], cc2[0], 'purple')
            axes[1, 1].plot(cc1[1], cc2[1], 'purple')

        if name is not None:
            plt.savefig(name + '.jpg', format='jpg', dpi=1200)

        plt.show()
