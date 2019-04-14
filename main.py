# controller
import numpy as np
np.seterr(divide='ignore', invalid='ignore')
import matplotlib.pyplot as plt
import pandas as pd
from utils.animation import Animation_robot
import math
import sys

from utils.basic_functions import min_max_normalize, angle_range_corrector
from dwa import DWA, Simulator_DWA_robot
from utils.car import Two_wheeled_robot, Const_goal, Const_obs, Path, Obstacle

# ルール
# x, y, thは基本的に今のstate
# g_ はgoal
# traj_ は過去の軌跡
# 単位は，角度はrad，位置はm
# 二輪モデルなので入力は速度と角速度


class Main_controller():# Mainの制御クラス
    def __init__(self):
        self.robot = Two_wheeled_robot(0.0, 0.0, 0.0)
        self.goal_maker = Const_goal()
        self.obstacles = [Const_obs() for i in range(2)]        
        self.controller = DWA()

        # ここを変えたら他もチェック
        self.samplingtime = 0.1

    def run_to_goal(self):
        goal_flag = False
        time_step = 0

        while not goal_flag:
        # for i in range(250):
            g_x, g_y = self.goal_maker.calc_goal(time_step)

            # 障害物の位置を設定
            if time_step <= 50:
                self.obstacles[0].set_info(4, 1, 0.25)
                self.obstacles[1].set_info(4, 4, 0.25)
            elif time_step <= 100:
                self.obstacles[0].set_info(-4, -1, 0.25)
                self.obstacles[1].set_info(4, 4, 0.25)
            else:
                self.obstacles[0].set_info(5, -5, 1)
                self.obstacles[1].set_info(4, 4, 0.25)
            # 障害物の位置の設定終わり
            # 入力決定
            paths, opt_path = self.controller.calc_input(g_x, g_y, self.robot, self.obstacles)

            u_th = opt_path.u_th
            u_v = opt_path.u_v

            # 入力で状態更新
            self.robot.update_state(u_th, u_v, self.samplingtime)

            # goal判定
            dis_to_goal = np.sqrt((g_x-self.robot.x)*(g_x-self.robot.x) + (g_y-self.robot.y)*(g_y-self.robot.y))
            if dis_to_goal < 0.5:
                goal_flag = True
            
            time_step += 1

        return self.robot.traj_x, self.robot.traj_y, self.robot.traj_th, \
                self.goal_maker.traj_g_x, self.goal_maker.traj_g_y, self.controller.traj_paths, self.controller.traj_opt, self.obstacles

def main():
    animation = Animation_robot()
    animation.fig_set()

    controller = Main_controller()
    traj_x, traj_y, traj_th, traj_g_x, traj_g_y, traj_paths, traj_opt, obstacles = controller.run_to_goal()
    animation.func_anim_plot(traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y, traj_opt, obstacles)

if __name__ == '__main__':
    main()
