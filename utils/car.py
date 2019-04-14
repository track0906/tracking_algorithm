import math
# path
class Path():
    def __init__(self, u_th, u_v): 
        self.x = None
        self.y = None
        self.th = None
        self.u_v = u_v
        self.u_th = u_th

class Two_wheeled_robot(): # 実際のロボット
    def __init__(self, init_x, init_y, init_th):
        # 初期状態
        self.x = init_x
        self.y = init_y
        self.th = init_th
        self.u_v = 0.0
        self.u_th = 0.0

        # 時刻歴保存用
        self.traj_x = [init_x]
        self.traj_y = [init_y]
        self.traj_th = [init_th]
        self.traj_u_v = [0.0]
        self.traj_u_th = [0.0]

    def update_state(self, u_th, u_v, dt): # stateを更新

        self.u_th = u_th
        self.u_v = u_v

        next_x = self.u_v * math.cos(self.th) * dt + self.x
        next_y = self.u_v * math.sin(self.th) * dt + self.y
        next_th = self.u_th * dt + self.th

        self.traj_x.append(next_x)
        self.traj_y.append(next_y)
        self.traj_th.append(next_th)

        self.x = next_x
        self.y = next_y
        self.th = next_th

        return self.x, self.y, self.th # stateを更新

class Const_goal():# goal作成プログラム
    def __init__(self):
        # self.human_trajectory = ...的な
        self.traj_g_x = []
        self.traj_g_y = []

    def calc_goal(self, time_step): # 本当は人の値が入ってもよいかも
        if time_step <= 50:
        # if time_step <= 100:
            g_x  = 10.0
            g_y = 10.0
        elif time_step <= 100:
            g_x = -10.0
            g_y = -10.0
        else:
            g_x = 0
            g_y = -10.0
        
        self.traj_g_x.append(g_x)
        self.traj_g_y.append(g_y)

        return g_x, g_y

class Obstacle():
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size

class Const_obs(): #１つの障害物のデータを時系列で保存
    def __init__(self):
        self.traj_obs = []

    def set_info(self, x, y, size):
        self.traj_obs.append(Obstacle(x, y, size))


    
            
        
        
    
        