import math
import numpy as np
from utils.basic_functions import min_max_normalize, angle_range_corrector
from utils.car import Path
class Simulator_DWA_robot(): # DWAのシミュレータ用
    def __init__(self):
        # self.model 独立二輪型
        # 加速度制限
        self.max_accelation = 1.0
        self.max_ang_accelation = 100 * math.pi /180
        # 速度制限
        self.lim_max_velo = 1.6 # m/s
        self.lim_min_velo = 0.0 # m/s
        self.lim_max_ang_velo = math.pi
        self.lim_min_ang_velo = -math.pi

    # 予想状態を作成する
    def predict_state(self, ang_velo, velo, x, y, th, dt, pre_step): # DWA用(何秒後かのstateを予測))
        next_xs = []
        next_ys = []
        next_ths = []

        for i in range(pre_step):
            temp_x = velo * math.cos(th) * dt + x
            temp_y = velo * math.sin(th) * dt + y
            temp_th = ang_velo * dt + th

            next_xs.append(temp_x)
            next_ys.append(temp_y)
            next_ths.append(temp_th)

            x = temp_x
            y = temp_y
            th = temp_th

        # print('next_xs = {0}'.format(next_xs))

        return next_xs, next_ys, next_ths # 予想した軌跡


class DWA():
    def __init__(self):
        # 初期化
        # simulation用のロボット
        self.simu_robot = Simulator_DWA_robot()

        # 予測時間(s)
        self.pre_time = 3
        self.pre_step = 30

        # 探索時の刻み幅
        self.delta_velo = 0.02
        self.delta_ang_velo = 0.02

        # サンプリングタイム(変更の場合，共通する項があるので，必ずほかのところも確認する)
        self.samplingtime = 0.1

        # 重みづけ
        self.weight_angle = 0.04
        self.weight_velo = 0.2
        self.weight_obs = 0.1

        # すべてのPathを保存
        self.traj_paths = []
        self.traj_opt = []

    def calc_input(self, g_x, g_y, state, obstacles): # stateはロボットクラスでくる
        # Path作成
        paths = self._make_path(state)
        # Path評価
        opt_path = self._eval_path(paths, g_x, g_y, state, obstacles)

        self.traj_opt.append(opt_path)

        return paths, opt_path

    def _make_path(self, state): 
        # 角度と速度の範囲算出
        min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_velos(state)

        # 全てのpathのリスト
        paths = []

        # 角速度と速度の組み合わせを全探索
        for ang_velo in np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo):
            for velo in np.arange(min_velo, max_velo, self.delta_velo):

                path = Path(ang_velo, velo)

                next_x, next_y, next_th \
                    = self.simu_robot.predict_state(ang_velo, velo, state.x, state.y, state.th, self.samplingtime, self.pre_step)

                path.x = next_x
                path.y = next_y
                path.th = next_th

                # 作ったpathを追加
                paths.append(path)

        # 時刻歴Pathを保存
        self.traj_paths.append(paths)

        return paths

    def _calc_range_velos(self, state): # 角速度と角度の範囲決定①
        # 角速度
        range_ang_velo = self.samplingtime * self.simu_robot.max_ang_accelation
        min_ang_velo = state.u_th - range_ang_velo
        max_ang_velo = state.u_th + range_ang_velo
        # 最小値
        if min_ang_velo < self.simu_robot.lim_min_ang_velo:
            min_ang_velo = self.simu_robot.lim_min_ang_velo
        # 最大値
        if max_ang_velo > self.simu_robot.lim_max_ang_velo:
            max_ang_velo = self.simu_robot.lim_max_ang_velo

        # 速度
        range_velo = self.samplingtime * self.simu_robot.max_accelation
        min_velo = state.u_v - range_velo
        max_velo = state.u_v + range_velo
        # 最小値
        if min_velo < self.simu_robot.lim_min_velo:
            min_velo = self.simu_robot.lim_min_velo
        # 最大値
        if max_velo > self.simu_robot.lim_max_velo:
            max_velo = self.simu_robot.lim_max_velo

        return min_ang_velo, max_ang_velo, min_velo, max_velo

    def _eval_path(self, paths, g_x, g_y, state, obastacles):
        # 一番近い障害物判定
        nearest_obs = self._calc_nearest_obs(state, obastacles)

        score_heading_angles = []
        score_heading_velos = []
        score_obstacles = []

        # 全てのpathで評価を検索
        for path in paths:
            # (1) heading_angle
            score_heading_angles.append(self._heading_angle(path, g_x, g_y))
            # (2) heading_velo
            score_heading_velos.append(self._heading_velo(path))
            # (3) obstacle
            score_obstacles.append(self._obstacle(path, nearest_obs))

        # print('angle = {0}'.format(score_heading_angles))
        # print('velo = {0}'.format(score_heading_velos))
        # print('obs = {0}'.format(score_obstacles))

        # 正規化
        for scores in [score_heading_angles, score_heading_velos, score_obstacles]:
            scores = min_max_normalize(scores)

        score = 0.0
        # 最小pathを探索
        for k in range(len(paths)):
            temp_score = 0.0

            temp_score = self.weight_angle * score_heading_angles[k] + \
                         self.weight_velo * score_heading_velos[k] + \
                         self.weight_obs * score_obstacles[k]

            if temp_score > score:
                opt_path = paths[k]
                score = temp_score

        return opt_path

    def _heading_angle(self, path, g_x, g_y): # ゴールに向いているか
        # 終端の向き
        last_x = path.x[-1]
        last_y = path.y[-1]
        last_th = path.th[-1]

        # 角度計算
        angle_to_goal = math.atan2(g_y-last_y, g_x-last_x)

        # score計算
        score_angle = angle_to_goal - last_th

        # ぐるぐる防止
        score_angle = abs(angle_range_corrector(score_angle))

        # 最大と最小をひっくり返す
        score_angle = math.pi - score_angle

        # print('score_sngle = {0}' .format(score_angle))

        return score_angle

    def _heading_velo(self, path): # 速く進んでいるか（直進）

        score_heading_velo = path.u_v

        return score_heading_velo

    def _calc_nearest_obs(self, state, obstacles):
        area_dis_to_obs = 5 # パラメータ（何メートル考慮するか，本当は制動距離）
        nearest_obs = [] # あるエリアに入ってる障害物

        for obs in obstacles:
            #obs[-1] .. 現在の障害物の位置
            temp_dis_to_obs = math.sqrt((state.x - obs.traj_obs[-1].x) ** 2 + (state.y - obs.traj_obs[-1].y) ** 2)

            if temp_dis_to_obs < area_dis_to_obs :
                nearest_obs.append(obs.traj_obs[-1])

        return nearest_obs

    def _obstacle(self, path, nearest_obs):
        # 障害物回避（エリアに入ったらその線は使わない）/ (障害物ともっとも近い距離距離)))
        score_obstacle = 2
        temp_dis_to_obs = 0.0

        for i in range(len(path.x)):
            for obs in nearest_obs: 
                temp_dis_to_obs = math.sqrt((path.x[i] - obs.x) * (path.x[i] - obs.x) + (path.y[i] - obs.y) *  (path.y[i] - obs.y))

                if temp_dis_to_obs < score_obstacle:
                    score_obstacle = temp_dis_to_obs # 一番近いところ

                # そもそも中に入ってる判定
                if temp_dis_to_obs < obs.size + 0.75: # マージン
                    score_obstacle = -float('inf')
                    break

            else:
                continue

            break

        return score_obstacle
