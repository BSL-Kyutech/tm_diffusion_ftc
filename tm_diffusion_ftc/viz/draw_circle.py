import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# CSVファイルの読み込みと変換
df_result = pd.read_csv('result_circle_path.csv').T
df_target = pd.read_csv('target_circle_path.csv').T

# numpy 配列に変換
result_x, result_y, result_z = df_result[0].to_numpy(), df_result[1].to_numpy(), df_result[2].to_numpy()
target_x, target_y, target_z = df_target[0].to_numpy(), df_target[1].to_numpy(), df_target[2].to_numpy()

# 描画領域の設定
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 2)

# 初期のプロット（空の線）
line_result, = ax.plot([], [], [], 'b-', label="Result")  # 青の線と点
line_target, = ax.plot([], [], [], 'r-', label="Target")  # 赤の線と点

for i in range(len(target_x)):
    line_target.set_data(target_x[:i], target_y[:i])
    line_target.set_3d_properties(target_z[:i])

ax.legend()
ax.view_init(elev=30, azim=-80)

# アニメーションの更新関数
def update(num):
    line_result.set_data(result_x[:num], result_y[:num])
    line_result.set_3d_properties(result_z[:num])
    
    return line_result, line_target

# アニメーションの作成
num_frames = len(result_x)  
ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=50, blit=True, repeat=False)
ani.save('result.mp4', writer="ffmpeg")

plt.show()