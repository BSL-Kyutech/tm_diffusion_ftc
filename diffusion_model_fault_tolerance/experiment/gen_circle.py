from __future__ import annotations

import time
from typing import List, Sequence, Tuple
import os
import numpy as np
import pandas as pd
import rclpy
from pathlib import Path
from rclpy.node import Node
from diffusion_model_fault_tolerance.trajectory.target_circle import generate_circle
from robotdiffusion.diffuser import Diffuser
from diffusion_model_fault_tolerance.io.pressure_publisher import PressurePublisher
from diffusion_model_fault_tolerance.io.mocap_subscriber import MocapSubscriber
from ament_index_python.packages import get_package_share_directory

def pressure40_to_list(output) -> List[int]:
    #output(torch.Tensor)を長さ40のlist[int]に変換する．
    arr = output.detach().cpu().numpy()
    arr = arr.reshape(-1)
    if arr.size != 40:
        raise ValueError(f"Expected 40 values from diffusion output, got {arr.size}")
    arr = np.clip(arr, 0, 255).astype(np.int32)
    return arr.tolist()

    # # diffusion_model.genの返り値を長さ40のlist[int]に整形する．
    # arr = np.asarray(output)
    # # arrを(40,)の形に整形
    # arr = arr.reshape(-1)
    # if arr.size != 40:
    #     raise ValueError(f"Expected 40 values from diffusion output, got {arr.size}")
    # # 圧力コマンドとして整数化
    # arr = np.clip(arr, 0, 255).astype(np.int32)
    # return arr.tolist()

def interp40(a: Sequence[int], b: Sequence[int], resolution: int) -> List[List[int]]:
    # aからbを線形補完した列を40ch列で返す．
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    mids: List[List[int]] = []
    for j in range(1, resolution):
        w = j / resolution
        m = (1 - w) * a + w * b
        mids.append(np.round(m).astype(np.int32).tolist())
    return mids

def main(): 
    # ROSの初期化&Node準備
    rclpy.init()
    node = Node("gen_circle_experiment")
    
    # Pub&Sub
    pub = PressurePublisher()
    mocap = MocapSubscriber(node)
    
    # パスの設定（ROS2のinstall環境でも確実）
    PKG_NAME = "diffusion_model_fault_tolerance"  # ← package.xml の <name> と一致させる

    model_dir = Path(get_package_share_directory(PKG_NAME)) / "model"
    print(f"model_dir = {model_dir}")

    # モデルと目標軌道の提示
    diffusion_model = Diffuser()
    diffusion_model.load_dir(model_dir)
    X, Y, Z = generate_circle()
    print(f"x, y, z lengths: {len(X)}, {len(Y)}, {len(Z)}")
    # 推論の実行
    output_list:List[List[int]] = []
    for x, y, z in zip(X, Y, Z):
        output = diffusion_model.gen({
        '/mocap/rigidbody1/pos0':x,
        '/mocap/rigidbody1/pos1':y,
        '/mocap/rigidbody1/pos2':z
        },{
        },initial_control_input={'/tenpa/pressure/desired0/pressure0':150,
                                 '/tenpa/pressure/desired0/pressure1':150,
                                 '/tenpa/pressure/desired0/pressure2':150,
                                 '/tenpa/pressure/desired0/pressure3':150,
                                 '/tenpa/pressure/desired0/pressure4':150,
                                 '/tenpa/pressure/desired0/pressure5':150,
                                 '/tenpa/pressure/desired0/pressure6':150,
                                 '/tenpa/pressure/desired0/pressure7':150,
                                 '/tenpa/pressure/desired0/pressure8':150,
                                 '/tenpa/pressure/desired0/pressure9':150,})
        print(f"output_shape: {output.shape}")
        # outputを長さ40のlist[int]に変換
        output = pressure40_to_list(output)
        output_list.append(output)
    print(f"output_list_shape: {len(output_list)}")   
    
    # 実験の実行(publish & mocap受信)
    sleep_time = 0.005
    first_wait = 2.0
    interp_resolution = 100
    
    result_x: List[float] = []
    result_y: List[float] = []
    result_z: List[float] = []
    
    # 受信町の最大時間
    receive_time_sec = 0.2
    
    def wait_mocap_once(timeout_sec: float) -> Tuple[float, float, float] | None:
        # mocapから1サンプル受信するまでspin_onceで待つ．
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.01)
            if getattr(mocap, "has_received")() is True:
                msg = getattr(mocap, "consume_latest")()
                if msg is None:
                    continue
                # PointStamped を想定
                return float(msg.point.x), float(msg.point.y), float(msg.point.z)
        return None
    
    for i,u40 in enumerate(output_list):
        
        # 目標点をiとしてpublish
        pub.publish40(u40)
        
        # 最初だけ少し待つ
        time.sleep(first_wait if i == 0 else sleep_time)
        
        # mocapの受信（1回分）
        got = wait_mocap_once(receive_time_sec)
        if got is not None:
            rx,ry,rz = got
            result_x.append(rx)
            result_y.append(ry)
            result_z.append(rz)
        else:
            # 受信できなくても実験を止めない（必要なら raise に変更）
            node.get_logger().warn("No mocap message received within timeout.")
            result_x.append(float("nan"))
            result_y.append(float("nan"))
            result_z.append(float("nan"))
        
        # 次の点があるなら補間して滑らかに送る
        if i + 1 < len(output_list):
            for mid in interp40(u40, output_list[i + 1], interp_resolution):
                pub.publish40(mid)
                time.sleep(sleep_time)
    
    # 終了時にマニピュレータを立たせる処理
    pub.publish40([150] * 40)
    time.sleep(0.5)
    pub.publish40([200] * 40)
    time.sleep(0.5)
    pub.publish40([250] * 40)
    time.sleep(0.5)
    
    # csvファイルに保存
    target_df = pd.DataFrame({"x": X, "y": Y, "z": Z})
    result_df = pd.DataFrame({"x": result_x, "y": result_y, "z": result_z})

    target_df.to_csv("target_circle_path.csv", index=False)
    result_df.to_csv("result_circle_path.csv", index=False)

    node.get_logger().info("Saved target_circle_path.csv / result_circle_path.csv")
    
    # nodeの終了処理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "main":
    main()
    
            
