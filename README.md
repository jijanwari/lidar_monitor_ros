# lidar_monitor_ros
rosでlidar搭載亀ロボットを移動制御して、lidarのデータ取得するデモ

# 1. git clone

```
git clone https://github.com/jijanwari/lidar_monitor_ros.git
cd lidar_monitor_ros
```
```
export ROS_WS_ROOT="$PWD"
```
```
mkdir $ROS_WS_ROOT/src
cd $ROS_WS_ROOT/src
```

# 2. lidar_monitor_pkg のビルド

## 2.1. パッケージの作成

```
ros2 pkg create --build-type ament_cmake --dependencies rclcpp sensor_msgs --node-name lidar_monitor lidar_monitor_pkg
```
依存関係 --dependencies を最初に入れると後が楽です

--node-name は最初に自動で作られる実行ファイルの名前です

package.xml を開き、以下の箇所を探して修正してください。
```
<maintainer email="jijanwari">nsai</maintainer>
→	<maintainer email="jijanwari@example.com">nsai</maintainer>
```

## 2.2. ワークスペースのトップへ移動
```
cd $ROS_WS_ROOT
```

## 2.3. (.bahsrcに入れている場合は実行不要) ROS 2 本体の環境読み込み (※[distro]は jazzy や humble など)
```
source /opt/ros/jazzy/setup.bash
```

## 2.4. 1回目のビルド (まず土台を作る)して、このワークスペース専用の環境読み込み
新しいパッケージを作成した直後は、必ず１回実行必要
```
colcon build --packages-select lidar_monitor_pkg
```
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select lidar_monitor_pkg
```
```
source install/setup.bash
```
(source install/setup.bashを行うことで、この後実行するコマンドが このパッケージを認識します)

## 2.5. lidar_monitor.cpp を置き換える
```
cp srcbkup/lidar_monitor_pkg/src/lidar_monitor.cpp src/lidar_monitor_pkg/src/lidar_monitor.cpp
```

## 2.6. 本ビルド、このワークスペース専用の環境再読み込み
```
colcon build --packages-select lidar_monitor_pkg
```
```
source install/setup.bash
```
新しいパッケージを作成した直後の初回ビルド後は　必ず source install/setup.bash を１回実行<br><br>

---

# 3. avoidance_pkg のビルド

事前準備　ワークスペースへ移動して環境変数を設定して、パッケージルートsrcへ移動
```
cd lidar_monitor_ros
```
```
export ROS_WS_ROOT="$PWD"
```
```
cd $ROS_WS_ROOT/src
```

## 3.1. パッケージの作成
```
ros2 pkg create --build-type ament_cmake --dependencies rclcpp sensor_msgs geometry_msgs --node-name avoidance_node avoidance_pkg
```
依存関係 --dependencies を最初に入れると後が楽です

--node-name は最初に自動で作られる実行ファイルの名前です

package.xml を開き、以下の箇所を探して修正してください。
```
<maintainer email="jijanwari">nsai</maintainer>
→	<maintainer email="jijanwari@example.com">nsai</maintainer>
```

## 3.2. ワークスペースのトップへ移動
```
cd $ROS_WS_ROOT
```

## 3.3. (.bahsrcに入れている場合は実行不要) ROS 2 本体の環境読み込み (※[distro]は jazzy や humble など)
```
source /opt/ros/jazzy/setup.bash
```

## 3.4. 1回目のビルド (まず土台を作る)して、このワークスペース専用の環境読み込み
新しいパッケージを作成した直後は、必ず１回実行必要
```
colcon build --packages-select avoidance_pkg
source install/setup.bash
```
(source install/setup.bashを行うことで、この後実行するコマンドが このパッケージを認識します)

## 3.5. avoidance_node.cpp を置き換える
```
cp srcbkup/avoidance_pkg/src/avoidance_node.cpp src/avoidance_pkg/src/avoidance_node.cpp
```

## 3.6. 本ビルド、このワークスペース専用の環境再読み込み
```
colcon build --packages-select avoidance_pkg
source install/setup.bash
```
新しいパッケージを作成した直後の初回ビルド後は　必ず source install/setup.bash を１回実行<br><br>


---

# 4. アプリの起動

## 4.1. gazebo起動

```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r $ROS_WS_ROOT/sdf/lidar4_visual_lider_box_cylinder.sdf"
```

## 4.2. LiDARデータ受信ノード起動
新しく起動したターミナルの場合、以下の通り ワークスペースへ移動して環境変数を設定する
```
cd lidar_monitor_ros
export ROS_WS_ROOT="$PWD"
```

ROS 2への「橋」を架ける（ターミナル2）
GazeboのLiDARデータをROS 2が読める形式に変換します。
```
ros2 run ros_gz_bridge parameter_bridge "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" --ros-args -r /lidar:=/scan &
```
ノード起動
```
ros2 run lidar_monitor_pkg lidar_monitor
```
	
## 4.3. 移動制御ノード起動
新しく起動したターミナルの場合、以下の通り ワークスペースへ移動して環境変数を設定する
```
cd lidar_monitor_ros
export ROS_WS_ROOT="$PWD"
source $ROS_WS_ROOT/install/setup.bash
```
大事なポイント：速度用の「橋」
Gazeboに速度を伝えるために、もう一本ブリッジを起動するのを忘れないでください（別ターミナル）。
```
ros2 run ros_gz_bridge parameter_bridge "/model/model_with_lidar/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" &
```
ノード起動
```
ros2 run avoidance_pkg avoidance_node
```
	
キーボードで動かす場合は上の手順は行わず、以下のコマンドを実行すること
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/model_with_lidar/cmd_vel
```
	
## 4.4. RVIZでLiDAR点群表示
以下の通りブリッジを２つ作成
```
ros2 run ros_gz_bridge parameter_bridge "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" --ros-args -r /lidar:=/scan &
ros2 run ros_gz_bridge parameter_bridge /model/model_with_lidar/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V &
```
RVIZ起動
```
ros2 run rviz2 rviz2
```

以下の操作を実施すると、LiDARの点群が赤い点で表示される

	左下の [Add] ボタン → [By topic] タブ → /scan を選択。
	左側のパネルの [Global Options] -> [Fixed Frame] を、
		model_with_lidar/lidar_link/gpu_lidar	に書き換える（または選択肢から選ぶ）。
		
RVizでの点のサイズ変更手順

	Displaysパネルを確認 左側の「Displays」プロパティの中から、LiDARを表示している 「LaserScan」（または自分で追加したTopic名）の項目を探し、左側の小さな三角マークをクリックして展開します。
	Size (m): デフォルトでは 0.01（1cm）などの非常に小さい値になっていることが多いです。この数値を 0.05 や 0.1 などに大きくすると、一つ一つの点が見やすくなります。









