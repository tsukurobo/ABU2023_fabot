## 環境構築
# 1.tf2のインストール
sudo apt install ros-noetic-geometry2


# 2.ydlidarの環境構築
https://hackmd.io/@yuzuafro/ros-ydlidarx2
// 必要パッケージのインストール
sudo apt install cmake pkg-config
// YDLidar-SDK のビルド・インストール(任意のフォルダ)
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
// シリアルポートのエイリアス設定
chmod 777 src/ydlidar_ros_driver/startup/*
sudo sh src/ydlidar_ros_driver/startup/initenv.sh

# 3.urg(イーサネットタイプ)の環境構築
https://qiita.com/MMM-lab/items/e8beedf48d064dbc6a21

sudo apt install ros-noetic-urg-node
設定->ネットワーク->USB Ethernetで以下のように設定
・IPv4メソッド：手動
・アドレス：192.168.0.20
・ネットマスク：255.255.255.0
・ゲートウェイ：192.168.0.1
・DNS：192.168.0.1

