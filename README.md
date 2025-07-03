# ROS2 Unity Simulator
Unity simulation environment for UniversityRoverChallenge2026. 　　　

We use [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) between ros2 and unity communication which allow native DDS communication.

## Platforms
WindowsとUbuntu22.04で動作確認済み。
| OS | Version |                              
|-|-| 
| Ubuntu | 22.04 |
| Unity | 2022.3.20f1 |
| ROS 2 | Humble |
| CPU | Intel Core i7-12700H|
| GPU | Intel Arc A550M |
| Memory | 16GB |

## Install Unity
windows,mac,ubuntuでそれぞれ必要なパッケージをインストール。基本的にはすべてチェックを付けてインストールで問題ない。
### [Install unity](https://unity.com/ja/releases/editor/whats-new/2022.3.20#notes)

## Open packages
### 1. Launch Unity
windows,macの場合はこのGithub内のwindows-mac内のares8_modelをunityでプロジェクトとして開くことでシミュレーションを起動できる。
![image](https://github.com/user-attachments/assets/787ef718-d6d9-4ed9-8ee1-cdeb5d700b45)

### 2. Open Project
ares8_modelを立ち上げた後は上のバーにある再生ボタンを押すことでシミュレーションを開始することができる。
![image](https://github.com/user-attachments/assets/ba89ae2c-f167-41ec-a1d9-8c821d6d0593)

シミュレーション開始時の画面。
![image](https://github.com/user-attachments/assets/38dcc74f-0cc9-4582-907b-219d81e82b27)

操作はキーボードで行い下のキーで操作することができる。
| keyboard | rover |                              
|-|-| 
| w | 前進 |
| a | 左旋回|
| s | 右旋回 |
| d | 後退 |












