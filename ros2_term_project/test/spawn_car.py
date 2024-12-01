import os
import sys

# 현재 스크립트의 위치를 기준으로 상대 경로 설정
script_dir = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(script_dir, '../models/prius_hybrid_custom/model.sdf')

# Gazebo 명령 실행
os.system(f"ros2 run gazebo_ros spawn_entity.py -file {model_path} -entity PR001 -x 93 -y -11.7 -Y -1.57")
os.system(f"ros2 run gazebo_ros spawn_entity.py -file {model_path} -entity PR002 -x 93 -y -15.9 -Y -1.57")