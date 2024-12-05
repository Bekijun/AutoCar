import os
import sys

# 현재 스크립트의 위치를 기준으로 상대 경로 설정
script_dir = os.path.dirname(os.path.realpath(__file__))
model1_path = os.path.join(script_dir, '../models/prius_hybrid_custom/model1.sdf')
model2_path = os.path.join(script_dir, '../models/prius_hybrid_custom/model2.sdf')

# 런치 파일에서 전달된 car 값 읽기
if len(sys.argv) < 2:
    print("Usage: spawn_car.py <car_id>")
    sys.exit(1)

car_id = sys.argv[1]

# car_id에 따라 스폰할 엔티티 결정
if car_id == "PR001":
    os.system(f"ros2 run gazebo_ros spawn_entity.py -file {model1_path} -entity PR001 -x 80 -y -11.7 -Y -1.57")
elif car_id == "PR002":
    os.system(f"ros2 run gazebo_ros spawn_entity.py -file {model2_path} -entity PR002 -x 93 -y -15.9 -Y -1.57")
else:
    print(f"Unknown car_id: {car_id}")
    sys.exit(1)