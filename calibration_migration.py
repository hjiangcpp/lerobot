#!/usr/bin/env python3

import json
import argparse
from pathlib import Path
from typing import Dict, Any
from src.lerobot.robots.so101_follower import SO101Follower
from src.lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from src.lerobot.motors.motors_bus import MotorCalibration

def load_calibration_file(file_path: str) -> Dict[str, MotorCalibration]:
    """加载标定文件"""
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    calibration = {}
    for motor_name, cal_data in data.items():
        calibration[motor_name] = MotorCalibration(
            id=cal_data['id'],
            drive_mode=cal_data['drive_mode'],
            homing_offset=cal_data['homing_offset'],
            range_min=cal_data['range_min'],
            range_max=cal_data['range_max']
        )
    
    return calibration


def migrate_calibration(source_file: str, target_robot_id: str, target_port: str):
    """迁移标定数据"""
    
    # 1. 加载源标定文件
    print(f"加载源标定文件: {source_file}")
    source_calibration = load_calibration_file(source_file)
    
    # 2. 创建目标机器人配置
    target_config = SO101FollowerConfig(
        port=target_port,
        id=target_robot_id
    )
    
    # 3. 创建目标机器人实例
    target_robot = SO101Follower(target_config)
    target_calibration_path = target_robot.calibration_fpath
    
    print(f"目标标定文件路径: {target_calibration_path}")
    
    
    # 4. 将标定数据写入电机
    print("连接机器人并应用标定数据...")
    try:
        target_robot.connect(calibrate=False)  # 不重新标定
    
        # 显示差异
        current_cal = target_robot.bus.read_calibration()
        
        target_robot.bus.write_calibration(source_calibration)
    
        target_robot.disconnect()
        
    except Exception as e:
        print(f"❌ 连接机器人时出错: {e}")
        print("标定文件已复制，但未应用到电机")
    
    print("✅ 标定数据迁移完成")

def main():
    parser = argparse.ArgumentParser(description="标定数据迁移工具")
    parser.add_argument("--source", required=True, help="源标定文件路径")
    parser.add_argument("--target-id", required=True, help="目标机器人ID")
    parser.add_argument("--target-port", required=True, help="目标机器人端口")
    
    args = parser.parse_args()
   
    # 迁移模式
    migrate_calibration(
        source_file=args.source,
        target_robot_id=args.target_id,
        target_port=args.target_port,
    )

if __name__ == "__main__":
    main() 