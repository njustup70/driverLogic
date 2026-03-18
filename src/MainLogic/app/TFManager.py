'''
坐标管理类
'''
import asyncio
from Lib.odomVec import Odom
from Lib.bytes import turn_to_bytes
import Lib.rosBridgeNode as ros_bridge_module
from Lib.AsyncTools import async_property
import math
class TFManager:
    baseLinkOdom = async_property(Odom)
    climb_type = async_property(list[bool])
    climb_arm = async_property(list[int])
async def move_to(x, y, yaw):
    targetOdom = Odom(x, y, yaw)
    #给电控发坐标指令
    assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
    ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
    #rosBridgeNode.writeBytes(b'\xA1' + list_turn_to_bytes([x, y, yaw]))
    #发送指令代码还没有
    while True:
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xA1' + turn_to_bytes([x, y, yaw]))
        # 等待baseLinkOdom更新
        current_odom = await TFManagerInstance.baseLinkOdom
        dx = targetOdom - current_odom
        # 距离小于1cm且角度误差小于0.05rad就认为到达目标了
        if dx.dist < 0.01 and abs(dx.yaw) < 0.05:
            print("Arrived at target!")
            break

from Lib.bytes import turn_to_bytes


meilin_place = [0.0, 0.0]
meilin_distance = [1.2, -1.2]
meilin_height = [[0, 0, 0], [1, 2, 3], [2, 3, 2], [1, 2, 3], [2, 1, 2], [0, 0, 0]]
async def climb(this_post: list, next_post: list):  # 输入: [x, y] - 梅林网格坐标
    """
    梅林爬墙控制函数 - 基于 climb_height 的动态腿部控制
    
    参数:
        this_post: [x, y] - 当前梅林网格位置 (0-5, 0-2)
        next_post: [x, y] - 目标梅林网格位置 (0-5, 0-2)
    
    腿部高度映射:
        climb_height = 1 → 腿部目标高度 200 (编码01)
        climb_height = 2 → 腿部目标高度 400 (编码10)
    
    控制流程:
        1. 计算climb_height
        2. 设置前后腿均为目标高度
        3. 等待腿部到位（climb_arm均为2）
        4. 发送B0直到标志位1激活
        5. 前腿调至0，后腿保持目标高度
        6. 等待腿部调整完成（climb_arm不为1）
        7. 发送B0直到标志位1,2,3全部激活
        8. 前后腿均调至0
        9. 等待腿部调整完成（climb_arm不为1）
        10. 发送B0直到标志位1,2,3,4全部激活
    
    输出: None
    """
    # 步骤 1: 计算climb_height
    this_place = [this_post[0] * meilin_distance[0] + meilin_place[0], 
                  this_post[1] * meilin_distance[1] + meilin_place[1],
                  meilin_height[this_post[0]][this_post[1]]]
    next_place = [next_post[0] * meilin_distance[0] + meilin_place[0], 
                  next_post[1] * meilin_distance[1] + meilin_place[1],
                  meilin_height[next_post[0]][next_post[1]]]
    climb_height = this_place[2] - next_place[2]
    
    if abs(climb_height) not in [1, 2]:
        print(f"error:错误的攀爬要求, climb_height={climb_height}")
        return
    
    target_dir = [this_post[0] - next_post[0], this_post[1] - next_post[1]]
    if abs(target_dir[0] + target_dir[1]) != 1:
        print(f"error:错误的梅林目标要求, target_dir={target_dir}")
        return
    
    # 移动到目标位置
    await move_to(this_place[0], this_place[1], -math.atan2(target_dir[1], target_dir[0]))
    
    # 腿部高度编码函数
    def get_leg_encoding(front_height, rear_height):
        """将腿部高度(0/200/400)编码为十六进制"""
        height_to_bits = {0: "00", 200: "01", 400: "10"}
        front_bits = height_to_bits.get(front_height, "00")
        rear_bits = height_to_bits.get(rear_height, "00")
        combined = rear_bits + front_bits
        return int(combined, 2)
    
    # 根据climb_height确定目标腿部高度
    # climb_height=1 → 200, climb_height=2 → 400
    target_leg_height = 200 if climb_height == 1 else 400
    
    if climb_height > 0:
        # 步骤 2: 设置前后腿均为目标高度
        leg_code_stage1 = get_leg_encoding(target_leg_height, target_leg_height)
        print(f"步骤2: 设置前后腿均为{target_leg_height} → 发送 FA B1 {leg_code_stage1:02X}")
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code_stage1]))
        await asyncio.sleep(0.1)
        
        # 步骤 3: 等待腿部到位（climb_arm均为2）
        print("步骤3: 等待腿部到位...")
        max_retries = 100
        for retry in range(max_retries):
            current_arm = await TFManagerInstance.climb_arm
            if current_arm[0] == 2 and current_arm[1] == 2:
                print(f"✓ 腿部到位 [前腿={current_arm[0]}, 后腿={current_arm[1]}]")
                break
            await asyncio.sleep(0.05)
        
        # 步骤 4: 发送B0直到标志位1激活
        print("步骤4: 发送B0直到标志位1激活...")
        max_retries = 200
        for retry in range(max_retries):
            current_type = await TFManagerInstance.climb_type
            if current_type[0] == True:
                print(f"✓ 标志位1已激活")
                break
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
            await asyncio.sleep(0.05)
        
        # 步骤 5: 前腿调至0，后腿保持目标高度
        leg_code_stage2 = get_leg_encoding(0, target_leg_height)
        print(f"步骤5: 前腿调至0，后腿保持{target_leg_height} → 发送 FA B1 {leg_code_stage2:02X}")
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code_stage2]))
        await asyncio.sleep(0.1)
        
        # 步骤 6: 等待腿部调整完成（climb_arm不为1）& 重新校准朝向
        print("步骤6: 等待腿部调整完成...")
        max_retries = 100
        for retry in range(max_retries):
            current_arm = await TFManagerInstance.climb_arm
            if current_arm[0] != 1 and current_arm[1] != 1:
                print(f"✓ 腿部调整完成 [前腿={current_arm[0]}, 后腿={current_arm[1]}]")
                break
            await asyncio.sleep(0.05)
        
        # 重新校准朝向（仅调整方向，不移动位置）
        current_odom = await TFManagerInstance.baseLinkOdom
        print(f"步骤6b: 重新校准朝向到 {-math.atan2(target_dir[1], target_dir[0]):.2f} rad")
        await move_to(current_odom.x, current_odom.y, -math.atan2(target_dir[1], target_dir[0]))
        # 步骤 7: 发送B0直到标志位1,2,3全部激活
        print("步骤7: 发送B0直到标志位1,2,3全部激活...")
        max_retries = 200
        for retry in range(max_retries):
            current_type = await TFManagerInstance.climb_type
            if current_type[0] == True and current_type[1] == True and current_type[2] == True:
                print(f"✓ 标志位1,2,3已激活")
                break
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
            await asyncio.sleep(0.05)
        
        # 步骤 8: 前后腿均调至0
        leg_code_stage3 = get_leg_encoding(0, 0)
        print(f"步骤8: 前后腿均调至0 → 发送 FA B1 {leg_code_stage3:02X}")
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code_stage3]))
        await asyncio.sleep(0.1)
        
        # 步骤 9: 等待腿部调整完成（climb_arm不为1）
        print("步骤9: 等待腿部调整完成...")
        max_retries = 100
        for retry in range(max_retries):
            current_arm = await TFManagerInstance.climb_arm
            if current_arm[0] != 1 and current_arm[1] != 1:
                print(f"✓ 腿部调整完成 [前腿={current_arm[0]}, 后腿={current_arm[1]}]")
                break
            await asyncio.sleep(0.05)
        
        # 步骤 10: 发送B0直到标志位1,2,3,4全部激活
        print("步骤10: 发送B0直到标志位1,2,3,4全部激活...")
        max_retries = 200
        for retry in range(max_retries):
            current_type = await TFManagerInstance.climb_type
            if all(current_type):  # 所有标志位均为True
                print(f"✓ 标志位1,2,3,4已全部激活")
                break
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
            await asyncio.sleep(0.05)
        
        print("✓ 爬墙流程完成！")




TFManagerInstance = TFManager()
