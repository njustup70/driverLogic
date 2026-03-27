import asyncio
import math

from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.Lib.AsyncTools import async_property
from MainLogic.core.tf_manager import TFManagerInstance, move_to
from MainLogic.Lib.odomVec import Odom
from MainLogic.Lib.bytes import turn_to_bytes
from MainLogic.core.tf_manager import TFManager

class ClimbManager:
    # ✅ 修复：初值设置为具体的列表而非类型构造函数
    climb_type = async_property(lambda: [False, False, False, False])
    climb_arm = async_property(lambda: [0, 0])  # 0=缩回, 1=调整中, 2=到位

    meilin_place = [2.4, 4.2]
    meilin_distance = [1.2, -1.2]
    meilin_height = [[0, 0, 0], [1, 2, 3], [2, 3, 2], [1, 2, 3], [2, 1, 2], [0, 0, 0]]

    @staticmethod
    def _get_leg_encoding(front_height: int, rear_height: int) -> int:
        """将腿部高度(0/200/400)编码为 1 字节。"""
        height_to_bits = {0: "00", 200: "01", 400: "10"}
        front_bits = height_to_bits.get(front_height, "00")
        rear_bits = height_to_bits.get(rear_height, "00")
        return int(rear_bits + front_bits, 2)
    def _climb_forward(self):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xBB' + turn_to_bytes([0.6, 0.0, 0.0]))
    def _climb_stop(self):
        ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xBB' + turn_to_bytes([0.0, 0.0, 0.0]))
    def climb_find_grid(self, this_post: list, next_post: list):
        this_place = [
            this_post[0] * self.meilin_distance[0] + self.meilin_place[0],
            this_post[1] * self.meilin_distance[1] + self.meilin_place[1],
            self.meilin_height[this_post[0]][this_post[1]],
        ]
        next_place = [
            next_post[0] * self.meilin_distance[0] + self.meilin_place[0],
            next_post[1] * self.meilin_distance[1] + self.meilin_place[1],
            self.meilin_height[next_post[0]][next_post[1]],
        ]
        climb_height = this_place[2] - next_place[2]
        if abs(climb_height) not in [1, 2]:
            print(f"error:错误的攀爬要求, climb_height={climb_height}")
            return

        target_dir = [this_post[0] - next_post[0], this_post[1] - next_post[1]]
        if abs(target_dir[0] + target_dir[1]) != 1:
            print(f"error:错误的梅林目标要求, target_dir={target_dir}")
            return
        return [this_place[0], this_place[1], next_place[0], next_place[1], climb_height*200]
    def climb_armup(self, height, front_back):
        if height == 0:#回收
            print("双腿回收")
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xB1\x00')
        elif front_back == 3:
            if height == 1:
                print("抬升200")
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xB1\x05')
            elif height == 2:
                print("抬升400")
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xB1\x0A')
            else:
                print(f"error:错误的抬升指令, target_dir={height*200}")
        elif front_back == 1:#仅后腿
            if height == 1:
                print("仅后腿抬升200")
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xB1\x01')
            elif height == 2:
                print("仅后腿抬升400")
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xB1\x02')
            else:
                print(f"error:错误的抬升指令, target_dir={height*200}")
        elif front_back == 2:#仅前腿
            if height == 1:
                print("仅前腿抬升200")
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xB1\x04')
            elif height == 2:
                print("仅前腿抬升400")
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xB1\x08')
            else:
                print(f"error:错误的抬升指令, target_dir={height*200}")
                return
        else:
            print(f"error:错误的抬升指令")
            return
        for i in range (100):
            if self.check_arm_state() == front_back:
                print("抬升完成！！")
                return
        print("抬升检查失败")
        return
    async def check_type(self):
        current_type = await ClimbManagerInstance.climb_type
        if not current_type and not len(current_type) >= 3:
            print("抬升标志位数据类型错误")
            return
        else:#从车体前到后位数下降
            this_time_type = 0
            if current_type[0] == True:
                this_time_type += 8
            if current_type[1] == True:
                this_time_type += 4
            if current_type[2] == True:
                this_time_type += 2
            if current_type[3] == True:
                this_time_type += 1
            return this_time_type
    async def check_arm_state(self):
         max_retries = 100
         for i in range (max_retries):
            current_arm_state = await ClimbManagerInstance.climb_arm
            if current_arm_state[0] == 0 and current_arm_state[1] == 0:
                print("爬升机构放下")
                return 0
            elif current_arm_state[0] == 0 and current_arm_state[1] == 1:
                print("爬升机构后腿抬起前腿放下")
                return 1
            elif current_arm_state[0] == 2 and current_arm_state[1] == 0:
                print("爬升机构前腿抬起后腿放下")
                return 2
            elif current_arm_state[0] == 2 and current_arm_state[1] == 2:
                print("爬升机构抬起")
                return 3
            elif current_arm_state[0] == 1 or current_arm_state[1] == 1:
                print("抬升中")    
            await asyncio.sleep(0.05)
    async def climb(self, this_post: list, next_post: list):
        max_retries = 100
        climb_instruct = self.climb_find_grid(this_post, next_post)
        await move_to(climb_instruct[0], climb_instruct[1], 0.0)
        self.climb_armup(climb_instruct[4],3) 
        for i in range (max_retries):
            self._climb_forward()
            if await self.check_type() == 8:
                print("标志位1已激活")
                break
            print("直线前进")
            await asyncio.sleep(0.01)
        self._climb_stop()
        self.climb_armup(climb_instruct[4],2) 
        for i in range (max_retries):
            self._climb_forward()
            if await self.check_type() == 14:
                print("标志位123已激活")
                break
            print("直线前进")
            await asyncio.sleep(0.01)
        self._climb_stop()
        self.climb_armup(climb_instruct[4],2) 
        for i in range (max_retries):
            self._climb_forward()
            if await self.check_type() == 15:
                print("标志位1234已激活")
                break
            print("直线前进")
            await asyncio.sleep(0.01)
        self._climb_stop()
        self.climb_armup(0,3) 
        print("爬完成！！！！！！")
   

ClimbManagerInstance = ClimbManager()


async def climb(this_post: list, next_post: list):
    return await ClimbManagerInstance.climb(this_post, next_post)
