import asyncio
import math
from typing import NamedTuple

from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.Lib.AsyncTools import async_property
from MainLogic.core.tf_manager import TFManagerInstance, move_to
from MainLogic.Lib.odomVec import Odom
from MainLogic.Lib.bytes import turn_to_bytes
class ClimbManager:
    
    climb_type = async_property(bytes)
    climb_arm = async_property(bytes)

    class ClimbInstruction(NamedTuple):
        this_place_x: float
        this_place_y: float
        next_place_x: float
        next_place_y: float
        climb_height: int
        climb_dir: float

    meilin_place = [2.50, 4.2]
    meilin_distance = [1.2, -1.2]
    meilin_height = [[0, 0, 0], [2, 1, 2], [3, 2, 1], [2, 3, 2], [2, 1, 2], [0, 0, 0]]
    max_retries = 100

    just_back_arm = 1
    just_front_arm = 2
    two_arms = 3

    first_type = 8
    second_type = 4
    third_type = 2
    forth_type = 1

    forward_step = 0.05
    start_to_front_climb_distance = 0.40
    front_climb_to_back_climb_distance = 0.50
    back_climb_to_finish_distance = 0.35

    @staticmethod
    def _trans_type(a,b,c,d) -> int:
        return (d<<3)|(c<<2)|(b<<1)|a
    def _trans_climb_type(self, climb_type_bytes: bytes) -> int:
        print(f"解析爬墙类型数据: {climb_type_bytes.hex()}")
        return climb_type_bytes[0] & 0x0F
    def _trans_climb_arm(self, climb_arm_bytes: bytes) -> int:
        # print(f"解析爬墙手臂数据: {climb_arm_bytes.hex()}")
        return climb_arm_bytes[0] & 0x05
    @staticmethod
    def _get_leg_encoding(front_height: int, rear_height: int) -> int:
        height_to_bits = {0: "00", 200: "01", 400: "10"}
        front_bits = height_to_bits.get(front_height, "00")
        rear_bits = height_to_bits.get(rear_height, "00")
        return int(rear_bits + front_bits, 2)

    async def _climb_forward(self,distance: float, climb_dir: float):
        current_odom = await TFManagerInstance.baseLinkOdom
        print("向前移动中...")
        await move_to(current_odom.x + (1 - abs(climb_dir))*distance, current_odom.y + climb_dir * distance, current_odom.yaw, 1.5)
        print("向前移动完成")

    async def _climb_stop(self):
        current_odom = await TFManagerInstance.baseLinkOdom
        print("停止移动...")
        await move_to(current_odom.x, current_odom.y, current_odom.yaw, 0.5)

    async def climb_move(self,type_num, distance: float, climb_dir: int):
        for i in range (self.max_retries):
            await self._climb_forward(self.forward_step, climb_dir)
            this_type = self._trans_climb_type(await ClimbManagerInstance.climb_type)
            if this_type == type_num:
                break
        await self._climb_stop()
        print("停止移动完成，检查标志位...")
    async def send_climb_command(self, data: bytes):
        for i in range(self.max_retries):
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(data)
            await asyncio.sleep(0.01)
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
        climb_height =  next_place[2] - this_place[2]
        if abs(climb_height) not in [1, 2]:
            print(f"error:错误的攀爬要求, climb_height={climb_height}")
            return
        target_dir = [this_post[0] - next_post[0], this_post[1] - next_post[1]]
        if abs(target_dir[0] + target_dir[1]) != 1:
            print(f"error:错误的梅林目标要求, target_dir={target_dir}")
            return
        if target_dir[0] < 0:
            climb_dir = 0.0
        if target_dir[1] > 0:
            climb_dir = 1.0
        if target_dir[1] < 0:
            climb_dir = -1.0
        print(f"this_place={this_place}, next_place={next_place}, climb_height={climb_height}")
        return self.ClimbInstruction(this_place[0], this_place[1], next_place[0], next_place[1], climb_height, climb_dir)

    async def _climb_armup(self, height, front_back):
        assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"
        height = abs(height)
        if height == 0:
            print("双腿回收")
            await self.send_climb_command(b'\xB1\x00')
        elif front_back == self.two_arms:
            if height == 1:
                print("抬升200")
                await self.send_climb_command(b'\xB1\x05')
            elif height == 2:
                print("抬升400")
                await self.send_climb_command(b'\xB1\x0A')
            else:
                print(f"error:错误的抬升指令, target_dir={height}")
        elif front_back == self.just_back_arm:
            if height == 1:
                print("仅后腿抬升200")
                await self.send_climb_command(b'\xB1\x04')
            elif height == 2:
                print("仅后腿抬升400")
                await self.send_climb_command(b'\xB1\x08')
            else:
                print(f"error:错误的抬升指令, target_dir={height}")
        elif front_back == self.just_front_arm:
            if height == 1:
                print("仅前腿抬升200")
                await self.send_climb_command(b'\xB1\x01')
            elif height == 2:
                print("仅前腿抬升400")
                await self.send_climb_command(b'\xB1\x02')
            else:
                print(f"error:错误的抬升指令, target_dir={height}")
                return
        else:
            print(f"error:错误的抬升指令")
            return

    async def climb_arm_act(self, height, front_back):
        await self._climb_armup(height, front_back)
        print("抬升检查中")
        await asyncio.sleep(1)
        print("抬升检查失败")
        return

    async def check_type(self):
        current_type = await ClimbManagerInstance.climb_type
        if not current_type or len(current_type) < 4:
            print("抬升标志位数据类型错误")
            return -1
        else:
            this_time_type = 0
            if current_type[0] == True:
                this_time_type += self.first_type
            if current_type[1] == True:
                this_time_type += self.second_type
            if current_type[2] == True:
                this_time_type += self.third_type
            if current_type[3] == True:
                this_time_type += self.forth_type
            print(f"当前抬升标志位: {current_type}, 编码为: {this_time_type}")
            return this_time_type

    async def check_arm_state(self):
        for i in range (self.max_retries):
            current_arm_state = await ClimbManagerInstance.climb_arm
            if current_arm_state[0] == 0 and current_arm_state[1] == 0:
                return 0
            elif current_arm_state[0] == 0 and current_arm_state[1] == 1:
                return 1
            elif current_arm_state[0] == 2 and current_arm_state[1] == 0:
                return 2
            elif current_arm_state[0] == 2 and current_arm_state[1] == 2:
                return 3
            elif current_arm_state[0] == 1 or current_arm_state[1] == 1:
                print("抬升中")
            await asyncio.sleep(0.05)
    async def climb(self, this_post: list, next_post: list):
        climb_instruct = self.climb_find_grid(this_post, next_post)
        await move_to(climb_instruct.this_place_x, climb_instruct.this_place_y, 0.0)
        await move_to(climb_instruct.this_place_x, climb_instruct.this_place_y, climb_instruct.climb_dir*3.14/2)
        await asyncio.sleep(2)
        print("到达攀爬起点，准备爬升")
        if climb_instruct.climb_height == 1 or climb_instruct.climb_height == 2:
            await self.climb_arm_act(climb_instruct.climb_height,self.two_arms)
            print("抬升完成，准备前进")
            await self.climb_move(self._trans_type(1,0,0,0),self.start_to_front_climb_distance,climb_instruct.climb_dir)
            print("前进中，等待标志位1激活")
            await self.climb_arm_act(climb_instruct.climb_height,self.just_back_arm)
            print("前腿放下，准备前进")
            await self.climb_move(self._trans_type(1,1,1,0),self.front_climb_to_back_climb_distance,climb_instruct.climb_dir)
            print("前进中，等待标志位123激活")
            await self.climb_arm_act(0,self.two_arms)
            print("双腿放下，调整位置")
            await self.climb_move(self._trans_type(1,1,1,1),self.back_climb_to_finish_distance,climb_instruct.climb_dir)
            print("前进中，等待标志位1234激活")
            await move_to(climb_instruct.next_place_x, climb_instruct.next_place_y, 0.0)
            print("爬完成！！！！！！")
        elif climb_instruct.climb_height == -1 or climb_instruct.climb_height == -2:
            print("下攀，准备前进")
            await self.climb_move(self._trans_type(0,1,1,1),self.start_to_front_climb_distance,climb_instruct.climb_dir)
            print("前进中，等待标志位1激活")
            await self.climb_arm_act(climb_instruct.climb_height,self.just_front_arm)
            print("前腿放下，准备前进")
            await self.climb_move(self._trans_type(0,0,0,1),self.front_climb_to_back_climb_distance,climb_instruct.climb_dir)
            print("前进中，等待标志位123激活")
            await self.climb_arm_act(climb_instruct.climb_height,self.two_arms)
            print("双腿放下，调整位置")
            await self.climb_move(self._trans_type(1,0,0,0),self.back_climb_to_finish_distance,climb_instruct.climb_dir)
            print("前进中，等待标志位1234激活")
            await self.climb_arm_act(0,self.two_arms)
            print("双腿放下，调整位置")
            await move_to(climb_instruct.next_place_x, climb_instruct.next_place_y, 0.0)
            print("爬完成！！！！！！")


ClimbManagerInstance = ClimbManager()

async def climb(this_post: list, next_post: list):
    return await ClimbManagerInstance.climb(this_post, next_post)
async def climb_arm_act(height, front_back):
    print("处罚动作")
    return await ClimbManagerInstance.climb_arm_act(height, front_back)
async def climb_move(type, distance, direction):
    return await ClimbManagerInstance.climb_move(type, distance, direction)
async def  climb_arm_act(height, front_back):
    return await ClimbManagerInstance.climb_arm_act(height, front_back)