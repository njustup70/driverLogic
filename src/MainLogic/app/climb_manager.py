import asyncio
import math
from typing import NamedTuple

from MainLogic.core import ros_bridge_node as ros_bridge_module
from MainLogic.Lib.AsyncTools import AsyncVariable
from MainLogic.core.tf_manager import TFManagerInstance, move_to
from MainLogic.core.tf_manager import MoveControll

class ClimbManager:

    climb_type = AsyncVariable(0)
    climb_arm = AsyncVariable(0)

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

    forward_step = 0.1
    start_to_front_climb_distance = 0.40
    front_climb_to_back_climb_distance = 0.50
    back_climb_to_finish_distance = 0.35

    @staticmethod
    def _trans_type(a, b, c, d) -> int:
        return (d << 3) | (c << 2) | (b << 1) | a

    # def _trans_climb_type(self, climb_type_bytes: bytes) -> int:
    #     print(f"解析爬墙类型数据: {climb_type_bytes.hex()}")
    #     return climb_type_bytes[0] & 0x0F

    # def _trans_climb_arm(self, climb_arm_bytes: bytes) -> int:
    #     return climb_arm_bytes[0] & 0x05

    @staticmethod
    def _get_leg_encoding(front_height: int, rear_height: int) -> int:
        height_to_bits = {0: "00", 200: "01", 400: "10"}
        front_bits = height_to_bits.get(front_height, "00")
        rear_bits = height_to_bits.get(rear_height, "00")
        return int(rear_bits + front_bits, 2)

    async def _climb_forward(self, distance: float, climb_dir: float):
        current_odom = await TFManagerInstance.baseLinkOdom
        print("向前移动中...")
        await move_to(
            current_odom.x + (1 - abs(climb_dir)) * distance,
            current_odom.y + climb_dir * distance,
            current_odom.yaw,
            0.5
        )
        print("向前移动完成")

    async def _climb_stop(self):
        current_odom = await TFManagerInstance.baseLinkOdom
        print("停止移动...")
        await move_to(current_odom.x, current_odom.y, current_odom.yaw, 0.5)

    async def climb_move(self, type_num, distance: float, climb_dir: int):
        for _ in range(self.max_retries):
            this_type = await self.check_type()
            if this_type == type_num:
                MoveControll.stop()
                print("移动了指定距离，停止移动")
                break
            await self._climb_forward(self.forward_step, climb_dir)
            await asyncio.sleep(0.05)
        await self._climb_stop()
        print("停止移动完成")

    async def send_climb_command(self, data: bytes):
        for _ in range(self.max_retries//5):
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

        climb_height = next_place[2] - this_place[2]

        if abs(climb_height) not in [1, 2]:
            print(f"error:错误的攀爬要求, climb_height={climb_height}")
            return

        target_dir = [this_post[0] - next_post[0], this_post[1] - next_post[1]]

        if abs(target_dir[0] + target_dir[1]) != 1:
            print(f"error:错误的梅林目标要求, target_dir={target_dir}")
            return

        if target_dir[0] < 0:
            climb_dir = 0.0
        elif target_dir[1] > 0:
            climb_dir = 1.0
        elif target_dir[1] < 0:
            climb_dir = -1.0

        print(f"this_place={this_place}, next_place={next_place}, climb_height={climb_height}")

        return self.ClimbInstruction(
            this_place[0],
            this_place[1],
            next_place[0],
            next_place[1],
            climb_height,
            climb_dir
        )

    async def _climb_armup(self, height, front_back):
        assert ros_bridge_module.RosBridgeNodeInstance is not None

        height = abs(height)

        if height == 0:
            await self.send_climb_command(b'\xB1\x00')

        elif front_back == self.two_arms:
            await self.send_climb_command(b'\xB1\x05' if height == 1 else b'\xB1\x0A')

        elif front_back == self.just_back_arm:
            await self.send_climb_command(b'\xB1\x04' if height == 1 else b'\xB1\x08')

        elif front_back == self.just_front_arm:
            await self.send_climb_command(b'\xB1\x01' if height == 1 else b'\xB1\x02')

    async def climb_arm_act(self, height, front_back):
        await self._climb_armup(height, front_back)
        await asyncio.sleep(1)

    async def check_type(self):
        current_type = await ClimbManagerInstance.climb_type
        # print("climb内部回调收到：当前爬墙类型: {}".format(current_type))
        return current_type

    # async def check_arm_state(self):
    #     for _ in range(self.max_retries):
    #         current_arm_state = await ClimbManagerInstance.climb_arm

    #         if current_arm_state[0] == 0 and current_arm_state[1] == 0:
    #             return 0
    #         elif current_arm_state[0] == 0 and current_arm_state[1] == 1:
    #             return 1
    #         elif current_arm_state[0] == 2 and current_arm_state[1] == 0:
    #             return 2
    #         elif current_arm_state[0] == 2 and current_arm_state[1] == 2:
    #             return 3

    #         await asyncio.sleep(0.05)

    async def climb(self, this_post: list, next_post: list):
        climb_instruct = self.climb_find_grid(this_post, next_post)

        await move_to(climb_instruct.this_place_x, climb_instruct.this_place_y, 0.0)
        await move_to(climb_instruct.this_place_x,climb_instruct.this_place_y,climb_instruct.climb_dir * math.pi / 2)

        if climb_instruct.climb_height > 0:
            print("开始爬升，准备臂膀动作")
            await self.climb_arm_act(climb_instruct.climb_height, self.two_arms)
            print("双臂抬起，准备移动")
            await self.climb_move(self._trans_type(1, 0, 0, 0), self.start_to_front_climb_distance, climb_instruct.climb_dir)
            print("第一次移动完成，准备臂膀动作")
            await self.climb_arm_act(climb_instruct.climb_height, self.just_back_arm)
            print("前臂放下，准备移动")
            await self.climb_move(self._trans_type(1, 1, 1, 0), self.front_climb_to_back_climb_distance, climb_instruct.climb_dir)
            print("第二次移动完成，准备臂膀动作")
            await self.climb_arm_act(0, self.two_arms)
            print("后臂放下，准备移动")
            await self.climb_move(self._trans_type(1, 1, 1, 1), self.back_climb_to_finish_distance, climb_instruct.climb_dir)
            print("最后一次移动完成")
        else:
            print("开始下降，准备臂膀动作")
            await self.climb_move(self._trans_type(0, 1, 1, 1), self.start_to_front_climb_distance, climb_instruct.climb_dir)
            print("第一次移动完成，准备臂膀动作")
            await self.climb_arm_act(climb_instruct.climb_height, self.just_front_arm)
            print("前臂抬起，准备移动")
            await self.climb_move(self._trans_type(0, 0, 0, 1), self.front_climb_to_back_climb_distance, climb_instruct.climb_dir)
            print("第二次移动完成，准备臂膀动作")
            await self.climb_arm_act(climb_instruct.climb_height, self.two_arms)
            print("后臂抬起，准备移动")
            await self.climb_move(self._trans_type(1, 0, 0, 0), self.back_climb_to_finish_distance, climb_instruct.climb_dir)
            print("最后一次移动完成")
            await self.climb_arm_act(0, self.two_arms)
            print("双臂放下")
        await move_to(climb_instruct.next_place_x, climb_instruct.next_place_y, 0.0)


ClimbManagerInstance = ClimbManager()


async def climb(this_post: list, next_post: list):
    return await ClimbManagerInstance.climb(this_post, next_post)


async def climb_arm_act(height, front_back):
    return await ClimbManagerInstance.climb_arm_act(height, front_back)


async def climb_move(type, distance, direction):
    return await ClimbManagerInstance.climb_move(type, distance, direction)


async def check_types():
    return await ClimbManagerInstance.check_type()
