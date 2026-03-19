import asyncio
import math

from MainLogic.Lib import rosBridgeNode as ros_bridge_module
from MainLogic.Lib.AsyncTools import async_property
from MainLogic.app.TFManager import TFManagerInstance, move_to


class ClimbManager:
    climb_type = async_property(list[bool])
    climb_arm = async_property(list[int])

    meilin_place = [0.0, 0.0]
    meilin_distance = [1.2, -1.2]
    meilin_height = [[0, 0, 0], [1, 2, 3], [2, 3, 2], [1, 2, 3], [2, 1, 2], [0, 0, 0]]

    @staticmethod
    def _get_leg_encoding(front_height: int, rear_height: int) -> int:
        """将腿部高度(0/200/400)编码为 1 字节。"""
        height_to_bits = {0: "00", 200: "01", 400: "10"}
        front_bits = height_to_bits.get(front_height, "00")
        rear_bits = height_to_bits.get(rear_height, "00")
        return int(rear_bits + front_bits, 2)

    async def climb(self, this_post: list, next_post: list):
        """梅林爬墙控制主流程。"""
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

        assert ros_bridge_module.RosBridgeNodeInstance is not None, "RosBridgeNodeInstance is not initialized yet!"

        await move_to(this_place[0], this_place[1], -math.atan2(target_dir[1], target_dir[0]))

        target_leg_height = 200 if climb_height == 1 else 400

        if climb_height > 0:
            leg_code_stage1 = self._get_leg_encoding(target_leg_height, target_leg_height)
            print(f"步骤2: 设置前后腿均为{target_leg_height} -> 发送 FA B1 {leg_code_stage1:02X}")
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code_stage1]))
            await asyncio.sleep(0.1)

            print("步骤3: 等待腿部到位...")
            max_retries = 100
            for _ in range(max_retries):
                current_arm = await ClimbManagerInstance.climb_arm
                if current_arm[0] == 2 and current_arm[1] == 2:
                    print(f"\u2713 腿部到位 [前腿={current_arm[0]}, 后腿={current_arm[1]}]")
                    break
                await asyncio.sleep(0.05)

            print("步骤4: 发送B0直到标志位1激活...")
            max_retries = 200
            for _ in range(max_retries):
                current_type = await ClimbManagerInstance.climb_type
                if current_type[0] is True:
                    print("\u2713 标志位1已激活")
                    break
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
                await asyncio.sleep(0.05)

            leg_code_stage2 = self._get_leg_encoding(0, target_leg_height)
            print(f"步骤5: 前腿调至0，后腿保持{target_leg_height} -> 发送 FA B1 {leg_code_stage2:02X}")
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code_stage2]))
            await asyncio.sleep(0.1)

            print("步骤6: 等待腿部调整完成...")
            max_retries = 100
            for _ in range(max_retries):
                current_arm = await ClimbManagerInstance.climb_arm
                if current_arm[0] != 1 and current_arm[1] != 1:
                    print(f"\u2713 腿部调整完成 [前腿={current_arm[0]}, 后腿={current_arm[1]}]")
                    break
                await asyncio.sleep(0.05)

            current_odom = await TFManagerInstance.baseLinkOdom
            target_yaw = -math.atan2(target_dir[1], target_dir[0])
            print(f"步骤6b: 重新校准朝向到 {target_yaw:.2f} rad")
            await move_to(current_odom.x, current_odom.y, target_yaw)

            print("步骤7: 发送B0直到标志位1,2,3全部激活...")
            max_retries = 200
            for _ in range(max_retries):
                current_type = await ClimbManagerInstance.climb_type
                if current_type[0] and current_type[1] and current_type[2]:
                    print("\u2713 标志位1,2,3已激活")
                    break
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
                await asyncio.sleep(0.05)

            leg_code_stage3 = self._get_leg_encoding(0, 0)
            print(f"步骤8: 前后腿均调至0 -> 发送 FA B1 {leg_code_stage3:02X}")
            ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code_stage3]))
            await asyncio.sleep(0.1)

            print("步骤9: 等待腿部调整完成...")
            max_retries = 100
            for _ in range(max_retries):
                current_arm = await ClimbManagerInstance.climb_arm
                if current_arm[0] != 1 and current_arm[1] != 1:
                    print(f"\u2713 腿部调整完成 [前腿={current_arm[0]}, 后腿={current_arm[1]}]")
                    break
                await asyncio.sleep(0.05)

            print("步骤10: 发送B0直到标志位1,2,3,4全部激活...")
            max_retries = 200
            for _ in range(max_retries):
                current_type = await ClimbManagerInstance.climb_type
                if all(current_type):
                    print("\u2713 标志位1,2,3,4已全部激活")
                    break
                ros_bridge_module.RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
                await asyncio.sleep(0.05)

            print("\u2713 爬墙流程完成！")


ClimbManagerInstance = ClimbManager()


async def climb(this_post: list, next_post: list):
    return await ClimbManagerInstance.climb(this_post, next_post)
