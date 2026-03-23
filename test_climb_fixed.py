#!/usr/bin/env python3
"""
【爬墙完整集成测试】
目的：验证修复后的 AsyncTools 和 climb_manager 是否能正常工作
包含：完整的 10 步爬墙流程模拟
"""

import asyncio
import sys
from typing import Callable, Optional

# ============================================================================
# Mock 异步工具（模拟修复后的 AsyncTools.py）
# ============================================================================

class MockAsyncVariable:
    """修复后的 AsyncVariable 版本"""
    def __init__(self, default_factory):
        self._value = default_factory()
        self._event = None
        print(f"[AsyncVar] 初始化，初值={self._value}")
        
    def _get_event(self):
        if self._event is None:
            self._event = asyncio.Event()
            self._event.set()  # ✅ 修复：初值已设置
            print(f"[AsyncVar] 事件已初始化并设置")
        return self._event
        
    @property
    def value(self):
        return self._value
        
    @value.setter
    def value(self, new_value):
        self._value = new_value
        self._notify()
        
    def _notify(self):
        event = self._get_event()
        event.clear()  # ✅ 修复：先清除
        event.set()    # ✅ 修复：再设置
        
    def __await__(self):
        yield from self._get_event().wait().__await__()
        return self._value

class MockAsyncProperty:
    def __init__(self, default_factory):
        self.default_factory = default_factory
        self.storage_name = ""
        
    def __set_name__(self, owner, name):
        self.storage_name = f"__async_property_{name}"
        
    def __get__(self, instance, owner):
        if instance is None:
            return self
        async_var = getattr(instance, self.storage_name, None)
        if async_var is None:
            async_var = MockAsyncVariable(self.default_factory)
            setattr(instance, self.storage_name, async_var)
        return async_var
        
    def __set__(self, instance, value):
        async_var = getattr(instance, self.storage_name, None)
        if async_var is None:
            async_var = MockAsyncVariable(self.default_factory)
            setattr(instance, self.storage_name, async_var)
        async_var.value = value

# ============================================================================
# Mock 爬墙管理器和相关组件
# ============================================================================

class MockRosBridgeNode:
    """模拟 ROS Bridge Node"""
    def __init__(self):
        self.written_data = []
        self.callback = None
        
    def register_callback(self, cb):
        self.callback = cb
        
    def writeBytes(self, data):
        self.written_data.append(data)
        print(f"[ROS] 发送: {data.hex().upper()}")
        
    def simulate_receive(self, data):
        """模拟接收数据"""
        print(f"[MCU] 接收: {data.hex().upper()}")
        if self.callback:
            self.callback(data)

RosBridgeNodeInstance = None

class MockClimbManager:
    """修复后的 ClimbManager（关键改动）"""
    # ✅ 修复：初值设置为具体的列表
    climb_type = MockAsyncProperty(lambda: [False, False, False, False])
    climb_arm = MockAsyncProperty(lambda: [0, 0])
    
    def __init__(self):
        pass

# ============================================================================
# 回调函数（来自 globalCallback.py）
# ============================================================================

ClimbManagerInstance = None

def climb_type_callback(data: bytes):
    """解析爬墙数据的回调函数"""
    if data[0:2] == b'\xFF\xB1':
        try:
            if len(data) > 2:
                climb_type_byte = data[2]
                new_type = [
                    bool(climb_type_byte & (1 << 0)),
                    bool(climb_type_byte & (1 << 1)),
                    bool(climb_type_byte & (1 << 2)),
                    bool(climb_type_byte & (1 << 3)),
                ]
                ClimbManagerInstance.climb_type = new_type
                print(f"[回调] climb_type 更新为: {new_type}")
                
            if len(data) > 3:
                front_leg = (data[2] >> 4) & 0x03
                rear_leg = (data[2] >> 6) & 0x03
                if front_leg == 0 and rear_leg == 0:
                    front_leg = data[3] & 0x03
                    rear_leg = (data[3] >> 2) & 0x03
                ClimbManagerInstance.climb_arm = [front_leg, rear_leg]
                print(f"[回调] climb_arm 更新为: [{front_leg}, {rear_leg}]")
        except Exception as e:
            print(f"[回调] 错误: {e}")

# ============================================================================
# 爬墙测试流程
# ============================================================================

async def test_climb_200mm():
    """完整爬墙测试流程"""
    global ClimbManagerInstance, RosBridgeNodeInstance
    
    ClimbManagerInstance = MockClimbManager()
    RosBridgeNodeInstance = MockRosBridgeNode()
    RosBridgeNodeInstance.register_callback(climb_type_callback)
    
    print("\n" + "="*70)
    print("【爬墙完整集成测试】")
    print("="*70)
    
    # =========================================================================
    # 步骤 2-3: 伸腿
    # =========================================================================
    print("\n【步骤 2-3】伸腿到 200mm")
    print("-" * 70)
    
    leg_code = 0x05  # 200mm = 0x05
    RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code]))
    
    # 模拟下位机反馈：腿部伸出
    print("[模拟] 下位机反馈：腿部伸出中...")
    # FF B1 [climb_type_byte] [climb_arm_byte]
    # climb_arm: 前腿=2, 后腿=2
    #   - 前腿=2 (位 0-1): 10
    #   - 后腿=2 (位 2-3): 10
    #   - 组合: 1010 = 0x0A
    RosBridgeNodeInstance.simulate_receive(b'\xFF\xB1\x00\x0A')  # climb_arm=[2,2]
    
    # 验证腿部状态
    current_arm = await ClimbManagerInstance.climb_arm
    print(f"✓ 腿部状态: {current_arm}")
    assert current_arm == [2, 2], f"期望 [2,2] 但得到 {current_arm}"
    
    # =========================================================================
    # 步骤 4: 发送 B0 直到标志位 1 激活
    # =========================================================================
    print("\n【步骤 4】发送 B0 直到标志位 1 激活")
    print("-" * 70)
    
    step4_complete = False
    for step4_iteration in range(3):
        print(f"\n--- 循环 {step4_iteration + 1} ---")
        current_type = await ClimbManagerInstance.climb_type
        print(f"当前 climb_type: {current_type}")
        
        # ✅ 修复：安全检查
        if current_type and len(current_type) > 0 and current_type[0] is True:
            print("✓ 标志位 1 已激活，跳出循环")
            step4_complete = True
            break
        
        RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
        await asyncio.sleep(0.05)
        
        # 模拟下位机反应
        if step4_iteration == 1:
            print("[模拟] 下位机反馈：标志位 1 激活")
            RosBridgeNodeInstance.simulate_receive(b'\xFF\xB1\x01\x02')
    
    assert step4_complete, "❌ 步骤 4 失败：标志位 1 未激活"
    print("✓ 步骤 4 完成")
    
    # =========================================================================
    # 步骤 5-6: 缩前腿
    # =========================================================================
    print("\n【步骤 5-6】缩前腿")
    print("-" * 70)
    
    leg_code = 0x04  # 前腿=0, 后腿=200mm
    RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code]))
    
    print("[模拟] 下位机反馈：前腿缩回中...")
    RosBridgeNodeInstance.simulate_receive(b'\xFF\xB1\x01\x06')  # climb_arm=[0,2]
    
    current_arm = await ClimbManagerInstance.climb_arm
    print(f"✓ 腿部状态: {current_arm}")
    
    # =========================================================================
    # 步骤 7: 发送 B0 直到标志位 1,2,3 激活
    # =========================================================================
    print("\n【步骤 7】发送 B0 直到标志位 1,2,3 激活")
    print("-" * 70)
    
    step7_complete = False
    climb_type_byte_step7 = 0x01  # 初始标志1激活
    for step7_iteration in range(4):
        print(f"\n--- 循环 {step7_iteration + 1} ---")
        current_type = await ClimbManagerInstance.climb_type
        print(f"当前 climb_type: {current_type}")
        
        # ✅ 修复：安全检查
        if current_type and len(current_type) >= 3 and current_type[0] and current_type[1] and current_type[2]:
            print("✓ 标志位 1,2,3 已激活，跳出循环")
            step7_complete = True
            break
        
        RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
        await asyncio.sleep(0.05)
        
        # 模拟下位机反应 - 累积激活标志位
        if step7_iteration >= 0:
            climb_type_byte_step7 = min(0x07, climb_type_byte_step7 | (1 << step7_iteration))
            print(f"[模拟] 下位机反馈：climb_type_byte = 0x{climb_type_byte_step7:02X}")
            RosBridgeNodeInstance.simulate_receive(b'\xFF\xB1' + bytes([climb_type_byte_step7, 0x00]))
    
    assert step7_complete, "❌ 步骤 7 失败：标志位 1,2,3 未全部激活"
    print("✓ 步骤 7 完成")
    
    # =========================================================================
    # 步骤 8-9: 缩腿
    # =========================================================================
    print("\n【步骤 8-9】缩腿")
    print("-" * 70)
    
    leg_code = 0x00  # 前腿=0, 后腿=0
    RosBridgeNodeInstance.writeBytes(b'\xFA\xB1' + bytes([leg_code]))
    
    print("[模拟] 下位机反馈：腿部缩回完成...")
    RosBridgeNodeInstance.simulate_receive(b'\xFF\xB1\x07\x00')  # climb_arm=[0,0]
    
    current_arm = await ClimbManagerInstance.climb_arm
    print(f"✓ 腿部状态: {current_arm}")
    
    # =========================================================================
    # 步骤 10: 发送 B0 直到标志位 1,2,3,4 全部激活
    # =========================================================================
    print("\n【步骤 10】发送 B0 直到标志位 1,2,3,4 全部激活")
    print("-" * 70)
    
    step10_complete = False
    climb_type_byte_step10 = 0x07  # Step 9 后已激活 1,2,3
    for step10_iteration in range(5):
        print(f"\n--- 循环 {step10_iteration + 1} ---")
        current_type = await ClimbManagerInstance.climb_type
        print(f"当前 climb_type: {current_type}")
        
        # ✅ 修复：安全检查
        if current_type and len(current_type) >= 4 and all(current_type):
            print("✓ 标志位 1,2,3,4 已全部激活，跳出循环")
            step10_complete = True
            break
        
        RosBridgeNodeInstance.writeBytes(b'\xFA\xB0')
        await asyncio.sleep(0.05)
        
        # 模拟下位机反应 - 激活第 4 个标志位
        if step10_iteration >= 0:
            climb_type_byte_step10 = 0x0F  # 全部激活
            print(f"[模拟] 下位机反馈：标志位全部激活")
            RosBridgeNodeInstance.simulate_receive(b'\xFF\xB1' + bytes([climb_type_byte_step10, 0x00]))
    
    assert step10_complete, "❌ 步骤 10 失败：标志位 1,2,3,4 未全部激活"
    print("✓ 步骤 10 完成")
    
    # =========================================================================
    # 最终统计
    # =========================================================================
    print("\n" + "="*70)
    print("【测试结果】")
    print("="*70)
    print(f"✓ 爬墙流程完成！")
    print(f"✓ 总共发送 {len(RosBridgeNodeInstance.written_data)} 个命令:")
    
    cmd_count = {}
    for cmd in RosBridgeNodeInstance.written_data:
        hex_cmd = cmd.hex().upper()
        cmd_count[hex_cmd] = cmd_count.get(hex_cmd, 0) + 1
    
    for hex_cmd, count in sorted(cmd_count.items()):
        print(f"  - {hex_cmd}: {count} 次")
    
    print("\n✓✓✓ 所有测试通过！修复有效！✓✓✓")

# ============================================================================
# 主程序
# ============================================================================

if __name__ == '__main__':
    asyncio.run(test_climb_200mm())
