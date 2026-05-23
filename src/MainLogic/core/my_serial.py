"""_summary_串口异步读写库
@Author: LiuXuanze(Elaina-rascal)
@Date: 2024-12-29
@Description 使用方法:
1. 接收: AsyncSerial_t("COM2", 115200) 创建一个串口对象, 然后调用 register_callback() 开始监听串口数据;
   串口数据到来时会调用 callback 函数, 如果不传入 callback, 则可自行从队列处理。
2. 发送: write 函数用于向串口写入数据(阻塞函数)。

例程:
serial = AsyncSerial_t("COM2", 115200)
serial.register_callback(lambda data: serial.write(data))
"""

import asyncio
import serial_asyncio
import threading,time

class AsyncSerial_t:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.callback=None
        self.read_queue = asyncio.Queue()
        self.write_queue = asyncio.Queue()
        
        self.loop = None
        self.writer = None
        self.reader = None
        self.is_online = False  # 新增：状态标识
        
        self._thread = threading.Thread(target=self._run_thread, daemon=True)
        self._thread.start()
    def register_callback(self, callback):
        assert callable(callback), "回调必须是可调用对象"
        self.callback = callback
    def _run_thread(self):
        asyncio.run(self._main_logic())

    async def _main_logic(self):
        self.loop = asyncio.get_running_loop()
        
        # 启动核心任务，增加监视器任务
        await asyncio.gather(
            self._connection_monitor(), # 独立检测任务
            self._sender_loop(),
            self._receiver_loop(),
            self._parser_loop()
        )

    # --- 新增：独立检测与重连任务 ---

    async def _connection_monitor(self):
        """每隔 1 秒检查一次串口状态"""
        while True:
            if self.writer is None or self.writer.transport.is_closing():
                if self.is_online:
                    print(f"\033[91m[LOST] {self.port} 串口掉线！\033[0m")
                    self.is_online = False
                
                # 尝试重新连接
                try:
                    self.reader, self.writer = await serial_asyncio.open_serial_connection(
                        url=self.port, baudrate=self.baudrate
                    )
                    self.is_online = True
                    print(f"\033[92m[UP] {self.port} 串口已上线\033[0m")
                except Exception:
                    # 连接失败，静默等待下次尝试
                    pass
            await asyncio.sleep(1) # 检测频率

    # --- 修改：内部任务增加状态检查 ---

    async def _sender_loop(self):
        last_send_time = 0  # 记录上一次发送完成的时间戳
        MIN_INTERVAL = 0.001  # 最短间隔 1ms (单位：秒)

        while True:
            data = await self.write_queue.get()
            if self.is_online and self.writer:
                try:
                    # 1. 计算自上次发送以来的耗时
                    elapsed = time.perf_counter() - last_send_time
                    
                    # 2. 如果间隔小于 1ms，则异步等待补齐
                    if elapsed < MIN_INTERVAL:
                        await asyncio.sleep(MIN_INTERVAL - elapsed)
                    # 3. 执行发送
                    self.writer.write(data)
                    # 4. 等待数据进入系统底层缓冲区
                    await self.writer.drain()
                    # 5. 更新最后发送时间
                    # 注意：这里记录的是 drain 完成的时间，确保是“发完”后的 1ms 间隔
                    last_send_time = time.perf_counter()
                    
                except Exception as e:
                    self.is_online = False 
                    print(f"\033[91m[ERROR] 发送失败: {e}\033[0m")
                finally:
                    # 无论成功失败，都标记任务完成（防止 queue.join 挂起）
                    self.write_queue.task_done()
            else:
                self.write_queue.task_done()
                pass

    async def _receiver_loop(self):
        while True:
            if self.is_online and self.reader:
                try:
                    # 使用 wait_for 防止 read 在掉线时无限阻塞
                    bytes = await asyncio.wait_for(self.reader.read(4096), timeout=1.0)
                    if not bytes: 
                        raise ConnectionError("读取到空字节，连接可能关闭")
                    await self.read_queue.put(bytes)
                except (asyncio.TimeoutError, Exception):
                    # 读取异常通常意味着硬件问题
                    continue
            else:
                await asyncio.sleep(0.5)

    async def _parser_loop(self):
        buffer = bytearray() # 使用可变的 bytearray
        while True:
            chunk = await self.read_queue.get()
            buffer.extend(chunk) # 原位追加，效率远高于 +=

            while len(buffer) >= 2:
                if buffer[0] != 0xFF:
                    del buffer[0] # 原位删除第一个字节
                    continue
                
                length = buffer[1]
                total_frame_len = 2 + length
                
                if len(buffer) < total_frame_len:
                    break
                
                # 提取 payload
                payload = buffer[2:total_frame_len]
                
                if self.callback:
                    # 传入 callback 的数据依然建议转回 bytes 保证数据安全性
                    self.callback(b'\xFF' + payload)
                
                # 核心优化：使用 del 进行原位内存删除，避免创建新对象
                del buffer[:total_frame_len]
    def write(self, data: bytes):
        if self.loop:
            self.loop.call_soon_threadsafe(self.write_queue.put_nowait, data)
# --- 使用示例 ---
receive_cnt=0
def mcu_transmit_callback(data: bytes): # 0xAA
    """下位机串口数据帧回调（新协议：无帧头、无功能码）。"""
    # odom数据帧：3个float，共12字节
    if data[0:2]!=b'\xFF\xAA':
        return
    else:
        data=data[2:]
        print(data.hex())
        x, y, yaw = struct.unpack('<fff', data)
        print(f"ODOM数据解析成功: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
def my_data_callback(payload: bytes):
    """
    负责解析收到的纯数据部分
    根据类逻辑，传入的 payload 已经是 [0xFF][Len] 之后的部分了
    所以 payload 的第一个字节应该是我们自定义的 0xBB
    """
    global receive_cnt             
    # 检查我们自定义的次级头 0xBB，并确保长度足够（1字节头 + 8字节双精度）
    if len(payload) >= 9 and payload[0:2] == b'\xFF\xBB':
        try:
            # 提取 8 字节的时间戳部分（跳过 0xBB）
            timestamp_bytes = payload[2:10]
            sent_time = struct.unpack('>d', timestamp_bytes)[0]
            
            # 计算延迟
            now = time.time()
            delay_ms = (now - sent_time) * 1000
            
            print(f"\033[94m[RECV] 延迟: {delay_ms:.2f} ms | 原始时间戳: {sent_time:.6f}\033[0m")
            receive_cnt += 1
        except Exception as e:
            print(f"解析时间数据失败: {e}")
    else:
        print(f"收到非时间帧或格式错误: {payload.hex()}")

import time,struct
# --- 主函数示例 ---
if __name__ == "__main__":
    # 1. 初始化串口管理器
    ser = AsyncSerial_t("/dev/serial_ch340", 921600)

    # 2. 注册回调
    ser.register_callback(mcu_transmit_callback)
    while True:
        time.sleep(1)
        
    print("开始发送 [FF][09][BB][8字节时间戳] 帧...")

    try:
        cnt=0
        while True:
            # 3. 构造数据部分 (Payload)
            # 包含 1 字节 0xBB 和 8 字节时间戳
            current_time = time.time()
            timestamp_bin = struct.pack('>d', current_time)
            inner_payload = b'\xBB' + timestamp_bin
            
            # 4. 构造完整物理帧
            # 格式: 0xFF + 长度(9) + Payload
            frame_header = b'\xFF'
            frame_len = bytes([len(inner_payload)]) # 长度为 9
            full_frame = frame_header + frame_len + inner_payload
            
            # 5. 发送
            ser.write(full_frame)
            
            # print(f"[SEND] 发送长度: {len(inner_payload)} | 时间: {current_time:.4f}")
            
            time.sleep(0.003)
            cnt += 1
            if cnt >= 1000:
                break
        time.sleep(0.09)# 等待所有数据处理完毕
        print(f"\033[92m[SEND] 发送帧 {cnt} 接受帧 {receive_cnt} \033[0m")
        print(f"\033[92m[SEND] 发送帧 {cnt} 接受帧 {receive_cnt} \033[0m")
    except KeyboardInterrupt:
        print("\n程序手动停止")
