import serial
import serial.tools.list_ports
import socket
import threading
import sys
import select
import time
from datetime import datetime
from pathlib import Path
from colorama import init, Fore, Style

# 初始化 Colorama 以支持 Windows 终端颜色
init(autoreset=True)

# ================= 配置区域 =================
# 默认串口配置
DEFAULT_BAUD_RATE = 921600
FRAME_START_BYTE = ord("[")
SERIAL_REOPEN_DELAY_SEC = 0.05
SERIAL_REOPEN_COOLDOWN_SEC = 0.3
JUSTFLOAT_TAIL = b"\x00\x00\x80\x7f"
TRACK_FIREWATER_HEAD = b"[v]"
TRACK_JUSTFLOAT_HEAD = b"[j]"
PORTS_NAME="/cp2102"
# 是否显示每一行接收到的原始数据 (调试开关)，可通过终端动态配置
DEBUG_SHOW_RAW_LINE = False

# 是否将识别到的日志自动写入 ./logs/y_m_d_h_m_s_log.txt
ENABLE_FILE_LOGGING = True

# 转发给 VOFA+ 的 TCP 端口 (VOFA+ 选 TCP Client 连接此端口)
TCP_HOST = "0.0.0.0"
TCP_PORT = 8888

# 根据 bsp_log.cpp 定义的日志前缀与颜色映射
# 只有匹配到以下前缀的文本，才被认为是日志在终端截留着色，否则一律当作波形转给 VOFA+
# 您可以在这里面自定义需要截获的前缀和您期望设定的颜色
LOG_STYLES = {
    "[Error] ": Fore.RED + Style.BRIGHT,
    "[Warn] ": Fore.YELLOW + Style.BRIGHT,
    "[Well] ": Fore.GREEN + Style.BRIGHT,
    "[Note] ": Fore.MAGENTA + Style.BRIGHT,  # 对应 STM32 的 Purple
    "[Respond] ": Fore.CYAN + Style.BRIGHT,  # 对应 STM32 的 Blue
    "[Info] ": Fore.WHITE + Style.BRIGHT,  # 例子：添加对 Info 的白字拦截
}

# ===========================================


class DataForwarder:
    def __init__(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket = None
        self.running = True
        self.vofa_notice_printed = False

    def start_server(self):
        """启动 TCP 服务器等待 VOFA+ 连接"""
        try:
            self.server_socket.bind((TCP_HOST, TCP_PORT))
            self.server_socket.listen(1)
            print(
                f"{Fore.CYAN}[System] TCP Server listening on port {TCP_PORT}. Connect VOFA+ via TCP Client mode."
            )
        except Exception as e:
            print(f"{Fore.RED}[System] Failed to bind TCP port: {e}")
            return

        while self.running:
            try:
                # 阻塞等待连接
                client, addr = self.server_socket.accept()
                print(f"{Fore.GREEN}[System] VOFA+ Connected from {addr}")
                self.client_socket = client
                self.vofa_notice_printed = False

                # 保持连接直到断开
                while self.running:
                    try:
                        # select(read_list, write_list, error_list, timeout)
                        # timeout=0 表示非阻塞，立即返回结果
                        # r 列表如果不为空，说明 client_socket 有数据（或断开信号）来了
                        r, _, _ = select.select([self.client_socket], [], [], 0)

                        if r:
                            # 只有当 select 说“有动静”时，才去 peek 数据
                            # 这样就不需要 MSG_DONTWAIT 标志了
                            data = self.client_socket.recv(16, socket.MSG_PEEK)
                            if data == b"":
                                # 读到空数据意味着对端（VOFA+）关闭了连接
                                raise ConnectionResetError
                    except (ConnectionResetError, OSError):
                        break
                    time.sleep(0.5)

                print(f"{Fore.YELLOW}[System] VOFA+ Disconnected")
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
            except Exception as e:
                if self.running:
                    print(f"{Fore.RED}[System] TCP Error: {e}")

    def send_waveform(self, data_bytes, announce=False):
        """转发数据给 VOFA+"""
        if DEBUG_SHOW_RAW_LINE:
            print(f"[{Fore.BLUE}DEBUG-WAVE{Fore.RESET}] Forwarding: {repr(data_bytes)}")

        if self.client_socket:
            try:
                if announce and not self.vofa_notice_printed:
                    print(
                        f"{Fore.BLUE}{Style.DIM}[vofa] 正在转发Track数据到端口127.0.0.1 {TCP_PORT}{Fore.RESET}",
                        flush=True,
                    )
                    self.vofa_notice_printed = True
                self.client_socket.sendall(data_bytes)
            except:
                # 发送失败通常意味着连接断开了，主循环会处理
                pass

    def stop(self):
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def get_serial_config():
    """默认自动配置；仅在 -p 模式下进入交互配置。"""
    # ports = list_serial_ports()
    # if not ports:
    #     print(f"{Fore.RED}No serial ports found!")
    #     return None

    # interactive_mode = "-p" in sys.argv[1:]

    # if not interactive_mode:
    return PORTS_NAME, DEFAULT_BAUD_RATE, DEBUG_SHOW_RAW_LINE

    print("Available Ports:")
    for i, p in enumerate(ports):
        print(f"{i}: {p}")

    try:
        idx = int(input("Select Port Index: "))
        port_name = ports[idx]
        baud_rate = int(input(f"Baudrate [{DEFAULT_BAUD_RATE}]: ") or str(DEFAULT_BAUD_RATE))

        debug_input = input("Show raw/debug binary data? (y/N): ").strip().lower()
        debug_show_raw_line = debug_input == "y"
        return port_name, baud_rate, debug_show_raw_line

    except:
        print("Invalid selection.")
        return None


def main():
    global DEBUG_SHOW_RAW_LINE

    serial_config = get_serial_config()
    if serial_config is None:
        return

    port_name, baud_rate, DEBUG_SHOW_RAW_LINE = serial_config

    log_file = None
    if ENABLE_FILE_LOGGING:
        try:
            logs_dir = Path("./logs")
            logs_dir.mkdir(parents=True, exist_ok=True)
            log_filename = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_log.txt")
            log_path = logs_dir / log_filename
            log_file = log_path.open("a", encoding="utf-8", buffering=1)
            print(f"{Fore.CYAN}[System] File logging enabled: {log_path}")
        except Exception as e:
            log_file = None
            print(f"{Fore.YELLOW}[System] File logging disabled: {e}")

    if "-p" not in sys.argv[1:]:
        print(
            f"{Fore.CYAN}[System] Auto selected {port_name} @ {baud_rate}, raw/debug output: {'ON' if DEBUG_SHOW_RAW_LINE else 'OFF'}"
        )

    # 启动 TCP 转发线程
    forwarder = DataForwarder()
    tcp_thread = threading.Thread(target=forwarder.start_server, daemon=True)
    tcp_thread.start()

    # 打开串口并开始处理
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=0.1)
        print(f"{Fore.GREEN}[System] Opened {port_name} @ {baud_rate}")
        print(
            f"{Fore.GREEN}[System] Logs will appear here. Waveforms forwarded to TCP :{TCP_PORT}"
        )
        print("-" * 40)

        buffer = bytearray()
        last_reopen_time = 0.0
        # 预先将所有的前缀转换为 bytes，以优化判断速度
        # 注意：这里只能做“完整前缀”匹配，不能只看首字节 '['。
        # VOFA+ 的 JustFloat 帧也以 '[' 开头，若放宽成首字节命中，就会被误判成日志。
        prefixes_bytes = [p.encode("utf-8") for p in LOG_STYLES.keys()]
        shortest_log_prefix_len = min(len(p) for p in prefixes_bytes)

        def could_be_log(buf):
            # 判断 buf 是否是彩色日志的完整前缀，或仍可能增长为一个完整前缀。
            # 串口 read() 没有消息边界，例如 "[Respond] " 可以被分成
            # "[Respond]" 和 " " 两次读到。若只接受完整前缀，较长的
            # 日志头会在读到末尾空格前被误判为乱码并触发重启。
            for p in prefixes_bytes:
                if p.startswith(buf) or buf.startswith(p):
                    return True
            return False

        def drop_garbled_and_resync(garbled_bytes):
            # 乱码提示只展示前 80 字节，避免刷屏
            preview = repr(garbled_bytes[:80])
            if len(garbled_bytes) > 80:
                preview += "..."
            print(
                f"{Fore.YELLOW}[System] 丢弃乱码（{preview}），已恢复脚本。",
                flush=True,
            )

        def quick_restart_serial():
            nonlocal ser, last_reopen_time
            now = time.time()
            if now - last_reopen_time < SERIAL_REOPEN_COOLDOWN_SEC:
                return
            last_reopen_time = now

            print(f"{Fore.YELLOW}[System] 检测到帧错误，快速重启串口中...", flush=True)
            try:
                if ser.is_open:
                    ser.close()
            except:
                pass

            time.sleep(SERIAL_REOPEN_DELAY_SEC)
            try:
                ser = serial.Serial(port_name, baud_rate, timeout=0.1)
                print(
                    f"{Fore.GREEN}[System] 串口已恢复: {port_name} @ {baud_rate}",
                    flush=True,
                )
            except serial.SerialException as e:
                print(f"{Fore.RED}[System] 串口重启失败: {e}", flush=True)

        while True:
            # 使用 read 获取可用数据，避免 readline 在无 '\n' 的纯二进制流上阻塞
            # in_waiting 是当前串口缓冲区已接收的字节数
            data = ser.read(max(1, ser.in_waiting))
            if not data:
                continue

            buffer.extend(data)

            while buffer:
                if could_be_log(buffer):
                    # 如果匹配了日志的前缀特征，则尝试寻找 '\n' 来截取完整的一行
                    nl_idx = buffer.find(b"\n")
                    if nl_idx != -1:
                        # 找到了换行符
                        # 向后探查是否紧跟着多个 \r 或 \n (处理用户额外的换行排版)
                        end_idx = nl_idx + 1
                        extra_newlines = bytearray()
                        while end_idx < len(buffer) and buffer[end_idx] in (10, 13):
                            extra_newlines.append(buffer[end_idx])
                            end_idx += 1

                        line_bytes = bytes(buffer[:end_idx])
                        del buffer[:end_idx]

                        try:
                            line_str = line_bytes.decode("utf-8", errors="ignore")
                            is_log = False
                            matched_color = Fore.WHITE

                            for prefix, color in LOG_STYLES.items():
                                if line_str.startswith(prefix):
                                    is_log = True
                                    matched_color = color
                                    break

                            if DEBUG_SHOW_RAW_LINE:
                                print(
                                    f"[{Fore.BLUE}DEBUG{Fore.RESET}] Got line: {repr(line_str)}"
                                )

                            if is_log:
                                # ==== 路径 1: 显示彩色日志 ====
                                # 去掉尾部空白符，仅将颜色重置放在文本末，以防终端干扰，并原样输出其包含的换行排版
                                stripped_line = line_str.rstrip()
                                trailing_chars = line_str[len(stripped_line) :]
                                print(
                                    f"{matched_color}{stripped_line}{Fore.RESET}{trailing_chars}",
                                    end="",
                                    flush=True,
                                )
                                if log_file:
                                    log_file.write(line_str)
                                    log_file.flush()

                                # 主动避免粘包: 多截取的尾部换行极低概率可能会混入波形的有效有效帧 (0x0A/0x0D)。
                                # 一旦错杀没发给波形会导致帧错位死机。
                                # 而从 BspLog 发出的附加换行转发给波形也是安全合规的间隙数据（Just Float忽略这部分间隙）
                                # 因此采取“主动抄送副本”的策略。
                                if extra_newlines:
                                    forwarder.send_waveform(bytes(extra_newlines))
                            else:
                                # ==== 路径 2: 转发未匹配特征的数据 ====
                                forwarder.send_waveform(line_bytes)
                        except Exception as e:
                            print(f"{Fore.RED}[Error parsing]: {e}")
                    else:
                        # 没有找到换行符，但前缀有可能合法，等待后续数据
                        # 避免异常长数据（如含前缀但无换行符的二进制）无限堆积内存，设个2048字节阈值
                        if len(buffer) > 2048:
                            garbled = bytes(buffer)
                            buffer.clear()
                            drop_garbled_and_resync(garbled)
                            quick_restart_serial()
                        else:
                            break  # 跳出内层缓冲区处理，继续从串口读取新数据
                elif buffer.startswith(TRACK_FIREWATER_HEAD):
                    # [v] + FireWater/ASCII Track 数据
                    nl_idx = buffer.find(b"\n", len(TRACK_FIREWATER_HEAD))
                    if nl_idx != -1:
                        end_idx = nl_idx + 1
                        extra_newlines = bytearray()
                        while end_idx < len(buffer) and buffer[end_idx] in (10, 13):
                            extra_newlines.append(buffer[end_idx])
                            end_idx += 1

                        line_bytes = bytes(buffer[:end_idx])
                        del buffer[:end_idx]

                        try:
                            line_str = line_bytes.decode("utf-8", errors="ignore")
                            if DEBUG_SHOW_RAW_LINE:
                                print(
                                    f"[{Fore.BLUE}DEBUG{Fore.RESET}] Got [v] line: {repr(line_str)}"
                                )

                            forwarder.send_waveform(
                                line_bytes[len(TRACK_FIREWATER_HEAD):], announce=True
                            )
                            if extra_newlines:
                                forwarder.send_waveform(bytes(extra_newlines))
                        except Exception as e:
                            print(f"{Fore.RED}[Error parsing]: {e}")
                    else:
                        if len(buffer) > 2048:
                            garbled = bytes(buffer)
                            buffer.clear()
                            drop_garbled_and_resync(garbled)
                            quick_restart_serial()
                        else:
                            break
                elif buffer.startswith(TRACK_JUSTFLOAT_HEAD):
                    # [j] + JustFloat Track 数据
                    payload = buffer[len(TRACK_JUSTFLOAT_HEAD):]
                    tail_idx = payload.find(JUSTFLOAT_TAIL, 1)
                    if tail_idx != -1:
                        frame_end = tail_idx + len(JUSTFLOAT_TAIL)
                        if frame_end > 1:
                            if DEBUG_SHOW_RAW_LINE:
                                print(
                                    f"[{Fore.BLUE}DEBUG{Fore.RESET}] Got [j] frame: {frame_end} bytes"
                                )
                            forwarder.send_waveform(bytes(payload[:frame_end]), announce=True)
                        del buffer[:len(TRACK_JUSTFLOAT_HEAD) + frame_end]
                        continue

                    if len(buffer) > 96:
                        garbled = bytes(buffer)
                        buffer.clear()
                        drop_garbled_and_resync(garbled)
                        quick_restart_serial()
                    else:
                        break
                else:
                    # 这里只保留真正的噪声处理：
                    # 1) 不是 '[' 开头的字节，先跳到下一个 '['
                    # 2) '[' 开头但不是已定义协议头，认为是乱码
                    if buffer[0] != FRAME_START_BYTE:
                        next_start = buffer.find(b"[")
                        if next_start == -1:
                            garbled = bytes(buffer)
                            buffer.clear()
                        else:
                            garbled = bytes(buffer[:next_start])
                            del buffer[:next_start]
                        drop_garbled_and_resync(garbled)
                        quick_restart_serial()
                        continue

                    if len(buffer) < max(len(TRACK_JUSTFLOAT_HEAD), shortest_log_prefix_len):
                        break

                    garbled = bytes(buffer[:1])
                    del buffer[:1]
                    drop_garbled_and_resync(garbled)
                    quick_restart_serial()

    except serial.SerialException as e:
        print(f"{Fore.RED}Serial Error: {e}")
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Stopping...")
    finally:
        forwarder.stop()
        if log_file:
            log_file.close()
        if "ser" in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
