'''
全局回调函数串口接收回调和ros2话题回调
'''
from app.actions import SpearHeadInstance, SpearBuildInstance, QRRecogInstance


def example_serial_callback(data: bytes):
    #示例函数
    #检查第一位 非常重要
    if data[0] != 0xAA:
        print(f"Received serial data: {data}")

def serial_action_return_callback(data: bytes):
    if data[0] == 0xFF and data[1] == 0xFF: # 后面根据帧头改
        return_statu = data[3]
        if return_statu == 0x00:
            print("Action executed successfully!")
            SpearHeadInstance.take_spearhead_ok.value = True
            SpearBuildInstance.build_spear_ok.value = True

STATUS_MAP = {"空": "00", "R1": "01", "R2": "10", "假": "11"}
REVERSE_MAP = {v: k for k, v in STATUS_MAP.items()}
def ros_qr_callback(msg):
    hex_str = msg.data

    if not hex_str or len(hex_str) != 8:
        return 
    try:
        binary = bin(int(hex_str, 16))[2:].zfill(32)
        state_bits = binary[:24]
        
        states = []
        for i in range(0, 24, 2):
            bits = state_bits[i:i+2]
            states.append(REVERSE_MAP.get(bits, "未知"))
        
        QRRecogInstance.recog_qr_result.value = ", ".join(states)
        return
    except:
        return 