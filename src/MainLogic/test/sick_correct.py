from MainLogic.Lib.odomVec import Odom
import math
from scipy.optimize import fsolve
import numpy as np
def main():
    dyaw=5/180*math.pi
    baseyaw=-5/180*math.pi #slam座标系下面的旋转
    init_x,init_y=0.39,4.6
    #slam真实座标系
    map2init=Odom(init_x, init_y,dyaw)
    #在倾斜座标系下面的差别
    init2base=Odom(6,1.02,0)
    map2base=map2init@init2base
    #map座标系下观测的y
    y_correct=map2base.y/math.cos(map2base.yaw+baseyaw)
    y_left=(6.0-map2base.y)/math.cos(map2base.yaw)
    #slam设置座标系
    map2initset=Odom(init_x,init_y,0)
    map2baseset=map2initset@init2base
    #进行纠正
    # dyaw=math.atan2(map2baseset.y-y_correct,map2baseset.x)
    
    # print(dyaw*180/math.pi)
    dy=map2base.y-map2baseset.y
    dx=map2base.x-map2baseset.x

    print(f"dx:{dx},dy:{dy}")
    #左侧的yaw
    # dleft=min([fsolve(lambda theta: np.tan(theta) - (map2baseset.y - (6 / np.cos(theta) - y_left)) / map2baseset.x, guess)[0] for guess in (-0.5, 0.5)], key=abs) 
    # print(dleft*180/math.pi)
    # print(map2base)
    # yaw=fsolve(lambda theta:-y_correct+map2base.x*math.sin(theta)+map2base.y*math.cos(theta)+init_y,0)[0]
    yaw=fsolve(lambda theta:(-y_correct)*math.cos(theta+baseyaw)+init2base.x*math.sin(theta)+init2base.y*math.cos(theta)+init_y,0)[0]
    # yaw=fsolve(lambda theta:-y_correct+Odom(init_x,init_y,theta)@init2base,0)[0]
    print(yaw*180/math.pi)
if __name__ == "__main__":
    main()