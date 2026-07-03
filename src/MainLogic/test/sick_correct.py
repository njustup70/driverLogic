from MainLogic.Lib.odomVec import Odom
import math
from scipy.optimize import fsolve
import numpy as np
def main():
    init_yaw=0.0*math.pi
    dyaw=5/180*math.pi #旋转误差
    baseyaw=-5/180*math.pi #slam座标系下面的旋转
    init_x,init_y=0.39,4.6
    #slam真实座标系
    map2init_real=Odom(init_x, init_y,dyaw+init_yaw) #带有初始值误差的真实起点
    map2init_set=Odom(init_x,init_y,init_yaw) #代码设置的起点
    # map2baseset=map2initset@init2base
    #在地图座标系下面真实值
    map2base_real=Odom(6.2,2,0.1*math.pi) #真实的移动

    init2base_obs=map2init_real.inverse()@map2base_real #用真实的起点和真实的移动计算出来的观测值
    init2base_real=map2init_set.inverse()@map2base_real #用代码设置的起点和真实的移动计算出来的slam观测值
    y_real=map2base_real.y
    x_real=map2base_real.x
    y_cal=(map2init_set@init2base_obs)
    print(f"odom真实值{init2base_real}\nslam观测值{init2base_obs}")
    dx=init2base_real.x-init2base_obs.x
    dy=init2base_real.y-init2base_obs.y
    print(f"dx:{dx},dy:{dy}")
    
    #slam设置座标系
    yaw=fsolve(lambda theta:-y_real+init2base_obs.x*math.sin(theta+init_yaw)+init2base_obs.y*math.cos(theta+init_yaw)+init_y,0)[0]
    yawx=fsolve(lambda theta:-x_real-init2base_obs.y*math.sin(theta+init_yaw)+init2base_obs.x*math.cos(theta+init_yaw)+init_x,0)[0]
    #进行纠正
    print(yaw*180/math.pi)
    print(yawx*180/math.pi)
    map2init_correct=Odom(init_x,init_y,init_yaw+yaw) #纠正后的起点
    init2base_correct=map2init_correct.inverse()@map2base_real #纠正后的移动
    map2base_correct=map2init_correct@init2base_correct #纠正后的slam移动
    print(f"纠正后坐标{map2base_correct}")
if __name__ == "__main__":
    main()