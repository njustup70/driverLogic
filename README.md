<!--
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2024-11-14 22:46:41
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2024-11-24 19:29:49
 * @FilePath: \docker\ros2\README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
<div align="center">
<h1>ros2子模块<h1>
</div>

## 版本与发布记录
~~Elaina_v0.1~~

| 功能包                                 | 介绍         |
| -------------------------------------- | ------------ |
| [my_driver](./src/my_driver/README.md) | ros2硬件驱动 |
| [rc_bringup](src/rc_bringup/README.md) | 综合启动     |
## 1.在第一次打开项目的时候先运行自定义udev规则(注意其中的sudo权限)
```bash
ros2_ws$ sudo ./udev_init.bash
```
- 需要在主机下运行
