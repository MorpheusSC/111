# IsaacSim 相机链路修复说明

## 问题描述
在仿真模式下使用 X200 机器人进行遥操作时，头显端可以连接到服务器，但显示黑屏，无法看到相机画面。

## 问题原因
1. 仿真模式下，相机数据应该来自 IsaacSim 的共享内存，而不是物理相机
2. `image_server` 默认尝试从物理相机（/dev/video0）读取数据
3. 在仿真环境中没有物理相机设备，导致服务器无法发布图像数据
4. `teleop_hand_and_arm.py` 没有在仿真模式下启动 IsaacSim 图像服务器

## 修复内容

### 1. 修改 `teleop/teleimager/src/teleimager/image_server.py`
- 添加 `--isaacsim` 命令行参数
- 在 IsaacSim 模式下，自动强制所有相机类型为 `isaacsim`
- 添加等待时间，确保共享内存已准备好

### 2. 修改 `teleop/teleop/teleop_hand_and_arm.py`
- 在仿真模式下自动启动 IsaacSim 图像服务器
- 在程序退出时自动清理图像服务器进程
- 添加必要的导入和进程管理代码

## 使用方法

### 正常仿真模式测试
使用原有的测试命令即可：
```bash
python teleop/teleop_hand_and_arm.py --arm=X200 --ee=inspire_dfx --sim --record --img-server-ip=172.18.60.81
```

修复后的系统会自动：
1. 启动 IsaacSim 图像服务器
2. 从 IsaacSim 共享内存读取相机数据
3. 通过 ZMQ 将图像数据发送给头显端

### 验证修复
可以运行测试脚本验证修复是否正常工作：
```bash
python test_isaacsim_fix.py
```

## 技术细节

### IsaacSim 相机数据流
```
IsaacSim (仿真环境)
    ↓ 共享内存
image_server (--isaacsim)
    ↓ ZMQ/WebRTC
ImageClient (teleop_hand_and_arm.py)
    ↓
TeleVuer
    ↓
头显端 (VR 设备)
```

### 关键代码变更
1. `image_server.py` 添加 IsaacSim 模式支持
2. `teleop_hand_and_arm.py` 自动启动 IsaacSim 服务器
3. 强制配置文件中的相机类型为 `isaacsim`

## 故障排除

### 问题1：仍然显示黑屏
**可能原因：**
- IsaacSim 仿真环境没有启动或没有启用相机
- 共享内存未正确初始化

**解决方法：**
1. 确保 IsaacSim 仿真环境已启动并启用了相机 (`--enable_cameras`)
2. 检查仿真环境是否在运行正确的任务（如 X200 相关任务）
3. 查看日志中的相机初始化信息

### 问题2：连接超时
**可能原因：**
- 图像服务器启动失败
- 网络配置问题

**解决方法：**
1. 检查日志中的 "Starting IsaacSim image server" 消息
2. 确认 `img-server-ip` 参数正确
3. 检查防火墙设置

### 问题3：相机帧率过低
**可能原因：**
- IsaacSim 渲染性能不足
- 网络带宽限制

**解决方法：**
1. 降低 IsaacSim 的渲染质量或分辨率
2. 减少相机数量
3. 检查网络连接质量

## 相关文件
- `teleop/teleimager/src/teleimager/image_server.py` - 图像服务器
- `teleop/teleop_hand_and_arm.py` - 主程序
- `test_isaacsim_fix.py` - 测试脚本

## 注意事项
1. 仿真模式需要先启动 IsaacSim 环境，并启用相机功能
2. 确保 `unitree_sim_isaaclab` 已正确安装并配置
3. 首次使用可能需要调整相机分辨率和帧率参数
4. X200 机器人模型需要在 IsaacSim 中正确加载
