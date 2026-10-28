# 原版四舞蹈 Mod

从 `bxi_rl_controller_ros2_example` 移植的四组动作，适配目标项目 Mod API 4。
模型和动作文件全部包含在本目录内，不依赖原项目目录；依赖目标的
`com.bxi.basic_actions`，默认启用并由框架自动发现。

| 原动作 | 手柄（USB / CRSF） | 键盘 | 目标事件 | 动作 / 模型 |
| --- | --- | --- | --- | --- |
| 跳舞片段3 | RT+Y | 4 | btn_10=55 | face3.npz / face3_17k.onnx |
| 跳舞片段5 | RT+B | 5 | btn_10=56 | face4.npz / face4_4k.onnx |
| 机械舞 | RT+A | 8 | btn_10=57 | jixiewu.npz / jixiewu2.onnx |
| lichenxi舞蹈 | RT+X | 9 | btn_10=58 | lichenxi.npz / lichenxi3.onnx |

先进入普通行走状态（RB+X / 键盘 1），再选择舞蹈。单独按 X 暂停/继续，
RB+X / 键盘 1 返回行走，RB+A 切零力矩。播放结束自动回到行走；姿态超限
沿用目标框架的零力矩保护。片段3与片段5保留原项目的互切和重播入口。

片段3和片段5使用 `DanceMotionPolicyGravityIsaaclabV3`，分别部署
`face3_17k.onnx` 和 `face4_4k.onnx`。
目标关节位置为默认姿态加缩放后的动作输出；其余两个舞蹈沿用
`DanceMotionPolicyGravityIsaaclabV2`，目标位置为模型参考关节位置加缩放后的动作输出。
默认从第10帧开始、固定参考朝向、50 Hz 动作时间线，以及0.3秒 smootherstep 动态混合切入。
手动返回行走和播放结束返回也使用0.3秒过渡。
模型由目标框架在后台按需加载；首次选择动作需等待资源准备完成。

片段3和片段5当前各裁掉末尾50帧（1秒），分别播放至原动作第1375帧和
第1349帧（从0编号、包含该帧）；机械舞和 lichenxi 舞蹈不裁尾。
裁剪只限制播放起止点，不修改原始 NPZ/ONNX，也不截断模型读取的参考窗口。
可在对应状态的 `params.start_frame` 设置起始帧号（从0编号，包含该帧），
默认10；设为0从头播放，设为60则比默认少播开头1秒。
当前清单中片段5从第50帧开始，其余三个舞蹈从第10帧开始；移植保留该配置。
`params.end_trim_frames` 设置裁尾帧数，设为0不裁尾。

四个舞蹈都支持独立配置，例如修改 `mod.yaml` 中已有的对应字段：

```yaml
states:
  dance_part_3:
    params:
      start_frame: 60      # 比默认第10帧起播，再跳过开头1秒
      end_trim_frames: 50  # 裁掉末尾1秒
  dance_part_5:
    params:
      start_frame: 10
      end_trim_frames: 50
  dance_part_6:
    params:
      start_frame: 0       # 从头完整播放
      end_trim_frames: 0   # 改成50即裁掉末尾1秒
  dance_part_7:
    params:
      start_frame: 10
      end_trim_frames: 0
```

以上仅展示裁剪字段，编辑时保留各状态已有的蓝牙参数、manifest等配置。
两项必须是非负整数，并且 `start_frame + end_trim_frames < 动作总帧数`，
至少保留1帧；不合法配置会报错。到达裁剪终点后自动以0.3秒过渡返回行走。
蓝牙触发点如果被裁掉则
不会发送指令；当前片段3/5各裁50帧仍保留原触发点。
修改后重新构建该包并重启控制器生效，不支持运行中热更新。

RT 四个组合键现在用于本 Mod；基础 Mod 中的鼓掌、挥手、躺下动作已删除。
深度行走的键盘 0 映射仍保留（需另行安装对应 Mod）；LB+B / LB+Y 仍保留
两种跑步入口。事件使用55–58，
避免原项目3–6的编号触发目标已有的 LT 动作和“物品讲解”。原项目把三个
舞蹈重复绑定到键盘5，本次分别使用5、8、9，避免 `first_wins` 屏蔽后两个。

## 蓝牙变脸联动

片段3在第900帧、片段5在第1000帧向原设备写入字节 `0x01`，不要求响应。
地址默认为 `E0:72:A1:D8:39:8E`，特征 UUID 默认为
`0000ee01-0000-1000-8000-00805f9b34fb`。蓝牙操作在后台执行；可选依赖
`bleak` 不可用时仅禁用联动，不阻断舞蹈。机械舞和 lichenxi 舞蹈不连接蓝牙。
可在 `mod.yaml` 对应状态的 `params` 中修改 `ble_mac`、`ble_characteristic`，
或设置 `ble_enabled: false` 关闭联动。

发送时机也在对应状态的 `params.ble_trigger_frames` 中配置：

```yaml
# states.dance_part_3.params
ble_trigger_frames: [900]
# states.dance_part_5.params
ble_trigger_frames: [1000]
```

这里填写原始动作帧号（50帧=1秒），修改 `start_frame` 后不会重新从0编号，
小于起始帧的触发点不会补发。提前1秒将帧号减50，
延后1秒加50。支持多个触发点，例如 `[900, 1100]`；每个帧号每轮播放仅
发送一次。省略该字段或填写 `[]` 表示不发送。列表元素必须是非负整数；
暂停时不会推进触发时间，被裁掉的触发点不会发送。修改后重新构建该包并
重启控制器生效，不支持运行中热更新。

## 构建与移植

在 `bxi_controller_ros2` 工作区执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to bxi_example_py_elf3 remote_controller
source install/setup.bash
```

使用完整复制安装；当前框架会校验资产解析后的路径，`--symlink-install`
生成的逐文件资产链接可能被判定为超出 Mod 的 `assets/` 目录。

重新启动控制器和遥控节点后生效。向其他目标工作区复制时，需要同时复制
本 mod 目录和本次更新后的 `src/remote_controller/config/xbox_default.yaml`。
只复制 mod 会增加动作状态，但不会修改遥控节点的物理按键映射。
合并遥控配置时仅同步舞蹈按键相关字段，保留目标工程已有的 `system` 启停配置，
包括 IMU 独占读取与守护逻辑。

本地验证（在工作区根目录执行）：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/bxi_example_py_elf3:$PYTHONPATH" python3 -m pytest \
  src/bxi_example_py_elf3/mods/com.bxi.dances/tests -q
```

测试涵盖实际 ONNX 全动作帧推理、暂停/恢复、结束返回、重入、安全退出、
Mod 状态图与事件冲突、蓝牙帧触发逻辑。离线推理测试不等同于机器人运动
仿真或真机验证；本次未验证真实机器人和蓝牙设备。
