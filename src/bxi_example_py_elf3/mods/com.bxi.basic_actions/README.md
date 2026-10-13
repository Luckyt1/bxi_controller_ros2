# 基础动作 Mod

提供行走、零力矩、IMU 失联保护、初始姿态、零位、基础舞蹈、起身，以及两种奔跑状态。
躺下、鼓掌、挥手的状态、App 入口、事件、路由和专用资产已移除。

RT 四个组合键的舞蹈由 `com.bxi.dances` 提供；本 Mod 继续提供其依赖的
`normal`、`zero_torque` 和 `toggle_pause` 事件。

在工作区根目录运行本 Mod 的离线回归测试：

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src/bxi_example_py_elf3:$PYTHONPATH" \
  python3 -m pytest -p no:cacheprovider \
  src/bxi_example_py_elf3/mods/com.bxi.basic_actions/tests -q
```

测试检查保留状态的工厂、资源、参数、按键事件、安全退出和四舞蹈入口。
部署时重新构建 `bxi_example_py_elf3` 并重启控制器，安装步骤会清理旧 Mod 文件。
离线测试不能替代真机运动验证。
