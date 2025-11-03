# PI-D-
Basstt用来理解PI控制的，但由于水平有限，目前还没有D的部分

## PI控制器实现 (PI Controller Implementation)

这个项目包含一个用Python编写的PI（比例-积分）控制器实现。

### 文件说明 (Files)

- `pi_controller.py` - PI控制器的核心实现
- `example.py` - 使用示例，展示了如何使用PI控制器

### 什么是PI控制？(What is PI Control?)

PI控制器是一种反馈控制器，由两部分组成：

- **P（比例）项**: 根据当前误差（目标值 - 实际值）进行调整
- **I（积分）项**: 根据累积误差进行调整，消除稳态误差

控制器输出 = Kp × 误差 + Ki × 误差积分

### 使用方法 (Usage)

```python
from pi_controller import PIController

# 创建PI控制器
controller = PIController(
    kp=2.0,           # 比例增益
    ki=0.5,           # 积分增益
    setpoint=100.0,   # 目标值
    output_limits=(0, 100)  # 输出限制
)

# 在控制循环中使用
measured_value = 20.0  # 当前测量值
control_output = controller.update(measured_value, dt=1.0)
```

### 运行示例 (Run Examples)

```bash
python3 example.py
```

示例包括：
1. 温度控制系统模拟
2. 速度控制系统模拟
3. 设定值变化演示

### 特性 (Features)

- ✅ 比例控制（P项）
- ✅ 积分控制（I项）
- ✅ 可配置的输出限制
- ✅ 动态设定值调整
- ✅ 详细的中文和英文注释
- ❌ 微分控制（D项）- 还未实现

### 注意事项 (Notes)

由于Basstt水平有限，目前只实现了PI控制，D（微分）部分还在学习中。
