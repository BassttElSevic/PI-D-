"""
PI控制器使用示例
PI Controller Usage Example

这个示例展示了如何使用PI控制器来控制一个简单的系统
This example demonstrates how to use the PI controller to control a simple system
"""

from pi_controller import PIController

def simulate_temperature_control():
    """
    模拟温度控制系统
    Simulate a temperature control system
    
    假设我们要将温度控制在目标值（如100°C）
    Assume we want to control temperature to a target value (e.g., 100°C)
    """
    print("=" * 60)
    print("温度控制示例 (Temperature Control Example)")
    print("=" * 60)
    
    # 创建PI控制器
    # 目标温度: 100°C
    # Kp = 2.0 (比例增益)
    # Ki = 0.5 (积分增益)
    # 输出限制: 0-100 (加热功率百分比)
    controller = PIController(
        kp=2.0, 
        ki=0.5, 
        setpoint=100.0,
        output_limits=(0, 100)
    )
    
    # 模拟系统
    current_temp = 20.0  # 初始温度
    dt = 1.0  # 时间步长（秒）
    
    print(f"\n目标温度: {controller.setpoint}°C")
    print(f"初始温度: {current_temp}°C")
    print(f"\n{'时间':>6} {'温度':>8} {'误差':>8} {'P项':>8} {'I项':>8} {'输出':>8}")
    print("-" * 60)
    
    # 运行模拟
    for i in range(20):
        time = i * dt
        
        # 计算控制器输出
        control_output = controller.update(current_temp, dt)
        
        # 获取控制器各项用于显示
        components = controller.get_components(current_temp)
        
        # 打印当前状态
        print(f"{time:6.1f} {current_temp:8.2f} {components['error']:8.2f} "
              f"{components['p_term']:8.2f} {components['i_term']:8.2f} {control_output:8.2f}")
        
        # 模拟系统响应（简化的温度模型）
        # 温度变化 = (加热功率 - 散热) * 时间步长
        heating = control_output * 0.5  # 加热效果
        cooling = (current_temp - 20.0) * 0.1  # 散热效果
        current_temp += (heating - cooling) * dt
    
    print("\n模拟完成！")

def simulate_speed_control():
    """
    模拟速度控制系统
    Simulate a speed control system
    
    假设我们要将电机速度控制在目标值
    Assume we want to control motor speed to a target value
    """
    print("\n" + "=" * 60)
    print("速度控制示例 (Speed Control Example)")
    print("=" * 60)
    
    # 创建PI控制器
    # 目标速度: 50 RPM
    # Kp = 1.5 (比例增益)
    # Ki = 0.3 (积分增益)
    # 输出限制: -100到100 (电机功率百分比，正负表示方向)
    controller = PIController(
        kp=1.5,
        ki=0.3,
        setpoint=50.0,
        output_limits=(-100, 100)
    )
    
    # 模拟系统
    current_speed = 0.0  # 初始速度
    dt = 0.5  # 时间步长（秒）
    
    print(f"\n目标速度: {controller.setpoint} RPM")
    print(f"初始速度: {current_speed} RPM")
    print(f"\n{'时间':>6} {'速度':>8} {'误差':>8} {'输出':>8}")
    print("-" * 40)
    
    # 运行模拟
    for i in range(15):
        time = i * dt
        
        # 计算控制器输出
        control_output = controller.update(current_speed, dt)
        
        # 获取误差
        components = controller.get_components(current_speed)
        
        # 打印当前状态
        print(f"{time:6.1f} {current_speed:8.2f} {components['error']:8.2f} {control_output:8.2f}")
        
        # 模拟系统响应（简化的速度模型）
        # 速度变化 = (驱动力 - 阻力) * 时间步长
        acceleration = control_output * 0.3
        friction = current_speed * 0.05
        current_speed += (acceleration - friction) * dt
    
    print("\n模拟完成！")

def demonstrate_setpoint_change():
    """
    演示改变设定值
    Demonstrate setpoint change
    """
    print("\n" + "=" * 60)
    print("设定值变化示例 (Setpoint Change Example)")
    print("=" * 60)
    
    controller = PIController(kp=1.0, ki=0.2, setpoint=10.0)
    
    current_value = 0.0
    dt = 1.0
    
    print(f"\n{'时间':>6} {'当前值':>8} {'目标值':>8} {'输出':>8}")
    print("-" * 40)
    
    for i in range(25):
        time = i * dt
        
        # 在第10秒改变设定值
        if i == 10:
            controller.set_setpoint(20.0)
            print("  >>> 设定值改变为 20.0 <<<")
        
        control_output = controller.update(current_value, dt)
        
        print(f"{time:6.1f} {current_value:8.2f} {controller.setpoint:8.2f} {control_output:8.2f}")
        
        # 简单的一阶系统响应
        current_value += control_output * 0.3 * dt
    
    print("\n模拟完成！")

if __name__ == "__main__":
    # 运行所有示例
    try:
        simulate_temperature_control()
    except Exception as e:
        print(f"\n错误: 温度控制示例失败 - {e}")
    
    try:
        simulate_speed_control()
    except Exception as e:
        print(f"\n错误: 速度控制示例失败 - {e}")
    
    try:
        demonstrate_setpoint_change()
    except Exception as e:
        print(f"\n错误: 设定值变化示例失败 - {e}")
    
    print("\n" + "=" * 60)
    print("所有示例运行完成！")
    print("=" * 60)
