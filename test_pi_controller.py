"""
PI控制器测试
PI Controller Tests

简单的测试来验证PI控制器的基本功能
Simple tests to verify basic PI controller functionality
"""

from pi_controller import PIController

def test_proportional_response():
    """测试比例响应 (Test proportional response)"""
    print("测试1: 比例响应")
    controller = PIController(kp=2.0, ki=0.0, setpoint=10.0)
    
    # 当误差为5时，输出应该是 2.0 * 5 = 10.0
    output = controller.update(5.0, dt=1.0)
    expected = 10.0
    
    assert abs(output - expected) < 0.01, f"预期 {expected}, 得到 {output}"
    print(f"  ✓ 比例项测试通过: 输出 = {output}")

def test_integral_response():
    """测试积分响应 (Test integral response)"""
    print("\n测试2: 积分响应")
    controller = PIController(kp=0.0, ki=1.0, setpoint=10.0)
    
    # 第一次更新：误差=5, 积分=5*1=5, 输出=1.0*5=5.0
    output1 = controller.update(5.0, dt=1.0)
    assert abs(output1 - 5.0) < 0.01, f"预期 5.0, 得到 {output1}"
    print(f"  ✓ 第一次更新: 输出 = {output1}")
    
    # 第二次更新：误差=5, 积分=10, 输出=1.0*10=10.0
    output2 = controller.update(5.0, dt=1.0)
    assert abs(output2 - 10.0) < 0.01, f"预期 10.0, 得到 {output2}"
    print(f"  ✓ 第二次更新: 输出 = {output2}")

def test_combined_pi():
    """测试PI组合 (Test combined PI)"""
    print("\n测试3: PI组合")
    controller = PIController(kp=1.0, ki=0.5, setpoint=20.0)
    
    # 误差=10, P项=1.0*10=10, I项=0.5*10*1=5, 总输出=15
    output = controller.update(10.0, dt=1.0)
    expected = 15.0
    
    assert abs(output - expected) < 0.01, f"预期 {expected}, 得到 {output}"
    print(f"  ✓ PI组合测试通过: 输出 = {output}")

def test_output_limits():
    """测试输出限制 (Test output limits)"""
    print("\n测试4: 输出限制")
    controller = PIController(kp=10.0, ki=0.0, setpoint=100.0, output_limits=(0, 50))
    
    # 误差=90, 输出应该是10*90=900, 但限制在50
    output = controller.update(10.0, dt=1.0)
    assert output <= 50.0, f"输出 {output} 超过最大限制 50.0"
    assert output >= 0.0, f"输出 {output} 低于最小限制 0.0"
    print(f"  ✓ 输出限制测试通过: 输出 = {output} (限制在0-50之间)")

def test_reset():
    """测试重置功能 (Test reset functionality)"""
    print("\n测试5: 重置功能")
    controller = PIController(kp=1.0, ki=1.0, setpoint=10.0)
    
    # 累积一些积分
    controller.update(5.0, dt=1.0)
    controller.update(5.0, dt=1.0)
    output_before = controller.update(5.0, dt=1.0)
    
    # 重置
    controller.reset()
    
    # 重置后，积分应该为0
    output_after = controller.update(5.0, dt=1.0)
    
    # 重置前输出应该大于重置后（因为有积分累积）
    assert output_before > output_after, "重置后输出应该减少"
    print(f"  ✓ 重置测试通过: 重置前 = {output_before:.2f}, 重置后 = {output_after:.2f}")

def test_setpoint_change():
    """测试设定值改变 (Test setpoint change)"""
    print("\n测试6: 设定值改变")
    controller = PIController(kp=1.0, ki=0.0, setpoint=10.0)
    
    output1 = controller.update(5.0, dt=1.0)
    
    # 改变设定值
    controller.set_setpoint(20.0)
    output2 = controller.update(5.0, dt=1.0)
    
    # 设定值增加后，误差和输出都应该增加
    assert output2 > output1, "设定值增加后输出应该增加"
    print(f"  ✓ 设定值改变测试通过: 设定值10时输出 = {output1}, 设定值20时输出 = {output2}")

def test_get_components():
    """测试获取组件 (Test get components)"""
    print("\n测试7: 获取组件")
    controller = PIController(kp=2.0, ki=0.5, setpoint=10.0)
    
    controller.update(5.0, dt=1.0)
    components = controller.get_components(5.0)
    
    assert 'error' in components, "应该包含error"
    assert 'p_term' in components, "应该包含p_term"
    assert 'i_term' in components, "应该包含i_term"
    assert 'integral' in components, "应该包含integral"
    
    print(f"  ✓ 获取组件测试通过:")
    print(f"    误差 = {components['error']}")
    print(f"    P项 = {components['p_term']}")
    print(f"    I项 = {components['i_term']}")
    print(f"    积分 = {components['integral']}")

def run_all_tests():
    """运行所有测试 (Run all tests)"""
    print("=" * 60)
    print("开始PI控制器测试")
    print("=" * 60)
    
    try:
        test_proportional_response()
        test_integral_response()
        test_combined_pi()
        test_output_limits()
        test_reset()
        test_setpoint_change()
        test_get_components()
        
        print("\n" + "=" * 60)
        print("✓ 所有测试通过！")
        print("=" * 60)
        return True
    except AssertionError as e:
        print(f"\n✗ 测试失败: {e}")
        return False
    except Exception as e:
        print(f"\n✗ 测试错误: {e}")
        return False

if __name__ == "__main__":
    import sys
    success = run_all_tests()
    sys.exit(0 if success else 1)
