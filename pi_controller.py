"""
PI控制器（比例-积分控制器）
PI Controller (Proportional-Integral Controller)

PI控制器用于反馈控制系统，通过调整控制输出来减小设定值和实际值之间的误差。
The PI controller is used in feedback control systems to adjust the control output 
to reduce the error between the setpoint and the actual value.
"""

class PIController:
    """
    PI控制器类
    PI Controller class
    
    参数 (Parameters):
    - kp: 比例增益 (Proportional gain)
    - ki: 积分增益 (Integral gain)
    - setpoint: 目标设定值 (Target setpoint)
    - output_limits: 输出限制 (Output limits) - tuple (min, max)
    """
    
    def __init__(self, kp, ki, setpoint=0, output_limits=(None, None)):
        """
        初始化PI控制器
        Initialize PI controller
        
        Args:
            kp (float): 比例增益
            ki (float): 积分增益
            setpoint (float): 目标设定值
            output_limits (tuple): 输出限制 (最小值, 最大值)
        """
        self.kp = kp  # 比例增益
        self.ki = ki  # 积分增益
        self.setpoint = setpoint  # 目标设定值
        self.output_limits = output_limits  # 输出限制
        
        # 内部状态
        self._integral = 0  # 积分累积值
        self._last_time = None  # 上次更新时间
    
    def reset(self):
        """
        重置控制器状态
        Reset controller state
        """
        self._integral = 0
        self._last_time = None
    
    def update(self, measured_value, dt=None):
        """
        更新控制器并计算输出
        Update controller and calculate output
        
        Args:
            measured_value (float): 当前测量值
            dt (float): 时间间隔，如果为None则使用默认值1.0
        
        Returns:
            float: 控制器输出值
        """
        if dt is None:
            dt = 1.0
        
        # 计算误差
        error = self.setpoint - measured_value
        
        # 比例项 (Proportional term)
        p_term = self.kp * error
        
        # 积分项 (Integral term)
        self._integral += error * dt
        i_term = self.ki * self._integral
        
        # 计算总输出
        output = p_term + i_term
        
        # 应用输出限制
        if self.output_limits[0] is not None:
            output = max(output, self.output_limits[0])
        if self.output_limits[1] is not None:
            output = min(output, self.output_limits[1])
        
        return output
    
    def set_setpoint(self, setpoint):
        """
        设置新的目标值
        Set new setpoint
        
        Args:
            setpoint (float): 新的目标设定值
        """
        self.setpoint = setpoint
    
    def set_gains(self, kp=None, ki=None):
        """
        更新控制器增益
        Update controller gains
        
        Args:
            kp (float, optional): 新的比例增益
            ki (float, optional): 新的积分增益
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
    
    def get_components(self, measured_value):
        """
        获取控制器各项的值（用于调试和分析）
        Get individual controller components (for debugging and analysis)
        
        Args:
            measured_value (float): 当前测量值
        
        Returns:
            dict: 包含error, p_term, i_term的字典
        """
        error = self.setpoint - measured_value
        p_term = self.kp * error
        i_term = self.ki * self._integral
        
        return {
            'error': error,
            'p_term': p_term,
            'i_term': i_term,
            'integral': self._integral
        }
