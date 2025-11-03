# -*- coding: utf-8 -*-
"""
一个最小可跑的 PI 控制"空调恒温"模拟（不含 D），带流程图、反饱和与可视化。

控制目标：通过调节空调功率，使房间温度稳定在设定值（26℃）

控制变量说明：
- r: 温度设定值（目标温度）
- y: 实际测量温度（当前房间温度）
- u: 控制输出（空调功率指令，正数表示加热，负数表示制冷）
- e: 控制误差（设定值与实际值的差值，e = r - y）

被控对象数学模型（简化的一阶惯性系统+环境换热）：
    y[k+1] = y[k] + alpha * ( -(y[k] - ambient[k]) + beta * u[k] ) + w[k]
  其中各参数含义：
    - alpha: 系统响应速度系数（越大表示温度变化越快）
    - beta: 控制量作用效率（u对温度变化的影响程度）
    - ambient[k]: 环境温度（会随时间变化，如外界温度变化或日晒影响）
    - w[k]: 随机扰动（如测量噪声或模型不确定性）

依赖库：
- 可选：matplotlib（用于图形化显示结果，如果未安装则退化为文本输出）
- numpy（用于数值计算）

运行方式：
- 直接执行：python pid.py
- 程序会依次运行"P控制"和"PI控制"两种情况并对比效果

ASCII 流程图（每个采样周期 Ts 的 PI 算法）：
    +-------------------------+
    |  读取设定 r、测量 y     |
    +-----------+-------------+
                |
                v
         e = r - y  (误差计算)
                |
                v
      I_try = I + Ki * e * Ts     (积分项更新尝试值)
                |
                v
    u_raw = Kp * e + I_try         (P项和I项合成控制量)
                |
                v
         u = clamp(u_raw)          (控制量限幅处理)
                |
         反饱和（条件积分）：
         若 u 被限住 且 e 的方向会让 u 更饱和
         则丢弃本次积分：I 不更新
         否则：I = I_try
                |
                v
          作用到对象 y[k+1]
                |
                v
              下一拍

你可以在下面"参数区"调整控制器参数（Kp、Ki、Ts、限幅范围）或添加扰动（如"日晒"）来观察控制效果变化
"""
from typing import List, Tuple, Optional

try:
    import numpy as np
    import matplotlib.pyplot as plt
    HAS_MPL = True
    
    # 设置中文字体支持
    from matplotlib import rcParams
    # 尝试不同的中文字体
    rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'SimSun', 'FangSong']
    rcParams['axes.unicode_minus'] = False  # 解决负号 '-' 显示为方块的问题
except Exception:
    HAS_MPL = False

# 尝试导入PyQt5用于图形界面
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
        QLabel, QSlider, QPushButton, QLineEdit, QGroupBox, QFormLayout, QMessageBox
    )
    from PyQt5.QtCore import Qt
    HAS_QT = True
except Exception:
    HAS_QT = False


def clamp(x: float, lo: float, hi: float) -> float:
    """
    限幅函数：将输入值限制在指定范围内
    参数：
        x: 待限制的输入值
        lo: 最小值限制
        hi: 最大值限制
    返回：
        如果 x < lo，返回 lo
        如果 x > hi，返回 hi
        否则返回 x
    """
    return lo if x < lo else (hi if x > hi else x)


class PIController:
    """
    PI控制器实现类，包含以下特性：
    1. 比例-积分控制算法
    2. 条件积分（Anti-windup）防饱和机制
    3. 可配置的采样时间
    4. 控制输出限幅功能
    """
    def __init__(self, Kp: float, Ki: float, Ts: float,
                 u_min: float = -100.0, u_max: float = 100.0):
        """
        PI控制器初始化
        参数：
            Kp: 比例增益系数
            Ki: 积分增益系数
            Ts: 采样周期（秒）
            u_min: 控制输出最小值限制
            u_max: 控制输出最大值限制
        """
        self.Kp = float(Kp)          # 比例系数
        self.Ki = float(Ki)          # 积分系数
        self.Ts = float(Ts)          # 采样时间
        self.u_min = float(u_min)    # 控制输出下限
        self.u_max = float(u_max)    # 控制输出上限
        self.I = 0.0                 # 积分项状态值（初始为0）
        self.last_u = 0.0            # 上一次的控制输出（用于调试或特殊用途）

    def reset(self, I0: float = 0.0):
        """
        重置控制器状态
        参数：
            I0: 积分项初始值，默认为0
        """
        self.I = float(I0)           # 重置积分项
        self.last_u = 0.0            # 重置上一次输出

    def step(self, r: float, y: float) -> float:
        """
        PI控制器单步计算（每个采样周期执行一次）
        参数：
            r: 设定值（目标值）
            y: 实际测量值（当前值）
        返回：
            u_sat: 限幅后的控制输出
        """
        # 计算误差：设定值 - 实际值
        e = r - y
        
        # 计算积分项候选值：当前积分 + 误差积分增量
        # 积分增量 = Ki * e * Ts（积分项随时间累积误差）
        I_try = self.I + self.Ki * self.Ts * e
        
        # 计算未限幅的原始控制输出：比例项 + 积分项
        # 比例项 = Kp * e（误差越大，控制作用越强）
        u_raw = self.Kp * e + I_try
        
        # 对控制输出进行限幅处理
        u_sat = clamp(u_raw, self.u_min, self.u_max)

        # 条件积分（反饱和）机制：
        # 当控制输出被限幅且误差方向会使输出更加饱和时，停止积分累积
        # 这可以防止积分项过度累积导致系统响应变差
        saturated = (u_raw != u_sat)  # 判断是否发生限幅
        # 判断是否在推向更严重的饱和状态：
        # 1. 输出已达上限且误差为正（还想继续增加） 
        # 2. 输出已达下限且误差为负（还想继续减小）
        pushing_harder_into_saturation = ( (u_raw > u_sat and e > 0) or
                                           (u_raw < u_sat and e < 0) )
        
        # 如果发生饱和且正在推向更严重饱和，则不更新积分项（丢弃本次积分）
        if saturated and pushing_harder_into_saturation:
            # 丢弃本次积分（不更新 I），防止积分饱和
            pass
        else:
            # 否则正常更新积分项
            self.I = I_try

        # 保存本次控制输出
        self.last_u = u_sat
        return u_sat


def simulate_room(
    steps: int,
    Ts: float,
    controller: PIController,
    r: float,
    y0: float,
    alpha: float = 0.2,       # 响应速度（越大越快）
    beta: float = 0.5,        # 执行量作用效率（u -> y）
    ambient0: float = 20.0,   # 初始环境温度
    ambient_step: Optional[Tuple[int, float]] = (120, 24.0),  # t=120s 切到 24℃
    w_std: float = 0.0,       # 测量/过程噪声标准差
    u_limits: Tuple[float, float] = (-100.0, 100.0),
) -> dict:
    """
    房间温度模拟函数：模拟房间温度在控制器作用下的变化过程
    参数说明：
        steps: 模拟步数（总时间 = steps * Ts）
        Ts: 采样周期（秒）
        controller: 使用的控制器对象
        r: 温度设定值（目标温度，℃）
        y0: 初始温度（℃）
        alpha: 系统响应速度系数（越大表示温度变化越快）
        beta: 控制量作用效率系数（u对温度变化的影响程度）
        ambient0: 初始环境温度（℃）
        ambient_step: 环境温度阶跃扰动（在指定时间点改变环境温度）
                     格式：(时间步数, 新的环境温度) 或 None（无扰动）
        w_std: 随机扰动强度（标准差，0表示无扰动）
        u_limits: 控制输出限幅范围
    返回：
        包含所有模拟数据的字典
    """
    # 设置控制器输出限幅
    lo, hi = u_limits
    controller.u_min, controller.u_max = lo, hi
    controller.reset(I0=0.0)  # 重置控制器状态

    # 初始化记录变量
    t = [0.0]              # 时间序列（从0开始）
    y = [y0]               # 温度序列（从初始温度开始）
    u = []                 # 控制输出序列（空调功率）
    e = []                 # 误差序列（设定值-实际值）
    I_track = [controller.I]  # 积分项序列（跟踪积分状态）
    r_track = [r]          # 设定值序列（目标温度）
    ambient = [ambient0]   # 环境温度序列

    # 主循环：逐步模拟系统行为
    for k in range(steps):
        # 环境温度阶跃扰动处理（模拟日晒、开门等外界影响）
        # 如果设置了扰动且当前步数等于扰动发生时间步数，则改变环境温度
        if ambient_step is not None and k == ambient_step[0]:
            ambient_now = ambient_step[1]   # 应用新的环境温度
        else:
            ambient_now = ambient[-1]       # 保持当前环境温度不变

        # 控制器计算：根据当前设定值和测量值计算控制输出
        uk = controller.step(r=r, y=y[-1])
        # 计算当前误差
        ek = r - y[-1]

        # 添加随机扰动（模拟测量噪声或模型不确定性）
        noise = np.random.normal(0.0, w_std) if HAS_MPL and w_std > 0 else 0.0

        # 被控对象模型更新（计算下一时刻的房间温度）
        # 物理模型：温度变化 = alpha * (环境影响 + 控制作用) + 扰动
        # 其中：环境影响 = -(当前温度-环境温度) 表示热量自然传递
        #      控制作用 = beta * 控制输出 表示空调作用
        y_next = y[-1] + alpha * (-(y[-1] - ambient_now) + beta * uk) + noise

        # 记录当前步的数据
        u.append(uk)           # 记录控制输出
        e.append(ek)           # 记录误差
        I_track.append(controller.I)  # 记录积分状态
        y.append(y_next)       # 记录新的温度值
        ambient.append(ambient_now)   # 记录环境温度
        r_track.append(r)      # 记录设定值
        t.append((k + 1) * Ts) # 记录时间点

    # 返回所有模拟数据
    return {
        "t": np.array(t) if HAS_MPL else t,           # 时间序列
        "y": np.array(y) if HAS_MPL else y,           # 温度序列
        "u": np.array(u) if HAS_MPL else u,           # 控制输出序列
        "e": np.array(e) if HAS_MPL else e,           # 误差序列
        "I": np.array(I_track) if HAS_MPL else I_track,  # 积分项序列
        "r": np.array(r_track) if HAS_MPL else r_track,  # 设定值序列
        "ambient": np.array(ambient) if HAS_MPL else ambient,  # 环境温度序列
        "Ts": Ts,                                     # 采样周期
    }


class PIDControlUI(QMainWindow):
    """
    PID控制器可视化界面主窗口类
    """
    def __init__(self):
        super(PIDControlUI, self).__init__()
        self.setWindowTitle('PID控制器参数调节可视化界面')
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建主窗口部件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(main_widget)
        
        # 左侧控制面板
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)
        
        # 右侧图表区域
        self.create_plot_area()
        main_layout.addWidget(self.canvas, stretch=3)
        
        # 初始化参数
        self.init_parameters()
        
        # 初始绘制
        self.run_simulation_and_plot()

    def create_control_panel(self):
        """
        创建控制面板界面
        """
        control_group = QGroupBox("控制器参数设置")
        control_layout = QVBoxLayout()
        
        # P参数设置
        p_layout = QHBoxLayout()
        p_layout.addWidget(QLabel("比例系数 Kp:"))
        self.kp_input = QLineEdit("10.0")
        self.kp_slider = QSlider(Qt.Horizontal)
        self.kp_slider.setMinimum(0)
        self.kp_slider.setMaximum(500)
        self.kp_slider.setValue(100)
        self.kp_slider.valueChanged.connect(self.update_kp_from_slider)
        self.kp_input.textChanged.connect(self.update_kp_from_input)
        p_layout.addWidget(self.kp_input)
        p_layout.addWidget(self.kp_slider)
        control_layout.addLayout(p_layout)
        
        # I参数设置
        i_layout = QHBoxLayout()
        i_layout.addWidget(QLabel("积分系数 Ki:"))
        self.ki_input = QLineEdit("0.5")
        self.ki_slider = QSlider(Qt.Horizontal)
        self.ki_slider.setMinimum(0)
        self.ki_slider.setMaximum(500)
        self.ki_slider.setValue(50)
        self.ki_slider.valueChanged.connect(self.update_ki_from_slider)
        self.ki_input.textChanged.connect(self.update_ki_from_input)
        i_layout.addWidget(self.ki_input)
        i_layout.addWidget(self.ki_slider)
        control_layout.addLayout(i_layout)
        
        # 其他参数设置
        param_group = QGroupBox("系统参数设置")
        param_layout = QFormLayout()
        
        # 目标温度
        self.target_temp_input = QLineEdit("26.0")
        param_layout.addRow(QLabel("目标温度 (℃):"), self.target_temp_input)
        
        # 初始温度
        self.initial_temp_input = QLineEdit("20.0")
        param_layout.addRow(QLabel("初始温度 (℃):"), self.initial_temp_input)
        
        # 环境温度
        self.ambient_temp_input = QLineEdit("20.0")
        param_layout.addRow(QLabel("环境温度 (℃):"), self.ambient_temp_input)
        
        # 扰动时间
        self.disturbance_time_input = QLineEdit("120")
        param_layout.addRow(QLabel("扰动时间 (s):"), self.disturbance_time_input)
        
        # 扰动后温度
        self.disturbed_temp_input = QLineEdit("24.0")
        param_layout.addRow(QLabel("扰动后温度 (℃):"), self.disturbed_temp_input)
        
        # 模拟时间
        self.simulation_time_input = QLineEdit("240")
        param_layout.addRow(QLabel("模拟时间 (s):"), self.simulation_time_input)
        
        param_group.setLayout(param_layout)
        control_layout.addWidget(param_group)
        
        # 控制按钮
        button_layout = QHBoxLayout()
        self.run_button = QPushButton("运行仿真")
        self.run_button.clicked.connect(self.run_simulation_and_plot)
        self.reset_button = QPushButton("重置参数")
        self.reset_button.clicked.connect(self.reset_parameters)
        button_layout.addWidget(self.run_button)
        button_layout.addWidget(self.reset_button)
        control_layout.addLayout(button_layout)
        
        # 说明文本
        info_label = QLabel(
            "说明:\n"
            "1. 调整Kp、Ki参数观察控制效果\n"
            "2. 修改系统参数模拟不同场景\n"
            "3. 点击运行仿真查看结果\n"
            "4. 在120秒时会有环境温度扰动"
        )
        info_label.setWordWrap(True)
        control_layout.addWidget(info_label)
        
        control_group.setLayout(control_layout)
        return control_group

    def create_plot_area(self):
        """
        创建图表显示区域
        """
        self.figure = plt.Figure(figsize=(5, 4), dpi=100)
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
        self.canvas = FigureCanvasQTAgg(self.figure)

    def init_parameters(self):
        """
        初始化界面参数
        """
        pass

    def update_kp_from_slider(self, value):
        """
        根据滑块值更新Kp输入框
        参数：
            value: 滑块当前值
        """
        kp_value = value / 10.0
        self.kp_input.setText(str(kp_value))

    def update_kp_from_input(self, text):
        """
        根据输入框值更新Kp滑块
        参数：
            text: 输入框当前文本
        """
        try:
            value = float(text)
            self.kp_slider.setValue(int(value * 10))
        except ValueError:
            pass

    def update_ki_from_slider(self, value):
        """
        根据滑块值更新Ki输入框
        参数：
            value: 滑块当前值
        """
        ki_value = value / 100.0
        self.ki_input.setText(str(ki_value))

    def update_ki_from_input(self, text):
        """
        根据输入框值更新Ki滑块
        参数：
            text: 输入框当前文本
        """
        try:
            value = float(text)
            self.ki_slider.setValue(int(value * 100))
        except ValueError:
            pass

    def reset_parameters(self):
        """
        重置所有参数为默认值
        """
        self.kp_input.setText("10.0")
        self.ki_input.setText("0.5")
        self.target_temp_input.setText("26.0")
        self.initial_temp_input.setText("20.0")
        self.ambient_temp_input.setText("20.0")
        self.disturbance_time_input.setText("120")
        self.disturbed_temp_input.setText("24.0")
        self.simulation_time_input.setText("240")
        self.kp_slider.setValue(100)
        self.ki_slider.setValue(50)

    def get_parameters(self):
        """
        从界面获取所有参数
        返回：
            dict: 包含所有参数的字典，如果输入无效则返回None
        """
        try:
            kp = float(self.kp_input.text())
            ki = float(self.ki_input.text())
            target_temp = float(self.target_temp_input.text())
            initial_temp = float(self.initial_temp_input.text())
            ambient_temp = float(self.ambient_temp_input.text())
            disturbance_time = int(float(self.disturbance_time_input.text()))
            disturbed_temp = float(self.disturbed_temp_input.text())
            simulation_time = int(float(self.simulation_time_input.text()))
            
            return {
                'kp': kp,
                'ki': ki,
                'target_temp': target_temp,
                'initial_temp': initial_temp,
                'ambient_temp': ambient_temp,
                'disturbance_time': disturbance_time,
                'disturbed_temp': disturbed_temp,
                'simulation_time': simulation_time
            }
        except ValueError as e:
            QMessageBox.warning(self, "参数错误", "请输入有效的参数值！")
            return None

    def run_simulation_and_plot(self):
        """
        运行仿真并绘制结果
        """
        params = self.get_parameters()
        if not params:
            return
            
        # 创建控制器
        Ts = 1.0  # 采样周期
        steps = params['simulation_time']
        controller = PIController(
            Kp=params['kp'], 
            Ki=params['ki'], 
            Ts=Ts, 
            u_min=-100.0, 
            u_max=100.0
        )
        
        # 运行仿真
        try:
            res = simulate_room(
                steps=steps,
                Ts=Ts,
                controller=controller,
                r=params['target_temp'],
                y0=params['initial_temp'],
                alpha=0.2,
                beta=0.5,
                ambient0=params['ambient_temp'],
                ambient_step=(params['disturbance_time'], params['disturbed_temp']),
                w_std=0.0,
                u_limits=(-100.0, 100.0)
            )
        except Exception as e:
            QMessageBox.critical(self, "仿真错误", f"仿真过程中出现错误：{str(e)}")
            return
        
        # 绘制结果
        self.plot_results(res, params)

    def plot_results(self, res, params):
        """
        绘制仿真结果图表
        参数：
            res: 仿真结果数据
            params: 仿真参数
        """
        self.figure.clear()
        
        # 创建三个子图
        ax1 = self.figure.add_subplot(311)
        ax2 = self.figure.add_subplot(312, sharex=ax1)
        ax3 = self.figure.add_subplot(313, sharex=ax1)
        
        # 温度曲线
        ax1.plot(res["t"], res["y"], label="实际温度", color="#1f77b4")
        ax1.plot(res["t"], res["r"], label="目标温度", color="#ff7f0e", linestyle="--")
        ax1.plot(res["t"], res["ambient"], label="环境温度", color="#2ca02c", alpha=0.7)
        ax1.set_ylabel("温度 (°C)")
        ax1.set_title(f"温度控制效果 (Kp={params['kp']}, Ki={params['ki']})")
        ax1.legend()
        ax1.grid(True)
        
        # 控制输出曲线
        ax2.plot(res["t"][:-1], res["u"], label="控制输出", color="#d62728")
        ax2.set_ylabel("控制输出 (%)")
        ax2.set_title("控制器输出")
        ax2.legend()
        ax2.grid(True)
        
        # 误差曲线
        ax3.plot(res["t"][:-1], res["e"], label="控制误差", color="#9467bd")
        ax3.set_ylabel("误差 (°C)")
        ax3.set_xlabel("时间 (s)")
        ax3.set_title("控制误差")
        ax3.legend()
        ax3.grid(True)
        
        self.figure.tight_layout()
        self.canvas.draw()


def run_demo():
    """
    运行演示函数：对比P控制和PI控制的效果
    """
    # 参数区（你可以修改这些参数观察控制效果变化）
    Ts = 1.0          # 采样周期 1s
    steps = 240       # 模拟步数（总模拟时间240秒）
    y0 = 20.0         # 初始室温 20℃
    r = 26.0          # 设定温度 26℃
    alpha = 0.2       # 被控对象响应速度系数
    beta = 0.5        # 控制量作用效率系数
    u_limits = (-100.0, 100.0)  # 执行器功率限幅范围（-100%到100%）

    # 1) 仅使用P控制（Ki=0）：预期会存在稳态误差（设定值与实际值之间的恒定差值）
    print("正在运行仅P控制模拟...")
    ctrl_P = PIController(Kp=10.0, Ki=0.0, Ts=Ts, u_min=u_limits[0], u_max=u_limits[1])
    res_P = simulate_room(
        steps=steps, Ts=Ts, controller=ctrl_P, r=r, y0=y0,
        alpha=alpha, beta=beta, ambient0=20.0, ambient_step=(120, 24.0),
        w_std=0.0, u_limits=u_limits
    )

    # 2) 使用PI控制（Ki=0.5）：预期能消除稳态误差，即使有扰动也能回到设定值
    print("正在运行PI控制模拟...")
    ctrl_PI = PIController(Kp=10.0, Ki=0.5, Ts=Ts, u_min=u_limits[0], u_max=u_limits[1])
    res_PI = simulate_room(
        steps=steps, Ts=Ts, controller=ctrl_PI, r=r, y0=y0,
        alpha=alpha, beta=beta, ambient0=20.0, ambient_step=(120, 24.0),
        w_std=0.0, u_limits=u_limits
    )

    # 根据是否有matplotlib库决定显示方式
    if HAS_MPL:
        # 使用图形化显示结果
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        # 温度曲线图
        axs[0].plot(res_P["t"], res_P["y"], label="温度 (仅P控制)", color="#d95f02", linestyle="--")
        axs[0].plot(res_PI["t"], res_PI["y"], label="温度 (PI控制)", color="#1b9e77")
        axs[0].plot(res_PI["t"], res_PI["r"], label="目标温度", color="#7570b3", alpha=0.6)
        axs[0].plot(res_PI["t"], res_PI["ambient"], label="环境温度", color="#666666", alpha=0.4)
        axs[0].set_ylabel("温度 (°C)")
        axs[0].set_title("房间温度控制：P控制 vs PI控制")
        axs[0].grid(True)
        axs[0].legend(loc="best")

        # 控制量曲线图
        axs[1].plot(res_P["t"][:-1], res_P["u"], label="控制量 (仅P控制)", color="#d95f02", linestyle="--")
        axs[1].plot(res_PI["t"][:-1], res_PI["u"], label="控制量 (PI控制)", color="#1b9e77")
        axs[1].set_ylabel("控制输出 u (功率%)")
        axs[1].grid(True)
        axs[1].legend(loc="best")

        # 误差曲线图
        axs[2].plot(res_P["t"][:-1], res_P["e"], label="误差 (仅P控制)", color="#d95f02", linestyle="--")
        axs[2].plot(res_PI["t"][:-1], res_PI["e"], label="误差 (PI控制)", color="#1b9e77")
        axs[2].set_ylabel("误差 e = 目标 - 实际")
        axs[2].set_xlabel("时间 (s)")
        axs[2].grid(True)
        axs[2].legend(loc="best")

        plt.tight_layout()
        plt.show()
    else:
        # 没有matplotlib库时，使用文本方式打印关键数值
        print("Matplotlib 不可用，将打印关键数值（前 10 步和最后 5 步）\n")
        def print_snap(tag: str, res: dict):
            print(f"==== {tag} ====")
            for k in range(10):
                print(f"t={res['t'][k]:4.0f}s  y={res['y'][k]:6.2f}  r={res['r'][k]:5.1f}  "
                      f"e={res['e'][k] if k < len(res['e']) else 0:6.2f}  "
                      f"u={res['u'][k] if k < len(res['u']) else 0:7.2f}")
            print("... ...")
            for k in range(len(res['t'])-5, len(res['t'])):
                idx_u = min(k, len(res['u'])-1)
                idx_e = min(k, len(res['e'])-1)
                print(f"t={res['t'][k]:4.0f}s  y={res['y'][k]:6.2f}  r={res['r'][k]:5.1f}  "
                      f"e={res['e'][idx_e]:6.2f}  u={res['u'][idx_u]:7.2f}")
            print()

        print_snap("P-only", res_P)
        print_snap("PI", res_PI)


def main():
    """
    主函数：根据是否安装了PyQt5决定运行命令行版本还是GUI版本
    """
    if HAS_QT:
        # 如果安装了PyQt5，则运行图形界面版本
        app = QApplication(sys.argv)
        window = PIDControlUI()
        window.show()
        sys.exit(app.exec_())
    else:
        # 否则运行命令行版本
        run_demo()


if __name__ == "__main__":
    import sys
    main()
