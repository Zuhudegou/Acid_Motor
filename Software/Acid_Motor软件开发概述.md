前言 由于本人学艺不精 如果有错误的地方 还恳请批评指出

## 1. 控制方法

当前固件真正实现的是基于增量式编码器的有感 FOC 我现在使用的还是参考板 后续再移植到 Acid_Motor 无感控制暂时只作为学习和验证内容

### 1.1 无感控制

无感控制并不是不需要转子角度 而是由单片机根据电压、电流和电机模型估算角度

依赖反电动势的无感算法满足

$$
E\propto\omega_e
$$

静止和低速时反电动势很小 电阻压降、采样误差和噪声反而更明显 所以我把以后可能采用的无感启动分成四步

1. 使用固定电角度完成转子对齐
2. 使用开环旋转电角度将电机带到可观测转速
3. 等待观测器输出稳定
4. 将开环角度平滑切换到观测角度

角度不能直接切换 否则 Park 坐标系以及 $i_d,i_q$ 会同时跳变 造成转矩冲击和过流

#### 1.1.1 滑模观测器

我目前保留的无感方案是滑模观测器（其实是因为目前只会这个 想写高频注入来着） 它根据 PMSM 的 $\alpha-\beta$ 模型估算电流 再利用估算电流与实际电流的误差修正反电动势

以 $\alpha$ 轴为例 电机模型为

$$
\frac{di_\alpha}{dt}=
-\frac{R_s}{L_s}i_\alpha
+\frac{1}{L_s}u_\alpha
-\frac{1}{L_s}e_\alpha
$$

观测器使用估算电流 $\hat i_\alpha$ 并以滑模项 $z_\alpha$ 代替未知反电动势

$$
\frac{d\hat i_\alpha}{dt}=
-\frac{R_s}{L_s}\hat i_\alpha
+\frac{1}{L_s}u_\alpha
-\frac{1}{L_s}z_\alpha
$$

$\beta$ 轴采用相同计算 当估算电流逐渐跟随实际电流后 我对 $z_\alpha,z_\beta$ 进行低通滤波 得到反电动势估算

$$
\hat e_\alpha\approx LPF(z_\alpha)
\qquad
\hat e_\beta\approx LPF(z_\beta)
$$

电角度可以由反电动势方向得到

$$
\hat\theta_e=
\operatorname{atan2}(-\hat e_\alpha,\hat e_\beta)
$$

后续真正实现时 我还会加入 PLL 和相位补偿 并按照自己的相序、Clarke 与 Park 定义重新确认符号 目前固件虽然能够辨识 $R_s,L_s$ 但这些结果还没有接入滑模观测器

#### 1.1.2 当前没有采用的无感方法

反电动势过零检测主要用于六步换相 当前项目采用 FOC 所以我没有选择这条路径

高频注入可以在低速和零速估算转子方向 但它依赖电机凸极性 还会增加噪声、损耗和实现复杂度 当前也没有加入

如果后续增加无感控制 我会先实现“开环启动 + 中高速滑模观测器” 不同时展开多套观测器

### 1.2 有感控制

有感控制直接读取编码器角度 编码器给出机械角度 FOC 使用电角度 两者关系为

$$
\theta_e=
\operatorname{wrap}
\left(
s\,p\,\theta_m-\theta_{offset}
\right)
$$

其中 $p$ 是极对数 $s$ 表示编码器方向 $\theta_{offset}$ 表示编码器零位与转子磁极零位之间的偏差 当前固件已经自动辨识方向和零位

获得电角度后 我的有感 FOC 按照下面的顺序运行

$$
\text{采样相电流和编码器}
\longrightarrow
\text{计算电角度}
\longrightarrow
\text{Clarke/Park}
\longrightarrow
\text{电流 PI}
\longrightarrow
\text{逆 Park/SVPWM}
$$

有感控制在静止和低速时仍能获得角度 更适合当前关节电机 缺点是增加传感器、布线和安装要求

### 1.3 我的控制方案选择

我当前先把基于编码器的有感 FOC 做可靠 无感控制保留为后续学习和扩展方向

## 2. 本项目的软件实现

下面我按照当前固件的真实执行顺序记录软件实现 整体流程如下

```text
通电
→ 初始化
→ 建立 20kHz 控制周期
→ 校准电流零点
→ 辨识 Rs、Ls、编码器方向和零位
→ 进入有感 FOC
→ 根据电流、速度或位置目标输出 PWM
```
### 2.1 软件分层

这次我按照职责把软件拆分为以下几层

```text
App/Command 与 App/UI
└─ 负责接收命令和显示状态

App/motor_hal.c
└─ 在板级驱动与控制库之间转换数据

App/board
└─ 封装 ADC、PWM、编码器和功率级控制

App/motor_board.c
└─ 保存与电机相关的控制参数

Lib/Motor
└─ 状态机、参数辨识、FOC、PID 和三级闭环
```

`Lib/Motor` 实现电流换算、状态机、参数辨识和控制算法 `motor_tick()` 接收的是 U/W 相原始 ADC、母线原始 ADC 和编码器计数 控制库内部再将它们换算成控制量

`App/board` 负责板级外设访问 包括读取原始 ADC 与编码器以及向 TIM1 写入三个比较值

`motor_hal.c` 是控制库与板级驱动之间的适配层 我在这里把同一周期的硬件输入保存 再把计算结果提交给硬件

我把当前参考板的固定参数集中在 `App/motor_board.c`

```c
#define MOTOR_BOARD_PWM_PERIOD         (4250U)
#define MOTOR_BOARD_CONTROL_TS         (0.00005f)
#define MOTOR_BOARD_OVER_CURRENT       (12.0f)
#define MOTOR_BOARD_BUS_VOLTAGE_MIN    (10.0f)
#define MOTOR_BOARD_BUS_VOLTAGE_MAX    (40.0f)
#define MOTOR_BOARD_POLE_PAIRS         (7U)
#define MOTOR_BOARD_ENCODER_LINES      (1024U)
#define MOTOR_BOARD_ACCELERATION       (4800.0f)
#define MOTOR_BOARD_SPEED_DIV          (2U)
#define MOTOR_BOARD_POS_DIV            (4U)
```

这些值由我提前配置 主要包括 PWM 与控制周期、采样和保护参数、电机与编码器参数以及速度和位置分频

### 2.2 系统初始化

`main()` 中与电机控制有关的代码如下

```c
Motor_UI_Init();
motor_hal_init();

if (motor_hal_start(&g_motor) != 0)
{
    Error_Handler();
}

Motor_Command_Init();

while (1)
{
    Motor_Command_Task();
    Motor_UI_Task();
}
```

`main()` 只负责初始化和前台任务 `while(1)` 处理命令与屏幕 FOC 不放在主循环中 因为屏幕刷新和阻塞式串口发送会造成控制周期抖动

功率级的启动过程如下

```c
void motor_hal_init(void)
{
    motor_init(&g_motor, motor_board_default_config());
    motor_hal_write_pwm(&g_motor);
    board_motor_powerstage(0U);
}

int motor_hal_start(motor_t *m)
{
    board_delay_ms(100U);
    if (board_motor_adc_calibrate() != 0) return -1;

    board_delay_ms(10U);
    if (board_motor_start_io(m->mc.Sample.AdcBuff, 3U) != 0) return -1;

    board_motor_powerstage(1U);
    return 0;
}
```

这段启动过程可以压成四步

```text
建立电机对象并关闭功率级
→ ADC 硬件自校准
→ 启动 ADC、编码器、PWM 和控制中断
→ 拉高 SD 使能功率级
```

100ms 和 10ms 两次等待都在主线程中执行 此时 50μs 状态机还没有开始 TIM1 与 ADC 中断启动后 软件零偏和参数辨识才按控制周期运行

### 2.3 控制

我使用 TIM1 更新事件触发 ADC 注入组采样 ADC 转换完成后再进入 FOC

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BOARD_TIM_PWM.Instance)
    {
        (void)HAL_ADCEx_InjectedStart_IT(&BOARD_ADC_PHASE);
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != BOARD_ADC_PHASE_INST) return;
    motor_hal_isr_tick();
}
```

TIM1 更新发生时本周期电流还没有转换完成 如果立刻执行 FOC 可能读到上一周期数据

当前实时链路为

```text
TIM1 更新
→ 软件触发 ADC2 注入组
→ U、W 相电流转换完成
→ ADC 完成中断
→ motor_hal_isr_tick()
→ 读取输入
→ 运行状态机或 FOC
→ 写入三相 CCR
```

`motor_hal_isr_tick()` 是硬件和控制算法的交界线

```c
void motor_hal_isr_tick(void)
{
    motor_hw_inputs_t in;

    motor_hal_read_inputs(&g_motor, &in);
    motor_tick(&g_motor, &in);
    motor_hal_write_pwm(&g_motor);
}
```

一次控制中断按照读取输入、执行控制、提交输出的顺序运行

- `motor_hal_read_inputs()` 读取 U/W 相原始 ADC、母线原始 ADC 和编码器计数
- `motor_tick()` 保存这些原始值并在控制库内完成换算和控制
- `motor_hal_write_pwm()` 将控制结果写入定时器

U、W 相电流、母线和编码器会先保存成一份软件快照 后面的算法使用同一份数据 电位器是一个例外 它目前仍然直接读取持续更新的 DMA 缓冲区

当前基础控制周期为

$$
T_s=50\,\mu s
\qquad
f_s=20\,kHz
$$

电流采样、状态机、参数辨识、公共保护和电流环都以 20kHz 运行 速度与位置分频放到对应控制模式中再记录

#### 2.3.1 ADC 常规组与注入组

我让 ADC2 同时承担两类采样任务

```text
常规组 + DMA
├─ AdcBuff[0]：母线电压
├─ AdcBuff[1]：电位器
└─ AdcBuff[2]：已经采样 但控制暂时没使用

注入组
├─ U 相电流
└─ W 相电流
```

常规组通过 DMA 连续更新母线电压、电位器等低实时性数据

注入组在每个 PWM 周期由 TIM1 更新事件触发 用于采集实时性要求更高的相电流

只测 U、W 两相是因为星形连接的三相电流满足

$$
I_u+I_v+I_w=0
$$

所以第三相可以重建

$$
I_v=-I_u-I_w
$$

我这里仍由 TIM1 更新中断***软件触发***注入 ADC 目前只能保证采样与 PWM 周期同步 还不是硬件精确中点采样

### 2.4 状态机

我用外层状态机按固定顺序调度零偏校准、参数辨识和正常控制

```text
ADC_CALIB
→ MOTOR_IDENTIFY
   ├─ RESISTANCE_IDENTIFICATION
   ├─ INDUCTANCE_IDENTIFICATION
   └─ ENCODER_ROTOR_ALIGN
→ MOTOR_SENSORUSE
```

状态机的核心分支位于 `motor_system.c`

```c
switch (mc->Motor.RunState)
{
    case ADC_CALIB:
        Calculate_Adc_Offset(smp);
        if (smp->CalibEndFlag == 1)
            mc->Motor.RunState = MOTOR_IDENTIFY;
        break;

    case MOTOR_IDENTIFY:
        Motor_Identify(m);
        break;

    case MOTOR_SENSORUSE:
        Calculate_Encoder_Data(&mc->EAngle);
        Sensoruse_Control(m);
        break;

    case MOTOR_ERROR:
    case MOTOR_STOP:
        break;
}
```

各状态采用非阻塞方式执行 每个 50μs 周期只完成一次状态更新 不在控制中断中持续等待 因此屏幕和串口任务仍可在主循环运行

我把运行阶段和控制模式分成两个变量

- `RunState` 决定现在处于校准、辨识、有感控制、STOP 还是 ERROR
- `RunMode` 只在有感状态内部决定运行电流、速度还是位置模式

我在 `motor_tick()` 中先更新目标 再进入状态机 目标来源记录在 2.7 状态机只决定这些目标何时进入闭环和 PWM

### 2.4 电机参数识别

我当前只自动辨识 $R_s$、一个等效 $L_s$ 以及增量式编码器的方向和电角度零位 极对数、采样比例和 PID 仍然由我写在配置中 $L_q$、磁链和转动惯量目前没有辨识

辨识入口在 `motor_identify.c` `MOTOR_IDENTIFY` 按照 2.4 状态图中的三个子状态执行

辨识期间会建立静止磁场 转轴会发生对齐并产生电流

#### 2.4.1 定子电阻 Rs

我先在定子 $+\alpha$ 轴施加静止电压 此时

$$
U_q=0
\qquad
U_\alpha=U_d
\qquad
U_\beta=0
$$

电流稳定后 $\frac{di}{dt}\approx0$ 因此

$$
u=R_si+L_s\frac{di}{dt}
\quad\Longrightarrow\quad
u\approx R_si
$$

代码不会直接给定一个较大的 $U_d$ 而是每个 50μs 周期增加 0.0001V 也就是按照 2V/s 缓慢上升

```c
float current = (smp->IuReal * foc->Ud * 1.5f) / smp->BusReal;

if (current >= 0.6f * id->CurMax)
{
    id->Flag = 2;
}
else
{
    foc->Ud += 0.0001f;
    id->VoltageSet[0] = foc->Ud;
}
```

这里的 `current` 是母线等效电流估算值

$$
I_{dc,est}=\frac{1.5U_dI_u}{V_{bus}}
$$

当前 `CurMax=0.6` 第一、第二工作点分别取 0.36A 和 0.6A 这两个值不是相电流限幅

每个工作点保持 4100 个控制周期 前 4000 个周期用于稳定 最后 100 个周期求平均 得到 $U_1,I_1$ 和 $U_2,I_2$ 最后使用两点斜率计算

$$
R_s=\frac{U_2-U_1}{I_2-I_1}
$$

我这里使用的是指令电压而不是电机端实测电压 死区和 MOS 管压降都会进入结果 目前也没有给 $U_d$ 增加独立上限、辨识超时和分母有效性检查

#### 2.4.2 定子电感 Ls

测完电阻后 我先把 $U_d$ 清零 等待 $I_u$ 回到 $\pm0.05A$ 再阶跃施加第二工作点电压 $U_2$ 并记录电流达到 $0.95I_2$ 所经历的控制周期数

```c
foc->Ud = id->VoltageSet[1];
id->WaitTim++;

if (smp->IuReal >= id->CurAverage[1] * 0.95f)
{
    id->LsSum += id->Rs * 0.334f * ts * id->WaitTim;
    id->WaitTim = 0;
    id->Count++;
    id->Flag = 0;
    foc->Ud = 0;
}
```

其中 0.334 来自 RL 阶跃响应

$$
L_s=\frac{R_s t_{95}}{\ln20}
\approx0.334R_sT_sN
$$

我重复测量 100 次后取平均 再执行 `Ld=Ls` 当前并没有单独测量 $L_q$ 这组结果也还没有用于电流 PI 自动整定和 dq 解耦

#### 2.4.3 编码器方向与零位

最后一步是确定编码器方向和磁极零位 我先在 $+\beta$ 轴建立静止磁场并记录 $E_1$ 再切换到 $+\alpha$ 轴记录 $E_2$ 两次对齐都会让 $U_d$ 从 0 按每周期 0.0001V 上升到 $U_2$ 达到后立即记录编码器 当前没有单独的稳定等待

按当前 7 极对、4096 计数配置 两次位置理论变化量的绝对值约为 146 个计数 我用经过回绕处理后的 $\Delta E=E_2-E_1$ 判断方向

```c
ea->EncoderValChange = ea->EncoderVal - ea->EncoderValChange;

if (ea->EncoderValChange < -encoder_half_counts)
    ea->EncoderValChange += encoder_counts;
else if (ea->EncoderValChange > encoder_half_counts)
    ea->EncoderValChange -= encoder_counts;

if (ea->EncoderValChange > 0)
    ea->Dir = 1;
else if (ea->EncoderValChange < 0)
    ea->Dir = 0;
```

第二次 $+\alpha$ 轴对齐的位置被我定义为电角度 0 我根据 `Dir` 修正这次编码器读数后再保存到 `CalibOffset`

#### 2.4.4 辨识结束

`Motor_Identify()` 完成后会将 `EndFlag` 置 1 外层状态机随后完成状态切换和运行数据初始化

```c
mc->Motor.RunState = MOTOR_SENSORUSE;
Calculate_Encoder_Data(&mc->EAngle);
motor_home_position(m);

mc->Speed.ElectricalPosThis = mc->EAngle.ElectricalAnglePU;
mc->Speed.ElectricalPosLast = mc->EAngle.ElectricalAnglePU;
mc->Speed.ElectricalSpeedLPF = 0.0f;
mc->Speed.MechanicalSpeed = 0.0f;
```

我在软件中保留了两个不同的零点

- `CalibOffset` 是编码器零位与转子磁极零位的关系 用于 FOC
- `Home` 是我定义的位置坐标起点 用于位置控制

辨识结果目前只保存在 RAM 中 断电后会重新执行 编码器对齐还没有最小移动检查 辨识内部状态也没有完整复位 所以我暂时把它当成冷启动流程 而不是可以随时重跑的参数辨识

### 2.5 有感 FOC 执行

#### 2.5.1 编码器计数转换为电角度

进入 `MOTOR_SENSORUSE` 后 我每个控制周期都先更新编码器角度

```c
if (p->Dir == 1)
{
    p->EncoderVal = p->EncoderValMax - p->EncoderVal;
}

s32 ElectricalVal =
    ((p->EncoderVal - p->CalibOffset) * p->PolePairs)
    % encoder_counts;

if (ElectricalVal < 0)
    ElectricalVal += encoder_counts;

p->ElectricalAnglePU =
    (float)ElectricalVal / (float)encoder_counts;
```

这段代码依次完成方向修正、减去零偏、乘极对数和一圈回绕 最后得到 0 到 1 的电角度标幺值 再由查表函数得到正弦和余弦

#### 2.5.2 公共电流内环

电流、速度和位置三种模式只负责产生不同的 $i_q^*$ 最后都会进入同一个 `Sensoruse_Current_Inner_Loop()`

```c
foc->Iu = smp->IuReal;
foc->Iv = smp->IvReal;

Clark_Transform(foc);
Park_Transform(foc);

foc->IdLPF = foc->Id * foc->IdLPFFactor
           + foc->IdLPF * (1.0f - foc->IdLPFFactor);
foc->IqLPF = foc->Iq * foc->IqLPFFactor
           + foc->IqLPF * (1.0f - foc->IqLPFFactor);

mc->IqPid.Fbk = foc->IqLPF;
mc->IdPid.Fbk = foc->IdLPF;
PID_Control(&mc->IqPid);
PID_Control(&mc->IdPid);

foc->Uq = mc->IqPid.Out;
foc->Ud = mc->IdPid.Out;

IPark_Transform(foc);
foc->Ubus = smp->BusReal;
Calculate_SVPWM(foc);
```

我在 Park 变换后对 $i_d,i_q$ 做了低通滤波 再由两个 PI 计算 $U_d,U_q$ 当前两个电流 PI 都使用 $K_p=0.2$、$K_i=0.002$ 其中 $K_i$ 是每次调用直接累加的离散增益

$U_d,U_q$ 分别限幅到 $\pm V_{bus}/\sqrt3$ 目前还是逐轴限幅 不是电压矢量圆限幅 dq 解耦也还没有加入

#### 2.5.3 SVPWM 与 CCR

SVPWM 将 $U_\alpha,U_\beta$ 转换成三相比较值 当两个有效矢量时间超过限制时 我按比例同时缩小 $T_1,T_2$

```c
if (T1 + T2 > p->PwmLimit)
{
    T1 = p->PwmLimit * T1Temp / (T1Temp + T2Temp);
    T2 = p->PwmLimit * T2Temp / (T1Temp + T2Temp);
}

Ta = (p->PwmCycle - T1 - T2) * FOC_QUARTER;
Tb = Ta + T1 * FOC_HALF;
Tc = Tb + T2 * FOC_HALF;
```

TIM1 使用中心对齐计数 `ARR=4250` 所以 SVPWM 内部使用两倍计数空间 `PwmCycle=8500` `Ta,Tb,Tc` 在计算时通过 $1/4$ 和 $1/2$ 系数映射回 CCR 尺度 `motor_hal_write_pwm()` 最后再限制到 4250 并写入 CCR1、CCR2、CCR3

### 2.6 三种控制模式

#### 2.6.1 目标来源

我保留了电位器和命令两套目标来源 `use_adc_target=1` 时读取 `AdcBuff[1]` 否则使用串口或屏幕保存的目标

| 模式 | 电位器换算 |
|---|---|
| 电流 | $i_q^*=ADC_{pot}\times0.002$ |
| 速度 | $n_m^*=Direction\times ADC_{pot}\times0.5$ |
| 位置 | $Position^*=wrap(-ADC_{pot},4096)$ |

电流目标理论上可以达到约 8.19A 但公共限幅借用了 `SpdPid.OutMax=6A` 所以最后只能到 ±6A 这是我还没有拆掉的一处隐藏耦合

切换模式时 `motor_enter_mode()` 会清空各级 PI 和测速历史 进入位置模式时还会先把当前位置写成目标 避免继续使用上一次的位置指令

#### 2.6.2 电流模式

```text
Iq 目标
→ q 轴电流 PI
→ Uq
→ SVPWM
```

电流模式不经过速度环和位置环 我保持 $i_d^*=0$ 并直接控制 $i_q^*$ 当前公共目标范围约为 ±6A 这也是我最先用来检查电流方向和电角度的模式

#### 2.6.3 速度模式

```text
机械速度目标
→ 加减速规划
→ 速度 PI
→ Iq 目标
→ 电流内环
```

速度 PI 不直接生成 PWM 它的输出只是电流内环的 $i_q^*$

```c
spd->SpeedCalculateCnt++;
acc->TargetSpeed = mc->Speed.MechanicalSpeedSet;
T_Shaped_Acc_Dec(acc);

if (spd->SpeedCalculateCnt >= spd->SpeedDivisionFactor)
{
    spd->SpeedCalculateCnt = 0;
    spd->ElectricalPosThis = mc->EAngle.ElectricalAnglePU;
    Calculate_Speed(spd);

    mc->SpdPid.Ref = acc->SpeedOut;
    mc->SpdPid.Fbk = spd->ElectricalSpeedLPF;
    PID_Control(&mc->SpdPid);

    mc->IqPid.Ref = mc->SpdPid.Out;
    if (mc->IqPid.Ref > SENSORUSE_SPEED_IQ_LIMIT_A)
        mc->IqPid.Ref = SENSORUSE_SPEED_IQ_LIMIT_A;
    else if (mc->IqPid.Ref < -SENSORUSE_SPEED_IQ_LIMIT_A)
        mc->IqPid.Ref = -SENSORUSE_SPEED_IQ_LIMIT_A;
}
```

当前速度环约 10kHz 运行 目标按照 4800rpm/s 变化 ±5rpm 内按 0 处理 速度 PI 输出先限到 ±6A 最终 $i_q^*$ 再收紧到 ±2A

我当前的调参记录为低速 $K_p=0.005$、高速 $K_p=0.001$、$K_i=0.000002$

#### 2.6.4 位置模式

```text
单圈位置目标
→ 最近路径位置误差
→ 位置 PI
→ 速度 PI
→ 电流 PI
```

我把位置环和速度环放在两个独立分频块中 位置环约 5kHz 速度环约 10kHz

```c
Calculate_Position(pos, mc->EAngle.PolePairs);

s32 target =
    Position_NearestMechanicalSetpoint(pos->MechanicalPosRaw,
                                       pos->MechanicalPosSet,
                                       mechanical_counts);

mc->PosPid.Fbk = (float)pos->MechanicalPosRaw * pole_pairs;
mc->PosPid.Ref = (float)target * pole_pairs;
PID_Control(&mc->PosPid);
```

当前反馈由电角度连续展开后再除以极对数得到 因此可以记录相对多圈位置 但目标仍然是 0 到 4095 的单圈值 `Position_NearestMechanicalSetpoint()` 会选择离当前位置最近的等价目标

所以这版位置模式其实可以说是

***连续位置反馈 + 单圈位置目标 + 最近路径***

多圈位置以最近一次 `Home` 为起点 断电后不会保存 位置环当前是 $K_p=0.5$ 的纯 P 控制 输出范围 ±14000 对应电气转速尺度

### 2.7 命令（MIT协议 待提上日程）

我把串口接收和命令执行分开 串口中断只把字节写入环形缓冲区 命令拼接、查表和执行都在主循环完成 因此不会占用 20kHz 控制中断

UI 当前只用于调试 后续会删除 MIT 协议目前还没有接入

当前主要命令为

```text
/run  /stop
/mode sens cur|spd|pos
/set cur|spd|pos|ang <value>
/set pot on|off
/home  /clear  /status  /stream on|off
```

`/stream` 输出状态、错误、母线、电流、速度和编码器等数据 发送周期目前由主循环次数近似得到 不是严格定时

命令层和运行状态目前还没有完全收口 我这里还有三处问题

- `/run` 和部分 `/set` 在辨识完成前仍可能使能功率级
- 辨识过程中执行 `/stop` 后没有完整的重新启动路径
- `/clear` 只清错误状态 不会完整复位辨识内部变量

### 2.8 保护与错误处理

我当前真正实现的公共保护只有母线越界和软件过流

- $V_{bus}\le10V$ 或 $V_{bus}\ge40V$ 进入 `MOTOR_ERROR`
- 任意相电流绝对值超过 12A 进入 `MOTOR_ERROR`
- 速度模式中任意相电流达到 6.6A 时会清空测速、斜坡和 PI 状态 并把本周期 $i_q^*$ 置 0 但用户保存的速度目标没有清除 下一周期仍可能重新启动 这不是全局错误
- 位置模式没有使用速度模式的 ±2A 和 6.6A 软动作 主要受速度 PI 的 ±6A 输出和全局 12A 软件过流保护约束

`ADC_CALIB_ERR`、`ENCODER_ERR` 和 `TEMPERATURE_ERR` 目前只有枚举 还没有对应的检测代码 零偏阶段的保护空窗已经记录在 2.4.1

我在复查停止路径时发现 ERROR 和正常 `/stop` 并不等价

```c
/* ERROR：只清软件变量 */
case MOTOR_ERROR:
    mc->Foc.DutyCycleA = 0;
    mc->Foc.DutyCycleB = 0;
    mc->Foc.DutyCycleC = 0;
    m->output_enable = 0;
    break;

/* /stop：最后还会关闭功率级 */
void motor_hal_stop(motor_t *m)
{
    motor_stop(m);
    motor_hal_write_pwm(m);
    motor_hal_enable_output(m, 0U);
}
```

ERROR 没有调用 `board_motor_powerstage(0U)` 因此不会主动拉低三个 SD

`output_enable` 也不是全局硬件门控 `ADC_CALIB` 和 `MOTOR_IDENTIFY` 仍然允许提交 PWM 有感状态下禁止写 CCR 后各级 PI 也还会继续计算

当前 TIM1 使用 PWM2、死区为 0、Break 未启用 所以 CCR=0 不能直接等同于功率管完全关闭

移植到 Acid_Motor 前 还会补充快速硬件过流、温度检测和编码器有效性检查

### 2.9 移植到 Acid_Motor

这次解耦以后 `Lib/Motor` 内的 Clarke、Park、PID、SVPWM 和三级闭环不需要跟着硬件重写 移植时我主要修改板级实现和配置参数
