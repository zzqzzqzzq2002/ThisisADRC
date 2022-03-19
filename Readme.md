#**ADRC使用说明**
-----------------------
2021.12.9版本 @zzqzzqzzq2002::zzq2002=*Illya

####结构体说明
```C
typedef struct _ADRC_TypeDef
{
    float Target; // *目标值
    float Lastnot0_target; // 最后一次非零的目标值
    float Measure; // *测量值

    struct _TD
    {
        float R1; // 跟踪微分器当前跟踪值
        float R2; // 跟踪微分器当前跟踪速度
        float V1; // 跟踪微分器预估的跟踪速度
        float V2; // 跟踪微分器预估的跟踪加速度
        float R; // *跟踪微分器最大跟踪速度
    }TD; // 跟踪微分器子结构

    struct _NLC
    {
        float E0; // 当前预测误差
        float E1; // 当前预测误差的变化速度
        float E2; // 当前预测误差的变化加速度
        float U0; // 非线性混合律的原始控制量
        float U; // 考虑扰动得出的最终控制量
        float B; // *非线性混合律建模量与实际控制量的映射系数
        float Alpha_p; // *非线性混合律积分项的指数
        float Alpha_n; // *非线性混合律控制量的指数
        float Alpha_f; // *非线性混合率预测项的指数
        float Beta_p; // *非线性混合律积分项的权重
        float Beta_n; // *非线性混合律控制量的权重
        float Beta_f; // *非线性混合率预测项的权重
        float Delta; // *饱和函数的临界值
        float MaxOutputClamp; // *最大输出范围
    }NLC;

    struct _ESO
    {
        float Z1; // 扩张状态观测器观测的当前状态值
        float Z2; // 扩张状态观测器观测的当前变化速度
        float Z3; // 扩张状态观测器观测的当前总系统扰动
        float Alpha1; // *观测误差的指数
        float Alpha2; // *观测误差速度的指数
        float Alpha3; // *观测误差加速度与系统扰动的指数
        float Omega; // *系统观测带宽
        float Delta; // *饱和函数的临界值
    }ESO;

    struct _Time
    {
        uint32_t Time_p; // 上次控制的时刻
        uint32_t Time_n; // 本次控制的时刻
        float Dtime; // 两次控制之间的时间
    }Time;

    float Output; // 本次最终输出
    float Last_output; // 最后一次的输出
    float MaxErr_recoder; // 最大误差记录

    struct _Other
    {
        float DeadBand; // *控制死区范围
    }Other;

    void (*TD_cal)(struct _ADRC_TypeDef * adrc); // 计算跟踪微分器部分
    void (*NLC_cal)(struct _ADRC_TypeDef * adrc); // 计算非线性混合律部分
    void (*ESO_cal)(struct _ADRC_TypeDef * adrc); // 计算扩张状态观测器部分
    void (*fullCal)(struct _ADRC_TypeDef * adrc); // 计算整个自抗扰控制器
    void (*getTimeStamp)(struct _ADRC_TypeDef * adrc); // 记录当前的时间戳
    void (*param_init)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init); // 初始化参数
    void (*inputStatus)(struct _ADRC_TypeDef * adrc, float target, float feedback); // 输入当前状态
    float (*getlastOutput)(struct _ADRC_TypeDef * adrc); // 获取最后一次的输出
    float (*getOutput)(struct _ADRC_TypeDef * adrc, float target, float feedback); // 根据当前状态给出输出
    void (*reset_TD)(struct _ADRC_TypeDef * adrc, float new_R); // 重设跟踪微分器参数
    void (*reset_NLC)(struct _ADRC_TypeDef * adrc, ADRC_InitNLC new_NLC); // 重设非线性混合律参数
    void (*reset_ESO)(struct _ADRC_TypeDef * adrc, ADRC_InitESO new_ESO); // 重设扩张状态观测器参数
    void (*reset_Other)(struct _ADRC_TypeDef * adrc, float new_DeadBand); // 重设其他参数
    void (*reset_allParam)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init); // 重设所有参数
    void (*restart)(struct _ADRC_TypeDef * adrc); // 重启自抗扰控制器
    void (*clean_recoder)(struct _ADRC_TypeDef * adrc); // 清除误差记录
}ADRC;
```

其余各初始化结构与之类似,不再赘述.
```C
extern const ADRC_Init Default_3508_speed;
extern const ADRC_Init Default_3508_position;
extern const ADRC_Init Default_6020_speed;
extern const ADRC_Init Default_6020_position;
extern const ADRC_Init Default_2006_speed;
extern const ADRC_Init Default_2006_position;
```
同时提供了三种电机分别使用速度闭环和位置闭环时的标准结构(待完善,有bug),方便直接调用.

####ADRC对象构造函数
```C
ADRC zepi_create_ADRC(void);
ADRC create_ADRC(void);
```

####下面简单介绍一下怎么自定义调参
	1.将 TD.R 设置为一个很大的值; NLC.MaxOutputClamp 设置成被控系统最大输入值,如果怕直接疯掉建议调小一点.
	2.将 NLC.Alpha_p / NLC.Alpha_n / NLC.Alpha_f 设置为1.
	3.将 NLC.Delta / ESO.Delta / Other.DeadBand 设置为0.
	4.将 ESO.Alpha1 / ESO.Alpha2 / ESO.Alpha3 分别设置为1, 0.5, 0.25.
	5.NLC.B 根据被控对象自己估算一下输出量与实际状态的对应系数填上.
	6.NLC.Beta_n 设置成和 PID 控制时的 kP, NLC.Beta_p / NLC.Beta_f 设置为0.
	7.给一个较小的目标值, 慢慢加大ESO.Omega, 且在确保系统不发生振荡的前提下尽可能调大.
其余细节的调参自己摸索,开始愉快的玩耍吧.