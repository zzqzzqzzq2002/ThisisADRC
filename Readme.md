#**ADRCʹ��˵��**
-----------------------
2021.12.9�汾 @zzqzzqzzq2002::zzq2002=*Illya

####�ṹ��˵��
```C
typedef struct _ADRC_TypeDef
{
    float Target; // *Ŀ��ֵ
    float Lastnot0_target; // ���һ�η����Ŀ��ֵ
    float Measure; // *����ֵ

    struct _TD
    {
        float R1; // ����΢������ǰ����ֵ
        float R2; // ����΢������ǰ�����ٶ�
        float V1; // ����΢����Ԥ���ĸ����ٶ�
        float V2; // ����΢����Ԥ���ĸ��ټ��ٶ�
        float R; // *����΢�����������ٶ�
    }TD; // ����΢�����ӽṹ

    struct _NLC
    {
        float E0; // ��ǰԤ�����
        float E1; // ��ǰԤ�����ı仯�ٶ�
        float E2; // ��ǰԤ�����ı仯���ٶ�
        float U0; // �����Ի���ɵ�ԭʼ������
        float U; // �����Ŷ��ó������տ�����
        float B; // *�����Ի���ɽ�ģ����ʵ�ʿ�������ӳ��ϵ��
        float Alpha_p; // *�����Ի���ɻ������ָ��
        float Alpha_n; // *�����Ի���ɿ�������ָ��
        float Alpha_f; // *�����Ի����Ԥ�����ָ��
        float Beta_p; // *�����Ի���ɻ������Ȩ��
        float Beta_n; // *�����Ի���ɿ�������Ȩ��
        float Beta_f; // *�����Ի����Ԥ�����Ȩ��
        float Delta; // *���ͺ������ٽ�ֵ
        float MaxOutputClamp; // *��������Χ
    }NLC;

    struct _ESO
    {
        float Z1; // ����״̬�۲����۲�ĵ�ǰ״ֵ̬
        float Z2; // ����״̬�۲����۲�ĵ�ǰ�仯�ٶ�
        float Z3; // ����״̬�۲����۲�ĵ�ǰ��ϵͳ�Ŷ�
        float Alpha1; // *�۲�����ָ��
        float Alpha2; // *�۲�����ٶȵ�ָ��
        float Alpha3; // *�۲������ٶ���ϵͳ�Ŷ���ָ��
        float Omega; // *ϵͳ�۲����
        float Delta; // *���ͺ������ٽ�ֵ
    }ESO;

    struct _Time
    {
        uint32_t Time_p; // �ϴο��Ƶ�ʱ��
        uint32_t Time_n; // ���ο��Ƶ�ʱ��
        float Dtime; // ���ο���֮���ʱ��
    }Time;

    float Output; // �����������
    float Last_output; // ���һ�ε����
    float MaxErr_recoder; // �������¼

    struct _Other
    {
        float DeadBand; // *����������Χ
    }Other;

    void (*TD_cal)(struct _ADRC_TypeDef * adrc); // �������΢��������
    void (*NLC_cal)(struct _ADRC_TypeDef * adrc); // ��������Ի���ɲ���
    void (*ESO_cal)(struct _ADRC_TypeDef * adrc); // ��������״̬�۲�������
    void (*fullCal)(struct _ADRC_TypeDef * adrc); // ���������Կ��ſ�����
    void (*getTimeStamp)(struct _ADRC_TypeDef * adrc); // ��¼��ǰ��ʱ���
    void (*param_init)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init); // ��ʼ������
    void (*inputStatus)(struct _ADRC_TypeDef * adrc, float target, float feedback); // ���뵱ǰ״̬
    float (*getlastOutput)(struct _ADRC_TypeDef * adrc); // ��ȡ���һ�ε����
    float (*getOutput)(struct _ADRC_TypeDef * adrc, float target, float feedback); // ���ݵ�ǰ״̬�������
    void (*reset_TD)(struct _ADRC_TypeDef * adrc, float new_R); // �������΢��������
    void (*reset_NLC)(struct _ADRC_TypeDef * adrc, ADRC_InitNLC new_NLC); // ��������Ի���ɲ���
    void (*reset_ESO)(struct _ADRC_TypeDef * adrc, ADRC_InitESO new_ESO); // ��������״̬�۲�������
    void (*reset_Other)(struct _ADRC_TypeDef * adrc, float new_DeadBand); // ������������
    void (*reset_allParam)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init); // �������в���
    void (*restart)(struct _ADRC_TypeDef * adrc); // �����Կ��ſ�����
    void (*clean_recoder)(struct _ADRC_TypeDef * adrc); // �������¼
}ADRC;
```

�������ʼ���ṹ��֮����,����׸��.
```C
extern const ADRC_Init Default_3508_speed;
extern const ADRC_Init Default_3508_position;
extern const ADRC_Init Default_6020_speed;
extern const ADRC_Init Default_6020_position;
extern const ADRC_Init Default_2006_speed;
extern const ADRC_Init Default_2006_position;
```
ͬʱ�ṩ�����ֵ���ֱ�ʹ���ٶȱջ���λ�ñջ�ʱ�ı�׼�ṹ(������,��bug),����ֱ�ӵ���.

####ADRC�����캯��
```C
ADRC zepi_create_ADRC(void);
ADRC create_ADRC(void);
```

####����򵥽���һ����ô�Զ������
	1.�� TD.R ����Ϊһ���ܴ��ֵ; NLC.MaxOutputClamp ���óɱ���ϵͳ�������ֵ,�����ֱ�ӷ�������Сһ��.
	2.�� NLC.Alpha_p / NLC.Alpha_n / NLC.Alpha_f ����Ϊ1.
	3.�� NLC.Delta / ESO.Delta / Other.DeadBand ����Ϊ0.
	4.�� ESO.Alpha1 / ESO.Alpha2 / ESO.Alpha3 �ֱ�����Ϊ1, 0.5, 0.25.
	5.NLC.B ���ݱ��ض����Լ�����һ���������ʵ��״̬�Ķ�Ӧϵ������.
	6.NLC.Beta_n ���óɺ� PID ����ʱ�� kP, NLC.Beta_p / NLC.Beta_f ����Ϊ0.
	7.��һ����С��Ŀ��ֵ, �����Ӵ�ESO.Omega, ����ȷ��ϵͳ�������񵴵�ǰ���¾����ܵ���.
����ϸ�ڵĵ����Լ�����,��ʼ������ˣ��.