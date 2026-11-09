/*
 * auto_car.c
 * PIC16F877A @ 4MHz
 * 功能：自动行驶小车，任务一~四，按键选择任务
 *
 * ============================================================
 * 接线说明
 * ============================================================
 * 【JY62 陀螺仪】
 *   JY62 TX  -->  RC7 (USART RX)
 *   JY62 GND -->  GND
 *   JY62 VCC -->  3.3V
 *
 * 【TB6612 电机驱动】
 *   PWMA  <--  RC2 (CCP1，左电机速度)
 *   AIN1  <--  RB4 (左电机方向1)
 *   AIN2  <--  RB5 (左电机方向2)
 *   PWMB  <--  RC1 (CCP2，右电机速度)
 *   BIN1  <--  RB6 (右电机方向1)
 *   BIN2  <--  RB7 (右电机方向2)
 *   STBY  -->  VCC
 *
 * 【八路灰度循迹（地址选择式）】
 *   OUT  -->  RA0
 *   AD0  <--  RA1
 *   AD1  <--  RA2
 *   AD2  <--  RA3
 *
 * 【按键（RB口，低电平有效，内部上拉，无需外接电阻）】
 *   RB0  : 停止/复位
 *   RB1  : 启动任务一
 *   RB2  : 启动任务二
 *   RB3  : 启动任务三（长按>1s=任务四）
 *
 * 【声光提示】
 *   RE0      -->  蜂鸣器（有源，高电平响）
 *   RD0~RD7  -->  LED（低电平点亮，高电平熄灭）
 *
 * ============================================================
 * 按键说明（RB口内部上拉，低电平有效，无需外接电阻）
 * ============================================================
 *   RB0  : 停止/复位，任何状态下按下立即停车回待机
 *   RB1  : 直接启动任务一
 *   RB2  : 直接启动任务二
 *   RB3  : 短按=启动任务三，长按>1s=启动任务四
 *
 * ============================================================
 * LED说明（低电平点亮，高电平熄灭）
 * ============================================================
 *   待机：RD0慢闪（500ms周期）表示就绪
 *   运行中：RD0~RD3显示任务编号（对应位清零亮），RD4~RD7熄灭
 *   到点提示：RD4~RD7全亮（低电平），持续BEEP_TIME_MS
 *   完成：RD0~RD7全亮，等待RB0复位
 *
 * ============================================================
 * 任务路径
 * ============================================================
 *   任务一：A→直线→B 停车
 *   任务二：A→直线→B→弧BC→C→直线→D→弧DA→A 停车
 *   任务三：A→旋转对准→直线→C→弧CB→B→直线→D→弧DA→A 停车
 *   任务四：任务三路径×4圈
 *
 * ============================================================
 * 场地坐标与角度计算
 * ============================================================
 *   场地220×120cm，以左上角为原点，x向右，y向下
 *   A=(40,0)  B=(180,0)  C=(180,120)  D=(40,120)
 *   AB方向：水平向右，定义为Yaw基准0°
 *   AC方向：右下偏角 = arctan(120/140) ≈ 40.6°（顺时针）
 *   JY62顺时针转Yaw减小（需实测确认，若相反修改SPIN_TARGET符号）
 *   AC偏角原始值 = -40.6 * 32768 / 180 ≈ -7396 raw
 *
 * ============================================================
 * PWM参数
 * ============================================================
 *   TMR2预分频1:1，PR2=249 → PWM频率4kHz
 *   占空比范围0~1000（10bit）
 */

#include <pic.h>

__CONFIG(0xFF29);

/* ============================================================
 * 可调参数（根据实测修改）
 * ============================================================ */
#define BASE_PWM        500     /* 直线基础速度 */
#define ARC_FAST        530     /* 弧线外侧轮速度 */
#define ARC_SLOW        220     /* 弧线内侧轮速度 */
#define ARC_HARD_FAST   620     /* 边缘检测到线时外侧轮强修正速度 */
#define ARC_HARD_SLOW   120     /* 边缘检测到线时内侧轮强修正速度 */
#define SPIN_SPEED      240     /* 原地旋转速度 */
#define KP              3       /* 直线P控制系数 */
#define STRAIGHT_DEADBAND 80    /* 约0.4度内不修正，避免抖动 */
#define STRAIGHT_ERR_DIV  64    /* Yaw raw缩放，避免小偏差过度修正 */
#define STRAIGHT_MAX_CORR 180   /* 单侧最大PWM修正量 */
#define ARC_MIN_YAW_RAW  26400 /* 弧线至少转过约145度后才允许判端点 */
#define ARC_LOST_CONFIRM 8     /* 全白连续确认次数，数值越小到点/停止越快 */
#define ARC_ENTRY_YAW_RAW 1200 /* 任务二刚入弧先右转约7度，避免B点后直冲 */
#define ARC_ENTRY_CB_YAW_RAW 1800 /* 任务三C->B刚入弧先左转约10度 */
#define ALIGN_BASE_PWM   320   /* C->D开头低速修正速度 */
#define ALIGN_MAX_CORR   220   /* C->D开头最大PWM修正量 */
#define ALIGN_DONE_RAW   900   /* 约5度内退出强修正 */
#define STRAIGHT_IGNORE_MS 700 /* 进入直线后先忽略黑线，防止重复判点 */
#define GRAY_SETTLE_LOOPS 80  /* 地址切换后等待OUT稳定，防止读到假全白 */
#define GRAY_SAMPLE_GAP   20  /* 同一路多次采样间隔 */
#define GRAY_SAMPLE_COUNT 3   /* 每一路采样次数 */
#define GRAY_SAMPLE_HIGH  2   /* 多数为高才认为该路检测到黑线 */
#define GRAY_BLACK_HIGH  1   /* 1=黑线输出高电平，0=黑线输出低电平 */


/* AC方向偏角，JY62原始值，顺时针为负
 * 若实测方向相反，改为正值 +7396 */
#define AC_SPIN_TARGET  (-6400)

/* 旋转到位容差：约±3° = 3*32768/180 ≈ ±546 raw */
#define SPIN_TOLERANCE  750


/* 声光提示持续时间(ms) */
#define BEEP_TIME_MS    200

/* 长按任务三按键触发任务四的时间(ms) */
#define LONG_PRESS_MS   1000

/* ============================================================
 * 引脚定义
 * ============================================================ */
/* 电机方向 */
#define AIN1    RB4
#define AIN2    RB5
#define BIN1    RB6
#define BIN2    RB7

/* 灰度传感器（RA口，OUT/AD0/AD1/AD2地址选择式） */
#define GRAY_OUT    RA0
#define GRAY_AD0    RA1
#define GRAY_AD1    RA2
#define GRAY_AD2    RA3

/* 按键（RB口，低电平有效，内部上拉） */
#define BTN_RESET   RB0     /* 停止/复位 */
#define BTN_TASK1   RB1     /* 启动任务一 */
#define BTN_TASK2   RB2     /* 启动任务二 */
#define BTN_TASK3   RB3     /* 启动任务三（长按=任务四） */

/* 声光 */
#define BUZZER      RE0
#define LED_TASK    PORTD       /* RD0~RD3任务指示，RD4~RD7到点提示 */

/* ============================================================
 * 顶层模式定义
 * ============================================================ */
typedef enum {
    MODE_IDLE = 0,
    MODE_TASK1,
    MODE_TASK2,
    MODE_TASK3,
    MODE_TASK4,
    MODE_DONE
} CarMode;

/* ============================================================
 * 路径段类型
 * ============================================================ */
typedef enum {
    SEG_SPIN = 0,   /* 原地旋转到目标角度 */
    SEG_STRAIGHT,   /* 直线（陀螺仪P控制） */
    SEG_ARC,        /* 弧线（循迹跟线） */
    SEG_STOP        /* 终止 */
} SegType;

typedef struct {
    SegType type;
    int     spin_target;    /* SEG_SPIN：目标偏转角(raw)，相对于初始基准 */
    char    label;          /* 到达点标识，声光提示用 */
} Segment;

/* ============================================================
 * 路径表
 * ============================================================ */

/* 任务一：A→B */
const Segment route1[] = {
    {SEG_STRAIGHT, 0,              'B'},
    {SEG_STOP,     0,              'B'}
};
#define ROUTE1_LEN 2

/* 任务二：A→B→弧BC→C→D→弧DA→A */
const Segment route2[] = {
    {SEG_STRAIGHT, 0,              'B'},
    {SEG_ARC,      0,              'C'},
    {SEG_STRAIGHT, 0,              'D'},
    {SEG_ARC,      0,              'A'},
    {SEG_STOP,     0,              'A'}
};
#define ROUTE2_LEN 5

/* 任务三/四：A→旋转→直线C→弧CB→B→直线D→弧DA→A */
const Segment route3[] = {
    {SEG_SPIN,     AC_SPIN_TARGET, 'A'},    /* 原地旋转对准AC方向 */
    {SEG_STRAIGHT, 0,              'C'},    /* 直线到C */
    {SEG_ARC,      0,              'B'},    /* 弧线CB */
    {SEG_STRAIGHT, 0,              'D'},    /* 直线到D */
    {SEG_ARC,      0,              'A'},    /* 弧线DA */
    {SEG_STOP,     0,              'A'}
};
#define ROUTE3_LEN 6

/* ============================================================
 * 全局变量
 * ============================================================ */
/* 顶层模式 */
volatile CarMode g_mode = MODE_IDLE;

/* 路径执行器 */
const Segment   *g_route     = route1;
unsigned char    g_route_len = ROUTE1_LEN;
unsigned char    g_step      = 0;
unsigned char    g_lap       = 0;       /* 任务四圈数计数 */
unsigned char    g_arc_lost_cnt = 0;    /* 弧线端点连续确认计数 */
char             g_arc_turn_ready = 0;  /* 弧线转角已达到端点判定阈值 */
signed char      g_gray_last_side = 1;   /* 灰度丢线时沿上次方向找线 */
char             g_straight_wait_clear = 0; /* 直线段先等离开上一条线 */
unsigned int     g_straight_start_ms = 0;

/* 段内子状态（用于BEEP后跳转） */
/* SEG_BEEP结束后直接step++，不需要额外变量 */

/* 陀螺仪（USART中断更新） */
volatile int  g_yaw       = 0;
volatile char g_new_frame = 0;

/* USART接收缓冲 */
#define FRAME_LEN 11
volatile unsigned char g_uart_buf[FRAME_LEN];
volatile unsigned char g_uart_idx  = 0;
volatile char          g_uart_sync = 0;

/* 定时器（TMR1中断更新） */
volatile unsigned int g_timer_ms = 0;

/* 段执行基准角 */
int g_yaw_ref      = 0;     /* 直线段/旋转段：出发时的Yaw基准 */
int g_arc_yaw_ref  = 0;     /* 弧线段：入弧时的Yaw基准 */
int g_ab_yaw_ref   = 0;     /* 任务二AB实际直线基准 */
int g_bd_yaw_ref   = 0;     /* 任务三B->D理论直线基准 */
char g_align_cd    = 0;     /* 1=C->D开头低速强修正 */

/* BEEP计时 */
unsigned int g_beep_start = 0;
char         g_in_beep    = 0;  /* 1=正在BEEP中 */


/* 待机闪烁计时 */
unsigned int g_idle_blink_ts = 0;
char         g_idle_blink_state = 0;    /* 0=灭，1=亮 */

/* 旋转基准 */
int g_spin_yaw_ref = 0;

/* 初始绝对Yaw基准（上电时记录，用于计算AC段绝对目标角） */
int g_yaw_init = 0;

/* ============================================================
 * 工具函数
 * ============================================================ */
int abs_val(int x) { return (x < 0) ? -x : x; }

/* Yaw角差，归一化到 -32768~+32767 (对应-180°~+180°) */
int yaw_diff(int current, int ref)
{
    long diff = (long)current - (long)ref;
    if (diff >  32767L) diff -= 65536L;
    if (diff < -32768L) diff += 65536L;
    return (int)diff;
}

int yaw_add_raw(int ref, int delta)
{
    int value = ref + delta;
    if (delta > 0 && value < ref) value -= 32768;
    if (delta < 0 && value > ref) value += 32768;
    return value;
}

/* 返回与ref相差180度的Yaw原始值 */

int yaw_opposite(int ref)
{
    if (ref >= 0) return ref - 32767 - 1;
    return ref + 32767 + 1;
}

unsigned int yaw_abs_diff(int current, int ref)
{
    long diff = (long)current - (long)ref;
    if (diff >  32767L) diff -= 65536L;
    if (diff < -32768L) diff += 65536L;
    if (diff < 0) diff = -diff;
    return (unsigned int)diff;
}

/* ============================================================
 * PWM 初始化
 * ============================================================ */
void pwm_init(void)
{
    TRISC1  = 0;
    TRISC2  = 0;
    PR2     = 249;
    T2CON   = 0x04;
    CCP1CON = 0x0C;
    CCPR1L  = 0;
    CCP2CON = 0x0C;
    CCPR2L  = 0;
}

void pwm_set(unsigned int left, unsigned int right)
{
    if (left  > 1000) left  = 1000;
    if (right > 1000) right = 1000;
    CCPR1L  = (unsigned char)(left  >> 2);
    CCP1CON = (CCP1CON & 0xCF) | (unsigned char)((left  & 0x03) << 4);
    CCPR2L  = (unsigned char)(right >> 2);
    CCP2CON = (CCP2CON & 0xCF) | (unsigned char)((right & 0x03) << 4);
}

/* ============================================================
 * 电机控制
 * ============================================================ */
/* 两轮前进 */
void motor_forward(unsigned int l, unsigned int r)
{
    AIN1 = 1; AIN2 = 0;
    BIN1 = 1; BIN2 = 0;
    pwm_set(l, r);
}

/* 原地左转：左轮后退，右轮前进 */
void motor_spin_left(unsigned int spd)
{
    AIN1 = 0; AIN2 = 1;    /* 左轮反转 */
    BIN1 = 1; BIN2 = 0;    /* 右轮正转 */
    pwm_set(spd, spd);
}

/* 原地右转：左轮前进，右轮后退 */
void motor_spin_right(unsigned int spd)
{
    AIN1 = 1; AIN2 = 0;    /* 左轮正转 */
    BIN1 = 0; BIN2 = 1;    /* 右轮反转 */
    pwm_set(spd, spd);
}

void motor_stop(void)
{
    AIN1 = 0; AIN2 = 0;
    BIN1 = 0; BIN2 = 0;
    pwm_set(0, 0);
}

void motor_init(void)
{
    TRISB4 = 0; TRISB5 = 0;
    TRISB6 = 0; TRISB7 = 0;
    motor_stop();
}

/* ============================================================
 * USART 初始化（9600bps，接收中断）
 * ============================================================ */
void uart_init(void)
{
    TRISC7 = 1;
    TRISC6 = 0;
    SPBRG  = 25;
    BRGH   = 1;
    SYNC   = 0;
    SPEN   = 1;
    CREN   = 1;
    TXEN   = 0;
    RCIE   = 1;
    PEIE   = 1;
    GIE    = 1;
}

/* ============================================================
 * TMR1 初始化（1ms中断）
 * ============================================================ */
void timer1_init(void)
{
    T1CON  = 0x00;
    TMR1H  = 0xFC;
    TMR1L  = 0x18;
    TMR1IE = 1;
    TMR1ON = 1;
}

/* ============================================================
 * 中断服务程序
 * ============================================================ */
void interrupt isr(void)
{
    /* USART接收：逐字节拼帧 */
    if (RCIF && RCIE)
    {
        unsigned char rx;
        unsigned char i;
        unsigned char sum;

        if (OERR) { CREN = 0; CREN = 1; }
        rx = RCREG;

        if (!g_uart_sync)
        {
            if (rx == 0x55) {
                g_uart_buf[0] = 0x55;
                g_uart_idx    = 1;
                g_uart_sync   = 1;
            }
        }
        else
        {
            g_uart_buf[g_uart_idx++] = rx;

            if (g_uart_idx == 2) {
                if (rx != 0x53) {
                    g_uart_sync = 0;
                    g_uart_idx  = 0;
                }
            }
            else if (g_uart_idx >= FRAME_LEN)
            {
                sum = 0;
                for (i = 0; i < FRAME_LEN - 1; i++)
                    sum += g_uart_buf[i];

                if (sum == g_uart_buf[FRAME_LEN - 1]) {
                    g_yaw       = (int)((unsigned int)g_uart_buf[7] << 8
                                        | g_uart_buf[6]);
                    g_new_frame = 1;
                }
                g_uart_sync = 0;
                g_uart_idx  = 0;
            }
        }
    }

    /* TMR1：1ms计时 */
    if (TMR1IF && TMR1IE)
    {
        TMR1IF = 0;
        TMR1H  = 0xFC;
        TMR1L  = 0x18;
        g_timer_ms++;
    }
}

/* ============================================================
 * 八路灰度传感器（地址选择式）
 * AD2~AD0选择通道0~7，OUT输出该通道状态
 * S1=bit7 ... S8=bit0
 * 检测到黑线=1
 * ============================================================ */
unsigned char gray_read(void)
{
    unsigned char data = 0;
    unsigned char i;
    unsigned char sample;
    unsigned char high_cnt;
    volatile unsigned char settle;

    TRISA0 = 1;     /* RA0/OUT 输入 */
    TRISA1 = 0;     /* RA1/AD0 输出 */
    TRISA2 = 0;     /* RA2/AD1 输出 */
    TRISA3 = 0;     /* RA3/AD2 输出 */

    for (i = 0; i < 8; i++) {
        GRAY_AD0 = (i & 0x01) ? 1 : 0;
        GRAY_AD1 = (i & 0x02) ? 1 : 0;
        GRAY_AD2 = (i & 0x04) ? 1 : 0;

        for (settle = 0; settle < GRAY_SETTLE_LOOPS; settle++) { ; }

        high_cnt = 0;
        for (sample = 0; sample < GRAY_SAMPLE_COUNT; sample++) {
            if (GRAY_OUT)
                high_cnt++;
            for (settle = 0; settle < GRAY_SAMPLE_GAP; settle++) { ; }
        }

        if (GRAY_BLACK_HIGH)
        {
            if (high_cnt >= GRAY_SAMPLE_HIGH)
                data |= (unsigned char)(0x80 >> i);
        }
        else
        {
            if (high_cnt < GRAY_SAMPLE_HIGH)
                data |= (unsigned char)(0x80 >> i);
        }
    }
    return data;
}

/* 中间6路(S2~S7)任意一路亮=有线，掩码0x7E */
char gray_line_detected(void)
{
    return (gray_read() & 0x7E) ? 1 : 0;
}

/* 8路任意一路检测到黑线=1 */
char gray_any_line_detected(void)
{
    return gray_read() ? 1 : 0;
}

/* 线偏向：-1=左，0=中，1=右 */
signed char gray_line_side(void)
{
    unsigned char data       = gray_read();
    unsigned char left_bits  = (data >> 4) & 0x07;   /* S2~S4 */
    unsigned char mid_bits   = (data >> 3) & 0x03;   /* S4~S5 */
    unsigned char right_bits = (data >> 1) & 0x07;   /* S5~S7 */
    unsigned char lc = 0, rc = 0, mask;

    if (!data)
        return g_gray_last_side;

    if (data & 0x80) {          /* S1最左边检测到线，强左修正 */
        g_gray_last_side = -2;
        return -2;
    }
    if (data & 0x01) {          /* S8最右边检测到线，强右修正 */
        g_gray_last_side = 2;
        return 2;
    }

    for (mask = 0x04; mask; mask >>= 1) {
        if (left_bits  & mask) lc++;
        if (right_bits & mask) rc++;
    }

    if (mid_bits == 0x03 || lc == rc) return 0;

    if (lc > rc) {
        g_gray_last_side = -1;
        return -1;
    }

    g_gray_last_side = 1;
    return 1;
}


signed char gray_line_side_from_data(unsigned char data)
{
    unsigned char left_bits  = (data >> 4) & 0x07;   /* S2~S4 */
    unsigned char mid_bits   = (data >> 3) & 0x03;   /* S4~S5 */
    unsigned char right_bits = (data >> 1) & 0x07;   /* S5~S7 */
    unsigned char lc = 0, rc = 0, mask;

    if (!data)
        return g_gray_last_side;

    if (data & 0x80) {
        g_gray_last_side = -2;
        return -2;
    }
    if (data & 0x01) {
        g_gray_last_side = 2;
        return 2;
    }

    for (mask = 0x04; mask; mask >>= 1) {
        if (left_bits  & mask) lc++;
        if (right_bits & mask) rc++;
    }

    if (mid_bits == 0x03 || lc == rc) return 0;

    if (lc > rc) {
        g_gray_last_side = -1;
        return -1;
    }

    g_gray_last_side = 1;
    return 1;
}

void arc_control_side(signed char side)
{
    if      (side ==  0) motor_forward(ARC_FAST, ARC_FAST);
    else if (side == -2) motor_forward(ARC_HARD_SLOW, ARC_HARD_FAST);
    else if (side ==  2) motor_forward(ARC_HARD_FAST, ARC_HARD_SLOW);
    else if (side == -1) motor_forward(ARC_SLOW, ARC_FAST);
    else                 motor_forward(ARC_FAST, ARC_SLOW);
}/* ============================================================
 * 声光控制
 * ============================================================ */
/* 到点提示：RD1亮（清零），蜂鸣器响；RD0保持状态灯 */
void beep_led_on(void)
{
    BUZZER = 1;
    PORTD  = (unsigned char)((PORTD | 0xFC) & ~0x02);
}

/* 提示结束：RD1灭（置1），蜂鸣器停；RD0保持状态灯 */
void beep_led_off(void)
{
    BUZZER = 0;
    PORTD  = (unsigned char)(PORTD | 0x02);
}

/* 两灯方案：运行中只亮RD0，RD1留给到点提示 */
void led_show_task(char task_num)
{
    task_num = task_num;
    PORTD = 0xFE;
}

/* 待机闪烁：RD0每500ms切换一次 */
void idle_blink_update(void)
{
    if ((unsigned int)(g_timer_ms - g_idle_blink_ts) >= 500)
    {
        g_idle_blink_ts    = g_timer_ms;
        g_idle_blink_state = !g_idle_blink_state;
        if (g_idle_blink_state)
            PORTD = (unsigned char)(PORTD & ~0x01); /* RD0清零=亮 */
        else
            PORTD = (unsigned char)(PORTD |  0x01); /* RD0置1=灭 */
    }
}

/* ============================================================
 * 直线P控制
 * ============================================================ */
void straight_control_base(unsigned int base_pwm, int max_corr)
{
    int error        = yaw_diff(g_yaw, g_yaw_ref);
    int error_scaled = 0;
    int correction   = 0;
    int left_pwm;
    int right_pwm;

    if (abs_val(error) > STRAIGHT_DEADBAND)
    {
        error_scaled = error / STRAIGHT_ERR_DIV;
        correction   = KP * error_scaled;

        if (correction >  max_corr) correction =  max_corr;
        if (correction < -max_corr) correction = -max_corr;
    }

    left_pwm  = (int)base_pwm + correction;
    right_pwm = (int)base_pwm - correction;

    if (left_pwm  > 1000) left_pwm  = 1000;
    if (left_pwm  < 100 ) left_pwm  = 100;
    if (right_pwm > 1000) right_pwm = 1000;
    if (right_pwm < 100 ) right_pwm = 100;

    motor_forward((unsigned int)left_pwm, (unsigned int)right_pwm);
}

void straight_control(void)
{
    straight_control_base(BASE_PWM, STRAIGHT_MAX_CORR);
}

/* ============================================================
 * 弧线循迹控制
 * ============================================================ */
void arc_control(void)
{
    arc_control_side(gray_line_side());
}

/* ============================================================
 * 进入下一段的统一处理
 * 停车，声光提示，step推进
 * ============================================================ */
void enter_beep_and_next(void)
{
    const Segment *seg = &g_route[g_step];

    motor_stop();
    beep_led_on();
    g_beep_start = g_timer_ms;
    g_in_beep    = 1;   /* 标志位：正在BEEP，runner_run等待 */
    g_step++;           /* 预先推进，BEEP结束后直接执行新段 */
    if (g_step < g_route_len && g_route[g_step].type == SEG_STRAIGHT)
    {
        g_straight_wait_clear = 1;
        g_straight_start_ms = g_timer_ms;
    }
}


/* ============================================================
 * 段执行器：run_spin
 * 原地旋转到目标角度（相对于g_spin_yaw_ref的偏转量）
 * ============================================================ */
void run_spin(void)
{
    const Segment *seg = &g_route[g_step];
    int current_diff   = yaw_diff(g_yaw, g_spin_yaw_ref);
    int remaining      = seg->spin_target - current_diff;

    if (abs_val(remaining) <= SPIN_TOLERANCE)
    {
        /* 到达目标角度，用绝对目标角作为直线基准，消除旋转停止误差 */
        GIE = 0;
        g_yaw_ref = yaw_add_raw(g_yaw_init, seg->spin_target);
        GIE = 1;
        enter_beep_and_next();
    }
    else if (remaining < 0)
    {
        /* 还需要顺时针转（Yaw减小方向） */
        motor_spin_right(SPIN_SPEED);
    }
    else
    {
        /* 还需要逆时针转（Yaw增大方向） */
        motor_spin_left(SPIN_SPEED);
    }
}

/* ============================================================
 * 段执行器：run_straight
 * 直线行驶，循迹检测到线时切换
 * ============================================================ */
void run_straight(void)
{
    if (g_new_frame) {
        g_new_frame = 0;
        if (g_align_cd)
        {
            straight_control_base(ALIGN_BASE_PWM, ALIGN_MAX_CORR);
            if (abs_val(yaw_diff(g_yaw, g_yaw_ref)) <= ALIGN_DONE_RAW)
                g_align_cd = 0;
        }
        else
        {
            straight_control();
        }
    }

    if ((unsigned int)(g_timer_ms - g_straight_start_ms) < STRAIGHT_IGNORE_MS)
        return;

    if (g_straight_wait_clear)
    {
        if (gray_line_detected())
            return;
        g_straight_wait_clear = 0;
    }

    /* 循迹检测到线：到达下一个点 */
    if (gray_line_detected())
    {
        GIE = 0;
        if (g_mode == MODE_TASK2 && g_step == 0)
            g_ab_yaw_ref = g_yaw_ref;
        g_arc_yaw_ref = g_yaw;  /* 记录入弧Yaw基准 */
        g_arc_lost_cnt = 0;
        g_arc_turn_ready = 0;
        if ((g_mode == MODE_TASK3 || g_mode == MODE_TASK4) && g_step == 1)
            g_gray_last_side = -1;   /* 任务三C->B默认向左找线 */
        else
            g_gray_last_side = 1;
        GIE = 1;
        enter_beep_and_next();
    }
}

/* ============================================================
 * 段执行器：run_arc
 * 弧线跟线，转角足够且连续全离线时切换
 * ============================================================ */
void run_arc(void)
{
    const Segment *seg = &g_route[g_step];
    unsigned int arc_turn = yaw_abs_diff(g_yaw, g_arc_yaw_ref);
    unsigned char gray = gray_read();
    char all_white = (gray == 0) ? 1 : 0;
    signed char side = gray_line_side_from_data(gray);

    if (g_mode == MODE_TASK2 && arc_turn < ARC_ENTRY_YAW_RAW)
    {
        motor_forward(ARC_FAST, ARC_SLOW);  /* B->C、D->A入弧先主动右转 */
        return;
    }

    if ((g_mode == MODE_TASK3 || g_mode == MODE_TASK4)
        && seg->label == 'B'
        && arc_turn < ARC_ENTRY_CB_YAW_RAW)
    {
        motor_forward(ARC_SLOW, ARC_FAST);  /* C->B入弧先主动左转 */
        return;
    }

    if (!g_arc_turn_ready)
    {
        if (arc_turn < ARC_MIN_YAW_RAW)
        {
            arc_control_side(side);
            return;
        }
        g_arc_turn_ready = 1;
    }

    if (!all_white)
    {
        g_arc_lost_cnt = 0;
        arc_control_side(side);
        return;
    }

    if (g_arc_lost_cnt < ARC_LOST_CONFIRM)
    {
        g_arc_lost_cnt++;
        arc_control_side(side);
        return;
    }

    /* 转角达到过阈值，并且8路灰度连续全白后，认为已经出弧到点 */
    GIE = 0;
    if (g_mode == MODE_TASK2 && seg->label == 'C')
    {
        g_yaw_ref = yaw_opposite(g_ab_yaw_ref);  /* C->D：AB实际方向+180度 */
        g_align_cd = 1;
    }
    else if ((g_mode == MODE_TASK3 || g_mode == MODE_TASK4) && seg->label == 'B')
    {
        g_yaw_ref = g_bd_yaw_ref;                /* B->D：任务三理论斜线方向 */
        g_align_cd = 1;
    }
    else
    {
        g_yaw_ref = g_yaw;
        g_align_cd = 0;
    }
    GIE = 1;
    enter_beep_and_next();
}

/* ============================================================
 * 段执行器：run_stop
 * ============================================================ */
void run_stop(void)
{
    /* 任务四：判断圈数 */
    if (g_mode == MODE_TASK4)
    {
        g_lap++;
        if (g_lap < 4)
        {
            /* 未满4圈，重置路径从第一段重新跑 */
            g_step = 0;
            GIE = 0;
            g_spin_yaw_ref = g_yaw_init;
            g_yaw_ref      = g_yaw_init;
            g_arc_yaw_ref  = g_yaw_init;
            g_bd_yaw_ref   = yaw_add_raw(g_yaw_init, -26000);
            g_arc_lost_cnt = 0;
            g_arc_turn_ready = 0;
            g_gray_last_side = 1;
            g_align_cd = 0;
            g_straight_wait_clear = 0;
GIE = 1;
            return;
        }
    }
    /* 完成 */
    motor_stop();
    BUZZER = 1;
    PORTD  = 0xFC;      /* 完成：RD0/RD1亮，其余灭 */
    g_mode = MODE_DONE;
}

/* ============================================================
 * BEEP状态处理
 * ============================================================ */
void beep_update(void)
{
    if (!g_in_beep) return;
    if ((unsigned int)(g_timer_ms - g_beep_start) >= BEEP_TIME_MS)
    {
        beep_led_off();
        g_in_beep = 0;
        /* BEEP结束，g_step已在enter_beep_and_next里推进，继续执行新段的初始化 */
        /* 新段若是SEG_STRAIGHT或SEG_ARC，不需要额外初始化 */
        /* 新段若是SEG_SPIN，需要记录旋转基准——在run_spin首次调用时已有g_spin_yaw_ref */
    }
}

/* ============================================================
 * 路径执行器主入口
 * ============================================================ */
void runner_run(void)
{
    /* 正在BEEP中，等待结束 */
    if (g_in_beep)
    {
        beep_update();
        return;
    }

    /* 防止step越界 */
    if (g_step >= g_route_len) return;

    switch (g_route[g_step].type)
    {
    case SEG_SPIN:     run_spin();     break;
    case SEG_STRAIGHT: run_straight(); break;
    case SEG_ARC:      run_arc();      break;
    case SEG_STOP:     run_stop();     break;
    }
}


/* ============================================================
 * 启动任务公共处理
 * ============================================================ */
void start_task(char task_num)
{
    unsigned char stable_cnt;

    /* 只在待机状态下响应 */
    if (g_mode != MODE_IDLE) return;

    g_step    = 0;
    g_lap     = 0;
    g_in_beep = 0;
    g_arc_lost_cnt = 0;
    g_arc_turn_ready = 0;
    g_gray_last_side = 1;
    g_align_cd = 0;
    g_straight_wait_clear = 0;
    g_straight_start_ms = g_timer_ms;

    /* 等待JY62连续5帧稳定 */
    g_new_frame = 0;
    stable_cnt  = 0;
    while (stable_cnt < 5) {
        if (g_new_frame) { g_new_frame = 0; stable_cnt++; }
    }

    /* 记录绝对初始基准 */
    GIE = 0;
    g_yaw_init     = g_yaw;
    g_yaw_ref      = g_yaw;
    g_spin_yaw_ref = g_yaw;
    g_arc_yaw_ref  = g_yaw;
    g_ab_yaw_ref   = g_yaw;
    g_bd_yaw_ref   = yaw_add_raw(g_yaw_init, -26000);
    GIE = 1;

    /* 清LED，显示任务编号 */
    PORTD = 0xFF;               /* 先全灭 */
    led_show_task(task_num);

    switch (task_num)
    {
    case 1:
        g_route     = route1;
        g_route_len = ROUTE1_LEN;
        g_mode      = MODE_TASK1;
        break;
    case 2:
        g_route     = route2;
        g_route_len = ROUTE2_LEN;
        g_mode      = MODE_TASK2;
        break;
    case 3:
        g_route     = route3;
        g_route_len = ROUTE3_LEN;
        g_mode      = MODE_TASK3;
        break;
    case 4:
        g_route     = route3;
        g_route_len = ROUTE3_LEN;
        g_mode      = MODE_TASK4;
        break;
    }
}

/* ============================================================
 * 按键扫描
 * RB0：复位（任何状态）
 * RB1：启动任务一
 * RB2：启动任务二
 * RB3：短按=任务三，长按>1s=任务四
 * 低电平有效，时间戳去抖20ms
 * ============================================================ */
#define DEBOUNCE_MS  20

static char         btn_reset_down  = 0;
static char         btn_t1_down     = 0;
static char         btn_t2_down     = 0;
static char         btn_t3_down     = 0;
static unsigned int btn_reset_ts    = 0;
static unsigned int btn_t1_ts       = 0;
static unsigned int btn_t2_ts       = 0;
static unsigned int btn_t3_ts       = 0;
static unsigned int btn_t3_hold_ts  = 0;
static char         btn_t3_fired    = 0;    /* 长按已触发，避免重复 */
static char         btn_t3_confirmed= 0;    /* 短按已确认，避免重复 */

void key_scan(void)
{
    /* ---- RB0 复位（任何状态有效） ---- */
    if (!BTN_RESET) {
        if (!btn_reset_down) {
            btn_reset_down = 1;
            btn_reset_ts   = g_timer_ms;
        }
        else if ((unsigned int)(g_timer_ms - btn_reset_ts) >= DEBOUNCE_MS) {
            /* 确认按下：停车，回待机，LED全灭等待闪烁 */
            motor_stop();
            BUZZER        = 0;
            PORTD         = 0xFF;
            g_in_beep     = 0;
            g_arc_lost_cnt = 0;
            g_arc_turn_ready = 0;
            g_gray_last_side = 1;
            g_align_cd = 0;
            g_straight_wait_clear = 0;
            g_straight_start_ms = g_timer_ms;
            g_mode        = MODE_IDLE;
            g_idle_blink_ts    = g_timer_ms;
            g_idle_blink_state = 0;
            btn_reset_ts  = g_timer_ms + 30000; /* 防止持续触发 */
        }
    } else { btn_reset_down = 0; }

    /* 任务运行中或完成后只响应复位键 */
    if (g_mode != MODE_IDLE) return;

    /* ---- RB1 启动任务一 ---- */
    if (!BTN_TASK1) {
        if (!btn_t1_down) {
            btn_t1_down = 1;
            btn_t1_ts   = g_timer_ms;
        }
        else if ((unsigned int)(g_timer_ms - btn_t1_ts) >= DEBOUNCE_MS) {
            start_task(1);
            btn_t1_ts = g_timer_ms + 30000;
        }
    } else { btn_t1_down = 0; }

    /* ---- RB2 启动任务二 ---- */
    if (!BTN_TASK2) {
        if (!btn_t2_down) {
            btn_t2_down = 1;
            btn_t2_ts   = g_timer_ms;
        }
        else if ((unsigned int)(g_timer_ms - btn_t2_ts) >= DEBOUNCE_MS) {
            start_task(2);
            btn_t2_ts = g_timer_ms + 30000;
        }
    } else { btn_t2_down = 0; }

    /* ---- RB3 启动任务三（长按=任务四） ---- */
    if (!BTN_TASK3) {
        if (!btn_t3_down) {
            btn_t3_down      = 1;
            btn_t3_ts        = g_timer_ms;
            btn_t3_hold_ts   = g_timer_ms;
            btn_t3_fired     = 0;
            btn_t3_confirmed = 0;
        }
        /* 长按优先：超过1s升级为任务四 */
        if (!btn_t3_fired
            && (unsigned int)(g_timer_ms - btn_t3_hold_ts) >= LONG_PRESS_MS) {
            btn_t3_fired     = 1;
            btn_t3_confirmed = 1;
            start_task(4);
        }
    }
    else {
        /* 松开时：若未触发长按且已去抖确认，启动任务三 */
        if (btn_t3_down && !btn_t3_fired && !btn_t3_confirmed
            && (unsigned int)(g_timer_ms - btn_t3_ts) >= DEBOUNCE_MS) {
            start_task(3);
        }
        btn_t3_down      = 0;
        btn_t3_fired     = 0;
        btn_t3_confirmed = 0;
    }
}



/* ============================================================
 * 系统初始化
 * ============================================================ */
void init_all(void)
{
    CMCON  = 0x07;      /* 关比较器 */
    ADCON1 = 0x06;      /* RA全部数字IO */

    /* RA：RA0输入(灰度OUT)，RA1~RA3输出(灰度AD0~AD2)，其余输出 */
    TRISA  = 0x01;
    /* RB：RB0~RB3输入(按键)，RB4~RB7输出(电机方向) */
    TRISB  = 0x0F;
    TRISD  = 0x00;      /* RD全部输出(LED) */
    TRISE  = 0x00;      /* RE输出(蜂鸣器) */

    /* 使能RB口内部上拉（OPTION_REG的RBPU位清零） */
    RBPU = 0;

    PORTD  = 0xFF;      /* 高电平=全灭（低电平点亮） */
    BUZZER = 0;
    GRAY_AD0 = 0;
    GRAY_AD1 = 0;
    GRAY_AD2 = 0;

    pwm_init();
    motor_init();
    uart_init();
    timer1_init();

    /* 待机状态，RD0开始闪烁 */
    g_idle_blink_ts    = 0;
    g_idle_blink_state = 0;
}

/* ============================================================
 * 主程序
 * ============================================================ */
void main(void)
{
    init_all();

    while (1)
    {
        key_scan();

        if (g_mode == MODE_IDLE)
            idle_blink_update();
        else if (g_mode != MODE_DONE)
            runner_run();
        /* MODE_DONE：全亮等复位，不需要额外处理 */
    }
}
