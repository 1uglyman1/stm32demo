#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Delay.h"
#include <stdint.h>                     // 标准整数类型头文件

// 全局变量定义（中断函数和测距函数共用）
uint8_t KeyNum;                         // 按键键码变量（预留）
volatile uint32_t time = 0;             // 定时器溢出计数（volatile防止编译器优化）
float length = 0.0f;                    // 测距结果（单位：cm）

// 硬件宏定义
#define  TIM                            TIM2
#define  TIM_IRQHandler                 TIM2_IRQHandler 
#define  HCSR04_PORT_Trig               GPIOA
#define  HCSR04_PORT_Echo               GPIOA
#define  HCSR04_CLK_Trig                RCC_APB2Periph_GPIOA
#define  HCSR04_CLK_Echo                RCC_APB2Periph_GPIOA
#define  HCSR04_TRIG                    GPIO_Pin_9
#define  HCSR04_ECHO                    GPIO_Pin_10

// 中断优先级配置
void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 分组2：2位抢占优先级，2位响应优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;       
    NVIC_Init(&NVIC_InitStructure);
}

// HC-SR04初始化（GPIO+定时器）
void HC_SR04_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_hcsr04init;

    // 使能时钟
    RCC_APB2PeriphClockCmd(HCSR04_CLK_Trig | HCSR04_CLK_Echo, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Trig引脚（PA9）：推挽输出
    GPIO_InitStructure.GPIO_Pin = HCSR04_TRIG;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HCSR04_PORT_Trig, &GPIO_InitStructure);
    GPIO_ResetBits(HCSR04_PORT_Trig, HCSR04_TRIG);  // 默认低电平

    // Echo引脚（PA10）：上拉输入（抗干扰）
    GPIO_InitStructure.GPIO_Pin = HCSR04_ECHO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    // 上拉输入
    GPIO_Init(HCSR04_PORT_Echo, &GPIO_InitStructure);

    // 定时器2初始化（72MHz / 72 = 1MHz计数频率 → 1us/计数）
    TIM_hcsr04init.TIM_ClockDivision = TIM_CKD_DIV1;    // 不分频
    TIM_hcsr04init.TIM_CounterMode   = TIM_CounterMode_Up;  // 向上计数
    TIM_hcsr04init.TIM_Period        = 65535 - 1;        // 最大计数65535（覆盖400cm所需的23200us）
    TIM_hcsr04init.TIM_Prescaler     = 72 - 1;          // 预分频：72（72MHz/72=1MHz）
    TIM_TimeBaseInit(TIM, &TIM_hcsr04init);
    TIM_ClearFlag(TIM, TIM_FLAG_Update);                // 清除更新标志
    TIM_ITConfig(TIM, TIM_IT_Update, ENABLE);          // 使能更新中断
    NVIC_Config();                                      // 配置中断优先级
    TIM_Cmd(TIM, DISABLE);                              // 初始失能定时器
}

// 发送10us触发脉冲（严格时序）
void HC_SR04_start(void)
{   
    GPIO_ResetBits(HCSR04_PORT_Trig, HCSR04_TRIG);
    Delay_us(2);   // 确保低电平稳定
    GPIO_SetBits(HCSR04_PORT_Trig, HCSR04_TRIG);
    Delay_us(10);  // 10us高电平触发（必须精准）
    GPIO_ResetBits(HCSR04_PORT_Trig, HCSR04_TRIG);
}

// 获取Echo脉冲持续时间（单位：us，精准计算）
uint32_t GetEchoTimer(void)
{
    uint32_t t = 0;
    // 超时次数×65535us + 当前计数值（1MHz计数→1us/数）
    t = time * 65535 + TIM_GetCounter(TIM); 
    TIM_SetCounter(TIM, 0);   // 计数器清零
    time = 0;                 // 溢出计数清零
    return t;
}

// TIM2中断服务函数（溢出计数）
void TIM_IRQHandler(void)  
{
    if (TIM_GetITStatus(TIM, TIM_IT_Update) != RESET)  
    {
        TIM_ClearITPendingBit(TIM, TIM_IT_Update);
        time++;  // 每溢出65535us，time加1
    }
}

// 测距函数（精准厘米计算，5次采样平均）
float HCSR04GetLength(void)
{
    uint32_t t = 0;
    uint8_t i = 0;
    float lengthTemp = 0.0f;
    float sum = 0.0f;
    uint32_t timeout = 0; // 超时计数器

    while(i < 5)
    {
        HC_SR04_start();  // 发送触发脉冲
        
        // 等待Echo高电平（超时保护：200us）
        timeout = 0;
        while(GPIO_ReadInputDataBit(HCSR04_PORT_Echo, HCSR04_ECHO) == 0)
        {
            timeout++;
            if(timeout > 20000) break; // 200us超时
        }
        if(timeout > 20000) continue;

        // 启动定时器计数
        TIM_SetCounter(TIM, 0);
        time = 0;
        TIM_Cmd(TIM, ENABLE);

        // 等待Echo低电平（超时保护：400cm对应23200us）
        timeout = 0;
        while(GPIO_ReadInputDataBit(HCSR04_PORT_Echo, HCSR04_ECHO) == 1)
        {
            timeout++;
            if(timeout > 30000) break; // 30000us≈400cm+余量
        }
        TIM_Cmd(TIM, DISABLE); // 停止定时器

        if(timeout > 30000) continue;

        // 核心公式：距离(cm) = 时间(us) / 58
        t = GetEchoTimer();
        lengthTemp = (float)t / 58.0f;  
        // 过滤异常值（HC-SR04有效范围：2~400cm）
        if(lengthTemp >= 2.0f && lengthTemp <= 400.0f)  
        {
            sum += lengthTemp;
            i++;
        }
        Delay_us(200);  // 采样间隔，避免干扰
    }

    // 防止除以0，返回0表示采样失败
    return (i == 0) ? 0.0f : (sum / i);
}

// 格式化显示距离（cm，保留1位小数，适配OLED）
void ShowDistanceOnOLED(float dist)
{
    uint16_t integer_part = (uint16_t)dist;          // 整数部分（如12.3→12）
    uint16_t decimal_part = (uint16_t)(dist * 10) % 10; // 1位小数（如12.3→3）
    
    // 显示整数部分（3行1列，占3位，适配0~400）
    OLED_ShowNum(3, 1, integer_part, 3);
    // 显示小数点（3行4列）
    OLED_ShowString(3, 4, ".");
    // 显示1位小数（3行5列）
    OLED_ShowNum(3, 5, decimal_part, 1);
    // 显示单位（3行7列）
    OLED_ShowString(3, 7, "cm");
}

int main(void)
{
    // 初始化外设
    OLED_Init();
    HC_SR04_init();

    // OLED初始化显示
    OLED_Clear(); // 清屏
    OLED_ShowString(2, 1, "Distance:");  // 第2行第1列提示文字

    while (1)
    {
        length = HCSR04GetLength();  // 获取精准距离（cm）
        ShowDistanceOnOLED(length);  // 显示到OLED
        Delay_ms(300);  // 刷新间隔，平衡刷新率和稳定性
    }
}
