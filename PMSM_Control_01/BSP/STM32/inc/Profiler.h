/**
 * @file    Profiler.h
 * @brief   ISR-aware runtime profiler for FreeRTOS + STM32G4
 *
 * 使用 DWT CYCCNT (32-bit, CPU 频率) 作为时间基准, 通过在 ISR 入口/出口
 * 累加 ISR 执行时间, 任务测量时自动扣除 ISR 占用, 得到净运行时间。
 *
 * 用法:
 *   1. main() 中调用 Profiler_Init()
 *   2. ISR 入口: uint32_t entry = Profiler_ISR_Enter();
 *      ISR 出口: Profiler_ISR_Exit(entry);
 *   3. 任务中:
 *        uint32_t isr_before, start;
 *        Profiler_TaskBegin(&isr_before, &start);
 *        // ... 业务代码 ...
 *        uint32_t net_cycles = Profiler_TaskEnd(isr_before, start);
 *        Profiler_Record(CPU_TASK1_INDEX, net_cycles);
 */

#ifndef __PROFILER_H__
#define __PROFILER_H__

#include "stm32g4xx.h"

/* SystemCoreClock 声明 — 确保 static inline 函数中可用 */
extern uint32_t SystemCoreClock;

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/*  Profiler 槽位数据结构                                                    */
/* ======================================================================== */

#define PROFILER_SLOT_COUNT  10

typedef struct {
    uint32_t count;       /* 测量次数                           */
    uint32_t net_min;     /* 最小净运行时间 (CPU cycles)         */
    uint32_t net_max;     /* 最大净运行时间 (CPU cycles)         */
    uint64_t net_cur;     /* 累计净运行时间 (CPU cycles)         */
} Profiler_Slot_t;

/* ======================================================================== */
/*  全局变量 (在 Bsp_STM32G431.c 中定义)                                     */
/* ======================================================================== */

/** ISR 累计执行时间 (CPU cycles), 仅最外层 ISR 退出时累加 */
extern volatile uint32_t g_isr_accumulated_cycles;

/** ISR 嵌套深度, 用于支持嵌套中断 */
extern volatile uint32_t g_isr_nest_level;

/** Profiler 槽位数组 */
extern Profiler_Slot_t g_profiler_slots[PROFILER_SLOT_COUNT];

/* ======================================================================== */
/*  Profiler API (全部 static inline)                                         */
/* ======================================================================== */

/**
 * @brief  初始化 DWT 周期计数器
 * @note   必须在 FreeRTOS 调度器启动前调用一次
 */
static inline void Profiler_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 使能 DWT 跟踪单元 */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;            /* 使能周期计数器    */
}

/**
 * @brief  ISR 入口 — 原子记录进入时刻 + 增加嵌套计数
 * @return 进入时的 DWT CYCCNT, 必须传入 Profiler_ISR_Exit()
 * @note   内部关中断 ~5 cycles 保证 nest_level++ 和 DWT 读的原子性,
 *         防止被更高优先级 ISR 打断导致嵌套计数错乱。
 */
static inline uint32_t Profiler_ISR_Enter(void)
{
    uint32_t sr = __get_PRIMASK();
    __disable_irq();
    g_isr_nest_level++;
    uint32_t now = DWT->CYCCNT;
    __set_PRIMASK(sr);
    return now;
}

/**
 * @brief  ISR 出口 — 原子累加 ISR 执行时间 + 减少嵌套计数
 * @param  entry_cycle   Profiler_ISR_Enter() 的返回值
 * @return 本段 ISR 从进入到退出的 CPU 周期数 (含被嵌套 ISR 的时间)
 * @note   只有最外层 ISR 退出时才更新 g_isr_accumulated_cycles,
 *         嵌套内层 ISR 退出时只减少计数, 不重复累加。
 *         内部关中断保证 nest_level-- 和累加的原子性。
 */
static inline uint32_t Profiler_ISR_Exit(uint32_t entry_cycle)
{
    uint32_t sr = __get_PRIMASK();
    __disable_irq();
    uint32_t elapsed = DWT->CYCCNT - entry_cycle;
    g_isr_nest_level--;
    if (g_isr_nest_level == 0) {
        g_isr_accumulated_cycles += elapsed;
    }
    __set_PRIMASK(sr);
    return elapsed;
}

/**
 * @brief  任务 profiling 开始 — 原子快照 ISR 累加值和起始时刻
 * @param[out] p_isr_before  快照的 ISR 累计值
 * @param[out] p_start       快照的 DWT 起始时刻
 * @note   短暂关中断 (~6 cycles @ 160MHz = 37ns) 保证原子性
 */
static inline void Profiler_TaskBegin(uint32_t *p_isr_before, uint32_t *p_start)
{
    uint32_t sr = __get_PRIMASK();
    __disable_irq();
    *p_isr_before = g_isr_accumulated_cycles;
    *p_start      = DWT->CYCCNT;
    __set_PRIMASK(sr);
}

/**
 * @brief  任务 profiling 结束 — 计算净运行时间
 * @param  isr_before  Profiler_TaskBegin 快照的 ISR 累计值
 * @param  start       Profiler_TaskBegin 快照的起始时刻
 * @return 净 CPU 周期数 (已扣除期间发生的 ISR 执行时间)
 * @note   返回 0 当 ISR 时间超过墙上时间 (理论上不应发生)
 */
static inline uint32_t Profiler_TaskEnd(uint32_t isr_before, uint32_t start)
{
    uint32_t sr = __get_PRIMASK();
    __disable_irq();
    uint32_t end        = DWT->CYCCNT;
    uint32_t isr_during = g_isr_accumulated_cycles - isr_before;
    __set_PRIMASK(sr);

    uint32_t wall = end - start;
    return (wall > isr_during) ? (wall - isr_during) : 0;
}

/**
 * @brief  记录一次测量到 profiler 槽位
 * @param  slot       槽位索引 (0..PROFILER_SLOT_COUNT-1)
 * @param  net_cycles 净运行时间 (CPU cycles)
 */
static inline void Profiler_Record(uint8_t slot, uint32_t net_cycles)
{
    if (slot >= PROFILER_SLOT_COUNT) return;

    Profiler_Slot_t *s = &g_profiler_slots[slot];
    s->count++;
    s->net_cur = net_cycles;
    if (net_cycles > s->net_max) s->net_max = net_cycles;
    if (s->count == 1 || net_cycles < s->net_min) s->net_min = net_cycles;
}

/**
 * @brief  将 CPU 周期数转换为微秒
 * @param  cycles  CPU 周期数
 * @return 微秒值
 * @note   基于 SystemCoreClock (160 MHz), 除法为整数运算
 */
static inline uint32_t Profiler_CyclesToUs(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000UL);
}

#ifdef __cplusplus
}
#endif

#endif /* __PROFILER_H__ */
