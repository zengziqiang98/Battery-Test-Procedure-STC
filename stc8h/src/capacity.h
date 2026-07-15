#ifndef __CAPACITY_H__
#define __CAPACITY_H__

// 时间戳初始化（启动定时器2，100us中断）
void TimeStamp_Init(void);

// 获取当前时间戳（100us tick）
unsigned long TimeStamp_GetTick(void);

// 容量检测初始化（清零累计值，记录起始时间）
void Capacity_Init(void);

// 容量更新函数（需在主循环中定期调用）
void Capacity_Update(void);

#endif