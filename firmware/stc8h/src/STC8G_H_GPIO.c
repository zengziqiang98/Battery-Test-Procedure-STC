/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序            */
/*---------------------------------------------------------------------*/

#include	"STC8G_H_GPIO.h"

void GPIO_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;  // 结构定义
		
    // ==================== P0 端口 ====================
    // ADDR_3 ~ ADDR_6 (P0.0~P0.3) : 地址脚 → 高阻输入
    GPIO_InitStructure.Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.Mode = GPIO_HighZ;
    GPIO_Inilize(GPIO_P0, &GPIO_InitStructure);

    // ==================== P1 端口 ====================
    // BAT_DISCHG_CURRENT_SENSE (P1.0) : 放电电流检测 → 高阻输入
    GPIO_InitStructure.Pin  = GPIO_Pin_0;
    GPIO_InitStructure.Mode = GPIO_HighZ;
    GPIO_Inilize(GPIO_P1, &GPIO_InitStructure);

    // CHG_LED (P1.3) 和 DISCHG_LED (P1.6) : 指示灯 → 推挽输出
    GPIO_InitStructure.Pin  = GPIO_Pin_3 | GPIO_Pin_1;
    GPIO_InitStructure.Mode = GPIO_OUT_PP;
    GPIO_Inilize(GPIO_P1, &GPIO_InitStructure);

    // I2C_SDA (P1.4) 和 I2C_SCL (P1.5) : I2C 总线 → 开漏输出
    GPIO_InitStructure.Pin  = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.Mode = GPIO_OUT_OD;
    GPIO_Inilize(GPIO_P1, &GPIO_InitStructure);

    // ==================== P2 端口 ====================
    // 控制输出：CHARGE_CTRL (P2.0), CHG_CURRENT_SET (P2.1),
    // DISCHARGE_CTRL (P2.3), FAN_CTRL (P2.4) → 推挽输出
    GPIO_InitStructure.Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.Mode = GPIO_OUT_PP;
    GPIO_Inilize(GPIO_P2, &GPIO_InitStructure);

    // 地址脚：ADDR_0 (P2.5), ADDR_1 (P2.6), ADDR_2 (P2.7) → 高阻输入
    GPIO_InitStructure.Pin  = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.Mode = GPIO_HighZ;
    GPIO_Inilize(GPIO_P2, &GPIO_InitStructure);

    // ==================== P3 端口 ====================
    // 充电电压设置：CHG_VOLT_SET_1 (P3.2), CHG_VOLT_SET_2 (P3.3) → 推挽输出
    GPIO_InitStructure.Pin  = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.Mode = GPIO_OUT_PP;
    GPIO_Inilize(GPIO_P3, &GPIO_InitStructure);

    // 检测输入：BAT_CHG_CURRENT_SENSE (P3.4), BAT_VOLT_SENSE (P3.6) → 高阻输入
    GPIO_InitStructure.Pin  = GPIO_Pin_4 | GPIO_Pin_6;
    GPIO_InitStructure.Mode = GPIO_HighZ;
    GPIO_Inilize(GPIO_P3, &GPIO_InitStructure);
		
		GPIO_InitStructure.Pin  = GPIO_Pin_0 | GPIO_Pin_1;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7
		GPIO_InitStructure.Mode = GPIO_PullUp;	//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
		GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//初始化
}

//========================================================================
// 函数: u8	GPIO_Inilize(u8 GPIO, GPIO_InitTypeDef *GPIOx)
// 描述: 初始化IO口.
// 参数: GPIOx: 结构参数,请参考头文件里的定义.
// 返回: 成功返回 SUCCESS, 错误返回 FAIL.
// 版本: V1.0, 2012-10-22
//========================================================================
u8	GPIO_Inilize(u8 GPIO, GPIO_InitTypeDef *GPIOx)
{
	if(GPIO > GPIO_P7)				return FAIL;	//错误
	if(GPIOx->Mode > GPIO_OUT_PP)	return FAIL;	//错误
	if(GPIO == GPIO_P0)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P0M1 &= ~GPIOx->Pin,	P0M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P0M1 |=  GPIOx->Pin,	P0M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P0M1 |=  GPIOx->Pin,	P0M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P0M1 &= ~GPIOx->Pin,	P0M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P1)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P1M1 &= ~GPIOx->Pin,	P1M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P1M1 |=  GPIOx->Pin,	P1M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P1M1 |=  GPIOx->Pin,	P1M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P1M1 &= ~GPIOx->Pin,	P1M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P2)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P2M1 &= ~GPIOx->Pin,	P2M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P2M1 |=  GPIOx->Pin,	P2M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P2M1 |=  GPIOx->Pin,	P2M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P2M1 &= ~GPIOx->Pin,	P2M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P3)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P3M1 &= ~GPIOx->Pin,	P3M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P3M1 |=  GPIOx->Pin,	P3M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P3M1 |=  GPIOx->Pin,	P3M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P3M1 &= ~GPIOx->Pin,	P3M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P4)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P4M1 &= ~GPIOx->Pin,	P4M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P4M1 |=  GPIOx->Pin,	P4M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P4M1 |=  GPIOx->Pin,	P4M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P4M1 &= ~GPIOx->Pin,	P4M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P5)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P5M1 &= ~GPIOx->Pin,	P5M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P5M1 |=  GPIOx->Pin,	P5M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P5M1 |=  GPIOx->Pin,	P5M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P5M1 &= ~GPIOx->Pin,	P5M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P6)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P6M1 &= ~GPIOx->Pin,	P6M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P6M1 |=  GPIOx->Pin,	P6M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P6M1 |=  GPIOx->Pin,	P6M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P6M1 &= ~GPIOx->Pin,	P6M0 |=  GPIOx->Pin;	 //推挽输出
	}
	if(GPIO == GPIO_P7)
	{
		if(GPIOx->Mode == GPIO_PullUp)		P7M1 &= ~GPIOx->Pin,	P7M0 &= ~GPIOx->Pin;	 //上拉准双向口
		if(GPIOx->Mode == GPIO_HighZ)			P7M1 |=  GPIOx->Pin,	P7M0 &= ~GPIOx->Pin;	 //浮空输入
		if(GPIOx->Mode == GPIO_OUT_OD)		P7M1 |=  GPIOx->Pin,	P7M0 |=  GPIOx->Pin;	 //开漏输出
		if(GPIOx->Mode == GPIO_OUT_PP)		P7M1 &= ~GPIOx->Pin,	P7M0 |=  GPIOx->Pin;	 //推挽输出
	}
	return SUCCESS;	//成功
}
