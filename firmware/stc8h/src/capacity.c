/*---------------------------------------------------------------------*/
/* --- ����������ģ�� ---                                            */
/* �ļ���: capacity.c                                                  */
/* ��������: ����100usʱ����ĵ��������ʱ����ʵ��                     */
/*           ֧�ֳ�������ͷŵ������ֱ��ۼ�                            */
/* Ӳ��ƽ̨: STC8Hϵ�е�Ƭ��                                           */
/* ����: [�������/�Ŷ�]                                               */
/* ����: 2026-03-10                                                    */
/* �汾: V2.0                                                          */
/* �޸ļ�¼:                                                           */
/*   V2.0 - ���ӳ�������ۼƣ����ֳ�ŵ�״̬                           */
/*   V1.0 - ��ʼ�汾                                                   */
/*                                                                     */
/* �����ļ�:                                                           */
/*   - STC8G_H_ADC.h   : ADC����                                       */
/*   - STC8G_H_Timer.h : ��ʱ������                                    */
/*   - i2c_cmd.h        : �����������壨����� curChargeMAH, curDischargeMAH, curBatVolt��*/
/*   - adc_measure.h    : ADC��ȡ����������read_battery_voltage�ȣ�    */
/*                                                                     */
/* ʹ��˵��:                                                           */
/*   1. �� main �����е��� TimeStamp_Init() ������ʱ��                */
/*   2. ���� Capacity_Init() ��ʼ����������                           */
/*   3. ����ѭ���ж��ڵ��� Capacity_Update() ������������             */
/*   4. ȷ�� adc_measure.h �ṩ�����������ȡ������                   */
/*        read_battery_voltage()  ���ص�ص�ѹ (V)                    */
/*        read_charge_current()   ���س����� (A)                    */
/*        read_discharge_current() ���طŵ���� (A)                   */
/*   5. ���� i2c_cmd.h �������ⲿ���� curChargeMAH ����               */
/*                                                                     */
/* ע��: ��ʱ��2�ж�����ֵ����24MHzϵͳʱ�Ӽ��㣬��ʱ�Ӳ�ͬ�����      */
/*---------------------------------------------------------------------*/

#include "STC8G_H_ADC.h"
#include "STC8G_H_Timer.h"
#include "capacity.h"
#include "i2c_cmd.h"          // ���� curDischargeMAH, curBatVolt, �Լ������� curChargeMAH
#include "adc_measure.h"      // ���� ADC ��ȡ����ԭ��

// ---------- ʱ���ģ�� ----------
static xdata unsigned long sys_tick_us;  // 100us ����
unsigned int adc_bat_code;

// ȫ�ֱ�����64λ��
unsigned long charge_capacity_num = 0;
unsigned long discharge_capacity_num = 0;

xdata unsigned long charge_capacity_int;   // ��������������� (mAh)
xdata unsigned long charge_capacity_frac;  // ��������������� (mA*ms)��< 3600000

xdata unsigned long discharge_capacity_int;   // �ŵ������������� (mAh)
xdata unsigned long discharge_capacity_frac;  // �ŵ������������� (mA*ms)��< 3600000


// ��ʼ����ʱ��2������100us�ж�
void TimeStamp_Init(void)
{
    // ����ϵͳʱ�� 24MHz����ʱ��ʱ�� = Fosc/12 = 2MHz
    // 100us ��Ӧ 200 ��������16λ�Զ����أ�Ԥ��Ƶ=1 (TM2PS=0)
    AUXR |= 0x04;			//��ʱ��ʱ��1Tģʽ
		T2L = 0xA0;				//���ö�ʱ��ʼֵ
		T2H = 0xF6;				//���ö�ʱ��ʼֵ
		AUXR |= 0x10;			//��ʱ��2��ʼ��ʱ
    IE2 |= 0x04;               // ʹ�ܶ�ʱ��2�ж� ET2 = 1
    EA = 1;                    // �������ж�
}

// ��ȡ��ǰʱ�����100us tick������֤ԭ�Ӷ�ȡ
unsigned long TimeStamp_GetTick(void)
{
    unsigned long tick;
    EA = 0;
    tick = sys_tick_us;
		EA = 1;
    return tick;
}

// ��ʱ��2�жϷ�������жϺ�12��
void Timer2_ISR(void) interrupt 12
{
    sys_tick_us++;
    // Ӳ���Զ�����жϱ�־���Զ�����ģʽ��
}

// ---------- �������ģ�� ----------
static xdata unsigned long last_tick;               // �ϴβ���ʱ���
static xdata unsigned long charge_capacity_x100;    // �ۼƳ����������λ 0.01mAh
static xdata unsigned long discharge_capacity_x100; // �ۼƷŵ���������λ 0.01mAh

// ��ʼ������������
void Capacity_Init(void)
{
    last_tick = TimeStamp_GetTick();
    charge_capacity_x100 = 0;
    discharge_capacity_x100 = 0;
    
    // �����ⲿ���������Ѷ��壩
    curChargeMAH = 0;
    curDischargeMAH = 0;
    curBatVolt = 0;
		charge_capacity_int = 0;
    charge_capacity_frac = 0;
    discharge_capacity_int = 0;
    discharge_capacity_frac = 0;
}

// 主循环中定期调用，检查并执行容量积分
void Capacity_Update(void)
{
    unsigned long current_tick = TimeStamp_GetTick();

    if ((current_tick - last_tick) >= 1000)
    {
        unsigned long delta_ms = (current_tick - last_tick) / 10;

        /* 整数版: 直接读 mV/mA, 零浮点 */
        unsigned int volt_mv   = read_battery_voltage_mv();
        unsigned int charge_ma = read_charge_current_ma();
        unsigned int discharge_ma = read_discharge_current_ma();

        last_tick = current_tick;
        curChargeCurrentMA = charge_ma;
        curDischargeCurrentMA = discharge_ma;

        if (charge_ma > 150)
        {
            charge_capacity_frac += (unsigned long)charge_ma * delta_ms;
            while (charge_capacity_frac >= 3600000UL)
            {
                charge_capacity_frac -= 3600000UL;
                charge_capacity_int++;
            }
            adc_bat_code++;
        }
        else if (discharge_ma > 150)
        {
            discharge_capacity_frac += (unsigned long)discharge_ma * delta_ms;
            while (discharge_capacity_frac >= 3600000UL)
            {
                discharge_capacity_frac -= 3600000UL;
                discharge_capacity_int++;
            }
            adc_bat_code++;
        }

        curChargeMAH = (unsigned int)charge_capacity_int;
        curDischargeMAH = (unsigned int)discharge_capacity_int;
        curBatVolt = volt_mv;
    }
}