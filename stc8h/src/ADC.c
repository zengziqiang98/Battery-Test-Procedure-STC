/*---------------------------------------------------------------------*/
/* --- ��ص��/����ADC����ģ�飨���ӻ���ƽ���˲���---                  */
/* �ļ���: adc_measure.c                                               */
/* ��������: ����STC8H��ADC������ʵ�ֵ�ص�ѹ�����������ŵ������    */
/*           �ɼ������������㡣֧�ֶ�ͨ����������ѹ�ȡ��������衢      */
/*           �˷������Ӳ���������á�                                  */
/* Ӳ��ƽ̨: STC8Hϵ�е�Ƭ�� (�� STC8H8K64U)                           */
/* ����: [��������/�Ŷ�]                                               */
/* ����: 2026-03-09                                                    */
/* �汾: V1.1                                                          */
/* �޸ļ�¼:                                                           */
/*   V1.0 - ��ʼ�汾                                                   */
/*   V1.1 - ���ӻ���ƽ���˲�����                                       */
/*                                                                     */
/* �����ļ�:                                                           */
/*   - stc8h.h              : STC8Hϵ�мĴ�������                     */
/*   - STC8G_H_ADC.h        : ADC�����⣨�ٷ����Զ��壩                */
/*                                                                     */
/* ʹ��˵��:                                                           */
/*   1. �� main �������ȵ��� ADC_config() ��ʼ��ADC                   */
/*   2. ��Ҫ��ȡ��ѹ�����ʱ��ֱ�ӵ��ö�Ӧ������                      */
/*        float v = read_battery_voltage();                            */
/*        float c = read_charge_current();                             */
/*        float d = read_discharge_current();                          */
/*   3. ��������ֵΪ����������λ����ѹΪ����(V)������Ϊ����(A)��      */
/*      ����ȡʧ�ܣ����� -1.0f��                                       */
/*   4. �˲����ö�ԭʼADC��ֵ���л���ƽ�������ڴ�С�� ADC_FILTER_WINDOW */
/*      �궨�壬��ͨ�� adc_filter_reset() ��λ�����˲�����            */
/*---------------------------------------------------------------------*/

#include "stc8h.h"
#include <stdio.h>          /* ����Ҫprintf���ɲ����� */
#include "STC8G_H_ADC.h"
#include "adc_measure.h"

/* �ⲿ������������ADC�������ṩ�� */
extern unsigned int Get_ADCResult(unsigned char channel);  /* ��ѯ��ʽ��ȡADC������0~4095��4096��ʾ���� */
extern void ADC_PowerControl(unsigned char enable);        /* ADC��Դ���� */
extern void NVIC_ADC_Init(unsigned char enable, unsigned char priority); /* ADC�ж����� */

/*==============================================================================
 * ����ƽ���˲����ṹ�弰������C89��֧���ں����ڶ��嶯̬���飬�ʲ��þ�̬ȫ�֣�
 *============================================================================*/
typedef struct {
    unsigned int buffer[ADC_FILTER_WINDOW];   /* ���λ��������洢ԭʼADC��ֵ */
    unsigned char index;                      /* ��ǰд��λ�� */
    unsigned char count;                      /* �Ѵ������Ч���ݸ�������ʼ�׶��ã� */
    unsigned long sum;                        /* ������������ֵ���ۼӺ� */
} ADC_Filter;

/* ����ͨ�����˲���ʵ�� */
static ADC_Filter filter_volt;    /* ��ص�ѹ��� */
static ADC_Filter filter_chg;     /* ������ͨ�� */
static ADC_Filter filter_dischg;  /* �ŵ������� */

/*==============================================================================
 * �ڲ�����������ָ���˲����������˲����ADC��ֵ
 * ������filter - �˲����ṹ��ָ��
 *       new_adc - �²ɼ���ԭʼADC��ֵ��0~4095��
 * ���أ��˲����ADC��ֵ�������������ں���������ת����
 *============================================================================*/
static float UpdateFilter(ADC_Filter *filter, unsigned int new_adc)
{
    unsigned char i;
    unsigned long temp_sum;
    float average;

    /* �����������δ���������������ݣ���ִ�л��� */
    if (filter->count < ADC_FILTER_WINDOW) {
        /* ֱ�Ӵ洢����ǰindexλ�� */
        filter->buffer[filter->index] = new_adc;
        filter->sum += new_adc;
        filter->count++;
        filter->index++;
        if (filter->index >= ADC_FILTER_WINDOW) {
            filter->index = 0;   /* ������countδ��ʱ�����ƻأ���������ȫ�� */
        }
        /* ���ص�ǰ�������ݵ�ƽ��ֵ */
        temp_sum = filter->sum;
        average = (float)temp_sum / (float)filter->count;
        return average;
    }
    else {
        /* �������������Ƴ����ֵ��������� */
        unsigned int oldest = filter->buffer[filter->index];
        filter->sum = filter->sum - oldest + new_adc;
        filter->buffer[filter->index] = new_adc;
        filter->index++;
        if (filter->index >= ADC_FILTER_WINDOW) {
            filter->index = 0;
        }
        /* ���ػ���ƽ��ֵ */
        temp_sum = filter->sum;
        average = (float)temp_sum / (float)ADC_FILTER_WINDOW;
        return average;
    }
}

/*==============================================================================
 * ������������λ�����˲��������㻺�������ۼӺͣ�
 *============================================================================*/
void adc_filter_reset(void)
{
    unsigned char i;

    /* ��λ��ѹ�˲��� */
    for (i = 0; i < ADC_FILTER_WINDOW; i++) {
        filter_volt.buffer[i] = 0;
    }
    filter_volt.index = 0;
    filter_volt.count = 0;
    filter_volt.sum = 0UL;

    /* ��λ�������˲��� */
    for (i = 0; i < ADC_FILTER_WINDOW; i++) {
        filter_chg.buffer[i] = 0;
    }
    filter_chg.index = 0;
    filter_chg.count = 0;
    filter_chg.sum = 0UL;

    /* ��λ�ŵ�����˲��� */
    for (i = 0; i < ADC_FILTER_WINDOW; i++) {
        filter_dischg.buffer[i] = 0;
    }
    filter_dischg.index = 0;
    filter_dischg.count = 0;
    filter_dischg.sum = 0UL;
}

/*==============================================================================
 * ADC��ʼ������
 *============================================================================*/
void ADC_config(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_InitStructure.ADC_SMPduty   = 31;       /* ����ʱ����ƣ�����С��10 */
    ADC_InitStructure.ADC_CsSetup   = 0;        /* ͨ��ѡ��ʱ�� */
    ADC_InitStructure.ADC_CsHold    = 1;        /* ͨ��ѡ�񱣳�ʱ�� */
    ADC_InitStructure.ADC_Speed     = ADC_SPEED_2X8T;   /* ADC����ʱ��Ƶ�� */
    ADC_InitStructure.ADC_AdjResult = ADC_RIGHT_JUSTIFIED; /* ����Ҷ��� */
    ADC_Inilize(&ADC_InitStructure);            /* ��ʼ��ADC */
    ADC_PowerControl(ENABLE);                   /* ����ADC��Դ */
    NVIC_ADC_Init(DISABLE, Priority_0);         /* ��ʹ���жϣ����ȼ��ɺ��� */

    /* ��λ�����˲�������֤��ʼ״̬һ�� */
    adc_filter_reset();
}

/*==============================================================================
 * ��ȡ��ص�ѹ���˲���
 *============================================================================*/
float read_battery_voltage(void)
{
    unsigned int adc_raw;
    float adc_filtered;
    float volt_adc;
    float battery_volt;

    /* ��ȡԭʼADCֵ */
    adc_raw = Get_ADCResult(BAT_VOLT_CH);
    if (adc_raw == 4096) {
        return -1.0f;               /* ��ȡ���� */
    }

    /* �����˲�������ȡ�˲����ADC��ֵ���������� */
    adc_filtered = UpdateFilter(&filter_volt, adc_raw);

    /* ת��Ϊ��ѹֵ */
    volt_adc = adc_filtered * ADC_VREF / 4096.0f;
    battery_volt = volt_adc * BAT_VOLT_DIVIDER;
    return battery_volt;
}

/*==============================================================================
 * ��ȡ���������˲���
 *============================================================================*/
float read_charge_current(void)
{
    unsigned int adc_raw;
    float adc_filtered;
    float volt_adc;
    float volt_sense;
    float current;

    adc_raw = Get_ADCResult(BAT_CHG_CH);
    if (adc_raw == 4096) {
        return -1.0f;
    }

    adc_filtered = UpdateFilter(&filter_chg, adc_raw);
    volt_adc = adc_filtered * ADC_VREF / 4096.0f;
    volt_sense = volt_adc - CHG_OFFSET;            /* �۳������� */
    current = volt_sense / (CHG_SENSE_RES * CHG_AMP_GAIN) * CHG_CALIB_COEF;
    return current;
}

/*==============================================================================
 * ��ȡ�ŵ�������˲���
 *============================================================================*/
float read_discharge_current(void)
{
    unsigned int adc_raw;
    float adc_filtered;
    float volt_adc;
    float volt_sense;
    float current;

    adc_raw = Get_ADCResult(BAT_DISCHG_CH);
    if (adc_raw == 4096) {
        return -1.0f;
    }

    adc_filtered = UpdateFilter(&filter_dischg, adc_raw);
    volt_adc = adc_filtered * ADC_VREF / 4096.0f;
    volt_sense = volt_adc - DISCHG_OFFSET;
    current = volt_sense / (DISCHG_SENSE_RES * DISCHG_AMP_GAIN) * DISCHG_CALIB_COEF;
    return current;
}
/*==============================================================================
 * 整数版滑动滤� (替代 float UpdateFilter)
 *============================================================================*/
static unsigned int UpdateFilterInt(ADC_Filter *filter, unsigned int new_adc)
{
    unsigned int oldest;
    if (filter->count < ADC_FILTER_WINDOW) {
        filter->buffer[filter->index] = new_adc;
        filter->sum += new_adc;
        filter->count++;
        filter->index++;
        if (filter->index >= ADC_FILTER_WINDOW) filter->index = 0;
        return (unsigned int)(filter->sum / filter->count);
    } else {
        oldest = filter->buffer[filter->index];
        filter->sum = filter->sum - oldest + new_adc;
        filter->buffer[filter->index] = new_adc;
        filter->index++;
        if (filter->index >= ADC_FILTER_WINDOW) filter->index = 0;
        return (unsigned int)(filter->sum / ADC_FILTER_WINDOW);
    }
}

/*==============================================================================
 * 整数版�数: 直接返回 mV / mA, 零浮点运�
 *============================================================================*/
unsigned int read_battery_voltage_mv(void)
{
    unsigned int adc_raw, adc_f;
    adc_raw = Get_ADCResult(BAT_VOLT_CH);
    if (adc_raw == 4096) return 0;
    adc_f = UpdateFilterInt(&filter_volt, adc_raw);
    return (unsigned int)(((unsigned long)adc_f * 30500 + 2048) / 4096);
}

unsigned int read_charge_current_ma(void)
{
    unsigned int adc_raw, adc_f;
    adc_raw = Get_ADCResult(BAT_CHG_CH);
    if (adc_raw == 4096) return 0;
    adc_f = UpdateFilterInt(&filter_chg, adc_raw);
    return (unsigned int)(((unsigned long)adc_f * 481 + 5200) / 10401);
}

unsigned int read_discharge_current_ma(void)
{
    unsigned int adc_raw, adc_f;
    adc_raw = Get_ADCResult(BAT_DISCHG_CH);
    if (adc_raw == 4096) return 0;
    adc_f = UpdateFilterInt(&filter_dischg, adc_raw);
    return (unsigned int)(((unsigned long)adc_f * 660 + 3789) / 7578);
}
