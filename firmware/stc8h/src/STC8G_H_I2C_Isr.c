/*---------------------------------------------------------------------*/
/* --- �Զ���I2C�ӻ�����Э���жϴ���ģ�� ---                          */
/* �ļ���: STC8G_H_I2C_Isr.c                                          */
/* ��������: ����STC8HӲ��I2C�ӻ�ģʽ���жϴ������ʵ���Զ������ￄ1�7    */
/*           ���������ݽ�����֧�ֳ�ŵ���ơ�������ѹ���á�ѭ������    */
/*           ���á���ѹ/����/������ȡ�����                          */
/* Ӳ��ƽ̨: STC8Hϵ�е�Ƭ�� (�� STC8H8K64U)                           */
/* ����: [��������/�Ŷ�]                                               */
/* ����: 2026-03-09                                                    */
/* �汾: V1.1                                                          */
/* �޸ļ�¼:                                                           */
/*   V1.0 - ��ʼ�汾 (2026-03-07)                                      */
/*   V1.1 - ���ӳ���ѹ����������������������������ע�ￄ1�7         */
/*   V1.2 - ����STC8H�ٷ�I2C�жϱ�־��ʹ��״̬���ع�                   */
/*                                                                     */
/* �����ļ�:                                                           */
/*   - STC8G_H_I2C.h      : I2CӲ������                                */
/*   - i2c_cmd.h          : �����붨����ȫ�ֱ�������                   */
/*   - STC8G_H_GPIO.h     : ���Ų������壨���ڿ��Ƴ�ŵ����ţￄ1�7         */
/*                                                                     */
/* ʹ��˵��:                                                           */
/*   1. �� main �����г�ʼ�� I2C �ӻ�ģʽ���������ж�                  */
/*   2. ���ļ��ṩ I2C_ISR_Handler �жϺ���������ȷӳ�䵽�ж�����     */
/*      (�жϺ� I2C_VECTOR��ͨ��Ϊ24)                                  */
/*   3. ��������������������ͨ���ⲿ����ʵ�֣���ￄ1�7 i2c_cmd.h           */
/*   4. д�����д����ݵģ������õ�ѹ�����յ�STOP�źź�ʵ�ʸ��±�����    */
/*      ��������Ҫ����Ӳ�����ţ������ó���ѹ���ţ�                    */
/*   5. �������ڵ�ַƥ��ʱ��׼�����ݣ��������ݷ��ͽ׶����ֽڷ���        */
/*                                                                     */
/* ע������:                                                           */
/*   - ȷ�� i2c_cmd.h �ж�����������뱾�ļ�һ�ￄ1�7                       */
/*   - ��ŵ�������ţ�P10��P11������ main.c ��ʼ��Ϊ������ￄ1�7          */
/*   - ����ѹ/�����������ţ�P3.2/P3.3/P2.1�������ʵ��Ӳ���������ￄ1�7   */
/*   - �жϴ����豣�ּ�̣����ⳤʱ��ռ�ￄ1�7                               */
/*   - ��ʹ����������ӳ�䣬���޸���Ӧ�� sbit ����                       */
/*---------------------------------------------------------------------*/

#include "STC8G_H_I2C.h"
#include "i2c_cmd.h"
#include "STC8G_H_GPIO.h"

/* ״̬������ */
#define I2C_STATE_IDLE      0   // ���У��ȴ�START
#define I2C_STATE_ADDR      1   // ���յ�START���ȴ���ַ�ֽ�
#define I2C_STATE_RX_DATA   2   // дģʽ����������
#define I2C_STATE_TX_DATA   3   // ��ģʽ����������

static u8 i2c_state = I2C_STATE_IDLE;   // ��ǰ״̬
bit i2c_cmd_valid = 0;
volatile unsigned char i2c_last_cmd = 0;
bit pwm_update_req = 0;

//========================================================================
// ����: I2C_ISR_Handler (�ع��棬����STC8H�ٷ��жϱ�־)
// ����: I2C�ӻ��жϴ��������ʵ��������������ݽ���
// ����: none.
// ����: none.
// �汾: V1.2, 2026-03-14 (����ٷ��궨�ￄ1�7)
//========================================================================
void I2C_ISR_Handler() interrupt I2C_VECTOR
{
    // ========== 1. ����START�¼� ==========
    if (I2CSLST & I2C_ESTAI)
    {
        I2CSLST &= ~I2C_ESTAI;          // ���START��־
        i2c_state = I2C_STATE_ADDR;     // �����ַ�ȴ�״̄1�7
        i2c_rx_index = 0;                // ���ý���������׼������������
        i2c_tx_index = 0;                // ���÷�������
        i2c_rx_len = 0;                  // �����������ݳ���
        // ע�⣺������ i2c_cmd_valid������֮ǰ���յ������룬�����ظ�START��Ķ����ￄ1�7
    }

    // ========== 2. �����жϣ�������ַ�����ݣ� ==========
    if (I2CSLST & I2C_ERXI)
    {
        u8 dat = I2CRXD;

        if (i2c_state == I2C_STATE_ADDR)
        {
            // ���յ��ӻ���ַ�ֽڣ�Ӳ�����Զ�ƥ���ַ�ￄ1�7
            if (dat & 0x01)  // �������������ӻ���
            {
                i2c_state = I2C_STATE_TX_DATA; // ���뷢��ģʽ

                // ����������׼��Ҫ���͵�����
                if (!i2c_cmd_valid)
                {
                    i2c_tx_len = 0;    // �������Ч���ￄ1�7
                }
                else
                {
                    switch (i2c_cmd)
                    {
                        case CMD_ALARM_OVERCHARGE:
                            i2c_tx_buf[0] = alarmOvercharge;
                            i2c_tx_len = 1;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_ALARM_OVERDISCHARGE:
                            i2c_tx_buf[0] = alarmOverdischarge;
                            i2c_tx_len = 1;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_CUR_CHARGE_MAH:
                            i2c_tx_buf[0] = curChargeMAH >> 8;
                            i2c_tx_buf[1] = curChargeMAH & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_CUR_DISCHARGE_MAH:
                            i2c_tx_buf[0] = curDischargeMAH >> 8;
                            i2c_tx_buf[1] = curDischargeMAH & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_CUR_BAT_VOLT:
                            i2c_tx_buf[0] = curBatVolt >> 8;
                            i2c_tx_buf[1] = curBatVolt & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_READ_OVERC_PROT_VOLT:
                            i2c_tx_buf[0] = readOverchargeProtectVolt >> 8;
                            i2c_tx_buf[1] = readOverchargeProtectVolt & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_READ_OVERD_PROT_VOLT:
                            i2c_tx_buf[0] = readOverdischargeProtectVolt >> 8;
                            i2c_tx_buf[1] = readOverdischargeProtectVolt & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_CUR_CYCLE_NUM:
                            i2c_tx_buf[0] = curCycleNum;
                            i2c_tx_len = 1;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_CUR_CHARGE_CURRENT:
                            i2c_tx_buf[0] = curChargeCurrentMA >> 8;
                            i2c_tx_buf[1] = curChargeCurrentMA & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        case CMD_CUR_DISCHARGE_CURRENT:
                            i2c_tx_buf[0] = curDischargeCurrentMA >> 8;
                            i2c_tx_buf[1] = curDischargeCurrentMA & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        // ѭ���ŵ����mAh (0x10~0x19)
                        case CMD_CYCLE_DISCH_END_MAH_1:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 1:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 2:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 3:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 4:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 5:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 6:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 7:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 8:
                        case CMD_CYCLE_DISCH_END_MAH_1 + 9:
                        {
                            u8 idx = i2c_cmd - CMD_CYCLE_DISCH_END_MAH_1;
                            u16 val = cycleDischargeEndMAH[idx];
                            i2c_tx_buf[0] = val >> 8;
                            i2c_tx_buf[1] = val & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                        }
                        break;
                        // ѭ��������mAh (0x1A~0x23)
                        case CMD_CYCLE_CHARGE_END_MAH_1:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 1:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 2:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 3:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 4:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 5:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 6:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 7:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 8:
                        case CMD_CYCLE_CHARGE_END_MAH_1 + 9:
                        {
                            u8 idx = i2c_cmd - CMD_CYCLE_CHARGE_END_MAH_1;
                            u16 val = cycleChargeEndMAH[idx];
                            i2c_tx_buf[0] = val >> 8;
                            i2c_tx_buf[1] = val & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                        }
                        break;
                        case CMD_READ_BAT_VOLT:
                            i2c_tx_buf[0] = curBatVolt >> 8;
                            i2c_tx_buf[1] = curBatVolt & 0xFF;
                            i2c_tx_len = 2;
                            // Append XOR checksum
                            {
                                unsigned char ck = i2c_tx_buf[0];
                                unsigned char k;
                                for (k = 1; k < i2c_tx_len; k++) ck ^= i2c_tx_buf[k];
                                i2c_tx_buf[i2c_tx_len] = ck;
                                i2c_tx_len++;
                            }
                            break;
                        default:
                            i2c_tx_len = 0; // δ֪�������������
                            break;
                    }
                }

                i2c_tx_index = 0;
                // ��������һ���ֽ�д�뷢�ͼĴ�����������������0xFF��
                if (i2c_tx_len > 0) {
                    I2CTXD = i2c_tx_buf[0];
                    i2c_tx_index = 1;
                } else {
                    I2CTXD = 0xFF;      // ����Ĭ��ֵ
                }
            }
            else  // д��������д�ӻ���
            {
                i2c_state = I2C_STATE_RX_DATA; // �������ģʄ1�7
                i2c_rx_index = 0;                // ׼������������
                i2c_rx_len = 0;                  // δ֪���ݳ��ȣ����ڽ����������ȷ�ￄ1�7
            }
        }
        else if (i2c_state == I2C_STATE_RX_DATA)
        {
            // ���յ������ֽڣ�дģʽ��
            if (i2c_rx_index == 0) {
                // ��һ���ֽ���������
                i2c_cmd = dat;
                i2c_cmd_valid = 1;     // �����������Є1�7

                // ����������ȷ����Ҫ���յ������ֽ���
                switch (i2c_cmd) {
                    case CMD_SET_OVERC_PROT_VOLT:
                    case CMD_SET_OVERD_PROT_VOLT:
                        i2c_rx_len = 2; // ��Ҫ2�ֽ�����
                        break;
                    case CMD_SET_CYCLE_DISCH_NUM:
                    case CMD_SET_CHARGE_VOLT:
                    case CMD_SET_CHARGE_CURRENT:
                    case CMD_SET_BAT_TYPE:
                        i2c_rx_len = 1; // ��Ҫ1�ֽ�����
                        break;
                    default:
                        i2c_rx_len = 0; // �������ݻ�����ￄ1�7
                        break;
                }
                i2c_rx_index = 1;       // �ѽ���������
            } else {
                // ���������ֽڣ�������Ҫ������δ�չ�ʱ�Ŵ洢��
                if (i2c_rx_len > 0 && (i2c_rx_index - 1) < i2c_rx_len) {
                    if ((i2c_rx_index - 1) < sizeof(i2c_rx_buf)) {
                        i2c_rx_buf[i2c_rx_index - 1] = dat;
                    }
                    i2c_rx_index++;      // ��Ч���ݼ�������
                }
                // �����Ҫ���ݻ����չ�������Ը��ֽڣ����洢��������������
            }
        }
        // ����״̬�½��յ����ݣ��緢��ģʽ�������ϲ��ᷢ��������
        I2CSLST &= ~I2C_ERXI;         // ������ձ�ք1�7
    }

    // ========== 3. �����жϣ����������ݣ���Ҫ������һ���ֽڣ� ==========
    if (I2CSLST & I2C_ETXI)
    {
        if (i2c_state == I2C_STATE_TX_DATA)
        {
            if (i2c_tx_index < i2c_tx_len) {
                I2CTXD = i2c_tx_buf[i2c_tx_index];
                i2c_tx_index++;
            } else {
                I2CTXD = 0xFF;        // ����Ĭ��ֵ
            }
        }
        else
        {
            // ��ȫ������Ƿ���ģʽ�´��������жϣ�����Ĭ��ք1�7
            I2CTXD = 0xFF;
        }
        I2CSLST &= ~I2C_ETXI;         // ������ͱ�ք1�7
    }

    // ========== 4. ��⵽STOP�źţ�ִ��д�����ʵ�ʲ��ￄ1�7 ==========
    if (I2CSLST & I2C_ESTOI)
    {
        // ����дģʽ�´���������ￄ1�7
        if (i2c_state == I2C_STATE_RX_DATA && i2c_cmd_valid)
        {
            if (i2c_rx_len > 0)
            {
                // ����Ƿ���յ����㹻�����ݣ������� + ���������ֽ�����
                u8 required_len = 1 + i2c_rx_len;  // ������1�ֽ� + ���ݳ���
                if (i2c_rx_index >= required_len) {
                    // ��������������������±��ￄ1�7
                    switch (i2c_cmd) {
                        case CMD_SET_OVERC_PROT_VOLT:
                            overchargeProtectVolt = (i2c_rx_buf[0] << 8) | i2c_rx_buf[1];
                            // �ɴ���Ӳ�������������ñȽ�����
                            break;
                        case CMD_SET_OVERD_PROT_VOLT:
                            overdischargeProtectVolt = (i2c_rx_buf[0] << 8) | i2c_rx_buf[1];
                            break;
                        case CMD_SET_CYCLE_DISCH_NUM:
                            cycleDischargeNum = i2c_rx_buf[0];
                            break;
                        case CMD_SET_CHARGE_VOLT:
                            chargeVoltSet = i2c_rx_buf[0];
                            CHG_VOLT_SET_1 = (chargeVoltSet & 0x01) ? 1 : 0;
                            CHG_VOLT_SET_2 = (chargeVoltSet & 0x02) ? 1 : 0;
                            break;
                        case CMD_SET_CHARGE_CURRENT:
                            chargeCurrentSet = i2c_rx_buf[0];
                            // �ɿ��Ƴ���������
                            break;
                        case CMD_SET_BAT_TYPE:
                            batteryType = i2c_rx_buf[0];
                            break;
                        default:
                            break;
                    }
                    i2c_last_cmd = i2c_cmd;  // ��¼�ɹ�ִ�е�����
                }
                // �����ݲ��㣬��ִ���κβ�����������
            }
            else // i2c_rx_len == 0����������д����������룩
            {
                switch (i2c_cmd) {
                    case CMD_START_DISCHARGE:
                        start_discharge_req = 1;
                        i2c_last_cmd = i2c_cmd;
                        break;
                    case CMD_STOP_DISCHARGE:
                        stop_discharge_req = 1;
                        i2c_last_cmd = i2c_cmd;
                        break;
                    case CMD_START_CHARGE:
                        start_charge_req = 1;
                        i2c_last_cmd = i2c_cmd;
                        break;
                    case CMD_STOP_CHARGE:
                        stop_charge_req = 1;
                        i2c_last_cmd = i2c_cmd;
                        break;
                    default:
                        break;
                }
            }
        }

        // ���״̬��׼����һ�δ��ￄ1�7
        i2c_state = I2C_STATE_IDLE;
        i2c_rx_index = 0;
        i2c_tx_index = 0;
        i2c_rx_len = 0;
        i2c_cmd_valid = 0;
        I2CSLST &= ~I2C_ESTOI;         // ���STOP��־
    }
}