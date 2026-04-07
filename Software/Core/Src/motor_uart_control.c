#include "motor_uart_control.h"
#include "bldc_foc.h"
#include "circular_buffer.h"
#include <math.h>

static UartControlBlock_t g_uart_ctrl = {0};

#define UART3_TX_FIFO_SIZE 256
static uint8_t uart3_tx_fifo_buffer[UART3_TX_FIFO_SIZE];
static CircularBuffer_t uart3_tx_fifo;
static uint8_t uart3_dma_buffer[16];
static uint8_t uart3_packet_buffer[UART_PACKET_SIZE];

static void MotorUart_RxByte(uint8_t byte);
static void MotorUart_ResetState(void);
static bool MotorUart_VerifyChecksum(const MotorUartPacket_t *packet);
static uint8_t MotorUart_CalcChecksum(const MotorUartPacket_t *packet);
static float MotorUart_DecodeIqRef(int16_t q15_value);
static float MotorUart_DecodeAngle(uint16_t q16_value);
static int16_t MotorUart_EncodeIqRef(float iq_float);
static uint16_t MotorUart_EncodeAngle(float theta_float);

void MotorUart_Init(void)
{
    g_uart_ctrl.byte_count = 0;
    g_uart_ctrl.state = UART_STATE_WAIT_HEADER;
    g_uart_ctrl.valid_packets = 0;
    g_uart_ctrl.crc_errors = 0;
    g_uart_ctrl.timeout_errors = 0;

    g_uart_ctrl.current_packet.header = UART_PACKET_HEADER;
    g_uart_ctrl.current_packet.cmd = MOTOR_CMD_DISABLE;
    g_uart_ctrl.current_packet.iq_ref = 0;
    g_uart_ctrl.current_packet.theta_e = 0;

    HAL_UART_Receive_IT(&huart1, &g_uart_ctrl.rx_byte, 1);

    CircularBuffer_Init(&uart3_tx_fifo, uart3_tx_fifo_buffer, UART3_TX_FIFO_SIZE);
}

void MotorUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {

        MotorUart_RxByte(g_uart_ctrl.rx_byte);

        HAL_UART_Receive_IT(&huart1, &g_uart_ctrl.rx_byte, 1);
    }
}

static void MotorUart_RxByte(uint8_t byte)
{
    switch (g_uart_ctrl.state) {

        case UART_STATE_WAIT_HEADER:

            if (byte == UART_PACKET_HEADER) {
                g_uart_ctrl.pending_packet.header = byte;
                g_uart_ctrl.byte_count = 1;
                g_uart_ctrl.state = UART_STATE_COLLECTING;
            }
            break;

        case UART_STATE_COLLECTING:

            switch (g_uart_ctrl.byte_count) {
                case 1:
                    g_uart_ctrl.pending_packet.cmd = byte;
                    g_uart_ctrl.byte_count++;
                    break;
                case 2:
                    g_uart_ctrl.pending_packet.iq_ref = (int16_t)((byte << 8) & 0xFF00);
                    g_uart_ctrl.byte_count++;
                    break;
                case 3:
                    g_uart_ctrl.pending_packet.iq_ref |= (int16_t)(byte & 0x00FF);
                    g_uart_ctrl.byte_count++;
                    break;
                case 4:
                    g_uart_ctrl.pending_packet.theta_e = (uint16_t)((byte << 8) & 0xFF00);
                    g_uart_ctrl.byte_count++;
                    break;
                case 5:
                    g_uart_ctrl.pending_packet.theta_e |= (uint16_t)(byte & 0x00FF);
                    g_uart_ctrl.byte_count++;
                    break;
                case 6:

                    g_uart_ctrl.pending_packet.crc = byte;
                    g_uart_ctrl.byte_count++;

                    if (MotorUart_VerifyChecksum(&g_uart_ctrl.pending_packet)) {
                        g_uart_ctrl.current_packet = g_uart_ctrl.pending_packet;
                        g_uart_ctrl.valid_packets++;
                        g_uart_ctrl.state = UART_STATE_COMPLETE;
                    } else {
                        g_uart_ctrl.crc_errors++;
                        MotorUart_ResetState();
                    }
                    break;

                default:

                    MotorUart_ResetState();
                    break;
            }
            break;

        case UART_STATE_COMPLETE:

            if (byte == UART_PACKET_HEADER) {
                g_uart_ctrl.pending_packet.header = byte;
                g_uart_ctrl.byte_count = 1;
                g_uart_ctrl.state = UART_STATE_COLLECTING;
            }
            break;
    }
}

bool MotorUart_ProcessPacket(void)
{
    if (g_uart_ctrl.state != UART_STATE_COMPLETE) {
        return false;  
    }

    g_uart_ctrl.state = UART_STATE_WAIT_HEADER;

    MotorCommand_t cmd = (MotorCommand_t)g_uart_ctrl.current_packet.cmd;

    uint8_t forward_buf[7];
    forward_buf[0] = g_uart_ctrl.current_packet.header;
    forward_buf[1] = g_uart_ctrl.current_packet.cmd;
    forward_buf[2] = (g_uart_ctrl.current_packet.iq_ref >> 8) & 0xFF;
    forward_buf[3] = g_uart_ctrl.current_packet.iq_ref & 0xFF;
    forward_buf[4] = (g_uart_ctrl.current_packet.theta_e >> 8) & 0xFF;
    forward_buf[5] = g_uart_ctrl.current_packet.theta_e & 0xFF;
    forward_buf[6] = g_uart_ctrl.current_packet.crc;

    CircularBuffer_Write(&uart3_tx_fifo, forward_buf, 7);

    switch (cmd) {
        case MOTOR_CMD_DISABLE:
            FOC_SetEnable(false);
            break;

        case MOTOR_CMD_ENABLE_TORQUE: {
            float target = MotorUart_DecodeIqRef(g_uart_ctrl.current_packet.iq_ref);
            float theta = MotorUart_DecodeAngle(g_uart_ctrl.current_packet.theta_e);

            FOC_SetControlMode(FOC_MODE_TORQUE);
            FOC_SetEnable(true);
            FOC_SetReferences(0.0f, target, theta);
            break;
        }

        case MOTOR_CMD_ENABLE_VELOCITY: {

            float target_vel = MotorUart_DecodeIqRef(g_uart_ctrl.current_packet.iq_ref) * 10.0f;
            float theta = MotorUart_DecodeAngle(g_uart_ctrl.current_packet.theta_e);

            FOC_SetControlMode(FOC_MODE_VELOCITY);
            FOC_SetEnable(true);
            FOC_SetReferences(0.0f, target_vel, theta);
            break;
        }

        case MOTOR_CMD_ENABLE_POSITION: {

            float target_pos = MotorUart_DecodeIqRef(g_uart_ctrl.current_packet.iq_ref) * 10.0f;
            float theta = MotorUart_DecodeAngle(g_uart_ctrl.current_packet.theta_e);

            FOC_SetControlMode(FOC_MODE_POSITION);
            FOC_SetEnable(true);
            FOC_SetReferences(0.0f, target_pos, theta);
            break;
        }

        case MOTOR_CMD_ENABLE_HOMING: {

            FOC_SetControlMode(FOC_MODE_HOMING);
            FOC_SetEnable(true);
            break;
        }

        default:
            return false;
    }

    return true;
}

void MotorUart_ProcessForward(void)
{

    if (huart3.gState != HAL_UART_STATE_READY) {
        return;
    }

    uint32_t available = CircularBuffer_Available(&uart3_tx_fifo);
    if (available > 0) {
        uint32_t to_send = (available > sizeof(uart3_dma_buffer)) ? sizeof(uart3_dma_buffer) : available;

        if (CircularBuffer_Read(&uart3_tx_fifo, uart3_dma_buffer, to_send) > 0) {
            HAL_UART_Transmit_IT(&huart3, uart3_dma_buffer, to_send);
        }
    }
}

void MotorUart_SendTelemetry(void)
{

}

const UartControlBlock_t* MotorUart_GetStats(void)
{
    return &g_uart_ctrl;
}

void MotorUart_SendPacket(uint8_t cmd, float iq_ref, float theta_e)
{
    MotorUartPacket_t packet;

    packet.header = UART_PACKET_HEADER;
    packet.cmd = cmd;
    packet.iq_ref = MotorUart_EncodeIqRef(iq_ref);
    packet.theta_e = MotorUart_EncodeAngle(theta_e);
    packet.crc = MotorUart_CalcChecksum(&packet);

    uart3_packet_buffer[0] = packet.header;
    uart3_packet_buffer[1] = packet.cmd;
    uart3_packet_buffer[2] = (packet.iq_ref >> 8) & 0xFF;
    uart3_packet_buffer[3] = packet.iq_ref & 0xFF;
    uart3_packet_buffer[4] = (packet.theta_e >> 8) & 0xFF;
    uart3_packet_buffer[5] = packet.theta_e & 0xFF;
    uart3_packet_buffer[6] = packet.crc;

    if (huart3.gState == HAL_UART_STATE_READY) {
        (void)HAL_UART_Transmit_IT(&huart3, uart3_packet_buffer, UART_PACKET_SIZE);
    }
}

static void MotorUart_ResetState(void)
{
    g_uart_ctrl.state = UART_STATE_WAIT_HEADER;
    g_uart_ctrl.byte_count = 0;
}

static bool MotorUart_VerifyChecksum(const MotorUartPacket_t *packet)
{
    uint8_t crc = 0;

    crc ^= packet->cmd;
    crc ^= (packet->iq_ref >> 8) & 0xFF;
    crc ^= packet->iq_ref & 0xFF;
    crc ^= (packet->theta_e >> 8) & 0xFF;
    crc ^= packet->theta_e & 0xFF;

    return (crc == packet->crc);
}

static uint8_t MotorUart_CalcChecksum(const MotorUartPacket_t *packet)
{
    uint8_t crc = 0;

    crc ^= packet->cmd;
    crc ^= (packet->iq_ref >> 8) & 0xFF;
    crc ^= packet->iq_ref & 0xFF;
    crc ^= (packet->theta_e >> 8) & 0xFF;
    crc ^= packet->theta_e & 0xFF;

    return crc;
}

static float MotorUart_DecodeIqRef(int16_t q15_value)
{

    return (float)q15_value * (10.0f / 32768.0f);
}

static float MotorUart_DecodeAngle(uint16_t q16_value)
{

    return (float)q16_value * (6.283185307f / 65536.0f);
}

static int16_t MotorUart_EncodeIqRef(float iq_float)
{

    if (iq_float > 10.0f) iq_float = 10.0f;
    if (iq_float < -10.0f) iq_float = -10.0f;

    int16_t q15 = (int16_t)(iq_float * 3276.8f);

    return q15;
}

static uint16_t MotorUart_EncodeAngle(float theta_float)
{

    while (theta_float >= 6.283185307f) theta_float -= 6.283185307f;
    while (theta_float < 0) theta_float += 6.283185307f;

    uint16_t q16 = (uint16_t)((theta_float / 6.283185307f) * 65536.0f);

    return q16;
}
