#include "Can.h"
#include "fdcan.h"
#include "CAN_DBC_Messages.h"
#include "Speed_Loop.h"
#include "Current_Loop.h"
#include "Observer.h"

static uint8_t can_cnt;
uint8_t fdcanrx_Data[16];
uint8_t send_buffer[16];

FDCAN_TxHeaderTypeDef tx_Head;

CAN_Motor_para_t can_dbc_msg;
CAN_Motor_List1_t can_dbc_list1;
CAN_Motor_List2_t can_dbc_list2;
extern struct SMO_Parameter SMO_OB;
extern struct NonFluxObserver_Parameter NonFlux_OB;
extern Speed_Loop_t Speed_Loop;
extern Current_Loop_t Current_Loop;
uint8_t Can_Run_request;
float Can_speed_request;

void fdcan_transmit_data(void)
{
    // Prepare the data to be sent
    tx_Head.Identifier = 0x123; // Set the CAN identifier
    tx_Head.IdType = FDCAN_STANDARD_ID;
    tx_Head.TxFrameType = FDCAN_DATA_FRAME;
    tx_Head.DataLength = FDCAN_DLC_BYTES_8;
    tx_Head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // Set error state indicator
    tx_Head.BitRateSwitch = FDCAN_BRS_OFF;           // Disable bit rate switching
    tx_Head.FDFormat = FDCAN_CLASSIC_CAN;            // Use classic CAN format
    tx_Head.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // No Tx events
    tx_Head.MessageMarker = 0;                       // Message marker for identification
    can_dbc_msg.Motor_status = (float)Current_Loop.Motor_State;
    can_dbc_msg.Speed_status = (float)Speed_Loop.spd_ctrl_state;
    can_dbc_msg.Bus_Voltage = (float)(0); // Convert Vbus to integer and scale by 10
    can_dbc_msg.Id = (Current_Loop.Id_fb);                                                   // Scale Id_fb by 4
    can_dbc_msg.Iq = (Current_Loop.Iq_fb);                                                   // Convert Iq_fb to integer
    can_dbc_msg.Speed_rpm = (Speed_Loop.Speed_Fb);                                         // Shift Speed_Fb by 80 to fit in uint8_t
    CAN_Pack_Motor_para(send_buffer, &can_dbc_msg);
    // Transmit the data over FDCAN
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_Head, (uint8_t *)send_buffer);

    tx_Head.Identifier = 0x145; // Set the CAN identifier
    tx_Head.IdType = FDCAN_STANDARD_ID;
    tx_Head.TxFrameType = FDCAN_DATA_FRAME;
    tx_Head.DataLength = FDCAN_DLC_BYTES_8;
    tx_Head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // Set error state indicator
    tx_Head.BitRateSwitch = FDCAN_BRS_OFF;           // Disable bit rate switching
    tx_Head.FDFormat = FDCAN_CLASSIC_CAN;            // Use classic CAN format
    tx_Head.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // No Tx events
    tx_Head.MessageMarker = 0;                       // Message marker for identification
    can_dbc_list1.Signal_1 = (float)SMO_OB.ia_mat_k;
    can_dbc_list1.Signal_2 = (float)SMO_OB.ib_mat_k;
    CAN_Pack_Motor_List1(send_buffer, &can_dbc_list1);
    // Transmit the data over FDCAN
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_Head, (uint8_t *)send_buffer);

    tx_Head.Identifier = 0x150; // Set the CAN identifier
    tx_Head.IdType = FDCAN_STANDARD_ID;
    tx_Head.TxFrameType = FDCAN_DATA_FRAME;
    tx_Head.DataLength = FDCAN_DLC_BYTES_8;
    tx_Head.ErrorStateIndicator = FDCAN_ESI_ACTIVE;    // Set error state indicator
    tx_Head.BitRateSwitch = FDCAN_BRS_OFF;             // Disable bit rate switching
    tx_Head.FDFormat = FDCAN_CLASSIC_CAN;              // Use classic CAN format
    tx_Head.TxEventFifoControl = FDCAN_NO_TX_EVENTS;   // No Tx events
    tx_Head.MessageMarker = 0;                         // Message marker for identification
    can_dbc_list2.Signal_3 = (float)Can_speed_request; // Example value for Signal_3
    can_dbc_list2.Signal_4 = (float)Can_Run_request;   // Example value for Signal_4
    CAN_Pack_Motor_List2(send_buffer, &can_dbc_list2);
    // Transmit the data over FDCAN
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_Head, (uint8_t *)send_buffer);
}

FDCAN_RxHeaderTypeDef rx_Head;

extern uint8_t MOTOR_Run_flag;
extern float Speed_Command;


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // Handle received messages from FDCAN Rx FIFO 0
    // Get the received message

    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_Head, fdcanrx_Data);
        if (rx_Head.Identifier == 0x122)
        {
            // Process the received message with identifier 0x122
            CAN_Motor_CMD_t received_msg;
            CAN_Parse_Motor_CMD(fdcanrx_Data, &received_msg);
            // Now you can use the fields of received_msg as needed
            Speed_Command = (float)received_msg.Motor_Run_Rpm;
            MOTOR_Run_flag = (uint8_t)received_msg.Motor_Run_Req;
        }
    }
}

void Can_Task_Init(void)
{
    // CAN总线初始化代码
    can_cnt = 0;
}

void Can_Task_Run(void)
{
    fdcan_transmit_data();
}