#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;


extern ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;


uint8_t RefereeSendBuf[128];                   /* 发送到裁判系统的数据 */
void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));
    

    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));


    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
    
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;

}


uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_heat0_cooling_limit;
    *heat0 = power_heat_data_t.shooter_heat0;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_heat1_cooling_limit;
    *heat1 = power_heat_data_t.shooter_heat1;
}


/**
 * @brief 将数据段配置帧头，CRC8，帧尾CRC16校验成发送到裁判系统的RefereeSendBuf
 *
 * @param cmd_id 数据段ID
 * @param data   要发送数据段，不含数据头和帧头
 * @param length 要发送数据长度，即内容数据段长度，不含数据头和帧头
 * @return int 整包数据长度
 */
int Referee_ConfigFrameData(uint16_t cmd_id, void *data, int length) {
    referee_send_data_t referee_send_data;
    uint16_t            data_length =
        length + sizeof(ext_student_interactive_header_data_t); /* 包含数据头的长度，写入到数据帧帧头data_length字段 */

    memset(&referee_send_data, 0, sizeof(referee_send_data_t));

    /* 配置帧头 */
    RefereeSendBuf[REFEREE_OFFSET_SOF]             = REFEREE_FRAME_HEADER;
    RefereeSendBuf[REFEREE_OFFSET_DATA_LENGTH + 1] = (data_length & 0xFF00) >> 8;
    RefereeSendBuf[REFEREE_OFFSET_DATA_LENGTH]     = data_length & 0x00FF;
    RefereeSendBuf[REFEREE_OFFSET_SEQ]             = 0;

    /* 帧头数据追加CRC8校验 */
    append_CRC8_check_sum(RefereeSendBuf, REFEREE_LEN_HEADER);

    /* 裁判系统命令码，数据交互固定值0x0301 */
    referee_send_data.cmd_id = 0x0301;

    /* 配置发送者ID，接收者ID */
    referee_send_data.data_header.data_cmd_id = cmd_id;
    referee_send_data.data_header.sender_ID   = robot_state.robot_id;
    if (robot_state.robot_id > 100) 
    { 
        referee_send_data.data_header.receiver_ID =
            0x0167 + (robot_state.robot_id - 103); 
    }
    else 
    {                                             
        referee_send_data.data_header.receiver_ID =
            0x0103 + (robot_state.robot_id - 3); 
    }

    /* 先复制数据到发送数据结构体 */
    memcpy(referee_send_data.data, data, length);

   /* 再将结构体复制到RefereeSendBuf */
    memcpy(RefereeSendBuf + REFEREE_LEN_HEADER, &referee_send_data.cmd_id,
              data_length + REFEREE_LEN_CMDID); 

    /* CRC16 整包校验 */
    append_CRC16_check_sum(RefereeSendBuf, data_length + REFEREE_LEN_CMDID + REFEREE_LEN_HEADER +
                                               REFEREE_LEN_TAIL); 

    return data_length + REFEREE_LEN_CMDID + REFEREE_LEN_HEADER + REFEREE_LEN_TAIL;
}











