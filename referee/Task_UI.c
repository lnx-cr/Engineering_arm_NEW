#include "task_ui.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "referee.h"


/* 图形对象 */
ext_client_custom_graphic_delete_t graphic_delete;    /* 删除所有图形数据 */
ext_client_custom_graphic_double_t graphic_target;    /* 两个图形数据，准心 */
ext_client_custom_character_t      graphic_character; /* 单个图形数据，画字符 */
ext_client_custom_graphic_seven_t  graphic_auxiliary_line;  /* 七个图像数据，瞄准辅助线 */

extern uint8_t RefereeSendBuf[128]; /* 发送到裁判系统的数据 */

/**
 * @brief 准心图形对象操作函数，第一图层，图形索引：GRAPIC_TARGET_X, GRAPIC_TARGET_Y，坐标在target结构体中调用
 *
 * @param operation_tpye 操作类型：1新增图形，2修改图形
 * @param xy x,y坐标
 * @return void
 */
void Grapic_Target(int operation_tpye, int x, int y) 
{
    graphic_target.grapic_data_struct[0].graphic_name[0] = GRAPIC_TARGET_X;
    graphic_target.grapic_data_struct[0].operate_tpye    = operation_tpye; /* 1新增图形 */
    graphic_target.grapic_data_struct[0].graphic_tpye    = 0;              /* 0直线 */
    graphic_target.grapic_data_struct[0].layer           = 1;              /* 第1图层 */
    graphic_target.grapic_data_struct[0].color           = 3;              /* 2：绿色 */
    graphic_target.grapic_data_struct[0].width           = 2;              /* 线宽 */
    graphic_target.grapic_data_struct[0].start_x         = x+30;
    graphic_target.grapic_data_struct[0].end_x           = x+30;
    graphic_target.grapic_data_struct[0].start_y         = y+20;
    graphic_target.grapic_data_struct[0].end_y           = y-200;

    graphic_target.grapic_data_struct[1].graphic_name[0] = GRAPIC_TARGET_Y;
    graphic_target.grapic_data_struct[1].operate_tpye    = operation_tpye; 
    graphic_target.grapic_data_struct[1].graphic_tpye    = 0;              
    graphic_target.grapic_data_struct[1].layer           = 1;              
    graphic_target.grapic_data_struct[1].color           = 4;              
    graphic_target.grapic_data_struct[1].width           = 2;              
    graphic_target.grapic_data_struct[1].start_x         = x-85;
    graphic_target.grapic_data_struct[1].end_x           = x+115;
    graphic_target.grapic_data_struct[1].start_y         = y-30;
    graphic_target.grapic_data_struct[1].end_y           = y-30;
}


/**
 * @brief 字符图形对象操作函数，第二图层，图形索引：GRAPIC_CHARACTER_1
 *
 * @param operation_tpye 操作类型：1新增图形，2修改图形
 * @param char_data      字符数据
 * @param char_length    字符数据长度
 * @param xy             放置的x，y坐标
 * @param name           图形索引名称
 * @return void
 */
void Grapic_Character(int operation_tpye, char *char_data, int char_length, int x, int y, int name) 
{
    graphic_character.grapic_data_struct.graphic_name[0] = name;
    graphic_character.grapic_data_struct.operate_tpye    = operation_tpye;
    graphic_character.grapic_data_struct.graphic_tpye    = 7; 
    graphic_character.grapic_data_struct.layer           = 2;
    graphic_character.grapic_data_struct.color           = 1; 
    graphic_character.grapic_data_struct.width           = 2;  
    graphic_character.grapic_data_struct.start_angle     = 20; 
    graphic_character.grapic_data_struct.end_angle       = char_length; 
    graphic_character.grapic_data_struct.start_x         = x;
    graphic_character.grapic_data_struct.start_y         = y;

    memset(graphic_character.data, 0, 30);
    strncpy(graphic_character.data, char_data, char_length);
}

void Grapic_auxiliaryline(int operation_tpye,int name,int x, int y)
{
    graphic_auxiliary_line.grapic_data_struct[0].graphic_name[0] = name;
    graphic_auxiliary_line.grapic_data_struct[0].operate_tpye    = operation_tpye;
    graphic_auxiliary_line.grapic_data_struct[0].graphic_tpye    =0;
    graphic_auxiliary_line.grapic_data_struct[0].layer           =3;
    graphic_auxiliary_line.grapic_data_struct[0].color           =3;
    graphic_auxiliary_line.grapic_data_struct[0].width           =2;
    //graphic_auxiliary_line.grapic_data_struct->start_angle     =
    graphic_auxiliary_line.grapic_data_struct[0].start_x         =x;
    graphic_auxiliary_line.grapic_data_struct[0].start_y         =y+20;
    graphic_auxiliary_line.grapic_data_struct[0].end_x           =x;
    graphic_auxiliary_line.grapic_data_struct[0].end_y           =y-200;

    graphic_auxiliary_line.grapic_data_struct[1].graphic_name[0] = 0x07;
    graphic_auxiliary_line.grapic_data_struct[1].operate_tpye    = operation_tpye;
    graphic_auxiliary_line.grapic_data_struct[1].graphic_tpye    =0;
    graphic_auxiliary_line.grapic_data_struct[1].layer           =3;
    graphic_auxiliary_line.grapic_data_struct[1].color           =4;
    graphic_auxiliary_line.grapic_data_struct[1].width           =2;
    //graphic_auxiliary_line.grapic_data_struct->start_angle     =
    graphic_auxiliary_line.grapic_data_struct[1].start_x         =x-85;
    graphic_auxiliary_line.grapic_data_struct[1].start_y         =y;
    graphic_auxiliary_line.grapic_data_struct[1].end_x           =x+115;
    graphic_auxiliary_line.grapic_data_struct[1].end_y           =y;

    graphic_auxiliary_line.grapic_data_struct[2].graphic_name[0] = 0x08;
    graphic_auxiliary_line.grapic_data_struct[2].operate_tpye    = operation_tpye;
    graphic_auxiliary_line.grapic_data_struct[2].graphic_tpye    =0;
    graphic_auxiliary_line.grapic_data_struct[2].layer           =3;
    graphic_auxiliary_line.grapic_data_struct[2].color           =3;
    graphic_auxiliary_line.grapic_data_struct[2].width           =2;
    //graphic_auxiliary_line.grapic_data_struct->start_angle     =
    graphic_auxiliary_line.grapic_data_struct[2].start_x         =x+30;
    graphic_auxiliary_line.grapic_data_struct[2].start_y         =y+20;
    graphic_auxiliary_line.grapic_data_struct[2].end_x           =x+30;
    graphic_auxiliary_line.grapic_data_struct[2].end_y           =y-200;

    graphic_auxiliary_line.grapic_data_struct[3].graphic_name[0] = 0x09;
    graphic_auxiliary_line.grapic_data_struct[3].operate_tpye    = operation_tpye;
    graphic_auxiliary_line.grapic_data_struct[3].graphic_tpye    =0;
    graphic_auxiliary_line.grapic_data_struct[3].layer           =3;
    graphic_auxiliary_line.grapic_data_struct[3].color           =4;
    graphic_auxiliary_line.grapic_data_struct[3].width           =2;
    //graphic_auxiliary_line.grapic_data_struct->start_angle     =
    graphic_auxiliary_line.grapic_data_struct[3].start_x         =x-85;
    graphic_auxiliary_line.grapic_data_struct[3].start_y         =y-30;
    graphic_auxiliary_line.grapic_data_struct[3].end_x           =x+115;
    graphic_auxiliary_line.grapic_data_struct[3].end_y           =y-30;

    graphic_auxiliary_line.grapic_data_struct[4].graphic_name[0] = name;
    graphic_auxiliary_line.grapic_data_struct[4].operate_tpye    = operation_tpye;
    graphic_auxiliary_line.grapic_data_struct[4].graphic_tpye    =0;
    graphic_auxiliary_line.grapic_data_struct[4].layer           =3;
    graphic_auxiliary_line.grapic_data_struct[4].color           =3;
    graphic_auxiliary_line.grapic_data_struct[4].width           =2;
    //graphic_auxiliary_line.grapic_data_struct->start_angle     =
    graphic_auxiliary_line.grapic_data_struct[4].start_x         =0;
    graphic_auxiliary_line.grapic_data_struct[4].start_y         =0;
    graphic_auxiliary_line.grapic_data_struct[4].end_x           =0;
    graphic_auxiliary_line.grapic_data_struct[4].end_y           =0;

    graphic_auxiliary_line.grapic_data_struct[5].graphic_name[0] = name;
    graphic_auxiliary_line.grapic_data_struct[5].operate_tpye    = operation_tpye;
    graphic_auxiliary_line.grapic_data_struct[5].graphic_tpye    =0;
    graphic_auxiliary_line.grapic_data_struct[5].layer           =3;
    graphic_auxiliary_line.grapic_data_struct[5].color           =3;
    graphic_auxiliary_line.grapic_data_struct[5].width           =2;
    //graphic_auxiliary_line.grapic_data_struct->start_angle     =
    graphic_auxiliary_line.grapic_data_struct[5].start_x         =0;
    graphic_auxiliary_line.grapic_data_struct[5].start_y         =0;
    graphic_auxiliary_line.grapic_data_struct[5].end_x           =0;
    graphic_auxiliary_line.grapic_data_struct[5].end_y           =0;

    graphic_auxiliary_line.grapic_data_struct[6].graphic_name[0] = name;
    graphic_auxiliary_line.grapic_data_struct[6].operate_tpye    = operation_tpye;
    graphic_auxiliary_line.grapic_data_struct[6].graphic_tpye    =0;
    graphic_auxiliary_line.grapic_data_struct[6].layer           =3;
    graphic_auxiliary_line.grapic_data_struct[6].color           =3;
    graphic_auxiliary_line.grapic_data_struct[6].width           =2;
    //graphic_auxiliary_line.grapic_data_struct->start_angle     =
    graphic_auxiliary_line.grapic_data_struct[6].start_x         =0;
    graphic_auxiliary_line.grapic_data_struct[6].start_y         =0;
    graphic_auxiliary_line.grapic_data_struct[6].end_x           =0;
    graphic_auxiliary_line.grapic_data_struct[6].end_y           =0;
    
}


/**
 * @brief 删除所有图形
 *
 */
void UI_DeleteAllGrapic(void)
{
    int length;

    graphic_delete.operate_tpye = 2;
    length                      = Referee_ConfigFrameData(0x0100, &graphic_delete, 2);

    HAL_UART_Transmit(&REFEREE_UART, RefereeSendBuf, length, 0xFFFF);
}

/**
 * @brief 新增或修改准心坐标
 *
 * @param operation_tpye 操作类型：新增或修改，可选参数：AddGrapic, ModifyGrapic
 * @param xy 准心的xy坐标
 */
void UI_Target(int operation_tpye, int x, int y) 
{
    int length;

    Grapic_Target(operation_tpye, x, y);
    length = Referee_ConfigFrameData(0x0102, &graphic_target, 30);

    HAL_UART_Transmit(&REFEREE_UART, RefereeSendBuf, length, 0xFFFF);
}

void UI_line(int operation_tpye,int name, int x, int y)
{
    int length;

    Grapic_auxiliaryline(operation_tpye,name,x, y);
    length = Referee_ConfigFrameData(0x0104, &graphic_auxiliary_line, 60);

    HAL_UART_Transmit(&REFEREE_UART, RefereeSendBuf, length, 0xFFFF);
}


void referee_entry(void) 
{
    uint8_t is_show_ui = 0;  
    if (is_show_ui) 
    {
        UI_DeleteAllGrapic();
    } 
    else 
    {
        UI_Target(AddGrapic, 960, 445);
    }
    is_show_ui=0;
}









