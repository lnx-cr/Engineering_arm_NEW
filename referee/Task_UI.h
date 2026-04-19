#ifndef __TASK_UI_H
#define __TASK_UI_H

#include "main.h"


/******** 图形名称定义 *******/
#define GRAPIC_TARGET_X           0x01  /* 准心十字线 横线 */
#define GRAPIC_TARGET_Y           0x02  /* 准心十字线 竖线 */
#define GRAPIC_CHARACTER_SUPERCAP 0x03  /* 超级电容 */
#define GRAPIC_CHARACTER_CHASSIS  0x04  /* 底盘 */
#define GRAPIC_CHARACTER_STUCK    0x05  /* 卡弹 */
#define GRAPIC_auxiliary_line     0x06  /*辅助线*/
#define GRAPIC_auxiliary_x        0x07  /*辅助线*/
#define GRAPIC_auxiliary_y        0x08  /*辅助线*/
#define GRAPIC_auxiliary_z        0x09  /*辅助线*/

void UI_Target(int operation_tpye, int x, int y);
void UI_DeleteAllGrapic(void);
void Grapic_Character(int operation_tpye, char *char_data, int char_length, int x, int y, int name);
void Grapic_Target(int operation_tpye, int x, int y);
void UI_line(int operation_tpye,int name, int x, int y);
void referee_entry(void);

#endif





