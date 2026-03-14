#ifndef __SPEED_CTRL_H__
#define __SPEED_CTRL_H__


#define SPEED_ADD_STEP              (500/1000.0f)
#define SPEED_SUB_STEP              (500/1000.0f)
#define SPEED_ID_ADD_STEP           (1.5/1000.0f)
#define SPEED_ID_SUB_STEP           (1.5/1000.0f)
#define SPEED_SWITCH_ID_SUB_STEP    (0.0005f)

enum SpeedCtrlState_t {
    SPEED_CTRL_IDLE = 0,
    SPEED_CTRL_ALIGN,
    SPEED_CTRL_OPEN,
    SPEED_CTRL_SWITCH,
    SPEED_CTRL_RUN,
    SPEED_CTRL_WAIT,
};

void Speed_Ctrl_Init(void);
void Speed_Ctrl_Task(void);


#endif // __SPEED_CTRL_H__
