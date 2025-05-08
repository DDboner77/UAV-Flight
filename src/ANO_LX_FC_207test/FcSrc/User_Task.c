#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "User_Com.h"    // ***** 新增，确保能访问 user_pwm *****

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step;
    // ***** 舵机控制相关变量 *****
    static u8 servo_state = 0;         // 0=等待触发，1=+90，2=0度，3=等待3s
    static uint32_t servo_timer = 0;
    static u8 last_ch7_state = 0;
    // *****

    //判断有遥控信号才执行
    if (rc_in.no_signal == 0)
    {
        //判断第6通道拨杆位置 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            if (one_key_takeoff_f == 0)
            {
                one_key_takeoff_f =
                    OneKey_Takeoff(100);
            }
        }
        else
        {
            one_key_takeoff_f = 0;
        }
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            if (one_key_land_f == 0)
            {
                one_key_land_f =
                    OneKey_Land();
            }
        }
        else
        {
            one_key_land_f = 0;
        }

        // ***** 新增：第七通道控制9g舵机 *****
        // 假设第七通道为 ch_7_aux3，实际请根据你的通道定义修改
        u16 ch7_val = rc_in.rc_ch.st_data.ch_[ch_7_aux3];
        u8 ch7_state = (ch7_val > 1300) ? 1 : 0;
        static u8 servo_step = 0; // 0:500, 1:1500, 2:2500
        if (ch7_state && !last_ch7_state) {
            // 每次ch7由低到高，切换一次
            if (servo_step == 0) {
                user_pwm[0] = 500;
                servo_step = 1;
            } else if (servo_step == 1) {
                user_pwm[0] = 1500;
                servo_step = 2;
            } else {
                user_pwm[0] = 2500;
                servo_step = 0;
            }
			
        }
        last_ch7_state = ch7_state;
        // *****

    }
    ////////////////////////////////////////////////////////////////////////
}