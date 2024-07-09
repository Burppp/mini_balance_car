//
// Created by xhuanc on 2021/10/13.
//

/*  Include */
#include <filter.h>
#include "Gimbal.h"
#include "cmsis_os.h"
#include "launcher.h"
//#include "can_receive.h"
#include "user_lib.h"
#include "Atti.h"
//#include "bsp_laser.h"
#include "Auto.h"
#include "key_board.h"
#include "Referee.h"
#include "Detection.h"
#include "bsp_servo_pwm.h"
#include "bsp_led.h"
/*      define      */

/*      变量      */
gimbal_t gimbal;
int16_t y_cnt=0;
fp32 gyro_yaw=0.0f;
fp32 gyro_pitch=0.0f;
extern int32_t total_ecd_ref;
/*     结构体      */
extern RC_ctrl_t rc_ctrl;
extern led_t led;
extern launcher_t launcher;
extern Eulr_t Eulr;
extern key_board_t KeyBoard;
extern osThreadId ChassisTaskHandle;
extern robot_ctrl_info_t robot_ctrl;
extern vision_t vision_data;
extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];
/*      滤波      */
first_order_filter_type_t pitch_first_order_set;
first_order_filter_type_t pitch_current_first_order_set;
first_order_filter_type_t filter_yaw_gyro_in;
first_order_filter_type_t filter_pitch_gyro_in;
first_order_filter_type_t mouse_in_y;
first_order_filter_type_t mouse_in_x;
first_order_filter_type_t auto_pitch;
first_order_filter_type_t auto_yaw;
first_kalman_filter_t filter_autoYaw;
moving_Average_Filter MF_auto_yaw={.length=3};//yaw??????????
moving_Average_Filter MF_auto_pitch={.length=3};
second_lowPass_filter pitch_current_out;
second_lowPass_filter pitch_speed_out;


/*      函数及声明   */
static void gimbal_init();
static void gimbal_mode_set();
static void gimbal_back_handle();
static void gimbal_active_handle();
static void gimbal_relax_handle();
static void gimbal_ctrl_loop_cal();
static void gimbal_man_ctrl_loop_cal();
static void gimbal_angle_update();
static void gimbal_auto_handle();
static void pit_offset_get();
static void gimbal_device_offline_handle();
static void gimbal_turn_back_judge();
static void magazine_cover_control();
static void gimbal_uiInfo_packet();
static void gimbal_power_stop();
static void gimbal_mode_change();
static void gimbal_can_send_back_mapping();



void gimbal_task(void const*pvParameters)
{
    //任务初始化时间
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //云台初始化
    gimbal_init();

    //发射机构初始化
    launcher_init();

    vTaskResume(ChassisTaskHandle);

    while(1){
//        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误

        gimbal_angle_update();//更新绝对、相对角度接收值

        gimbal_mode_set();//根据遥控器设置云台控制模式

        launcher_mode_set();//发射模式设置

        switch (gimbal.mode) {

            case GIMBAL_RELAX://云台失能
                gimbal_relax_handle();
                break;

            case GIMBAL_BACK://云台回中
                gimbal_back_handle();
                break;

            case GIMBAL_ACTIVE://云台控制
                gimbal_active_handle();  //得到遥控器对云台电机的控制
                gimbal_ctrl_loop_cal();
                break;

            case GIMBAL_AUTO:
            case GIMBAL_BUFF:
            case GIMBAL_SBUFF:
            {
                gimbal_auto_handle();
                gimbal_ctrl_loop_cal();
            }
                break;
        }

        magazine_cover_control();//弹仓盖开合控制

        launcher_control();//发射机构控制
        gimbal_device_offline_handle();//检测离线

        gimbal_can_send_back_mapping();

//        xTaskResumeAll();
        vTaskDelay(1);
    }
}

pid_t auto_yaw_angle_pid;
pid_t auto_yaw_speed_pid;
static void gimbal_init(){

    gimbal.yaw.motor_measure=&motor_yaw_measure;
    gimbal.pitch.motor_measure=&motor_pitch_measure;

    gimbal.mode=gimbal.last_mode=GIMBAL_RELAX;//初始化默认状态为失能

    //yaw轴电机 角度环和速度环PID初始化
    pid_init(&gimbal.yaw.angle_p,
             GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI,
             GIMBAL_YAW_ANGLE_PID_KD);

    pid_init(&gimbal.yaw.speed_p,
             GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI,
             GIMBAL_YAW_SPEED_PID_KD);

    //pit轴电机 角度环和速度环PID初始化
    pid_init(&gimbal.pitch.angle_p,
             GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT,
             GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI,
             GIMBAL_PITCH_ANGLE_PID_KD);

    pid_init(&gimbal.pitch.speed_p,
             GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT,
             GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI,
             GIMBAL_PITCH_SPEED_PID_KD);

//    pid_init(&auto_yaw_angle_pid, GIMBAL_AUTO_YAW_ANGLE_MAX_OUT,
//             GIMBAL_AUTO_YAW_ANGLE_MAX_IOUT,
//             GIMBAL_AUTO_YAW_ANGLE_PID_KP,
//             GIMBAL_AUTO_YAW_ANGLE_PID_KI
//             , GIMBAL_AUTO_YAW_ANGLE_PID_KD);
//
//
//    pid_init(&auto_yaw_speed_pid, GIMBAL_AUTO_YAW_SPEED_MAX_OUT,
//             GIMBAL_AUTO_YAW_SPEED_MAX_IOUT,
//             GIMBAL_AUTO_YAW_SPEED_PID_KP,
//             GIMBAL_AUTO_YAW_SPEED_PID_KI
//            , GIMBAL_AUTO_YAW_SPEED_PID_KD);


    first_order_filter_init(&pitch_first_order_set, 0.f, 500);
    first_order_filter_init(&pitch_current_first_order_set, 5, 30);
    first_order_filter_init(&filter_yaw_gyro_in, 5, 30);
    first_order_filter_init(&mouse_in_x, 1, 40);
    first_order_filter_init(&mouse_in_y, 0.5, 20);
    first_order_filter_init(&filter_pitch_gyro_in, 1, 20);
    first_order_filter_init(&auto_pitch, 1, 30);
    first_order_filter_init(&auto_yaw, 1, 30);

    //卡尔曼滤波
    first_Kalman_Create(&filter_autoYaw,1,20);

    SetCutoffFreq(&pitch_current_out,500,188);
    SetCutoffFreq(&pitch_speed_out,500,188 );

    //初始化时 云台设为未回中状态
    gimbal.yaw_is_back=0;
    gimbal.pitch_is_back=0;

    //上电时默认先设置成失能模式，再切换到当前遥控设置模式
    gimbal.last_mode=GIMBAL_RELAX;

    //yaw轴和pitch轴电机的校准编码值
    gimbal.yaw.motor_measure->offset_ecd=7492;
    gimbal.pitch.motor_measure->offset_ecd=3196;

    //4号ecd
    // gimbal.yaw.motor_measure->offset_ecd=6700;
    //gimbal.pitch.motor_measure->offset_ecd=6943;
}

//云台模式设置（获取遥控器信息，判断模式）
static void gimbal_mode_set(){

    //根据遥控器设置云台模式
    switch (rc_ctrl.rc.s[RC_s_R]) {

        case RC_SW_DOWN:
        {
            gimbal.mode=GIMBAL_RELAX;
            gimbal.last_mode=gimbal.mode;
            break;
        }

        case RC_SW_MID:
        case RC_SW_UP:
        {
            gimbal.last_mode=gimbal.mode;

            if(gimbal.last_mode==GIMBAL_RELAX)
            {
                gimbal.mode=GIMBAL_BACK;
                gimbal.yaw_is_back=0;
                gimbal.pitch_is_back=0;
            }

//            else if(gimbal.mode==GIMBAL_BACK && gimbal.pitch_is_back==1)
            else if(1)
            {
                gimbal.mode=GIMBAL_ACTIVE;
            }

            break;
        }

        default:{
            break;
        }
    }

    gimbal_mode_change();

}

static void gimbal_mode_change() {
    if (gimbal.mode == GIMBAL_ACTIVE) {   //自瞄判定
        if ((KeyBoard.Mouse_r.status == KEY_PRESS) || (rc_ctrl.rc.ch[4] > 300)) {
            vision_data.mode = 0x21;
        } else {
            vision_data.mode = 0;
        }
        if ((KeyBoard.Mouse_r.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE)) ||
            (rc_ctrl.rc.ch[4]>300 && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE))) {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_AUTO;
        }
//            robot_ctrl.target_2   lock=0x32;

        if (KeyBoard.C.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE)) {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_BUFF;
        }

        if (KeyBoard.X.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE)) {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_SBUFF;
        }

    } else if (gimbal.mode == GIMBAL_AUTO) {   //自瞄失效判定                                                //0x32表示自瞄数据无效
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) || robot_ctrl.target_lock == 0x32) {
            gimbal.last_mode = GIMBAL_AUTO;
            gimbal.mode = GIMBAL_ACTIVE;//默认回到一般模式
            vision_data.mode = 0;
        }
    } else if (gimbal.mode == GIMBAL_BUFF) {
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) ||
            !(KeyBoard.C.status == KEY_PRESS && robot_ctrl.target_lock == 0x31)) {
            gimbal.last_mode = GIMBAL_BUFF;
            gimbal.mode = GIMBAL_ACTIVE;
        }
    } else if (gimbal.mode == GIMBAL_SBUFF) {
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) ||
            !(KeyBoard.X.status == KEY_PRESS && robot_ctrl.target_lock == 0x31)) {
            gimbal.last_mode = GIMBAL_SBUFF;
            gimbal.mode = GIMBAL_ACTIVE;
        }
    }
}

    //UI更新---云台模式

static void gimbal_can_send_back_mapping(){
    int16_t *real_motor_give_current[4];

    real_motor_give_current[0] = &launcher.fire_l.give_current;
    real_motor_give_current[1] = &gimbal.pitch.give_current;
    real_motor_give_current[2] = &launcher.fire_r.give_current;
    real_motor_give_current[3] = &launcher.trigger.give_current;

    CAN_cmd_motor(CAN_1,
                  CAN_MOTOR_0x1FF_ID,
                  gimbal.yaw.give_current,
                  0,
                  0,
                  0);

//    CAN_cmd_motor(CAN_1,
//                  CAN_MOTOR_0x1FF_ID,
//                  0,
//                  0,
//                  0,
//                  0);

    CAN_cmd_motor(CAN_2,
                  CAN_MOTOR_0x1FF_ID,
                  launcher.fire_r.give_current,
                  gimbal.pitch.give_current,
                  launcher.fire_l.give_current,
                  launcher.trigger.give_current);

//    CAN_cmd_motor(CAN_2,
//                  CAN_MOTOR_0x1FF_ID,
//                  0,
//                  gimbal.pitch.give_current,
//                  0,
//                  launcher.trigger.give_current);
}

//回中处理函数（判断云台是否回中，将标识符置1）
static void gimbal_back_handle(){

    if(gimbal.yaw.relative_angle_get>-2&&gimbal.yaw.relative_angle_get<2)
    {
        gimbal.yaw_is_back=1;
    }
    else {
        gimbal.yaw.relative_angle_set=0;
    }

    if(gimbal.pitch.relative_angle_get>-2&&gimbal.pitch.relative_angle_get<2)
    {
        gimbal.pitch_is_back=1;
    }
    else {
        //先通过相对角0度回中，再将绝对角设为0度准备进行控制
        gimbal.pitch.relative_angle_set=0;
        gimbal.pitch.absolute_angle_set=0;
        //对pitch回中时进行动态限位，防止坡上复活出现超过限位的情况
        gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                       MIN_ABS_ANGLE+
                                                       gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get,
                                                       MAX_ABS_ANGLE+
                                                       gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get);
//        gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
//                                                       gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get,
//                                                       gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get);
    }

    //在没有回中时，每次执行函数都进行闭环控制回中
    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.relative_angle_get,
                                    gimbal.pitch.relative_angle_set);

    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);
    gimbal.pitch.give_current= pid_calc(&gimbal.pitch.speed_p,
                                        filter_pitch_gyro_in.out,
                                        gimbal.pitch.gyro_set);//加负号为了电机反转

    //在回中完成后yaw轴按照当前相对角为0度为初始角度控制
    gimbal.yaw.absolute_angle_set=gimbal.yaw.absolute_angle_get;
    total_ecd_ref=launcher.trigger.motor_measure->total_ecd;
}
//使能模式
static void gimbal_active_handle(){
    //在yaw期望值上按遥控器进行增减

    //鼠标输入滤波
    first_order_filter_cali(&mouse_in_x,rc_ctrl.mouse.x);
    //云台期望设置
    gimbal.yaw.absolute_angle_set-=rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_YAW*GIMBAL_RC_MOVE_RATIO_YAW
                                   +mouse_in_x.out*MOUSE_X_RADIO;

    //一键掉头判断
    gimbal_turn_back_judge();

    //鼠标输入滤波
    first_order_filter_cali(&mouse_in_y,rc_ctrl.mouse.y);
    //云台期望设置
    gimbal.pitch.absolute_angle_set+=rc_ctrl.rc.ch[PITCH_CHANNEL]*RC_TO_PITCH*GIMBAL_RC_MOVE_RATIO_PIT
                                     -mouse_in_y.out*MOUSE_Y_RADIO;

//    VAL_LIMIT(gimbal.pitch.absolute_angle_set,MAX_RELA_ANGLE,MIN_RELA_ANGLE);

    //云台绕圈时进行绝对角循环设置
    if(gimbal.yaw.absolute_angle_set>=180){
        gimbal.yaw.absolute_angle_set-=360;
    }
    else if(gimbal.yaw.absolute_angle_set<=-180){
        gimbal.yaw.absolute_angle_set+=360;
    }


    //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
    gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                   MIN_RELA_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get,
                                                   MAX_RELA_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get);

}
fp32 radio_y=0.0098f;
fp32 radio_p=0.01f;
fp32 auto_yaw_add;
int16_t p_cnt=0;

static void gimbal_auto_handle(){

//    first_Kalman_Filter(&filter_autoYaw,Vision_yaw_add);
//    //获取视觉发送的角度误差
//    if(!isnan(MF_auto_yaw.aver_num))
//        gimbal.yaw.absolute_angle_set=MF_auto_yaw.aver_num;
//    else
//        average_clear(&MF_auto_yaw);
//    if(!isnan(MF_auto_pitch.aver_num))
//        gimbal.pitch.absolute_angle_set=MF_auto_pitch.aver_num;
//    else
//        average_clear(&MF_auto_pitch);

    //获取视觉发送的角度误差
//    average_add(&MF_auto_yaw,robot_ctrl.yaw);
//    average_add(&MF_auto_yaw,robot_ctrl.pitch);
    first_order_filter_cali(&auto_pitch, robot_ctrl.pitch);
    first_order_filter_cali(&auto_yaw, robot_ctrl.yaw);
    if(KeyBoard.F.click_flag == 0) {
        if(robot_ctrl.yaw >178 || robot_ctrl.yaw < -178) {
            gimbal.yaw.absolute_angle_set = robot_ctrl.yaw;
        }
        else{
            gimbal.yaw.absolute_angle_set = auto_yaw.out;
        }
    }
    else{
        //鼠标输入滤波
        first_order_filter_cali(&mouse_in_x,rc_ctrl.mouse.x);
        //云台期望设置
        gimbal.yaw.absolute_angle_set-=rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_YAW*GIMBAL_RC_MOVE_RATIO_YAW
                                       +mouse_in_x.out*MOUSE_X_RADIO;
    }

        gimbal.pitch.absolute_angle_set=auto_pitch.out;

//    gimbal.yaw.absolute_angle_set=robot_ctrl.yaw;
//
//    gimbal.pitch.absolute_angle_set=robot_ctrl.pitch;

    //云台绕圈时进行绝对角循环设置
    if(gimbal.yaw.absolute_angle_set>=180){
        gimbal.yaw.absolute_angle_set-=360;
    }
    else if(gimbal.yaw.absolute_angle_set<=-180){
        gimbal.yaw.absolute_angle_set+=360;
    }
}
//失能模式处理（两轴电流为0）
static void gimbal_relax_handle(){
    gimbal.yaw.give_current=0;
    gimbal.pitch.give_current=0;
    //TODO: 拨弹电机电流 gimbal.trigger.give_current=0;
}

//云台上相关电机离线 则给电机0电流
static void gimbal_device_offline_handle() {
    if(detect_list[DETECT_REMOTE].status == OFFLINE){
        gimbal.pitch.give_current = 0;
        gimbal.yaw.give_current = 0;
//        launcher.fire_l.speed = 0;
//        launcher.fire_r.speed = 0;
        launcher.trigger.give_current = 0;
    }
    if (detect_list[DETECT_GIMBAL_6020_PITCH].status == OFFLINE) {
        gimbal.pitch.give_current = 0;
    }
    if (detect_list[DETECT_GIMBAL_6020_YAW].status == OFFLINE) {
        gimbal.yaw.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_L].status == OFFLINE) {
        launcher.fire_l.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_R].status == OFFLINE) {
        launcher.fire_r.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_2006_TRIGGER].status == OFFLINE) {
        launcher.trigger.give_current = 0;
    }
}

//云台电机闭环控制函数
fp32 radio_rc=0.8f;
int16_t pitch_LF2_test;
static void gimbal_ctrl_loop_cal(){

    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                       gimbal.yaw.absolute_angle_get,
                                       gimbal.yaw.absolute_angle_set,
                                       180,
                                       -180);

    first_order_filter_cali(&filter_yaw_gyro_in, gyro_yaw);

    gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
                                      gyro_yaw,
                                      gimbal.yaw.gyro_set);


////调试速度环用（注意角度环所输出的量级可能与单环时给的set不同，可能会导致在调角度环时的问题，建议双环一起调）
//    first_order_filter_cali(&filter_yaw_gyro_in,gyro_yaw);
//    gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
//                                      filter_yaw_gyro_in.out,
//                                      -rc_ctrl.rc.ch[YAW_CHANNEL]);


    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.absolute_angle_get,
                                    gimbal.pitch.absolute_angle_set);

    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);

    gimbal.pitch.give_current=-pid_calc(&gimbal.pitch.speed_p,
                                        filter_pitch_gyro_in.out,
                                        gimbal.pitch.gyro_set);//加负号为了电机反转

//    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);
//    gimbal.pitch.give_current= -pid_calc(&gimbal.pitch.speed_p,
//                                      filter_pitch_gyro_in.out,
//                                      rc_ctrl.rc.ch[PITCH_CHANNEL]*0.5);

    gimbal.pitch.give_current= (int16_t)Apply(&pitch_current_out,gimbal.pitch.give_current);
}


static void gimbal_man_ctrl_loop_cal(){
    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                       gimbal.yaw.absolute_angle_get,
                                       gimbal.yaw.absolute_angle_set,
                                       180,
                                       -180);

    first_order_filter_cali(&filter_yaw_gyro_in, gyro_yaw);

    gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
                                      gyro_yaw,
                                      gimbal.yaw.gyro_set);

    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.absolute_angle_get,
                                    gimbal.pitch.absolute_angle_set);
    gyro_pitch=Apply(&pitch_speed_out,gyro_pitch);
    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);

//    first_order_filter_cali(&pitch_first_order_set,gimbal.pitch.gyro_set);

    gimbal.pitch.give_current= -pid_calc(&gimbal.pitch.speed_p,
                                         gyro_pitch,
                                         gimbal.pitch.gyro_set);//?????????????

    gimbal.pitch.give_current= (int16_t)Apply(&pitch_current_out,gimbal.pitch.give_current);
}


//云台角度更新
static void gimbal_angle_update(){
    gimbal.pitch.absolute_angle_get = (fp32) INS_angle[2] * MOTOR_RAD_TO_ANGLE;
    gimbal.pitch.relative_angle_get = -motor_ecd_to_angle_change(gimbal.pitch.motor_measure->ecd,
                                                                gimbal.pitch.motor_measure->offset_ecd);

    gimbal.yaw.absolute_angle_get = (fp32) INS_angle[0] * MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.relative_angle_get = motor_ecd_to_angle_change(gimbal.yaw.motor_measure->ecd,
                                                              gimbal.yaw.motor_measure->offset_ecd);
    gimbal.absolute_gyro_yaw = (fp32) INS_gyro[2];
    gimbal.absolute_gyro_pitch = (fp32) INS_gyro[0];

    if (Referee.GameRobotStat.robot_id<10){
        vision_data.id = 7;
    }else{
        vision_data.id = 107;
    }
    vision_data.yaw = gimbal.yaw.absolute_angle_get;
    vision_data.pitch = gimbal.pitch.absolute_angle_get;
    vision_data.shoot_speed = Referee.ShootData.bullet_speed;
    vision_data.roll = (fp32) INS_angle[1] * MOTOR_RAD_TO_ANGLE;
    for (int i = 0; i < 4; ++i) {
        vision_data.quaternion[i] = INS_quat[i];
    }
    rm_queue_data(VISION_ID, &vision_data, sizeof(vision_t));
}

//通过这个函数，云台检测到pit绝对角为接近0时，获取当前pit_offset。在调试模式获取了offset后，在初始化函数中更改。
static void pit_offset_get(){
    if(gimbal.pitch.absolute_angle_get<=-0.002 && gimbal.pitch.absolute_angle_get>=0.002){
        gimbal.pitch.motor_measure->offset_ecd=motor_pitch_measure.ecd;//在这一行打断点调试，触发时成功获取0度时offset
    }
}

//一键掉头
static void gimbal_turn_back_judge(){
    if(KeyBoard.R.click_flag == 1){
        KeyBoard.R.click_flag = 0;
        gimbal.yaw.absolute_angle_set+=180;
    }
}

//弹仓盖控制
static void magazine_cover_control() {
    if (KeyBoard.G.click_flag == 1 || rc_ctrl.rc.ch[4]<-300) {//搓那个滚轮
        servo_pwm_set(1800, 2);//TIM_CHANNEL_4||左弹仓
        servo_pwm_set(800, 3);//TIM_CHANNEL_3||右弹仓
    } else {
        servo_pwm_set(800, 2);
        servo_pwm_set(1800, 3);
    }
}

static void gimbal_uiInfo_packet(){
    ui_robot_status.gimbal_mode=gimbal.mode;
    ui_robot_status.fire_mode=launcher.fire_mode;
    ui_robot_status.relative_yaw_value=gimbal.yaw.relative_angle_get;
    ui_robot_status.pitch_value=gimbal.pitch.absolute_angle_get;
    //?????????????灯
//    if(ABS(launcher.fire_l.motor_measure->speed_rpm)>500&&ABS(launcher.fire_r.motor_measure->speed_rpm)>500)
////        HAL_GPIO_WritePin(LED7_PORT,LED5_PIN,GPIO_PIN_RESET);
//    else
////        HAL_GPIO_WritePin(LED7_PORT,LED5_PIN,GPIO_PIN_SET);
    if(ABS(launcher.fire_l.motor_measure->speed_rpm)>500&&ABS(launcher.fire_r.motor_measure->speed_rpm)>500)
        led.mode=SHOOT;

}

static void gimbal_power_stop(){
    if(Referee.GameRobotStat.mains_power_gimbal_output==0)
    {
        gimbal.mode=GIMBAL_RELAX;
    }

    if(Referee.GameRobotStat.mains_power_shooter_output==0)
    {
        launcher.fire_l.give_current = 0;
        launcher.fire_r.give_current = 0;
        launcher.trigger.give_current = 0;
    }
}