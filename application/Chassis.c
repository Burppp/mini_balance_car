﻿//
// Created by xhuanc on 2021/10/10.
//

/*include*/
#include "Chassis.h"
/*define*/
/*轮子控制映射：                                 解算坐标：      x(前)
            ****      前       ****                                |
           * 2 LF *          * 1 RF *                              |
            ****              ****                                 |
                                                                   |
           左                   右                 y  --------------z-----------
                                                                   |
            ****              ****                                 |
          * 3 LB *          * 4 RB *                               |
            ****      后      ****                                 |

*/
/*变量*/
extern RC_ctrl_t rc_ctrl;
ramp_function_source_t chassis_auto_vx_ramp;
ramp_function_source_t chassis_auto_vy_ramp;
ramp_function_source_t chassis_auto_vw_ramp;
ramp_function_source_t chassis_3508_ramp[4];
chassis_t chassis;
extern gimbal_t gimbal;
//上位机下发数据
extern robot_ctrl_info_t robot_ctrl;
//底盘解算发送数据
extern chassis_odom_info_t chassis_odom;
//遥控器数据
extern key_board_t KeyBoard;
//电容数据
extern cap2_info_t cap2;
//发送机器人id
vision_t vision_data;
//功率控制中的电流控制
int16_t give_current_limit[4];
static fp32 rotate_ratio_f = ((Wheel_axlespacing + Wheel_spacing) / 2.0f - GIMBAL_OFFSET); //rad
static fp32 rotate_ratio_b = ((Wheel_axlespacing + Wheel_spacing) / 2.0f + GIMBAL_OFFSET);
static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO); //车轮转速比
extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
/*      函数及声明   */
static void chassis_init(chassis_t *chassis_ptr);

static void chassis_set_mode(chassis_t *chassis_ptr);

static void chassis_ctrl_info_get();

static void chassis_relax_handle();

static void chassis_spin_handle();

static void chassis_wheel_cal();

static void chassis_wheel_loop_cal();

void chassis_device_offline_handle();

void send_robot_id();

void chassis_follow_gimbal_handle();

static void chassis_power_limit();

static void chassis_vector_powerControl();

void chassis_can_send_back_mapping();

static void chassis_pc_ctrl();

static float chassis_speed_change();
//创建电流滤波器，输出数据给功率控制系数求解
first_kalman_filter_t currentKal;
first_kalman_filter_t chassis_filter[4];
float Q,R,filterCurrent,nowCurrent;
motor_t motorL;
motor_t motorR;
pid_t standstill_pid;
extern fp32 INS_angle[3];
float ins_angle[3];
uint8_t mode=0;
void change_current_to_pwm(motor_t *motor)
{

    motor->pwm1=(uint16_t)(1000+motor->give_current);
    motor->pwm2=(uint16_t)(1000-motor->give_current);
    if(motor->pwm1>2000)
    {
        motor->pwm1=2000;
    }
    if(motor->pwm2>2000)
    {
        motor->pwm2=2000;
    }
    if(motor->pwm1<0)
    {
        motor->pwm1=0;
    }
    if(motor->pwm2<0)
    {
        motor->pwm2=0;
    }
}
/*程序主体*/
float speed_set;
float aver_speed;
float turn_speed_set;
_Noreturn void chassis_task(void const *pvParameters) {

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
//    chassis_init(&chassis);//底盘初始化
    pid_init(&standstill_pid, 1000, 1000, 38, 0, 750);
    pid_init(&motorL.pid, 500, 200, 14, 0.00f, 90);
    pid_init(&motorR.pid, 500, 200, 14, 0.00f, 90);
    first_Kalman_Create(&motorR.kalman, 1, 1);
    first_Kalman_Create(&motorL.kalman, 1, 1);
    //主任务循环
    while (1) {
        speed_set=(float)(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]) * 0.03f;
        turn_speed_set=(float)(rc_ctrl.rc.ch[2])*0.08f;
        ins_angle[0]=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
        ins_angle[1]=INS_angle[1]*MOTOR_RAD_TO_ANGLE;
        ins_angle[2]=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
        if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
        {
            mode=0;
        }
        if(switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
        {
            mode=1;
        }
        if(mode==0)
        {
            motorL.give_current=0;
            motorR.give_current=0;
            change_current_to_pwm(&motorL);
            change_current_to_pwm(&motorR);
            can_send_motor_lg(&motorL,&motorR);
        }
        else{
            first_Kalman_Filter(&motorL.kalman, motorL.speed);
            first_Kalman_Filter(&motorR.kalman, motorR.speed);
            float angle_loop_out= pid_calc(&standstill_pid, ins_angle[2], -0.8f);
            aver_speed=(-motorL.kalman.X_now+motorR.kalman.X_now)/2;
            float speed_out_r=pid_calc(&motorR.pid, aver_speed, speed_set);
            motorR.give_current=angle_loop_out+speed_out_r+turn_speed_set;
            motorL.give_current=-angle_loop_out-speed_out_r+turn_speed_set;
            change_current_to_pwm(&motorL);
            change_current_to_pwm(&motorR);
            can_send_motor_lg(&motorL,&motorR);
            vTaskDelay(10);
        }

    }

}
void send_robot_id()
{
    if(Referee.GameRobotStat.robot_id<10)//红色方的ID小于10
    {
        vision_data.id=1;
    }
    else{
        vision_data.id=0;
    }
}
static void chassis_init(chassis_t *chassis_ptr) {

    if (chassis_ptr == NULL)
        return ;
    pid_init(&chassis_ptr->chassis_vw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, (uint32_t) CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,
             CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD);
    //底盘驱动电机速度环初始化和电机数据结构体获取
    for (int i = 0; i < 4; i++)
    {

        chassis_ptr->motor_chassis[i].motor_measure= motor_3508_measure + i;

        pid_init(&chassis_ptr->motor_chassis[i].speed_p,
                 CHASSIS_3508_PID_MAX_OUT,
                 CHASSIS_3508_PID_MAX_IOUT,
                 CHASSIS_3508_PID_KP,
                 CHASSIS_3508_PID_KI,
                 CHASSIS_3508_PID_KD);
    }
    //初始时底盘模式为失能
    chassis_ptr->mode = chassis_ptr->last_mode = CHASSIS_RELAX;
    ramp_init(&chassis_3508_ramp[LF],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RF],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RB],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[LB],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_auto_vx_ramp, 0.8f, MAX_CHASSIS_AUTO_VX_SPEED, -MAX_CHASSIS_AUTO_VX_SPEED);
    ramp_init(&chassis_auto_vy_ramp, 0.8f, MAX_CHASSIS_AUTO_VY_SPEED, -MAX_CHASSIS_AUTO_VY_SPEED);
    ramp_init(&chassis_auto_vw_ramp, 0.3f, MAX_CHASSIS_AUTO_VW_SPEED, -MAX_CHASSIS_AUTO_VW_SPEED);

    chassis.spin_mode=NORMAL_SPIN;
    //设定底盘最大功率，比赛根据裁判系统反馈动态设定
    chassis.chassis_power_limit.power_set = 50;

}

static void chassis_set_mode(chassis_t* chassis){

    if(chassis==NULL)
        return;

    if(switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_RELAX;
    }

    else if(switch_is_down(rc_ctrl.rc.s[RC_s_R])&&!switch_is_down(rc_ctrl.rc.s[RC_s_L]))
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_ONLY;
    }

    else if(switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_FOLLOW_GIMBAL;
    }

    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_SPIN;
    }

    //打符时最好保持底盘静止
    if(gimbal.mode==GIMBAL_BUFF||gimbal.mode==GIMBAL_SBUFF)//如果开启打符模式 则底盘刹车
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_BLOCK;
    }

    //UI更新---底盘模式
    ui_robot_status.chassis_mode=chassis->mode;
}

static void chassis_ctrl_info_get() {
    chassis_pc_ctrl();

    chassis.vx = -(float)(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX-chassis.vx_pc;
//    VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-0.300),(MAX_CHASSIS_VX_SPEED-0.300));

    chassis.vy = (float)(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]) * RC_TO_VY-chassis.vy_pc;
    chassis.vw = (float)(rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL]) * RC_TO_VW+chassis.vw_pc;
}

//将期望速度转为转子期望转速
static void chassis_wheel_cal(){
    fp32 max=0;
    fp32 wheel_rpm[4];
    fp32 vx, vy, vw;

    //底盘最大速度控制（功率控制）根据裁判信息选择
    chassis_vector_powerControl();

    vx=chassis.vx;
    vy=chassis.vy;
    vw=chassis.vw;

    //轮运动学解算
    wheel_rpm[0] = (-vy - vx - vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = (-vy + vx - vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (vy + vx - vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (vy - vx - vw * rotate_ratio_b) * wheel_rpm_ratio;
    // find max item
    for (uint8_t i = 0; i < 4; i++) {
        if (abs(wheel_rpm[i]) > max) {
            max = abs(wheel_rpm[i]);
        }
    }
    // equal proportion
    if (max > M3508_MAX_RPM) {
        float rate = M3508_MAX_RPM / max;
        for (uint8_t i = 0; i < 4; i++) wheel_rpm[i] *= rate;
    }

    chassis.motor_chassis[RF].rpm_set=wheel_rpm[0];
    chassis.motor_chassis[LF].rpm_set=wheel_rpm[1];
    chassis.motor_chassis[LB].rpm_set=wheel_rpm[2];
    chassis.motor_chassis[RB].rpm_set=wheel_rpm[3];

}

static void chassis_wheel_loop_cal() {

    ramp_calc(&chassis_3508_ramp[RF],chassis.motor_chassis[RF].rpm_set);
    chassis.motor_chassis[RF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                     chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[RF].rpm_set);

    ramp_calc(&chassis_3508_ramp[LF],chassis.motor_chassis[LF].rpm_set);

    chassis.motor_chassis[LF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LF].speed_p,
                                                     chassis.motor_chassis[LF].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[LF].rpm_set);

    ramp_calc(&chassis_3508_ramp[RB],chassis.motor_chassis[RB].rpm_set);

    chassis.motor_chassis[RB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RB].speed_p,
                                                     chassis.motor_chassis[RB].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[RB].rpm_set);

    ramp_calc(&chassis_3508_ramp[LB],chassis.motor_chassis[LB].rpm_set);
    chassis.motor_chassis[LB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LB].speed_p,
                                                     chassis.motor_chassis[LB].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[LB].rpm_set);

}
//计算函数f(iset)=K*Ωset+M的系数K，M
void calc_power_limit(pid_t *pid,int i)
{
    //计算积分累计
    pid->sum_err += (pid->err[NOW]+pid->err[LAST])/2;
    //根据pid计算函数推出的
    chassis.chassis_power_limit.K[i]=pid->p*CHASSIS_CURRENT_CONVERT;
    chassis.chassis_power_limit.M[i]=-(pid->p*chassis.motor_chassis[i].motor_measure->speed_rpm-pid->iout)*CHASSIS_CURRENT_CONVERT;
    pid->sum_err = fp32_constrain(pid->sum_err,-CHASSIS_3508_PID_MAX_IOUT,-CHASSIS_3508_PID_MAX_IOUT);
}
//功率控制
void chassis_power_limit() {
    //更新电容状态
    cap_info_update();
    for (int i = 0; i < 4; ++i) {
        //计算K,M
        calc_power_limit(&chassis.motor_chassis[i].speed_p,i);
    }
    float a=0,b=0,c=0;
    for (int i = 0; i < 4; ++i) {
        //a,b,c的计算过程，具体参看华中科技大学功率控制模型
        a += CHASSIS_POWER_R0 * pow(chassis.motor_chassis[i].rpm_set,2) * pow(chassis.chassis_power_limit.K[i],2);
        b += chassis.motor_chassis[i].rpm_set * (2 * CHASSIS_POWER_R0 * chassis.chassis_power_limit.K[i] * chassis.chassis_power_limit.M[i] + CHASSIS_POWER_K0 * chassis.chassis_power_limit.K[i] * chassis.motor_chassis[i].motor_measure->speed_rpm);
        c += CHASSIS_POWER_R0 * pow(chassis.chassis_power_limit.M[i],2) + CHASSIS_POWER_K0 * chassis.chassis_power_limit.M[i] * chassis.motor_chassis[i].motor_measure->speed_rpm;

    }
    c += CHASSIS_POWER_P0;

    //算出下一周期的功率
    chassis.chassis_power_limit.predict_send_power = fp32_constrain(a+b+c,-1000,1000);
    c-=chassis.chassis_power_limit.power_set;
    bool_t dangerous=false;//缓冲能量过低触发危险信号
    //缓冲能量小于10J，危险信号开启
    if(Referee.PowerHeatData.chassis_power_buffer<10){
        dangerous=true;
        //系数c越大，限制的功率值越低，比如最大功率80w，限制的功率降至60w，那底盘功率被限制后最大仅可达到60w上下，几乎不会超过80w
        c+=(10-chassis.chassis_power_limit.power_buff)*2;
    }
    //当预测功率大于最大功率时或者缓冲能量过低时，启动功率限制，求出刚好令预测功率<=最大功率的系数K_c
    if(chassis.chassis_power_limit.predict_send_power > chassis.chassis_power_limit.power_set||dangerous)
    {
        if(b*b < 4*c*a)
        {
            chassis.chassis_power_limit.k_c = fp32_constrain(-b/(2*a),0.0f,1.0f);
        }
        else
        {
            float sqrt_result;
            arm_sqrt_f32(b*b - 4*c*a ,&sqrt_result);
            chassis.chassis_power_limit.k_c = fp32_constrain((-b + sqrt_result)/2/a,0.0f,1.0f);
        }
        //当开电容并且缓冲能量充足时，函数直接返回，不执行后续的功率控制
        //电容返回的数据具体参考电容手册
        if(cap2.mode==1&&!dangerous){
            return;
        }
        for (int i = 0; i < 4; ++i) {
            //对rpm_set进行比例缩减，达到功率控制效果
            chassis.motor_chassis[i].rpm_set *= chassis.chassis_power_limit.k_c;
        }

    }

    /*
    //更新电容状态
    cap_info_update();

    if(cap2.charge_status!=1) {

        chassis.chassis_power_limit.total_current = 0;
        chassis.chassis_power_limit.total_current_limit = 0;
        fp32 power_buffer = chassis.chassis_power_limit.power_buff;
        fp32 limit_k;
        if (detect_list[DETECT_REFEREE].status != ONLINE) {
            chassis.chassis_power_limit.total_current_limit = CHASSIS_CURRENT_LIMIT_40W;
        } else {
            chassis.chassis_power_limit.power_buff = Referee.PowerHeatData.chassis_power_buffer > CHASSIS_POWER_BUFF ?
                                                     CHASSIS_POWER_BUFF : Referee.PowerHeatData.chassis_power_buffer;
//                chassis.chassis_power_limit.limit_k=(cap_info.cap_value-cap_info.min_voltage)/(cap_info.max_cap_voltage-cap_info.min_voltage);
            chassis.chassis_power_limit.limit_k= chassis.chassis_power_limit.power_buff/CHASSIS_POWER_BUFF;
//            if (Referee.PowerHeatData.chassis_power = Referee.GameRobotStat.max_chassis_power-20) {
            if(chassis.chassis_power_limit.limit_k <=0)
            {
                chassis.chassis_power_limit.limit_k =0.15f;
            }
            else if(Referee.PowerHeatData.chassis_power_buffer<(Referee.PowerHeatData.chassis_power_buffer)*0.3) {
                chassis.chassis_power_limit.limit_k =
                        chassis.chassis_power_limit.limit_k*chassis.chassis_power_limit.limit_k;
            }

            chassis.chassis_power_limit.total_current_limit =
                    chassis.chassis_power_limit.limit_k * CHASSIS_CURRENT_LIMIT_TOTAL;
        }
        for (uint8_t i = 0; i < 4; i++)
            chassis.chassis_power_limit.total_current += abs(chassis.motor_chassis[i].give_current);

        for (uint8_t i = 0; i < 4; i++)
            give_current_limit[i] = chassis.motor_chassis[i].give_current;

        if (chassis.chassis_power_limit.total_current > chassis.chassis_power_limit.total_current_limit) {
            for (uint8_t i = 0; i < 4; i++)
                give_current_limit[i] = (int16_t) (chassis.motor_chassis[i].give_current *
                                                   chassis.chassis_power_limit.total_current_limit
                                                   / chassis.chassis_power_limit.total_current);
        }
        //????????
        for (uint8_t i = 0; i < 4; i++)
            chassis.motor_chassis[i].give_current = give_current_limit[i];
    }
//    power_real=((ABS(chassis.motor_chassis[0].motor_measure->given_current)+
//                 ABS(chassis.motor_chassis[1].motor_measure->given_current)+
//                 ABS(chassis.motor_chassis[2].motor_measure->given_current)+
//                 ABS(chassis.motor_chassis[3].motor_measure->given_current))*20/16384)*cap_info.input_current;
     */


}

//把can接收时对真实电机的映射，在发送控制时映射回去为真实的电机，因为控制函数要按电机ID 1～4发送
void chassis_can_send_back_mapping(){

    int16_t *real_motor_give_current[4];

    real_motor_give_current[0] = &chassis.motor_chassis[LF].give_current;
    real_motor_give_current[1] = &chassis.motor_chassis[RF].give_current;
    real_motor_give_current[2] = &chassis.motor_chassis[RB].give_current;
    real_motor_give_current[3] = &chassis.motor_chassis[LB].give_current;

    CAN_cmd_motor(CAN_1,
                  CAN_MOTOR_0x200_ID,
                  *real_motor_give_current[0],
                  *real_motor_give_current[1],
                  *real_motor_give_current[2],
                  *real_motor_give_current[3]
                  );

//    CAN_cmd_motor(CAN_1,
//                  CAN_MOTOR_0x200_ID,
//                  0,
//                  0,
//                  0,
//                  0
//    );

}

void chassis_device_offline_handle() {
    if(detect_list[DETECT_REMOTE].status==OFFLINE) {
        chassis.mode = CHASSIS_RELAX;//防止出现底盘疯转
    }
}
static void chassis_relax_handle() {
    CAN_cmd_motor(CAN_1, CAN_MOTOR_0x200_ID, 0, 0, 0, 0);
}

void chassis_follow_gimbal_handle(){
    // 以上电时的yaw为中心，yaw不动使底盘去归中到对应的编码器中点，在运动时底盘运动方向为底盘正方向，
    // 当yaw转动后产生了编码器与编码器中值的偏移使底盘进行“跟随yaw”转动，
    // 然后将底盘正方向移至yaw所指向的方向，就做到了底盘跟云台然后yaw指哪走哪
    fp32 yaw_relative_radian=gimbal.yaw.relative_angle_get*ANGLE_TO_RAD;//相对角度的弧度
    fp32 sin_yaw,cos_yaw;

    sin_yaw=(fp32)sin(yaw_relative_radian);
    cos_yaw=(fp32) cos(yaw_relative_radian);

    fp32 vx_temp=chassis.vx;
    fp32 vy_temp=chassis.vy;

    //速度矢量分解
    chassis.vx=cos_yaw*vx_temp-sin_yaw*vy_temp;
    chassis.vy=-(sin_yaw*vx_temp+cos_yaw*vy_temp);
    chassis.vw = pid_calc(&chassis.chassis_vw_pid, -gimbal.yaw.relative_angle_get, 0);
    VAL_LIMIT(chassis.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);
}

void chassis_spin_handle()
{
    //小陀螺实现分为三步：
    //1、获取底盘与云台的相对角度θ。底盘绝对角度由YAW轴电机提供，云台绝对角度由云台上的角度传感器提供。
    //2、根据θ，把整车运动速度（大小和方向）分解到底盘坐标
    //3、根据底盘坐标速度进行麦轮速度分解，整车效果则表现为按云台坐标运动
    fp32 yaw_relative_radian=gimbal.yaw.relative_angle_get*ANGLE_TO_RAD;//相对角度的弧度
    fp32 sin_yaw,cos_yaw;

    sin_yaw=(fp32)sin(yaw_relative_radian);
    cos_yaw=(fp32) cos(yaw_relative_radian);

    fp32 vx_temp=chassis.vx;
    fp32 vy_temp=chassis.vy;

    //速度矢量分解
    chassis.vx=vx_temp*cos_yaw - vy_temp*sin_yaw;
    chassis.vy=-(vx_temp*sin_yaw + vy_temp*cos_yaw);//y is left

    switch(chassis.spin_mode){
        case NORMAL_SPIN:
            chassis.vw = CHASSIS_SWING_SPEED;
            break;

        case HIDDEN_ARMOR_SPEED_CHANGE:
            if((gimbal.yaw.relative_angle_get>=0 && gimbal.yaw.relative_angle_get<=60) ||
               (gimbal.yaw.relative_angle_get>=90 && gimbal.yaw.relative_angle_get<=150)||
               (gimbal.yaw.relative_angle_get>=-90 && gimbal.yaw.relative_angle_get<=-30)||
               (gimbal.yaw.relative_angle_get>=-180 && gimbal.yaw.relative_angle_get<=-120)
               ){
                chassis.vw=CHASSIS_ARMOR_NOT_FACING_ENEMY_SPIN_SPEED; //装甲板没有面向敌人的速度
            }
            else{
                chassis.vw=CHASSIS_ARMOR_FACING_ENEMY_SPIN_SPEED;  //装甲板面向敌人的速度
            }
    }
}

static void chassis_pc_ctrl(){

    float speed_change=chassis_speed_change();//获取加速度

    //键盘控制下的底盘以斜坡式变化
    if(KeyBoard.W.status==KEY_PRESS)//键盘前进键按下
    {
        chassis.vx_pc+=speed_change;//速度增量
    }
    else if(KeyBoard.S.status==KEY_PRESS)
    {
        chassis.vx_pc-=speed_change;
    }
    else{
        chassis.vx_pc=0;
    }

    if(KeyBoard.A.status==KEY_PRESS)//键盘前进键按下
    {
        chassis.vy_pc+=speed_change;
    }
    else if(KeyBoard.D.status==KEY_PRESS)
    {
        chassis.vy_pc-=speed_change;
    }
    else{
        chassis.vy_pc=0;
    }
//    if(chassis.mode==CHASSIS_SPIN)//灯
//    {
//        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_RESET);
//    } else{
//        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_SET);
//    }

    if(KeyBoard.E.click_flag==1)//
    {
        chassis.mode=CHASSIS_SPIN;
    }

    if(chassis.mode==CHASSIS_SPIN)//灯
    {
//        led.mode=SPIN;//无led
    }

//    if(chassis.mode==CHASSIS_CLIMBING)//灯
//    {
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_RESET);
//    } else{
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
//    }
}

static float chassis_speed_change() {
    float speed_change = 0;
    if (cap2.mode==1) {//开启电容 增加加速度
        speed_change = 0.0025;//最大加速度
    } else {
        switch (Referee.GameRobotStat.max_chassis_power) {//最大限制功率
            case 40: {
                speed_change=0.0015;
            }
                break;
            case 45: {
                speed_change=0.0018;
            }
                break;
            case 50: {
                speed_change=0.0018;
            }
                break;
            case 55: {
                speed_change=0.0018;
            }
                break;
            case 60: {
                speed_change=0.0018;
            }
                break;
            case 70: {
                speed_change=0.0020;
            }
                break;
            case 80: {
                speed_change=0.0020;
            }
                break;
            case 100: {
                speed_change=0.0023;
            }
                break;
            case 120: {
                speed_change=0.0025;
            }
                break;
            default:{
                speed_change=0.001;
            }break;
        }
    }
    return speed_change;
}

static void chassis_vector_powerControl(){
    cap_info_update();//根据电容开启和否限制底盘移动速度
    if(cap2.mode==1 && cap2.cap_voltage>14)//电容电压大于13 不限制底盘功率 通过增大pc_ctrl下的speed_change 提高底盘加速度;
    {
        VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED+2),(MAX_CHASSIS_VX_SPEED+2));
        VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED+2),(MAX_CHASSIS_VY_SPEED+2));
        VAL_LIMIT(chassis.vw,-6.5,6.5);
    }
    else {
        switch (Referee.GameRobotStat.max_chassis_power) {
            case 40:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.6),(MAX_CHASSIS_VX_SPEED-1.6));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.6),(MAX_CHASSIS_VY_SPEED-1.6));
                VAL_LIMIT(chassis.vw,-2.20,2.20);
            }break;
            case 45:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.500),(MAX_CHASSIS_VX_SPEED-1.500));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.500),(MAX_CHASSIS_VY_SPEED-1.500));
                VAL_LIMIT(chassis.vw,-2.25,2.25);
            }break;
            case 50:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.500),(MAX_CHASSIS_VX_SPEED-1.500));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.500),(MAX_CHASSIS_VY_SPEED-1.500));
                VAL_LIMIT(chassis.vw,-2.35,2.35);
            }break;
            case 55:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.400),(MAX_CHASSIS_VX_SPEED-1.400));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.400),(MAX_CHASSIS_VY_SPEED-1.400));
                VAL_LIMIT(chassis.vw,-2.50,2.50);
            }break;
            case 60:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.300),(MAX_CHASSIS_VX_SPEED-1.300));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.300),(MAX_CHASSIS_VY_SPEED-1.300));
                VAL_LIMIT(chassis.vw,-2.60,2.60);
            }break;
            case 70:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.100),(MAX_CHASSIS_VX_SPEED-1.100));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.100),(MAX_CHASSIS_VY_SPEED-1.100));
                VAL_LIMIT(chassis.vw,-2.75,2.75);
            }break;
            case 80:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.00),(MAX_CHASSIS_VX_SPEED-1.00));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.00),(MAX_CHASSIS_VY_SPEED-1.00));
                VAL_LIMIT(chassis.vw,-3.10,3.10);
            }break;
            case 100:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED+0.100),(MAX_CHASSIS_VX_SPEED+0.100));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED+0.100),(MAX_CHASSIS_VY_SPEED+0.100));
                VAL_LIMIT(chassis.vw,-3.30,3.30);
            }break;
            case 120:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED+0.200),(MAX_CHASSIS_VX_SPEED+0.200));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED+0.200),(MAX_CHASSIS_VY_SPEED+0.200));
                VAL_LIMIT(chassis.vw,-3.35,3.35);
            }break;
            default:{
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.500),(MAX_CHASSIS_VX_SPEED-1.500));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.500),(MAX_CHASSIS_VY_SPEED-1.500));
                VAL_LIMIT(chassis.vw,-2.20,2.20);
            }
        }
    }

    //操作手主动慢速
    if(KeyBoard.CTRL.status==KEY_PRESS)
    {
        VAL_LIMIT(chassis.vx,-MAX_CHASSIS_VX_SPEED*0.2,MAX_CHASSIS_VX_SPEED*0.2);
        VAL_LIMIT(chassis.vy,-MAX_CHASSIS_VY_SPEED*0.2,MAX_CHASSIS_VY_SPEED*0.2);
        VAL_LIMIT(chassis.vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);
    }
//    if(KeyBoard.SHIFT.status==KEY_PRESS){
//        VAL_LIMIT(chassis.vx,-MAX_CHASSIS_VX_SPEED*1.5,MAX_CHASSIS_VX_SPEED*1.5);
//        VAL_LIMIT(chassis.vy,-MAX_CHASSIS_VY_SPEED*1.5,MAX_CHASSIS_VY_SPEED*1.5);
//        VAL_LIMIT(chassis.vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);
//    }
}