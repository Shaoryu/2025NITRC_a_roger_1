/*確認事項
　UnbufferedSerialはコメントアウトされているか？シリアル読み込みはどうか？(rbms.cppも)
　can_motorはPB_5,6か？
　max_degは-か+か
　data以下のnumber_accは正しいか？
　ソフトリミットの範囲は正しいか？
  吸引ポンプのは正しいか？
*/
#include "mbed.h"
#include "rbms.h"
#include <cstdio>
// UnbufferedSerial pc(USBTX, USBRX, 921600);
enum number_acc{
    ours_front,
    ours_back,
    common_front,
    common_back,
    transform_front,
    transform_back
};

CAN can_motor(PB_5, PB_6);  // CAN RX, TX
CAN can_signal(PA_11,PA_12,1000000);

CANMessage msg_controller,msg_limit,msg_can1;

static constexpr int motor_num = 4;
static constexpr int max_speed = 300;
static constexpr int max_deg = -4160;
static constexpr int setting_speed = -20;//0点だしの時の速度(1次？)
static constexpr short receivecan_id = 0x05;//コントローラの信号ID正直命名ミスった枠その１
static constexpr short limitcan_id = 0x301;//pc用
bool limit_status[6]={1};//リミット
bool is_pump_start=false;
char pre_pump=0;

int motor_output[motor_num] = {0};
int setpoint[motor_num] = {0};              // 指令値（角度 or 速度）
short offset_deg;


MotorType mtype[motor_num] = {
    M2006,
    M2006,
    M2006,
    M3508
};
ControlMode control_mode[motor_num] = {};  // 制御モード配列（初期値: すべて ANGLE）

EventQueue queue_motor(32 * EVENTS_EVENT_SIZE);
EventQueue queue_control(32 * EVENTS_EVENT_SIZE);
rbms motor_controller(can_motor, mtype, motor_num);
// UnbufferedSerial pc(USBTX, USBRX, 921600);
Thread can_thread_motor,can_thread_control;
Thread control_thread;
Thread send_thread;
Timeout tout;

void limit_func(){
    int i=0,j=0;
    if(msg_limit.id==0x301)i=0;
    else if (msg_limit.id==0x302)i=1;
    for(;i<6;i+=2)limit_status[i]=(bool)msg_limit.data[j++];
}


//DigitalOut ledmain(LED1);
DigitalOut leds[4]{
    DigitalOut(PC_0),
    DigitalOut(PC_1),
    DigitalOut(PC_2),
    DigitalOut(PC_3),
};

auto& now_ang = motor_controller.getSumDeg();

void can_receive_loop_motor() {
    motor_controller.can_read();
    // can_signal.read(msg_controller);
}

void can_receive_loop_control() {
        //motor_controller.can_read();
        if(can_signal.read(msg_can1))
            leds[1] =! leds[1];
        if(msg_can1.id==receivecan_id){
            leds[2] =! leds[2];
            msg_controller = msg_can1;
        }
        else if(msg_can1.id==0x301 ||msg_can1.id==0x302){
            msg_limit = msg_can1;
           // limit_func();
        }
}
//
//
void control_loop() {
    motor_controller.unified_control(control_mode, setpoint, motor_output,max_speed);
}

void send_loop() {
    while (true) {
        motor_controller.rbms_send(motor_output);
        ThisThread::sleep_for(1ms);
    }
}


void onReceive_motor() {
    queue_motor.call(can_receive_loop_motor);
}

void onReceive_control() {
    queue_control.call(can_receive_loop_control);
}


//void limitswitch();
PwmOut pump(PB_9);
void switch_pump(){
    is_pump_start=false;
    switch (msg_controller.data[5]) {
        case 0:pump.pulsewidth_us(1000);break;
        case 1: pump.pulsewidth_us(1500);break;
        case 2: pump.pulsewidth_us(1600);break;
        case 3: pump.pulsewidth_us(1700);break;
        case 0x11: pump.pulsewidth_us(1100);break;
        case 0x12: pump.pulsewidth_us(1200);break;
        case 0x13: pump.pulsewidth_us(1300);break;
        case 0x14: pump.pulsewidth_us(1400);break;
        case 0x15: pump.pulsewidth_us(1500);break;
        case 0x16: pump.pulsewidth_us(1600);break;
        case 0x17: pump.pulsewidth_us(1700);break;
        case 0x18: pump.pulsewidth_us(1800);break;
        case 0x19: pump.pulsewidth_us(1900);break;
        case 0x1a: pump.pulsewidth_us(2000);break;
        default:pump.pulsewidth_us(1000);
    }
}

void switch_bump(){
    pump.pulsewidth_us(1000);
    ThisThread::sleep_for(200ms);
    switch_pump();
}


int main() {
    //can_motor.reset();
    leds[0] = true;
    pump.pulsewidth_us(1000);
    can_motor.frequency(1000000);
    // char key;
    // char feight=0;//高さの誤字
    bool pre_flag_limit=false;
    bool flag_limit1=false;
    bool flag_limit2=false;
    //printf("Unified control start...\r\n");
    can_thread_motor.start(callback(&queue_motor, &EventQueue::dispatch_forever));
    can_motor.attach(onReceive_motor, CAN::RxIrq);
    can_thread_control.start(callback(&queue_control, &EventQueue::dispatch_forever));
    can_signal.attach(onReceive_control, CAN::RxIrq);
    control_thread.start(control_loop);
    send_thread.start(send_loop);
    ThisThread::sleep_for(2ms);
    offset_deg = now_ang[0] /36;
    control_mode[0]=SPEED;
    setpoint[0]=0;
    msg_controller.id=limitcan_id;
    msg_controller.len=8;
    for(int i=0;i<8;i++)msg_controller.data[i]=0;
    while (true) {
        // can_signal.read(msg_con);
        limit_func();
        if(msg_controller.id==receivecan_id){
            if(msg_controller.data[7]==0xff)NVIC_SystemReset();
            else if(msg_controller.data[7]==0x20)flag_limit1 = true;
            else {
                 if(!flag_limit1) motor_controller.control_tracon(control_mode,setpoint,(int)((float)msg_controller.data[common_front]*max_deg/0xff+offset_deg) , 1);
                 leds[2]=!leds[2];
            }
            if(!is_pump_start){
                switch_pump();
                if(pre_pump==0&&msg_controller.data[5]!=0){
                    //switch_pump();
                    tout.attach(switch_bump,300ms);
                    is_pump_start=true;
                }
                // else{
                //     switch_pump();
                // }
            }
            pre_pump=msg_controller.data[5];
            //if(pre_pump==0)is_pump_start=true;
        }
        // if(flag_limit1){
        //     if(limit_status[common_front]){
        //         flag_limit2 = true;
        //         control_mode[0] = SPEED;
        //         setpoint[0]=setting_speed;
        //     }else{
        //         if(flag_limit2){
        //             offset_deg = now_ang[0]/36;
        //             setpoint[0] = offset_deg;
        //             control_mode[0] = ANGLE;
        //             flag_limit1 = false;
        //             flag_limit2 = false;
        //         }else{
        //             control_mode[0] = SPEED;
        //             setpoint[0]=-setting_speed;
        //         }
        //     }
        // }        
        // else if(limit_status[common_front]&&pre_flag_limit!=limit_status[common_front]){
        //     setpoint[0] = now_ang[0]/36;
        //     control_mode[0] = ANGLE;
        //     pre_flag_limit = limit_status[common_front];
        // }
        leds[3] = limit_status[common_front];
        //ledmain = flag_limit2;
        // #ifdef deb//ours
        //     if (pc.readable()) {
        //         pc.read(&key, 1);
        //         if(key>='0'&&key<='9'){
        //             msg_con.id=receivecan_id;msg_con.data[common_front]=(key-'0')*28;msg_con.data[7]=0x0;
        //             // motor_controller.control_tracon(control_mode ,setpoint,( key-'0')*max_deg/10+offset_deg , 1);
        //             //setpoint[0] =( key-'0')*max_deg/10+offset_deg;
        //             //control_mode[0] = abs(now_ang[0]/36-setpoint[0])>1800?TRACON:ANGLE;
        //         }else{
        //             switch (key) {
        //                 case 'l': msg_con.id=receivecan_id;msg_con.data[common_front]=0xfd;msg_con.data[7]=0x0;break;
        //                 case 'k': msg_con.id=receivecan_id;msg_con.data[common_front]=0x0;msg_con.data[7]=0x0;break;
        //                 case '-': msg_con.id=limitcan_id;msg_con.data[1]=0xff;msg_con.data[7]=0x0;break;
        //                 case '=': msg_con.id=limitcan_id;msg_con.data[1]=0x0;msg_con.data[7]=0x0;break;
        //                 case 'j': control_mode[0] = TRACON; setpoint[0] = -max_deg+offset_deg; break;
        //                 case 'n':control_mode[0] = ANGLE; setpoint[0] -= 90; break;
        //                 case 'm':control_mode[0] = ANGLE; setpoint[0] += 90; break;
        //                 case 'h': msg_con.id=receivecan_id;msg_con.data[common_front]=0x0;msg_con.data[7]=0x20;break;
        //                 case 'd': flag_limit1 = false; break;
        //                 case 'g': msg_con.id=receivecan_id;msg_con.data[common_front]=0x0;msg_con.data[7]=0x20;break;
        //                 case 'a': for(int i = 0;i < 4 ;i++) {control_mode[i] = TRACON; setpoint[i] = 0;}break;
        //                 case 'r': NVIC_SystemReset();
        //                 default:  for(int i = 0;i < 4 ;i++) {control_mode[i] = SPEED;  setpoint[i] = 0;}break;
                        
        //             }
        //         }
        //         //printf("%c\r\n",key);
        //         printf("%d\r\n",now_ang[0]/36-offset_deg);
        //     }
        // #endif
        if(now_ang[0]/36 < -4230){
            flag_limit1=false;
            flag_limit2 = false;
            control_mode[0]=ANGLE;
            setpoint[0] = max_deg;
        }else if(now_ang[0]/36 > 360){
            flag_limit1=false;
            flag_limit2 = false;
            control_mode[0] = ANGLE;
            setpoint[0] = 100;
        }
        //printf("data[3]:%d\r\n",msg_controller.data[3]);
        // ThisThread::sleep_for(2ms);
    }
}//