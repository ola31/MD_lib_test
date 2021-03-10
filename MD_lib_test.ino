
/*
 * CAN_driver.h  
 * md_can_motor_driver.h 
 * 테스트 코드
 */

/*
 * 모터가 90,180,360도 회전시 엔코더 값 확인하기
 * 모터신호 반전 시 반대방향으로 돌아가는지 확인하기
 * 모터신호 반전 시 엔코더신호 부호도 바뀌는지 확인하기
 */

#include "CAN_driver.h"
#include "md_can_motor_driver.h"
#include "md_can_sensor.h"

#define PID_INV_SIGN_CMD 16
#define PID_INV_SIGN_CMD2 18

#define INV_ON 1
#define INV_OFF 0 

//객체선언
DrokMdMotorDriver motor_driver;
DrokMdSensor sensor;


uint8_t* read_test=NULL;

bool enc_test=false;
int32_t l_enc_test=0,r_enc_test=0;

uint32_t begin_time=0;
uint32_t end_time=0;
uint32_t diff_time=0;

bool once = true;
int count = 0;
int flag = 0;
uint8_t inv_sign_arr[8] = {PID_INV_SIGN_CMD,INV_ON,0,0,0,0,0,0};
uint8_t inv_sign_arr2[8] = {PID_INV_SIGN_CMD2,INV_ON,0,0,0,0,0,0};
//uint8_t aa[8] = {156,0,64,0,0,0,0,0};
uint8_t aa[8] = {244,0,128,7,0,0,0,0};

uint8_t posi_reset[8] = {13,0,0,0,0,0,0,0};

sensor_msgs::Imu a;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  CAN_initialize();

}

void loop() {
/*
  if(once){
  for(int i=167;i<170;i++)
  {
    read_test = CAN_read(i);
    int16_t value = LHByte2Int16(read_test[1],read_test[2]);
    Serial.print("PID "); Serial.print(i);Serial.print(" : ");Serial.println(value);
  }
  Serial.println("===============================================");
  Serial.println("***********************************************");
  Serial.println("===============================================");
  //once=false;
  delay(1);
  }*/
  // put your main code here, to run repeatedly:

  //CAN_write(inv_sign_arr);  //모터1의 신호반전
  //CAN_write(inv_sign_arr2); //모터2의 신호반전   ->엔코더 신호도 반전되는지 확인
  //if(flag==0){CAN_write(aa);flag=1;}

  bool WriteVel_result = motor_driver.writeVelocity(30,30); //입력축900rpm, 출력축30rpm(2초에 한 바퀴)
/*
  // CAN_read 함수 테스트
  begin_time = micros();
  read_test = CAN_read(1);             
  end_time = micros();
  
  diff_time = end_time-begin_time;//
  
  //Serial.print("diff_time(read_test) : "); Serial.println(diff_time);
  //Serial.print("PID 1 : "); Serial.println(read_test[1]);
*/
/*
  // readEncoder 함수 테스트
  begin_time = micros();
  enc_test = motor_driver.readEncoder(l_enc_test,r_enc_test);
  end_time = micros();
  
  diff_time = end_time-begin_time;
  Serial.print("diff_time(enc) : "); Serial.println(diff_time);
  Serial.print("left_tick : "); Serial.print(l_enc_test); Serial.print("right_tick : "); Serial.println(r_enc_test);
  */
/*
  // md_can_sensor 테스트
  a=sensor.getIMU();
  a.angular_velocity.z;
  Serial.print("ang_vel : "); Serial.println(a.angular_velocity.z);
  
  //Serial.println("===============================================");
*/

  //PID 156번 확인(엔코더세팅)
  //if(once == true){
  //Serial.println("ddd");
  read_test = CAN_read(196);   //196 156
  //Serial.println("ddd");
 // int16_t value = LHByte2Int16(read_test[1],read_test[2]);
  //int16_t value = read_test[1];
  int32_t value = Byte2Int32(read_test[4],read_test[5],read_test[6],read_test[7]);
  //int32_t value = Byte2Int32(read_test[1],read_test[2],read_test[3],read_test[4]);
  delayMicroseconds(1000);
  Serial.println("PID196 : ");  
  Serial.println(read_test[0]);
  Serial.println(read_test[1]);
  Serial.println(read_test[2]);
  Serial.println(read_test[3]);
  Serial.println(read_test[4]);
  Serial.println(read_test[5]);
  Serial.println(read_test[6]);
  Serial.println(read_test[7]);Serial.println(value);
  //if(count>10){once = false;}
  //count++;
  //}
  
  
  
  delayMicroseconds(500000);
  //CAN_write(posi_reset);
  //delay 5 msec
//  bl9l 400W 16384, 740 25
}
