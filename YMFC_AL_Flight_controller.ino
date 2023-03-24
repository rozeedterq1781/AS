

#include <Wire.h>    //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>  //Include the EEPROM.h library so we can store information onto the EEPROM

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;   // รับการตั้งค่าสำหรับตัวควบคุม P ม้วน
float pid_i_gain_roll = 0.04;  // รับการตั้งค่าสำหรับตัวควบคุม I-roll
float pid_d_gain_roll = 18.0;  // รับการตั้งค่าสำหรับโรล D-controller
int pid_max_roll = 400;        // เอาต์พุตสูงสุดของตัวควบคุม PID (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  // รับการตั้งค่าสำหรับ pitch P-controller
float pid_i_gain_pitch = pid_i_gain_roll;  // รับการตั้งค่าสำหรับ pitch I-controller
float pid_d_gain_pitch = pid_d_gain_roll;  // รับการตั้งค่าสำหรับ pitch D-controller
int pid_max_pitch = pid_max_roll;          // เอาต์พุตสูงสุดของตัวควบคุม PID (+/-)

float pid_p_gain_yaw = 4.0;   // รับการตั้งค่าสำหรับ pitch P-controller //4.0
float pid_i_gain_yaw = 0.02;  // รับการตั้งค่าสำหรับ pitch I-controller //0.02
float pid_d_gain_yaw = 0.0;   // รับการตั้งค่าสำหรับ pitch D-controller
int pid_max_yaw = 400;        // เอาต์พุตสูงสุดของตัวควบคุม PID (+/-)

boolean auto_level = true;  // ระดับอัตโนมัติเปิด (จริง) หรือปิด (เท็จ)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
/////////////////////////////////////////////////// [เก็บค่า 0 - 255 Last_Channel] //////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
/////////////////////////////////////////////////// [เก็บค่าข้อมูล 0 - 255 eeprom] //////////////////////////////////////////////////////
byte eeprom_data[36];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte highByte, lowByte;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int esc_1, esc_2, esc_3, esc_4;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int throttle, battery_voltage;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int cal_int, start, gyro_address;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int receiver_input[5];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int temperature;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int acc_axis[4], gyro_axis[4];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float roll_level_adjust, pitch_level_adjust;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long acc_x, acc_y, acc_z, acc_total_vector;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long loop_timer;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double gyro_pitch, gyro_roll, gyro_yaw;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double gyro_axis_cal[4];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_error_temp;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean gyro_angles_set;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(57600);
  for (start = 0; start <= 35; start++) eeprom_data[start] = EEPROM.read(start); // คัดลอกข้อมูล EEPROM เพื่อเข้าถึงข้อมูลอย่างรวดเร็ว
  start = 0;                       // ตั้งค่าเริ่มต้นกลับเป็นศูนย์
  gyro_address = eeprom_data[32];  //เก็บ gyro address ไว้ในตัวแปร

  Wire.begin();  //เริ่ม I2C เป็นมาสเตอร์

  TWBR = 12;  //ตั้งค่าความเร็วสัญญาณนาฬิกา I2C เป็น 400kHz

  //Arduino (Atmega) ปักหมุดค่าเริ่มต้นเป็นอินพุต ดังนั้นจึงไม่จำเป็นต้องประกาศอย่างชัดเจนว่าเป็นอินพุต
  DDRD |= B11110000;  //กำหนดค่าสัญญาณดิจิตอล 4, 5, 6 และ 7 เป็นเอาต์พุต
  DDRB |= B00110000;  //กำหนดค่าคนจนดิจิตอล 12 และ 13 เป็นเอาต์พุต

  //ใช้ไฟ LED บน Arduino เพื่อบ่งชี้การเริ่มต้น
  digitalWrite(12, HIGH);  //เปิดไฟเตือน

  //ตรวจสอบลายเซ็น EEPROM เพื่อให้แน่ใจว่าได้ดำเนินการโปรแกรมติดตั้งแล้ว
  while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B') delay(10);

  // เครื่องควบคุมการบินต้องการ MPU-6050 ที่มีไจโรและมาตรความเร่ง
  //หากการตั้งค่าเสร็จสิ้นโดยไม่มี MPU-6050 ให้หยุดโปรแกรมควบคุมการบิน
  if (eeprom_data[31] == 2 || eeprom_data[31] == 3) delay(10);

  set_gyro_registers();  // ตั้งค่าการลงทะเบียนไจโรเฉพาะ

  for (cal_int = 0; cal_int < 1250; cal_int++) {  //รอ 5 วินาทีก่อนดำเนินการต่อ
    PORTD |= B11110000;                           // ตั้งค่าดิจิตอลที่ไม่ดี 4, 5, 6 และ 7 สูง
    delayMicroseconds(1000);                      //รอ 1000us.
    PORTD &= B00001111;                           // ตั้งค่าดิจิตอลที่ไม่ดี 4, 5, 6 และ 7 ต่ำ
    delayMicroseconds(3000);                      //รอ 3000us.
  }

  // ลองใช้ตัวอย่างข้อมูลไจโรหลายตัวอย่างเพื่อให้เราสามารถกำหนดไจโรออฟเซ็ตเฉลี่ย (การสอบเทียบ)
  for (cal_int = 0; cal_int < 2000; cal_int++) {                // อ่านค่า 2,000 ค่าสำหรับการสอบเทียบ
    if (cal_int % 15 == 0) digitalWrite(12, !digitalRead(12));  // เปลี่ยนสถานะไฟ LED เพื่อระบุการสอบเทียบ
    gyro_signalen();                                            // อ่านเอาต์พุตไจโร
    gyro_axis_cal[1] += gyro_axis[1];                           // ค่าม้วนโฆษณาเป็น gyro_roll_cal
    gyro_axis_cal[2] += gyro_axis[2];                           // เพิ่มค่า pitch เป็น gyro_pitch_cal
    gyro_axis_cal[3] += gyro_axis[3];                           // เพิ่มค่าการหันเหเป็น gyro_yaw_cal

    // เราไม่ต้องการให้ esc ส่งเสียงบี๊บอย่างน่ารำคาญ ลองให้พัลส์ 1,000us แก่พวกเขาในขณะที่ปรับเทียบไจโร
    PORTD |= B11110000;       // ตั้งค่าดิจิตอลที่ไม่ดี 4, 5, 6 และ 7 สูง
    delayMicroseconds(1000);  //รอ 1000us.
    PORTD &= B00001111;       // ตั้งค่าดิจิตอลที่ไม่ดี 4, 5, 6 และ 7 ต่ำ
    delay(3);                 // รอ 3 มิลลิวินาทีก่อนที่จะวนรอบถัดไป
  }
  // ตอนนี้เรามีหน่วยวัด 2,000 หน่วย เราต้องหารด้วย 2,000 เพื่อให้ได้ไจโรออฟเซ็ตเฉลี่ย
  gyro_axis_cal[1] /= 2000;  // หารผลรวมม้วนด้วย 2,000
  gyro_axis_cal[2] /= 2000;  // แบ่งระดับเสียงทั้งหมดด้วย 2,000
  gyro_axis_cal[3] /= 2000;  // หารผลรวมของการหันเหด้วย 2,000

  PCICR |= (1 << PCIE0);    //ตั้งค่า PCIE0 เพื่อเปิดใช้งานการสแกน PCMSK0
  PCMSK0 |= (1 << PCINT0);  // ตั้งค่า PCINT0 (อินพุตดิจิทัล 8) เพื่อทริกเกอร์การขัดจังหวะเมื่อเปลี่ยนสถานะ
  PCMSK0 |= (1 << PCINT1);  // ตั้งค่า PCINT1 (อินพุตดิจิทัล 9) เพื่อกระตุ้นการขัดจังหวะเมื่อเปลี่ยนสถานะ
  PCMSK0 |= (1 << PCINT2);  // ตั้งค่า PCINT2 (อินพุตดิจิทัล 10) เพื่อทริกเกอร์การขัดจังหวะเมื่อเปลี่ยนสถานะ
  PCMSK0 |= (1 << PCINT3);  // ตั้งค่า PCINT3 (อินพุตดิจิทัล 11) เพื่อทริกเกอร์การขัดจังหวะเมื่อเปลี่ยนสถานะ

  // รอจนกว่าเครื่องรับจะทำงานและคันเร่งถูกตั้งค่าไปที่ตำแหน่งล่าง
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    receiver_input_channel_3 = convert_receiver_channel(3);  //แปลงสัญญาณรับสัญญาณจริงสำหรับคันเร่งเป็นมาตรฐาน 1,000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);  //แปลงสัญญาณรับสัญญาณจริงสำหรับการหันเหเป็นมาตรฐาน 1,000 - 2000us
    start++;                                                 //ในขณะที่รอการเพิ่มเริ่มทุกวง

    //เราไม่ต้องการให้ esc ส่งเสียงบี๊บอย่างน่ารำคาญ ลองให้พัลส์ 1,000us ขณะที่รออินพุตตัวรับ
    PORTD |= B11110000;                    //ตั้งค่าดิจิตอลที่ไม่ดี 4, 5, 6 และ 7 สูง
    delayMicroseconds(1000);               //รอ 1000us.
    PORTD &= B00001111;                    //ตั้งค่าดิจิตอลที่ไม่ดี 4, 5, 6 และ 7 ต่ำ
    delay(3);                              //รอ 3 มิลลิวินาทีก่อนที่จะวนรอบถัดไป
    if (start == 125) {                    //ทุก ๆ 125 ลูป (500ms)
      digitalWrite(12, !digitalRead(12));  //เปลี่ยนสถานะนำ
      start = 0;                           //เริ่มต้นใหม่อีกครั้งที่ 0
    }
  }
  start = 0;  //ตั้งค่าเริ่มต้นกลับเป็น 0

  // โหลดแรงดันแบตเตอรี่ไปที่ตัวแปร battery_voltage
  //65 คือการชดเชยแรงดันไฟฟ้าสำหรับไดโอด
  // 12.6V เท่ากับ ~5V @ อะนาล็อก 0
  // 12.6V เท่ากับ 1023 analogRead(0)
  //1260/1023 = 1.2317.
  // ตัวแปร battery_voltage เก็บ 1,050 ถ้าแรงดันแบตเตอรี่คือ 10.5V
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();  // ตั้งเวลาสำหรับลูปถัดไป

  //เมื่อทุกอย่างเรียบร้อย ให้ปิดไฟ LED
  digitalWrite(12, LOW);  // ปิดไฟเตือน
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ลูปโปรแกรมหลัก
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  //65.5 = 1 องศา/วินาที (ตรวจสอบข้อมูลเพิ่มเติมจาก MPU-6050)
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);     // อินพุต Gyro pid คือ deg/sec
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);  // อินพุต Gyro pid คือ deg/sec
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);        // อินพุต Gyro pid คือ deg/sec


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // นี่คือรหัส IMU ที่เพิ่มเข้ามาจากวิดีโอ:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // การคำนวณมุมไจโร
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;  // คำนวณมุมพิทช์เดินทางและเพิ่มไปยังตัวแปร angle_pitch
  angle_roll += gyro_roll * 0.0000611;    // คำนวณมุมม้วนเดินทางและเพิ่มไปยังตัวแปร angle_roll

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) ฟังก์ชัน Arduino sin มีหน่วยเป็นเรเดียน
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);  // ถ้า IMU หาวแล้ว ให้โอนมุมม้วนไปที่ pitch angel
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);  // ถ้า IMU หาวแล้ว ให้โอนมุมสนามไปที่โรลแองเจิล

  // การคำนวณมุมมาตรความเร่ง
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));  // คำนวณเวกเตอร์มาตรวัดความเร่งทั้งหมด

  if (abs(acc_y) < acc_total_vector) {                                 // ป้องกันไม่ให้ฟังก์ชัน asin สร้าง NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;  // คำนวณมุมสนาม
  }
  if (abs(acc_x) < acc_total_vector) {                                 // ป้องกันไม่ให้ฟังก์ชัน asin สร้าง NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;  // คำนวณมุมม้วน
  }

  // วางระดับจิตวิญญาณ MPU-6050 และบันทึกค่าในสองบรรทัดต่อไปนี้สำหรับการสอบเทียบ
  angle_pitch_acc -= 0.0;  // ค่าการปรับเทียบมาตรวัดความเร่งสำหรับระยะพิทช์
  angle_roll_acc -= 0.0;   // ค่าการปรับเทียบมาตรวัดความเร่งสำหรับม้วน

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;  // แก้ไขการเลื่อนของมุมพิทช์ของไจโรด้วยมุมพิทช์ของมาตรวัดความเร่ง
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     // แก้ไขการเลื่อนของมุมม้วนไจโรด้วยมุมม้วนตัววัดความเร่ง

  pitch_level_adjust = angle_pitch * 15;  // คำนวณการแก้ไขมุมพิทช์
  roll_level_adjust = angle_roll * 15;    // คำนวณการแก้ไขมุมม้วน

  if (!auto_level) {         //หาก Quadcopter ไม่ได้อยู่ในโหมดปรับระดับอัตโนมัติ
    pitch_level_adjust = 0;  //ตั้งค่าการแก้ไขมุมพิทช์เป็นศูนย์
    roll_level_adjust = 0;   //ตั้งค่าคอร์เรชั่นของมุมม้วนเป็นศูนย์
  }


  //สำหรับการสตาร์ทมอเตอร์: คันเร่งต่ำและหันซ้าย (ขั้นตอนที่ 1)
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050) start = 1;
  //เมื่อไม้หันกลับมาที่ตำแหน่งกึ่งกลาง ให้สตาร์ทมอเตอร์ (ขั้นตอนที่ 2)
  if (start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) {
    start = 2;

    angle_pitch = angle_pitch_acc;  // ตั้งค่ามุมพิทช์ของไจโรให้เท่ากับมุมพิทช์ของมาตรวัดความเร่งเมื่อสตาร์ทควอดคอปเตอร์
    angle_roll = angle_roll_acc;    // ตั้งค่ามุมการหมุนของไจโรเท่ากับมุมการหมุนของมาตรวัดความเร่งเมื่อสตาร์ทควอดคอปเตอร์
    gyro_angles_set = true;         // ตั้งค่าสถานะเริ่มต้นของ IMU

    // รีเซ็ตตัวควบคุม PID เพื่อเริ่มต้นแบบไม่มีบัมเปอร์
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //หยุดมอเตอร์: คันเร่งต่ำและหันขวา
  if (start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950) start = 0;

  // จุดตั้งค่า PID เป็นองศาต่อวินาทีถูกกำหนดโดยอินพุตตัวรับม้วน
  // จุดที่ตั้งค่า PID เป็นโหมดต่อเนื่องจะทำให้โดยต่อไปนี้ตัวรับม้วน
  pid_roll_setpoint = 0;
  // เราต้องการเดดแบนด์เล็กน้อยที่ 16us เพื่อผลลัพธ์ที่ดีกว่า
  if (receiver_input_channel_1 > 1508) pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if (receiver_input_channel_1 < 1492) pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;  // ลบการแก้ไขมุมออกจากค่าอินพุตม้วนรับมาตรฐาน
  pid_roll_setpoint /= 3.0;                // หาร setpoint สำหรับตัวควบคุมม้วน PID ด้วย 3 เพื่อให้ได้มุมเป็นองศา


  // จุดตั้งค่า PID เป็นองศาต่อวินาทีถูกกำหนดโดยอินพุตตัวรับสัญญาณพิทช์
  //ในกรณีที่หารด้วย 3 อัตราเสียงสูงสุดจะอยู่ที่ประมาณ 164 องศาต่อวินาที ( (500-8)/3 = 164 d/s )
  pid_pitch_setpoint = 0;
  // เราต้องการเดดแบนด์เล็กน้อยที่ 16us เพื่อผลลัพธ์ที่ดีกว่า
  if (receiver_input_channel_2 > 1508) pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if (receiver_input_channel_2 < 1492) pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;  // ลบการแก้ไขมุมออกจากค่าอินพุตระดับเสียงมาตรฐานของเครื่องรับ
  pid_pitch_setpoint /= 3.0;                 // หาร setpoint สำหรับ PID pitch controller ด้วย 3 เพื่อให้ได้มุมเป็นองศา

  // จุดตั้งค่า PID เป็นองศาต่อวินาทีถูกกำหนดโดยอินพุตตัวรับสัญญาณหันเห
  //ในกรณีที่หารด้วย 3 อัตราการหันเหสูงสุดจะอยู่ที่ประมาณ 164 องศาต่อวินาที ( (500-8)/3 = 164d/s )
  pid_yaw_setpoint = 0;
  // เราต้องการเดดแบนด์เล็กน้อยที่ 16us เพื่อผลลัพธ์ที่ดีกว่า
  if (receiver_input_channel_3 > 1050) {  //อย่าห้าวเมื่อปิดมอเตอร์
    if (receiver_input_channel_4 > 1508) pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492) pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  calculate_pid();  // รู้จักอินพุต PID ดังนั้นเราจึงสามารถคำนวณเอาต์พุต pid

  // ต้องใช้แรงดันแบตเตอรี่เพื่อชดเชย
  // ใช้ตัวกรองเสริมเพื่อลดสัญญาณรบกวน
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  // เปิดไฟ LED หากแรงดันแบตเตอรี่ต่ำ
  if (battery_voltage < 1000 && battery_voltage > 600) digitalWrite(12, HIGH);


  throttle = receiver_input_channel_3;  // เราต้องการสัญญาณเค้นเป็นสัญญาณพื้นฐาน

  if (start == 2) {                                                          // มอเตอร์สตาร์ท
    if (throttle > 1800) throttle = 1800;                                    // เราต้องการพื้นที่บางส่วนเพื่อควบคุมอย่างเต็มที่เมื่อเค้นเต็มที่
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;  // คำนวณพัลส์สำหรับ esc 1 (หน้า-ขวา - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;  // คำนวณพัลส์สำหรับ esc 2 (หลังขวา - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;  // คำนวณพัลส์สำหรับ esc 3 (หลังซ้าย - ทวนเข็มนาฬิกา)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;  // คำนวณพัลส์สำหรับ esc 4 (หน้า-ซ้าย - CW)

    if (battery_voltage < 1240 && battery_voltage > 800) {        // เชื่อมต่อแบตเตอรี่หรือไม่?
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);  // ชดเชยพัลส์ esc-1 สำหรับแรงดันตก
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);  // ชดเชยพัลส์ esc-2 สำหรับแรงดันตก
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);  // ชดเชยพัลส์ esc-3 สำหรับแรงดันตก
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);  // ชดเชยพัลส์ esc-4 สำหรับแรงดันตก
    }

    if (esc_1 < 1100) esc_1 = 1100;  // ให้มอเตอร์ทำงาน
    if (esc_2 < 1100) esc_2 = 1100;  // ให้มอเตอร์ทำงาน
    if (esc_3 < 1100) esc_3 = 1100;  // ให้มอเตอร์ทำงาน
    if (esc_4 < 1100) esc_4 = 1100;  // ให้มอเตอร์ทำงาน

    if (esc_1 > 2000) esc_1 = 2000;  // จำกัด esc-1 พัลส์เป็น 2000us
    if (esc_2 > 2000) esc_2 = 2000;  // จำกัด esc-2 พัลส์เป็น 2000us
    if (esc_3 > 2000) esc_3 = 2000;  // จำกัด esc-3 พัลส์เป็น 2000us
    if (esc_4 > 2000) esc_4 = 2000;  // จำกัด esc-4 พัลส์เป็น 2000us
  }

  else {
    esc_1 = 1000;  // ถ้า start ไม่ใช่ 2 ให้ 1,000us pulse สำหรับ ess-1
    esc_2 = 1000;  // ถ้า start ไม่ใช่ 2 ให้ 1,000us pulse สำหรับ ess-2
    esc_3 = 1000;  // ถ้า start ไม่ใช่ 2 ให้ 1,000us pulse สำหรับ ess-3
    esc_4 = 1000;  // ถ้า start ไม่ใช่ 2 ให้ 1,000us pulse สำหรับ ess-4
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // อธิบายการสร้างพัลส์สำหรับ ESC ในวิดีโอนี้:
  //https://youtu.be/fqEkVcqxtU8
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  // เนื่องจากการคำนวณมุม เวลาวนซ้ำจึงมีความสำคัญมาก ถ้ารอบเวลาเป็น
  // ยาวหรือสั้นกว่า 4000us การคำนวณมุมปิดอยู่ หากคุณแก้ไขรหัสให้แน่ใจว่า
  // ว่าเวลาวนซ้ำยังคงเป็น 4000us และไม่นาน! สามารถดูข้อมูลเพิ่มเติมได้ที่
  // หน้าถามตอบ:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050) digitalWrite(12, HIGH);  // เปิด LED หากเวลาวนซ้ำเกิน 4050us

  // มีข้อมูลทั้งหมดสำหรับการควบคุมมอเตอร์
  // อัตราการรีเฟรชคือ 250Hz นั่นหมายถึงความต้องการของ esc ที่นั่นจะเต้นเป็นจังหวะทุกๆ 4 มิลลิวินาที
  while (micros() - loop_timer < 4000)
    ;                     // เรารอจนกว่าจะผ่าน 4000us
  loop_timer = micros();  // ตั้งเวลาสำหรับลูปถัดไป

  PORTD |= B11110000;                    // ตั้งค่าเอาต์พุตดิจิตอลสูง 4,5,6 และ 7
  timer_channel_1 = esc_1 + loop_timer;  // คำนวณเวลาที่ขอบตกของพัลส์ esc-1
  timer_channel_2 = esc_2 + loop_timer;  // คำนวณเวลาที่ขอบตกของพัลส์ esc-2
  timer_channel_3 = esc_3 + loop_timer;  // คำนวณเวลาที่ขอบตกของพัลส์ esc-3
  timer_channel_4 = esc_4 + loop_timer;  // คำนวณเวลาที่ขอบตกของพัลส์ esc-4

  //มีเวลาว่าง 1,000us เสมอ มาทำสิ่งที่มีประโยชน์ซึ่งเสียเวลามากกันเถอะ
  // รับไจโรและข้อมูลตัวรับปัจจุบันและปรับขนาดเป็นองศาต่อวินาทีสำหรับการคำนวณ pid
  gyro_signalen();

  while (PORTD >= 16) {                                         //อยู่ในลูปนี้จนกว่าเอาต์พุต 4,5,6 และ 7 จะเหลือน้อย
    esc_loop_timer = micros();                                  // อ่านเวลาปัจจุบัน
    if (timer_channel_1 <= esc_loop_timer) PORTD &= B11101111;  // ตั้งค่าเอาต์พุตดิจิตอล 4 เป็นต่ำหากหมดเวลา
    if (timer_channel_2 <= esc_loop_timer) PORTD &= B11011111;  // ตั้งค่าเอาต์พุตดิจิตอล 5 เป็นต่ำหากหมดเวลา
    if (timer_channel_3 <= esc_loop_timer) PORTD &= B10111111;  // ตั้งค่าเอาต์พุตดิจิตอล 6 เป็นต่ำหากหมดเวลา
    if (timer_channel_4 <= esc_loop_timer) PORTD &= B01111111;  // ตั้งค่าเอาต์พุตดิจิตอล 7 เป็นต่ำหากหมดเวลา
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//รูทีนนี้ถูกเรียกทุกครั้งที่อินพุต 8, 9, 10 หรือ 11 เปลี่ยนสถานะ ใช้สำหรับอ่านสัญญาณเครื่องรับ
// ดูข้อมูลเพิ่มเติมเกี่ยวกับรูทีนย่อยนี้ได้ในวิดีโอนี้:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001) {       // อินพุต 8 สูงหรือไม่
    if (last_channel_1 == 0) {  // อินพุต 8 เปลี่ยนจาก 0 เป็น 1
      last_channel_1 = 1;       // จดจำสถานะอินพุตปัจจุบัน
      timer_1 = current_time;   // ตั้ง timer_1 เป็น current_time
    }
  } else if (last_channel_1 == 1) {              // อินพุต 8 ไม่สูงและเปลี่ยนจาก 1 เป็น 0
    last_channel_1 = 0;                          // จดจำสถานะอินพุตปัจจุบัน
    receiver_input[1] = current_time - timer_1;  // ช่อง 1 คือ current_time - timer_1
  }
  //Channel 2=========================================
  if (PINB & B00000010) {       // อินพุต 9 สูงหรือไม่
    if (last_channel_2 == 0) {  // อินพุต 9 เปลี่ยนจาก 0 เป็น 1
      last_channel_2 = 1;       // จดจำสถานะอินพุตปัจจุบัน
      timer_2 = current_time;   // ตั้ง timer_2 เป็น current_time
    }
  } else if (last_channel_2 == 1) {              // อินพุต 9 ไม่สูงและเปลี่ยนจาก 1 เป็น 0
    last_channel_2 = 0;                          // จดจำสถานะอินพุตปัจจุบัน
    receiver_input[2] = current_time - timer_2;  // ช่อง 2 คือ current_time - timer_2
  }
  //Channel 3=========================================
  if (PINB & B00000100) {       // อินพุต 10 สูงหรือไม่
    if (last_channel_3 == 0) {  // อินพุต 10 เปลี่ยนจาก 0 เป็น 1
      last_channel_3 = 1;       // จดจำสถานะอินพุตปัจจุบัน
      timer_3 = current_time;   // ตั้ง timer_3 เป็น current_time
    }
  } else if (last_channel_3 == 1) {              // อินพุต 10 ไม่สูงและเปลี่ยนจาก 1 เป็น 0
    last_channel_3 = 0;                          // จดจำสถานะอินพุตปัจจุบัน
    receiver_input[3] = current_time - timer_3;  // ช่อง 3 คือ current_time - timer_3
  }
  //Channel 4=========================================
  if (PINB & B00001000) {       // อินพุต 11 สูงหรือไม่
    if (last_channel_4 == 0) {  // อินพุต 11 เปลี่ยนจาก 0 เป็น 1
      last_channel_4 = 1;       // จดจำสถานะอินพุตปัจจุบัน
      timer_4 = current_time;   // ตั้ง timer_4 เป็น current_time
    }
  } else if (last_channel_4 == 1) {              // อินพุต 11 ไม่สูงและเปลี่ยนจาก 1 เป็น 0
    last_channel_4 = 0;                          // จดจำสถานะอินพุตปัจจุบัน
    receiver_input[4] = current_time - timer_4;  // ช่อง 4 คือ current_time - timer_4
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// รูทีนย่อยสำหรับอ่านไจโร
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen() {
  //Read the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);  // เริ่มสื่อสารกับไจโร
    Wire.write(0x3B);                      // เริ่มอ่าน @ register 43h และเพิ่มอัตโนมัติทุกครั้งที่อ่าน
    Wire.endTransmission();                //ยุติการส่งสัญญาณ
    Wire.requestFrom(gyro_address, 14);    // ขอ 14 ไบต์จากไจโร

    receiver_input_channel_1 = convert_receiver_channel(1);  // แปลงสัญญาณรับสัญญาณจริงสำหรับระดับเสียงเป็นมาตรฐาน 1,000 - 2000us
    receiver_input_channel_2 = convert_receiver_channel(2);  // แปลงสัญญาณรับสัญญาณจริงสำหรับการม้วนเป็นมาตรฐาน 1,000 - 2000us
    receiver_input_channel_3 = convert_receiver_channel(3);  // แปลงสัญญาณรับสัญญาณจริงสำหรับคันเร่งเป็นมาตรฐาน 1,000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);  // แปลงสัญญาณรับสัญญาณจริงสำหรับการหันเหเป็นมาตรฐาน 1,000 - 2000us

    while (Wire.available() < 14)
      ;                                             // รอจนกว่าจะได้รับ 14 ไบต์
    acc_axis[1] = Wire.read() << 8 | Wire.read();   // เพิ่มไบต์ต่ำและสูงให้กับตัวแปร acc_x
    acc_axis[2] = Wire.read() << 8 | Wire.read();   // เพิ่มไบต์ต่ำและสูงให้กับตัวแปร acc_y
    acc_axis[3] = Wire.read() << 8 | Wire.read();   // เพิ่มไบต์ต่ำและสูงให้กับตัวแปร acc_z
    temperature = Wire.read() << 8 | Wire.read();   // เพิ่มไบต์ต่ำและสูงให้กับตัวแปรอุณหภูมิ
    gyro_axis[1] = Wire.read() << 8 | Wire.read();  // อ่านส่วนสูงและต่ำของข้อมูลเชิงมุม
    gyro_axis[2] = Wire.read() << 8 | Wire.read();  // อ่านส่วนสูงและต่ำของข้อมูลเชิงมุม
    gyro_axis[3] = Wire.read() << 8 | Wire.read();  // อ่านส่วนสูงและต่ำของข้อมูลเชิงมุม
  }

  if (cal_int == 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];  // ชดเชยหลังจากการสอบเทียบเท่านั้น
    gyro_axis[2] -= gyro_axis_cal[2];  // ชดเชยหลังจากการสอบเทียบเท่านั้น
    gyro_axis[3] -= gyro_axis_cal[3];  // ชดเชยหลังจากการสอบเทียบเท่านั้น
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];   // ตั้งค่า gyro_roll เป็นแกนที่ถูกต้องที่เก็บไว้ใน EEPROM
  if (eeprom_data[28] & 0b10000000) gyro_roll *= -1;     // สลับ gyro_roll หากตั้งค่า MSB ของ EEPROM บิต 28
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];  // ตั้งค่า gyro_pitch เป็นแกนที่ถูกต้องที่เก็บไว้ใน EEPROM
  if (eeprom_data[29] & 0b10000000) gyro_pitch *= -1;    // สลับ gyro_pitch หากตั้งค่า MSB ของ EEPROM บิต 29
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];    // ตั้งค่า gyro_yaw เป็นแกนที่ถูกต้องที่เก็บไว้ใน EEPROM
  if (eeprom_data[30] & 0b10000000) gyro_yaw *= -1;      // สลับ gyro_yaw หากตั้งค่า MSB ของ EEPROM บิต 30

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];  //ตั้งค่า acc_x เป็นแกนที่ถูกต้องที่เก็บไว้ใน EEPROM
  if (eeprom_data[29] & 0b10000000) acc_x *= -1;   // สลับ acc_x หากตั้งค่า MSB ของ EEPROM บิต 29
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];  // ตั้งค่า acc_y เป็นแกนที่ถูกต้องซึ่งจัดเก็บไว้ใน EEPROM
  if (eeprom_data[28] & 0b10000000) acc_y *= -1;   // สลับ acc_y หากตั้งค่า MSB ของ EEPROM บิต 28
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];  //ตั้งค่า acc_z เป็นแกนที่ถูกต้องที่เก็บไว้ใน EEPROM
  if (eeprom_data[30] & 0b10000000) acc_z *= -1;   // สลับ acc_z หากตั้งค่า MSB ของ EEPROM บิต 30
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// รูทีนย่อยสำหรับคำนวณเอาต์พุต pid
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ตัวควบคุม PID ได้รับการอธิบายในส่วนที่ 5 ของเซสชันวิดีโอ YMFC-3D:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {
  // Roll คำนวณ
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  // การคำนวณระดับเสียง
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //หันเหการคำนวณ
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

// ส่วนนี้จะแปลงสัญญาณเครื่องรับจริงเป็นค่ามาตรฐาน 1,000 – 1,500 – 2,000 ไมโครวินาที
// ข้อมูลที่เก็บไว้ใน EEPROM ถูกใช้
int convert_receiver_channel(byte function) {
  byte channel, reverse;  // ก่อนอื่นเราประกาศตัวแปรท้องถิ่น
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;         // ช่องใดที่สอดคล้องกับฟังก์ชันเฉพาะ
  if (eeprom_data[function + 23] & 0b10000000) reverse = 1;  // ย้อนกลับช่องเมื่อตั้งค่าบิตที่สำคัญที่สุด
  else reverse = 0;                                          // ถ้าไม่ได้ตั้งค่าที่สำคัญที่สุด จะไม่มีการย้อนกลับ

  actual = receiver_input[channel];                                             // อ่านค่าตัวรับจริงสำหรับฟังก์ชันที่เกี่ยวข้อง
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];   // เก็บค่าต่ำสำหรับช่องรับสัญญาณเฉพาะ
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];  // เก็บค่ากลางสำหรับช่องรับสัญญาณเฉพาะ
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];    // เก็บค่าสูงสำหรับช่องรับสัญญาณเฉพาะ

  if (actual < center) {                                                   // ค่าที่รับจริงต่ำกว่าค่าศูนย์
    if (actual < low) actual = low;                                        // จำกัดค่าต่ำสุดเป็นค่าที่ตรวจพบระหว่างการตั้งค่า
    difference = ((long)(center - actual) * (long)500) / (center - low);   // คำนวณและปรับขนาดค่าจริงเป็นค่า 1,000 - 2000us
    if (reverse == 1) return 1500 + difference;                            //ถ้ากลับช่อง
    else return 1500 - difference;                                         //หากไม่กลับช่องสัญญาณ
  } else if (actual > center) {                                            // ค่าที่รับจริงสูงกว่าค่าศูนย์
    if (actual > high) actual = high;                                      // จำกัดค่าต่ำสุดเป็นค่าที่ตรวจพบระหว่างการตั้งค่า
    difference = ((long)(actual - center) * (long)500) / (high - center);  // คำนวณและปรับขนาดค่าจริงเป็นค่า 1,000 - 2000us
    if (reverse == 1) return 1500 - difference;                            //ถ้ากลับช่อง
    else return 1500 + difference;                                         //หากไม่กลับช่องสัญญาณ
  } else return 1500;
}

void set_gyro_registers() {
  //Setup the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);  //// เริ่มการสื่อสารด้วยที่อยู่ที่พบระหว่างการค้นหา
    Wire.write(0x6B);                      //เราต้องการเขียนไปยัง PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                      //ตั้งค่าบิตการลงทะเบียนเป็น 00000000 เพื่อเปิดใช้งานไจโร
    Wire.endTransmission();                //สิ้นสุดการส่งด้วยไจโร

    Wire.beginTransmission(gyro_address);  //// เริ่มการสื่อสารด้วยที่อยู่ที่พบระหว่างการค้นหา
    Wire.write(0x1B);                      //เราต้องการเขียนไปยังการลงทะเบียน GYRO_CONFIG (1B hex)
    Wire.write(0x08);                      //ตั้งค่าบิตรีจิสเตอร์เป็น 00001000 (เต็มสเกล 500dps)
    Wire.endTransmission();                //สิ้นสุดการส่งด้วยไจโร

    Wire.beginTransmission(gyro_address);  //// เริ่มการสื่อสารด้วยที่อยู่ที่พบระหว่างการค้นหา
    Wire.write(0x1C);                      //เราต้องการเขียนการลงทะเบียน ACCEL_CONFIG (1A hex)
    Wire.write(0x10);                      //ตั้งค่าบิตรีจิสเตอร์เป็น 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                //สิ้นสุดการส่งด้วยไจโร

// ลองตรวจสอบการลงทะเบียนแบบสุ่มเพื่อดูว่าเขียนค่าถูกต้องหรือไม่
    Wire.beginTransmission(gyro_address);  // เริ่มการสื่อสารด้วยที่อยู่ที่พบระหว่างการค้นหา
    Wire.write(0x1B);                      //เริ่มอ่าน @ register 0x1B
    Wire.endTransmission();               //ยุติการส่งสัญญาณ
    Wire.requestFrom(gyro_address, 1);     // ขอ 1 ไบต์จากไจโร
    while (Wire.available() < 1)
      ;                        // รอจนกว่าจะได้รับ 6 ไบต์
    if (Wire.read() != 0x08) {  //ตรวจสอบว่าค่าเป็น 0x08 หรือไม่
      digitalWrite(12, HIGH);   //เปิดไฟเตือน
      while (1) delay(10);      //อยู่ในวังวนนี้ตลอดไป
    }

    Wire.beginTransmission(gyro_address);  // เริ่มการสื่อสารด้วยที่อยู่ที่พบระหว่างการค้นหา
    Wire.write(0x1A);                      // เราต้องการเขียนไปยัง CONFIG register (1A hex)
    Wire.write(0x03);                      //ตั้งค่าบิตเรจิสเตอร์เป็น 00000011 (ตั้งค่า Digital Low Pass Filter เป็น ~43Hz)
    Wire.endTransmission();                // จบการส่งด้วยไจโร
  }
}
