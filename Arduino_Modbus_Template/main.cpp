#include <Arduino.h>
#include <PololuLedStrip.h>
//#include <avr/wdt.h>
//#include "ADC_Sampler.h"

#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long

#define NOP                      \
  do                             \
  {                              \
    __asm__ __volatile__("nop"); \
  } while (0)

PololuLedStrip<22> ledStrip1;  // LED引脚
PololuLedStrip<23> ledStrip2;  // LED引脚
PololuLedStrip<24> ledStrip3;  // LED引脚
PololuLedStrip<25> ledStrip4;  // LED引脚
PololuLedStrip<26> ledStrip5;  // LED引脚
PololuLedStrip<27> ledStrip6;  // LED引脚
PololuLedStrip<28> ledStrip7;  // LED引脚
PololuLedStrip<29> ledStrip8;  // LED引脚
PololuLedStrip<30> ledStrip9;  // LED引脚
PololuLedStrip<31> ledStrip10; // LED引脚
PololuLedStrip<32> ledStrip11; // LED引脚
PololuLedStrip<33> ledStrip12; // LED引脚
PololuLedStrip<34> ledStrip13; // LED引脚
PololuLedStrip<35> ledStrip14; // LED引脚
PololuLedStrip<36> ledStrip15; // LED引脚

PololuLedStrip<37> ledStrip16; // LED引脚
PololuLedStrip<39> ledStrip17; // LED引脚
PololuLedStrip<38> ledStrip18; // LED引脚

PololuLedStrip<40> ledStrip19; // LED引脚
PololuLedStrip<41> ledStrip20; // LED引脚

#define LED_COUNT1 160
#define LED_COUNT2 250

rgb_color colors1[LED_COUNT1];
rgb_color colors2[LED_COUNT2];
rgb_color colors3[LED_COUNT1];

rgb_color colors4[LED_COUNT1];
rgb_color colors5[LED_COUNT1];
rgb_color colors6[LED_COUNT1];

rgb_color colors7[LED_COUNT1];
rgb_color colors8[LED_COUNT1];
rgb_color colors9[LED_COUNT1];

rgb_color colors10[LED_COUNT1];
rgb_color colors11[LED_COUNT1];
rgb_color colors12[LED_COUNT1];

rgb_color colors13[LED_COUNT1];
rgb_color colors14[LED_COUNT1];
rgb_color colors15[LED_COUNT1];

rgb_color colors16[LED_COUNT1];
rgb_color colors17[LED_COUNT1];
rgb_color colors18[LED_COUNT1];

rgb_color colors19[LED_COUNT1];
rgb_color colors20[LED_COUNT1];

uint ys = 0, ysdata = 48;
uint zs = 0;

/*
向舵机写入一个微秒的值来控制舵机的轴。在一个标准舵机中，这将设置舵机齿轮的角度。在标准舵机中，参数设置为1000为完全逆时针方向，2000完全顺时针方向，1501为在中间。
注意：一些生产厂商没有按照这个标准，以至于，舵机通常响应在700到2300之间的值。自由地增加终点值直到舵机不再增加它的范围。注意，让舵机旋转超过它的终点（通常会发出异常声响）是一个高电流状态，应该被避免。
连续旋转舵机对该函数的响应类似于write()函数
语法
servo.writeMicroseconds(uS)
myservo.writeMicroseconds(1501); // set servo to mid-point

语法
servo.read()
参数说明
servo，一个类型为servo的变量
返回值
舵机的角度，从0至180度

语法
servo.detach()
参数说明
servo，一个类型为servo的变量
*/
String comdata = "";
String comdata1 = "";
uchar QieHuan = 0, QieHuan1 = 0, QieHuan2 = 0, QieHuan3 = 0; //效果切换
uchar AnJianQieHuan = 0;                                     //按键切换

uchar CeiQi = 0; //车企

uchar TuiGanWeiZhi = 0; //实时开关值
uchar BiJiao = 0;       //比较
uint ShuLiang = 0;      //数量

char data[100];
int ind = 0;

byte xx = 0XFF;
uint YiCi = 0;

boolean M1L = 0, M1H = 0, M2L = 0, M2H = 0, M3L = 0, M3H = 0;
boolean Y1 = 1, Y2 = 1, Y3 = 1, Y4 = 1, Y5 = 1, Y6 = 1, Y7 = 1, Y8 = 1, Y9 = 1, Y10 = 1, Y11 = 1, Y12 = 1;

ulong XiuMian = 0; //休眠
boolean Mian = 0;  //休眠位

void DuChuanKou();  //读串口
void DuChuanKou1(); //读串口
void ms(uint AA);   //自制软件延时

void key(void); //

void send_end(void);
void YunXing(void);           //开始运行
void DengDai(uchar BianHuan); //灯带
void YongHu(void);            //用户

void AGV1(void); // AGV小车1
void AGV2(void); // AGV小车2

// Ticker timer1(printMessage, 10);
// Ticker timer1(printMessage, 100, 0, MICROS_MICROS);//us
// Ticker timer1(printMessage, 1000, 0, MICROS_MICROS); //us
// Ticker timer1(printMessage, 400);
// Ticker timer2(printCounter, 1000, MILLIS);
// Ticker timer3(printCountdown, 1000, 5);
// Ticker timer4(blink, 501);
// Ticker timer5(printCountUS, 100, 0, MICROS_MICROS);

void setup()
{
  // SerialUSB.begin(9600);
  Serial.begin(9600);
  // Serial1.begin(9600); // 485
  Serial2.begin(9600); // LCD

  // timer1.start();
  // timer2.start();
  // timer3.start();
  // timer4.start();
  // timer5.start();

  pinMode(18, OUTPUT); //机械臂1
  pinMode(19, OUTPUT); //机械臂1
  digitalWrite(18, 1); //机械臂1
  digitalWrite(19, 1); //机械臂1

  pinMode(20, OUTPUT); //机械臂2
  pinMode(21, OUTPUT); //机械臂2
  digitalWrite(20, 1); //机械臂2
  digitalWrite(21, 1); //机械臂2

  pinMode(2, INPUT_PULLUP); //
  pinMode(3, INPUT_PULLUP); //

  pinMode(4, INPUT_PULLUP); //
  pinMode(5, INPUT_PULLUP); //

  pinMode(6, INPUT_PULLUP); //
  pinMode(7, INPUT_PULLUP); //
  pinMode(8, INPUT_PULLUP); //
  pinMode(9, INPUT_PULLUP); //

  pinMode(14, INPUT_PULLUP); // AGV 传感器 2
  pinMode(15, INPUT_PULLUP); // AGV 传感器 2

  pinMode(16, INPUT_PULLUP); // AGV 传感器 1
  pinMode(17, INPUT_PULLUP); // AGV 传感器 1

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);
  pinMode(53, OUTPUT);
  pinMode(52, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(50, OUTPUT);

  pinMode(49, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(46, OUTPUT);

  pinMode(45, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(42, OUTPUT);

  digitalWrite(A8, 1);  //
  digitalWrite(A9, 1);  //
  digitalWrite(A10, 1); //
  digitalWrite(A11, 1); //

  digitalWrite(53, 1); //
  digitalWrite(52, 1); //
  digitalWrite(51, 1); //
  digitalWrite(50, 1); //

  digitalWrite(49, 1); //
  digitalWrite(48, 1); //
  digitalWrite(47, 1); //
  digitalWrite(46, 1); //

  digitalWrite(A0, 1); //传送带
  digitalWrite(A1, 1); //传送带
  digitalWrite(A2, 1); //传送带
  digitalWrite(A3, 1); //传送带
  digitalWrite(A4, 1); //传送带
  digitalWrite(A5, 1); //传送带
  digitalWrite(A6, 1); //传送带
  digitalWrite(A7, 1); //传送带

  digitalWrite(45, 1); //传送带
  digitalWrite(44, 1); //传送带
  digitalWrite(43, 1); //传送带
  digitalWrite(42, 1); //传送带

  // myservo1.attach(2, 1000, 2501); ;  //
  // myservo2.attach(3, 1000, 2501); ;  //
  // myservo3.attach(4, 1000, 2501); ;  //

  // pinMode(encoder0PinA, INPUT);
  // digitalWrite(encoder0PinA, HIGH); // turn on pullup resistor
  // pinMode(encoder0PinB, INPUT);
  // digitalWrite(encoder0PinB, HIGH); // turn on pullup resistor

  // attachInterrupt(5, doEncoder_Expanded, CHANGE); // encoder pin on interrupt 0 - pin 2

  // ADC_Sampler.begin(200, A7,A6,A5,A4,A3,A2,A0,A1,A9,A10,A11);
  // ADC_Sampler.begin(200, A0);
  // delay(501);
  // ADC_Sampler.bufferReset(); //discard unread data in buffer for rear to catch front

  // wdt_enable(WDTO_2S); //开启看门狗
  M1L = 1;
  M1H = 1;

  M2L = 1;
  M2H = 1;

  M3L = 1;
  M3H = 1;

  Y1 = 1;  // 1
  Y11 = 1; // 2
  Y7 = 1;  // 3
  Y6 = 1;  // 4
  Y5 = 1;  // 5
  Y4 = 1;  // 6
  Y3 = 1;  // 7
  Y12 = 1; // 8
  Y8 = 1;  // 9
  Y2 = 1;  // 10
  Y10 = 1; // 11
  Y9 = 1;  // 12

  QieHuan = 0;

  // delay(1000);
  Serial2.print("page MY41"); //串口屏图片切换指令
  send_end();
  delay(100);
}

void loop()
{
  DuChuanKou(); //读串口

  // wdt_reset();
  // timer1.update();

  YunXing(); //开始运行
  if (QieHuan != 2)
  {
    AGV1(); // AGV小车1
  }

  AGV2(); // AGV小车2

  digitalWrite(A8, M3L); //
  digitalWrite(A9, M3H); //

  digitalWrite(A10, M3L); //
  digitalWrite(A11, M3H); //

  digitalWrite(53, M1L); // AGV小车 1
  digitalWrite(52, M1H); // AGV小车 1

  digitalWrite(51, M2L); // AGV小车 2
  digitalWrite(50, M2H); // AGV小车 2

  digitalWrite(A0, Y1); //传送带
  digitalWrite(A1, Y2); //传送带
  digitalWrite(A2, Y3); //传送带
  digitalWrite(A3, Y4); //传送带
  digitalWrite(A4, Y5); //传送带
  digitalWrite(A5, Y6); //传送带
  digitalWrite(A6, Y7); //传送带
  digitalWrite(A7, Y8); //传送带

  digitalWrite(45, Y9);  //传送带
  digitalWrite(44, Y10); //传送带
  digitalWrite(43, Y11); //传送带
  digitalWrite(42, Y12); //传送带

  // DuChuanKou1();
  YiCi++;
  if (YiCi > 500)
  {

    YiCi = 0;
    ledStrip1.write(colors1, LED_COUNT1);
    ledStrip2.write(colors2, LED_COUNT2);
    ledStrip3.write(colors3, LED_COUNT1);

    ledStrip4.write(colors4, LED_COUNT1);
    ledStrip5.write(colors5, LED_COUNT1);
    ledStrip6.write(colors6, LED_COUNT1);

    ledStrip7.write(colors7, LED_COUNT1);
    ledStrip8.write(colors8, LED_COUNT1);
    ledStrip9.write(colors9, LED_COUNT1);

    ledStrip10.write(colors10, LED_COUNT1);
    ledStrip11.write(colors11, LED_COUNT1);
    ledStrip12.write(colors12, LED_COUNT1);

    ledStrip13.write(colors13, LED_COUNT1);
    ledStrip14.write(colors14, LED_COUNT1);
    ledStrip15.write(colors15, LED_COUNT1);

    ledStrip16.write(colors16, LED_COUNT1);

    ledStrip17.write(colors17, LED_COUNT1);
    ledStrip18.write(colors18, LED_COUNT1);
    ledStrip19.write(colors19, LED_COUNT1);
    ledStrip20.write(colors20, LED_COUNT1);
  }
}
void AGV1(void) // AGV小车1
{

  static uint i = 0;
  static ulong YanShi1 = 0, YanShi2 = 0, YanShi3 = 0, YanShi4 = 0, YanShi5 = 0, YanShi6 = 0;
  static uchar temp = 20, temp1 = 0;
  static boolean Y = 0;
  static ulong Shan = 0;

  switch (QieHuan1) //　　QieHuan1=0;//切换
  {
  case 0:

    M1L = 1;
    M1H = 1;
    break;
  case 6:

    M1L = 0;
    M1H = 1;
    QieHuan1 = 7; //切换
    break;
  case 7:

    if (digitalRead(14) == 0) // 3   左       2 右
    {
      YanShi1++;
      if (YanShi1 > 10)
      {

        M1L = 0;
        M1H = 0;
        QieHuan1 = 8; //切换
        YanShi1 = 0;
      }
    }
    break;
  case 8:
    M1L = 0;
    M1H = 0;

    YanShi1++;
    if (YanShi1 > 3000)
    {
      QieHuan1 = 12; //切换
      YanShi1 = 0;
    }

    break;
  case 9:

    break;
  case 10:

    break;

  case 11:

    break;
  case 12:

    M1L = 1;
    M1H = 0;

    QieHuan1 = 13; //切换

    break;
  case 13:

    if (digitalRead(15) == 0) // 3   左       2 右
    {
      YanShi1++;
      if (YanShi1 > 10)
      {

        M1L = 0;
        M1H = 0;
        QieHuan1 = 14; //切换
        YanShi1 = 0;
      }
    }
    break;
  case 14:
    M1L = 0;
    M1H = 0;
    YanShi1++;
    if (YanShi1 > 3000)
    {
      QieHuan1 = 6; //切换
      YanShi1 = 0;
    }
    // QieHuan3 = 0; //切换
    break;
  case 15:

    break;
  case 16:

    break;
  case 17:

    M1L = 0;
    M1H = 1;
    QieHuan1 = 18; //切换

    break;
  case 18:

    break;
  case 19:

    break;
  }
}
void AGV2(void) // AGV小车2
{

  static uint i = 0;
  static ulong YanShi1 = 0, YanShi2 = 0, YanShi3 = 0, YanShi4 = 0, YanShi5 = 0, YanShi6 = 0;
  static uchar temp = 20, temp1 = 0;
  static boolean Y = 0;
  static ulong Shan = 0;

  switch (QieHuan2) //　　QieHuan3=0;//切换
  {
  case 0:

    M2L = 1;
    M2H = 1;
    break;
  case 6:

    M2L = 0;
    M2H = 1;
    QieHuan2 = 7; //切换
    break;
  case 7:

    if (digitalRead(16) == 0) // 3   左       2 右
    {

      YanShi1++;
      if (YanShi1 > 10)
      {
        M2L = 0;
        M2H = 0;

        QieHuan2 = 8; //切换
        YanShi1 = 0;
      }
    }
    break;
  case 8:
    M2L = 0;
    M2H = 0;
    YanShi1++;
    if (YanShi1 > 3000)
    {
      QieHuan2 = 12; //切换
      YanShi1 = 0;
    }
    break;
  case 9:

    break;
  case 10:

    break;

  case 11:

    break;
  case 12:

    M2L = 1;
    M2H = 0;

    QieHuan2 = 13; //切换

    break;
  case 13:

    if (digitalRead(17) == 0) // 3   左       2 右
    {
      YanShi1++;
      if (YanShi1 > 10)
      {
        M2L = 0;
        M2H = 0;

        QieHuan2 = 14; //切换
        YanShi1 = 0;
      }
    }
    break;
  case 14:
    M2L = 0;
    M2H = 0;
    YanShi1++;
    if (YanShi1 > 3000)
    {
      QieHuan2 = 6; //切换
      YanShi1 = 0;
    }
    // QieHuan3 = 0; //切换
    break;
  case 15:

    break;
  case 16:

    break;
  case 17:

    M2L = 0;
    M2H = 1;
    QieHuan2 = 18; //切换

    break;
  case 18:

    break;
  case 19:

    break;
  }
}
void YunXing(void) //开始运行
{
  static ulong YanShi1 = 0, YanShi2 = 0, YanShi3 = 0, YanShi4 = 0, YanShi5 = 0, YanShi6 = 0;
  static uchar temp = 0, temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;

  switch (QieHuan) //开始
  {
  case 0:
    QieHuan1 = 0; //切换  右AGV   下方   AGV
    QieHuan2 = 0; //切换  左AGV   右侧   AGV    =6 退   =12进

    digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(19, 1); //下方    机械臂1  执行

    digitalWrite(20, 1); //上方   机械臂2  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(21, 1); //上方   机械臂2  执行

    digitalWrite(49, 1); //
    digitalWrite(48, 1); //
    digitalWrite(47, 1); //

    // digitalWrite(46, 1); //
    digitalWrite(46, 1); //小显示屏
    for (int C = 0; C < LED_COUNT2; C++)
    {

      colors2[C] = rgb_color(0, 0, 0); // 右上角
    }
    for (int C = 0; C < LED_COUNT1; C++)
    {
      colors5[C] = rgb_color(0, 0, 0); // 右下角

      colors13[C] = rgb_color(0, 0, 0); // 1
      colors15[C] = rgb_color(0, 0, 0); // 2
      colors14[C] = rgb_color(0, 0, 0); // 3
      colors11[C] = rgb_color(0, 0, 0); // 4
      colors12[C] = rgb_color(0, 0, 0); // 5
      colors4[C] = rgb_color(0, 0, 0);  // 6
      colors3[C] = rgb_color(0, 0, 0);  // 7
      colors1[C] = rgb_color(0, 0, 0);  // 8
      colors9[C] = rgb_color(0, 0, 0);  // 9
      colors20[C] = rgb_color(0, 0, 0); // 10
      colors10[C] = rgb_color(0, 0, 0); // 11

      colors17[C] = rgb_color(0, 0, 0); // 12
      colors18[C] = rgb_color(0, 0, 0); // 13
      colors19[C] = rgb_color(0, 0, 0); // 14
      colors8[C] = rgb_color(0, 0, 0);  // 15
      colors6[C] = rgb_color(0, 0, 0);  // 16
      colors7[C] = rgb_color(0, 0, 0);  // 17

      colors16[C] = rgb_color(100, 100, 100); //
    }
    temp1 = 0;
    temp2 = 0;
    temp3 = 0;
    temp4 = 0;
    break;
  case 1:                //////BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB
    digitalWrite(46, 0); //小显示屏
    for (int C = 0; C < LED_COUNT2; C++)
    {

      colors2[C] = rgb_color(0, 0, 100); // 右上角
    }
    for (int C = 0; C < LED_COUNT1; C++)
    {
      colors5[C] = rgb_color(0, 150, 100); // 右下角

      colors13[C] = rgb_color(100, 0, 100); // 1
      colors15[C] = rgb_color(100, 0, 100); // 2
      colors14[C] = rgb_color(100, 0, 100); // 3
      colors11[C] = rgb_color(100, 0, 100); // 4
      colors12[C] = rgb_color(100, 0, 100); // 5
      colors4[C] = rgb_color(100, 0, 100);  // 6
      colors3[C] = rgb_color(100, 0, 100);  // 7
      colors1[C] = rgb_color(100, 0, 100);  // 8
      colors9[C] = rgb_color(100, 0, 100);  // 9
      colors20[C] = rgb_color(100, 0, 100); // 10
      colors10[C] = rgb_color(100, 0, 100); // 11

      colors17[C] = rgb_color(100, 0, 100); // 12
      colors18[C] = rgb_color(100, 0, 100); // 13
      colors19[C] = rgb_color(100, 0, 100); // 14
      colors8[C] = rgb_color(100, 0, 100);  // 15
      colors6[C] = rgb_color(0, 0, 100);    // 16
      colors7[C] = rgb_color(0, 0, 100);    // 17

      colors16[C] = rgb_color(100, 100, 100); //
    }
    Y1 = 1;  // 1
    Y11 = 1; // 2
    Y7 = 1;  // 3
    Y6 = 1;  // 4
    Y5 = 1;  // 5
    Y4 = 1;  // 6
    Y3 = 1;  // 7
    Y12 = 1; // 8
    Y8 = 1;  // 9
    Y2 = 1;  // 10
    Y10 = 1; // 11
    Y9 = 1;  // 12
    temp1 = 0;
    temp2 = 0;
    temp3 = 0;
    temp4 = 0;
    QieHuan1 = 0; //切换  右AGV   下方   AGV
    QieHuan2 = 0; //切换  左AGV   右侧   AGV    =6 退   =12进

    digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(19, 1); //下方    机械臂1  执行

    digitalWrite(20, 1); //上方   机械臂2  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(21, 1); //上方   机械臂2  执行

    break;
  case 2: ////////////CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
    // temp1 = 0;
    temp2 = 0;
    temp3 = 0;
    // temp4 = 0;
    digitalWrite(46, 0); //小显示屏
    switch (temp1)       //
    {
    case 0:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(0, 0, 0);     // 3
        colors11[C] = rgb_color(0, 0, 0);     // 4
        colors12[C] = rgb_color(0, 0, 0);     // 5
        colors4[C] = rgb_color(0, 0, 0);      // 6
        colors3[C] = rgb_color(0, 0, 0);      // 7
        colors1[C] = rgb_color(0, 0, 0);      // 8
        colors9[C] = rgb_color(0, 0, 0);      // 9
        colors10[C] = rgb_color(0, 0, 0);     // 11
        colors20[C] = rgb_color(0, 0, 0);     // 10
        colors17[C] = rgb_color(0, 0, 0);     // 12
        colors18[C] = rgb_color(0, 0, 0);     // 13
        colors19[C] = rgb_color(0, 0, 0);     // 14
        colors8[C] = rgb_color(0, 0, 0);      // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 1; // 2
      Y7 = 1;  // 3
      Y6 = 1;  // 4
      Y5 = 1;  // 5
      Y4 = 1;  // 6
      Y3 = 1;  // 7
      Y12 = 1; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12

      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 1;
        YanShi1 = 0;
      }
      break;
    case 1:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(0, 0, 0);     // 5
        colors4[C] = rgb_color(0, 0, 0);      // 6
        colors3[C] = rgb_color(0, 0, 0);      // 7
        colors1[C] = rgb_color(0, 0, 0);      // 8
        colors9[C] = rgb_color(0, 0, 0);      // 9
        colors10[C] = rgb_color(0, 0, 0);     // 11
        colors20[C] = rgb_color(0, 0, 0);     // 10
        colors17[C] = rgb_color(0, 0, 0);     // 12
        colors18[C] = rgb_color(0, 0, 0);     // 13
        colors19[C] = rgb_color(0, 0, 0);     // 14
        colors8[C] = rgb_color(0, 0, 0);      // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 1;  // 3
      Y6 = 1;  // 4
      Y5 = 1;  // 5
      Y4 = 1;  // 6
      Y3 = 1;  // 7
      Y12 = 1; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 2;
        YanShi1 = 0;
      }
      break;
    case 2:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(0, 0, 0);      // 7
        colors1[C] = rgb_color(0, 0, 0);      // 8
        colors9[C] = rgb_color(0, 0, 0);      // 9
        colors10[C] = rgb_color(0, 0, 0);     // 11
        colors20[C] = rgb_color(0, 0, 0);     // 10
        colors17[C] = rgb_color(0, 0, 0);     // 12
        colors18[C] = rgb_color(0, 0, 0);     // 13
        colors19[C] = rgb_color(0, 0, 0);     // 14
        colors8[C] = rgb_color(0, 0, 0);      // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 1;  // 4
      Y5 = 1;  // 5
      Y4 = 1;  // 6
      Y3 = 1;  // 7
      Y12 = 1; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 3;
        YanShi1 = 0;
      }
      break;
    case 3:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(0, 0, 0);      // 7
        colors1[C] = rgb_color(0, 0, 0);      // 8
        colors9[C] = rgb_color(0, 0, 0);      // 9
        colors10[C] = rgb_color(0, 0, 0);     // 11
        colors20[C] = rgb_color(0, 0, 0);     // 10
        colors17[C] = rgb_color(0, 0, 0);     // 12
        colors18[C] = rgb_color(0, 0, 0);     // 13
        colors19[C] = rgb_color(0, 0, 0);     // 14
        colors8[C] = rgb_color(0, 0, 0);      // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 1;  // 5
      Y4 = 1;  // 6
      Y3 = 1;  // 7
      Y12 = 1; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 4;
        YanShi1 = 0;
      }
      break;
    case 4:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(0, 0, 0);      // 8
        colors9[C] = rgb_color(0, 0, 0);      // 9
        colors10[C] = rgb_color(0, 0, 0);     // 11
        colors20[C] = rgb_color(0, 0, 0);     // 10
        colors17[C] = rgb_color(0, 0, 0);     // 12
        colors18[C] = rgb_color(0, 0, 0);     // 13
        colors19[C] = rgb_color(0, 0, 0);     // 14
        colors8[C] = rgb_color(0, 0, 0);      // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 1;  // 6
      Y3 = 1;  // 7
      Y12 = 1; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 5;
        YanShi1 = 0;
      }
      break;
    case 5:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors10[C] = rgb_color(0, 0, 0);     // 11
        colors20[C] = rgb_color(0, 0, 0);     // 10
        colors17[C] = rgb_color(0, 0, 0);     // 12
        colors18[C] = rgb_color(0, 0, 0);     // 13
        colors19[C] = rgb_color(0, 0, 0);     // 14
        colors8[C] = rgb_color(0, 0, 0);      // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 0;  // 6
      Y3 = 1;  // 7
      Y12 = 1; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 6;
        YanShi1 = 0;
      }
      break;
    case 6:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors10[C] = rgb_color(0, 0, 0);     // 11

        colors17[C] = rgb_color(0, 0, 0); // 12
        colors18[C] = rgb_color(0, 0, 0); // 13
        colors19[C] = rgb_color(0, 0, 0); // 14
        colors8[C] = rgb_color(0, 0, 0);  // 15
        colors6[C] = rgb_color(0, 0, 0);  // 16
        colors7[C] = rgb_color(0, 0, 0);  // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 0;  // 6
      Y3 = 0;  // 7
      Y12 = 1; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 7;
        YanShi1 = 0;
      }
      break;
    case 7:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors10[C] = rgb_color(100, 0, 100); // 11
        colors17[C] = rgb_color(0, 0, 0);     // 12
        colors18[C] = rgb_color(0, 0, 0);     // 13
        colors19[C] = rgb_color(0, 0, 0);     // 14
        colors8[C] = rgb_color(0, 0, 0);      // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 0;  // 6
      Y3 = 0;  // 7
      Y12 = 0; // 8
      Y8 = 1;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 8;
        YanShi1 = 0;
      }
      break;
    case 8:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors10[C] = rgb_color(100, 0, 100); // 11

        colors17[C] = rgb_color(100, 0, 100); // 12
        colors18[C] = rgb_color(100, 0, 100); // 13

        colors19[C] = rgb_color(0, 0, 0); // 14
        colors8[C] = rgb_color(0, 0, 0);  // 15
        colors6[C] = rgb_color(0, 0, 0);  // 16
        colors7[C] = rgb_color(0, 0, 0);  // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 0;  // 6
      Y3 = 0;  // 7
      Y12 = 0; // 8
      Y8 = 0;  // 9
      Y2 = 1;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 9;
        YanShi1 = 0;
      }
      break;
    case 9:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors10[C] = rgb_color(100, 0, 100); // 11
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors17[C] = rgb_color(100, 0, 100); // 12
        colors18[C] = rgb_color(100, 0, 100); // 13
        colors19[C] = rgb_color(100, 0, 100); // 14

        colors8[C] = rgb_color(0, 0, 0); // 15
        colors6[C] = rgb_color(0, 0, 0); // 16
        colors7[C] = rgb_color(0, 0, 0); // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 0;  // 6
      Y3 = 0;  // 7
      Y12 = 0; // 8
      Y8 = 0;  // 9
      Y2 = 0;  // 10
      Y10 = 1; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 10;
        YanShi1 = 0;
      }
      break;
    case 10:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors10[C] = rgb_color(100, 0, 100); // 11
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors17[C] = rgb_color(100, 0, 100); // 12
        colors18[C] = rgb_color(100, 0, 100); // 13
        colors19[C] = rgb_color(100, 0, 100); // 14
        colors8[C] = rgb_color(100, 0, 100);  // 15

        colors6[C] = rgb_color(0, 0, 100); // 16
        colors7[C] = rgb_color(0, 0, 0);   // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 0;  // 6
      Y3 = 0;  // 7
      Y12 = 0; // 8
      Y8 = 0;  // 9
      Y2 = 0;  // 10
      Y10 = 0; // 11
      Y9 = 1;  // 12
      YanShi1++;
      if (YanShi1 > 501)
      {
        temp1 = 11;
        YanShi1 = 0;
      }
      break;
    case 11:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors10[C] = rgb_color(100, 0, 100); // 11
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors17[C] = rgb_color(100, 0, 100); // 12
        colors18[C] = rgb_color(100, 0, 100); // 13
        colors19[C] = rgb_color(100, 0, 100); // 14
        colors8[C] = rgb_color(100, 0, 100);  // 15
        colors6[C] = rgb_color(0, 0, 100);    // 16
        colors7[C] = rgb_color(0, 0, 100);    // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;  // 1
      Y11 = 0; // 2
      Y7 = 0;  // 3
      Y6 = 0;  // 4
      Y5 = 0;  // 5
      Y4 = 0;  // 6
      Y3 = 0;  // 7
      Y12 = 0; // 8
      Y8 = 0;  // 9
      Y2 = 0;  // 10
      Y10 = 0; // 11
      Y9 = 0;  // 12

      // QieHuan1 = 12; //切换 右AGV
      QieHuan2 = 6; //切换  左AGV   =6 退   =12进

      temp1 = 12;

      break;
    case 12:

      digitalWrite(20, 0); //机械臂2  循环 ==0 一直循环     ==1 执行一次
      digitalWrite(21, 0); //机械臂2  执行

      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 100); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 150, 100); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors10[C] = rgb_color(100, 0, 100); // 11
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors17[C] = rgb_color(100, 0, 100); // 12
        colors18[C] = rgb_color(100, 0, 100); // 13
        colors19[C] = rgb_color(100, 0, 100); // 14
        colors8[C] = rgb_color(100, 0, 100);  // 15
        colors6[C] = rgb_color(0, 0, 100);    // 16
        colors7[C] = rgb_color(0, 0, 100);    // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      switch (temp4) //
      {
      case 0:
        M1L = 1;
        M1H = 0;
        digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
        digitalWrite(19, 1); //下方    机械臂1  执行
        temp4 = 1;
        break;
      case 1:
        if (digitalRead(15) == 0) // 3   左       2 右
        {
          YanShi6++;
          if (YanShi6 > 10)
          {

            M1L = 0;
            M1H = 0;
            temp4 = 2; //切换
            YanShi6 = 0;
          }
        }
        break;
      case 2:
        M1L = 0;
        M1H = 0;
        digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
        digitalWrite(19, 0); //下方    机械臂1  执行

        YanShi6++;
        if (YanShi6 > 15000)
        {
          temp4 = 3;
          YanShi6 = 0;
        }
        break;
      case 3:
        M1L = 0;
        M1H = 1;

        temp4 = 5;
        break;
      case 5:
        if (digitalRead(14) == 0) // 3   左       2 右
        {
          YanShi6++;
          if (YanShi6 > 10)
          {

            M1L = 0;
            M1H = 0;
            temp4 = 6; //切换
            YanShi6 = 0;
          }
        }
        break;
      case 6:
        M1L = 0;
        M1H = 0;
        YanShi6++;
        if (YanShi6 > 2000)
        {
          temp4 = 0;
          YanShi6 = 0;
        }
        break;
      case 7:

        break;
      }

      break;
    case 13:

      break;
    case 14:

      break;
    }

    break;
  case 3: /////////DDDDDDDDDDDDDDDDDDDDDDDDDDD

    // M1L = 1;
    // M1H = 0;
    // M2L = 1;
    // M2H = 0;
    digitalWrite(46, 1); //小显示屏
    temp1 = 0;
    // temp2 = 0;
    temp3 = 0;
    temp4 = 0;
    switch (temp2) //
    {
    case 0:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 0); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(100, 0, 100); // 1
        colors15[C] = rgb_color(100, 0, 100); // 2
        colors14[C] = rgb_color(100, 0, 100); // 3
        colors11[C] = rgb_color(100, 0, 100); // 4
        colors12[C] = rgb_color(100, 0, 100); // 5
        colors4[C] = rgb_color(100, 0, 100);  // 6
        colors3[C] = rgb_color(100, 0, 100);  // 7
        colors1[C] = rgb_color(100, 0, 100);  // 8
        colors9[C] = rgb_color(100, 0, 100);  // 9
        colors10[C] = rgb_color(100, 0, 100); // 11
        colors20[C] = rgb_color(100, 0, 100); // 10
        colors17[C] = rgb_color(100, 0, 100); // 12
        colors18[C] = rgb_color(100, 0, 100); // 13
        colors19[C] = rgb_color(100, 0, 100); // 14
        colors8[C] = rgb_color(100, 0, 100);  // 15
        colors6[C] = rgb_color(0, 0, 0);      // 16
        colors7[C] = rgb_color(0, 0, 0);      // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 0;        // 1
      Y11 = 0;       // 2
      Y7 = 0;        // 3
      Y6 = 0;        // 4
      Y5 = 0;        // 5
      Y4 = 0;        // 6
      Y3 = 0;        // 7
      Y12 = 0;       // 8
      Y8 = 0;        // 9
      Y2 = 0;        // 10
      Y10 = 0;       // 11
      Y9 = 1;        // 12
      QieHuan1 = 12; //切换  右AGV   下方   AGV
      QieHuan2 = 0;  //切换  左AGV   右侧   AGV    =6 退   =12进

      temp2 = 1;
      break;
    case 1:
      digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
      digitalWrite(19, 1); //下方    机械臂1  执行

      digitalWrite(20, 0); //上方   机械臂2  循环 ==0 一直循环     ==1 执行一次
      digitalWrite(21, 0); //上方   机械臂2  执行

      break;
    case 2:

      break;
    }
    break;
  case 4: ///////EEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
    temp1 = 0;
    temp2 = 0;
    // temp3 = 0;
    temp4 = 0;
    // M1L = 0;
    // M1H = 1;
    // M2L = 0;
    // M2H = 1;

    digitalWrite(46, 1); //小显示屏

    digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(19, 1); //下方    机械臂1  执行

    digitalWrite(20, 1); //上方   机械臂2  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(21, 1); //上方   机械臂2  执行

    switch (temp3) //
    {
    case 0:
      for (int C = 0; C < LED_COUNT2; C++)
      {

        colors2[C] = rgb_color(0, 0, 100); // 右上角
      }
      for (int C = 0; C < LED_COUNT1; C++)
      {
        colors5[C] = rgb_color(0, 0, 0); // 右下角

        colors13[C] = rgb_color(0, 0, 0); // 1
        colors15[C] = rgb_color(0, 0, 0); // 2
        colors14[C] = rgb_color(0, 0, 0); // 3
        colors11[C] = rgb_color(0, 0, 0); // 4
        colors12[C] = rgb_color(0, 0, 0); // 5
        colors4[C] = rgb_color(0, 0, 0);  // 6
        colors3[C] = rgb_color(0, 0, 0);  // 7
        colors1[C] = rgb_color(0, 0, 0);  // 8
        colors9[C] = rgb_color(0, 0, 0);  // 9
        colors10[C] = rgb_color(0, 0, 0); // 11
        colors20[C] = rgb_color(0, 0, 0); // 10
        colors17[C] = rgb_color(0, 0, 0); // 12
        colors18[C] = rgb_color(0, 0, 0); // 13
        colors19[C] = rgb_color(0, 0, 0); // 14
        colors8[C] = rgb_color(0, 0, 0);  // 15

        colors6[C] = rgb_color(0, 0, 100); // 16
        colors7[C] = rgb_color(0, 0, 100); // 17

        colors16[C] = rgb_color(100, 100, 100); //
      }
      Y1 = 1;       // 1
      Y11 = 1;      // 2
      Y7 = 1;       // 3
      Y6 = 1;       // 4
      Y5 = 1;       // 5
      Y4 = 1;       // 6
      Y3 = 1;       // 7
      Y12 = 1;      // 8
      Y8 = 1;       // 9
      Y2 = 1;       // 10
      Y10 = 1;      // 11
      Y9 = 0;       // 12
      QieHuan1 = 0; //切换  右AGV   下方   AGV
      QieHuan2 = 6; //切换  右AGV   下方   AGV

      // digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
      // digitalWrite(19, 1); //下方    机械臂1  执行

      // digitalWrite(20, 1); //上方   机械臂2  循环 ==0 一直循环     ==1 执行一次
      // digitalWrite(21, 1); //上方   机械臂2  执行

      temp3 = 1;
      break;
    case 1:

      break;
    case 2:

      break;
    }
    break;
  case 5:
    digitalWrite(46, 0); //小显示屏
    for (int C = 0; C < LED_COUNT1; C++)
    {
      colors5[C] = rgb_color(0, 150, 100); // 右下角
    }

    digitalWrite(18, 1); //下方    机械臂1  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(19, 1); //下方    机械臂1  执行

    digitalWrite(20, 1); //上方   机械臂2  循环 ==0 一直循环     ==1 执行一次
    digitalWrite(21, 1); //上方   机械臂2  执行

    digitalWrite(49, 1); //
    digitalWrite(48, 1); //
    digitalWrite(47, 1); //

    for (int C = 0; C < LED_COUNT2; C++)
    {

      colors2[C] = rgb_color(0, 0, 0); // 右上角
    }
    for (int C = 0; C < LED_COUNT1; C++)
    {
      // colors5[C] = rgb_color(0, 0, 0); // 右下角

      colors13[C] = rgb_color(0, 0, 0); // 1
      colors15[C] = rgb_color(0, 0, 0); // 2
      colors14[C] = rgb_color(0, 0, 0); // 3
      colors11[C] = rgb_color(0, 0, 0); // 4
      colors12[C] = rgb_color(0, 0, 0); // 5
      colors4[C] = rgb_color(0, 0, 0);  // 6
      colors3[C] = rgb_color(0, 0, 0);  // 7
      colors1[C] = rgb_color(0, 0, 0);  // 8
      colors9[C] = rgb_color(0, 0, 0);  // 9
      colors20[C] = rgb_color(0, 0, 0); // 10
      colors10[C] = rgb_color(0, 0, 0); // 11

      colors17[C] = rgb_color(0, 0, 0); // 12
      colors18[C] = rgb_color(0, 0, 0); // 13
      colors19[C] = rgb_color(0, 0, 0); // 14
      colors8[C] = rgb_color(0, 0, 0);  // 15
      colors6[C] = rgb_color(0, 0, 0);  // 16
      colors7[C] = rgb_color(0, 0, 0);  // 17

      colors16[C] = rgb_color(100, 100, 100); //
    }
    temp1 = 0;
    temp2 = 0;
    temp3 = 0;
    temp4 = 0;
    break;
  case 6:

    break;
  }
}

void send_end(void)
{
  Serial2.write(xx);
  Serial2.write(xx);
  Serial2.write(xx);
}
void printMessage() //定时器调用
{
  static boolean temp = 0;
}

void ms(uint AA) //自制软件延时
{
  for (int ii = 0; ii < AA; ii++)
  {
    for (ulong jj = 0; jj < 10; jj++)
      NOP;
  }
}
void DuChuanKou1() //读串口
{

  if (Serial.available()) // if (Serial.available())
  {

    while (Serial.available() > 0)
    {

      comdata += char(Serial.read());
      delay(1);
    }
    if (!comdata.compareTo("0")) //
    {
    }

    if (!comdata.compareTo("1")) // comdata
    {

      Serial.println("OK");
    }

    // Serial.println(comdata);
    comdata = "";
  }
}

void DuChuanKou() //读串口
{
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      comdata1 += char(Serial.read());
      delay(2);
    }

    if (!comdata1.compareTo("A") || !comdata1.compareTo("a")) // 1
    {
      Serial.println("OK");
      QieHuan = 0;
      QieHuan1 = 0; //切换
      QieHuan2 = 0; //切换
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
    }
    if (!comdata1.compareTo("B") || !comdata1.compareTo("b")) // 2
    {

      Serial.println("OK");
      QieHuan = 1;
      // QieHuan1 = 6; //切换 右AGV
      // QieHuan2 = 6; //切换  左AGV   =6 退   =12进
    }
    if (!comdata1.compareTo("C") || !comdata1.compareTo("c")) // 3
    {

      Serial.println("OK");
      QieHuan = 2;
      // QieHuan1 = 6;  //切换 右AGV
      // QieHuan2 = 12; //切换  左AGV
    }
    if (!comdata1.compareTo("D") || !comdata1.compareTo("d")) // 4
    {
      Serial.println("OK");
      QieHuan = 3;
    }
    if (!comdata1.compareTo("E") || !comdata1.compareTo("e")) // 5
    {
      Serial.println("OK");
      QieHuan = 4;
    }
    if (!comdata1.compareTo("F") || !comdata1.compareTo("f")) // 6
    {
      Serial.println("OK");
      QieHuan = 5;
    }
    if (!comdata1.compareTo("1")) // 0
    {
      Y1 = 0;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("2")) // 0
    {
      Y1 = 1;
      Y2 = 0;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("3")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 0;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("4")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 0;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("5")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 0;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("6")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 0;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("7")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 0;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("8")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 0;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("9")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 0;
      Y10 = 1;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("10")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 0;
      Y11 = 1;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("11")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 0;
      Y12 = 1;
      Serial.println("OK");
    }
    if (!comdata1.compareTo("12")) // 0
    {
      Y1 = 1;
      Y2 = 1;
      Y3 = 1;
      Y4 = 1;
      Y5 = 1;
      Y6 = 1;
      Y7 = 1;
      Y8 = 1;
      Y9 = 1;
      Y10 = 1;
      Y11 = 1;
      Y12 = 0;
      Serial.println("OK");
    }
    comdata1 = "";
  }
}
