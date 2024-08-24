#define MAX_SPEED 170   // максимальная скорость моторов (0-255)

#define MOTOR_TEST 0    // тест моторов
// при запуске крутятся ВПЕРЁД по очереди:
// FL - передний левый
// FR - передний правый
// BL - задний левый
// BR - задний правый

// пины драйверов (_B должен быть ШИМ)
#define MOTOR1_A 2
#define MOTOR1_B 3  // ШИМ!
#define MOTOR2_A 4
#define MOTOR2_B 5  // ШИМ!
#define MOTOR3_A 7
#define MOTOR3_B 6  // ШИМ!
#define MOTOR4_A 8
#define MOTOR4_B 9  // ШИМ!

#include <GyverMotor.h>
// тут можно поменять моторы местами
GMotor motorBR(DRIVER2WIRE, MOTOR1_A, MOTOR1_B, HIGH);
GMotor motorFR(DRIVER2WIRE, MOTOR2_A, MOTOR2_B, HIGH);
GMotor motorBL(DRIVER2WIRE, MOTOR3_A, MOTOR3_B, HIGH);
GMotor motorFL(DRIVER2WIRE, MOTOR4_A, MOTOR4_B, HIGH);


char inChar;
String input = "                      ";
byte currCharN = 0;


void setup() 
{
  Serial.begin(115200);
  Serial.println("start");

  PWMOverclock();  // Разгон шима, чтоб не пищал 
  motorsTuning();  // Настройка моторов

#if (MOTOR_TEST == 1)
  motorsTest1();   // тест моторов
#endif

#if (MOTOR_TEST == 2)
  motorsTest2();   // тест моторов
#endif  
} 


void loop() 
{
  if (Serial.available() > 0)
  {
    inChar = Serial.read();
    if (inChar == '\n')
    {
      return;
    }
    input[currCharN++] = inChar;
    
    if (inChar == '\r')
    {
      if (input[0] == 'm' && input[1] == '2')
      {
        updateMotors2();
        delayMicroseconds(100);

      }
      else if (input[0] == 'm' && input[1] == '4')
      {
        updateMotors4();
        delayMicroseconds(100);
      }
      else if (input[0] == 's')
      {
        motorsStop();
        Serial.println("Stop motors");
      }
      else if (input[0] == 'e')
      {
        
        Serial.println("No data");
      }

      inputClear();
      currCharN = 0;
    }
    if (currCharN > 31)
    {
      inputClear();
      currCharN = 0;
    }
  }
}


void inputClear()
{
  for (int i = 0; i < 32; i++)
  {
    input[i] = ' ';
  }
}


void PWMOverclock()
{
  // чуть подразгоним ШИМ https://alexgyver.ru/lessons/pwm-overclock/
  // Пины D3 и D11 - 980 Гц
  TCCR2B = 0b00000100;  // x64
  TCCR2A = 0b00000011;  // fast pwm

  // Пины D9 и D10 - 976 Гц
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00001011;  // x64 fast pwm
}


void motorsTest1()
{
  Serial.println("Test 1");
  Serial.println("front left");
  motorFL.run(FORWARD, 100);
  delay(3000);
  motorFL.run(STOP);
  delay(1000);
  Serial.println("front right");
  motorFR.run(FORWARD, 100);
  delay(3000);
  motorFR.run(STOP);
  delay(1000);
  Serial.println("back left");
  motorBL.run(FORWARD, 100);
  delay(3000);
  motorBL.run(STOP);
  delay(1000);
  Serial.println("back right");
  motorBR.run(FORWARD, 100);
  delay(3000);
  motorBR.run(STOP);
}


void motorsTest2()
{
  Serial.println("Test 2");
  
  Serial.println("front left");
  for (int i=100; i < 200; i += 10)
  {
    motorFL.run(FORWARD, i);
    Serial.println(i);
    delay(250);
  }
  motorFL.run(STOP);
  delay(1000);
  
  Serial.println("front right");
  for (int i=100; i < 200; i += 10)
  {
    motorFR.run(FORWARD, i);
    Serial.println(i);
    delay(250);
  }
  motorFR.run(STOP);
  delay(1000);
  
  Serial.println("back left");
  for (int i=100; i < 200; i += 10)
  {
    motorBL.run(FORWARD, i);
    Serial.println(i);
    delay(250);
  }
  motorBL.run(STOP);
  delay(1000);
  
  Serial.println("back right");
  for (int i=100; i < 200; i += 10)
  {
    motorBR.run(FORWARD, i);
    Serial.println(i);
    delay(250);
  }
  motorBR.run(STOP);
}


void motorsTuning()
{
  // направление глобального вращегия моторов
  motorFR.setDirection(FORWARD);
  motorBR.setDirection(FORWARD);
  motorFL.setDirection(REVERSE);
  motorBL.setDirection(REVERSE);

  // минимальный сигнал на мотор
  motorFR.setMinDuty(30);
  motorBR.setMinDuty(30);
  motorFL.setMinDuty(30);
  motorBL.setMinDuty(30);

  // режим мотора в АВТО
  motorFR.setMode(AUTO);
  motorBR.setMode(AUTO);
  motorFL.setMode(AUTO);
  motorBL.setMode(AUTO);

  // скорость плавности
  motorFR.setSmoothSpeed(60);
  motorBR.setSmoothSpeed(60);
  motorFL.setSmoothSpeed(60);
  motorBL.setSmoothSpeed(60);
}


void motorsStop()
{
  motorFR.setSpeed(0);
  motorBR.setSpeed(0);
  motorFL.setSpeed(0);
  motorBL.setSpeed(0);
}


void updateMotors2()
{
  String vals = input.substring(3, 32);
  byte dividerIndex = vals.indexOf(' ');   // ищем индекс разделителя
  String buf_1 = vals.substring(0, dividerIndex);    // создаём строку с первым числом
  String buf_2 = vals.substring(dividerIndex + 1);   // создаём строку со вторым числом
  int val_1 = buf_1.toInt();    // преобразуем во int
  int val_2 = buf_2.toInt();    // ...

  Serial.println("Parse data2: " + String(val_1) + " " + String(val_2));
  changeMotorsSpeed(val_1, val_1, val_2, val_2);
}


void updateMotors4()
{
  String vals = input.substring(3, 32);
  byte dividerIndex1 = vals.indexOf(' ');   // ищем индекс разделителя
  byte dividerIndex2 = vals.indexOf(' ', dividerIndex1+1);   // ищем индекс разделителя
  byte dividerIndex3 = vals.indexOf(' ', dividerIndex2+1);   // ищем индекс разделителя
  String buf_1 = vals.substring(0, dividerIndex1);    // создаём строку с первым числом
  String buf_2 = vals.substring(dividerIndex1+1, dividerIndex2);   // создаём строку со вторым числом
  String buf_3 = vals.substring(dividerIndex2+1, dividerIndex3);    // создаём строку с третьим числом
  String buf_4 = vals.substring(dividerIndex3 + 1);   // создаём строку с четвертым числом
  int val_1 = buf_1.toInt();    // преобразуем во int
  int val_2 = buf_2.toInt();    // ...
  int val_3 = buf_3.toInt();    // ...
  int val_4 = buf_4.toInt();    // ...

  Serial.println("Parse data4: " + String(val_1) + " " + String(val_2) + " " + String(val_3) + " " + String(val_4));
  changeMotorsSpeed(val_1, val_2, val_3, val_4);
}


void changeMotorsSpeed(int speedFR, int speedBR, int speedFL, int speedBL)
{
  motorFR.setSpeed(speedFR);
  motorBR.setSpeed(speedBR);
  motorFL.setSpeed(speedFL);
  motorBL.setSpeed(speedBL);
}