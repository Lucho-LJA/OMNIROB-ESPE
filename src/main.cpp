
//#define CONTROL_PID


//#define _INIT_MPU_ /**uncomment to calibrate MPU by first time**/
#ifndef _INIT_MPU_
  #include "config.h"
  #include "ESP32_Encoder.h"
  #include "ROS_CONFIG.h"
  #include "MOVE_OMNI_ROB.h"

  /*#include <string>
  #include "MPU6050_LECTURA.h"
*/
  //Variables de estado
  String Error_sistema ="NULL";
  
  //Variables MPU
  
  double mpu_acelX=0;
  double mpu_acelY=0;
  double mpu_acelZ=0;
  double mpu_gyroX=0;
  double mpu_gyroY=0;
  double mpu_gyroZ=0;

  
  String sms_1="";




  Encoder EMotor_1(MOTOR1_ENCODER_A,MOTOR1_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);
  Encoder EMotor_2(MOTOR2_ENCODER_A,MOTOR2_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);
  Encoder EMotor_3(MOTOR3_ENCODER_A,MOTOR3_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);
  Encoder EMotor_4(MOTOR4_ENCODER_A,MOTOR4_ENCODER_B,CHIHAI_E_PPR,WHEEL_DIAMETER);





#else
  #include "INIT_MPU6050.h"
#endif





String mensaje="";

unsigned long prev_time_board=0;
unsigned long time_board=0;
unsigned long time_motor3=0;
unsigned long time_motor4=0;





void setup()
{

  #ifndef _INIT_SERIAL_
    #define _INIT_SERIAL_
    Serial.begin(115200);
  #endif
  #ifdef _INIT_MPU_
    mpu_calibration();
  #else
    //Iniciar MPU6050
    //Iniciar_MPU6050();


    Serial.println();
    //Serial.print("Connecting to ");
    Serial.println(ssid);

    // Connect the ESP32 the the wifi AP
    WiFi.mode(WIFI_STA);
    WiFi.config(ip,gateway,subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      Serial.println("Connecting...");
      delay(500);
    }
    //Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    // Another way to get IP
    //Serial.print("IP = ");
    //Serial.println(nh.getHardware()->getLocalIP());

    // Start to be polite
    nh.advertise(omni_rpm);
    nh.advertise(planta_omni);
    nh.subscribe(omni_pwm);
    nh.subscribe(omni_mov);
    nh.subscribe(control_rpm_omni);
    nh.subscribe(accion_opc);
    stopCar();
    EMotor_1.reset();
    EMotor_2.reset();
    EMotor_3.reset();
    EMotor_4.reset();
  #endif
}


void loop()
{
  #ifndef _INIT_MPU_
    //Leer_mpu6050(&Error_sistema,&mpu_acelX, &mpu_acelY, &mpu_acelZ, &mpu_gyroX, &mpu_gyroY, &mpu_gyroZ);
    //Serial.println("Aceleracion");
    //Serial.println(mpu_acelX);
    //Serial.println(mpu_acelY);
    //Serial.println(mpu_acelZ);
    //Serial.println("Angulo");
    //Serial.println(mpu_gyroX);
    //Serial.println(mpu_gyroY);
    //Serial.println(mpu_gyroZ);
    //delay(100);

    
    
    Serial.println(mensaje);

    if (nh.connected()) {
      omni_rpm.publish( &str_msg );
      // Say hello
      #include "omni_move_case.h"

    } else {
    Serial.println("Not Connected");
    }

    RPM_motor[0]=-EMotor_1.getRPM();
    RPM_motor[1]=EMotor_2.getRPM();
    RPM_motor[2]=-EMotor_3.getRPM();
    RPM_motor[3]=EMotor_4.getRPM();
    pwm_msg.data=RPM_motor;

    nh.spinOnce();
    delay(100);


    
   #endif
}
