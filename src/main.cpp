
//#define _INIT_MPU_ /**uncomment to calibrate MPU by first time**/
//#define_ALONE_ACT_
#ifndef _INIT_MPU_
  #include "config.h"
  #include "ESP32_Encoder.h"
  #include "ROS_CONFIG.h"
  #include "MOVE_OMNI_ROB.h"
  #include <string>
  #ifndef _ALONE_ACT_
    #include "MPU6050_LECTURA.h"
  #endif
  String ip_board=" ";
  //Variables MPU
  



#else
  #include "INIT_MPU6050.h"
#endif





unsigned long prev_time_board=0;
unsigned long time_board=0;





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
    #ifndef _ALONE_ACT_
      Iniciar_MPU6050();
    #endif


    // Connect the ESP32 the the wifi AP
    WiFi.mode(WIFI_STA);
    WiFi.config(ip,gateway,subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      Serial.println("Connecting...");
      delay(500);
    }
    ip_board=String(WiFi.localIP());
    str_msg1.data= ip_board.c_str();

    
    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
    
    nh.initNode();
    rpm_msg.data_length=4;
    mpu_msg.data_length=6;
    

    // Start to be polite
    nh.advertise(omni_rpm);
    nh.advertise(omni_mpu);
    nh.advertise(error_esp);
    nh.advertise(ip_esp);
    nh.subscribe(omni_pwm);
    nh.subscribe(omni_mov);
    nh.subscribe(tipo_mov);
    
    stopCar();
    EMotor_1.reset();
    EMotor_2.reset();
    EMotor_3.reset();
    EMotor_4.reset();
  #endif
  prev_time_board =millis();
}


void loop()
{
  time_board = millis();
  #ifndef _INIT_MPU_
 
        prev_time_board=time_board;
        if (nh.connected()) 
        {
        
          omni_rpm.publish( &rpm_msg );
          omni_mpu.publish( &mpu_msg );
          ip_esp.publish( &str_msg1 );
          // Say hello
          //Serial.println(movimiento);
          #include "omni_move_case.h"



        } else {
          Serial.println("Not Connected RASPBERRY");
          stopCar();
        }

        #ifndef _ALONE_ACT_
          Leer_mpu6050();
          mpu_msg.data= MPU_motor;
        #endif

        RPM_motor[0]=abs(EMotor_1.getRPM());
        RPM_motor[1]=abs(EMotor_2.getRPM());
        RPM_motor[2]=abs(EMotor_3.getRPM());
        RPM_motor[3]=abs(EMotor_4.getRPM());
        rpm_msg.data=RPM_motor;


        
        nh.spinOnce();
        delay(dt_board);


    
    
   #endif
}
