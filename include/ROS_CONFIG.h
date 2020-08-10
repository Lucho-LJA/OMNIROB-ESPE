#ifndef _ROS_CONFIG_
    #define _ROS_CONFIG_
    #include "config.h"
    #include "ros.h"
    #include "std_msgs/String.h"
    #include "std_msgs/Int8.h"
    #include "std_msgs/Int16.h"
    #include "std_msgs/Float32MultiArray.h"
    #include "std_msgs/Char.h"
    #include <string>

    const char* ssid     = ROUTER_SSID;
    const char* password = ROUTER_PASWORD;
    //Set the IP's configuration
    IPAddress ip(IP_ESP32);
    IPAddress gateway(IP_GATEWAY);
    IPAddress subnet(IP_SUBNET);

    // Set the rosserial socket server IP address
    IPAddress server(ROS_SERVER);
    // Set the rosserial socket server port
    const uint16_t serverPort = ROS_SERVER_PORT;

    ros::NodeHandle nh;
    
    
    char movimiento=' ';
    int8_t accion=0;
    int8_t opc=0;

    void Lectura_PWM( const std_msgs::Float32MultiArray& msg)
    {
        
        PWM_motor[0]=msg.data[0];
        PWM_motor[1]=msg.data[1];
        PWM_motor[2]=msg.data[2];
        PWM_motor[3]=msg.data[3];
    }
    
    void Lectura_mov( const std_msgs::Char& msg)
    {
        
        movimiento=msg.data;

    }
    
    void messageCb( const std_msgs::Int8& msg)
    {
        
        accion=msg.data;
        //accion=String(msg.data);
        //Serial.println(sms_1);
    }

    void messageRPM( const std_msgs::Int16& msg)
    {
        
        opc=msg.data;
        //accion=String(msg.data);
        //Serial.println(sms_1);
    }
    
    // Make a chatter publisher
    std_msgs::Float32MultiArray pwm_msg;
    std_msgs::String str_msg;

    //std_msgs::Int16 int16_msg;
    //std_msgs::String str_msg2;
    ros::Publisher omni_rpm("omni/rpm", &pwm_msg);
    ros::Publisher planta_omni("planta_omni1", &str_msg);
    
    ros::Subscriber<std_msgs::Float32MultiArray> omni_pwm("omni/pwm",&Lectura_PWM);
    ros::Subscriber<std_msgs::Char> omni_mov("omni/movimiento",&Lectura_mov);
    ros::Subscriber<std_msgs::Int8> accion_opc("accion_omni1",&messageCb);
    ros::Subscriber<std_msgs::Int16> control_rpm_omni("rpm_omni1",&messageRPM);


#endif