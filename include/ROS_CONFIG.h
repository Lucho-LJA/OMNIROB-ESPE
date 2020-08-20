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
    
    char movimiento='K';
    char tipo_func='0';
    int8_t opc=0;

    /* VARIABLES TO TOPICS */

    String opc_omni = "omni1";
    String pRPM=opc_omni+"/rpm";
    String pMPU=opc_omni+"/mpu";
    String pSET=opc_omni+"/setpoint";
    String pPKP=opc_omni+"/pid_kp";
    String pPKI=opc_omni+"/pid_ki";
    String pPKD=opc_omni+"/pid_kd";
    String pMOV=opc_omni+"/movimiento";




    void Lectura_SETPOINT( const std_msgs::Float32MultiArray& msg)
    {
        
        SET_motor[0]=msg.data[0];
        SET_motor[1]=msg.data[1];
        SET_motor[2]=msg.data[2];
        SET_motor[3]=msg.data[3];
    }
    void Lectura_KP_PID( const std_msgs::Float32MultiArray& msg)
    {
        
        kp_motor[0]=msg.data[0];
        kp_motor[1]=msg.data[1];
        kp_motor[2]=msg.data[2];
        kp_motor[3]=msg.data[3];
        
    }
    void Lectura_KI_PID( const std_msgs::Float32MultiArray& msg)
    {
        
        ki_motor[0]=msg.data[0];
        ki_motor[1]=msg.data[1];
        ki_motor[2]=msg.data[2];
        ki_motor[3]=msg.data[3];
        
    }
    void Lectura_KD_PID( const std_msgs::Float32MultiArray& msg)
    {
        
        kd_motor[0]=msg.data[0];
        kd_motor[1]=msg.data[1];
        kd_motor[2]=msg.data[2];
        kd_motor[3]=msg.data[3];
    }
    
    void Lectura_mov( const std_msgs::Char& msg)
    {
        
        movimiento=msg.data;

    }

    
    // Make a chatter publisher
    std_msgs::Float32MultiArray rpm_msg;
    std_msgs::Float32MultiArray mpu_msg;

    ros::Publisher omni_rpm(pRPM.c_str(), &rpm_msg);
    ros::Publisher omni_mpu(pMPU.c_str(), &mpu_msg);
    
    ros::Subscriber<std_msgs::Float32MultiArray> omni_setpoint(pSET.c_str(),&Lectura_SETPOINT);
    ros::Subscriber<std_msgs::Float32MultiArray> omni_kp(pPKP.c_str(),&Lectura_KP_PID);
    ros::Subscriber<std_msgs::Float32MultiArray> omni_ki(pPKI.c_str(),&Lectura_KI_PID);
    ros::Subscriber<std_msgs::Float32MultiArray> omni_kd(pPKD.c_str(),&Lectura_KD_PID);
    ros::Subscriber<std_msgs::Char> omni_mov(pMOV.c_str(),&Lectura_mov);


#endif