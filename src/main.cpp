#include <iostream>
#include <cmath>
#include <thread>

//If you want to log some data for post-processing --> #define LOGGING
#define LOGGING

//If you want to calibrate the system, deactivate the motor --> #define CALIBRATION
//#define CALIBRATION

//If you want to see the runtimes and calculated PWM-Signal --> #define DEBUG
//#define DEBUG

#ifdef LOGGING
//includes for Logging
#include <chrono>
#include <fstream>
#include <vector>
#endif

//Raspberry-specific includes
#include <wiringPi.h>

//third party includes
#include"MPU6050.h"

//Controller Class Include
#include "controller.h"


#define PWM_PIN 26
#define DIRECTION_CCW_PIN 21   //CCW
#define DIRECTION_CW_PIN 22   //CW
#define PWM_MAX_VALUE 1024
#define RISE_UP_ANGLE 5
//Compensation of random Sensor-Kicks
#define SATURATION_GYRO 0.4

#define PI 3.1415

//****************************************
//Mit dem Anschlagwinkel den gemittelten Beschleunigungswinkel auf 0° ausrichten
#define ANGLE_COMPENSATION -1.2//-0.7//-0.955
#define ANGLE_FIXRATE 0.6


int main(){

    #ifdef LOGGING
    //Datenlogging
    std::vector<float> datapoint;
    std::vector<std::vector<float>> LogData;
    std::ofstream myFile;
    #endif

    //Create objects
    controller MyController(1024,-1024,190,620,0.7);
    MPU6050 device(0x68);

    //Calibration of MPU6050 -- only for installation
    // device.getOffsets(&ax_off,&ay_off, &az_off, &gr_off, &gp_off, &gy_off);

    //Pin-Layout wPI from gpio readall overview
    wiringPiSetup();    
    pinMode(PWM_PIN,PWM_OUTPUT);
    pinMode(DIRECTION_CCW_PIN,OUTPUT);
    pinMode(DIRECTION_CW_PIN,OUTPUT); 
    digitalWrite(DIRECTION_CCW_PIN,LOW);
    digitalWrite(DIRECTION_CW_PIN,LOW);

    //Start with Rise-Up? true=yes false=no
    bool riseUp=false;

    float ax, ay, az,gr,gp,gy;
    float manipulatedVariable=0;
    float deltaAngle=20.0;
    float dithering_increment=ANGLE_FIXRATE;
    float AngleACC=0.0;
    float AngleACCDegree=0.0;
    float dt=0.015; // Just for the first step!
    float target_angle=0.0;
    float timer=0.0;

    float filteredAngle=std::numeric_limits<double>::quiet_NaN();
    float GyroAngle=std::numeric_limits<double>::quiet_NaN();
    float deltaYaw=0.0;

    
    //Variables for logging
    int numberLogs=13;
    float Plog=0.0;
    float Ilog=0.0;
    float Dlog=0.0;
    float dtLog=0.0;
    

    std::cout<<"Please arrange Pendulum vertically! You've got five Seconds!"<<std::endl;
    sleep(5); //Wait for the MPU6050 to stabilize 
    
    
    std::cout<<"Start"<<std::endl;
    
    #ifdef LOGGING
    std::chrono::steady_clock::time_point runClock_begin=std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point runClock_dataPoint;
    #endif

    //Initialize clocks
    std::chrono::steady_clock::time_point dt_end;
    std::chrono::steady_clock::time_point dt_begin=std::chrono::steady_clock::now();

    for (int i=0;i<3000;i++) {
        

        //Angle Compensation
        filteredAngle-=ANGLE_COMPENSATION;
        
        std::chrono::steady_clock::time_point measure_begin=std::chrono::steady_clock::now();

        std::thread GetAccData(&MPU6050::getAccel,&device, &ax,&ay,&az);
        std::thread GetGyroData(&MPU6050::getGyro,&device,&gr,&gp,&gy);
        GetAccData.join();
        GetGyroData.join();

        std::chrono::steady_clock::time_point measure_end=std::chrono::steady_clock::now();
        auto diffMeasure=std::chrono::duration_cast<std::chrono::milliseconds>(measure_end-measure_begin);

        #ifdef DEBUG
        std::cout<<"measure-time: "<<diffMeasure.count()<<" ms"<<std::endl;
        #endif

        AngleACC=atan2(ay,ax);
        AngleACCDegree=AngleACC*(180/PI);

        if(std::isnan(GyroAngle)) GyroAngle=AngleACCDegree;
        else GyroAngle-=gy*dt;

        //Saturation of Gyroscope
        deltaYaw=gy*dt;
        if(deltaYaw>SATURATION_GYRO)deltaYaw=SATURATION_GYRO;
        else if(deltaYaw<-SATURATION_GYRO)deltaYaw=-SATURATION_GYRO;

        //Complementary Filter
        if (std::isnan(filteredAngle))   filteredAngle=AngleACCDegree;
        else filteredAngle=0.99*(filteredAngle-deltaYaw)+0.01*AngleACCDegree;
        
        //Angle Compensation
        filteredAngle+=ANGLE_COMPENSATION;
        AngleACCDegree+=ANGLE_COMPENSATION;

        #ifdef LOGGING
        //*********************LOGGING*******************
        runClock_dataPoint=std::chrono::steady_clock::now();
        std::chrono::duration<double> diff_run =runClock_dataPoint-runClock_begin;

        timer=diff_run.count();
        datapoint.push_back(timer);
        datapoint.push_back(AngleACCDegree);
        datapoint.push_back(filteredAngle);
        //***********************************************
        #endif

        if (riseUp){

            if(filteredAngle<0){
                if(timer<0.5){
                    digitalWrite(DIRECTION_CCW_PIN,HIGH);
                    digitalWrite(DIRECTION_CW_PIN,LOW);
                    pwmWrite(PWM_PIN,PWM_MAX_VALUE);
                }
                //TODO Zeitabhängig
                if(timer>2.5){
                    digitalWrite(DIRECTION_CCW_PIN,LOW);
                    digitalWrite(DIRECTION_CW_PIN,HIGH);
                    pwmWrite(PWM_PIN,PWM_MAX_VALUE);
                
                    if(abs(filteredAngle)<RISE_UP_ANGLE) {
                        riseUp=false;
                    }
                }

            }

            if(filteredAngle>0){
                if(timer<0.5){
                    digitalWrite(DIRECTION_CCW_PIN,LOW);
                    digitalWrite(DIRECTION_CW_PIN,HIGH);
                    pwmWrite(PWM_PIN,PWM_MAX_VALUE);
                }
                
                if(timer>2.5){
                    digitalWrite(DIRECTION_CCW_PIN,HIGH);
                    digitalWrite(DIRECTION_CW_PIN,LOW);
                    pwmWrite(PWM_PIN,PWM_MAX_VALUE);

                    if(abs(filteredAngle)<RISE_UP_ANGLE){ 
                        riseUp=false;
                    }
                }

            }

        }

        if(!riseUp){

            //Dithering
            if (filteredAngle<target_angle) target_angle+= dithering_increment*dt;
            else target_angle-=dithering_increment*dt;

            #ifdef LOGGING
            datapoint.push_back(target_angle);
            #endif

            if (filteredAngle >deltaAngle || filteredAngle<-deltaAngle){
                std::cout<<"Programmabbruch! Winkel zu groß!"<<std::endl;
                break;
            }
            //Calculate the PWM-Signal to stabilize Pendulum
            dt_end=std::chrono::steady_clock::now();
            std::chrono::duration<double> diff =dt_end-dt_begin;
            dt=diff.count();
            #ifdef DEBUG
            std::cout<<"Step Time:"<<dt<<std::endl;
            #endif
            std::thread control(&controller::do_step,&MyController,std::ref(target_angle),std::ref(filteredAngle),std::ref(dt),std::ref(Plog),std::ref(Ilog),std::ref(Dlog),std::ref(dtLog),std::ref(manipulatedVariable));
            control.join();
            dt_begin=std::chrono::steady_clock::now();

            #ifdef DEBUG
            std::cout<<"PWM: "<<manipulatedVariable<<std::endl;
            #endif

            #ifndef CALIBRATION
            //directional decision
            if (manipulatedVariable>0) {
                digitalWrite(DIRECTION_CCW_PIN,LOW);
                digitalWrite(DIRECTION_CW_PIN,HIGH);
                pwmWrite(PWM_PIN,abs(manipulatedVariable));
            }
            else if (manipulatedVariable<0){
                digitalWrite(DIRECTION_CW_PIN,LOW);
                digitalWrite(DIRECTION_CCW_PIN,HIGH);
                pwmWrite(PWM_PIN,abs(manipulatedVariable));
            }
            #endif
        }
        
        #ifdef LOGGING
        //***************LOGGING*****************
        datapoint.push_back(Plog);
        datapoint.push_back(Ilog);
        datapoint.push_back(Dlog);
        datapoint.push_back(manipulatedVariable);
        datapoint.push_back(gy*dt);
        datapoint.push_back(gy);

        datapoint.push_back(deltaYaw);
        datapoint.push_back(dt);
        datapoint.push_back(GyroAngle+ANGLE_COMPENSATION);
        LogData.push_back(datapoint);
        datapoint.clear();
        //***************************************
        #endif
    } 

    #ifdef LOGGING
    //***************LOGGING*****************

    myFile.open("../LogData.csv",std::fstream::out);
    for(int i=0;i<LogData.size();i++){
        for(int j=0;j<numberLogs;j++){
        
            if (j==(numberLogs-1)) myFile<<LogData[i][j];
            else myFile<<LogData[i][j]<<",";
        }
        myFile<<"\n";
    }
    myFile.close();
    //***************************************
    #endif

    //Shutting down
    digitalWrite(DIRECTION_CCW_PIN,LOW);
    digitalWrite(DIRECTION_CW_PIN,LOW);
    pwmWrite(PWM_PIN,0);
    std::cout<<"Finished! "<<std::endl;
    return 0;
}