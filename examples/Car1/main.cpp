//
//  main.cpp
//  Kalman
//
//  Created by Sumeru Chatterjee on 1/13/16.
//  Copyright © 2016 Sumeru Chatterjee. All rights reserved.
//

// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include "Car1SystemModel.hpp"
#include "Car1MeasurementModel.hpp"

#include "../fast-cpp-csv-parser/csv.h"

using namespace KalmanExamples;

typedef float T;

typedef Car1::State<T> State;
typedef Car1::Control<T> Control;
typedef Car1::SystemModel<T> SystemModel;

typedef Car1::CarMeasurement<T> CarMeasurement;
typedef Car1::CarMeasurementModel<T> CarMeasurementModel;


int main(int argc, char** argv)
{
    io::CSVReader<25> in("sensordata.csv");
    
    in.read_header(io::ignore_extra_column, "date", "time", "millis", "ax", "ay","az","rollrate","pitchrate","yawrate","roll","pitch","yaw","speed","course","latitude", "longitude", "altitude", "pdop", "hdop", "vdop", "epe", "fix", "satellites_view", "satellites_used", "temp");
    
    std::string date; uint64_t time; double millis; double ax; double ay; double az; double rollrate; double pitchrate; double yawrate; double roll; double pitch; double yaw; float speed; float course; double latitude; double longitude; double altitude; float pdop; float hdop; float vdop; float epe; int fix; int satellites_view; int satellites_used; float temp;
    
    int counter = 0;
    
    double startMillis = 0;
    
    // Simulated (true) system state
    State x;
    x.setZero();
    
    // Control input
    Control u;
    // System
    SystemModel sys;

    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    
    //Measurement Models
    CarMeasurementModel cm(-10, -10, 30, 75);
    
    while(in.read_row(date, time, millis, ax, ay, az, rollrate, pitchrate, yawrate, roll, pitch, yaw, speed, course, latitude, longitude, altitude, pdop, hdop, vdop, epe, fix, satellites_view, satellites_used, temp))
    {
        if (startMillis == 0)
        {
            startMillis = millis;
        }
        
        double time = millis - startMillis;
        
        x = sys.f(x, u);
        
        auto x_ekf = ekf.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u);
        
        //measurement
        {
            // We can measure the orientation every 5th step
            CarMeasurement measurement = cm.h(x);
            
            // Update EKF
            x_ekf = ekf.update(cm, measurement);
            
            // Update UKF
            x_ukf = ukf.update(cm, measurement);
        }
        
        //mp.plotAx_.emplace_back(std::make_pair( time, ax));
        //mp.plotAy_.emplace_back(std::make_pair( time, ay));
        //mp.plotAz_.emplace_back(std::make_pair( time, az));
        //
        //mp.plotRoll_.emplace_back(std::make_pair(time, roll));
        //mp.plotPitch_.emplace_back(std::make_pair(time, pitch));
        //mp.plotYaw_.emplace_back(std::make_pair(time, yaw));
        //
        //mp.plotRollRate_.emplace_back(std::make_pair(time, rollrate));
        //mp.plotPitchRate_.emplace_back(std::make_pair(time, pitchrate));
        //mp.plotYawRate_.emplace_back(std::make_pair(time, yawrate));
        //
        //mp.plotCoordinates_.emplace_back(std::make_pair(latitude, longitude));

        //mp.plotPosition_.emplace_back(std::make_pair(x.x(), x.y()));
        //mp.plotEKFPosition_.emplace_back(std::make_pair(x_ekf.x(), x_ekf.y()));
        //mp.plotUKFPosition_.emplace_back(std::make_pair(x_ukf.x(), x_ukf.y()));

        std::cout << x.x() << "," << x.y() << "," << x.heading() << "," << x.velocity() << "," << x.yawrate()
            << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.heading() << "," << x_ekf.velocity() << "," << x_ekf.yawrate()
            << x_ukf.x() << "," << x_ukf.y() << "," << x_ukf.heading() << "," << x_ukf.velocity() << "," << x_ukf.yawrate()
            << std::endl;
        
        counter ++;
    }
    
    std::cout << "Total: " << counter << std::endl;
    
    return 0;
}
