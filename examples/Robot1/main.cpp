#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

#include "matplotpp.h"

class MyPlot :public MatPlot
{
public:
    
    std::vector<std::pair<double, double>> plot1_;
    
    std::vector<std::pair<double, double>> plot2_;
    
    std::vector<std::pair<double, double>> plot3_;
    
    std::vector<std::pair<double, double>> plot4_;
    
    void DISPLAY()
    {
        plot(plot1_);
        plot(plot2_);
        plot(plot3_);
        plot(plot4_);
    }
    
}mp;

void display(){ mp.display(); }
void reshape(int w,int h){ mp.reshape(w,h); }

int drawplot(int argc,char* argv[])
{
    glutInit(&argc, argv);
    glutCreateWindow(40,40,800,800);
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
    glutMainLoop();
    return 0;
}

using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;
typedef Robot1::OrientationMeasurementModel<T> OrientationModel;

int main(int argc, char** argv)
{
    // Simulated (true) system state
    State x;
    x.setZero();
    
    // Control input
    Control u;
    // System
    SystemModel sys;
    
    // Measurement models
    // Set position landmarks at (-10, -10) and (30, 75)
    PositionModel pm(-10, -10, 30, 75);
    OrientationModel om;
    
    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Some filters for estimation
    // Pure predictor without measurement updates
    Kalman::ExtendedKalmanFilter<State> predictor;
    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    
    // Init filters with true system state
    predictor.init(x);
    ekf.init(x);
    ukf.init(x);
    
    // Standard-Deviation of noise added to all state vector components during state transition
    T systemNoise = 0.1;
    // Standard-Deviation of noise added to all measurement vector components in orientation measurements
    T orientationNoise = 0.025;
    // Standard-Deviation of noise added to all measurement vector components in distance measurements
    T distanceNoise = 0.25;
    
    // Simulate for 100 steps
    const size_t N = 100;
    for(size_t i = 1; i <= N; i++)
    {
        // Generate some control input
        u.v() = 1. + std::sin( T(2) * T(M_PI) / T(N) );
        u.dtheta() = std::sin( T(2) * T(M_PI) / T(N) ) * (1 - 2*(i > 50));
        
        // Simulate system
        x = sys.f(x, u);
        
        // Add noise: Our robot move is affected by noise (due to actuator failures)
        x.x() += systemNoise*noise(generator);
        x.y() += systemNoise*noise(generator);
        x.theta() += systemNoise*noise(generator);
        
        // Predict state for current time-step using the filters
        auto x_pred = predictor.predict(sys, u);
        auto x_ekf = ekf.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u);
        
        // Orientation measurement
        {
            // We can measure the orientation every 5th step
            OrientationMeasurement orientation = om.h(x);
            
            // Measurement is affected by noise as well
            orientation.theta() += orientationNoise * noise(generator);
            
            // Update EKF
            x_ekf = ekf.update(om, orientation);
            
            // Update UKF
            x_ukf = ukf.update(om, orientation);
        }
        
        // Position measurement
        {
            // We can measure the position every 10th step
            PositionMeasurement position = pm.h(x);
            
            // Measurement is affected by noise as well
            position.d1() += distanceNoise * noise(generator);
            position.d2() += distanceNoise * noise(generator);
            
            // Update EKF
            x_ekf = ekf.update(pm, position);
            
            // Update UKF
            x_ukf = ukf.update(pm, position);
        }
        
        mp.plot1_.emplace_back(std::make_pair(x.x(), x.y()));
        mp.plot2_.emplace_back(std::make_pair(x_pred.x(), x_pred.y()));
        mp.plot3_.emplace_back(std::make_pair(x_ekf.x(), x_ekf.y()));
        mp.plot4_.emplace_back(std::make_pair(x_ukf.x(), x_ukf.y()));
        
        // Print to stdout as csv format
        std::cout   << x.x() << "," << x.y() << "," << x.theta() << ","
                    << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta()  << ","
                    << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta()  << ","
                    << x_ukf.x() << "," << x_ukf.y() << "," << x_ukf.theta()
                    << std::endl;
    }
    
    drawplot(argc, argv);
    
    return 0;
}