#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/altimeter.pb.h>  // For altitude data
#include <gz/msgs/imu.pb.h>        // For IMU data
#include <gz/transport/Node.hh>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>

// Define actuator and transport
gz::msgs::Actuators actuatorMsg;
gz::transport::Node node;

// Target and control variables
std::vector<double> hoverSpeed = {700, 700, 700, 700}; // Initial guess for hover speed
double targetAltitude = 3.0;                          // Target altitude in meters
std::atomic<double> currentAltitude(0.0);             // Current altitude (updated via callback)
std::atomic<double> verticalAcceleration(0.0);        // Vertical acceleration from IMU
std::atomic<double> currentMotorSpeed(700.0);         // Current motor speed estimate for hovering
std::atomic<bool> stopHovering(false);                // Stop hovering flag

// Publish motor speeds
void publishMotorSpeed(const std::vector<double> &speeds) {
    actuatorMsg.mutable_velocity()->Clear();
    for (const auto &speed : speeds) {
        actuatorMsg.add_velocity(speed);
    }
    static auto publisher = node.Advertise<gz::msgs::Actuators>("/X3/gazebo/command/motor_speed");
    publisher.Publish(actuatorMsg);
}

// Altimeter callback
void altimeterCallback(const gz::msgs::Altimeter &msg) {
    currentAltitude.store(msg.vertical_position()); // Update altitude
}

// IMU callback (for future stabilization improvements)
void imuCallback(const gz::msgs::IMU &msg) {
    verticalAcceleration.store(msg.linear_acceleration().z()); // Update vertical acceleration
}

// Altitude control logic
void controlAltitude() {
    const double kp = 10.0; // Proportional gain
    const double kd = 20.0; // Derivative gain
    const double tolerance = 0.1; // Altitude tolerance in meters
    double previousAltitude = currentAltitude.load();

    while (!stopHovering.load()) {
        double altitudeError = targetAltitude - currentAltitude.load();
        double altitudeDerivative = currentAltitude.load() - previousAltitude;
        previousAltitude = currentAltitude.load();

        // Compute motor speed adjustment
        double speedAdjustment = kp * altitudeError - kd * altitudeDerivative;
        double newSpeed = currentMotorSpeed.load();
        if((currentMotorSpeed.load() + speedAdjustment < 700) && (currentMotorSpeed.load() + speedAdjustment > 500)){
            newSpeed = currentMotorSpeed.load() + speedAdjustment;
        }
        

        // Clamp motor speed to prevent over/under speeding
        newSpeed = std::max(0.0, std::min(1200.0, newSpeed));
        currentMotorSpeed.store(newSpeed);

        // Update motor speeds
        std::vector<double> motorSpeeds(4, currentMotorSpeed.load());
        publishMotorSpeed(motorSpeeds);

        // Print telemetry
        std::cout << "Altitude: " << currentAltitude.load() << " m, "
                  << "Velocity: " << currentMotorSpeed.load() << " RPM, "
                  << "Error: " << altitudeError << " m\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Print telemetry periodically
void telemetryPrinter() {
    while (!stopHovering.load()) {
        std::cout << "Current Altitude: " << currentAltitude.load() 
                  << " m | Vertical Acceleration: " << verticalAcceleration.load() << " m/s²\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Main function
int main(int argc, char **argv) {
    std::cout << "Starting drone control program...\n";

    // Subscribe to altimeter and IMU topics
    if (!node.Subscribe("/quadcopter/altimeter", altimeterCallback)) {
        std::cerr << "Error subscribing to /quadcopter/altimeter\n";
        return -1;
    }

    if (!node.Subscribe("/imu", imuCallback)) {
        std::cerr << "Error subscribing to /imu\n";
        return -1;
    }

    // Start telemetry and altitude control threads
    std::thread telemetryThread(telemetryPrinter);
    std::thread altitudeControlThread(controlAltitude);

    // Main thread just waits for exit signal
    std::cout << "Hovering indefinitely at " << targetAltitude << " meters.\n";
    std::cout << "Press Ctrl+C to stop the program.\n";

    // Join threads (this blocks indefinitely unless `stopHovering` is set to true)
    if (telemetryThread.joinable()) {
        telemetryThread.join();
    }
    if (altitudeControlThread.joinable()) {
        altitudeControlThread.join();
    }

    return 0;
}
