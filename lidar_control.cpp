#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/altimeter.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <algorithm>

// Define actuator and transport
gz::msgs::Actuators actuatorMsg;
gz::transport::Node node;

// Target and control variables
double targetAltitude = 3.0;                  // Target altitude in meters
std::atomic<double> currentAltitude(0.0);     // Current altitude (updated via callback)
std::atomic<double> currentVelocity(0.0);     // Current velocity estimate
std::atomic<double> currentMotorSpeed(700.0); // Current motor speed estimate
std::atomic<bool> stopHovering(false);        // Stop hovering flag
std::atomic<double> imuRoll(0.0);             // Stores the roll value
std::atomic<double> imuPitch(0.0);            // Stores the pitch value

// PID Controller parameters
const double Kp = 1.5;      // Proportional gain
const double Ki = 0.05;     // Integral gain
const double Kd = 2.0;      // Derivative gain
double integralError = 0.0; // Accumulated integral error

// Speed bounds for adaptive adjustment
double maxSpeed = 900.0; // Maximum motor speed
double minSpeed = 0.0;   // Minimum motor speed

// Publish motor speeds
void publishMotorSpeed(const std::vector<double> &speeds)
{
    actuatorMsg.mutable_velocity()->Clear();
    for (const auto &speed : speeds)
    {
        actuatorMsg.add_velocity(speed);
    }
    static auto publisher = node.Advertise<gz::msgs::Actuators>("/X3/gazebo/command/motor_speed");
    publisher.Publish(actuatorMsg);
}

// Altimeter callback
const double alpha = 0.8; // Smoothing factor (0.0-1.0)
std::atomic<double> smoothedAltitude(0.0);
void altimeterCallback(const gz::msgs::Altimeter &msg)
{
    double rawAltitude = msg.vertical_position();
    smoothedAltitude.store(alpha * rawAltitude + (1 - alpha) * smoothedAltitude.load());
    currentAltitude.store(smoothedAltitude.load());
}

// IMU callback (for velocity estimation)
void imuCallback(const gz::msgs::IMU &msg)
{
    // Assuming the IMU orientation is a quaternion
    auto orientation = msg.orientation();
    double qx = orientation.x();
    double qy = orientation.y();
    double qz = orientation.z();
    double qw = orientation.w();

    // Convert quaternion to Euler angles
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    imuRoll.store(std::atan2(sinr_cosp, cosr_cosp)); // Roll

    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        imuPitch.store(std::copysign(M_PI / 2, sinp)); // Pitch (clamped)
    else
        imuPitch.store(std::asin(sinp));

    // Update linear acceleration for velocity estimation
    currentVelocity.store(msg.linear_acceleration().z()); // Assuming Z is upward
}

// PID control loop for altitude
void controlAltitude()
{
    double previousError = 0.0;                       // Error in the previous loop
    double previousAltitude = currentAltitude.load(); // Store for derivative calculation

    while (!stopHovering.load())
    {
        double currentAlt = currentAltitude.load();
        double altitudeError = targetAltitude - currentAlt;

        // Calculate derivative (rate of change of altitude)
        double altitudeDerivative = (currentAlt - previousAltitude) / 0.1; // Derivative over 100ms
        previousAltitude = currentAlt;

        // Update integral error
        integralError += altitudeError * 0.1; // Integrate over 100ms

        // Compute PID output
        double speedAdjustment = (Kp * altitudeError) +
                                 (Ki * integralError) -
                                 (Kd * altitudeDerivative);

        // Roll and pitch corrections
        double rollCorrection = 0.05 * imuRoll.load(); // Example correction based on IMU
        double pitchCorrection = 0.05 * imuPitch.load();

        // Adjust motor speeds considering roll and pitch corrections
        double newSpeed = currentMotorSpeed.load() + speedAdjustment;
        newSpeed = std::clamp(newSpeed, minSpeed, maxSpeed); // Clamp to allowable range

        // Apply motor speed corrections
        std::vector<double> motorSpeeds = {
            newSpeed - rollCorrection + pitchCorrection, // Rotor 1
            newSpeed + rollCorrection + pitchCorrection, // Rotor 2
            newSpeed + rollCorrection - pitchCorrection, // Rotor 3
            newSpeed - rollCorrection - pitchCorrection  // Rotor 4
        };

        publishMotorSpeed(motorSpeeds);

        // Print telemetry
        std::cout << "Altitude: " << currentAlt << " m, "
                  << "Motor Speeds: [" << motorSpeeds[0] << ", " << motorSpeeds[1] << ", "
                  << motorSpeeds[2] << ", " << motorSpeeds[3] << "] RPM, "
                  << "Error: " << altitudeError << " m, "
                  << "Vertical Velocity: " << currentVelocity.load() << " m/s\n";

        // Sleep for loop interval
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Telemetry printer thread
void telemetryPrinter()
{
    while (!stopHovering.load())
    {
        std::cout << "Current Altitude: " << currentAltitude.load()
                  << " m | Vertical Velocity: " << currentVelocity.load() << " m/s\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Takeoff function to safely lift the drone to a starting altitude
void takeoff()
{
    std::cout << "Starting takeoff...\n";
    for (double speed = minSpeed; speed < maxSpeed; speed += 50)
    {
        std::vector<double> motorSpeeds(4, speed);
        publishMotorSpeed(motorSpeeds);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (currentAltitude.load() > 0.1)
        {
            std::cout << "Lift-off detected. Transitioning to hover control.\n";
            break;
        }
    }
}

int main(int argc, char **argv)
{
    std::cout << "Starting drone control program...\n";
    std::cout << "Hovering target altitude: " << targetAltitude << " meters.\n";

    // Subscribe to altimeter and IMU topics
    if (!node.Subscribe("/quadcopter/altimeter", altimeterCallback))
    {
        std::cerr << "Error subscribing to /quadcopter/altimeter\n";
        return -1;
    }

    if (!node.Subscribe("/imu", imuCallback))
    {
        std::cerr << "Error subscribing to /imu\n";
        return -1;
    }

    // Perform takeoff before enabling control threads
    takeoff();

    // Start telemetry and altitude control threads
    std::thread telemetryThread(telemetryPrinter);
    std::thread altitudeControlThread(controlAltitude);

    // Main thread just waits for exit signal
    std::cout << "Hovering indefinitely at " << targetAltitude << " meters.\n";
    std::cout << "Press Ctrl+C to stop the program.\n";

    // Join threads (this blocks indefinitely unless `stopHovering` is set to true)
    if (telemetryThread.joinable())
    {
        telemetryThread.join();
    }
    if (altitudeControlThread.joinable())
    {
        altitudeControlThread.join();
    }

    return 0;
}
