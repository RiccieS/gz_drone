#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/altimeter.pb.h>  // For altitude data
#include <gz/msgs/imu.pb.h>        // For IMU data
#include <gz/transport/Node.hh>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>

// Define actuator and transport
gz::msgs::Actuators actuatorMsg;
gz::transport::Node node;

// Target and control variables
std::vector<double> hoverSpeed = {700, 700, 700, 700}; // Estimated hover motor speeds
std::vector<double> stopSpeed = {0, 0, 0, 0};         // Motors off
double targetAltitude = 3.0;                          // Target altitude in meters
std::atomic<double> currentAltitude(0.0);             // Current altitude (updated via callback)
std::atomic<double> verticalAcceleration(0.0);        // Vertical acceleration from IMU
std::atomic<bool> isHovering(false);                  // Hovering state flag

// Publish motor speeds
void publishMotorSpeed(const std::vector<double> &speeds) {
    actuatorMsg.mutable_velocity()->Clear();
    for (const auto &speed : speeds) {
        actuatorMsg.add_velocity(speed);
    }
    static auto publisher = node.Advertise<gz::msgs::Actuators>("/X3/gazebo/command/motor_speed");
    publisher.Publish(actuatorMsg);
}

// Adjust motor speeds gradually
void gradualMotorSpeedAdjustment(const std::vector<double> &targetSpeeds, int steps = 50, int durationMs = 3000) {
    std::vector<double> currentSpeeds(actuatorMsg.velocity().begin(), actuatorMsg.velocity().end());
    if (currentSpeeds.size() != targetSpeeds.size()) {
        currentSpeeds = std::vector<double>(targetSpeeds.size(), 0.0);
    }

    for (int i = 0; i <= steps; ++i) {
        for (size_t j = 0; j < currentSpeeds.size(); ++j) {
            currentSpeeds[j] += (targetSpeeds[j] - currentSpeeds[j]) / (steps - i + 1);
        }
        publishMotorSpeed(currentSpeeds);
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs / steps));
    }
}

// Altimeter callback
void altimeterCallback(const gz::msgs::Altimeter &msg) {
    currentAltitude.store(msg.vertical_position()); // Update altitude
}

// IMU callback (for future stabilization improvements)
void imuCallback(const gz::msgs::IMU &msg) {
    verticalAcceleration.store(msg.linear_acceleration().z()); // Update vertical acceleration
}

// Print telemetry
void telemetryPrinter() {
    while (!isHovering.load()) {
        std::cout << "Current Altitude: " << currentAltitude.load() << " meters\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Perform flight sequence
void performFlightSequence() {
    std::cout << "Starting flight sequence...\n";

    // Begin telemetry printing in a separate thread
    std::thread telemetryThread(telemetryPrinter);

    // Ascend to target altitude
    std::cout << "Ascending to target altitude...\n";
    gradualMotorSpeedAdjustment(hoverSpeed, 100, 5000);

    // Wait until target altitude is reached
    while (currentAltitude.load() < targetAltitude - 0.1) {
        std::cout << "Current Altitude: " << currentAltitude.load() << " meters\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Hover at target altitude
    std::cout << "Hovering at target altitude...\n";
    isHovering.store(true); // Set hovering state
    std::this_thread::sleep_for(std::chrono::seconds(10)); // Hover for 10 seconds

    // Descend and stop motors
    std::cout << "Landing...\n";
    gradualMotorSpeedAdjustment(stopSpeed, 100, 5000);
    std::cout << "Landed and motors stopped.\n";

    // Shutdown telemetry thread
    isHovering.store(false);
    if (telemetryThread.joinable()) {
        telemetryThread.join();
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

    // Perform flight sequence
    performFlightSequence();

    return 0;
}
