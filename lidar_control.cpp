#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

gz::msgs::Actuators actuatorMsg;
gz::transport::Node node;

std::vector<double> hoverSpeed = {700, 700, 700, 700}; // Hover speed
std::vector<double> moveForwardSpeed = {750, 750, 750, 750}; // Forward speed
std::vector<double> stopSpeed = {0, 0, 0, 0}; // Stop motors

double targetAltitude = 2.0; // Desired altitude in meters
bool isHovering = false;

void publishMotorSpeed(const std::vector<double>& speeds) {
    actuatorMsg.mutable_velocity()->Clear();
    for (const auto& speed : speeds) {
        actuatorMsg.add_velocity(speed);
    }
    static auto publisher = node.Advertise<gz::msgs::Actuators>("/X3/gazebo/command/motor_speed");
    publisher.Publish(actuatorMsg);
}

void gradualMotorSpeedAdjustment(const std::vector<double>& targetSpeeds, int steps = 50, int durationMs = 3000) {
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

void performFlightSequence() {
    // Ascend
    std::cout << "Ascending to target altitude...\n";
    gradualMotorSpeedAdjustment(hoverSpeed, 100, 5000);
    std::this_thread::sleep_for(std::chrono::seconds(5)); // Simulate ascent time
    std::cout << "Stabilized at hover.\n";

    // Forward flight
    std::cout << "Flying forward...\n";
    gradualMotorSpeedAdjustment(moveForwardSpeed, 100, 5000);
    std::this_thread::sleep_for(std::chrono::seconds(5)); // Simulate forward motion time
    std::cout << "Completed forward motion.\n";

    // Landing
    std::cout << "Landing...\n";
    gradualMotorSpeedAdjustment(stopSpeed, 100, 5000);
    std::cout << "Landed and motors stopped.\n";

    gz::transport::waitForShutdown();
}

int main(int argc, char** argv) {
    std::cout << "Starting simple flight sequence node...\n";

    performFlightSequence();

    return 0;
}
