#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <vector>

gz::msgs::Actuators actuatorMsg;
gz::transport::Node node;

std::vector<double> hoverSpeed = {700, 700, 700, 700}; // Hover speed
std::vector<double> moveForwardSpeed = {750, 750, 750, 750}; // Forward movement speed
std::vector<double> turnRightSpeed = {700, 750, 700, 750}; // Right turn speed
std::vector<double> stopSpeed = {0, 0, 0, 0}; // Stop motors

bool hasAscended = false;
bool hasTurned = false;
double distanceToWall = 1.0; // Distance threshold in meters
double forwardDistance = 5.0; // Distance to move forward after turning

void publishMotorSpeed(const std::vector<double>& speeds) {
    actuatorMsg.mutable_velocity()->Clear();
    for (const auto& speed : speeds) {
        actuatorMsg.add_velocity(speed);
    }
    auto publisher = node.Advertise<gz::msgs::Actuators>("/X3/gazebo/command/motor_speed");
    publisher.Publish(actuatorMsg);
}

void lidarCallback(const gz::msgs::LaserScan& msg) {
    double minRange = *std::min_element(msg.ranges().begin(), msg.ranges().end());

    if (!hasAscended) {
        std::cout << "Ascending to 2 meters...\n";
        publishMotorSpeed(hoverSpeed);
        // Assume the drone reaches 2 meters after a few seconds.
        std::this_thread::sleep_for(std::chrono::seconds(3));
        hasAscended = true;
    } else if (minRange > distanceToWall && !hasTurned) {
        std::cout << "Flying forward...\n";
        publishMotorSpeed(moveForwardSpeed);
    } else if (minRange <= distanceToWall && !hasTurned) {
        std::cout << "Wall detected! Turning right...\n";
        publishMotorSpeed(turnRightSpeed);
        // Simulate turning right for a short duration.
        std::this_thread::sleep_for(std::chrono::seconds(3));
        hasTurned = true;
    } else if (hasTurned) {
        std::cout << "Flying forward for 5 meters...\n";
        publishMotorSpeed(moveForwardSpeed);
        // Simulate moving forward for the required distance.
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "Mission Complete. Stopping drone...\n";
        publishMotorSpeed(stopSpeed);
        gz::transport::waitForShutdown();
    }
}

int main(int argc, char** argv) {
    std::cout << "Starting LiDAR control node...\n";

    std::string lidarTopic = "/quadcopter/lidar";

    if (!node.Subscribe(lidarTopic, lidarCallback)) {
        std::cerr << "Error subscribing to topic [" << lidarTopic << "]" << std::endl;
        return -1;
    }

    gz::transport::waitForShutdown();
    return 0;
}
