#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
#include <string>
#include <cmath>

// Function to move the drone
void MoveDrone(const std::string &modelName, double x, double y, double z, double roll, double pitch, double yaw) {
    // Create a transport node
    gz::transport::Node node;

    // Publisher to the model's pose topic
    std::string topic = "/model/" + modelName + "/pose";
    auto pub = node.Advertise<gz::msgs::Pose>(topic);

    if (!pub) {
        std::cerr << "Error: Could not create publisher on topic " << topic << std::endl;
        return;
    }

    // Create a Pose message
    gz::msgs::Pose poseMsg;
    auto *position = poseMsg.mutable_position();
    position->set_x(x);
    position->set_y(y);
    position->set_z(z);

    auto *orientation = poseMsg.mutable_orientation();

    // Convert roll, pitch, yaw to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    orientation->set_w(cr * cp * cy + sr * sp * sy);
    orientation->set_x(sr * cp * cy - cr * sp * sy);
    orientation->set_y(cr * sp * cy + sr * cp * sy);
    orientation->set_z(cr * cp * sy - sr * sp * cy);

    // Publish the message
    pub.Publish(poseMsg);
    std::cout << "Published new position for model: " << modelName << std::endl;
}

int main() {
    // Example usage
    std::string modelName = "drone_model";

    // Move the drone to (x=1.0, y=2.0, z=3.0) with no rotation
    MoveDrone(modelName, 1.0, 2.0, 3.0, 0, 0, 0);

    return 0;
}
