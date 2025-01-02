#ifndef DRONEMOVEMENT_HPP
#define DRONEMOVEMENT_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

class DroneMovement
{
private:
    gazebo::physics::ModelPtr model; // Pointer to the drone model
    ignition::math::Pose3d currentPose; // Current pose of the drone

public:
    // Constructor
    DroneMovement(gazebo::physics::ModelPtr _model);

    // Movement functions
    void MoveUp(double altitude);
    void MoveDown(double altitude);
    void MoveLeft(double distance);
    void MoveRight(double distance);
    void Rotate(double angle); // Rotate around Z-axis

    // Utility function to update the drone's position
    void UpdatePosition(const ignition::math::Pose3d &newPose);
};

#endif // DRONEMOVEMENT_HPP
