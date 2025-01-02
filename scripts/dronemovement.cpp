#include "DroneMovement.hpp"

// Constructor: Initializes the model pointer and gets the current pose
DroneMovement::DroneMovement(gazebo::physics::ModelPtr _model)
    : model(_model), currentPose(_model->WorldPose())
{
}

// Move the drone up by the specified altitude
void DroneMovement::MoveUp(double altitude)
{
    currentPose.Pos().Z() += altitude;
    UpdatePosition(currentPose);
}

// Move the drone down by the specified altitude
void DroneMovement::MoveDown(double altitude)
{
    currentPose.Pos().Z() -= altitude;
    UpdatePosition(currentPose);
}

// Move the drone left (negative X direction)
void DroneMovement::MoveLeft(double distance)
{
    currentPose.Pos().X() -= distance;
    UpdatePosition(currentPose);
}

// Move the drone right (positive X direction)
void DroneMovement::MoveRight(double distance)
{
    currentPose.Pos().X() += distance;
    UpdatePosition(currentPose);
}

// Rotate the drone around the Z-axis by the specified angle (in radians)
void DroneMovement::Rotate(double angle)
{
    currentPose.Rot() = ignition::math::Quaterniond(0, 0, currentPose.Rot().Yaw() + angle);
    UpdatePosition(currentPose);
}

// Update the drone's position in the simulation
void DroneMovement::UpdatePosition(const ignition::math::Pose3d &newPose)
{
    model->SetWorldPose(newPose);
}
