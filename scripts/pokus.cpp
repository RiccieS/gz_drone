#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "DroneMovement.hpp"

namespace gazebo
{
    class DroneController : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
        {
            DroneMovement drone(_model);

            // Example movements
            drone.MoveUp(5.0);        // Move up 5 meters
            drone.MoveRight(2.0);     // Move right 2 meters
            drone.Rotate(1.57);       // Rotate 90 degrees (in radians)
        }
    };

    // Register the plugin with Gazebo
    GZ_REGISTER_MODEL_PLUGIN(DroneController)
}
