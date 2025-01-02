#include <gz/sim/Model.hh>
//#include <gz/sim/ModelPlugin.hh>
#include <gz/sim/components/Link.hh>
#include <gz/math/PID.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
#include <iostream>

#include <gz/physics7/gz/physics.hh>
#include <gz/common5/gz/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <cmath>

namespace gazebo {
class QuadcopterHover : public ModelPlugin {
private:
    physics::ModelPtr model;
    physics::LinkPtr baseLink;
    event::ConnectionPtr updateConnection;
    common::Time lastUpdateTime;
    
    ignition::math::Vector3d hoverForce;
    ignition::math::Vector3d currentForce;

    double targetAltitude;
    double kp, ki, kd;
    double errorSum, lastError;

public:
    QuadcopterHover() : kp(10.0), ki(0.01), kd(2.0), targetAltitude(2.0) {
        errorSum = 0.0;
        lastError = 0.0;
    }

    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
        model = parent;
        baseLink = model->GetLink("base_link");

        if (!baseLink) {
            gzerr << "Base link not found! Plugin will not work.\n";
            return;
        }

        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&QuadcopterHover::OnUpdate, this));

        lastUpdateTime = model->GetWorld()->SimTime();
        std::cout << "QuadcopterHover Plugin Loaded!" << std::endl;
    }

    void OnUpdate() {
        common::Time currentTime = model->GetWorld()->SimTime();
        double dt = (currentTime - lastUpdateTime).Double();
        lastUpdateTime = currentTime;

        if (dt <= 0.0) return;

        // Get current altitude
        ignition::math::Pose3d pose = baseLink->WorldPose();
        double currentAltitude = pose.Pos().Z();

        // PID controller for altitude
        double error = targetAltitude - currentAltitude;
        errorSum += error * dt;
        double errorRate = (error - lastError) / dt;
        lastError = error;

        double controlForce = kp * error + ki * errorSum + kd * errorRate;

        // Apply force to hover (assuming quadcopter mass is 1.0 kg for simplicity)
        hoverForce = ignition::math::Vector3d(0, 0, controlForce);
        baseLink->AddForce(hoverForce);
    }
};

GZ_REGISTER_MODEL_PLUGIN(QuadcopterHover)
}
