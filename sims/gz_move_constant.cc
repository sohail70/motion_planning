#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class MoverPluginC : public System,
                      public ISystemConfigure,
                      public ISystemPreUpdate
{
public:
    MoverPluginC() = default;
    ~MoverPluginC() override = default;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &/*_eventMgr*/) override
    {
        // Parse required link name
        if (!_sdf->HasElement("link_name")) {
            gzerr << "MoverPluginC: Missing required parameter <link_name>\n";
            return;
        }
        std::string linkName = _sdf->Get<std::string>("link_name");

        Model model(_entity);
        if (!model.Valid(_ecm)) {
            gzerr << "MoverPluginC must be attached to a model entity.\n";
            return;
        }

        this->linkEntity = model.LinkByName(_ecm, linkName);
        if (this->linkEntity == kNullEntity) {
            gzerr << "Link [" << linkName << "] not found.\n";
            return;
        }

        Link link(this->linkEntity);
        auto initialPoseOpt = link.WorldPose(_ecm);
        if (!initialPoseOpt.has_value()) {
            gzerr << "Failed to get initial pose for link.\n";
            return;
        }

        this->direction = _sdf->Get<math::Vector3d>(
            "direction", math::Vector3d::UnitX).first.Normalized();
        this->amplitude = _sdf->Get<double>("amplitude", 2.0).first;
        this->velocity = _sdf->Get<double>("speed", 1.0).first;
        this->turnaround_threshold = _sdf->Get<double>("turnaround_threshold", 0.05).first;

        this->startPoint = initialPoseOpt.value().Pos();
        this->endPoint = this->startPoint + this->direction * this->amplitude;
        this->currentTarget = this->endPoint;

        gzmsg << "MoverPluginC (Constant Velocity Mode) configured for link '" << linkName
              << "': amplitude=" << this->amplitude
              << ", velocity=" << this->velocity
              << ", direction=" << this->direction << "\n";
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override
    {
        if (_info.paused || this->linkEntity == kNullEntity) return;

        Link link(this->linkEntity);
        auto currentPoseOpt = link.WorldPose(_ecm);
        if (!currentPoseOpt.has_value()) {
            return;
        }

        math::Vector3d currentPos = currentPoseOpt.value().Pos();
        math::Vector3d vectorToTarget = this->currentTarget - currentPos;

        // Check if the link has reached the current target endpoint
        if (vectorToTarget.Length() < this->turnaround_threshold)
        {
            // Switch targets to move in the opposite direction
            if (this->currentTarget == this->endPoint) {
                this->currentTarget = this->startPoint;
            } else {
                this->currentTarget = this->endPoint;
            }
            
            //  Recalculate vectorToTarget immediately after the swap
            vectorToTarget = this->currentTarget - currentPos;
        }

        // Calculate the desired constant velocity vector to move towards the target
        math::Vector3d desiredVelocity = vectorToTarget.Normalized() * this->velocity;

        // Set the link's velocity directly for precise, constant-velocity motion
        link.SetLinearVelocity(_ecm, desiredVelocity);
    }

private:
    Entity linkEntity{kNullEntity};
    double amplitude;
    double velocity;
    double turnaround_threshold;
    math::Vector3d direction;
    math::Vector3d startPoint;
    math::Vector3d endPoint;
    math::Vector3d currentTarget;
};

// Register the plugin
GZ_ADD_PLUGIN(
    MoverPluginC,
    System,
    MoverPluginC::ISystemConfigure,
    MoverPluginC::ISystemPreUpdate
)

// Optional: Alias for ease of use
GZ_ADD_PLUGIN_ALIAS(MoverPluginC, "gz::sim::systems::MoverPluginC")
