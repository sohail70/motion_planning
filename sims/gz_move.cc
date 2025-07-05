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

class MoverPlugin : public System,
                    public ISystemConfigure,
                    public ISystemPreUpdate
{
public:
    MoverPlugin() = default;
    ~MoverPlugin() override = default;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &/*_eventMgr*/) override
    {
        // Parse required link name
        if (!_sdf->HasElement("link_name")) {
            gzerr << "MoverPlugin: Missing required parameter <link_name>\n";
            return;
        }
        std::string linkName = _sdf->Get<std::string>("link_name");

        // Ensure the plugin is attached to a model
        Model model(_entity);
        if (!model.Valid(_ecm)) {
            gzerr << "MoverPlugin must be attached to a model entity.\n";
            return;
        }

        // Retrieve the link entity
        Entity linkEntity = model.LinkByName(_ecm, linkName);
        if (linkEntity == kNullEntity) {
            gzerr << "Link [" << linkName << "] not found.\n";
            return;
        }
        this->linkEntity = linkEntity;

        // Store initial pose of the link
        Link link(linkEntity);
        auto worldPoseOpt = link.WorldPose(_ecm);
        if (worldPoseOpt.has_value()) {
            this->initialPose = worldPoseOpt.value();
        } else {
            gzerr << "Failed to get initial pose for link.\n";
            return;
        }

        // Parse optional parameters with defaults
        this->direction = _sdf->Get<math::Vector3d>(
            "direction", math::Vector3d::UnitX).first;
        this->amplitude = _sdf->Get<double>("amplitude", 1.0).first;
        this->speed = _sdf->Get<double>("speed", 0.2).first;

        gzmsg << "MoverPlugin configured for link '" << linkName
              << "': amplitude=" << amplitude
              << ", speed=" << speed
              << ", direction=" << direction << "\n";
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override
    {
        if (_info.paused) return;

        // Calculate elapsed time in seconds
        double time = std::chrono::duration<double>(_info.simTime).count();

        // Compute displacement using sine wave
        double displacement = amplitude * std::sin(speed * time);

        // Calculate target position
        math::Vector3d targetPos = initialPose.Pos() + direction * displacement;

        // Get the current pose of the link
        Link link(this->linkEntity);
        auto currentPoseOpt = link.WorldPose(_ecm);
        if (!currentPoseOpt.has_value()) {
            gzerr << "Failed to get current pose for link.\n";
            return;
        }
        math::Pose3d currentPose = currentPoseOpt.value();

        // Calculate the force required to move the link towards the target position
        math::Vector3d force = (targetPos - currentPose.Pos()) * 10.0; // Scaling factor

        // Apply the force to the link
        link.AddWorldForce(_ecm, force);

        // Optional: Add damping to prevent overshooting
        link.SetLinearVelocity(_ecm, link.WorldLinearVelocity(_ecm).value_or(math::Vector3d::Zero) * 0.9);
    }

private:
    Entity linkEntity{kNullEntity};  // Target link entity
    math::Pose3d initialPose;       // Initial position and orientation
    math::Vector3d direction;       // Direction of movement (world frame)
    double amplitude;               // Amplitude of movement (meters)
    double speed;                   // Angular frequency (radians/second)
};

// Register the plugin
GZ_ADD_PLUGIN(
    MoverPlugin,
    System,
    MoverPlugin::ISystemConfigure,
    MoverPlugin::ISystemPreUpdate
)

// Optional: Alias for ease of use
GZ_ADD_PLUGIN_ALIAS(MoverPlugin, "gz::sim::systems::MoverPlugin")
