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

class CirclePlugin : public System,
                    public ISystemConfigure,
                    public ISystemPreUpdate
{
public:
    CirclePlugin() = default;
    ~CirclePlugin() override = default;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &/*_eventMgr*/) override
    {
        // Parse required link name
        if (!_sdf->HasElement("link_name")) {
            gzerr << "CirclePlugin: Missing required parameter <link_name>\n";
            return;
        }
        std::string linkName = _sdf->Get<std::string>("link_name");

        // Ensure the plugin is attached to a model
        Model model(_entity);
        if (!model.Valid(_ecm)) {
            gzerr << "CirclePlugin must be attached to a model entity.\n";
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
        this->radius = _sdf->Get<double>("radius", 100.0).first; // Radius of the circular path
        this->speed = _sdf->Get<double>("speed", 1.0).first;   // Angular speed (radians/second)

        gzmsg << "CirclePlugin configured for link '" << linkName
              << "': radius=" << radius
              << ", speed=" << speed << "\n";
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override
    {
        if (_info.paused) return;

        // Calculate elapsed time in seconds
        double time = std::chrono::duration<double>(_info.simTime).count();

        // Compute the angle based on time and speed
        double angle = speed * time;

        // Calculate target position in a circular path around the origin
        math::Vector3d targetPos(
            radius * std::cos(angle),  // x-coordinate
            radius * std::sin(angle) ,                      // y-coordinate (assuming motion in the x-z plane)
            0.0 // z-coordinate
        );

        // Get the current pose of the link
        Link link(this->linkEntity);
        auto currentPoseOpt = link.WorldPose(_ecm);
        if (!currentPoseOpt.has_value()) {
            gzerr << "Failed to get current pose for link.\n";
            return;
        }
        math::Pose3d currentPose = currentPoseOpt.value();

        // Calculate the force required to move the link towards the target position
        math::Vector3d force = (targetPos - currentPose.Pos()) * 200.0; // Scaling factor

        // Apply the force to the link
        link.AddWorldForce(_ecm, force);

        // Optional: Add damping to prevent overshooting
        link.SetLinearVelocity(_ecm, link.WorldLinearVelocity(_ecm).value_or(math::Vector3d::Zero) * 0.9);
    }

private:
    Entity linkEntity{kNullEntity};  // Target link entity
    math::Pose3d initialPose;       // Initial position and orientation
    double radius;                  // Radius of the circular path
    double speed;                   // Angular speed (radians/second)
};

// Register the plugin
GZ_ADD_PLUGIN(
    CirclePlugin,
    System,
    CirclePlugin::ISystemConfigure,
    CirclePlugin::ISystemPreUpdate
)

// Optional: Alias for ease of use
GZ_ADD_PLUGIN_ALIAS(CirclePlugin, "gz::sim::systems::CirclePlugin")
