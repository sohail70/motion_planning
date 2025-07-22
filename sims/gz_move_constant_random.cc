#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

// Include the C++ random library for generating random directions
#include <random>
// Include the algorithm library for std::clamp
#include <algorithm>

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

        // The 'direction' from SDF is now only the *initial* direction.
        this->direction = _sdf->Get<math::Vector3d>(
            "direction", math::Vector3d::UnitX).first.Normalized();
        this->amplitude = _sdf->Get<double>("amplitude", 2.0).first;
        this->velocity = _sdf->Get<double>("speed", 1.0).first;
        this->turnaround_threshold = _sdf->Get<double>("turnaround_threshold", 0.05).first;

        // Update boundaries to [-60, 60] and add an exclusion zone
        this->min_bounds = _sdf->Get<math::Vector2d>("min_bounds", math::Vector2d(-60.0, -60.0)).first;
        this->max_bounds = _sdf->Get<math::Vector2d>("max_bounds", math::Vector2d(60.0, 60.0)).first;
        this->exclusion_min = _sdf->Get<math::Vector2d>("exclusion_min", math::Vector2d(-60.0, -60.0)).first;
        this->exclusion_max = _sdf->Get<math::Vector2d>("exclusion_max", math::Vector2d(-45.0, -45.0)).first;

        // --- CHANGE: Add parameters for fixed random seed ---
        this->use_fixed_seed = _sdf->Get<bool>("use_fixed_seed", false).first;
        this->fixed_seed_value = _sdf->Get<int>("fixed_seed_value", 0).first;


        this->startPoint = initialPoseOpt.value().Pos();
        this->endPoint = this->startPoint + this->direction * this->amplitude;
        this->currentTarget = this->endPoint;

        // --- CHANGE: Initialize the random number generator based on the SDF parameter ---
        if (this->use_fixed_seed) {
            // Use the specified fixed seed for deterministic behavior
            this->gen = std::mt19937(this->fixed_seed_value);
        } else {
            // Use a random_device for non-deterministic, truly random behavior
            std::random_device rd;
            this->gen = std::mt19937(rd());
        }
        this->dis = std::uniform_real_distribution<>(-M_PI, M_PI);

        gzmsg << "MoverPluginC (Random Bounded Mode) configured for link '" << linkName
              << "': amplitude=" << this->amplitude
              << ", velocity=" << this->velocity
              << ", bounds=[" << this->min_bounds.X() << ", " << this->max_bounds.X() << "]"
              << ", exclusion_zone=[" << this->exclusion_min << " to " << this->exclusion_max << "]"
              << ", fixed_seed=" << (this->use_fixed_seed ? std::to_string(this->fixed_seed_value) : "false")
              << ", initial_direction=" << this->direction << "\n";
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
            // The current position becomes the starting point for the next movement segment.
            this->startPoint = currentPos;

            // Loop until a valid endpoint inside the boundaries and outside the exclusion zone is found
            int maxTries = 20; // Prevent an infinite loop in edge cases
            for (int i = 0; i < maxTries; ++i)
            {
                // Generate a new random angle to create a new 2D direction vector.
                double random_angle = this->dis(this->gen);
                this->direction.Set(cos(random_angle), sin(random_angle), 0);
                this->direction.Normalize();

                // Calculate a potential new endpoint.
                math::Vector3d potentialEndPoint = this->startPoint + this->direction * this->amplitude;

                // Check against both the main bounds and the exclusion zone
                bool in_bounds = potentialEndPoint.X() <= this->max_bounds.X() && potentialEndPoint.X() >= this->min_bounds.X() &&
                                 potentialEndPoint.Y() <= this->max_bounds.Y() && potentialEndPoint.Y() >= this->min_bounds.Y();

                bool in_exclusion_zone = potentialEndPoint.X() <= this->exclusion_max.X() && potentialEndPoint.X() >= this->exclusion_min.X() &&
                                         potentialEndPoint.Y() <= this->exclusion_max.Y() && potentialEndPoint.Y() >= this->exclusion_min.Y();


                if (in_bounds && !in_exclusion_zone)
                {
                    this->endPoint = potentialEndPoint;
                    break; // Found a valid point, exit the loop
                }

                // If after max tries we still don't have a valid point,
                // just move towards the center as a fallback to prevent getting stuck.
                if (i == maxTries - 1)
                {
                    //gzerr << "Could not find a valid random direction within bounds after "
                    //      << maxTries << " tries for link " << link.Name(_ecm).value_or("N/A")
                    //      << ". Moving towards center.\n";
                    math::Vector3d center(
                        (this->max_bounds.X() + this->min_bounds.X()) / 2.0,
                        (this->max_bounds.Y() + this->min_bounds.Y()) / 2.0,
                        currentPos.Z());
                    this->endPoint = center;
                }
            }
            
            // Set the new endpoint as the current target.
            this->currentTarget = this->endPoint;
            
            // Recalculate the vector to the new target for this update cycle.
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

    // Members for boundaries
    math::Vector2d min_bounds;
    math::Vector2d max_bounds;
    // Members for the exclusion zone
    math::Vector2d exclusion_min;
    math::Vector2d exclusion_max;

    // --- CHANGE: Add members for fixed seed logic ---
    bool use_fixed_seed;
    int fixed_seed_value;

    // Members for random number generation
    std::mt19937 gen; // Mersenne Twister random number generator
    std::uniform_real_distribution<> dis; // Distribution for random angles
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
