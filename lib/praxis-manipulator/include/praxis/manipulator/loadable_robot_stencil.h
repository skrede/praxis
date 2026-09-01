#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_LOADABLE_ROBOT_STENCIL_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_LOADABLE_ROBOT_STENCIL_H

#include "praxis/manipulator/arm_snapshot.h"

#include "praxis/scene/stencil.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/slots.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include "praxis/scheduler/strand.h"

#include <threepp/threepp.hpp>

#include <threepp/objects/Robot.hpp>

#include <Eigen/Core>

#include <span>
#include <array>
#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

// What the flange can carry. An attachment is an object drawn at the flange, placed by the flange's
// own pose composed with the offset it was installed under; that one rule carries every member of
// this set. The set is closed at what the flange holds, so an object standing at another link or at
// a pose in the world is not one of these.
enum class flange_attachment : std::uint8_t
{
    tool,
    frame_marker
};

// Written out beside the enumeration, so that the set and the count of it cannot disagree.
inline constexpr std::size_t flange_attachment_count = 2;

// Which of the two Jacobians every drawing taken from one is taken from. A space Jacobian's columns
// are twists expressed in the space frame and a body Jacobian's are expressed in the tool frame, so
// the same configuration gives two different matrices. Lynch & Park, Modern Robotics, section 5.1.
// The set is closed at the two the arm publishes.
enum class jacobian_frame : std::uint8_t
{
    space,
    body
};

// Which of the two reciprocal readings of one Jacobian an ellipsoid is drawn under: what the tool
// can be moved at per unit joint rate, or what it can bear per unit joint effort. Lynch & Park,
// Modern Robotics, section 5.4. The set is closed at the two, because they are the two readings one
// decomposition gives.
enum class ellipsoid_view : std::uint8_t
{
    velocity,
    force
};

// Which three rows of a six-row Jacobian a quantity belongs to: the top three or the bottom three.
// The set is closed at the two, because there is no third block.
enum class jacobian_block : std::uint8_t
{
    angular,
    linear
};

// Written out beside the enumeration, so that the set and the count of it cannot disagree.
inline constexpr std::size_t jacobian_block_count = 2;

// A coordinate triad drawn at the frame it is attached to, proportioned to how far the rendered
// arm's bounding box is across so that one call serves machines of different size. An arm carrying
// no drawn geometry has no extent to take a proportion of, and the stand-in praxis uses there is
// stated rather than derived.
std::shared_ptr<threepp::Object3D> make_flange_marker(threepp::Object3D &arm);

// The two models an arm carries beside the robot itself. A null tool leaves the flange carrying
// nothing under the tool key until one is installed; a null world reference is no world object at
// all rather than an empty one.
struct attached_models
{
    std::shared_ptr<threepp::Object3D> tool  = nullptr;
    std::shared_ptr<threepp::Object3D> world = nullptr;
};

// The rendered arm's single owner. The node mirrors the configuration the arm publishes and is
// written nowhere else in the tree, which is what lets the state it mirrors live on another strand.
class loadable_robot_stencil : public scene::stencil
{
    // What one key of the flange holds: the object carried there, and the offset from the flange it
    // is carried at.
    struct carried
    {
        std::shared_ptr<threepp::Object3D> object;
        threepp::Matrix4 offset;
    };

    // A path standing in the scene: the name it is carried under and the line drawn through its
    // points. The line is never null, because a run too short to draw is declined before it is ever
    // stored.
    struct drawn_path
    {
        std::string name;
        std::shared_ptr<threepp::Object3D> object;
    };

    // One configuration standing in the scene as a chain figure: the group its items hang under,
    // and the segments and joint marks that make it up.
    struct drawn_figure
    {
        std::shared_ptr<threepp::Object3D> object;
        std::vector<std::shared_ptr<threepp::Object3D>> segments;
        std::vector<std::shared_ptr<threepp::Object3D>> marks;
    };

    // One manipulability ellipsoid standing in the scene: the body, and the two lines per principal
    // axis saying which way a cut axis continues. Line 2*axis runs along the axis and line 2*axis+1
    // runs against it. Every handle is null until the drawing is told.
    struct drawn_ellipsoid
    {
        std::shared_ptr<threepp::Object3D> body;
        std::array<std::shared_ptr<threepp::Object3D>, 6> lines;
    };

    // One part of one Jacobian column standing in the scene as an arrow: the group the arrow hangs
    // under and the two pieces its placement stretches and stands. Every handle is null until the
    // drawing is told.
    struct drawn_column
    {
        std::shared_ptr<threepp::Object3D> object;
        std::shared_ptr<threepp::Object3D> shaft;
        std::shared_ptr<threepp::Object3D> tip;
    };

public:
    loadable_robot_stencil(std::shared_ptr<threepp::Robot> robot_object, attached_models attached, threepp::Scene &parent, scheduler::strand render, arm_reader seen,
                           rigid_motion::screw_ops screw, rigid_motion::screw_slot_set inert);
    ~loadable_robot_stencil() override = default;

    expected<void, refusal> initialize() override;
    void tear_down() override;

    void render() const override;

    // Installing over an occupied key detaches what stood there. An attachment installed without an
    // offset is carried at the flange itself.
    void set_flange_attachment(flange_attachment which, std::shared_ptr<threepp::Object3D> attached);
    void set_flange_attachment(flange_attachment which, std::shared_ptr<threepp::Object3D> attached, threepp::Matrix4 offset);
    void set_flange_attachment_offset(flange_attachment which, threepp::Matrix4 offset);

    void clear_flange_attachment(flange_attachment which);

    void set_world_object(std::shared_ptr<threepp::Object3D> world_object);
    void clear_world_object();

    // The axes are expressed in the frame of the model's root link, which is also the frame the
    // rendered robot is expressed in, and their count is the rendered arm's joint count.
    expected<void, refusal> set_joint_screws(const transform &home, std::span<const screw_axis> space_screws);
    void clear_joint_screws();

    // Whether a chain through the joint origins stands to be shown at all. It is folded from the
    // screws the stencil was told, so an arm told none carries no chain and nothing a switch over
    // one says reaches anything.
    bool holds_chain() const;

    // Which screw slots the composition left at their defaults. A slot carrying no refusal channel
    // answers a value whatever it is handed, so a drawing folded through an unbound one stands where
    // the arm is not; this is how anything built over the stencil learns that without asking the
    // composition again.
    rigid_motion::screw_slot_set inert_screw_slots() const
    {
        return m_inert;
    }

    void set_decoration_reach(double metres);

    // How far each drawn axis reaches either way of its anchor, in metres.
    double decoration_reach() const;

    // The drawn length one unit of an ellipsoid's semi-axis takes, one multiplier per block: an
    // angular semi-axis is per unit joint rate and a linear one is metres per unit joint rate, so one
    // multiplier cannot serve both.
    void set_ellipsoid_scale(jacobian_block which, double drawn_metres_per_unit);
    double ellipsoid_scale(jacobian_block which) const;

    // Where a force ellipsoid's body is cut flat, as a multiple of that block's own ellipsoid scale:
    // a block's cut in drawn metres is that multiple times that block's scale, so under the force
    // reading an axis is cut where its singular value falls below the multiple's reciprocal. Whether
    // the cut is taken at all is a separate question.
    void set_force_cap_ratio(double of_the_ellipsoid_scale);
    double force_cap_ratio() const;
    void set_force_capped(bool capped);
    bool force_capped() const;

    // The poses are expressed in the space frame and the drawing runs through their positions in the
    // order they arrive. A name already drawn carries the new run in place of the old and keeps the
    // switch it was last shown or hidden by, and a run of fewer than two poses is declined rather
    // than drawn as nothing.
    expected<void, refusal> set_pose_path(std::string_view named, std::span<const transform> through);
    expected<void, refusal> set_pose_path(std::string_view named, std::span<const transform> through, threepp::Color tone);

    void clear_pose_path(std::string_view named);
    void clear_pose_paths();

    // Each configuration is folded to joint origins against the screws the stencil was told and
    // drawn as the chain figure that fold gives, in a tone of its own, so a configuration the arm
    // is not at stands beside the arm in the shape of the arm's own chain. The figures already
    // standing are replaced whole, and no configuration at all leaves none. A configuration whose
    // width is not the joint count, or one the fold refuses, is declined by name and leaves the
    // figures already standing where they were.
    expected<void, refusal> set_solution_figures(std::span<const joint_vector> at);
    void clear_solution_figures();

    // Which Jacobian everything the stencil draws from a Jacobian is drawn from. One field governs
    // them all, so no two of those drawings can be standing for different matrices. Lynch & Park,
    // Modern Robotics, section 5.1.
    void set_jacobian_frame(jacobian_frame taken);
    jacobian_frame jacobian_frame_shown() const;

    // Both manipulability ellipsoids stand at the tool frame under the reading told here, taken from
    // what the arm publishes and placed again every frame. Telling raises whichever of the two is not
    // already standing; telling again with another reading changes the reading and keeps whatever the
    // two switches last said. Lynch & Park, Modern Robotics, section 5.4.
    void set_manipulability_ellipsoids(ellipsoid_view read);
    void clear_manipulability_ellipsoids();

    // The columns of whichever Jacobian is shown, each standing as two arrows from one anchor: the
    // column's angular part and its linear part. Telling a count raises two arrows per column in
    // place of whatever stood before; a count of zero is declined by name and leaves the arrows
    // already standing where they were. Lynch & Park, Modern Robotics, section 5.1.
    expected<void, refusal> set_jacobian_columns(std::size_t columns);
    void clear_jacobian_columns();

    // The drawn length one unit of a column part takes, one multiplier per block: an angular part is
    // per unit joint rate and a linear one is metres per unit joint rate, so one multiplier cannot
    // serve both.
    void set_column_scale(jacobian_block which, double drawn_metres_per_unit);
    double column_scale(jacobian_block which) const;

    // The arm's meshes, its screw axes, the chain through its joint origins, the paths it was told,
    // the figures of the configurations it was told, its angular manipulability ellipsoid, its
    // linear one and the columns of the Jacobian it is showing are eight drawings of one arm, and
    // each of these shows or hides its own and reaches none of the others; a path is shown or hidden
    // by name, one at a time, and a name no path stands under is declined rather than quietly
    // ignored. None of them decides when it should be on: that is the composition's to say. Eight
    // drawings, eight switches.
    void set_meshes_shown(bool shown);
    void set_decoration_shown(bool shown);
    void set_chain_shown(bool shown);
    void set_solution_figures_shown(bool shown);
    void set_angular_ellipsoid_shown(bool shown);
    void set_linear_ellipsoid_shown(bool shown);
    void set_jacobian_columns_shown(bool shown);
    expected<void, refusal> set_pose_path_shown(std::string_view named, bool shown);

    // Which joint the drawing tells apart from the rest, counted from zero: its point, the segment
    // leading to it and its screw axis take a tone of their own. An index the screws the stencil was
    // told do not reach is declined rather than clamped, and a stencil told nothing tells nothing
    // apart.
    expected<void, refusal> set_selected_joint(std::size_t joint);
    void clear_selected_joint();

    std::optional<std::size_t> selected_joint() const
    {
        return m_selected;
    }

    // The name each joint's drawn axis is carried under, the name the chain through the joint
    // origins hangs under, the names its segments and its joint marks are carried under, the name a
    // path told under some name of the caller's stands under, and the names the figure of a told
    // configuration and its own segments and joint marks stand under. The counted ones start at one.
    static std::string joint_axis_name(std::size_t joint);
    static std::string chain_name();
    static std::string chain_segment_name(std::size_t segment);
    static std::string joint_mark_name(std::size_t joint);
    static std::string pose_path_name(std::string_view named);
    static std::string solution_figure_name(std::size_t figure);
    static std::string solution_segment_name(std::size_t figure, std::size_t segment);
    static std::string solution_mark_name(std::size_t figure, std::size_t joint);

    // The name one manipulability ellipsoid's body stands under, and the name one of its continuation
    // lines stands under. The axis is counted from one and the direction is the axis's own way where
    // it is forward and its negation where it is not.
    static std::string manipulability_ellipsoid_name(jacobian_block which);
    static std::string ellipsoid_continuation_name(jacobian_block which, std::size_t axis, bool forward);

    // The name one part of one Jacobian column's arrow stands under. The column is counted from one.
    static std::string jacobian_column_name(std::size_t column, jacobian_block part);

    std::shared_ptr<threepp::Object3D> attached_at(flange_attachment which) const;
    std::shared_ptr<threepp::Object3D> world_object() const;

    threepp::Robot &robot();
    const threepp::Robot &robot() const;

private:
    arm_reader m_seen;
    threepp::Scene &m_scene;
    scheduler::strand m_render;
    rigid_motion::screw_ops m_screw;
    rigid_motion::screw_slot_set m_inert;
    // A fold through an unbound slot is reported once for as long as that condition stands, so a
    // placement running every frame says it once rather than once a frame.
    mutable bool m_reported_unbound;
    double m_reach;
    std::optional<std::size_t> m_selected;
    transform m_home;
    std::vector<screw_axis> m_screws;
    std::shared_ptr<threepp::Material> m_axis_tone;
    std::shared_ptr<threepp::Material> m_axis_told;
    std::shared_ptr<threepp::Material> m_chain_tone;
    std::shared_ptr<threepp::Material> m_chain_told;
    std::shared_ptr<threepp::Material> m_solution_tone;
    std::shared_ptr<threepp::Robot> m_robot;
    std::array<carried, flange_attachment_count> m_attached;
    std::shared_ptr<threepp::Object3D> m_axes;
    std::shared_ptr<threepp::Object3D> m_chain;
    std::shared_ptr<threepp::Object3D> m_figure;
    std::shared_ptr<threepp::Object3D> m_paths;
    std::shared_ptr<threepp::Object3D> m_solutions;
    std::array<std::shared_ptr<threepp::Object3D>, jacobian_block_count> m_ellipsoid_groups;
    std::shared_ptr<threepp::Object3D> m_columns;
    std::shared_ptr<threepp::Object3D> m_decoration;
    std::shared_ptr<threepp::Object3D> m_world_object;
    std::vector<drawn_path> m_path_lines;
    std::vector<drawn_figure> m_solution_figures;
    std::vector<std::shared_ptr<threepp::Object3D>> m_drawn;
    std::vector<std::shared_ptr<threepp::Object3D>> m_segments;
    std::vector<std::shared_ptr<threepp::Object3D>> m_marks;
    std::array<drawn_ellipsoid, jacobian_block_count> m_ellipsoids;
    // One entry per column of the Jacobian the stencil was told the width of, each holding the
    // arrow of that column's angular part and the arrow of its linear part.
    std::vector<std::array<drawn_column, jacobian_block_count>> m_column_arrows;
    // Drawn metres per unit semi-axis, one per block.
    std::array<double, jacobian_block_count> m_ellipsoid_scale;
    // Drawn metres per unit column part, one per block.
    std::array<double, jacobian_block_count> m_column_scale;
    // A multiple of the block's own ellipsoid scale.
    double m_force_cap_ratio;
    bool m_force_capped;
    jacobian_frame m_frame;
    ellipsoid_view m_view;
    // One material per step of the condition-number ramp, built once, so a placement running every
    // frame assigns a material rather than constructing one.
    std::vector<std::shared_ptr<threepp::Material>> m_ellipsoid_solid;
    std::vector<std::shared_ptr<threepp::Material>> m_ellipsoid_wire;
    std::vector<std::shared_ptr<threepp::Material>> m_continuation_tone;
    // One material per block, built once, for the same reason the ramp is.
    std::array<std::shared_ptr<threepp::Material>, jacobian_block_count> m_column_tone;

    void apply_published() const;
    void detach_flange_attachments();
    void place_flange_attachments() const;
    void rebuild_decoration();
    void rebuild_chain();
    void clear_chain();
    void apply_selection() const;
    void place_joint_decoration() const;
    void place_ellipsoids() const;
    void place_jacobian_columns() const;

    // Every arrow of every column left undrawn, which is what a placement that cannot honestly
    // stand them does rather than stand some of them.
    void hide_jacobian_columns() const;

    // Every configuration folded to the points its figure is drawn through, or the refusal the
    // first one that cannot be folded produced.
    expected<std::vector<std::vector<Eigen::Vector3d>>, refusal> fold_solutions(std::span<const joint_vector> at) const;

    void raise_solution_figure(std::size_t figure, std::size_t joints);

    // Builds the body and the six lines of one block where it carries none, and does nothing where
    // it already stands.
    void raise_ellipsoid(jacobian_block which);

    // The answer may be the end iterator.
    std::vector<drawn_path>::iterator standing_path(const std::string &under);

    bool path_shown_or_opening(const std::string &under);
};

}

#endif
