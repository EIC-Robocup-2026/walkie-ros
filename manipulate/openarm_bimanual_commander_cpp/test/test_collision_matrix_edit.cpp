// Unit tests of src/collision_matrix_edit.hpp. MoveIt's own AllowedCollisionMatrix stands in
// for move_group: each edit reads its message, as /get_planning_scene does, and the edited
// message becomes its new matrix, as /apply_planning_scene does.

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_matrix.hpp>

#include "../src/collision_matrix_edit.hpp"

namespace
{
namespace edit = collision_matrix_edit;
using collision_detection::AllowedCollision::ALWAYS;

const std::string kOctomap = "<octomap>";
const std::vector<std::string> kLinks = {
    "base_link", "lift_link", "left_link7", "left_hand", "left_finger", "right_link7", "right_hand",
    "right_finger"};
const std::vector<std::string> kLeftGripper = {"left_hand", "left_finger"};
const std::vector<std::string> kGrippers = {"left_hand", "left_finger", "right_hand", "right_finger"};
// The SRDF's disabled pairs: neighbouring links.
const std::vector<std::pair<std::string, std::string>> kDisabled = {
    {"base_link", "lift_link"}, {"left_link7", "left_hand"}, {"left_hand", "left_finger"},
    {"right_link7", "right_hand"}, {"right_hand", "right_finger"}};
// A name only the default entries can decide: a collision object added after the edit.
const std::string kObject = "table";

class MoveGroup
{
public:
    MoveGroup()
    {
        for (const auto & [a, b] : kDisabled) acm_.setEntry(a, b, true);
    }

    edit::Matrix get() const
    {
        edit::Matrix message;
        acm_.getMessage(message);
        return message;
    }

    void apply(const edit::Matrix & message)
    {
        acm_ = collision_detection::AllowedCollisionMatrix(message);
    }

    bool allows(const std::string & a, const std::string & b) const
    {
        collision_detection::AllowedCollision::Type type;
        return acm_.getAllowedCollision(a, b, type) && type == ALWAYS;
    }

    // Every pair of links, the octomap and the object.
    std::map<std::pair<std::string, std::string>, bool> pairs() const
    {
        std::vector<std::string> names = kLinks;
        names.push_back(kOctomap);
        names.push_back(kObject);
        std::map<std::pair<std::string, std::string>, bool> result;
        for (std::size_t i = 0; i < names.size(); ++i) {
            for (std::size_t j = i + 1; j < names.size(); ++j) {
                result[{names[i], names[j]}] = allows(names[i], names[j]);
            }
        }
        return result;
    }

private:
    collision_detection::AllowedCollisionMatrix acm_;
};

class CollisionMatrixEdit : public ::testing::Test
{
protected:
    // The commander's baseline: the SRDF's pairs, and the grippers against the octomap
    // while `allow_grippers` is set.
    bool allow_grippers = false;
    edit::Baseline baseline = [this](const std::string & a, const std::string & b) {
        for (const auto & [x, y] : kDisabled) {
            if ((a == x && b == y) || (a == y && b == x)) return true;
        }
        const auto gripper = [](const std::string & n) {
            for (const auto & g : kGrippers) {
                if (g == n) return true;
            }
            return false;
        };
        return allow_grippers && ((gripper(a) && b == kOctomap) || (gripper(b) && a == kOctomap));
    };
    MoveGroup move_group;

    void setIgnored(const std::vector<std::string> & names, bool ignore)
    {
        auto message = move_group.get();
        edit::setIgnored(message, names, ignore, baseline);
        move_group.apply(message);
    }

    void applyAllowance(bool allow)
    {
        allow_grippers = allow;
        auto message = move_group.get();
        for (const auto & link : kGrippers) edit::refresh(message, link, {kOctomap}, baseline);
        move_group.apply(message);
    }

    std::vector<std::string> everything() const
    {
        std::vector<std::string> names = kLinks;
        names.push_back(kOctomap);
        return names;
    }
};

TEST_F(CollisionMatrixEdit, IgnoringEverythingAllowsEveryPair)
{
    setIgnored(everything(), true);
    for (const auto & [pair, allowed] : move_group.pairs()) {
        EXPECT_TRUE(allowed) << pair.first << " / " << pair.second;
    }
}

TEST_F(CollisionMatrixEdit, IgnoringEverythingKeepsEveryDefaultInTheMessage)
{
    // Every pair then equals the default entries, which keep no entry of their own, and
    // getMessage lists only names that sort before one with an entry.
    setIgnored(everything(), true);
    const auto message = move_group.get();
    for (const auto & name : everything()) {
        EXPECT_TRUE(edit::defaultAllows(message, name)) << name;
    }
    setIgnored({kOctomap}, false);
    EXPECT_TRUE(move_group.allows("base_link", "left_link7"));
    EXPECT_TRUE(move_group.allows("left_hand", kObject));
}

TEST_F(CollisionMatrixEdit, IgnoringTheOctomapAllowsEveryLinkAgainstIt)
{
    applyAllowance(true);  // lists the octomap
    setIgnored({kOctomap}, true);
    for (const auto & link : kLinks) EXPECT_TRUE(move_group.allows(link, kOctomap)) << link;
    EXPECT_FALSE(move_group.allows("base_link", "left_link7"));
}

TEST_F(CollisionMatrixEdit, IgnoringAGripperAllowsItAgainstEverything)
{
    setIgnored(kLeftGripper, true);
    for (const auto & link : kLeftGripper) {
        EXPECT_TRUE(move_group.allows(link, "base_link")) << link;
        EXPECT_TRUE(move_group.allows(link, kOctomap)) << link;
        EXPECT_TRUE(move_group.allows(link, kObject)) << link;
    }
    EXPECT_FALSE(move_group.allows("right_hand", "base_link"));
}

TEST_F(CollisionMatrixEdit, EachToggleOffAndOnRestoresEveryPair)
{
    applyAllowance(true);
    const auto before = move_group.pairs();
    for (const auto & names : {everything(), std::vector<std::string>{kOctomap}, kLeftGripper}) {
        setIgnored(names, true);
        setIgnored(names, false);
        EXPECT_EQ(move_group.pairs(), before) << "after toggling " << names.front();
    }
}

TEST_F(CollisionMatrixEdit, TogglesWorkAfterEverythingWasOffAndOn)
{
    // Turning everything back on leaves every link with a default entry that forbids, and
    // one that forbids wins over the octomap's.
    setIgnored(everything(), true);
    setIgnored(everything(), false);
    setIgnored({kOctomap}, true);
    for (const auto & link : kLinks) EXPECT_TRUE(move_group.allows(link, kOctomap)) << link;
    setIgnored({kOctomap}, false);
    for (const auto & link : kLinks) EXPECT_FALSE(move_group.allows(link, kOctomap)) << link;
}

TEST_F(CollisionMatrixEdit, OverlappingTogglesKeepEachOther)
{
    setIgnored(kLeftGripper, true);
    setIgnored({kOctomap}, true);
    setIgnored({kOctomap}, false);
    for (const auto & link : kLeftGripper) EXPECT_TRUE(move_group.allows(link, kOctomap)) << link;
    EXPECT_FALSE(move_group.allows("base_link", kOctomap));
    setIgnored(kLeftGripper, false);
    for (const auto & link : kLeftGripper) EXPECT_FALSE(move_group.allows(link, kOctomap)) << link;
}

TEST_F(CollisionMatrixEdit, TheAllowanceFollowsAnIgnoredOctomap)
{
    setIgnored({kOctomap}, true);
    applyAllowance(false);
    for (const auto & link : kGrippers) EXPECT_TRUE(move_group.allows(link, kOctomap)) << link;
    applyAllowance(true);
    setIgnored({kOctomap}, false);
    for (const auto & link : kGrippers) EXPECT_TRUE(move_group.allows(link, kOctomap)) << link;
    EXPECT_FALSE(move_group.allows("base_link", kOctomap));
    applyAllowance(false);
    for (const auto & link : kGrippers) EXPECT_FALSE(move_group.allows(link, kOctomap)) << link;
}

TEST_F(CollisionMatrixEdit, ListingANameKeepsWhatItsDefaultsGave)
{
    // A name with a default entry that the message leaves out, as getMessage does with a
    // default-only name that sorts last.
    auto message = move_group.get();
    edit::setDefault(message, kObject, true);
    edit::list(message, kObject);
    move_group.apply(message);
    for (const auto & link : kLinks) EXPECT_TRUE(move_group.allows(link, kObject)) << link;
    EXPECT_FALSE(move_group.allows("base_link", "left_link7"));
}

TEST_F(CollisionMatrixEdit, AllowsReadsTheMessageAsMoveItReadsIt)
{
    applyAllowance(true);
    setIgnored(kLeftGripper, true);
    setIgnored({kOctomap}, true);
    const auto message = move_group.get();
    for (const auto & [pair, allowed] : move_group.pairs()) {
        EXPECT_EQ(edit::allows(message, pair.first, pair.second), allowed)
            << pair.first << " / " << pair.second;
    }
}
}  // namespace
