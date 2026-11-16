// Unit tests of the whole-body goal ranking in src/whole_body_goal.hpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <random>
#include <tuple>
#include <vector>

#include "../src/whole_body_goal.hpp"

namespace
{
using whole_body_goal::BasePose;
using whole_body_goal::BaseSample;
using whole_body_goal::Joints;
using whole_body_goal::Weights;

const BasePose kOrigin{0.0, 0.0, 0.0};

// Samples on the x axis at the given distances, all in tier 0 unless a tier is given.
std::vector<BaseSample> line(const std::vector<double> & xs, const std::vector<int> & tiers = {})
{
    std::vector<BaseSample> samples;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        samples.push_back({{xs[i], 0.0, 0.0}, tiers.empty() ? 0 : tiers[i]});
    }
    return samples;
}

// A solver that returns `arm[i]` for openarm_left_joint1 at sample i, or fails where
// `usable[i]` is false, and records the samples it was called for.
struct FakeSolver
{
    std::vector<double> arm;
    std::vector<bool> usable;
    std::vector<double> called;

    bool operator()(const BasePose & pose, Joints & joints)
    {
        called.push_back(pose[0]);
        const auto i = static_cast<std::size_t>(std::lround(pose[0] * 10.0)) - 1;
        if (!usable.empty() && !usable[i]) return false;
        joints = {{"openarm_left_joint1", arm[i]}, {"lift_joint", 0.0}};
        return true;
    }
};

const Joints kCurrent{{"openarm_left_joint1", 0.0}, {"lift_joint", 0.0}};
const Joints kRest{{"openarm_left_joint1", 0.0}};

TEST(WholeBodyGoal, ArmJointIndexCoversBothArms)
{
    EXPECT_EQ(whole_body_goal::armJointIndex("openarm_left_joint1"), 0);
    EXPECT_EQ(whole_body_goal::armJointIndex("openarm_right_joint7"), 6);
    EXPECT_EQ(whole_body_goal::armJointIndex("openarm_left_joint8"), -1);
    EXPECT_EQ(whole_body_goal::armJointIndex("lift_joint"), -1);
    EXPECT_EQ(whole_body_goal::armJointIndex("openarm_left_finger_joint1"), -1);
}

TEST(WholeBodyGoal, MeasureSplitsTheMotion)
{
    const Joints current{{"lift_joint", 0.30}, {"openarm_left_joint2", 0.5}, {"openarm_left_joint4", 1.0}};
    const Joints goal{
        {"lift_joint", 0.10}, {"openarm_left_joint2", -0.5}, {"openarm_left_joint4", 1.5}, {"openarm_left_joint5", 0.2}};
    const Joints rest{{"openarm_left_joint2", 0.0}, {"openarm_left_joint4", 2.0}};
    const auto m = whole_body_goal::measure({0.0, 0.0, 3.0}, {3.0, 4.0, -3.0}, current, goal, rest);
    EXPECT_DOUBLE_EQ(m.base_xy, 5.0);
    // The short way round: from 3 rad to -3 rad is 2 pi - 6 rad.
    EXPECT_NEAR(m.base_yaw, 2.0 * M_PI - 6.0, 1e-12);
    EXPECT_NEAR(m.lift, 0.2, 1e-12);
    EXPECT_DOUBLE_EQ(m.arm[1], 1.0);
    EXPECT_DOUBLE_EQ(m.arm[3], 0.5);
    // joint5 has no current value, so it counts as not moved.
    EXPECT_DOUBLE_EQ(m.arm[4], 0.0);
    EXPECT_DOUBLE_EQ(m.armTotal(), 1.5);
    // Only joints with a rest value count: |-0.5 - 0| + |1.5 - 2|.
    EXPECT_DOUBLE_EQ(m.rest, 1.0);
}

// The base x of the next `count` candidates of a search, as the commander takes them when
// each plan fails, until the search runs out.
std::vector<double> handOut(whole_body_goal::CandidateSearch & search, FakeSolver & solve, std::size_t count)
{
    std::vector<double> xs;
    while (xs.size() < count) {
        const auto candidate = search.next(std::ref(solve));
        if (!candidate) break;
        xs.push_back(candidate->base[0]);
    }
    return xs;
}

whole_body_goal::CandidateSearch searchOver(
    const std::vector<BaseSample> & samples, const Weights & w, std::size_t max_candidates = 8,
    const Joints & rest = kRest)
{
    return whole_body_goal::CandidateSearch(samples, w, kOrigin, kCurrent, rest, max_candidates);
}

TEST(WholeBodyGoal, DefaultWeightsTakeTheNearestUsableSampleWithoutExtraIk)
{
    FakeSolver solve{{9.0, 9.0, 0.0, 0.0}, {false, true, true, true}, {}};
    auto s = searchOver(line({0.1, 0.2, 0.3, 0.4}), Weights{});
    EXPECT_EQ(handOut(s, solve, 1), (std::vector<double>{0.2}));
    // The next sample costs 0.3 in base motion alone, more than 0.2, so it is not solved
    // until the plan to 0.2 fails.
    EXPECT_EQ(solve.called, (std::vector<double>{0.1, 0.2}));
    EXPECT_EQ(handOut(s, solve, 1), (std::vector<double>{0.3}));
    EXPECT_EQ(solve.called, (std::vector<double>{0.1, 0.2, 0.3}));
}

TEST(WholeBodyGoal, ArmWeightTradesBaseMotionForArmMotion)
{
    FakeSolver solve{{3.0, 2.0, 0.0, 0.0}, {}, {}};
    Weights w;
    w.arm[0] = 0.1;
    auto s = searchOver(line({0.1, 0.2, 0.3, 0.4}), w);
    // Costs: 0.1 + 0.3, 0.2 + 0.2, 0.3 + 0. Sample 0.4 costs 0.4 in the base alone, more
    // than the cheapest 0.3, so the search stops before it.
    const auto first = s.next(std::ref(solve));
    ASSERT_TRUE(first);
    EXPECT_DOUBLE_EQ(first->base[0], 0.3);
    EXPECT_NEAR(first->cost, 0.3, 1e-12);
    EXPECT_EQ(solve.called.size(), 3u);
}

TEST(WholeBodyGoal, RestWeightPrefersTheRestPosture)
{
    FakeSolver solve{{0.0, 1.0}, {}, {}};
    Weights w;
    w.rest = 1.0;
    auto s = searchOver(line({0.1, 0.2}), w, 8, {{"openarm_left_joint1", 1.0}});
    const auto first = s.next(std::ref(solve));
    ASSERT_TRUE(first);
    EXPECT_DOUBLE_EQ(first->base[0], 0.2);
    EXPECT_DOUBLE_EQ(first->motion.rest, 0.0);
}

TEST(WholeBodyGoal, FailedPlanResumesTheSearchAgainstTheCheapestHeldCandidate)
{
    // Base costs 0.1, 0.2, 0.3 and totals 2.1, 0.2, 0.3. Sample 0.3 is not solved at
    // first, because 0.2 is cheaper than its base cost. When the plan to 0.2 fails, 0.3
    // is still cheaper than the held 2.1 and has to come next.
    FakeSolver solve{{2.0, 0.0, 0.0}, {}, {}};
    Weights w;
    w.arm[0] = 1.0;
    auto s = searchOver(line({0.1, 0.2, 0.3}), w);
    EXPECT_EQ(handOut(s, solve, 1), (std::vector<double>{0.2}));
    EXPECT_EQ(solve.called, (std::vector<double>{0.1, 0.2}));
    EXPECT_EQ(handOut(s, solve, 3), (std::vector<double>{0.3, 0.1}));
}

TEST(WholeBodyGoal, HandsOutEveryCandidateOfATierBeforeTheNext)
{
    // Tier 1 holds the cheapest candidate, but tier 0 goes first.
    FakeSolver solve{{5.0, 5.0, 0.0}, {}, {}};
    Weights w;
    w.arm[0] = 1.0;
    auto s = searchOver(line({0.1, 0.2, 0.3}, {0, 0, 1}), w);
    EXPECT_EQ(handOut(s, solve, 1), (std::vector<double>{0.1}));
    EXPECT_EQ(solve.called, (std::vector<double>{0.1, 0.2}));
    EXPECT_EQ(handOut(s, solve, 3), (std::vector<double>{0.2, 0.3}));
}

TEST(WholeBodyGoal, HoldsAtMostMaxCandidates)
{
    FakeSolver solve{{9.0, 9.0, 9.0, 0.0}, {}, {}};
    Weights w;
    w.arm[0] = 1.0;
    auto s = searchOver(line({0.1, 0.2, 0.3, 0.4}), w, 2);
    // With room for two, the search stops before it reaches the cheap sample 0.4.
    EXPECT_EQ(handOut(s, solve, 1), (std::vector<double>{0.1}));
    EXPECT_EQ(solve.called, (std::vector<double>{0.1, 0.2}));
    EXPECT_EQ(handOut(s, solve, 4), (std::vector<double>{0.2, 0.4, 0.3}));
}

TEST(WholeBodyGoal, EndsWhenNoSampleIsUsable)
{
    FakeSolver solve{{0.0, 0.0}, {false, false}, {}};
    auto s = searchOver(line({0.1, 0.2}), Weights{});
    EXPECT_FALSE(s.next(std::ref(solve)));
    EXPECT_EQ(solve.called, (std::vector<double>{0.1, 0.2}));
    EXPECT_FALSE(s.next(std::ref(solve)));
    EXPECT_EQ(solve.called.size(), 2u);
}

TEST(WholeBodyGoal, SamplesAreSortedByBaseCostWithinEachTier)
{
    std::mt19937 rng(3);
    Weights w;
    w.base_yaw = 0.5;
    const BasePose from{0.2, -0.1, 0.4};
    const auto samples = whole_body_goal::sampleBasePoses(1.5, 0.0, 1.0, 0.0, from, 0.05, 0.2, w, rng);
    ASSERT_EQ(samples.size(), 80u);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_EQ(samples[i].tier, i < 40 ? 0 : 1);
        if (i > 0 && samples[i].tier == samples[i - 1].tier) {
            EXPECT_LE(
                whole_body_goal::baseCost(w, from, samples[i - 1].pose), whole_body_goal::baseCost(w, from, samples[i].pose));
        }
    }
}

TEST(WholeBodyGoal, WeightsDoNotChangeWhichBasePosesAreSampled)
{
    std::mt19937 rng_a(7);
    std::mt19937 rng_b(7);
    Weights heavy_yaw;
    heavy_yaw.base_yaw = 5.0;
    auto a = whole_body_goal::sampleBasePoses(1.5, 0.3, 0.0, 0.0, kOrigin, 0.0, 0.2, Weights{}, rng_a);
    auto b = whole_body_goal::sampleBasePoses(1.5, 0.3, 0.0, 0.0, kOrigin, 0.0, 0.2, heavy_yaw, rng_b);
    const auto key = [](const BaseSample & s) { return std::make_tuple(s.tier, s.pose[0], s.pose[1], s.pose[2]); };
    std::vector<std::tuple<int, double, double, double>> ka, kb;
    for (const auto & s : a) ka.push_back(key(s));
    for (const auto & s : b) kb.push_back(key(s));
    std::sort(ka.begin(), ka.end());
    std::sort(kb.begin(), kb.end());
    EXPECT_EQ(ka, kb);
}

}  // namespace
