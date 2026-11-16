// Choice of the goal state of a whole-body goal, shared by the commander
// (whole_body_option.hpp) and the planning benchmark (benchmark_planning.cpp), so
// that both choose the same goal.
//
// A candidate is a sampled base pose with an IK solution of the arm and the lift. Its
// cost is a weighted sum of how far the base, the lift and each arm joint move from the
// current state, plus how far the arm ends from a rest posture. See "Tune whole-body
// goals" in docs/guides/manipulation.md.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace whole_body_goal
{
using BasePose = std::array<double, 3>;  // x, y, yaw in map
using Joints = std::map<std::string, double>;

constexpr std::size_t kArmJoints = 7;

// Cost per unit of motion from the current state: per metre for base_xy and lift, per
// radian for the others. arm[i] is joint i + 1 of the reaching arm. rest is per radian
// between an arm joint and the rest posture, summed over the arm joints.
struct Weights
{
    double base_xy{1.0};
    double base_yaw{0.0};
    double lift{0.0};
    std::array<double, kArmJoints> arm{};
    double rest{0.0};
};

// How far a candidate moves each part, before weighting.
struct Motion
{
    double base_xy{0.0};
    double base_yaw{0.0};
    double lift{0.0};
    std::array<double, kArmJoints> arm{};
    double rest{0.0};

    double armTotal() const
    {
        double total = 0.0;
        for (const double a : arm) total += a;
        return total;
    }
};

// Tier 0 holds the poses where IK usually finds usable goals, tier 1 a wider fallback.
struct BaseSample
{
    BasePose pose;
    int tier;
};

struct Candidate
{
    BasePose base;
    Joints joints;  // the arm and the lift
    Motion motion;
    double cost;
};

// The index of an arm joint in Weights::arm, or -1 for any other joint.
inline int armJointIndex(const std::string & name)
{
    for (const std::string prefix : {"openarm_left_joint", "openarm_right_joint"}) {
        if (name.size() == prefix.size() + 1 && name.compare(0, prefix.size(), prefix) == 0) {
            const char digit = name.back();
            if (digit >= '1' && digit < static_cast<char>('1' + kArmJoints)) return digit - '1';
        }
    }
    return -1;
}

inline double baseCost(const Weights & w, const BasePose & from, const BasePose & to)
{
    return w.base_xy * std::hypot(to[0] - from[0], to[1] - from[1]) +
           w.base_yaw * std::abs(std::remainder(to[2] - from[2], 2.0 * M_PI));
}

// Joints missing from `current` or `rest` count as no motion.
inline Motion measure(
    const BasePose & from, const BasePose & to, const Joints & current, const Joints & goal, const Joints & rest)
{
    Motion m;
    m.base_xy = std::hypot(to[0] - from[0], to[1] - from[1]);
    m.base_yaw = std::abs(std::remainder(to[2] - from[2], 2.0 * M_PI));
    for (const auto & [name, value] : goal) {
        const auto now = current.find(name);
        const double moved = now == current.end() ? 0.0 : std::abs(value - now->second);
        if (name == "lift_joint") {
            m.lift = moved;
            continue;
        }
        const int index = armJointIndex(name);
        if (index < 0) continue;
        m.arm[static_cast<std::size_t>(index)] = moved;
        const auto preferred = rest.find(name);
        if (preferred != rest.end()) m.rest += std::abs(value - preferred->second);
    }
    return m;
}

inline double cost(const Weights & w, const Motion & m)
{
    double total = w.base_xy * m.base_xy + w.base_yaw * m.base_yaw + w.lift * m.lift + w.rest * m.rest;
    for (std::size_t i = 0; i < kArmJoints; ++i) total += w.arm[i] * m.arm[i];
    return total;
}

// Base poses that put the arm's shoulder behind the hand, cheapest base motion first
// within each tier. Reach and heading vary together, because the shoulder is off the base
// centre. The first tier keeps to where IK finds usable goals, measured for a hand
// pointing forward at about 1 m: the shoulder 0.3 to 0.6 m behind the hand, the hand
// mostly on the arm's own side of the shoulder, and the body turned up to 1 rad away from
// that side, which keeps the target near the head camera's view. The second tier is
// wider, for other hand orientations. hand_z is the goal orientation's z axis in map, and
// shoulder is the shoulder's position in base_footprint.
inline std::vector<BaseSample> sampleBasePoses(
    double goal_x, double goal_y, double hand_z_x, double hand_z_y, const BasePose & from, double shoulder_x,
    double shoulder_y, const Weights & w, std::mt19937 & rng)
{
    // Base behind the hand when the hand points sideways enough, otherwise on the line
    // to the target.
    const double approach = std::hypot(hand_z_x, hand_z_y) > 0.3 ? std::atan2(hand_z_y, hand_z_x)
                                                                  : std::atan2(goal_y - from[1], goal_x - from[0]);
    // Mirrors the first tier for the arm on the other side.
    const double side = shoulder_y >= 0.0 ? 1.0 : -1.0;

    struct Tier
    {
        double reach_min, reach_max, around_min, around_max, turn_min, turn_max;
    };
    const std::array<Tier, 2> tiers = {{{0.30, 0.60, -0.4, 0.8, -1.0, 0.0}, {0.20, 0.70, -0.8, 0.8, -1.5, 1.5}}};
    std::vector<BaseSample> samples;
    for (std::size_t t = 0; t < tiers.size(); ++t) {
        const auto & range = tiers[t];
        std::uniform_real_distribution<double> reach(range.reach_min, range.reach_max);
        std::uniform_real_distribution<double> around(range.around_min, range.around_max);
        std::uniform_real_distribution<double> turn(range.turn_min, range.turn_max);
        const auto first = static_cast<std::ptrdiff_t>(samples.size());
        for (int i = 0; i < 40; ++i) {
            const double r = reach(rng);
            const double a = approach + side * around(rng);
            const double yaw = approach + side * turn(rng);
            samples.push_back(
                {{goal_x - r * std::cos(a) - (std::cos(yaw) * shoulder_x - std::sin(yaw) * shoulder_y),
                  goal_y - r * std::sin(a) - (std::sin(yaw) * shoulder_x + std::cos(yaw) * shoulder_y),
                  std::remainder(yaw, 2.0 * M_PI)},
                 static_cast<int>(t)});
        }
        std::stable_sort(samples.begin() + first, samples.end(), [&](const BaseSample & a, const BaseSample & b) {
            return baseCost(w, from, a.pose) < baseCost(w, from, b.pose);
        });
    }
    return samples;
}

// Hands out the candidates of `samples` one at a time, cheapest first, and solves IK only
// as far as that needs. The caller asks for the next candidate when a plan to the last
// one fails.
//
// It holds up to `max_candidates` solved candidates that were not handed out yet, and
// hands out the cheapest of them. Before that it solves samples in order until the hold is
// full, the tier ends, or a sample's base cost alone reaches the cheapest total held: the
// samples of a tier are sorted by base cost and the other terms are never negative, so no
// later sample in the tier can be cheaper. The candidates of a tier are all handed out
// before the next tier is solved. When only the base terms have weight, each candidate is
// the next usable sample, nearest first, and the search costs no extra IK call.
class CandidateSearch
{
public:
    CandidateSearch(
        std::vector<BaseSample> samples, const Weights & w, const BasePose & from, Joints current, Joints rest,
        std::size_t max_candidates)
    : samples_(std::move(samples)),
      w_(w),
      from_(from),
      current_(std::move(current)),
      rest_(std::move(rest)),
      max_candidates_(std::max<std::size_t>(1, max_candidates))
    {
    }

    // solve(pose, joints) returns whether the sample has a usable IK solution and fills
    // joints with it. Returns nothing once every sample was solved and handed out.
    template <typename Solve>
    std::optional<Candidate> next(Solve && solve)
    {
        while (next_ < samples_.size() && held_.size() < max_candidates_) {
            const BaseSample & sample = samples_[next_];
            if (!held_.empty() && (sample.tier != tier_ || baseCost(w_, from_, sample.pose) >= cheapest()->cost)) {
                break;
            }
            ++next_;
            Joints joints;
            if (!solve(sample.pose, joints)) continue;
            Candidate candidate{sample.pose, std::move(joints), {}, 0.0};
            candidate.motion = measure(from_, sample.pose, current_, candidate.joints, rest_);
            candidate.cost = cost(w_, candidate.motion);
            tier_ = sample.tier;
            held_.push_back(std::move(candidate));
        }
        if (held_.empty()) return std::nullopt;
        const auto best = cheapest();
        Candidate candidate = std::move(*best);
        held_.erase(best);
        return candidate;
    }

private:
    // The first of equal costs, which has the lower base cost.
    std::vector<Candidate>::iterator cheapest()
    {
        return std::min_element(held_.begin(), held_.end(), [](const Candidate & a, const Candidate & b) {
            return a.cost < b.cost;
        });
    }

    std::vector<BaseSample> samples_;
    Weights w_;
    BasePose from_;
    Joints current_;
    Joints rest_;
    std::size_t max_candidates_;
    std::size_t next_{0};
    int tier_{-1};
    std::vector<Candidate> held_;
};

}  // namespace whole_body_goal
