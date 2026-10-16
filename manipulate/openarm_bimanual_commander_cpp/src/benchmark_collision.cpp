#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/kinematic_constraints/utils.hpp>

#include <chrono>
#include <random>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <sstream>

struct StatSummary {
    double mean;
    double median;
    double stddev;
    double min_val;
    double max_val;
    double p90;
    double p95;
    double p99;
};

StatSummary compute_stats(std::vector<double> values) {
    StatSummary s = {0, 0, 0, 0, 0, 0, 0, 0};
    if (values.empty()) return s;
    std::sort(values.begin(), values.end());
    size_t n = values.size();
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    s.mean = sum / n;
    s.median = (n % 2 == 0) ? (values[n / 2 - 1] + values[n / 2]) / 2.0 : values[n / 2];
    s.min_val = values.front();
    s.max_val = values.back();

    auto pct = [&](double p) {
        double idx = p * (n - 1) / 100.0;
        size_t lo = static_cast<size_t>(std::floor(idx));
        size_t hi = static_cast<size_t>(std::ceil(idx));
        if (lo == hi) return values[lo];
        return values[lo] + (values[hi] - values[lo]) * (idx - lo);
    };
    s.p90 = pct(90.0);
    s.p95 = pct(95.0);
    s.p99 = pct(99.0);

    double sq_sum = 0.0;
    for (double v : values) sq_sum += (v - s.mean) * (v - s.mean);
    s.stddev = std::sqrt(sq_sum / n);
    return s;
}

struct PlanResult {
    std::string scenario;
    std::string group;
    size_t total_trials;
    size_t successful_trials;
    StatSummary planning_time_ms;
    double mean_waypoints;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_benchmark_node");

    std::string output_json = "benchmark_results.json";
    int num_collision_checks = 5000;
    int num_planning_trials = 20;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--output" && i + 1 < argc) {
            output_json = argv[++i];
        } else if (arg == "--collision-checks" && i + 1 < argc) {
            num_collision_checks = std::stoi(argv[++i]);
        } else if (arg == "--planning-trials" && i + 1 < argc) {
            num_planning_trials = std::stoi(argv[++i]);
        }
    }

    std::cout << "================================================================================" << std::endl;
    std::cout << "               MOVEIT COLLISION & PLANNING BENCHMARK                            " << std::endl;
    std::cout << "================================================================================" << std::endl;
    std::cout << "Output JSON:        " << output_json << std::endl;
    std::cout << "Collision Checks:   " << num_collision_checks << std::endl;
    std::cout << "Planning Trials/Sc: " << num_planning_trials << std::endl;

    RCLCPP_INFO(node->get_logger(), "Loading robot model...");
    robot_model_loader::RobotModelLoader::Options opts("robot_description");
    robot_model_loader::RobotModelLoader loader(node, opts);
    auto robot_model = loader.getModel();
    if (!robot_model) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load robot model!");
        return 1;
    }

    planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    std::cout << "\n[1] Robot Model Info:" << std::endl;
    std::cout << "   Robot Name: " << robot_model->getName() << std::endl;
    std::cout << "   Model Links: " << robot_model->getLinkModelNames().size() << std::endl;
    std::cout << "   Active Joints: " << robot_model->getActiveJointModels().size() << std::endl;

    // -------------------------------------------------------------------------
    // Benchmark 1: Self-Collision Detection Throughput (FCL Microbenchmark)
    // -------------------------------------------------------------------------
    std::cout << "\n[2] Benchmarking Self-Collision Detection (" << num_collision_checks << " iterations)..." << std::endl;
    
    // Sample random joint states using fixed seed
    std::mt19937_64 rng(42);
    moveit::core::RobotState sample_state(robot_model);
    sample_state.setToDefaultValues();
    sample_state.setVariablePosition("lift_joint", 0.7435);

    collision_detection::CollisionRequest col_req;
    col_req.contacts = false;

    std::vector<double> check_times_us;
    check_times_us.reserve(num_collision_checks);
    size_t collisions_detected = 0;

    // Warm-up run (100 checks)
    for (int i = 0; i < 100; ++i) {
        sample_state.setToRandomPositions();
        sample_state.setVariablePosition("lift_joint", 0.7435);
        collision_detection::CollisionResult col_res;
        planning_scene->checkSelfCollision(col_req, col_res, sample_state);
    }

    // Timed run
    auto t_start_col = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_collision_checks; ++i) {
        sample_state.setToRandomPositions();
        sample_state.setVariablePosition("lift_joint", 0.7435);

        auto t0 = std::chrono::high_resolution_clock::now();
        collision_detection::CollisionResult col_res;
        planning_scene->checkSelfCollision(col_req, col_res, sample_state);
        auto t1 = std::chrono::high_resolution_clock::now();

        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        check_times_us.push_back(us);
        if (col_res.collision) collisions_detected++;
    }
    auto t_end_col = std::chrono::high_resolution_clock::now();
    double total_col_ms = std::chrono::duration<double, std::milli>(t_end_col - t_start_col).count();

    StatSummary col_stats = compute_stats(check_times_us);
    double col_hz = (num_collision_checks / (total_col_ms / 1000.0));

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "   Total Elapsed Time: " << total_col_ms << " ms" << std::endl;
    std::cout << "   Throughput:         " << col_hz << " checks/sec" << std::endl;
    std::cout << "   Mean Time / Check:  " << col_stats.mean << " us (" << (col_stats.mean / 1000.0) << " ms)" << std::endl;
    std::cout << "   Median:             " << col_stats.median << " us" << std::endl;
    std::cout << "   Min / Max:          " << col_stats.min_val << " us / " << col_stats.max_val << " us" << std::endl;
    std::cout << "   95th Percentile:    " << col_stats.p95 << " us" << std::endl;
    std::cout << "   Collisions Found:   " << collisions_detected << " / " << num_collision_checks
              << " (" << (100.0 * collisions_detected / num_collision_checks) << "%)" << std::endl;

    // Contact details benchmark (1,000 queries)
    std::cout << "\n[3] Benchmarking Contact Details Computation (1000 iterations)..." << std::endl;
    collision_detection::CollisionRequest contact_req;
    contact_req.contacts = true;
    contact_req.max_contacts = 10;
    std::vector<double> contact_times_us;
    contact_times_us.reserve(1000);

    for (int i = 0; i < 1000; ++i) {
        sample_state.setToRandomPositions();
        sample_state.setVariablePosition("lift_joint", 0.7435);

        auto t0 = std::chrono::high_resolution_clock::now();
        collision_detection::CollisionResult col_res;
        planning_scene->checkSelfCollision(contact_req, col_res, sample_state);
        auto t1 = std::chrono::high_resolution_clock::now();

        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        contact_times_us.push_back(us);
    }
    StatSummary contact_stats = compute_stats(contact_times_us);
    std::cout << "   Mean Contact Check: " << contact_stats.mean << " us" << std::endl;
    std::cout << "   Median Contact:     " << contact_stats.median << " us" << std::endl;
    std::cout << "   95th Percentile:    " << contact_stats.p95 << " us" << std::endl;

    // -------------------------------------------------------------------------
    // Benchmark 4: MoveIt Motion Planning Pipeline (OMPL / RRTConnect)
    // -------------------------------------------------------------------------
    std::cout << "\n[4] Initializing MoveIt Planning Pipeline (OMPL)..." << std::endl;
    auto pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "ompl");

    struct ScenarioDef {
        std::string name;
        std::string start_pose;
        std::string goal_pose;
    };

    std::vector<ScenarioDef> scenarios = {
        {"Home -> Standby",     "home",     "standby"},
        {"Standby -> Basket",   "standby",  "basket"},
        {"Standby -> Pre-place","standby",  "pre-place"},
        {"Pre-place -> Basket", "pre-place","basket"},
        {"Basket -> Home",      "basket",   "home"},
        {"Home -> Hands-up",    "home",     "hands_up"}
    };

    std::vector<std::string> groups = {"left_arm", "right_arm", "both_arms"};
    std::vector<PlanResult> plan_results;

    std::cout << "\n[5] Running Motion Planning Benchmarks (" << num_planning_trials << " trials per scenario)..." << std::endl;

    for (const auto& grp_name : groups) {
        auto jmg = robot_model->getJointModelGroup(grp_name);
        if (!jmg) {
            std::cerr << "Warning: Group " << grp_name << " not found!" << std::endl;
            continue;
        }

        std::cout << "\n========================================================" << std::endl;
        std::cout << "  PLANNING GROUP: " << grp_name << " (" << jmg->getVariableCount() << " DOF)" << std::endl;
        std::cout << "========================================================" << std::endl;

        for (const auto& sc : scenarios) {
            moveit::core::RobotState start_state(robot_model);
            start_state.setToDefaultValues();
            start_state.setVariablePosition("lift_joint", 0.7435);

            moveit::core::RobotState goal_state(robot_model);
            goal_state.setToDefaultValues();
            goal_state.setVariablePosition("lift_joint", 0.7435);

            // Populate start and goal from named SRDF states
            if (!start_state.setToDefaultValues(jmg, sc.start_pose)) {
                // If not defined for this specific arm, try both_arms
                auto jmg_both = robot_model->getJointModelGroup("both_arms");
                start_state.setToDefaultValues(jmg_both, sc.start_pose);
            }
            if (!goal_state.setToDefaultValues(jmg, sc.goal_pose)) {
                auto jmg_both = robot_model->getJointModelGroup("both_arms");
                goal_state.setToDefaultValues(jmg_both, sc.goal_pose);
            }

            // Verify start and goal states are collision-free
            collision_detection::CollisionResult cr_start, cr_goal;
            planning_scene->checkSelfCollision(col_req, cr_start, start_state);
            planning_scene->checkSelfCollision(col_req, cr_goal, goal_state);
            if (cr_start.collision || cr_goal.collision) {
                std::cout << "  Skipping " << sc.name << " (start or goal in collision)" << std::endl;
                continue;
            }

            std::vector<double> planning_times_ms;
            std::vector<double> waypoint_counts;
            size_t successes = 0;

            for (int trial = 0; trial < num_planning_trials; ++trial) {
                planning_interface::MotionPlanRequest req;
                planning_interface::MotionPlanResponse res;

                req.group_name = grp_name;
                req.allowed_planning_time = 10.0;
                req.num_planning_attempts = 1;

                moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
                req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg, 0.01, 0.01));

                // Execute planning pipeline
                pipeline->generatePlan(planning_scene, req, res);

                if (res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    successes++;
                    double ms = res.planning_time * 1000.0;
                    planning_times_ms.push_back(ms);
                    if (res.trajectory) {
                        waypoint_counts.push_back(static_cast<double>(res.trajectory->getWayPointCount()));
                    }
                }
            }

            StatSummary sc_stats = compute_stats(planning_times_ms);
            double mean_wp = waypoint_counts.empty() ? 0.0 :
                std::accumulate(waypoint_counts.begin(), waypoint_counts.end(), 0.0) / waypoint_counts.size();

            PlanResult pr;
            pr.scenario = sc.name;
            pr.group = grp_name;
            pr.total_trials = num_planning_trials;
            pr.successful_trials = successes;
            pr.planning_time_ms = sc_stats;
            pr.mean_waypoints = mean_wp;
            plan_results.push_back(pr);

            std::cout << "  " << std::left << std::setw(24) << sc.name << " | "
                      << "Success: " << std::setw(2) << successes << "/" << num_planning_trials << " | "
                      << "Mean: " << std::setw(7) << sc_stats.mean << " ms | "
                      << "Median: " << std::setw(7) << sc_stats.median << " ms | "
                      << "P95: " << std::setw(7) << sc_stats.p95 << " ms | "
                      << "StdDev: " << std::setw(6) << sc_stats.stddev << " ms | "
                      << "Waypoints: " << std::setw(4) << mean_wp << std::endl;
        }
    }

    // -------------------------------------------------------------------------
    // Write JSON Results
    // -------------------------------------------------------------------------
    std::ofstream jf(output_json);
    if (jf.is_open()) {
        jf << "{\n";
        jf << "  \"robot_name\": \"" << robot_model->getName() << "\",\n";
        jf << "  \"num_collision_checks\": " << num_collision_checks << ",\n";
        jf << "  \"collision_benchmark\": {\n";
        jf << "    \"total_time_ms\": " << total_col_ms << ",\n";
        jf << "    \"checks_per_sec\": " << col_hz << ",\n";
        jf << "    \"mean_us\": " << col_stats.mean << ",\n";
        jf << "    \"median_us\": " << col_stats.median << ",\n";
        jf << "    \"min_us\": " << col_stats.min_val << ",\n";
        jf << "    \"max_us\": " << col_stats.max_val << ",\n";
        jf << "    \"p90_us\": " << col_stats.p90 << ",\n";
        jf << "    \"p95_us\": " << col_stats.p95 << ",\n";
        jf << "    \"p99_us\": " << col_stats.p99 << ",\n";
        jf << "    \"stddev_us\": " << col_stats.stddev << ",\n";
        jf << "    \"collisions_detected\": " << collisions_detected << ",\n";
        jf << "    \"collision_rate\": " << (static_cast<double>(collisions_detected) / num_collision_checks) << "\n";
        jf << "  },\n";
        jf << "  \"contact_benchmark\": {\n";
        jf << "    \"mean_us\": " << contact_stats.mean << ",\n";
        jf << "    \"median_us\": " << contact_stats.median << ",\n";
        jf << "    \"p95_us\": " << contact_stats.p95 << "\n";
        jf << "  },\n";
        jf << "  \"planning_results\": [\n";
        for (size_t i = 0; i < plan_results.size(); ++i) {
            const auto& r = plan_results[i];
            jf << "    {\n";
            jf << "      \"group\": \"" << r.group << "\",\n";
            jf << "      \"scenario\": \"" << r.scenario << "\",\n";
            jf << "      \"total_trials\": " << r.total_trials << ",\n";
            jf << "      \"successful_trials\": " << r.successful_trials << ",\n";
            jf << "      \"mean_waypoints\": " << r.mean_waypoints << ",\n";
            jf << "      \"mean_ms\": " << r.planning_time_ms.mean << ",\n";
            jf << "      \"median_ms\": " << r.planning_time_ms.median << ",\n";
            jf << "      \"stddev_ms\": " << r.planning_time_ms.stddev << ",\n";
            jf << "      \"min_ms\": " << r.planning_time_ms.min_val << ",\n";
            jf << "      \"max_ms\": " << r.planning_time_ms.max_val << ",\n";
            jf << "      \"p90_ms\": " << r.planning_time_ms.p90 << ",\n";
            jf << "      \"p95_ms\": " << r.planning_time_ms.p95 << ",\n";
            jf << "      \"p99_ms\": " << r.planning_time_ms.p99 << "\n";
            jf << "    }" << (i + 1 < plan_results.size() ? "," : "") << "\n";
        }
        jf << "  ]\n";
        jf << "}\n";
        jf.close();
        std::cout << "\nResults successfully written to: " << output_json << std::endl;
    }

    std::cout << "\nBenchmark complete!" << std::endl;
    _exit(0);
}
