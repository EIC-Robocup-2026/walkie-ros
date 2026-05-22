// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <string.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <vector>
/*
 * Compute gravity and inertia compensation using Orocos
 * Kinematics and Dynamics Library (KDL).
 */
class Dynamics {
private:
    std::shared_ptr<urdf::ModelInterface> urdf_model_interface;

    std::string urdf_path;
    std::string start_link;
    std::string end_link;

    KDL::JntSpaceInertiaMatrix inertia_matrix;
    KDL::JntArray q;
    KDL::JntArray q_d;
    KDL::JntArray coriolis_forces;
    KDL::JntArray gravity_forces;

    KDL::JntArray biasangle;

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    std::unique_ptr<KDL::ChainDynParam> solver;

    // Configurable gravity vector expressed in the chain root frame.
    // Defaults to world gravity assuming the chain root is gravity-aligned.
    double gravity_x_ = 0.0;
    double gravity_y_ = 0.0;
    double gravity_z_ = -9.81;

    bool BuildSolver();

public:
    Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
    ~Dynamics();

    // Set the gravity vector (in chain root frame). Must be called before Init().
    void SetGravity(double gx, double gy, double gz);

    // Number of joints in the configured chain (valid after a successful init).
    size_t GetNrOfJoints() const { return kdl_chain.getNrOfJoints(); }

    // Initialize from the URDF file pointed to by urdf_path passed in the
    // constructor.
    bool Init();

    // Initialize from a URDF XML string directly (no file IO needed).
    bool InitFromString(const std::string &urdf_xml);

    void GetGravity(const double *motor_position, double *gravity);
    void GetCoriolis(const double *motor_position, const double *motor_velocity, double *coriolis);
    void GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag);

    void GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian);

    void GetNullSpace(const double *motor_positon, Eigen::MatrixXd &nullspace);

    void GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T);

    void GetEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    void GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);
};
