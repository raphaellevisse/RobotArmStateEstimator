#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H
#include <iostream>
#include <vector>
#include <random>
#include <numeric>
#include <cassert>
#include <Eigen/dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <numeric>
#include <iomanip>
#include "kinematics.h"
#include "inv_kin.h"


class RobotArm {
private:
    int n{}; // number of joints
    std::vector<double> m_lengths;
    Eigen::Matrix<double, 5, 1> m_thetas;
    std::vector<Eigen::Vector3d> m_axes; // Axis of rotation for each joint
    std::vector<Twist> m_twists{};
    Eigen::Vector4d m_initial_pos;
    Eigen::Vector4d m_actual_pos{};
    // define m_orientation as identity
    Eigen::Matrix3d m_orientation = Eigen::Matrix3d::Identity();

    double m_alpha{ 0.2 }; // default value for alpha which is the learning rate
    double m_K_orient{0.0 }; // default value for K_orient which is the Jacobian weighting of orientation error
public:
    RobotArm(const std::vector<double>& user_lengths, const std::vector<Eigen::Vector3d>& user_axes)
        : m_lengths(user_lengths), m_axes(user_axes) {
        n = static_cast<int>(user_lengths.size());
        m_twists.reserve(n);
        double x_position = 0;
        for (int i = 0; i < n; ++i) {
            Twist twist;
            twist.omega = user_axes[i];
            Eigen::Vector3d p;
            if (i == 0) {
                p = Eigen::Vector3d(0, 0, 0);
            }
            else {
                x_position = std::accumulate(user_lengths.begin(), user_lengths.begin() + i, 0.0);
                p = Eigen::Vector3d(x_position, 0, 0);
            }
            twist.v = -twist.omega.cross(p);
            // std::cout << "Twist " << i << " is " << twist.v << " " << twist.omega << std::endl;
            m_twists.push_back(twist);
        }
        m_initial_pos = Eigen::Vector4d(x_position + user_lengths[n - 1], 0, 0, 1);
        m_actual_pos = m_initial_pos;
    }
    Eigen::Matrix<double, 5, 1> get_thetas() {
        return m_thetas;
    }

    void set_i(int i, Radian theta) {
        assert(i >= 0 && i <= n);
        m_thetas(i) = theta;
    }

    std::pair<Eigen::Vector4d, Eigen::Matrix3d> FWD_Kinematics(const Eigen::Vector4d& end_effector) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < n; ++i) {
            T *= calculateExpMap(m_twists[i], m_thetas(i));
        }
        Eigen::Vector4d position = T * end_effector;
        Eigen::Matrix3d orientation = T.block<3, 3>(0, 0); // Extract rotation matrix
        std::cout << "Position is " << position << std::endl;

        return std::make_pair(position, orientation);
    }
    void update_pos() {
        auto [new_pos, new_orientation] = FWD_Kinematics(m_initial_pos);
        m_actual_pos = new_pos;
        m_orientation = new_orientation;
    }

    void updateTheta(const Eigen::Vector4d& e_pos, const Eigen::Vector3d& e_rot) {
        Eigen::Matrix<double, 6, 5> J = jacobian(m_twists, m_thetas, m_initial_pos.head(3)); 

        Eigen::Matrix<double, 5, 6> invJ = computePseudoInverse(J);
        std::cout << "Pseudo Inverse is " << invJ << std::endl;
        Eigen::VectorXd e_combined(6);
        e_combined << e_pos.head(3), e_rot;
        
        m_thetas += m_alpha * (invJ * e_combined);
        update_pos();
    }
 
    std::vector<std::vector<double>> joint_positions() {
        std::vector<std::vector<double>> joint_pos;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        double init_joint_x_pos = 0;
        joint_pos.push_back({ 0, 0, 0 });
        for (int i = 0; i < n; ++i) {
            init_joint_x_pos += m_lengths[i];
            Eigen::Vector4d joint_vec_pos = { init_joint_x_pos, 0, 0, 1 };
            T *= calculateExpMap(m_twists[i], m_thetas(i));
            Eigen::Vector4d joint_pos_i = T * joint_vec_pos;
            joint_pos.push_back({ joint_pos_i(0), joint_pos_i(1), joint_pos_i(2) });
        }
        return joint_pos;
    }
    void set_alpha(double alpha) {
        m_alpha = alpha;
    }
    void set_K_orient(double K) {
        m_K_orient = K;
    }
    double get_default_alpha() {
        return m_alpha;
    }
    double get_default_K_orient() {
        return m_K_orient;
    }
    bool goalReached(const Eigen::Vector4d& goal_pos, double tolerance = 0.05) { 
        return ((m_actual_pos - goal_pos).head<3>().norm() < tolerance);
    }

    void incrementalMoveTowardsGoal(const Eigen::Vector4d& goal_pos, const Eigen::Matrix3d& goal_orientation) {
        Eigen::Vector4d e = goal_pos - m_actual_pos;
        Eigen::Vector3d e_rot = (goal_orientation.eulerAngles(0, 1, 2) - m_orientation.eulerAngles(0, 1, 2)).normalized();
        if (!goalReached(goal_pos)) {
            updateTheta(e, m_K_orient*e_rot);
        }
    }
};

#endif
