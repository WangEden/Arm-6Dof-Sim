#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <string>
#include <vector>

namespace arm_kin {

    struct Pose
    {
        double pos[3];
        double quat[4];
    };

    struct IKResult
    {
        bool converged = false;
        int iters = 0;
        double pos_error = 0.0;
        double rot_error = 0.0;
        std::string message;
    };

    class ArmKinematics
    {
    public:
        static constexpr int kNumJoints = 6;
        static constexpr const char *kJointNames[kNumJoints] = {
            "Base_Joint", "Shoulder_Joint", "Elbow_Joint", "Wrist1_Joint", "Wrist2_Joint", "Wrist3_Joint"};
        static constexpr const char *kEndEffectorBody = "End_Link";
        explicit ArmKinematics(const mjModel *model);
        ~ArmKinematics();
        ArmKinematics(const ArmKinematics &) = delete;
        ArmKinematics &operator=(const ArmKinematics &) = delete;
        bool valid() const { return valid_; }
        double jointLow(int i) const { return joint_range_[i][0]; }
        double jointHigh(int i) const { return joint_range_[i][1]; }
        int qposAdr(int i) const { return qadr_[i]; }
        void forward(const double q[kNumJoints], Pose &out) const;
        void forwardPosition(const double q[kNumJoints], double pos[3]) const;
        IKResult inverse(const Pose &target,
                         double q[kNumJoints],
                         int max_iter = 200,
                         double tol_pos = 1e-4,
                         double tol_rot = 1e-3,
                         double damping = 0.05) const;

    private:
        void setQAndForward(const double q[kNumJoints]) const;
        const mjModel *m_ = nullptr;
        mjData *d_ = nullptr;
        bool valid_ = false;
        int ee_body_ = -1;
        std::array<int, kNumJoints> jid_{};
        std::array<int, kNumJoints> qadr_{};
        std::array<int, kNumJoints> dadr_{};
        std::array<std::array<double, 2>, kNumJoints> joint_range_{};
    };
} // namespace arm_kin
