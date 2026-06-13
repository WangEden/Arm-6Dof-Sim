#include "Algorithm/Kinematics.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace arm_kin {

    namespace {

        constexpr int kDim = 6;

        bool solveLinear(double A[kDim][kDim], double b[kDim])
        {
            for (int col = 0; col < kDim; ++col) {
                int pivot = col;
                for (int r = col + 1; r < kDim; ++r) {
                    if (std::fabs(A[r][col]) > std::fabs(A[pivot][col]))
                        pivot = r;
                }
                if (std::fabs(A[pivot][col]) < 1e-12)
                    return false;

                if (pivot != col) {
                    std::swap(A[pivot], A[col]);
                    std::swap(b[pivot], b[col]);
                }

                const double inv = 1.0 / A[col][col];
                for (int r = col + 1; r < kDim; ++r) {
                    const double f = A[r][col] * inv;
                    if (f == 0.0)
                        continue;
                    for (int c = col; c < kDim; ++c)
                        A[r][c] -= f * A[col][c];
                    b[r] -= f * b[col];
                }
            }

            for (int i = kDim - 1; i >= 0; --i) {
                double s = b[i];
                for (int c = i + 1; c < kDim; ++c)
                    s -= A[i][c] * b[c];
                b[i] = s / A[i][i];
            }
            return true;
        }

        void normalizeQuat(double q[4])
        {
            double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (n < 1e-12) {
                q[0] = 1.0;
                q[1] = q[2] = q[3] = 0.0;
                return;
            }
            const double inv = 1.0 / n;
            q[0] *= inv;
            q[1] *= inv;
            q[2] *= inv;
            q[3] *= inv;
            if (q[0] < 0.0) {
                q[0] = -q[0];
                q[1] = -q[1];
                q[2] = -q[2];
                q[3] = -q[3];
            }
        }

    } // namespace

    ArmKinematics::ArmKinematics(const mjModel *model) : m_(model)
    {
        if (!m_)
            return;

        for (int i = 0; i < kNumJoints; ++i) {
            jid_[i] = mj_name2id(m_, mjOBJ_JOINT, kJointNames[i]);
            if (jid_[i] < 0)
                return;
            qadr_[i] = m_->jnt_qposadr[jid_[i]];
            dadr_[i] = m_->jnt_dofadr[jid_[i]];
            joint_range_[i][0] = m_->jnt_range[2 * jid_[i] + 0];
            joint_range_[i][1] = m_->jnt_range[2 * jid_[i] + 1];
        }

        ee_body_ = mj_name2id(m_, mjOBJ_BODY, kEndEffectorBody);
        if (ee_body_ < 0)
            return;

        d_ = mj_makeData(m_);
        if (!d_)
            return;

        valid_ = true;
    }

    ArmKinematics::~ArmKinematics()
    {
        if (d_)
            mj_deleteData(d_);
    }

    void ArmKinematics::setQAndForward(const double q[kNumJoints]) const
    {
        for (int i = 0; i < kNumJoints; ++i) {
            double qi = q[i];
            if (m_->jnt_limited[jid_[i]] && joint_range_[i][0] < joint_range_[i][1]) {
                qi = std::clamp(qi, joint_range_[i][0], joint_range_[i][1]);
            }
            d_->qpos[qadr_[i]] = qi;
        }
        mj_kinematics(m_, d_);
        mj_comPos(m_, d_);
    }

    void ArmKinematics::forward(const double q[kNumJoints], Pose &out) const
    {
        if (!valid_)
            return;

        setQAndForward(q);

        const mjtNum *p = d_->xpos + 3 * ee_body_;
        const mjtNum *R = d_->xmat + 9 * ee_body_;
        for (int i = 0; i < 3; ++i)
            out.pos[i] = p[i];

        mjtNum quat[4];
        mju_mat2Quat(quat, R);
        for (int i = 0; i < 4; ++i)
            out.quat[i] = quat[i];
    }

    void ArmKinematics::forwardPosition(const double q[kNumJoints], double pos[3]) const
    {
        Pose pose;
        forward(q, pose);
        std::memcpy(pos, pose.pos, sizeof(double) * 3);
    }

    IKResult ArmKinematics::inverse(
        const Pose &target, double q[kNumJoints], int max_iter, double tol_pos, double tol_rot, double damping) const
    {
        IKResult res;
        if (!valid_) {
            res.message = "Kinematics model not valid";
            return res;
        }

        std::vector<mjtNum> jacp(3 * m_->nv, 0.0);
        std::vector<mjtNum> jacr(3 * m_->nv, 0.0);

        mjtNum tgt_pos[3] = {target.pos[0], target.pos[1], target.pos[2]};
        mjtNum tgt_quat[4] = {target.quat[0], target.quat[1], target.quat[2], target.quat[3]};
        mju_normalize4(tgt_quat);

        double lam2 = damping * damping;

        for (int iter = 0; iter < max_iter; ++iter) {
            setQAndForward(q);

            const mjtNum *cur_pos = d_->xpos + 3 * ee_body_;
            const mjtNum *cur_R = d_->xmat + 9 * ee_body_;
            mjtNum cur_quat[4];
            mju_mat2Quat(cur_quat, cur_R);

            double e[kDim];
            e[0] = tgt_pos[0] - cur_pos[0];
            e[1] = tgt_pos[1] - cur_pos[1];
            e[2] = tgt_pos[2] - cur_pos[2];

            mjtNum cur_inv[4], delta_q[4];
            mju_negQuat(cur_inv, cur_quat);
            mju_mulQuat(delta_q, tgt_quat, cur_inv);
            mjtNum rot_err[3];
            mju_quat2Vel(rot_err, delta_q, 1.0);
            e[3] = rot_err[0];
            e[4] = rot_err[1];
            e[5] = rot_err[2];

            const double pos_norm = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
            const double rot_norm = std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);
            res.pos_error = pos_norm;
            res.rot_error = rot_norm;
            res.iters = iter + 1;

            if (pos_norm < tol_pos && rot_norm < tol_rot) {
                res.converged = true;
                res.message = "Converged";
                return res;
            }

            mj_jacBody(m_, d_, jacp.data(), jacr.data(), ee_body_);

            double J[kDim][kDim];
            for (int i = 0; i < kNumJoints; ++i) {
                const int col = dadr_[i];
                J[0][i] = jacp[0 * m_->nv + col];
                J[1][i] = jacp[1 * m_->nv + col];
                J[2][i] = jacp[2 * m_->nv + col];
                J[3][i] = jacr[0 * m_->nv + col];
                J[4][i] = jacr[1 * m_->nv + col];
                J[5][i] = jacr[2 * m_->nv + col];
            }

            double A[kDim][kDim];
            for (int r = 0; r < kDim; ++r) {
                for (int c = 0; c < kDim; ++c) {
                    double s = 0.0;
                    for (int k = 0; k < kDim; ++k)
                        s += J[r][k] * J[c][k];
                    A[r][c] = s + ((r == c) ? lam2 : 0.0);
                }
            }
            double rhs[kDim];
            for (int r = 0; r < kDim; ++r)
                rhs[r] = e[r];

            if (!solveLinear(A, rhs)) {
                lam2 *= 4.0;
                continue;
            }

            double dq[kDim];
            for (int i = 0; i < kDim; ++i) {
                double s = 0.0;
                for (int r = 0; r < kDim; ++r)
                    s += J[r][i] * rhs[r];
                dq[i] = s;
            }

            double dq_norm = 0.0;
            for (int i = 0; i < kDim; ++i)
                dq_norm += dq[i] * dq[i];
            dq_norm = std::sqrt(dq_norm);
            constexpr double kMaxStep = 0.4; // rad
            if (dq_norm > kMaxStep) {
                const double scale = kMaxStep / dq_norm;
                for (int i = 0; i < kDim; ++i)
                    dq[i] *= scale;
            }

            for (int i = 0; i < kNumJoints; ++i) {
                q[i] += dq[i];
                if (m_->jnt_limited[jid_[i]] && joint_range_[i][0] < joint_range_[i][1])
                    q[i] = std::clamp(q[i], joint_range_[i][0], joint_range_[i][1]);
            }
        }

        res.converged = false;
        res.message = "Reached max iterations";
        return res;
    }

} // namespace arm_kin
