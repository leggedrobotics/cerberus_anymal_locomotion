//
// Created by joonho on 10.06.19.
//

#ifndef OLD_RAI_IK_HPP
#define OLD_RAI_IK_HPP

#include <Eigen/Core>
#include <Eigen/StdVector>

class InverseKinematics {

 public:
  InverseKinematics() {

    std::cout << "Launched the IK constructor!" << std::endl;

    ///Hard code all the shits
    PositionBaseToHipInBaseFrame.resize(4);
    positionHipToThighInHipFrame.resize(4);
    positionThighToShankInThighFrame.resize(4);
    positionShankToFootInShankFrame.resize(4);
    PositionBaseToHAACenterInBaseFrame.resize(4);

    PositionBaseToHipInBaseFrame[0] << 0.277, 0.116, 0.0;
    PositionBaseToHipInBaseFrame[1] << 0.277, -0.116, 0.0;
    PositionBaseToHipInBaseFrame[2] << -0.277, 0.116, 0.0;
    PositionBaseToHipInBaseFrame[3] << -0.277, -0.116, 0.0;

    positionHipToThighInHipFrame[0] << 0.0635, 0.055, 0.0;
    positionHipToThighInHipFrame[1] << 0.0635, -0.055, 0.0;
    positionHipToThighInHipFrame[2] << -0.0635, 0.055, 0.0;
    positionHipToThighInHipFrame[3] << -0.0635, -0.055, 0.0;

    positionThighToShankInThighFrame[0] << 0.0, 0.12205, -0.25;
    positionThighToShankInThighFrame[1] << 0.0, -0.12205, -0.25;
    positionThighToShankInThighFrame[2] << 0.0, 0.12205, -0.25;
    positionThighToShankInThighFrame[3] << 0.0, -0.12205, -0.25;

    positionShankToFootInShankFrame[0] << 0.1, -0.02, -0.298;
    positionShankToFootInShankFrame[1] << 0.1, 0.02, -0.298;
    positionShankToFootInShankFrame[2] << -0.1, -0.02, -0.298;
    positionShankToFootInShankFrame[3] << -0.1, 0.02, -0.298;

    for (size_t i = 0; i < 4; i++) {
      PositionBaseToHAACenterInBaseFrame[i] = PositionBaseToHipInBaseFrame[i];
      PositionBaseToHAACenterInBaseFrame[i][0] += positionHipToThighInHipFrame[i][0];
      d_[i] = positionHipToThighInHipFrame[i][1];
      d_[i] += positionThighToShankInThighFrame[i][1];
      d_[i] += positionShankToFootInShankFrame[i][1];
    }

    a1_squared_ = positionThighToShankInThighFrame[0][0] * positionThighToShankInThighFrame[0][0]
        + positionThighToShankInThighFrame[0][2] * positionThighToShankInThighFrame[0][2];
    a2_squared_ = positionShankToFootInShankFrame[0][0] * positionShankToFootInShankFrame[0][0]
        + positionShankToFootInShankFrame[0][2] * positionShankToFootInShankFrame[0][2];

    minReach_SP = std::abs(sqrt(a1_squared_) - sqrt(a2_squared_)) + 0.1;
    maxReach_SP = sqrt(a1_squared_) + sqrt(a2_squared_) - 0.05;

    minReach = std::sqrt(d_[0] * d_[0] + minReach_SP * minReach_SP);
    maxReach = sqrt(d_[0] * d_[0] + maxReach_SP * maxReach_SP);

    KFEOffset_ = atan2(0.1, 0.298);
  }

  ~InverseKinematics() = default;

  bool solveIK(
      Eigen::Vector3d &legJoints,
      const Eigen::Vector3d &positionBaseToFootInBaseFrame,
      size_t limb) {

    Eigen::Vector3d
        positionHipToFootInBaseFrame = positionBaseToFootInBaseFrame - PositionBaseToHAACenterInBaseFrame[limb];

    const double d = d_[limb];
    const double dSquared = d * d;

    ///Rescaling target

    double reach = positionHipToFootInBaseFrame.norm();
    double positionYzSquared = positionHipToFootInBaseFrame.tail(2).squaredNorm();
    if (reach > maxReach) {
      positionHipToFootInBaseFrame /= reach;
      positionHipToFootInBaseFrame *= maxReach;
      positionYzSquared = positionHipToFootInBaseFrame.tail(2).squaredNorm();
    }

    if (positionYzSquared < minReach * minReach) {
      positionHipToFootInBaseFrame[1] = d;
      positionHipToFootInBaseFrame[2] = 0.0;
      positionYzSquared = d * d;

      double reach_SP = std::abs(positionHipToFootInBaseFrame[0]);

      if (reach_SP > maxReach_SP) {
        positionHipToFootInBaseFrame[0] /= reach_SP;
        positionHipToFootInBaseFrame[0] *= maxReach_SP;
      } else if (reach_SP < minReach_SP) {
        positionHipToFootInBaseFrame[0] /= reach_SP;
        positionHipToFootInBaseFrame[0] *= minReach_SP;
      }

    }

    double rSquared = positionYzSquared - dSquared;
    const double r = std::sqrt(rSquared);
    const double delta = std::atan2(positionHipToFootInBaseFrame.y(),
                                    -positionHipToFootInBaseFrame.z());
    const double beta = std::atan2(r, d);
    const double qHAA = beta + delta - M_PI_2;
    legJoints[0] = qHAA;

    const double l_squared = (rSquared + positionHipToFootInBaseFrame[0] * positionHipToFootInBaseFrame[0]);
    const double phi1 = std::acos((a1_squared_ + l_squared - a2_squared_) * 0.5 / sqrt(a1_squared_ * l_squared));
    const double phi2 = std::acos((a2_squared_ + l_squared - a1_squared_) * 0.5 / sqrt(a2_squared_ * l_squared));

    double qKFE = phi1 + phi2 - KFEOffset_;

    if (limb < 2) {
      qKFE *= -1.0;
    }
    legJoints[2] = qKFE;

    double theta_prime = atan2(positionHipToFootInBaseFrame[0], r);
    double qHFE = phi1 - theta_prime;

    if (limb > 1) {
      qHFE = -phi1 - theta_prime;
    }
    legJoints[1] = qHFE;
    return true;
  }

  double d_[4];

  double a1_squared_;
  double a2_squared_;
  double KFEOffset_;
  double minReach;
  double maxReach;
  double minReach_SP;
  double maxReach_SP;

  std::vector<Eigen::Vector3d> positionHipToThighInHipFrame;
  std::vector<Eigen::Vector3d> positionThighToShankInThighFrame;
  std::vector<Eigen::Vector3d> positionShankToFootInShankFrame;
  std::vector<Eigen::Vector3d> PositionBaseToHipInBaseFrame;
  std::vector<Eigen::Vector3d> PositionBaseToHAACenterInBaseFrame;

};

#endif //OLD_RAI_IK_HPP
