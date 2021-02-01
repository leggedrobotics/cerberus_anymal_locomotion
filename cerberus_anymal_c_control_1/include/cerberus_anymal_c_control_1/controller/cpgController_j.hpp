#ifndef RAI_QUADRUPEDCONTROLLER_HPP
#define RAI_QUADRUPEDCONTROLLER_HPP

#include <ros/console.h>
#include <Eigen/Core>
#include "string"
#include "cerberus_anymal_utils/SimpleMLPLayer.hpp"
#include "cerberus_anymal_utils/IK.hpp"
#include "cerberus_anymal_utils/GraphLoader.hpp"
# define M_PI       3.14159265358979323846    /* pi */
# define M_PI_2        1.57079632679489661923    /* pi/2 */

namespace RAI {

class QuadrupedController {
  using Dtype = float;
  static constexpr int EstimationDim = 4;
  static constexpr int ObservationDim = 60;
  static constexpr int HistoryLength = 128;
  static constexpr int jointHistoryLength = 20;
  static constexpr unsigned int actionDimension = 16;
  static constexpr int stateDimension = 133;

  typedef typename Eigen::Matrix<double, 3, 1> AngularVelocity;
  typedef typename Eigen::Matrix<double, 3, 1> LinearVelocity;
  typedef typename Eigen::Matrix<double, 4, 1> Quaternion;
  typedef typename Eigen::Matrix<double, 3, 3> RotationMatrix;
  typedef typename Eigen::Matrix<double, 4, 1> Vector4d;
  typedef typename Eigen::Matrix<double, actionDimension, 1> Action;
  typedef typename Eigen::Matrix<Dtype, ObservationDim, 1> Observation;
  typedef typename Eigen::Matrix<Dtype, stateDimension, 1> State;

  typedef typename Eigen::VectorXd VectorXd;
  typedef typename Eigen::Matrix<Dtype, -1, 1> VectorXD;

 public:
  QuadrupedController() {

    footPositionOffset_ <<
                        0.3 + 0.1, 0.2, h0_,
        0.3 + 0.1, -0.2, h0_,
        -0.3 - 0.1, 0.2, h0_,
        -0.3 - 0.1, -0.2, h0_;

    jointNominalConfig_.setZero(12);
    Eigen::Vector3d sol;
    for (int i = 0; i < 4; i++) {
      IK_.solveIK(sol, footPositionOffset_.segment(3 * i, 3), i);
      jointNominalConfig_.segment(3 * i, 3) = sol;
    }

    observationOffset_ << 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, /// gravity axis
        VectorXD::Constant(6, 0.0),
        jointNominalConfig_.template cast<Dtype>(),
        VectorXD::Constant(12, 0.0),
        VectorXD::Constant(12, 0.0),
        VectorXD::Constant(8, 0.0),
        VectorXD::Constant(4, 0.0);

    observationScale_ <<
                      1.5, 1.5, 1.5, /// command
        5.0, 5.0, 5.0, /// gravity axis
        VectorXD::Constant(3, 2.0),
        VectorXD::Constant(3, 2.0),
        VectorXD::Constant(12, 2.0), /// joint angles
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        6.5, 4.5, 3.5,
        6.5, 4.5, 3.5,
        6.5, 4.5, 3.5,
        6.5, 4.5, 3.5,
        VectorXD::Constant(8, 1.5),
        VectorXD::Constant(4, 2.0 / freqScale_);



    /// state params
    stateOffset_ << 0.0, 0.0, 0.0, /// command
        0.0, 0.0, 1.0, /// gravity axis
        VectorXD::Constant(6, 0.0), /// body lin/ang vel
        jointNominalConfig_.template cast<Dtype>(), /// joint position
        VectorXD::Constant(12, 0.0), /// position error
        VectorXD::Constant(12, 0.0), /// position error
        VectorXD::Constant(4, 0.0),
        VectorXD::Constant(8, 0.0),
        VectorXD::Constant(24, 0.0),
        VectorXD::Constant(24, 0.0),
        jointNominalConfig_.template cast<Dtype>(),
        jointNominalConfig_.template cast<Dtype>(),
        0.0;

    stateScale_ << 1.5, 1.5, 1.5, /// command
        5.0, 5.0, 5.0, /// gravity axis
        VectorXD::Constant(3, 2.0),
        VectorXD::Constant(3, 2.0),
        VectorXD::Constant(12, 2.0), /// joint angles
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        6.5, 4.5, 3.5,
        6.5, 4.5, 3.5,
        6.5, 4.5, 3.5,
        6.5, 4.5, 3.5,
        VectorXD::Constant(4, 2.0 / freqScale_),
        VectorXD::Constant(8, 1.5),
        VectorXD::Constant(24, 5.0), /// joint position errors
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        0.5, 0.4, 0.3,
        VectorXD::Constant(12, 2.0),
        VectorXD::Constant(12, 2.0),
        2.0 / freqScale_;

    double amp1 = 0.5 * freqScale_, amp2 = 0.0;

    baseFreq_ = 1.3 * freqScale_;

    actionScale_ <<
                 amp1, amp1, amp1, amp1,
        VectorXd::Constant(12, 0.2);

    actionOffset_ <<
                  amp2, amp2, amp2, amp2,
        VectorXd::Constant(12, 0.0);

    for (size_t i = 0; i < 4; i++) {
      clearance_[i] = 0.2;
    }

    footPos_Target.resize(4);
    prevfootPos_Target.resize(4);
    prevfootPos_Target2.resize(4);

    //tensor config
    state_dims_inv.AddDim(1);
    state_dims_inv.AddDim(1);
    state_dims_inv.AddDim(stateDimension);
    state_tf_tensor = tensorflow::Tensor(tensorflow::DataType::DT_FLOAT, state_dims_inv);

    for (size_t i = 0; i < 4; i++) {
      piD_[i] = 0.0;
      footPos_Target[i] = footPositionOffset_.template segment<3>(3 * i);
      prevfootPos_Target[i] = footPositionOffset_.template segment<3>(3 * i);
      prevfootPos_Target2[i] = footPositionOffset_.template segment<3>(3 * i);
    }

    pi_[0] = 0.0;
    pi_[1] = M_PI;
    pi_[2] = M_PI;
    pi_[3] = 0.0;

    controlCounter = 0;
    stopCounter = 1000;
    baseFreq_ = 0.0;
    historyUpdateCounter = 0;
    scaledAction_ = actionOffset_;
    jointVelHist_.setZero();
    jointPosHist_.setZero();
    historyBuffer_.setZero();

    jointPositionTarget_ = jointNominalConfig_;
    previousjointPositionTarget_ = jointPositionTarget_;
    previousjointPositionTarget2_ = jointPositionTarget_;

    command_.setZero();
  }

  ~QuadrupedController() {}

  void reset(GraphLoader<Dtype> *graph,
             const Eigen::Matrix<double, 19, 1> &generalizedCoordinates,
             const Eigen::Matrix<double, 18, 1> &generalizedVelocities) { // = init()

    graph_ = graph;

    if (history_dims_inv.dims() != 3) {
      history_dims_inv.AddDim(1);
      history_dims_inv.AddDim(nStack_);
      history_dims_inv.AddDim(ObservationDim);
      history_tf_tensor = tensorflow::Tensor(tensorflow::DataType::DT_FLOAT, history_dims_inv);
      historyBuffer_.resize(ObservationDim, nStack_);
    }

    for (size_t i = 0; i < 4; i++) {
      piD_[i] = 0.0;
      footPos_Target[i] = footPositionOffset_.template segment<3>(3 * i);
      prevfootPos_Target[i] = footPositionOffset_.template segment<3>(3 * i);
      prevfootPos_Target2[i] = footPositionOffset_.template segment<3>(3 * i);
    }

    pi_[0] = 0.0;
    pi_[1] = M_PI;
    pi_[2] = M_PI;
    pi_[3] = 0.0;

    controlCounter = 0;
    stopCounter = 1000;
    baseFreq_ = 0.0;
    historyUpdateCounter = 0;
    scaledAction_ = actionOffset_;
    jointVelHist_.setZero();
    jointPosHist_.setZero();
    historyBuffer_.setZero();
    jointPositionTarget_ = jointNominalConfig_;
    previousjointPositionTarget_ = jointPositionTarget_;
    previousjointPositionTarget2_ = jointPositionTarget_;
    observation_unscaled_.setZero();
    state_unscaled_.setZero();

    command_.setZero();

    ///dummy call for memory allocation
    std::vector<tensorflow::Tensor> outputs;
    graph_->run({std::make_pair("state", state_tf_tensor), std::make_pair("history", history_tf_tensor)},
                {"policy/actionDist"},
                {},
                outputs);

    /// fill in history buffer
    updateHistory(generalizedCoordinates,
                  generalizedVelocities);
    for (int i = 1; i < nStack_; i++) {
      historyBuffer_.col(i) = historyBuffer_.col(0);
    }
  }

  void updateAction(Action action_in) {
    /// update buffer
    for (size_t i = 0; i < 4; i++) {
      prevfootPos_Target2[i] = prevfootPos_Target[i];
      prevfootPos_Target[i] = footPos_Target[i];
    }

    previousjointPositionTarget2_ = previousjointPositionTarget_;
    previousjointPositionTarget_ = jointPositionTarget_;

    scaledAction_ = action_in.cwiseProduct(actionScale_) + actionOffset_;

    /// update cpg
    for (size_t j = 0; j < 4; j++) {
      piD_[j] = scaledAction_[j] + baseFreq_;
      pi_[j] += piD_[j] * decimation;
      pi_[j] = anglemod(pi_[j]);
    }

    y_ << 0.0, e_g[2], -e_g[1];
    y_.normalize();
    x_ = y_.cross(e_g); // e_g cross y_;
    x_.normalize();

    for (size_t i = 0; i < 4; i++) {
      double dh = 0.0;
      if (pi_[i] > 0.0) {
        double t = pi_[i] / M_PI_2;
        if (t < 1.0) {
          double t2 = t * t;
          double t3 = t2 * t;
          dh = (-2 * t3 + 3 * t2);
        } else {
          t = t - 1;
          double t2 = t * t;
          double t3 = t2 * t;
          dh = (2 * t3 - 3 * t2 + 1.0);
        }
        dh *= clearance_[i];
      }

      footPos_Target[i][0] = footPositionOffset_.tail(12)[3 * i];
      footPos_Target[i][1] = footPositionOffset_.tail(12)[3 * i + 1];
      footPos_Target[i][2] = 0.0;
      footPos_Target[i] += e_g * (footPositionOffset_.tail(12)[3 * i + 2] + dh);
      // footPos_Target[i] += e_g * scaledAction_.tail(12)[3 * i + 2];
      // footPos_Target[i] += x_ * scaledAction_.tail(12)[3 * i];
      // footPos_Target[i] += y_ * scaledAction_.tail(12)[3 * i + 1];


      Eigen::Vector3d target;
      Eigen::Vector3d sol;
      target = footPos_Target[i];
//
      IK_.solveIK(sol, target, i);
      jointPositionTarget_.segment<3>(3 * i) = sol;
      jointPositionTarget_(3 * i) += scaledAction_.tail(12)[3 * i];;
      jointPositionTarget_(3 * i + 1) += scaledAction_.tail(12)[3 * i + 1];
      jointPositionTarget_(3 * i + 2) += scaledAction_.tail(12)[3 * i + 2];;
    }
  }

  void getDesJointPos(Eigen::Matrix<double, 12, 1> &output,
                      const Eigen::Matrix<double, 19, 1> &generalizedCoordinates,
                      const Eigen::Matrix<double, 18, 1> &generalizedVelocities,
                      double headingVel, double lateralVel, double yawrate) {
    double norm = std::sqrt(headingVel * headingVel + lateralVel * lateralVel + yawrate * yawrate);

    if (norm < 0.05) {
      command_.setZero();
    } else {
      double linvelNorm = std::sqrt(headingVel * headingVel + lateralVel * lateralVel);
      command_.setZero();
      if (linvelNorm > 0.2) {
        double command_dir = std::atan2(lateralVel, headingVel);
        command_[0] = std::cos(command_dir);
        command_[1] = std::sin(command_dir);
      }

      command_[2] = std::max(std::min(yawrate, 0.8), -0.8);
    }

    if (command_.norm() > 0.1 || generalizedVelocities.head(2).norm() > 0.3 || e_g[2] < 0.5) {
      if (stopCounter > 1200) { // While standin -> reset phase to react faster (unsmooth)
        pi_[0] = 0.0;
        pi_[1] = M_PI;
        pi_[2] = M_PI;
        pi_[3] = 0.0;
      }
      stopCounter = 0;
    } else {
      stopCounter++;
    }

    if (historyUpdateCounter % ObservationStride_ == 0) {
      updateHistory(generalizedCoordinates,
                    generalizedVelocities);
      historyUpdateCounter = 0;
    }

    if (controlCounter % decimation == 0) { // 8: 50 Hz

      // FSM
      if (stopCounter > 75) {
        bool check = true;
        for (int i = 0; i < 4; i++) {// Hack for nice looking stop
          if ((pi_[i] > -0.1 * M_PI) && (pi_[i] < 0.5 * M_PI)) check = false;
        }

        if (check || baseFreq_ < 1.3 * freqScale_) {
//        baseFreq_ = std::max(baseFreq_ - 0.05 * freqScale_, 0.0);
          baseFreq_ = 0.0;
        }
      } else {
//        baseFreq_ = std::min(baseFreq_ + 0.05 * freqScale_, 1.3 * freqScale_);
        baseFreq_ = 1.3 * freqScale_;
      }

      /// Get state
      conversion_GeneralizedState2LearningState(state_,
                                                generalizedCoordinates,
                                                generalizedVelocities);

      Eigen::Matrix<Dtype, actionDimension, 1> action_f_;
      std::memcpy(state_tf_tensor.template flat<Dtype>().data(),
                  state_.data(),
                  sizeof(Dtype) * stateDimension);
      std::memcpy(history_tf_tensor.template flat<Dtype>().data(),
                  historyBuffer_.data(),
                  sizeof(Dtype) * history_tf_tensor.NumElements());

      std::vector<tensorflow::Tensor> outputs;
      graph_->run({std::make_pair("state", state_tf_tensor), std::make_pair("history", history_tf_tensor)},
                  {"policy/actionDist"},
                  {},
                  outputs);

      std::memcpy(action_f_.data(),
                  outputs[0].template flat<Dtype>().data(),
                  sizeof(Dtype) * actionDimension);

      action_ = action_f_.template cast<double>();

      // std::cout << action_ << std::endl;
      /// update action
      updateAction(action_);
      controlCounter = 0;
    }

    // for (size_t i = 0; i < 4; i++) {
    //   Eigen::Vector3d target = (controlCounter + 1) * footPos_Target[i];
    //   target += (decimation - 1 - controlCounter) * prevfootPos_Target[i];
    //   target *= 0.125;

    //   Eigen::Vector3d sol;
    //   IK_.solveIK(sol, target, i);
    //   jointPositionTarget_.segment<3>(3 * i) = sol;
    // }
    output = jointPositionTarget_;

    tempHist_ = jointVelHist_;
    jointVelHist_.head(jointHistoryLength * 12 - 12) = tempHist_.tail(jointHistoryLength * 12 - 12);
    jointVelHist_.tail(12) = generalizedVelocities.tail(12).template cast<Dtype>();

    tempHist_ = jointPosHist_;
    jointPosHist_.head(jointHistoryLength * 12 - 12) = tempHist_.tail(jointHistoryLength * 12 - 12);
    jointPosHist_.tail(12) = (jointPositionTarget_ - generalizedCoordinates.tail(12)).template cast<Dtype>();

    controlCounter++;
    historyUpdateCounter++;
  };

  void updateHistory(const VectorXd &q,
                     const VectorXd &u) {
    Observation observation_scaled;

    Quaternion quat = q.template segment<4>(3);
    RotationMatrix R_b = quatToRotMat(quat);
    e_g = R_b.row(2).transpose();

    /// velocity in body coordinate
    LinearVelocity bodyVel = R_b.transpose() * u.template segment<3>(0);
    AngularVelocity bodyAngVel = u.template segment<3>(3);
    VectorXd jointVel = u.template segment<12>(6);

    observation_unscaled_[0] = command_[0];
    observation_unscaled_[1] = command_[1];
    observation_unscaled_[2] = command_[2];
    observation_unscaled_.template segment<3>(3) = e_g.template cast<Dtype>();;
    observation_unscaled_.template segment<3>(6) = bodyVel.template cast<Dtype>();
    observation_unscaled_.template segment<3>(9) = bodyAngVel.template cast<Dtype>();
    observation_unscaled_.template segment<12>(12) = q.tail(12).
        template cast<Dtype>(); /// position
    observation_unscaled_.template segment<12>(24) = jointVel.template cast<Dtype>();
    observation_unscaled_.template segment<12>(36) = (jointPositionTarget_ - q.tail(12)).template cast<Dtype>();
    for (size_t i = 0; i < 4; i++) {
      observation_unscaled_[48 + 2 * i] = std::sin(pi_[i]);
      observation_unscaled_[49 + 2 * i] = std::cos(pi_[i]);
      observation_unscaled_[56 + i] = piD_[i];
    }

    observation_scaled = (observation_unscaled_ - observationOffset_).cwiseProduct(observationScale_);

    Eigen::Matrix<Dtype, ObservationDim, -1> temp;
    temp = historyBuffer_;

    historyBuffer_.block(0, 1, ObservationDim, nStack_ - 1) = temp.block(0, 0, ObservationDim, nStack_ - 1);
    historyBuffer_.col(0) = observation_scaled.template cast<Dtype>();
  }

  static inline void QuattoEuler(const Quaternion &q, double &roll, double &pitch, double &yaw) {
    double ysqr = q[2] * q[2];

    // roll (x-axis rotation)
    double t0 = +2.0 * (q[0] * q[1] + q[2] * q[3]);
    double t1 = +1.0 - 2.0 * (q[1] * q[1] + ysqr);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q[0] * q[2] - q[3] * q[1]);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q[0] * q[3] + q[1] * q[2]);
    double t4 = +1.0 - 2.0 * (ysqr + q[3] * q[3]);
    yaw = std::atan2(t3, t4);
  }

 private:
  inline double anglediff(double target, double source) {
    //https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    return anglemod(target - source);
  }

  inline double anglemod(double a) {
    return wrapAngle((a + M_PI)) - M_PI;
  }

  inline double wrapAngle(double a) {
    double twopi = 2.0 * M_PI;
    return a - twopi * fastfloor(a / twopi);
  }

  inline int fastfloor(double a) {
    int i = int(a);
    if (i > a) i--;
    return i;
  }

  inline void conversion_GeneralizedState2LearningState(State &state,
                                                        const VectorXd &q,
                                                        const VectorXd &u) {
    Quaternion quat = q.template segment<4>(3);
    RotationMatrix R_b = quatToRotMat(quat);
    e_g = R_b.row(2).transpose();

    /// velocity in body coordinate
    LinearVelocity bodyVel = R_b.transpose() * u.template segment<3>(0);
    AngularVelocity bodyAngVel = u.template segment<3>(3);
    VectorXd jointVel = u.template segment<12>(6);

    /// command slots
    state_unscaled_[0] = command_[0];
    state_unscaled_[1] = command_[1];
    state_unscaled_[2] = command_[2];

    /// gravity vector
    state_unscaled_.template segment<3>(3) = e_g.template cast<Dtype>();

    /// velocities
    state_unscaled_.template segment<3>(6) = bodyVel.template cast<Dtype>();
    state_unscaled_.template segment<3>(9) = bodyAngVel.template cast<Dtype>();

    state_unscaled_.template segment<12>(12) = q.template segment<12>(7).
        template cast<Dtype>();
    state_unscaled_.template segment<12>(24) = jointVel.template cast<Dtype>();
    state_unscaled_.template segment<12>(36) = jointPosHist_.template segment<12>((jointHistoryLength - 1) * 12);

    for (size_t i = 0; i < 4; i++) {
      state_unscaled_[48 + i] = piD_[i];
    }

    int pos = 52;
    for (size_t i = 0; i < 4; i++) {
      state_unscaled_[pos + 2 * i] = std::sin(pi_[i]);
      state_unscaled_[pos + 1 + 2 * i] = std::cos(pi_[i]);
    }
    pos += 8;

    state_unscaled_.template segment<12>(pos) =
        jointPosHist_.template segment<12>((jointHistoryLength - 4) * 12);
    pos += 12;

    state_unscaled_.template segment<12>(pos) =
        jointPosHist_.template segment<12>((jointHistoryLength - 9) * 12);
    pos += 12;

    state_unscaled_.template segment<12>(pos) =
        jointVelHist_.template segment<12>((jointHistoryLength - 4) * 12);
    pos += 12;

    state_unscaled_.template segment<12>(pos) =
        jointVelHist_.template segment<12>((jointHistoryLength - 9) * 12);
    pos += 12;

    state_unscaled_.template segment<12>(pos) = previousjointPositionTarget_.template cast<Dtype>();
    pos += 12;

    state_unscaled_.template segment<12>(pos) = previousjointPositionTarget2_.template cast<Dtype>();
    pos += 12;

    state_unscaled_[pos] = baseFreq_;
    state = (state_unscaled_ - stateOffset_).cwiseProduct(stateScale_);
  }

  inline RotationMatrix quatToRotMat(Quaternion &q) {
    RotationMatrix R;
    R << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3), 2 * q(0) * q(2)
        + 2 * q(1) * q(3),
        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3), 2 * q(2) * q(3)
        - 2 * q(0) * q(1),
        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3), q(0) * q(0) - q(1) * q(1) - q(2) * q(2)
        + q(3) * q(3);
    return R;
  }

  template<typename T>
  int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  Eigen::Matrix<double, 12, 1> jointNominalConfig_;

  State stateOffset_;
  State stateScale_;
  Action actionOffset_;
  Action actionScale_;
  Action scaledAction_;
  Eigen::Matrix<double, 3, 1> e_g;
  Eigen::Matrix<double, 3, 1> x_, y_;
  Eigen::Matrix<Dtype, 3, 1> command_;
  Eigen::Matrix<Dtype, 12 * jointHistoryLength, 1> jointVelHist_, jointPosHist_, tempHist_;
  Observation observationScale_;
  Observation observationOffset_;
  std::vector<Eigen::Matrix<double, 3, 1>> footPos_Target;
  std::vector<Eigen::Matrix<double, 3, 1>> prevfootPos_Target;
  std::vector<Eigen::Matrix<double, 3, 1>> prevfootPos_Target2;
  Eigen::Matrix<double, 12, 1> footPositionOffset_;
  Eigen::Matrix<float, ObservationDim, -1> historyBuffer_;

  double h0_ = -0.55;
  double clearance_[4];

  double freqScale_ = 0.0025 * 2.0 * M_PI;
  double baseFreq_ = 0.0;

  unsigned long int controlCounter = 0;
  unsigned long int stopCounter = 0;
  unsigned long int historyUpdateCounter = 0;
  unsigned long int decimation = 8;

  InverseKinematics IK_;
  tensorflow::TensorShape state_dims_inv;
  tensorflow::Tensor state_tf_tensor;

  tensorflow::TensorShape history_dims_inv;
  tensorflow::Tensor history_tf_tensor;
  GraphLoader<float> *graph_;

 public:
  double pi_[4];
  double piD_[4];
  Action action_;
  State state_;
  Observation observation_unscaled_;
  State state_unscaled_;

  Eigen::Matrix<double, 12, 1> jointPositionTarget_;

  Eigen::Matrix<double, 12, 1> previousjointPositionTarget_;
  Eigen::Matrix<double, 12, 1> previousjointPositionTarget2_;
  int ObservationStride_ = 8;
  int nStack_ = 100;

};

}

#endif //RAI_QUADRUPEDCONTROLLER_HPP
