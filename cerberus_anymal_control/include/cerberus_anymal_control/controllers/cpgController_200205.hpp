#ifndef RAI_QUADRUPEDCONTROLLER_HPP
#define RAI_QUADRUPEDCONTROLLER_HPP

#include <Eigen/Core>
#include "string"
#include "SimpleMLPLayer.hpp"
#include <cerberus_anymal_control/utils/IK.hpp>
#include <cerberus_anymal_control/utils/GraphLoader.hpp>
# define M_PI       3.14159265358979323846    /* pi */
# define M_PI_2        1.57079632679489661923    /* pi/2 */

namespace RAI {

class QuadrupedController {
  static constexpr int EstimationDim = 4;
  static constexpr int ObservationDim = 60;
  static constexpr int ObservationStride = 4;
  static constexpr int HistoryLength = 128;
  static constexpr int jointHistoryLength = 20;
  static constexpr unsigned int actionDimension = 16;
  static constexpr int stateDimension = 133;
  static constexpr unsigned int nStack = 40;

  typedef typename Eigen::Matrix<double, 3, 1> AngularVelocity;
  typedef typename Eigen::Matrix<double, 3, 1> LinearVelocity;
  typedef typename Eigen::Matrix<double, 4, 1> Quaternion;
  typedef typename Eigen::Matrix<double, 3, 3> RotationMatrix;
  typedef typename Eigen::Matrix<double, 4, 1> Vector4d;
  typedef typename Eigen::Matrix<double, stateDimension, 1> State;
  typedef typename Eigen::Matrix<double, actionDimension, 1> Action;
  typedef typename Eigen::Matrix<double, ObservationDim, 1> Observation;
  using Dtype = double;
  using JointSpaceVector = Eigen::Matrix<double, 12, 1>;

  typedef typename Eigen::VectorXd VectorXd;

 public:
  QuadrupedController();

  ~QuadrupedController();

  void reset(GraphLoader<float> *graph);

  void updateAction(Action action_in);

  void getDesJointPos(Eigen::Matrix<double, 12, 1> &output,
                      const Eigen::Matrix<double, 19, 1> &generalizedCoordinates,
                      const Eigen::Matrix<double, 18, 1> &generalizedVelocities,
                      double headingVel, double lateralVel, double yawrate);

  void updateHistory(const VectorXd &q,
                     const VectorXd &u);

  static void QuattoEuler(const Quaternion &q, double &roll, double &pitch, double &yaw);

 private:
  double anglediff(double target, double source);

  double anglemod(double a);

  double wrapAngle(double a);

  int fastfloor(double a);

  void conversion_GeneralizedState2LearningState(State &state,
                                                        const VectorXd &q,
                                                        const VectorXd &u);

  RotationMatrix quatToRotMat(Quaternion &q);

  State state_;
  Action action_;
  JointSpaceVector jointNominalConfig;
  State state_unscaled_; /// just memory slot for computation
  State stateOffset_;
  State stateScale_;
  Action actionOffset_;
  Action actionScale_;
  Action scaledAction_;
  Eigen::Matrix<Dtype, 3, 1> e_g;
  Eigen::Matrix<Dtype, 3, 1> x_, y_;
  Eigen::Matrix<Dtype, 3, 1> command_;
  Eigen::Matrix<Dtype, 12 * jointHistoryLength, 1> jointVelHist_, jointPosHist_, tempHist_;
  Eigen::Matrix<float, ObservationDim, nStack> historyBuffer_;
  Observation observationScale_;
  Observation observationOffset_;
  std::vector<Eigen::Matrix<double, 3, 1>> footPos_Target;
  std::vector<Eigen::Matrix<double, 3, 1>> prevfootPos_Target;
  std::vector<Eigen::Matrix<double, 3, 1>> prevfootPos_Target2;
  Eigen::Matrix<Dtype, 12, 1> footPositionOffset_;

  double h0_ = -0.5;
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
  JointSpaceVector jointPositionTarget_;

};

}

#endif //RAI_QUADRUPEDCONTROLLER_HPP
