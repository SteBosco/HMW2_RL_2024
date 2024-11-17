#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::pd_plusgravity(KDL::JntArray &_qd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();
 
    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    // Eigen::VectorXd de = _dqd.data - dq;
 
    return  -_Kd*dq +_Kp*e
             + robot_->getGravity();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
   // calculate gain matrices
   Eigen::Matrix<double,6,6> Kp, Kd;
   Kp=Eigen::MatrixXd::Zero(6,6);
   Kd=Eigen::MatrixXd::Zero(6,6);
   Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
   Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
   Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
   Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

   // read current state
   KDL::Jacobian JEE=robot_->getEEJacobian();           
   Eigen::Matrix<double,6,7> J = toEigen(JEE);                  //added toEigen function in utils.h
   Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();
   Eigen::Matrix<double,7,7> M = robot_->getJsim();
   Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);
   //Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);

   // position
   KDL::Frame cart_pose = robot_->getEEFrame();
   Eigen::Vector3d p_d(_desPos.p.data);
   Eigen::Vector3d p_e(cart_pose.p.data);
   Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
   Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(cart_pose.M.data);
   R_d = matrixOrthonormalization(R_d);
   R_e = matrixOrthonormalization(R_e);

   // velocity
   KDL::Twist cart_twist = robot_->getEEVelocity();
   Eigen::Vector3d dot_p_d(_desVel.vel.data);
   Eigen::Vector3d dot_p_e(cart_twist.vel.data);
   Eigen::Vector3d omega_d(_desVel.rot.data);
   Eigen::Vector3d omega_e(cart_twist.rot.data);

   // acceleration
   Eigen::Matrix<double,6,1> dot_dot_x_d;
   Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
   Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);

   // compute linear errors
   Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
   Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);


   // compute orientation errors
   Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
   Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
                                                                       omega_e,
                                                                       R_d,
                                                                       R_e);
   Eigen::Matrix<double,6,1> x_tilde;
   Eigen::Matrix<double,6,1> dot_x_tilde;
   x_tilde << e_p, e_o;
   dot_x_tilde << dot_e_p, -omega_e;//dot_e_o;
   dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

   // null space control
   double cost;
   Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);


   // inverse dynamics
   Eigen::Matrix<double,6,1> y;

   //nota: getEEJacDotqDot è già della classe eigen::VectorXd, dunque non c'è bisogno di convertirlo
   
   y << dot_dot_x_d - robot_->getEEJacDotqDot()*robot_->getJntVelocities() + Kd*dot_x_tilde + Kp*x_tilde;

   return M * (Jpinv*y + (I-Jpinv*J)*(/*- 10*grad*/ - 1*robot_->getJntVelocities()))
           + robot_->getGravity() + robot_->getCoriolis();
}

