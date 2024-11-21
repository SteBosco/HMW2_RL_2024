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
            + robot_->getCoriolis();
}


Eigen::VectorXd KDLController::idCntr(
    KDL::Frame &_desPos,
    KDL::Twist &_desVel,
    KDL::Twist &_desAcc,
    double _Kpp, double _Kpo,
    double _Kdp, double _Kdo)
{


    // 2. Calcolo delle matrici di guadagno
    Eigen::Matrix<double, 6, 6> Kp = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Matrix<double, 6, 6> Kd = Eigen::MatrixXd::Zero(6, 6);
    Kp.block(0, 0, 3, 3) = _Kpp * Eigen::Matrix3d::Identity();
    Kp.block(3, 3, 3, 3) = _Kpo * Eigen::Matrix3d::Identity();
    Kd.block(0, 0, 3, 3) = _Kdp * Eigen::Matrix3d::Identity();
    Kd.block(3, 3, 3, 3) = _Kdo * Eigen::Matrix3d::Identity();

 
        KDL::Jacobian JEE = robot_->getEEJacobian();
       // Eigen::Matrix<double, 6, 7> J = toEigen(JEE);

        Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
        Eigen::Matrix<double, 7, 7> M = robot_->getJsim();

        // Calcolo pseudoinversa di J
        Eigen::Matrix<double, 7, 6> Jpinv = pseudoinverse(robot_->getEEJacobian().data);

        // Posizione
        KDL::Frame cart_pose = robot_->getEEFrame();
        Eigen::Vector3d p_d(_desPos.p.data);
        Eigen::Vector3d p_e(cart_pose.p.data);
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_d(_desPos.M.data);
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_e(cart_pose.M.data);
        R_d = matrixOrthonormalization(R_d);
        R_e = matrixOrthonormalization(R_e);

        // Velocità
        KDL::Twist cart_twist = robot_->getEEVelocity();
        Eigen::Vector3d dot_p_d(_desVel.vel.data);
        Eigen::Vector3d dot_p_e(cart_twist.vel.data);
        Eigen::Vector3d omega_d(_desVel.rot.data);
        Eigen::Vector3d omega_e(cart_twist.rot.data);

        // Accelerazione
        Eigen::Matrix<double, 6, 1> dot_dot_x_d;
        Eigen::Matrix<double, 3, 1> dot_dot_p_d(_desAcc.vel.data);
        Eigen::Matrix<double, 3, 1> dot_dot_r_d(_desAcc.rot.data);

        // Errori lineari
        Eigen::Matrix<double, 3, 1> e_p = computeLinearError(p_d, p_e);
        Eigen::Matrix<double, 3, 1> dot_e_p = computeLinearError(dot_p_d, dot_p_e);

        // Errori orientamento
        Eigen::Matrix<double, 3, 1> e_o = computeOrientationError(R_d, R_e);
        Eigen::Matrix<double, 3, 1> dot_e_o = computeOrientationVelocityError(
            omega_d, omega_e, R_d, R_e);

        // Costruzione degli stati
        Eigen::Matrix<double, 6, 1> x_tilde;
        Eigen::Matrix<double, 6, 1> dot_x_tilde;
        x_tilde << e_p, e_o;
        dot_x_tilde << dot_e_p, dot_e_o; // Puoi cambiare con dot_e_o se necessario
        dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

        // // Controllo nello spazio nullo
        // double cost;
        // Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(), robot_->getJntLimits(), cost);

        // Inverse Dynamics
        Eigen::Matrix<double, 6, 1> y = dot_dot_x_d - robot_->getEEJacDot().data * robot_->getJntVelocities()
                                        + Kd * dot_x_tilde + Kp * x_tilde;

        // Torques
        return M * (Jpinv * y) + robot_->getCoriolis();

    
}

