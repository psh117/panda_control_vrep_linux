#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>



void ArmController::compute()
{
	// Kinematics and dynamics calculation ------------------------------
	q_temp_ = q_;
	qdot_temp_ = qdot_;

	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], true);
	x_2_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], true);
	
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	Matrix3d body_to_ee_rotation;
	body_to_ee_rotation.setIdentity();
	body_to_ee_rotation(1, 1) = -1;
	body_to_ee_rotation(2, 2) = -1;
	rotation_ = rotation_ * body_to_ee_rotation;
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, true);
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], j_temp_2_, true);


	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);

	g_ = g_temp_;
	m_ = m_temp_;
	m_inverse_ = m_.inverse();

	for (int i = 0; i<2; i++)
	{
		j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
		j_2_.block<3, DOF>(i * 3, 0) = j_temp_2_.block<3, DOF>(3 - i * 3, 0);
	}
	// -----------------------------------------------------
	
	
	// ---------------------------------
	//
	// q_		: joint position
	// qdot_	: joint velocity
	// x_		: end-effector position 
	// j_		: end-effector basic jacobian
	// m_		: mass matrix
	//
	//-------------------------------------------------------------------
	
	j_v_ = j_.block < 3, DOF>(0, 0);

	x_dot_ = j_ * qdot_;
		
	
	
	if (is_mode_changed_)
	{
		is_mode_changed_ = false;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();

		x_init_ = x_;
		x_2_init_ = x_2_;
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
	}

	if (control_mode_ == "joint_ctrl_home")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4;
		moveJointPosition(target_position, 1.0);
	}
	else if(control_mode_ == "joint_ctrl_init")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0;
		moveJointPosition(target_position, 1.0);                     
	}
	else if (control_mode_ == "simple_jacobian")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		simpleJacobianControl(target_x, 2.0);
	}
	else if (control_mode_ == "feedback_jacobian")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		feedbackJacobianControl(target_x, 2.0);
	}
	else if (control_mode_ == "CLIK")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		CLIK(target_x, 2.0);
	}
	else
	{
		torque_desired_ = g_;
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}

void ArmController::record(int file_number, double duration)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] << x_.transpose() <<
			Map< Matrix<double, 1, 9> >(rotation_.data(), rotation_.size()) << x_cubic_.transpose() <<
			endl;
	}
}

void ArmController::record(int file_number, double duration, const stringstream & ss)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] << ss.str() << endl;
	}
}

void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 50.)
	{
		DBG_CNT = 0;

		cout << "q now    :\t";
		cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		cout << "q desired:\t";
		cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		cout << "t desired:\t";
		cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << endl;
		cout << "x        :\t";
		cout << x_.transpose() << endl;
		cout << "R        :\t" << endl;
		cout << std::fixed << std::setprecision(3) << rotation_ << endl;

		Matrix<double, 6, 7>j;
		j <<  j_.block <3, DOF>(0, 0),
		  j_2_.block<3, DOF>(0, 0); 
		  
		cout << "hw 3-1 jacobian:" << endl;
		cout << j << endl;

		Vector6d x;
		x << x_, x_2_;

		cout << "hw 3-1 x:\t" << x.transpose() << endl;
	}
}

void ArmController::moveJointPosition(const Vector7d & target_position, double duration)
{
	Vector7d zero_vector;
	zero_vector.setZero();
	q_desired_ = DyrosMath::cubicVector<7>(play_time_,
		control_start_time_,
		control_start_time_ + duration, q_init_, target_position, zero_vector, zero_vector);
}

void ArmController::moveJointPositionTorque(const Vector7d &target_position, double duration)
{
	Matrix7d kp, kv;
	Vector7d q_cubic, qd_cubic;
	
	kp = Matrix7d::Identity() * 500.0;
	kv = Matrix7d::Identity() * 20.0;

	for (int i = 0; i < 7; i++)
	{
		qd_cubic(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
		q_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
	}


	torque_desired_ = m_ * (kp*(q_cubic - q_) + kv*(qd_cubic - qdot_)) + g_;
}

void ArmController::simpleJacobianControl(const Vector12d & target_x, double duration)
{
	Vector6d xd_desired;
	for (int i = 0; i < 3; i++)
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i*3);
	}
	xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	// debug_file_ << xd_desired.transpose() << endl;
	// xd_desired.segment<3>(3).setZero();
	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * xd_desired;
	
	q_desired_ = q_desired_ + qd_desired / hz_;
	record(0, duration);
}

void ArmController::feedbackJacobianControl(const Vector12d & target_x, double duration)
{
	Vector6d delta_x_desired;
	Vector3d x_cubic;
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}

	for (int i = 0; i < 3; i++)
	{
		x_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	Matrix3d rotation_cubic = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	delta_x_desired.segment<3>(0) = x_cubic - x_;
	delta_x_desired.segment<3>(3) = - 0.5 * DyrosMath::getPhi(rotation_, rotation_cubic) ;

	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * delta_x_desired;

	q_desired_ = q_ + qd_desired;

	stringstream ss;
	ss << x_cubic.transpose() <<
		Map< Matrix<double, 1, 9> >(rotation_cubic.data(), rotation_cubic.size());
	record(1, duration);
	record(3, duration, ss);
}


void ArmController::CLIK(const Vector12d & target_x, double duration)
{
	Vector6d xd_desired, x_error;
	Vector3d x_cubic;

	for (int i = 0; i < 3; i++)
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}
	xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	for (int i = 0; i < 3; i++)
	{
		x_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	Matrix3d rotation_cubic = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	x_error.segment<3>(0) = x_cubic - x_;
	x_error.segment<3>(3) = -0.5 * DyrosMath::getPhi(rotation_, rotation_cubic);

	// debug_file_ << xd_desired.transpose() << endl;
	// xd_desired.segment<3>(3).setZero();
	Matrix6d kp;
	kp.setIdentity();
	kp = kp * hz_ * 1.5;
	//Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * (xd_desired + kp * x_error);

	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * xd_desired;
	q_desired_ = q_desired_ + qd_desired / hz_;
	record(2, duration);
}

// Controller Core Methods ----------------------------

void ArmController::setMode(const std::string & mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}
void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);

	j_temp_2_.resize(6, DOF);
	j_temp_2_.setZero();
}

void ArmController::initModel()
{
    model_ = make_shared<Model>();

    model_->gravity = Vector3d(0., 0, -GRAVITY);

    double mass[DOF];
    mass[0] = 1.0;
    mass[1] = 1.0;
    mass[2] = 1.0;
    mass[3] = 1.0;
    mass[4] = 1.0;
    mass[5] = 1.0;
    mass[6] = 1.0;

    Vector3d axis[DOF];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0*Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0*Eigen::Vector3d::UnitY();
	axis[6] = -1.0*Eigen::Vector3d::UnitZ();


	Eigen::Vector3d global_joint_position[DOF];

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

	for (int i = 0; i < DOF; i++)
		com_position_[i] -= global_joint_position[i];

    Math::Vector3d inertia[DOF];
	for (int i = 0; i < DOF; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

    for (int i = 0; i < DOF; i++) {
        body_[i] = Body(mass[i], com_position_[i], inertia[i]);
        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0)
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
        else
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    }
}

void ArmController::initFile()
{
	debug_file_.open("debug.txt");
	for (int i = 0; i < NUM_HW_PLOT; i++)
	{
		hw_plot_files_[i].open(hw_plot_file_names_[i] + ".txt");
	}
}

void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

const Vector7d & ArmController::getDesiredPosition()
{
	return q_desired_;
}

const Vector7d & ArmController::getDesiredTorque()
{
	return torque_desired_;
}



void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
}

// ----------------------------------------------------

