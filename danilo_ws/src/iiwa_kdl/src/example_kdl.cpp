#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <urdf/model.h>
using namespace std;


#ifndef PINOCCHIO_MODEL_DIR
  //#define PINOCCHIO_MODEL_DIR "/home/danilo/Desktop/lbr_iiwa7_r800-urdf-package-master/urdf/lbr_iiwa7_r800.urdf" //"path_to_the_model_dir"
#define PINOCCHIO_MODEL_DIR "/home/danilo/Desktop/iiwa7.urdf"
#endif

		using namespace std;

int main(int argc, char** argv) {
	int i=0;	
	std::string robot_desc_string;
	ros::init(argc, argv, "iiwa_kdl");
	ros::NodeHandle _nh;
	_nh.param("robot_description", robot_desc_string, std::string());
	KDL::Tree iiwa_tree; // tree crea l'albero dei collegamenti del robot
	KDL::Chain _k_chain; // oggetto che si occupa della creazione della struttura del manipolatore, tramite l'uso di segmenti
	if(!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)) ROS_ERROR("Failed to construct kdl tree");
	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) ROS_ERROR("robot not init");


	KDL::Frame _p_out; //rappresenta una rappresentazione 3d del frame
	geometry_msgs::Pose cpose;	

	KDL::JntArray *_q_in; //oggetto che costruisce un vettore dei giunti del manipolatore
	KDL::JntArray *_dq_in;
	KDL::JntArray *_ddq_in;
	KDL::JntArrayVel *_qvel_in;
	KDL::Jacobian J;
	KDL::Jacobian Jdot;	

	KDL::ChainFkSolverPos_recursive *_fksolver; //crea l'oggetto atto a ed effettuare la cinematica diretta, ricorsiva
	KDL::ChainJntToJacSolver *_jsolver;
	KDL::ChainJntToJacDotSolver *_jdotsolver;

	KDL::ChainDynParam *_dyn_param;
	KDL::JntSpaceInertiaMatrix B;
	KDL::JntArray Cdq(7);
  	KDL::JntArray G(7);

//---------------------

//---------------------------------------------------



	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain ); // crea nuovo oggetto cinematica inversa relativo alla catena
	_jsolver = new KDL::ChainJntToJacSolver(_k_chain);
	_jdotsolver = new KDL::ChainJntToJacDotSolver(_k_chain);

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() ); //crea l'array relativo ai giunti
	_dq_in= new KDL::JntArray( _k_chain.getNrOfJoints() );
	_ddq_in= new KDL::JntArray( _k_chain.getNrOfJoints() );
	J.resize(_k_chain.getNrOfJoints());
	B.resize(_k_chain.getNrOfJoints());
	_q_in->data(0)=0.1;
	_q_in->data(1)=0.2;
	_q_in->data(2)=0.3;
	_q_in->data(3)=-1.57;
	_q_in->data(4)=0.4;
	_q_in->data(5)=1.57;
	_q_in->data(6)=0.5;
	_dq_in->data(0)= 0.1;
	_dq_in->data(1)= 0.2;
	_dq_in->data(2)= 0.3;
	_dq_in->data(3)= 0.4;
	_dq_in->data(4)= 0.5;
	_dq_in->data(5)= 0.6;
	_dq_in->data(6)= 0.7;

	_qvel_in= new KDL::JntArrayVel(*_q_in, *_dq_in);
	_dyn_param = new KDL::ChainDynParam(_k_chain,KDL::Vector(0,0,-9.81));


    	_fksolver->JntToCart(*_q_in, _p_out);
    	Eigen::Matrix<double, 3, 1>  xe; 
    	xe << _p_out.p.x(), _p_out.p.y(), _p_out.p.z();

	_jsolver->JntToJac(*_q_in, J);

	_dyn_param->JntToMass(*_q_in, B);//c'era già prima
	_dyn_param->JntToCoriolis(*_q_in, *_dq_in, Cdq);//c'era già prima
	_dyn_param->JntToGravity(*_q_in, G);//c'era già prima
		

		std::cout << "J= " << std::endl;
		std::cout << J.data  << std::endl;
		std::cout << "xe " << std::endl;
		std::cout << xe  << std::endl;
		std::cout << "B= " << std::endl;
		std::cout << B.data << std::endl;
		std::cout << "Cdq= " << std::endl;
		std::cout << Cdq.data << std::endl;
		std::cout << "G= " << std::endl;
		std::cout << G.data << std::endl;

	
}
