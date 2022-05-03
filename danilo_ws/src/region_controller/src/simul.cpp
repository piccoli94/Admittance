/* ========================= ICR_Simulation ================ */
/*         All right this code is reserved for the ICAROS
* Desrcption:
This is a controller based on a region control law

Author: Piccoli Danilo
Email: piccoli94@gmail.com
Date: 21/07/21

Revision History:
--------------------------------
Version V1.0: Initial Version

=============================================================== */

//#include "ros/ros.h"
#include <iostream>
//#include "sensor_msgs/JointState.h"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "region_controller.cpp"
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#ifndef PINOCCHIO_MODEL_DIR
  //#define PINOCCHIO_MODEL_DIR "/home/danilo/Desktop/lbr_iiwa7_r800-urdf-package-master/urdf/lbr_iiwa7_r800.urdf" //"path_to_the_model_dir"
#define PINOCCHIO_MODEL_DIR "/home/danilo/Desktop/iiwa_description/urdf/iiwa7.urdf"
//#define PINOCCHIO_MODEL_DIR "/home/danilo/Desktop/RCM_kuka.zip (Unzipped Files)-20210610T091137Z-001/RCM_kuka.zip (Unzipped Files)/RCM_kuka/urdf/iiwa7.urdf"
#endif

 using namespace pinocchio;


void tokenize(std::string const &str, const char* delim, std::vector<std::string> &out) ;


void take_desire_motion(const char* delim, std::string f_name,std::vector<double> &xdes, std::vector<double> &ydes, std::vector<double> &zdes);

void take_theta_cap(std::string theta_name, std::vector<double>  &theta_vec, Eigen::Matrix<double,63,1> &theta_cap);
   
int main(int argc, char ** argv){

    Eigen::Matrix<double,7,1> q;
    Eigen::Matrix<double,7,1> dq;
    Eigen::Matrix<double,7,1> ddq;
    Eigen::Matrix<double,3,1> xdes;
    Eigen::Matrix<double,3,1> dxdes;
    Eigen::Matrix<double,3,1> x_i;
    Eigen::Matrix<double,63,1> theta_cap;
    Eigen::Matrix<double,7,1>  tau;
    Eigen::Matrix<double,7,1>  n;
    Eigen::Matrix<double,7,1> tau_robot;
    double a_h=0.05;
    double a_r=0.1;
    double a_t=0.5;
    double alpha=0.5;
    double k_max=10;
    double ts=0.001;
    double coef_ks=100;
    double coef_l=0.0001;
    Eigen::Matrix<double,7,7> K_s=Eigen::Matrix<double, 7, 7>::Identity()*coef_ks;
    Eigen::Matrix<double,63,63> L=Eigen::Matrix<double, 63, 63>::Identity()*coef_l;
    Eigen::Matrix<double, 7, 1> dqr_old;
    double k_pinvJ=0.0001;
    
    q << 0, 
         0, 
         0, 
        -1.57, 
         0, 
         1.57, 
         0;
    dq << 0,
	  0, 
	  0, 
	  0, 
	  0, 
	  0, 
	  0;

    //xdes << 0.4,0.1,0.614;
    //dxdes << 0,0,0;
    x_i << 0,0,0.126;


    std::vector<double> x_vec;
    std::vector<double> y_vec;
    std::vector<double> z_vec;
    std::vector<double> dx_vec;
    std::vector<double> dy_vec;
    std::vector<double> dz_vec;
    std::vector<double> ddx_vec;
    std::vector<double> ddy_vec;
    std::vector<double> ddz_vec;
    std::vector<double> theta_vec;
    const char* delim = "   "; 
    std::string pos_name="des_pos.txt";
    std::string vel_name="des_vel.txt";
    std::string acc_name="des_acc.txt";
    std::string theta_name="theta_cap_stimata.txt";
    std::string theta_name_2="theta_cap.txt";

 
  



    take_desire_motion(delim,pos_name,x_vec,y_vec,z_vec);
    take_desire_motion(delim,vel_name,dx_vec,dy_vec,dz_vec);
    take_desire_motion(delim,acc_name,ddx_vec,ddy_vec,ddz_vec);
    //take_theta_cap(theta_name_2,theta_vec,theta_cap) ;

    theta_cap << 3.45250000000000,0.0218300000000000,0,0,0.00770300000000000,-0.00388700000000000,0.0208300000000000,0,0.000100000000000000,3.48210000000000,0.0207600000000000,0,-0.00362600000000000,0.0217900000000000,0,0.00779000000000000,0,0.000100000000000000,4.05623000000000,0.0320400000000000,0,0,0.00972000000000000,0.00622700000000000,0.0304200000000000,0,0.000100000000000000,3.48220000000000,0.0217800000000000,0,0,0.0207500000000000,-0.00362500000000000,0.00778500000000000,0,0.000100000000000000,2.16330000000000,0.0128700000000000,0,0,0.00570800000000000,-0.00394600000000000,0.0111200000000000,0,0.000100000000000000,2.34660000000000,0.00650900000000000,0,0,0.00625900000000000,0.000318910000000000,0.00452700000000000,0,0.000100000000000000,3.12900000000000,0.0146400000000000,0.000591200000000000,0,0.0146500000000000,0,0.00287200000000000,0,0.000100000000000000;



  
    int w;
    const std::string urdf_filename =PINOCCHIO_MODEL_DIR;
    Model model;
    pinocchio::urdf::buildModel(urdf_filename,model);
    Data data(model);
    REGION_CONTROLLER controller(x_i, a_h, a_r, a_t, alpha, k_max, ts, K_s, L, k_pinvJ, model,data);

Eigen::Matrix<double,7,1> tau_limits;
Eigen::Matrix<double,7,1> ddq_limits;
Eigen::Matrix<double,7,1> dq_limits;
Eigen::Matrix<double,7,1> q_limits;
tau_limits << 176,176,110,110,110,40,40;
ddq_limits << 10,10,10,10,10,10,10;
dq_limits << 10,10,10,10,10,10,10;
q_limits << 3,3,3,3,3,3,3;
int end_iter=1e+5;
while(w<x_vec.size()){
    xdes << x_vec[w] , y_vec[w], z_vec[w];
    dxdes << dx_vec[w] , dy_vec[w], dz_vec[w];

    controller.torque_control(q,dq,xdes,dxdes,theta_cap,dqr_old,model,data);

    tau=controller.get_tau_m();
    for (int i = 0; i < tau.size(); i++)
    {
        if(tau(i)>tau_limits(i)){
            tau(i)=tau_limits(i);
        }
        else if(tau(i)<-tau_limits(i)){
            tau(i)=-tau_limits(i);
        }  
    }   
   
    //std::cout << "tau_ctrl: "<< std::setprecision(4)<< tau.transpose() << std::endl;
    theta_cap=controller.get_theta();
    //std::cout << "theta_cap: "<< theta_cap.transpose() << std::endl;
    dqr_old=controller.get_qr_dot();
    //std::cout << "get_dqr_old: "<< dqr_old.transpose() << std::endl;

    n=nonLinearEffects(model, data, q, dq);
    crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    //std::cout <<"tau_robot"<< (data.M *ddq + n).transpose() << std::endl;
    
   aba(model,data,q,dq,tau);
   ddq=data.ddq;
    
    for (int j = 0; j < dq.size(); j++)
    {
        if (ddq(j)>ddq_limits(j))
        {
            ddq(j)=ddq_limits(j);
        }
        else if (ddq(j)<-ddq_limits(j))
        {
                ddq(j)=-ddq_limits(j);
        }
        
    }
    dq=dq+ddq*0.001;
    for (int j = 0; j < dq.size(); j++)
    {
        if (dq(j)>dq_limits(j))
        {
            dq(j)=dq_limits(j);
        }
        else if (dq(j)<-dq_limits(j))
        {
                dq(j)=-dq_limits(j);
        }
        
    }
    
    q=q+dq*0.001;
    
    for (int j = 0; j < dq.size(); j++)
    {
        if (q(j)>q_limits(j))
        {
            q(j)=q_limits(j);
        }   
        
        else if (q(j)<-q_limits(j))
            {
                q(j)=-q_limits(j);
            }
   
    }
    tau_robot=data.M *ddq + n;
w++;

if(isnan(q(1))){
std::cout << "Instabile" << std::endl;
return 0;
}



} 

std::ofstream theta_file;
theta_file.open (theta_name_2);
theta_file << theta_cap;
theta_file.close();


}


void take_desire_motion(const char* delim,std::string f_name, std::vector<double> &xdes,std::vector<double> &ydes,std::vector<double> &zdes)
{
  
    std::string line;
    std::ifstream infile(f_name);
        while (std::getline(infile, line)) {
        std::vector<std::string> out; 
        tokenize(line, delim, out); 
        double x,y,z;
        x=atof(out[0].c_str());
        y=atof(out[1].c_str());
        z=atof(out[2].c_str());
        xdes.push_back(x);
        ydes.push_back(y);
        zdes.push_back(z);

    }

}


void tokenize(std::string const &str, const char* delim, 
            std::vector<std::string> &out) 
{ 

    char *token = strtok(const_cast<char*>(str.c_str()), delim); 
    while (token != nullptr) 
    { 
        out.push_back(std::string(token)); 
        token = strtok(nullptr, delim); 
    }
} 

void take_theta_cap(std::string theta_name, std::vector<double>  &theta_vec, Eigen::Matrix<double,63,1> &theta_cap){

    std::string line;
    std::ifstream infile(theta_name);
    double theta=0;
    while (std::getline(infile, line)) {
        theta=atof(line.c_str());
        theta_vec.push_back(theta);
    }
    int j=0;
    for(j=0;j<63;j++){
        theta_cap(j)=theta_vec[j];
}
}