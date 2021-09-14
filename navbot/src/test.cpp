/// \file
/// \brief This node simulates the environment that the navbot lives in.
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service

#include "ros/ros.h"
#include "navbot/navbot.hpp"
#include <boost/numeric/odeint.hpp>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    using std::vector;
    // using boost::numeric::odeint;
    using namespace boost::numeric::odeint;

    ros::init(argc, argv, "planner");

    ros::NodeHandle n;

    // publishers and subscribers
    
    ros::Rate loop_rate(10);

    // create navbot object
    navbot::Navbot navbot;

    // declare state lists
    vector<double> cont_phi;
    vector<double> cont_theta;
    vector<double> cont_psi;
    vector<double> cont_z;
    vector<double> cont_x;
    vector<double> cont_y;

    vector<double> disc_phi;
    vector<double> disc_theta;
    vector<double> disc_psi;
    vector<double> disc_z;
    vector<double> disc_x;
    vector<double> disc_y;

    // initialize numerical integrator parameters
    const double dt = 0.01;
    const double T = 0.1;
    state_type x = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    arma::mat X(16,1,arma::fill::zeros);
    arma::mat I(16,16,arma::fill::eye);
    arma::mat du = arma::zeros(4,1);
    arma::mat A;
    arma::mat B;
    arma::mat G = navbot.G();

    // Run integrator on nonlinear system
    for (double t = 0.0; t < T-dt; t += dt){
        cont_phi.push_back(x[0]);
        cont_theta.push_back(x[2]);
        cont_psi.push_back(x[4]);
        cont_z.push_back(x[6]);
        cont_x.push_back(x[8]);
        cont_y.push_back(x[10]);
        integrate(navbot, x, t,t+dt, dt);    
    }
    x[12] = 20;
    x[13] = 10;
    x[14] = 10;
    x[15] = 10;

    for (double t = 0.0; t < T-dt; t += dt){
        cont_phi.push_back(x[0]);
        cont_theta.push_back(x[2]);
        cont_psi.push_back(x[4]);
        cont_z.push_back(x[6]);
        cont_x.push_back(x[8]);
        cont_y.push_back(x[10]);
        integrate(navbot, x, t,t+dt, dt);    
    }

    x[12] = 40;
    x[13] = -5;
    x[14] = 30;
    x[15] = -10;

    for (double t = 0.0; t < T-dt; t += dt){
        cont_phi.push_back(x[0]);
        cont_theta.push_back(x[2]);
        cont_psi.push_back(x[4]);
        cont_z.push_back(x[6]);
        cont_x.push_back(x[8]);
        cont_y.push_back(x[10]);
        integrate(navbot, x, t,t+dt, dt);    
    }

    cont_phi.push_back(x[0]);
    cont_theta.push_back(x[2]);
    cont_psi.push_back(x[4]);
    cont_z.push_back(x[6]);
    cont_x.push_back(x[8]);
    cont_y.push_back(x[10]);

    for (double t = 0.0; t < 3*T; t+=dt){
        if (t < 0.005){
            A = navbot.A(X);
            B = navbot.B(X,dt);
        } else if (fabs(t-0.1) < 0.005){
            A = navbot.A(X);
            B = navbot.B(X,dt);
            du[0] = 20;
            du[1] = 10;
            du[2] = 10;
            du[3] = 10;
        } else if (fabs(t - 0.2) < 0.005){
            A = navbot.A(X);
            B = navbot.B(X,dt);
            du[0] = 20;
            du[1] = -15;
            du[2] = 20;
            du[3] = -20;
        } else{
            du = arma::zeros(4,1);
        }

        disc_phi.push_back(X[0]);
        disc_theta.push_back(X[2]);
        disc_psi.push_back(X[4]);
        disc_z.push_back(X[6]);
        disc_x.push_back(X[8]);
        disc_y.push_back(X[10]);
        X = (I + dt*A)*X + dt*B*du + dt*G;
    }

    disc_phi.push_back(X[0]);
    disc_theta.push_back(X[2]);
    disc_psi.push_back(X[4]);
    disc_z.push_back(X[6]);
    disc_x.push_back(X[8]);
    disc_y.push_back(X[10]);

    double phi_error = 0;
    double theta_error = 0;
    double psi_error = 0;
    double z_error = 0;
    double x_error = 0;
    double y_error = 0;

    for (int i = 0; i< disc_phi.size(); i++){
        // std::cout << cont_z[i] << " " << disc_z[i] << std::endl;
        phi_error += fabs(disc_phi[i] - cont_phi[i])/disc_phi.size();
        theta_error += fabs(disc_theta[i] - cont_theta[i])/disc_phi.size();
        psi_error += fabs(disc_psi[i] - cont_psi[i])/disc_phi.size();
        z_error += fabs(disc_z[i] - cont_z[i])/disc_phi.size();
        x_error += fabs(disc_x[i] - cont_x[i])/disc_phi.size();
        y_error += fabs(disc_y[i] - cont_y[i])/disc_phi.size();
    }

    std::cout << "sizes: " << cont_phi.size() << " " << disc_phi.size() << std::endl;

    std::cout << "Average Phi Error: " << phi_error << std::endl;
    std::cout << "Average Theta Error: " << theta_error << std::endl;
    std::cout << "Average Psi Error: " << psi_error << std::endl;
    std::cout << "Average X Error: " << x_error << std::endl;
    std::cout << "Average Y Error: " << y_error << std::endl;
    std::cout << "Average Z Error: " << z_error << std::endl;


    

    while (ros::ok())
    {

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}