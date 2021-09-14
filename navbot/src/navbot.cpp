#include "navbot/navbot.hpp"

namespace navbot{

    Navbot::Navbot() : a1(1), a2(1), a3(1), a4(1), a5(1), b1(1), b2(1), b3(1), m(1), g(10) {}

    void Navbot::operator() (const state_type &x, state_type &dxdt, const double /* t */){
        dxdt[0] = x[1];
        dxdt[1] = x[3]*x[5]*a1 + x[3]*a2*x[15] + b1*x[13];
        dxdt[2] = x[3];
        dxdt[3] = x[1]*x[5]*a3 - x[1]*a4*x[15] + b2*x[14];
        dxdt[4] = x[5];
        dxdt[5] = x[3]*x[1]*a5 + b3*x[15];
        dxdt[6] = x[7];
        dxdt[7] = g - (cos(x[0])*cos(x[2])*x[12]/m);
        dxdt[8] = x[9];
        dxdt[9] = (cos(x[0])*sin(x[2])*cos(x[4]) + sin(x[0])*sin(x[4])*x[12]/m);
        dxdt[10] = x[11];
        dxdt[11] = (cos(x[0])*sin(x[2])*sin(x[4]) - sin(x[0])*cos(x[4])*x[12]/m);
        dxdt[12] =  0;
        dxdt[13] =  0;
        dxdt[14] =  0;
        dxdt[15] =  0;
    }


    arma::mat Navbot::A(arma::mat X){

        arma::mat out = { {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, a1*X[1] + a2*X[15], 0, a1*X[3], 0, 0, 0, 0, 0, 0, 0, b1, 0, a2*X[3]},
                          {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                          {0, a3*X[5] - a4*X[15], 0, 0, 0, a3*X[1], 0, 0, 0, 0, 0, 0, 0, 0, b2, -a4*X[1]},
                          {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                          {0, a5*X[3], 0, a5*X[1], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b3},
                          {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                          {X[12]*sin(X[0])*cos(X[2])/m, 0, X[12]*sin(X[2])*cos(X[0])/m, 0, 0, 0, 0, 0, 0, 0, 0, 0, -cos(X[0])*cos(X[2])/m, 0, 0, 0},
                          {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
                          {-sin(X[0])*sin(X[2])*cos(X[4]) + X[12]*sin(X[4])*cos(X[0])/m, 0, cos(X[0])*cos(X[4])*cos(X[2]), 0, -sin(X[4])*sin(X[2])*cos(X[0]) + X[12]*sin(X[0])*cos(X[4])/m, 0, 0, 0, 0, 0, 0, 0, sin(X[0])*sin(X[4])/m, 0, 0, 0},
                          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
                          {-sin(X[0])*sin(X[4])*sin(X[2]) - X[12]*cos(X[0])*cos(X[4])/m, 0, sin(X[4])*cos(X[0])*cos(X[2]), 0, sin(X[2])*cos(X[0])*cos(X[4]) + X[12]*sin(X[0])*sin(X[4])/m, 0, 0, 0, 0, 0, 0, 0, -sin(X[0])*cos(X[4])/m, 0, 0, 0},
                          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };
                
        return out;
    }

    arma::mat Navbot::B(arma::mat X, double dt){

        arma::mat out = { {0, 0, 0, 0},
                          {0, b1, 0, a2*X[3]},
                          {0, 0, 0, 0},
                          {0, 0, b2, -a4*X[1]},
                          {0, 0, 0, 0},
                          {0, 0, 0, b3},
                          {0, 0, 0, 0},
                          {-cos(X[0])*cos(X[2])/m, 0, 0, 0},
                          {0, 0, 0, 0},
                          {sin(X[0])*sin(X[4])/m, 0, 0, 0},
                          {0, 0, 0, 0},
                          {-sin(X[0])*cos(X[4])/m, 0, 0, 0},
                          {1/dt, 0, 0, 0},
                          {0, 1/dt, 0, 0},
                          {0, 0, 1/dt, 0},
                          {0, 0, 0, 1/dt} };

        return out;
    }

    arma::mat Navbot::G(){

        arma::mat out = arma::zeros(16,1);
        out[7] = 10;
            
        return out;
    }
}
