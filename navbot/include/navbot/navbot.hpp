/// \file
/// \brief Library for handling navbot dynamics

#ifndef NAVBOT_INCLUDE_GUARD_HPP
#define NAVBOT_INCLUDE_GUARD_HPP

#include <armadillo>
#include <boost/numeric/odeint.hpp>
#include <cmath>

typedef std::vector<double> state_type;

namespace navbot{

class Navbot{

    double a1, a2, a3, a4, a5, b1, b2, b3, m, g;

    public:
    
    /// \brief default constructor
    Navbot();

    void operator() (const state_type &x, state_type &dxdt, const double /* t */);

    /// \brief
    arma::mat A(arma::mat X);

    /// \brief
    arma::mat B(arma::mat X,double dt);

    arma::mat G();







};

}

#endif
