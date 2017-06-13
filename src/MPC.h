#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

#define Lf (2.67)


// Evaluate a polynomial.
template <class T>
T polyeval(Eigen::VectorXd coeffs, T x) {
  T result(0.0);
  for (int i(coeffs.size()-1); i >= 0; --i) {
    result = result * x + coeffs[i];
  }
  return result;
}

// Evaluate first derivative of a polynomial.
template <class T>
T polyeval_D1(Eigen::VectorXd coeffs, T x) {
  T result(0.0);
  for (int i(coeffs.size()-1); i > 0; --i) {
    result = result * x + double(i) * coeffs[i];
  }
  return result;
}


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
