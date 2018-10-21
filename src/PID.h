#ifndef PID_H
#define PID_H

#include <vector>
#include <limits>
#include <ctime>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double error;

  // Iterate counter
  long int n;
  long int n_thres;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the PID Control Quantity
  */
  double CtrlQuantity();

    //track time
  time_t time_stamp;

  /*
  * Twiddle auto tunning
  */

  void setTwiddlePara(std::vector<double> _dp, double tol);
  std::vector<double> best_p = {0,0,0};
  std::vector<double> dp = {0.1,0.01,0.001};
  double best_err = std::numeric_limits<double>::max();
  double tol = 0.001;
  int tunning_state = 0;
  int tunning_index = 0;

  /*
  * Calculate the PID params
  */
  bool Twiddle(void);
  void SetPIDWithTwiddlePara(void);
};

#endif /* PID_H */
