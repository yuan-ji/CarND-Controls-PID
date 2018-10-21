#ifndef PID_H
#define PID_H

#include <ctime>
#include <limits>
#include <vector>

class PID {
   public:
    /*
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;
    double error;


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

    // track time
    time_t time_stamp;

    /*
     * Twiddle auto tunning
     */
    // Iterate counter
    long int n;
    long int n_thres;

    std::vector<double> best_p = {0, 0, 0};
    std::vector<double> dp = {0.1, 0.01, 0.001};
    double tol;
    int tunning_state;
    int tunning_index;
    double best_err = std::numeric_limits<double>::max();
    void setTwiddlePara(std::vector<double> _dp, double tol, int _thres);

    /*
     * Calculate the PID params
     */
    void Twiddle(void);
    void SetPIDWithTwiddlePara(double cte);
};

#endif /* PID_H */
