#include "PID.h"
#include <iostream>
#include <random>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    error = 0.0;
    n = 0;

    best_p[0] = Kp;
    best_p[1] = Ki;
    best_p[2] = Kd;
}

void PID::UpdateError(double cte) {
    if (p_error == 0) {
        time_stamp = clock();
    }

    time_t current_time = clock();

    double delta_t = (double(current_time - time_stamp) / CLOCKS_PER_SEC);
    time_stamp = current_time;

    if (delta_t > 0) {
        d_error = (cte - p_error) / delta_t;
    } else {
        d_error = 0;
    }
    p_error = cte;
    i_error += cte * delta_t;
    if(n >= 20) {
        error += cte * cte;
    }
    n++;
}

double PID::TotalError() {
    if (n == 0) {
        return std::numeric_limits<long int>::max();
    }
    return (error / n);
}

double PID::CtrlQuantity() {
    return -((Kp * p_error) + (Ki * i_error) + (Kd * d_error));
}

void PID::setTwiddlePara(std::vector<double> _dp, double _tol) {
    this->dp = _dp;
    tol = _tol;
}
bool PID::Twiddle() {
    double sum = std::accumulate(dp.begin(), dp.end(), 0.0);
    bool flag = true;
    bool run_flag = false;
    double err;
    while ((sum > tol) && (!run_flag)) {
        switch (tunning_state) {
            case 0:
                best_p[tunning_index] += dp[tunning_index];
                tunning_state = 1;
                run_flag = true;
                break;
            case 1:
                err = this->TotalError();
                std::cout << "err " << err << " best_err " << best_err << std::endl;
                if (err < best_err) {
                    best_err = err;
                    dp[tunning_index] *= 1.1;
                    tunning_state = 0;
                    tunning_index++;
                } else {
                    best_p[tunning_index] -= 2 * dp[tunning_index];
                    tunning_state = 2;
                    run_flag = true;
                }
                break;
            case 2:
                err = this->TotalError();
                std::cout << "err " << err << " best_err " << best_err << std::endl;
                if (err < best_err) {
                    best_err = err;
                    dp[tunning_index] *= 1.1;
                } else {
                    best_p[tunning_index] += dp[tunning_index];
                    dp[tunning_index] *= 0.9;
                }
                tunning_state = 0;
                tunning_index++;
                break;
            default:
                std::cout << "tunning_state is wrong !!!" << std::endl;
        }
        if (tunning_index > 2) {
            tunning_index = 0;
        }

        std::cout << "next tunning_index is " << tunning_index << " next tunning_state "
                  << tunning_state << std::endl;
    }
}

void PID::SetPIDWithTwiddlePara(void) {
    std::cout << "PID " << best_p[0] << " " << best_p[1] << " " << best_p[2]
              << "\n";
    // this->Init(best_p[0], best_p[1], best_p[2]);
    Kp = best_p[0];
    Ki = best_p[1];
    Kd = best_p[2];
}