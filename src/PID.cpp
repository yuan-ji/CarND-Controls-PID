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
    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;
    error += cte * cte;
    n++;
}

double PID::TotalError() {
    if (n == 0) {
        return std::numeric_limits<long int>::max();
    }
    return (error / n);
}

double PID::CtrlQuantity() {
    return -((Kp * p_error) + (Kd * d_error) + (Ki * i_error));
}

bool PID::Twiddle() {
    double sum = std::accumulate(dp.begin(), dp.end(), 0.0);
    bool flag = true;
    if (sum > tol) {
        double err = this->TotalError();
        switch (tunning_state) {
            case 0:
                best_p[tunning_index] += dp[tunning_index];
                tunning_state = 1;
                break;
            case 1:
                if (err < best_err) {
                    best_err = err;
                    dp[tunning_index] *= 1.1;
                    tunning_state = 0;
                    tunning_index++;
                } else {
                    best_p[tunning_index] -= 2 * dp[tunning_index];
                    tunning_state = 2;
                }
                break;
            case 2:
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
        flag = true;
    } else {
        flag = false;
    }

    std::cout << "tunning_index is " << tunning_index << " tunning_state "
              << tunning_state << " flag is " << flag << std::endl;
    return flag;
}

void PID::SetPIDWithTwiddlePara(void) {
    std::cout << "PID " << best_p[0] << " " << best_p[1] << " " << best_p[2]
              << "\n";
    this->Init(best_p[0], best_p[1], best_p[2]);
}

// def twiddle(tol=0.2):
//     p = [0, 0, 0]
//     dp = [1, 1, 1]
//     robot = make_robot()
//     x_trajectory, y_trajectory, best_err = run(robot, p)

//     it = 0
//     while sum(dp) > tol:
//         print("Iteration {}, best error = {}".format(it, best_err))
//         for i in range(len(p)):
//             p[i] += dp[i]
//             robot = make_robot()
//             x_trajectory, y_trajectory, err = run(robot, p)

//             if err < best_err:
//                 best_err = err
//                 dp[i] *= 1.1
//             else:
//                 p[i] -= 2 * dp[i]
//                 robot = make_robot()
//                 x_trajectory, y_trajectory, err = run(robot, p)

//                 if err < best_err:
//                     best_err = err
//                     dp[i] *= 1.1
//                 else:
//                     p[i] += dp[i]
//                     dp[i] *= 0.9
//         it += 1
//     return p