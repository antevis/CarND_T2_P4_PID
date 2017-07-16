#include "PID.h"



//using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    this->p_error = 0;
    this->i_error = 0;
    this->d_error = 0;
    
    counter_ = 0;
    
    // Training and evaluation variables
    epochCumulativeError_ = 0;
    previousEpochError_ = 0;
    currentEpochError_ = 0;
    needsTraining_ = true;
    
    i_e_fabs_ = 0;
}

void PID::updateEpochError(double cte) {
    i_e_fabs_ += fabs(cte);
    epochCumulativeError_ += pow(cte, 2);
}

void PID::resetEpochError() {
    i_e_fabs_ = 0;
    epochCumulativeError_ = 0;
}

void PID::evaluate() {
    // Permanently stop training once criteria has been met.
    if (needsTraining_) {
        currentEpochError_ = sqrt(epochCumulativeError_ / epochLength_);
        needsTraining_ = currentEpochError_ > errorThreshold_;
    }
}

void PID::printMessage(std::string message, double value) {
    std::cout << message << value << std::endl;
}

/**
 * @param Kx: Kp, Ki or Kd
 * @param dx: partial derivative for Kp, Ki or Kd respectively
 * @param dE: total delta error over the whole epoch (previous epoch - current epoch)
 */
void PID::adjust(double &Kx, double dx, double dE) {
    double partialDKx = Kx * dx * dE * learnRate_;
    Kx -= partialDKx;
}

double PID::getKp() const { return Kp; }
double PID::getKi() const { return Ki; }
double PID::getKd() const { return Kd; }


/** Since I tune Kp, Ki and Kd parameters, partial derivatives of f(Kp, Ki, Kd) = -Kp * p_error - Ki * i_error - Kd * d_error
 * WITH RESPECT TO Kp, Ki, Kd are -p_error, -i_error and -d_error respectively.
 *
 * There's a bit cheating with i_error here. Since I evaluate cumulative error through some number of iterations (say, 200),
 * and each iteration's CTE might be both positive or negative, their raw summation (which is i_error) doesn't repersent the
 * magnitude of that paramteter with positives and negatives cancelling each other out. To address the issue, I've introduced
 * new variable i_e_fabs_, that accumulates absolute values of CTE throughout the epoch, and use it as a measure of partial
 * derivative for Ki.
 */
void PID::backProp() {
    double deltaError = previousEpochError_ - currentEpochError_;
    previousEpochError_ = currentEpochError_;
    
    adjust(Kp, -p_error, deltaError);
    adjust(Ki, -i_e_fabs_, deltaError);
    adjust(Kd, -d_error, deltaError);
}

void PID::UpdateError(double cte) {
    
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    
    updateEpochError(cte);
}

double PID::TotalError() {
    return -Kp * p_error - Ki * i_error - Kd * d_error;
}

