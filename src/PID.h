#ifndef PID_H
#define PID_H

#include <random>
#include <iostream>

class PID {
public:
    /**
     * Constructor
     */
    PID();
    
    /**
     * Destructor.
     */
    virtual ~PID();
    
    /**
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd);
    
    /**
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);
    
    /**
     * Calculate the total PID error.
     */
    double TotalError();
    
    /**
     * Checks whether adjustment justified
     */
    void evaluate();
    
    /**
     * computes coefficients adjustments according to total epoch delta error and their partial derivatives.
     * And adjusts accordingly.
     */
    void backProp();
    
    /**
     * Resets epoch error back to zero (for next epoch)
     */
    void resetEpochError();
    
    /**
     * getters for Kp, Ki, Kd.
     */
    double getKp() const;
    double getKi() const;
    double getKd() const;
    
    /**
     * public training parameters
     */
    int counter_;
    bool needsTraining_;
    double currentEpochError_;
    const int epochLength_ = 200;
    
private:
    /**
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;
    
    /** Unlike i_error, this accumulates absolute values of CTE,
     * avoiding positive-negative cancelling each other.
     */
    double i_e_fabs_;
    
    /**
     * Coefficients
     */
    double Kp;
    double Ki;
    double Kd;
    
    /**
     * private training parameters
     */
    double epochCumulativeError_; //Accumulates squares of CTEs to further compute RMSE
    double previousEpochError_;
    const double errorThreshold_ = 5e-3;
    const double learnRate_ = 1e-2;
    
    /**
     * Increments epoch error
     * @param cte: cte of current timestep
     */
    void updateEpochError(double cte);
    
    /**
     * @param Kx: Kp, Ki or Kd
     * @param dx: partial derivative for Kp, Ki or Kd respectively
     * @param dE: total delta error over the whole epoch (prev. epoch - current epoch)
     */
    void adjust(double& Kx, double dx, double dE);
    
    void printMessage(std::string message, double value);
};

#endif /* PID_H */
