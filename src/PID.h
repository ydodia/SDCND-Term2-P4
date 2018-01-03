#ifndef PID_H
#define PID_H

#include <vector>
#include <iostream>
#include <limits>

class PID
{
  double prev_cte = 0.0;
  std::vector<double> summands = std::vector<double>(75, 0.0);
  int idx = 0;
public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

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
  void Init(double p, double i, double d);
  void Init(const std::vector<double>& p);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();
};


class Twiddle
{
  double best_err = std::numeric_limits<double>::max();
  int iteration = 0;
  std::string name;
  int idx = 0;
  bool increment = true, second = false;

public:
  Twiddle(std::string name = "Steering");
  ~Twiddle();

  void update(std::vector<double>& p, std::vector<double>& dp, double err);
  void printState(const std::vector<double>& p, const std::vector<double>& dp);
  int getIterations();
  void resetIterations();
};

#endif /* PID_H */
