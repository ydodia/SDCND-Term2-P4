#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double p, double i, double d)
{
  Kp = p;
  Ki = i;
  Kd = d;
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::Init(const std::vector<double>& p)
{
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  p_error = 0;
  i_error = 0;
  d_error = 0;
}


void PID::UpdateError(double cte)
{
  p_error = cte;
  summands[idx] = cte;
  idx = (idx+1) % summands.size();
  i_error = 0.0;
  for(const auto& x : summands)
    i_error += x;
  d_error = cte - prev_cte;
  prev_cte = cte;
}

double PID::TotalError()
{
  double sum_error = -Kp*p_error - Ki*i_error - Kd*d_error;
  if(sum_error < -1.0)
    sum_error = -1;
  if(sum_error > 1.0)
    sum_error = 1.0;
  return sum_error;
}


Twiddle::Twiddle(std::string name): name(name) {}

Twiddle::~Twiddle() {}

void Twiddle::update(std::vector<double>& p, std::vector<double>& dp, double err)
{
  if(increment)
  {
    p[idx] += dp[idx];
    increment = false;
    return;
  }
  else
  {
    if(err < best_err)
    {
      best_err = err;
      dp[idx] *= 1.1;
      idx = (idx+1) % dp.size();
      increment = true;
    }
    else if(!second)
    {
      p[idx] -= 2 * dp[idx];
      second = true;
      return;
    }
    else
    {
      p[idx] += dp[idx];
      dp[idx] *= 0.9;
      idx = (idx+1) % dp.size();
      second = false;
      increment = true;
    }
  }
}

void Twiddle::printState(const std::vector<double>& p, const std::vector<double>& dp)
{
  std::cerr << "\n --= " << name << " iteration " << ++iteration << " =--\n";
  std::cerr << "p = " << "[";
  printf("P: %1.5f, I: %1.2e, D: %1.5f] \n", p[0], p[1], p[2]);
  std::cerr << "dp = " << "[";
  printf("%1.5f, %1.2e, %1.5f] \n", dp[0], dp[1], dp[2]);
  double sum_dp = dp[0]+dp[1]+dp[2];
  if(sum_dp < 0.001)
    printf("sum(dp): %1.2e\n", sum_dp);
  else
    printf("sum(dp): %1.4f\n", sum_dp);
}

int Twiddle::getIterations() { return iteration; }
void Twiddle::resetIterations()
{
  iteration = 0;
  best_err = std::numeric_limits<double>::max();
}

