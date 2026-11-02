#include "wave_math.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace rig_controller
{

Blend quintic_blend(double elapsed, double duration)
{
  const double u = std::clamp(elapsed / duration, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return {
    10.0 * u3 - 15.0 * u4 + 6.0 * u5,
    (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration,
    (60.0 * u - 180.0 * u2 + 120.0 * u3) / (duration * duration)};
}

double solve_dispersion(double omega, double depth, double g)
{
  const double mu0 = (omega * omega * depth) / g;
  // Eckart 1952 approximation
  const double mu_a = mu0 / std::sqrt(std::tanh(mu0));

  // Beji 2013 correction terms
  constexpr double alpha = 1.09;
  constexpr double beta0 = 1.55;
  constexpr double beta1 = 1.30;
  constexpr double beta2 = 0.216;

  const double fc = std::pow(mu0, alpha) *
    std::exp(-(beta0 + beta1 * mu0 + beta2 * mu0 * mu0));
  return mu_a * (1.0 + fc);
}

double compute_stroke(
  double mu, double target_height, const std::string & type, double hinge_fraction)
{
  double transfer_ratio;  // H/S

  if (type == "piston") {
    transfer_ratio = (4.0 * std::sinh(mu) * std::sinh(mu)) /
      (std::sinh(2.0 * mu) + 2.0 * mu);
  } else {  // "flap", hinged hinge_fraction * depth above the floor (Dean & Dalrymple)
    const double hinge_mu = mu * hinge_fraction;
    const double bracket = std::sinh(mu) +
      (std::cosh(hinge_mu) - std::cosh(mu)) / (mu - hinge_mu);
    transfer_ratio = 4.0 * std::sinh(mu) * bracket / (std::sinh(2.0 * mu) + 2.0 * mu);
  }

  return target_height / transfer_ratio;
}

GainLookup interpolate_gain(
  const std::vector<double> & periods, const std::vector<double> & values, double period)
{
  if (period <= periods.front()) {
    return {values.front(), period < periods.front()};
  }
  if (period >= periods.back()) {
    return {values.back(), period > periods.back()};
  }
  // periods[i - 1] <= period < periods[i]
  const auto i = static_cast<std::size_t>(
    std::upper_bound(periods.begin(), periods.end(), period) - periods.begin());
  const double fraction = (period - periods[i - 1]) / (periods[i] - periods[i - 1]);
  return {values[i - 1] + fraction * (values[i] - values[i - 1]), false};
}

}  // namespace rig_controller
