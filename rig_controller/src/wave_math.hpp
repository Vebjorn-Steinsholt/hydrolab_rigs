#pragma once

#include <string>
#include <vector>

namespace rig_controller
{

// Peak of ds for the quintic blend: v_max = 1.875 * distance / duration.
constexpr double kQuinticPeakVelocityFactor = 1.875;
// Peak of dds for the quintic blend: a_max = 10 / sqrt(3) * distance / duration^2.
constexpr double kQuinticPeakAccelerationFactor = 5.773502691896258;

struct Blend
{
  double s;   // blend factor, 0 -> 1
  double ds;  // d(s)/dt
  double dds; // d^2(s)/dt^2
};

// Quintic smoothstep; duration must be > 0.
Blend quintic_blend(double elapsed, double duration);

// Beji 2013 dispersion relation; returns the dimensionless wavenumber k*h.
double solve_dispersion(double omega, double depth, double g = 9.81);

// Stroke at the still-water level needed for wave height target_height; type is "piston" or
// "flap". hinge_fraction is a flap's hinge height above the floor divided by the water depth
// (0 = hinged at the floor, must be < 1).
double compute_stroke(
  double mu, double target_height, const std::string & type, double hinge_fraction = 0.0);

struct GainLookup
{
  double gain;
  bool clamped;  // period outside the table; gain is the nearest end value
};

// Transfer gain for period, interpolated linearly between the table points. periods must
// ascend strictly and have the same length as values, with at least two points.
GainLookup interpolate_gain(
  const std::vector<double> & periods, const std::vector<double> & values, double period);

}  // namespace rig_controller
