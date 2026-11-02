#include "wave_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "wave_math.hpp"

namespace rig_controller
{

namespace
{

// Linear interpolation of positions; returns true once elapsed reaches duration.
// positions must not be empty.
bool sample_pregenerated(
  const std::vector<double> & positions, double sample_interval, double elapsed,
  double duration, double & position, double & velocity)
{
  if (elapsed >= duration) {
    position = positions.back();
    velocity = 0.0;
    return true;
  }

  const double sample = elapsed / sample_interval;
  // Rounding can put sample at n - 1 just below duration; keep index + 1 in range.
  const auto index = std::min(static_cast<std::size_t>(sample), positions.size() - 2);
  const double fraction = sample - static_cast<double>(index);
  const double delta = positions[index + 1] - positions[index];
  position = positions[index] + fraction * delta;
  velocity = delta / sample_interval;
  return false;
}

}  // namespace

void WaveTrajectory::configure(
  const WavemakerGeometry & geometry, double wave_height, double period,
  double start_position)
{
  omega_ = 2.0 * M_PI / period;
  if (geometry.type == "oscillator") {
    // H is the stroke itself: no dispersion relation, transfer gain or flap scaling.
    amplitude_ = wave_height / 2.0;
  } else {
    const double mu = solve_dispersion(omega_, geometry.water_depth);
    const bool flap = geometry.type == "flap";
    const double hinge = flap ? geometry.hinge_height : 0.0;
    const double stroke = compute_stroke(
      mu, wave_height, geometry.type, hinge / geometry.water_depth) / geometry.transfer_gain;
    amplitude_ = stroke / 2.0;
    if (flap) {
      // The stroke is at the still-water level; scale it up to the attachment point.
      amplitude_ *= (geometry.flap_attachment_height - hinge) / (geometry.water_depth - hinge);
    }
  }
  upright_position_ = geometry.upright_position;
  upright_is_minimum_ = geometry.upright_is_minimum;
  start_position_ = start_position;
  startup_duration_ = period;
}

double WaveTrajectory::required_minimum() const
{
  return upright_is_minimum_ ? upright_position_ : upright_position_ - amplitude_;
}

double WaveTrajectory::required_maximum() const
{
  return upright_position_ + amplitude_ * (upright_is_minimum_ ? 2.0 : 1.0);
}

double WaveTrajectory::peak_velocity() const
{
  // After the blend the speed peaks at amplitude * omega. In the blend, the blend's and the
  // wave's speeds peak at different times, so adding their peaks overestimates by up to a
  // third; sample the blend instead. Between samples the speed changes by at most a
  // relative ~(2 pi / kBlendSamples)^2 / 2 near a peak, which kSamplingMargin covers.
  constexpr int kBlendSamples = 10000;
  constexpr double kSamplingMargin = 1e-4;
  double peak = amplitude_ * omega_;
  for (int i = 0; i <= kBlendSamples; ++i) {
    double position = 0.0;
    double velocity = 0.0;
    sample(startup_duration_ * i / kBlendSamples, position, velocity);
    peak = std::max(peak, std::abs(velocity));
  }
  return peak * (1.0 + kSamplingMargin);
}

void WaveTrajectory::wave(double t, double & x, double & v, double & a) const
{
  const double phase = omega_ * t;
  if (upright_is_minimum_) {
    x = upright_position_ + amplitude_ * (1.0 - std::cos(phase));
    v = amplitude_ * omega_ * std::sin(phase);
    a = amplitude_ * omega_ * omega_ * std::cos(phase);
  } else {
    x = upright_position_ + amplitude_ * std::sin(phase);
    v = amplitude_ * omega_ * std::cos(phase);
    a = -amplitude_ * omega_ * omega_ * std::sin(phase);
  }
}
double WaveTrajectory::peak_acceleration() const
{
  // After the blend the acceleration peaks at amplitude * omega^2. In the blend the product rule
  // gives a = dds * (w - x0) + 2 * ds * w' + s * w'', which can exceed that; sample it like
  // peak_velocity() does.
  constexpr int kBlendSamples = 10000;
  constexpr double kSamplingMargin = 1e-4;
  double peak = amplitude_ * omega_ * omega_;
  for (int i = 0; i <= kBlendSamples; ++i) {
    const double t = startup_duration_ * i / kBlendSamples;
    double x, v, a;
    wave(upright_is_minimum_ ? t : t - startup_duration_, x, v, a);   // same phase as sample()
    const Blend b = quintic_blend(t, startup_duration_);
    peak = std::max(peak, std::abs(b.dds * (x - start_position_) + 2.0 * b.ds * v + b.s * a));
  }
  return peak * (1.0 + kSamplingMargin);
}

void WaveTrajectory::sample(double elapsed, double & position, double & velocity) const
{
  // Phase is continuous across the blend only because startup_duration_ equals the period.
  const double wave_time = (upright_is_minimum_ && elapsed < startup_duration_) ?
    elapsed : elapsed - startup_duration_;

  double wave_position;
  double wave_velocity;
  double wave_acceleration;

  wave(wave_time, wave_position, wave_velocity, wave_acceleration);

  if (elapsed < startup_duration_) {
    const Blend b = quintic_blend(elapsed, startup_duration_);
    position = start_position_ + b.s * (wave_position - start_position_);
    velocity = b.ds * (wave_position - start_position_) + b.s * wave_velocity;
  } else {
    position = wave_position;
    velocity = wave_velocity;
  }
}

void PregeneratedWaveTrajectory::configure(
  std::vector<double> positions, double sample_interval, double start_position,
  double blend_duration)
{
  positions_ = std::move(positions);
  sample_interval_ = sample_interval;
  start_position_ = start_position;
  blend_duration_ = blend_duration;
}

double PregeneratedWaveTrajectory::duration() const
{
  return blend_duration_ + sample_interval_ * static_cast<double>(positions_.size() - 1);
}

double PregeneratedWaveTrajectory::peak_velocity() const
{
  double peak = 0.0;
  if (blend_duration_ > 0.0) {
    peak = kQuinticPeakVelocityFactor * std::abs(positions_.front() - start_position_) /
      blend_duration_;
  }
  for (std::size_t i = 1; i < positions_.size(); ++i) {
    peak = std::max(peak, std::abs(positions_[i] - positions_[i - 1]) / sample_interval_);
  }
  return peak;
}
double PregeneratedWaveTrajectory::peak_acceleration() const
{
  double peak = 0.0;
  if (blend_duration_ > 0.0) {
    peak = kQuinticPeakAccelerationFactor * std::abs(positions_.front() - start_position_) /
      (blend_duration_ * blend_duration_);
  }
  // Second differences need a sample on each side; i + 1 < size also holds for 0 or 1 samples.
  const double dt2 = sample_interval_ * sample_interval_;
  for (std::size_t i = 1; i + 1 < positions_.size(); ++i) {
    peak = std::max(peak,
        std::abs(positions_[i + 1] - 2.0 * positions_[i] + positions_[i - 1]) / dt2);
  }
  return peak;
}

bool PregeneratedWaveTrajectory::sample(double elapsed, double & position, double & velocity) const
{
  if (elapsed < blend_duration_) {
    const Blend b = quintic_blend(elapsed, blend_duration_);
    const double delta = positions_.front() - start_position_;
    position = start_position_ + b.s * delta;
    velocity = b.ds * delta;
    return false;
  }
  return sample_pregenerated(
    positions_, sample_interval_, elapsed - blend_duration_, duration() - blend_duration_,
    position, velocity);
}

}  // namespace rig_controller
