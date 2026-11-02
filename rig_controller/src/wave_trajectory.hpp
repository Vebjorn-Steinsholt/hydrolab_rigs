#pragma once

#include <string>
#include <vector>

namespace rig_controller
{

struct WavemakerGeometry
{
  std::string type;  // "piston", "flap" or "oscillator"
  double water_depth{0.0};
  double flap_attachment_height{0.0};  // above the tank floor
  double upright_position{0.0};
  bool upright_is_minimum{false};
  // Measured / theoretical wave height; the stroke is divided by it. 1.0 = linear theory.
  double transfer_gain{1.0};
  double hinge_height{0.0};  // flap only: hinge above the tank floor, < water_depth
};

// Regular wave with a quintic blend from the start position over one period.
class WaveTrajectory
{
public:
  // wave_height is the target wave height H (crest to trough).
  void configure(
    const WavemakerGeometry & geometry, double wave_height, double period,
    double start_position);

  double required_minimum() const;
  double required_maximum() const;
  // Largest |velocity| over the whole trajectory, start blend included, with a 0.01 % margin
  // so it never falls below a sample.
  double peak_velocity() const;
  // Largest |acceleration| over the whole trajectory, start blend included, with the same
  // 0.01 % margin. The blend can accelerate harder than the wave itself.
  double peak_acceleration() const;

  void sample(double elapsed, double & position, double & velocity) const;

private:
  // The wave without the blend at time t: position, velocity, acceleration.
  void wave(double t, double & x, double & v, double & a) const;

  double upright_position_{0.0};
  bool upright_is_minimum_{false};
  double amplitude_{0.0};
  double omega_{0.0};
  double start_position_{0.0};
  double startup_duration_{0.0};
};

// Sampled positions, played after a quintic blend from the start position to positions[0].
class PregeneratedWaveTrajectory
{
public:
  // positions must not be empty. blend_duration 0 plays the samples immediately.
  void configure(
    std::vector<double> positions, double sample_interval, double start_position,
    double blend_duration);

  // Blend plus samples.
  double duration() const;

  // Largest |velocity| of the blend and of the samples.
  double peak_velocity() const;
  // Largest |acceleration| of the blend and of the samples (their second differences). Noisy
  // samples give high values; play_dat.py is meant to smooth them before sending.
  double peak_acceleration() const;

  // Returns true once elapsed reaches duration().
  bool sample(double elapsed, double & position, double & velocity) const;

private:
  std::vector<double> positions_;
  double sample_interval_{0.0};
  double start_position_{0.0};
  double blend_duration_{0.0};
};

}  // namespace rig_controller
