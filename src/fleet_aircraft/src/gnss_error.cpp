// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// GnssError: turns the error-free gz NavSat sensor of a model into a real receiver's output.
// The model carries a navsat sensor named <input_sensor> (ground truth, no noise) and this plugin
// republishes it on the topic of <output_sensor>, which is where PX4's gz bridge listens, adding
//   * horizontal and vertical position error (Gauss-Markov, <horizontal_stddev_m>,
//     <vertical_stddev_m>, <position_correlation_time_s>),
//   * velocity error (white, <velocity_stddev_mps>).
// All elements are optional; the defaults give an ideal receiver. <seed> makes a run repeatable.
#include <gz/msgs/navsat.pb.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "fleet_aircraft/gnss_core.hh"

namespace fleet_aircraft
{

class GnssError : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    gz::sim::World world(gz::sim::worldEntity(_ecm));
    auto getd = [&](const char * k, double d) {
        return _sdf->HasElement(k) ? _sdf->Get<double>(k) : d;
      };
    auto gets = [&](const char * k, const std::string & d) {
        return _sdf->HasElement(k) ? _sdf->Get<std::string>(k) : d;
      };
    const double sh = getd("horizontal_stddev_m", 0.0);
    const double sv = getd("vertical_stddev_m", 0.0);
    const double tau = getd("position_correlation_time_s", 300.0);
    velStd_ = getd("velocity_stddev_mps", 0.0);
    const unsigned seed = static_cast<unsigned>(getd("seed", 12345.0));
    north_ = std::make_unique<GaussMarkov>(sh, tau, seed);
    east_ = std::make_unique<GaussMarkov>(sh, tau, seed + 1);
    up_ = std::make_unique<GaussMarkov>(sv, tau, seed + 2);
    vrng_.seed(seed + 3);
    north_->Reset();
    east_->Reset();
    up_->Reset();

    const std::string prefix = "/world/" + world.Name(_ecm).value_or("default") + "/model/" +
      model.Name(_ecm) + "/link/" + gets("link", "base_link") + "/sensor/";
    const std::string suffix = "/navsat";
    const std::string in = prefix + gets("input_sensor", "navsat_truth") + suffix;
    const std::string out = prefix + gets("output_sensor", "navsat_sensor") + suffix;
    pub_ = node_.Advertise<gz::msgs::NavSat>(out);
    if (!node_.Subscribe(in, &GnssError::OnNavSat, this)) {
      gzerr << "GnssError: cannot subscribe to " << in << std::endl;
    }
    gzmsg << "GnssError: " << in << " -> " << out << " (sigma_h " << sh << " m, sigma_v " << sv
          << " m, tau " << tau << " s, sigma_vel " << velStd_ << " m/s)" << std::endl;
  }

private:
  void OnNavSat(const gz::msgs::NavSat & _in)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const double t = _in.header().stamp().sec() + 1e-9 * _in.header().stamp().nsec();
    const double dt = last_ >= 0 ? t - last_ : 0.0;
    last_ = t;
    gz::msgs::NavSat m = _in;
    const double en = north_->Step(dt), ee = east_->Step(dt), eu = up_->Step(dt);
    double mn, me;
    MetresPerRadian(_in.latitude_deg() * M_PI / 180.0, mn, me);
    m.set_latitude_deg(_in.latitude_deg() + en / mn * 180.0 / M_PI);
    m.set_longitude_deg(_in.longitude_deg() + ee / me * 180.0 / M_PI);
    m.set_altitude(_in.altitude() + eu);
    if (velStd_ > 0) {
      std::normal_distribution<double> n(0.0, velStd_);
      m.set_velocity_east(_in.velocity_east() + n(vrng_));
      m.set_velocity_north(_in.velocity_north() + n(vrng_));
      m.set_velocity_up(_in.velocity_up() + n(vrng_));
    }
    pub_.Publish(m);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
  std::unique_ptr<GaussMarkov> north_, east_, up_;
  std::mt19937 vrng_;
  double velStd_{0}, last_{-1};
  std::mutex mtx_;
};

}  // namespace fleet_aircraft

GZ_ADD_PLUGIN(fleet_aircraft::GnssError, gz::sim::System,
  fleet_aircraft::GnssError::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(fleet_aircraft::GnssError, "fleet_aircraft::GnssError")
