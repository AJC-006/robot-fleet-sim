// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
// LiIonBattery: series/parallel Li-ion pack with a tabulated open-circuit voltage curve.
//   V_bus = N_s * OCV_cell(SoC) - R_pack * I_total,   dSoC/dt = -I_total / Q
// I_total is the sum of the currents published by the thrusters (<current_topic> entries) plus the
// hotel load
// (<hotel_power_w> / V_bus). The plugin reports over-current (continuous and burst limits) but
// does not clip,
// since the pack's protection circuit behaviour is unpublished.
// Output gz.msgs.Double <voltage_topic>; gz.msgs.BatteryState <state_topic>.
#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/battery_state.pb.h>  // NOLINT(build/include_order)
#include <gz/msgs/double.pb.h>  // NOLINT(build/include_order)
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include "fleet_marine_plugins/t200_core.hh"

namespace fleet_marine
{

class LiIonBattery
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    const std::string mname = model.Name(_ecm);
    auto getd = [&](const char * k, double d) {
        return _sdf->HasElement(k) ? _sdf->Get<double>(k) : d;
      };
    ns_ = getd("cells_series", 4);
    np_ = getd("cells_parallel", 6);
    capacityAh_ = getd("capacity_ah", 18.0);
    rPack_ = getd("pack_resistance_ohm", 0.0107);
    hotelW_ = getd("hotel_power_w", 0.0);
    contA_ = getd("current_continuous_a", 60.0);
    burstA_ = getd("current_burst_a", 132.0);
    burstS_ = getd("burst_duration_s", 10.0);
    soc_ = getd("initial_soc", 1.0);
    // <ocv_table> text: "soc volt soc volt ..." (per cell)
    if (_sdf->HasElement("ocv_table")) {
      std::stringstream ss(_sdf->Get<std::string>("ocv_table"));
      double s, v;
      while (ss >> s >> v) {ocv_.emplace_back(s, v);}
      std::sort(ocv_.begin(), ocv_.end());
    }
    if (ocv_.size() < 2) {
      gzerr << "[LiIonBattery] <ocv_table> needs at least two soc/voltage pairs\n";
      return;
    }
    const std::string prefix = "/model/" + mname + "/battery";
    voltagePub_ = node_.Advertise<gz::msgs::Double>(
      _sdf->HasElement("voltage_topic") ? _sdf->Get<std::string>("voltage_topic") :
      prefix + "/voltage");
    statePub_ = node_.Advertise<gz::msgs::BatteryState>(
      _sdf->HasElement("state_topic") ? _sdf->Get<std::string>("state_topic") : prefix + "/state");
    for (auto e = _sdf->FindElement("current_topic"); e; e = e->GetNextElement("current_topic")) {
      const std::string t = e->Get<std::string>();
      currents_[t] = 0.0;
      node_.Subscribe<gz::msgs::Double>(
        t, [this, t](const gz::msgs::Double & m) {
          std::lock_guard<std::mutex> lk(mtx_);
          currents_[t] = m.data();
        });
    }
    voltage_ = ns_ * Interp1(ocv_, soc_);
    ok_ = true;
    gzmsg << "[LiIonBattery] " << ns_ << "S" << np_ << "P " << capacityAh_ << " Ah, "
          << currents_.size() << " current inputs\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager &) override
  {
    if (!ok_ || _info.paused) {return;}
    const double dt = std::chrono::duration<double>(_info.dt).count();
    double iLoad = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (const auto & kv : currents_) {iLoad += kv.second;}
    }
    const double ocv = ns_ * Interp1(ocv_, soc_);
    // hotel load is constant power: solve V = ocv - R (iLoad + P/V) for V (quadratic, positive
    // root)
    const double a = ocv - rPack_ * iLoad;
    double v = a;
    if (hotelW_ > 0) {
      const double disc = a * a - 4 * rPack_ * hotelW_;
      v = disc > 0 ? 0.5 * (a + std::sqrt(disc)) : 0.5 * a;
    }
    const double iTot = iLoad + (v > 1e-3 ? hotelW_ / v : 0.0);
    voltage_ = v;
    current_ = iTot;
    soc_ = std::max(0.0, soc_ - iTot * dt / (capacityAh_ * 3600.0));
    if (iTot > contA_) {
      overS_ += dt;
      if ((iTot > burstA_ || overS_ > burstS_) && !warned_) {
        gzwarn << "[LiIonBattery] pack current " << iTot << " A exceeds the rated "
               << (iTot > burstA_ ? "burst limit" :
                   "continuous limit for the burst duration") << "\n";
        warned_ = true;
      }
    } else {
      overS_ = 0;
      warned_ = false;
    }
    gz::msgs::Double vm;
    vm.set_data(voltage_);
    voltagePub_.Publish(vm);
    since_ += dt;
    if (since_ >= 0.1) {
      since_ = 0;
      gz::msgs::BatteryState s;
      s.set_voltage(voltage_);
      s.set_current(-current_);       // discharge negative (sensor_msgs/BatteryState convention)
      s.set_charge(soc_ * capacityAh_);
      s.set_capacity(capacityAh_);
      s.set_percentage(soc_);
      s.set_power_supply_status(gz::msgs::BatteryState::DISCHARGING);
      statePub_.Publish(s);
    }
  }

private:
  double ns_{4}, np_{6}, capacityAh_{18}, rPack_{0.0107}, hotelW_{0}, contA_{60}, burstA_{132},
    burstS_{10}, soc_{1}, voltage_{16}, current_{0}, overS_{0}, since_{0};
  bool ok_{false}, warned_{false};
  std::vector<std::pair<double, double>> ocv_;
  std::map<std::string, double> currents_;
  std::mutex mtx_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher voltagePub_, statePub_;
};

}  // namespace fleet_marine

GZ_ADD_PLUGIN(
  fleet_marine::LiIonBattery, gz::sim::System,
  fleet_marine::LiIonBattery::ISystemConfigure,
  fleet_marine::LiIonBattery::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(fleet_marine::LiIonBattery, "fleet_marine::LiIonBattery")
