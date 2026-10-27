// Copyright 2026 Alan
// SPDX-License-Identifier: Apache-2.0
//
// CoulombFriction: isotropic Coulomb friction (a friction cone) for gz-sim 8 + DART.
//
// Why: DART 6.13 (gz-physics-dartsim) solves friction as a box ("pyramid"): two tangent rows,
// each bounded independently by mu * N (dart/constraint/ContactConstraint.cpp, lo/hi with
// findex). The box is aligned with the first friction direction, which defaults to the world
// axes. A body sliding along a diagonal therefore feels up to sqrt(2) mu N, and the friction
// force is not opposite to the slip velocity (measured: box pushed at 45 deg with 1.5 mu m g
// accelerates at 0.67 m/s^2 instead of 3.92 m/s^2; a foot pushed at 45 deg breaks away at
// 1.14 mu instead of mu). Coulomb's law for an isotropic surface is a cone: friction never
// exceeds mu N, and while sliding it is mu N directly opposite to the slip velocity.
//
// What, per contact point, through gz-sim's contact surface customization event:
// - sliding (|v_t| > slip_threshold, v_t from the link twists at the start of the step): the
//   first friction direction is set along v_t and the second coefficient to zero, so DART
//   applies exactly mu N opposite to the slip;
// - sticking: the first friction direction is set along the incipient slip direction of the
//   contact, so the friction force lies on a box axis and breakaway happens at mu N in any
//   direction. The incipient slip comes from the previous step's contact forces between the
//   same two links (gz-sim ContactSensorData) through a uniform elastic foundation: the
//   tangential load (net force F, moment M about the centroid of the m contact points)
//   displaces the link by a small rigid twist, so contact i at r_i from the centroid moves
//   along F / m + M x r_i / sum |r_j|^2. With several contacts on one rigid link (box corners,
//   feet on one body) this removes the static indeterminacy that otherwise leaves DART's
//   per-contact forces on the world axes. Contacts with no previous sample keep the engine
//   default.
// Surfaces configured anisotropic (mu != mu2) are left unchanged: an anisotropic surface is a
// box by definition (fdir1, mu, mu2).
//
// Load it once per world:
//   <plugin filename="fleet_coulomb_friction" name="fleet_gz_plugins::CoulombFriction">
//     <slip_threshold>1e-4</slip_threshold>        <!-- m/s, optional -->
//     <stick_match_radius>0.02</stick_match_radius>  <!-- m, optional -->
//   </plugin>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/msgs/contacts.pb.h>  // NOLINT(build/include_order): generated .pb.h, not a C header
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/physics/Events.hh>

namespace fleet_gz_plugins
{

using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::EventManager;
using gz::sim::UpdateInfo;

class CoulombFriction
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const Entity & /*_entity*/, const std::shared_ptr<const sdf::Element> & _sdf,
    EntityComponentManager & /*_ecm*/, EventManager & _eventMgr) override
  {
    if (_sdf->HasElement("slip_threshold")) {
      this->slipThreshold = _sdf->Get<double>("slip_threshold");
    }
    if (_sdf->HasElement("stick_match_radius")) {
      this->matchRadius = _sdf->Get<double>("stick_match_radius");
    }
    using P = gz::physics::FeaturePolicy3d;
    using F = gz::physics::SetContactPropertiesCallbackFeature;
    this->connection = _eventMgr.Connect<gz::sim::events::CollectContactSurfaceProperties>(
      [this](const Entity & _c1, const Entity & _c2, const gz::math::Vector3d & _point,
      const std::optional<gz::math::Vector3d> /*_force*/,
      const std::optional<gz::math::Vector3d> _normal,
      const std::optional<double> /*_depth*/, const size_t /*_numContacts*/,
      F::ContactSurfaceParams<P> & _params)
      {
        this->Surface(_c1, _c2, _point, _normal, _params);
      });
    gzmsg << "CoulombFriction: friction cone, slip threshold " << this->slipThreshold
          << " m/s, stick match radius " << this->matchRadius << " m" << std::endl;
  }

  void PreUpdate(const UpdateInfo & /*_info*/, EntityComponentManager & _ecm) override
  {
    // Every collision (now and spawned later) takes part. Physics keeps the set after it
    // consumes the component. Collisions of non-static models also report their contact
    // forces (ContactSensorData), which the stick case needs.
    _ecm.EachNew<gz::sim::components::Collision, gz::sim::components::ParentEntity>(
      [&](const Entity & _e, const gz::sim::components::Collision *,
      const gz::sim::components::ParentEntity * _parent) -> bool
      {
        this->collisionLink[_e] = _parent->Data();
        _ecm.SetComponentData<gz::sim::components::EnableContactSurfaceCustomization>(_e, true);
        const auto model = gz::sim::Link(_parent->Data()).ParentModel(_ecm);
        const auto * isStatic = model ?
        _ecm.Component<gz::sim::components::Static>(model->Entity()) : nullptr;
        if ((isStatic == nullptr || !isStatic->Data()) &&
        !_ecm.Component<gz::sim::components::ContactSensorData>(_e))
        {
          _ecm.CreateComponent(_e, gz::sim::components::ContactSensorData());
        }
        return true;
      });
    _ecm.EachNew<gz::sim::components::Link>(
      [&](const Entity & _e, const gz::sim::components::Link *) -> bool
      {
        gz::sim::Link(_e).EnableVelocityChecks(_ecm, true);
        return true;
      });
    _ecm.EachRemoved<gz::sim::components::Collision>(
      [&](const Entity & _e, const gz::sim::components::Collision *) -> bool
      {
        this->collisionLink.erase(_e);
        return true;
      });
    _ecm.EachRemoved<gz::sim::components::Link>(
      [&](const Entity & _e, const gz::sim::components::Link *) -> bool
      {
        this->twist.erase(_e);
        return true;
      });
    // Link twists at the start of this step (written by Physics after the previous step).
    _ecm.Each<gz::sim::components::Link, gz::sim::components::WorldLinearVelocity,
      gz::sim::components::WorldAngularVelocity>(
      [&](const Entity & _e, const gz::sim::components::Link *,
      const gz::sim::components::WorldLinearVelocity * _v,
      const gz::sim::components::WorldAngularVelocity * _w) -> bool
      {
        Twist & t = this->twist[_e];
        t.origin = gz::sim::worldPose(_e, _ecm).Pos();
        t.v = _v->Data();
        t.w = _w->Data();
        return true;
      });
    // Contact forces of the previous step, grouped per link pair, turned into incipient slip
    // directions (see header).
    std::map<std::pair<Entity, Entity>, bool> seen;
    std::map<std::pair<Entity, Entity>, std::vector<ForceSample>> raw;
    _ecm.Each<gz::sim::components::Collision, gz::sim::components::ContactSensorData>(
      [&](const Entity &, const gz::sim::components::Collision *,
      const gz::sim::components::ContactSensorData * _data) -> bool
      {
        for (const auto & c : _data->Data().contact()) {
          const Entity c1 = c.collision1().id();
          const Entity c2 = c.collision2().id();
          if (!seen.try_emplace(Key(c1, c2), true).second) {
            continue;   // already read from the other collision of the pair
          }
          auto & samples = raw[Key(this->LinkOf(c1), this->LinkOf(c2))];
          const int n = std::min({c.position_size(), c.normal_size(), c.wrench_size()});
          for (int i = 0; i < n; ++i) {
            const auto & p = c.position(i);
            const auto & f = c.wrench(i).body_1_wrench().force();
            const auto & nm = c.normal(i);
            const gz::math::Vector3d force(f.x(), f.y(), f.z());
            const gz::math::Vector3d normal(nm.x(), nm.y(), nm.z());
            // Sign is irrelevant (the box is symmetric) as long as all samples of one link
            // pair use the same convention: orient each force as acting on the lower entity.
            const double sign = (this->LinkOf(c1) <= this->LinkOf(c2)) ? 1.0 : -1.0;
            samples.push_back(
              {gz::math::Vector3d(p.x(), p.y(), p.z()),
                sign * (force - normal * force.Dot(normal))});
          }
        }
        return true;
      });
    this->slipDirs.clear();
    for (auto & [key, samples] : raw) {
      gz::math::Vector3d centroid, net, moment;
      for (const auto & s : samples) {
        centroid += s.point;
        net += s.tangential;
      }
      const double m = static_cast<double>(samples.size());
      centroid /= m;
      double polar = 0.0;
      for (const auto & s : samples) {
        const gz::math::Vector3d r = s.point - centroid;
        moment += r.Cross(s.tangential);
        polar += r.SquaredLength();
      }
      auto & out = this->slipDirs[key];
      for (const auto & s : samples) {
        gz::math::Vector3d d = net / m;
        if (polar > 1e-12) {
          d += moment.Cross(s.point - centroid) / polar;
        }
        out.push_back({s.point, d});
      }
    }
  }

private:
  struct Twist
  {
    gz::math::Vector3d origin;
    gz::math::Vector3d v;
    gz::math::Vector3d w;
  };

  struct ForceSample
  {
    gz::math::Vector3d point;
    gz::math::Vector3d tangential;   // force, or incipient slip direction after grouping
  };

  static std::pair<Entity, Entity> Key(Entity _a, Entity _b)
  {
    return _a < _b ? std::make_pair(_a, _b) : std::make_pair(_b, _a);
  }

  Entity LinkOf(const Entity & _collision) const
  {
    const auto c = this->collisionLink.find(_collision);
    return c == this->collisionLink.end() ? _collision : c->second;
  }

  gz::math::Vector3d PointVelocity(const Entity & _collision, const gz::math::Vector3d & _p) const
  {
    const auto c = this->collisionLink.find(_collision);
    if (c == this->collisionLink.end()) {
      return gz::math::Vector3d::Zero;
    }
    const auto t = this->twist.find(c->second);
    if (t == this->twist.end()) {
      return gz::math::Vector3d::Zero;   // static link: no twist component
    }
    return t->second.v + t->second.w.Cross(_p - t->second.origin);
  }

  const ForceSample * Nearest(
    const Entity & _c1, const Entity & _c2, const gz::math::Vector3d & _p) const
  {
    const auto it = this->slipDirs.find(Key(this->LinkOf(_c1), this->LinkOf(_c2)));
    if (it == this->slipDirs.end()) {
      return nullptr;
    }
    const ForceSample * best = nullptr;
    double bestD = this->matchRadius;
    for (const auto & s : it->second) {
      const double d = s.point.Distance(_p);
      if (d <= bestD) {
        bestD = d;
        best = &s;
      }
    }
    return best;
  }

  template<typename ParamsT>
  void Surface(
    const Entity & _c1, const Entity & _c2, const gz::math::Vector3d & _point,
    const std::optional<gz::math::Vector3d> & _normal, ParamsT & _params)
  {
    if (!_normal || !_params.frictionCoeff) {
      return;
    }
    const double mu = *_params.frictionCoeff;
    const double mu2 = _params.secondaryFrictionCoeff ? *_params.secondaryFrictionCoeff : mu;
    if (std::abs(mu - mu2) > 1e-9 * std::max(1.0, mu)) {
      if (!this->warnedAnisotropic) {
        gzwarn << "CoulombFriction: contact with mu " << mu << " != mu2 " << mu2
               << " left to the engine's box friction (anisotropic surface)" << std::endl;
        this->warnedAnisotropic = true;
      }
      return;
    }
    gz::math::Vector3d n = *_normal;
    if (n.Length() < 1e-12) {
      return;
    }
    n.Normalize();
    const gz::math::Vector3d vrel = this->PointVelocity(_c1, _point) -
      this->PointVelocity(_c2, _point);
    const gz::math::Vector3d vt = vrel - n * vrel.Dot(n);
    const double speed = vt.Length();
    if (speed > this->slipThreshold) {
      // Sliding: one row along the slip carries the full mu N.
      _params.firstFrictionalDirection = gz::math::eigen3::convert(vt / speed);
      _params.secondaryFrictionCoeff = 0.0;
      return;
    }
    // Sticking: put the box axis on the incipient slip direction.
    const ForceSample * s = this->Nearest(_c1, _c2, _point);
    if (s == nullptr) {
      return;
    }
    const gz::math::Vector3d ft = s->tangential - n * s->tangential.Dot(n);
    const double f = ft.Length();
    if (f < std::numeric_limits<double>::epsilon()) {
      return;
    }
    _params.firstFrictionalDirection = gz::math::eigen3::convert(ft / f);
  }

  double slipThreshold{1e-4};
  double matchRadius{0.02};
  bool warnedAnisotropic{false};
  gz::common::ConnectionPtr connection;
  std::unordered_map<Entity, Entity> collisionLink;
  std::unordered_map<Entity, Twist> twist;
  std::map<std::pair<Entity, Entity>, std::vector<ForceSample>> slipDirs;
};

}  // namespace fleet_gz_plugins

GZ_ADD_PLUGIN(
  fleet_gz_plugins::CoulombFriction, gz::sim::System,
  fleet_gz_plugins::CoulombFriction::ISystemConfigure,
  fleet_gz_plugins::CoulombFriction::ISystemPreUpdate)
