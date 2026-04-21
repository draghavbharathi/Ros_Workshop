#include "GzLinkAttacherSystem.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

namespace gz_link_attacher
{

// ---------------------------------------------------------------------------
void GzLinkAttacherSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> & /*_sdf*/,
  gz::sim::EntityComponentManager & /*_ecm*/,
  gz::sim::EventManager & /*_eventMgr*/)
{
  this->worldEntity_ = _entity;

  std::function<bool(const gz::msgs::StringMsg_V &,
                      gz::msgs::Boolean &)> attachCb =
    [this](const gz::msgs::StringMsg_V &req,
           gz::msgs::Boolean &rep) -> bool {
      return this->OnAttach(req, rep);
    };
  std::function<bool(const gz::msgs::StringMsg_V &,
                      gz::msgs::Boolean &)> detachCb =
    [this](const gz::msgs::StringMsg_V &req,
           gz::msgs::Boolean &rep) -> bool {
      return this->OnDetach(req, rep);
    };
  node_.Advertise("/link_attacher/attach", attachCb);
  node_.Advertise("/link_attacher/detach", detachCb);

  gzmsg << "[GzLinkAttacher] Plugin loaded. Services:"
        << " /link_attacher/attach, /link_attacher/detach\n";
}

// ---------------------------------------------------------------------------
bool GzLinkAttacherSystem::OnAttach(
  const gz::msgs::StringMsg_V &_req, gz::msgs::Boolean &_rep)
{
  if (_req.data_size() != 4)
  {
    gzerr << "[GzLinkAttacher] attach: need 4 strings "
           << "(model1, link1, model2, link2)\n";
    _rep.set_data(false);
    return true;
  }
  std::lock_guard<std::mutex> lock(pendingMtx_);
  pendingAttach_.push_back(
    {_req.data(0), _req.data(1), _req.data(2), _req.data(3)});
  _rep.set_data(true);
  return true;
}

// ---------------------------------------------------------------------------
bool GzLinkAttacherSystem::OnDetach(
  const gz::msgs::StringMsg_V &_req, gz::msgs::Boolean &_rep)
{
  if (_req.data_size() != 4)
  {
    gzerr << "[GzLinkAttacher] detach: need 4 strings "
           << "(model1, link1, model2, link2)\n";
    _rep.set_data(false);
    return true;
  }
  std::lock_guard<std::mutex> lock(pendingMtx_);
  pendingDetach_.push_back(
    {_req.data(0), _req.data(1), _req.data(2), _req.data(3)});
  _rep.set_data(true);
  return true;
}

// ---------------------------------------------------------------------------
std::string GzLinkAttacherSystem::MakeKey(
  const std::string &m1, const std::string &l1,
  const std::string &m2, const std::string &l2)
{
  return m1 + "::" + l1 + "::" + m2 + "::" + l2;
}

// ---------------------------------------------------------------------------
static gz::sim::Entity FindLinkInModel(
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::Entity _modelEntity,
  const std::string &_linkName)
{
  gz::sim::Entity result{gz::sim::kNullEntity};
  _ecm.Each<gz::sim::components::Link,
             gz::sim::components::Name,
             gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Link *,
        const gz::sim::components::Name *_name,
        const gz::sim::components::ParentEntity *_parent) -> bool
    {
      if (_parent->Data() == _modelEntity && _name->Data() == _linkName)
      {
        result = _entity;
        return false;
      }
      return true;
    });
  return result;
}

static gz::sim::Entity FindModel(
  gz::sim::EntityComponentManager &_ecm,
  const std::string &_modelName)
{
  gz::sim::Entity result{gz::sim::kNullEntity};
  _ecm.Each<gz::sim::components::Model,
             gz::sim::components::Name>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Model *,
        const gz::sim::components::Name *_name) -> bool
    {
      if (_name->Data() == _modelName)
      {
        result = _entity;
        return false;
      }
      return true;
    });
  return result;
}

// ---------------------------------------------------------------------------
void GzLinkAttacherSystem::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // ---- Drain pending requests -------------------------------------------
  std::vector<AttachRequest> toAttach, toDetach;
  {
    std::lock_guard<std::mutex> lock(pendingMtx_);
    std::swap(toAttach, pendingAttach_);
    std::swap(toDetach, pendingDetach_);
  }

  // ---- Process ATTACH requests ------------------------------------------
  for (const auto &req : toAttach)
  {
    const std::string key = MakeKey(req.model1, req.link1, req.model2, req.link2);
    if (attachments_.count(key))
    {
      gzmsg << "[GzLinkAttacher] Already attached: " << key << "\n";
      continue;
    }

    auto model1Entity = FindModel(_ecm, req.model1);
    auto model2Entity = FindModel(_ecm, req.model2);
    if (model1Entity == gz::sim::kNullEntity)
    {
      gzerr << "[GzLinkAttacher] Model not found: " << req.model1 << "\n";
      continue;
    }
    if (model2Entity == gz::sim::kNullEntity)
    {
      gzerr << "[GzLinkAttacher] Model not found: " << req.model2 << "\n";
      continue;
    }

    auto link1Entity = FindLinkInModel(_ecm, model1Entity, req.link1);
    auto link2Entity = FindLinkInModel(_ecm, model2Entity, req.link2);
    if (link1Entity == gz::sim::kNullEntity)
    {
      gzerr << "[GzLinkAttacher] Link not found: "
             << req.model1 << "::" << req.link1 << "\n";
      continue;
    }
    if (link2Entity == gz::sim::kNullEntity)
    {
      gzerr << "[GzLinkAttacher] Link not found: "
             << req.model2 << "::" << req.link2 << "\n";
      continue;
    }

    // Snapshot the relative pose: child link in parent link frame
    auto worldPoseParent = gz::sim::worldPose(link1Entity, _ecm);
    auto worldPoseChild = gz::sim::worldPose(link2Entity, _ecm);
    auto relPose = worldPoseParent.Inverse() * worldPoseChild;

    attachments_[key] = {link1Entity, link2Entity, model2Entity, relPose};
    gzmsg << "[GzLinkAttacher] Attached (kinematic) " << key << "\n";
  }

  // ---- Process DETACH requests ------------------------------------------
  for (const auto &req : toDetach)
  {
    const std::string key = MakeKey(req.model1, req.link1, req.model2, req.link2);
    auto it = attachments_.find(key);
    if (it == attachments_.end())
    {
      gzwarn << "[GzLinkAttacher] No attachment to detach: " << key << "\n";
      continue;
    }
    // Remove velocity command components so physics fully takes over.
    // (Leaving them would continuously reset velocity to zero, making
    // the box fall like a feather.)
    auto childLink = it->second.childLinkEntity;
    _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(childLink);
    _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(childLink);

    attachments_.erase(it);
    gzmsg << "[GzLinkAttacher] Detached " << key << "\n";
  }

  // ---- Kinematic follow: teleport child + zero velocity every step ------
  for (auto &[key, att] : attachments_)
  {
    auto parentWorldPose = gz::sim::worldPose(att.parentLinkEntity, _ecm);
    auto targetChildWorldPose = parentWorldPose * att.relPose;

    // Teleport child model to follow parent link
    auto poseCmd = _ecm.Component<gz::sim::components::WorldPoseCmd>(
      att.childModelEntity);
    if (poseCmd)
      *poseCmd = gz::sim::components::WorldPoseCmd(targetChildWorldPose);
    else
      _ecm.CreateComponent(att.childModelEntity,
        gz::sim::components::WorldPoseCmd(targetChildWorldPose));

    // Zero velocity on the child link every step to prevent gravity drift
    auto lv = _ecm.Component<gz::sim::components::LinearVelocityCmd>(
      att.childLinkEntity);
    if (lv)
      *lv = gz::sim::components::LinearVelocityCmd({0, 0, 0});
    else
      _ecm.CreateComponent(att.childLinkEntity,
        gz::sim::components::LinearVelocityCmd({0, 0, 0}));

    auto av = _ecm.Component<gz::sim::components::AngularVelocityCmd>(
      att.childLinkEntity);
    if (av)
      *av = gz::sim::components::AngularVelocityCmd({0, 0, 0});
    else
      _ecm.CreateComponent(att.childLinkEntity,
        gz::sim::components::AngularVelocityCmd({0, 0, 0}));
  }
}

}  // namespace gz_link_attacher

GZ_ADD_PLUGIN(
  gz_link_attacher::GzLinkAttacherSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gz_link_attacher::GzLinkAttacherSystem,
  "gz_link_attacher::GzLinkAttacherSystem")
