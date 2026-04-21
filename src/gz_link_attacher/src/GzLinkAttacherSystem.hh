#ifndef GZ_LINK_ATTACHER_SYSTEM_HH_
#define GZ_LINK_ATTACHER_SYSTEM_HH_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

namespace gz_link_attacher
{

class GzLinkAttacherSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  GzLinkAttacherSystem() = default;
  ~GzLinkAttacherSystem() override = default;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  struct AttachRequest
  {
    std::string model1;
    std::string link1;
    std::string model2;
    std::string link2;
  };

  bool OnAttach(const gz::msgs::StringMsg_V &_req,
                gz::msgs::Boolean &_rep);
  bool OnDetach(const gz::msgs::StringMsg_V &_req,
                gz::msgs::Boolean &_rep);

  static std::string MakeKey(const std::string &m1, const std::string &l1,
                             const std::string &m2, const std::string &l2);

  gz::transport::Node node_;
  gz::sim::Entity worldEntity_{gz::sim::kNullEntity};

  std::mutex pendingMtx_;
  std::vector<AttachRequest> pendingAttach_;
  std::vector<AttachRequest> pendingDetach_;

  /// Active kinematic attachments: every PreUpdate step we teleport the
  /// child model to follow the parent link.
  struct Attachment
  {
    gz::sim::Entity parentLinkEntity;
    gz::sim::Entity childLinkEntity;
    gz::sim::Entity childModelEntity;
    gz::math::Pose3d relPose;  // child link in parent link frame
  };
  std::unordered_map<std::string, Attachment> attachments_;
};

}  // namespace gz_link_attacher

#endif  // GZ_LINK_ATTACHER_SYSTEM_HH_
