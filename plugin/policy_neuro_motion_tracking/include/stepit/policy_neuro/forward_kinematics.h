#ifndef STEPIT_NEURO_POLICY_FORWARD_KINEMATICS_H_
#define STEPIT_NEURO_POLICY_FORWARD_KINEMATICS_H_

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class ForwardKinematics : public Module {
 public:
  ForwardKinematics(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &, FieldMap &context) override;

 private:
  std::string urdf_path_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<std::string> body_names_;
  std::vector<pinocchio::FrameIndex> body_indices_;
  pinocchio::FrameIndex anchor_index_{};
  pinocchio::FrameIndex left_ee_index_{};
  pinocchio::FrameIndex right_ee_index_{};
  std::vector<pinocchio::JointIndex> joint_indices_;
  bool provide_current_ee_pose_{false};

  FieldId anchor_global_pos_id_{};
  FieldId anchor_global_ori_id_{};
  FieldId whole_body_local_pos_id_{};
  FieldId whole_body_local_ori_id_{};
  FieldId whole_body_global_pos_id_{};
  FieldId whole_body_global_ori_id_{};
  FieldId left_current_ee_pose_id_{};
  FieldId right_current_ee_pose_id_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_FORWARD_KINEMATICS_H_
