#ifndef IK_CONSTRAINT2_KEEPCOLLISIONSCFRCONSTRAINT_H
#define IK_CONSTRAINT2_KEEPCOLLISIONSCFRCONSTRAINT_H

#include <ik_constraint2_scfr/ScfrConstraint.h>

namespace ik_constraint2_keep_collision_scfr{
  class KeepCollisionScfrConstraint : public ik_constraint2_scfr::ScfrConstraint {
  public:
    // keepCollisionConstraintsで実際にcollisionしているものを使って計算されたSCFRの中にCOMを留める
    // keepCollisionConstraintsより後にupdateBoundsする必要がある
    const std::vector<std::shared_ptr<ik_constraint2::KeepCollisionConstraint> >& keepCollisionConstraints() const { return keepCollisionConstraints_;}
    std::vector<std::shared_ptr<ik_constraint2::KeepCollisionConstraint> >& keepCollisionConstraints() { return keepCollisionConstraints_;}

    virtual void updateBounds() override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<KeepCollisionScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    std::vector<std::shared_ptr<ik_constraint2::KeepCollisionConstraint> > keepCollisionConstraints_;

  };
}

#endif
