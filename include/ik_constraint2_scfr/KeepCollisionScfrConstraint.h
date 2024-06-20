#ifndef IK_CONSTRAINT2_KEEPCOLLISIONSCFRCONSTRAINT_H
#define IK_CONSTRAINT2_KEEPCOLLISIONSCFRCONSTRAINT_H

#include <ik_constraint2_scfr/ScfrConstraint.h>

namespace ik_constraint2_keep_collision_scfr{
  class KeepCollisionScfrConstraint : public ik_constraint2::ANDConstraint {
  public:
    // 以下の2つを行う.KeepCollisionConstraint的にはまだ触れているのにそのconstraintを触れていないとしてSCFRの計算を行うため一つのconstraintで表す
    //  1. 前周期で触れているconstraintのうち、離してもSCFRが解けるものを1つ選んで、それ以外の触れていたconstraintを触れさせる(keepcollisionANDConstraintにいれる)
    //  2. 1で今回触れることにしたconstraintを使ってSCFRを計算し、その中に重心を留める
    // 離してもSCFRが解けるものの選び方に注意.
    const std::shared_ptr<ik_constraint2::ANDConstraint>& keepCollisionANDConstraints() const { return keepCollisionANDConstraints_;}
    std::shared_ptr<ik_constraint2::ANDConstraint>& keepCollisionANDConstraints() { return keepCollisionANDConstraints_;}
    const std::vector<std::shared_ptr<ik_constraint2::KeepCollisionConstraint> >& keepCollisionConstraints() const { return keepCollisionConstraints_;}
    std::vector<std::shared_ptr<ik_constraint2::KeepCollisionConstraint> >& keepCollisionConstraints() { return keepCollisionConstraints_;}
    const std::shared_ptr<ik_constraint2_scfr::ScfrConstraint>& scfrConstraint() const { return scfrConstraint_;}
    std::shared_ptr<ik_constraint2_scfr::ScfrConstraint>& scfrConstraint() { return scfrConstraint_;}
    const scfr_solver::SCFRParam& breakableSCFRParam() const { return breakableSCFRParam_;}
    scfr_solver::SCFRParam& breakableSCFRParam() { return breakableSCFRParam_;}

    virtual void updateBounds() override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<KeepCollisionScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    std::shared_ptr<ik_constraint2::ANDConstraint> keepCollisionANDConstraints_ = std::make_shared<ik_constraint2::ANDConstraint>();
    std::vector<std::shared_ptr<ik_constraint2::KeepCollisionConstraint> > keepCollisionConstraints_;
    std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint_;
    scfr_solver::SCFRParam breakableSCFRParam_;

  };
}

#endif
