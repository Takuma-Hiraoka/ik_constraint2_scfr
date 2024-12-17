#ifndef IK_CONSTRAINT2_SLIDESCFRCONSTRAINT_H
#define IK_CONSTRAINT2_SLIDESCFRCONSTRAINT_H

#include <ik_constraint2_scfr/ScfrConstraint.h>

namespace ik_constraint2_slide_scfr{
  class SlideScfrConstraint : public ik_constraint2_scfr::ScfrConstraint {
  public:
    // positionConstraintsのA_link座標をもとに制約を作ってScrfConstraintに渡す.
    // 特に解きはじめから一定値以上滑っていたらそのように制約を修正する. positionConstraintを与えなければ普通のScfrConstraintと同じ
    // ScfrConstraint自体がisPosesEqualによってSCFRの再計算判定を行うので, SlideScfrConstraintは一定値を超えたら接触位置と制約を変えれば良い.
    const std::vector<std::shared_ptr<ik_constraint2::PositionConstraint> >& positionConstraints() const { return positionConstraints_;}
    std::vector<std::shared_ptr<ik_constraint2::PositionConstraint> >& positionConstraints() { return positionConstraints_;}
    const cnoid::Vector6& slideThreshold() const { return slideThreshold_;}
    cnoid::Vector6& slideThreshold() { return slideThreshold_;}

    virtual void updateBounds() override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<SlideScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    std::vector<std::shared_ptr<ik_constraint2::PositionConstraint> > positionConstraints_;
    std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint_ = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
    std::vector<cnoid::Isometry3> initialPoses_;
    cnoid::Vector6 slideThreshold_ = (cnoid::Vector6()<<0.05,0.05,0.05,0.05,0.05,0.05).finished(); // この値を超えたら滑り判定. x,y,yawしか使わない.

  };
}

#endif
