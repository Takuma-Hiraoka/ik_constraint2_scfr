#pragma once

#include <ik_constraint2/CollisionConstraint.h>

namespace ik_constraint2_or_keep_collision{
  class ORKeepCollisionConstraint : public ik_constraint2::CollisionConstraint {
  public:
    const std::vector<std::shared_ptr<ik_constraint2::CollisionConstraint> >& collisionConstraints() const { return collisionConstraints_;}
    std::vector<std::shared_ptr<ik_constraint2::CollisionConstraint> >& collisionConstraints() { return collisionConstraints_;}

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;
    // 制約を満たさなくなるまでの最短距離. 現在満たしていない場合は-distanceと同じ. getEqなどは、エラーの頭打ちを行うが、marginは行わないので、より純粋な距離を表す.
    virtual double margin() const override;    // for debug view
    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;

    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<ORKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    virtual bool computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) override;

    std::vector<std::shared_ptr<ik_constraint2::CollisionConstraint> > collisionConstraints_;
    int activeIdx_ = -1;
  };
}
