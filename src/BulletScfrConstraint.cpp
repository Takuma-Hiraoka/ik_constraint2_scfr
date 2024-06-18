#include <ik_constraint2_scfr/BulletScfrConstraint.h>

namespace ik_constraint2_bullet_scfr{
  void BulletScfrConstraint::updateBounds() {
    this->poses_.clear();
    this->As_.clear();
    this->bs_.clear();
    this->Cs_.clear();
    this->dls_.clear();
    this->dus_.clear();
    for (int i=0; i<this->keepCollisionConstraints_.size(); i++) {
      if (this->keepCollisionConstraints_[i]->isSatisfied()) {
        cnoid::Isometry3 pose = cnoid::Isometry3::Identity();
        pose.translation() = keepCollisionConstraints_[i]->currentp();
        this->poses_.push_back(pose);
        this->As_.emplace_back(0,6);
        this->bs_.emplace_back(0);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
        C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
        C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
        C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        this->Cs_.push_back(C);
        cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        this->dls_.push_back(dl);
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 2000.0;
        this->dus_.push_back(du);
      }
    }
    ScfrConstraint::updateBounds();
  }
  std::shared_ptr<ik_constraint2::IKConstraint> BulletScfrConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<BulletScfrConstraint> ret = std::make_shared<BulletScfrConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void BulletScfrConstraint::copy(std::shared_ptr<BulletScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->keepCollisionConstraints().clear();
    for (int i=0; i<this->keepCollisionConstraints_.size(); i++) {
      ret->keepCollisionConstraints().push_back(std::static_pointer_cast<ik_constraint2_bullet::BulletKeepCollisionConstraint>(this->keepCollisionConstraints_[i]->clone(modelMap)));
    }
  }
}
