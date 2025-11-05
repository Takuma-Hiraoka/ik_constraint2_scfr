#include <ik_constraint2_scfr/ORKeepCollisionConstraint.h>
#include <iostream>

namespace ik_constraint2_or_keep_collision{
  bool ORKeepCollisionConstraint::computeDistance(const cnoid::LinkPtr A_link, const cnoid::LinkPtr B_link, double& distance, cnoid::Vector3& direction/*B->A*/, cnoid::Vector3& A_v, cnoid::Vector3& B_v) {
    return true; // updateBounds内で使わない
  }
  void ORKeepCollisionConstraint::updateBounds() {
    this->activeIdx_ = -1;
    double maxMargin = -std::numeric_limits<double>::max();

    for(int i=0;i<this->collisionConstraints_.size();i++) {
      this->collisionConstraints_[i]->updateBounds();
      double mgn = this->collisionConstraints_[i]->margin();
      if(mgn > maxMargin) {
        maxMargin = mgn;
        this->activeIdx_ = i;
      }
    }

    if(this->activeIdx_ == -1) {
      this->eq_.resize(0);
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }else{
      this->eq_ = this->collisionConstraints_[this->activeIdx_]->getEq();
      this->minIneq_ = this->collisionConstraints_[this->activeIdx_]->getMinIneq();
      this->maxIneq_ = this->collisionConstraints_[this->activeIdx_]->getMaxIneq();
      this->currentDirection_ = this->collisionConstraints_[this->activeIdx_]->currentDirection();
      this->A_currentLocalp_ = this->collisionConstraints_[this->activeIdx_]->A_currentLocalp();
      this->B_currentLocalp_ = this->collisionConstraints_[this->activeIdx_]->B_currentLocalp();
    }

    if(this->debugLevel_>=2){
      std::cerr << "ORKeepCollisionConstraint " << this->activeIdx_ <<  std::endl;
      std::cerr << "direction" << std::endl;
      std::cerr << this->currentDirection_.transpose() << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }

  }

  void ORKeepCollisionConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(this->activeIdx_ == -1){
      double dim = 0;
      for(size_t i=0;i<joints.size();i++) dim+=IKConstraint::getJointDOF(joints[i]);
      this->jacobian_.resize(0,dim);
      this->jacobianIneq_.resize(0,dim);
      this->jacobianExt_.resize(0,0);
      this->jacobianIneqExt_.resize(0,0);
    }else{
      this->collisionConstraints_[this->activeIdx_]->updateJacobian(joints);
      this->jacobian_ = this->collisionConstraints_[this->activeIdx_]->getJacobian();
      this->jacobianIneq_ = this->collisionConstraints_[this->activeIdx_]->getJacobianIneq();
      this->jacobianExt_ = this->collisionConstraints_[this->activeIdx_]->getJacobianExt();
      this->jacobianIneqExt_ = this->collisionConstraints_[this->activeIdx_]->getJacobianIneqExt();
    }

    if(this->debugLevel_>=2){
      std::cerr << "ORKeepCollisionConstraint " << this->activeIdx_ << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
      std::cerr << "jacobianExt" << std::endl;
      std::cerr << this->jacobianExt_ << std::endl;
      std::cerr << "jacobianIneq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
      std::cerr << "jacobianIneqExt" << std::endl;
      std::cerr << this->jacobianIneqExt_ << std::endl;
    }
  }

  bool ORKeepCollisionConstraint::isSatisfied () const {
    if(this->collisionConstraints_.size() == 0) return true;
    for(int i=0;i<this->collisionConstraints_.size();i++) {
      if(this->collisionConstraints_[i]->isSatisfied()) return true;
    }
    return false;
  }

  double ORKeepCollisionConstraint::distance () const {
    if(this->activeIdx_ == -1) return 0.0;
    else return this->collisionConstraints_[this->activeIdx_]->distance();
  }

  double ORKeepCollisionConstraint::margin () const {
    if(this->activeIdx_ == -1) return 0.0;
    else return this->collisionConstraints_[this->activeIdx_]->margin();
  }

  std::vector<cnoid::SgNodePtr>& ORKeepCollisionConstraint::getDrawOnObjects(){
    this->drawOnObjects_.clear();
    for(int i=0;i<this->collisionConstraints_.size();i++) {
      std::vector<cnoid::SgNodePtr>& objs = this->collisionConstraints_[i]->getDrawOnObjects();
      std::copy(objs.begin(), objs.end(), std::back_inserter(this->drawOnObjects_));
    }
    return this->drawOnObjects_;
  }

  std::shared_ptr<ik_constraint2::IKConstraint> ORKeepCollisionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ORKeepCollisionConstraint> ret = std::make_shared<ORKeepCollisionConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void ORKeepCollisionConstraint::copy(std::shared_ptr<ORKeepCollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->collisionConstraints().clear();
    for(int i=0;i<this->collisionConstraints_.size();i++) {
      ret->collisionConstraints().push_back(std::static_pointer_cast<ik_constraint2::CollisionConstraint>(this->collisionConstraints_[i]->clone(modelMap)));
    }
  }

}
