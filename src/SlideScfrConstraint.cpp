#include <ik_constraint2_scfr/SlideScfrConstraint.h>

namespace ik_constraint2_slide_scfr{
  void SlideScfrConstraint::updateBounds() {
    if (this->initialPoses_.size()==0) {
      for(int i=0;i<this->positionConstraints_.size(); i++) {
        cnoid::Isometry3 initialPose = cnoid::Isometry3::Identity();
        initialPose.translation() = this->positionConstraints_[i]->eval_localR().transpose() * (this->positionConstraints_[i]->A_link()->p() + this->positionConstraints_[i]->A_link()->R() * this->positionConstraints_[i]->A_localpos().translation());
        initialPose.linear() = this->positionConstraints_[i]->eval_localR().transpose() * (this->positionConstraints_[i]->A_link()->R() * this->positionConstraints_[i]->A_localpos().linear());
        this->initialPoses_.push_back(initialPose);
      }
    }

    bool changed = false;
    std::vector<cnoid::Isometry3> poses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
    std::vector<cnoid::VectorX> bs;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorX> dls;
    std::vector<cnoid::VectorX> dus;
    for (int i=0;i<this->positionConstraints_.size();i++) {
      cnoid::Isometry3 currentPose;
      currentPose.translation() = this->positionConstraints_[i]->A_link()->p() + this->positionConstraints_[i]->A_link()->R() * this->positionConstraints_[i]->A_localpos().translation();
      currentPose.linear() = this->positionConstraints_[i]->A_link()->R() * this->positionConstraints_[i]->A_localpos().linear();
      poses.push_back(currentPose);

      cnoid::Vector3 transDiff = this->positionConstraints_[i]->eval_localR().transpose() * currentPose.translation() - initialPoses_[i].translation();
      cnoid::Matrix3 rotDiff = initialPoses_[i].linear().transpose()  * this->positionConstraints_[i]->eval_localR().transpose() * currentPose.linear();
      if (((std::abs(transDiff[0]) >= this->slideThreshold_[0]) || (std::abs(transDiff[1]) >= this->slideThreshold_[1])) &&
          (cnoid::rpyFromRot(rotDiff)[2] >= this->slideThreshold_[5])) { // 並進回転滑り
        changed = true;
        Eigen::SparseMatrix<double,Eigen::RowMajor> A(3,6);
        A.insert(0,0) = transDiff[0] > 0 ? -1.0 : 1.0; A.insert(0,2) = 0.2;
        A.insert(1,1) = transDiff[1] > 0 ? -1.0 : 1.0; A.insert(1,2) = 0.2;
        A.insert(2,5) = cnoid::rpyFromRot(rotDiff)[2] > 0 ? -1.0 : 1.0; A.insert(2,2) = 0.005;
        As.push_back(A);
        cnoid::VectorX b = Eigen::VectorXd::Zero(3);
        bs.push_back(b);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(5,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,2) = 0.05; C.insert(1,3) = 1.0;
        C.insert(2,2) = 0.05; C.insert(2,3) = -1.0;
        C.insert(3,2) = 0.05; C.insert(3,4) = 1.0;
        C.insert(4,2) = 0.05; C.insert(4,4) = -1.0;
        Cs.push_back(C);
        cnoid::VectorX dl = Eigen::VectorXd::Zero(5);
        dls.push_back(dl);
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(5);
        du[0] = 20000.0;
        dus.push_back(du);
      } else if ((std::abs(transDiff[0]) < this->slideThreshold_[0]) &&
          (std::abs(transDiff[1]) < this->slideThreshold_[1]) &&
          (cnoid::rpyFromRot(rotDiff)[2] >= this->slideThreshold_[5])) { // 回転滑り
        changed = true;
        Eigen::SparseMatrix<double,Eigen::RowMajor> A(1,6);
        A.insert(0,5) = cnoid::rpyFromRot(rotDiff)[2] > 0 ? -1.0 : 1.0; A.insert(0,2) = 0.005;
        As.push_back(A);
        cnoid::VectorX b = Eigen::VectorXd::Zero(1);
        bs.push_back(b);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(9,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,2) = 0.2; C.insert(1,0) = 1.0;
        C.insert(2,2) = 0.2; C.insert(2,0) = -1.0;
        C.insert(3,2) = 0.2; C.insert(3,1) = 1.0;
        C.insert(4,2) = 0.2; C.insert(4,1) = -1.0;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        Cs.push_back(C);
        cnoid::VectorX dl = Eigen::VectorXd::Zero(9);
        dls.push_back(dl);
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(9);
        du[0] = 20000.0;
        dus.push_back(du);
      } else { // 不動
        As.emplace_back(0,6);
        bs.emplace_back(0);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,2) = 0.2; C.insert(1,0) = 1.0;
        C.insert(2,2) = 0.2; C.insert(2,0) = -1.0;
        C.insert(3,2) = 0.2; C.insert(3,1) = 1.0;
        C.insert(4,2) = 0.2; C.insert(4,1) = -1.0;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.05; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        Cs.push_back(C);
        cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        dls.push_back(dl);
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 20000.0;
        dus.push_back(du);
      }
    }
    if (changed) {
      this->poses_ = poses;
      this->As_ = As;
      this->bs_ = bs;
      this->Cs_ = Cs;
      this->dls_ = dls;
      this->dus_ = dus;
    }
    ScfrConstraint::updateBounds();
  }

  std::shared_ptr<ik_constraint2::IKConstraint> SlideScfrConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<SlideScfrConstraint> ret = std::make_shared<SlideScfrConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void SlideScfrConstraint::copy(std::shared_ptr<SlideScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    ret->positionConstraints().clear();
    for (int i=0; i<this->positionConstraints_.size(); i++) {
      ret->positionConstraints().push_back(std::static_pointer_cast<ik_constraint2::PositionConstraint>(this->positionConstraints_[i]->clone(modelMap)));
    }
  }

}
