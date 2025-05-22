#include <ik_constraint2_scfr/ScfrConstraint.h>
#include <scfr_solver/scfr_solver.h>

namespace ik_constraint2_scfr{
  inline bool isPosesEqual(const std::vector<cnoid::Isometry3> ps1, const std::vector<cnoid::Isometry3> ps2) {
    if (ps1.size() != ps2.size()) return false;
    for (int i=0; i<ps1.size(); i++) {
      if (ps1[i].matrix() != ps2[i].matrix()) return false;
    }
    return true;
  }
  void ScfrConstraint::updateBounds() {
    std::vector<cnoid::Isometry3> poses;
    if (this->links_.size() == 0) poses = this->poses_;
    else {
    if (this->poses_.size() != this->links_.size()) {
      std::cerr << "[ScfrConstraint] error!! size of poses and links mismatch!" << std::endl;
      this->links_ = std::vector<cnoid::LinkPtr>(this->poses_.size(), nullptr);
    }
    for (int i=0; i<this->poses_.size();i++) {
      poses.push_back((this->links_[i]) ? this->links_[i]->T() * this->poses_[i] : this->poses_[i]);
    }

    }

    if (!isPosesEqual(poses, this->prevPoses_)) { // 接触点が変わったら再計算
      this->prevPoses_ = poses;

      Eigen::SparseMatrix<double,Eigen::RowMajor> M(0,2);
      Eigen::VectorXd l;
      Eigen::VectorXd u;
      std::vector<Eigen::Vector2d> vertices;
      scfr_solver::calcSCFR(poses,
                            this->As_,
                            this->bs_,
                            this->Cs_,
                            this->dls_,
                            this->dus_,
                            this->A_robot_->mass(),
                            M,
                            l,
                            u,
                            vertices,
                            this->SCFRParam_
                            );
      Eigen::SparseMatrix<double,Eigen::ColMajor> C(M.rows(),3);
      if (M.rows() > 0) C.leftCols<2>() = M;
      this->C_ = C;
      this->dl_ = l;
      this->du_ = u;

      if (this->debugLevel_>=2) {
        std::cerr << "ScfrConstraint" << std::endl;
        for (int i=0; i<vertices.size(); i++) {
          std::cerr << vertices[i][0] << " " << vertices[i][1] << std::endl;
        }
      }
    }
    COMConstraint::updateBounds();

  }

  bool ScfrConstraint::isSatisfied() const {
    if (this->minIneq_.size() == 0) return false; // COMConstraintのisSatisfiedでは接触がなくなったときにもtrueになってしまう. そもそも接触がなくならないようにすること
    double cost2=0.0;
    for(int i=0;i<this->minIneq_.size();i++){
      if(this->minIneq_[i] > 0.0) cost2 += std::pow(this->minIneq_[i], 2);
    }
    for(int i=0;i<this->maxIneq_.size();i++){
      if(this->maxIneq_[i] < 0.0) cost2 += std::pow(this->maxIneq_[i], 2);
    }
    return cost2 <= std::pow(this->CPrecision_,2);
  }

  std::shared_ptr<ik_constraint2::IKConstraint> ScfrConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ScfrConstraint> ret = std::make_shared<ScfrConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }
  void ScfrConstraint::copy(std::shared_ptr<ScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(modelMap.find(this->A_robot_) != modelMap.end()) ret->A_robot() = modelMap.find(this->A_robot_)->second;
    if(modelMap.find(this->B_robot_) != modelMap.end()) ret->B_robot() = modelMap.find(this->B_robot_)->second;
  }
}

