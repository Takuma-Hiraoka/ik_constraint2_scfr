#include <ik_constraint2_scfr/KeepCollisionScfrConstraint.h>
#include <cnoid/TimeMeasure>
namespace ik_constraint2_keep_collision_scfr{
  void KeepCollisionScfrConstraint::updateBounds() {

    if (this->children_.size() == 0) {
      this->children_.clear();
      this->children_.push_back(this->keepCollisionANDConstraints_);
      this->children_.push_back(this->scfrConstraint_); // この順番でpush_backすることでSCFR計算時すでに干渉計算が終わっているようにする
    }

    int breakableIdx = -1; // contactIdxsでのidx
    std::vector<int> contactIdxs;
    std::vector<cnoid::Isometry3> poses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
    std::vector<cnoid::VectorX> bs;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorX> dls;
    std::vector<cnoid::VectorX> dus;

    // 全てのkeepCollisionConstraintに関してboundsを計算して、接触しているもののうちSCFRがbreakしてもSCFRが存在し、かつその中で最もmarginが小さいもののIdを見つける.
    {
      cnoid::TimeMeasure timer;
      timer.begin();
      std::vector<double> mgns;
      for (int i=0; i<this->keepCollisionConstraints_.size(); i++) {
        this->keepCollisionConstraints_[i]->updateBounds();
        if (this->keepCollisionConstraints_[i]->isSatisfied()) {
          contactIdxs.push_back(i);
          mgns.push_back(this->keepCollisionConstraints_[i]->margin());
          cnoid::Isometry3 pose = cnoid::Isometry3::Identity();
          pose.translation() = this->keepCollisionConstraints_[i]->currentp();
          poses.push_back(pose);
          As.emplace_back(0,6);
          bs.emplace_back(0);
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
          Cs.push_back(C);
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          dls.push_back(dl);
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 2000.0;
          dus.push_back(du);
        }
      }

      double minMargin = std::numeric_limits<double>::max();
      if (contactIdxs.size() > 2) { // 少なくとも2点で触れるようにする
        for (int i=0; i<contactIdxs.size(); i++) {
          std::vector<cnoid::Isometry3> poses_;
          std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As_;
          std::vector<cnoid::VectorX> bs_;
          std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs_;
          std::vector<cnoid::VectorX> dls_;
          std::vector<cnoid::VectorX> dus_;
          for (int j=0; j<contactIdxs.size(); j++) { // この接触以外でSCFRが計算できるかどうか
            if(i==j) continue;
            poses_.push_back(poses[j]);
            As_.emplace_back(0,6);
            bs_.emplace_back(0);
            Cs_.push_back(Cs[j]);
            dls_.push_back(dls[j]);
            dus_.push_back(dus[j]);
          }
          Eigen::SparseMatrix<double,Eigen::RowMajor> M(0,2);
          Eigen::VectorXd l;
          Eigen::VectorXd u;
          std::vector<Eigen::Vector2d> vertices;
          bool solved = scfr_solver::calcSCFR(poses_,
                                              As_,
                                              bs_,
                                              Cs_,
                                              dls_,
                                              dus_,
                                              this->scfrConstraint_->A_robot()->mass(),
                                              M,
                                              l,
                                              u,
                                              vertices,
                                              this->breakableSCFRParam_
                                              );
          if (solved) { // 接触をbreakしても良い
            if(mgns[i] < minMargin) {
              minMargin = mgns[i];
              breakableIdx = i;
            }
          }
        }
      }
      if (this->debugLevel_>=1) {
        std::cerr << "KeepCollisionScfrConstraint" << " : " << contactIdxs.size() <<" contacts " << timer.measure() << " [s] for calcSCFRs" << std::endl;
      }
    }

    // 考慮するthis->keepCollisionANDConstraints_を作り、SCFR計算のための値を取得する
    {
      this->keepCollisionANDConstraints_->children().clear();
      this->scfrConstraint_->poses().clear();
      this->scfrConstraint_->As().clear();
      this->scfrConstraint_->bs().clear();
      this->scfrConstraint_->Cs().clear();
      this->scfrConstraint_->dls().clear();
      this->scfrConstraint_->dus().clear();

      for (int i=0; i<contactIdxs.size(); i++) {
        if (i == breakableIdx) continue; // この接触は離してもいい
        this->keepCollisionANDConstraints_->children().push_back(this->keepCollisionConstraints_[contactIdxs[i]]);
        this->scfrConstraint_->poses().push_back(poses[i]);
        this->scfrConstraint_->As().emplace_back(0,6);
        this->scfrConstraint_->bs().emplace_back(0);
        this->scfrConstraint_->Cs().push_back(Cs[i]);
        this->scfrConstraint_->dls().push_back(dls[i]);
        this->scfrConstraint_->dus().push_back(dus[i]);
      }
    }

    Eigen::VectorXd keepCollisionEq;
    Eigen::VectorXd keepCollisionMinIneq;
    Eigen::VectorXd keepCollisionMaxIneq;
    { // this->keepCollisionANDConstraints_->updateBounds() だが各chilerenのupdateBoundsは終わっているのでそれ以外を行う
      std::vector<std::reference_wrapper<const Eigen::VectorXd> > eqs;eqs.reserve(this->keepCollisionANDConstraints_->children().size());
      std::vector<std::reference_wrapper <const Eigen::VectorXd> > minIneqs;minIneqs.reserve(this->keepCollisionANDConstraints_->children().size());
      std::vector<std::reference_wrapper<const Eigen::VectorXd> > maxIneqs;maxIneqs.reserve(this->keepCollisionANDConstraints_->children().size());
      int num_eqs = 0;
      int num_ineqs = 0;
      for(int i=0;i<this->keepCollisionANDConstraints_->children().size();i++) {
        eqs.emplace_back(this->keepCollisionANDConstraints_->children()[i]->getEq());
        minIneqs.emplace_back(this->keepCollisionANDConstraints_->children()[i]->getMinIneq());
        maxIneqs.emplace_back(this->keepCollisionANDConstraints_->children()[i]->getMaxIneq());

        num_eqs += eqs[i].get().rows();
        num_ineqs += minIneqs[i].get().rows();
      }

      keepCollisionEq.resize(num_eqs);
      keepCollisionMinIneq.resize(num_ineqs);
      keepCollisionMaxIneq.resize(num_ineqs);

      int idx_eq = 0;
      int idx_ineq = 0;
      for(size_t i=0;i<this->keepCollisionANDConstraints_->children().size(); i++){
        keepCollisionEq.segment(idx_eq,eqs[i].get().rows()) = eqs[i].get();
        idx_eq += eqs[i].get().rows();
        keepCollisionMinIneq.segment(idx_ineq,minIneqs[i].get().rows()) = minIneqs[i].get();
        keepCollisionMaxIneq.segment(idx_ineq,minIneqs[i].get().rows()) = maxIneqs[i].get();
        idx_ineq += minIneqs[i].get().rows();
      }
    }

    // SCFRを計算し、重心制約を求める
    this->scfrConstraint_->updateBounds();

    // ANDConstraintとして各種制約をまとめる
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > eqs;eqs.reserve(this->children_.size());
    std::vector<std::reference_wrapper <const Eigen::VectorXd> > minIneqs;minIneqs.reserve(this->children_.size());
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > maxIneqs;maxIneqs.reserve(this->children_.size());

    int num_eqs = 0;
    int num_ineqs = 0;
    for(int i=0;i<this->children_.size();i++) {
      if (i==0) eqs.emplace_back(keepCollisionEq);
      else eqs.emplace_back(this->children_[i]->getEq());
      if (i==0) minIneqs.emplace_back(keepCollisionMinIneq);
      else minIneqs.emplace_back(this->children_[i]->getMinIneq());
      if (i==0) maxIneqs.emplace_back(keepCollisionMaxIneq);
      else maxIneqs.emplace_back(this->children_[i]->getMaxIneq());

      num_eqs += eqs[i].get().rows();
      num_ineqs += minIneqs[i].get().rows();
    }

    this->eq_.resize(num_eqs);
    this->minIneq_.resize(num_ineqs);
    this->maxIneq_.resize(num_ineqs);

    int idx_eq = 0;
    int idx_ineq = 0;
    for(size_t i=0;i<this->children_.size(); i++){
      this->eq_.segment(idx_eq,eqs[i].get().rows()) = eqs[i].get();
      idx_eq += eqs[i].get().rows();
      this->minIneq_.segment(idx_ineq,minIneqs[i].get().rows()) = minIneqs[i].get();
      this->maxIneq_.segment(idx_ineq,minIneqs[i].get().rows()) = maxIneqs[i].get();
      idx_ineq += minIneqs[i].get().rows();
    }

    if(this->debugLevel_>=2){
      std::cerr << "KeepCollisionScfrConstraint" << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }
  }

  std::shared_ptr<ik_constraint2::IKConstraint> KeepCollisionScfrConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<KeepCollisionScfrConstraint> ret = std::make_shared<KeepCollisionScfrConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void KeepCollisionScfrConstraint::copy(std::shared_ptr<KeepCollisionScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    //    ANDConstraint::copy(ret,modelMap);
    ret->keepCollisionConstraints().clear();
    for (int i=0; i<this->keepCollisionConstraints_.size(); i++) {
      ret->keepCollisionConstraints().push_back(std::static_pointer_cast<ik_constraint2::KeepCollisionConstraint>(this->keepCollisionConstraints_[i]->clone(modelMap)));
    }
    ret->keepCollisionANDConstraints() = std::static_pointer_cast<ik_constraint2::ANDConstraint>(this->keepCollisionANDConstraints_->clone(modelMap));
    ret->scfrConstraint() = std::static_pointer_cast<ik_constraint2_scfr::ScfrConstraint>(this->scfrConstraint_->clone(modelMap));
    ret->children_.clear();
    // ret->children_.push_back(ret->keepCollisionANDConstraints());
    // ret->children_.push_back(ret->scfrConstraint()); // この順番でpush_backすることでSCFR計算時すでに干渉計算が終わっているようにする
  }
}
