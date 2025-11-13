#include <ik_constraint2_scfr/KeepCollisionScfrConstraint.h>
#include <clpeigen/clpeigen.h>
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
          pose.translation() = this->keepCollisionConstraints_[i]->B_currentLocalp();
          cnoid::Vector3d z_axis = this->keepCollisionConstraints_[i]->currentDirection();
          cnoid::Vector3d x_axis = (z_axis==cnoid::Vector3d::UnitY() || z_axis==-cnoid::Vector3d::UnitY()) ? cnoid::Vector3d::UnitZ() : cnoid::Vector3d::UnitY().cross(z_axis);
          cnoid::Vector3d y_axis = z_axis.cross(x_axis);
          pose.linear().col(0) = x_axis.normalized(); pose.linear().col(1) = y_axis.normalized(); pose.linear().col(2) = z_axis.normalized();
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
      if (contactIdxs.size() > this->minimumContactCount_) { // 少なくともminimumContactCount_点で触れるようにする
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
          bool solved = checkSCFRExistance(poses_,
                                           As_,
                                           bs_,
                                           Cs_,
                                           dls_,
                                           dus_,
                                           this->scfrConstraint_->A_robot()->mass(),
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
      ret->keepCollisionConstraints().push_back(std::static_pointer_cast<ik_constraint2::CollisionConstraint>(this->keepCollisionConstraints_[i]->clone(modelMap)));
    }
    ret->keepCollisionANDConstraints() = std::static_pointer_cast<ik_constraint2::ANDConstraint>(this->keepCollisionANDConstraints_->clone(modelMap));
    ret->scfrConstraint() = std::static_pointer_cast<ik_constraint2_scfr::ScfrConstraint>(this->scfrConstraint_->clone(modelMap));
    ret->children_.clear();
  }

    inline bool appendRow(const std::vector<Eigen::VectorXd>& vs, Eigen::VectorXd& vout){
    size_t rows = 0;
    for(size_t i=0;i<vs.size();i++){
      rows += vs[i].size();
    }
    vout.resize(rows);
    size_t idx = 0;
    for(size_t i=0;i<vs.size();i++){
      vout.segment(idx,vs[i].size()) = vs[i];
      idx += vs[i].size();
    }

    return true;
  }

  inline bool appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(Mout.rows(),0);//もとのMoutのrowを用いる
      return true;
    }
    size_t rows = Ms[0].rows();
    size_t cols = 0;
    for(size_t i=0;i<Ms.size();i++){
      cols += Ms[i].cols();
      if(Ms[i].rows() != rows){
        std::cerr << "[appendCol] Ms[i].rows() " << Ms[i].rows() << " != rows " << rows << std::endl;
        return false;
      }
    }
    Mout.resize(rows,cols);
    size_t idx = 0;
    for(size_t i=0;i<Ms.size();i++){
      Mout.middleCols(idx,Ms[i].cols()) = Ms[i];
      idx += Ms[i].cols();
    }

    return true;
  }

  inline bool appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(0,0);
      return true;
    }
    size_t cols = 0;
    size_t rows = 0;
    for(size_t i=0;i<Ms.size();i++){
      rows += Ms[i].rows();
      cols += Ms[i].cols();
    }
    Mout.resize(rows,cols);
    size_t idx_row = 0;
    size_t idx_col = 0;
    for(size_t i=0;i<Ms.size();i++){
      Eigen::SparseMatrix<double, Eigen::ColMajor> M_ColMajor(Ms[i].rows(),cols);
      M_ColMajor.middleCols(idx_col,Ms[i].cols()) = Ms[i];
      Mout.middleRows(idx_row,M_ColMajor.rows()) = M_ColMajor;
      idx_row += Ms[i].rows();
      idx_col += Ms[i].cols();
    }

    return true;
  }

  bool checkSCFRExistance(std::vector<Eigen::Isometry3d>& poses,
                          const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As, // pose local frame. 列は6(F N)
                          const std::vector<Eigen::VectorXd>& bs,
                          const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs, // pose local frame. 列は6(F N)
                          const std::vector<Eigen::VectorXd>& dls,
                          const std::vector<Eigen::VectorXd>& dus,
                          const double& m, // robotの質量
                          const scfr_solver::SCFRParam& param) {
    if(poses.size() != As.size() ||
       poses.size() != bs.size() ||
       poses.size() != Cs.size() ||
       poses.size() != dls.size() ||
       poses.size() != dus.size()){
      std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch" << std::endl;
      return false;
    }

    if(poses.size() == 0) {
      return true;
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd b;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C;
    Eigen::VectorXd dl;
    Eigen::VectorXd du;
    {
      // compute A, b, C, dl, du
      // x = [dpx dpy dw1 dw2 ...]^T
      // wはpose座標系．poseまわり.サイズは6

      // Grasp Matrix Gx = h
      // 原点まわりのつりあい. 座標軸はworld系の向き
      Eigen::SparseMatrix<double,Eigen::RowMajor> G;
      Eigen::VectorXd h = Eigen::VectorXd::Zero(6);
      {
        h[2] = m*9.80665;

        std::vector<Eigen::SparseMatrix<double,Eigen::ColMajor> > Gs;
        {
          Eigen::SparseMatrix<double,Eigen::ColMajor> G01(6,2);
          G01.insert(3,1) = -m*9.80665;
          G01.insert(4,0) = m*9.80665;
          Gs.push_back(G01);
        }

        for(size_t i=0;i<poses.size();i++){
          Eigen::SparseMatrix<double,Eigen::ColMajor> GraspMatrix(6,6);
          {
            const Eigen::Isometry3d& pos = poses[i];
            const Eigen::Matrix3d& R = pos.linear();
            Eigen::Matrix3d p_x;
            p_x << 0.0, -pos.translation()(2), pos.translation()(1),
              pos.translation()(2), 0.0, -pos.translation()(0),
              -pos.translation()(1), pos.translation()(0), 0.0;
            const Eigen::Matrix3d& p_x_R = p_x * R;

            std::vector<Eigen::Triplet<double> > G_tripletList;
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(row,col,R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(3+row,col,p_x_R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(3+row,3+col,R(row,col)));
              }
            }
            GraspMatrix.setFromTriplets(G_tripletList.begin(), G_tripletList.end());
          }
          Gs.push_back(GraspMatrix);
        }

        Eigen::SparseMatrix<double,Eigen::ColMajor> G_ColMajor;
        appendCol(Gs,G_ColMajor);
        G = G_ColMajor;
      }

      //接触力制約
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_contact;
      Eigen::VectorXd b_contact;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_contact;
      Eigen::VectorXd dl_contact;
      Eigen::VectorXd du_contact;
      {
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > A_contacts;
        std::vector<Eigen::VectorXd> b_contacts;
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > C_contacts;
        std::vector<Eigen::VectorXd> dl_contacts;
        std::vector<Eigen::VectorXd> du_contacts;

        {
          Eigen::SparseMatrix<double,Eigen::RowMajor> A01(0,2);
          Eigen::VectorXd b01(0);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C01(0,2);
          Eigen::VectorXd dl01(0);
          Eigen::VectorXd du01(0);
          A_contacts.push_back(A01);
          b_contacts.push_back(b01);
          C_contacts.push_back(C01);
          dl_contacts.push_back(dl01);
          du_contacts.push_back(du01);
        }

        for (size_t i=0;i<poses.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A = As[i];
          Eigen::VectorXd b = bs[i];
          Eigen::SparseMatrix<double,Eigen::RowMajor> C = Cs[i];
          Eigen::VectorXd dl = dls[i];
          Eigen::VectorXd du = dus[i];
          if(A.cols() != 6 ||
             C.cols() != 6 ||
             A.rows() != b.size() ||
             C.rows() != dl.size() ||
             C.rows() != du.size()){
            std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch" << std::endl;
            return false;
          }
          A_contacts.push_back(A);
          b_contacts.push_back(b);
          C_contacts.push_back(C);
          dl_contacts.push_back(dl);
          du_contacts.push_back(du);
        }
        appendDiag(A_contacts,A_contact);
        appendRow(b_contacts,b_contact);
        appendDiag(C_contacts,C_contact);
        appendRow(dl_contacts,dl_contact);
        appendRow(du_contacts,du_contact);
      }

      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(G.rows()+A_contact.rows(),G.cols());
      A.topRows(G.rows()) = G;
      A.bottomRows(A_contact.rows()) = A_contact;
      b = Eigen::VectorXd(h.size()+b_contact.size());
      b.head(h.rows()) = h;
      b.tail(b_contact.rows()) = b_contact;
      C = C_contact;
      dl = dl_contact;
      du = du_contact;
    }

    if(param.debugLevel){
      std::cerr << "before check" << std::endl;
      std::cerr << "A" << std::endl << A << std::endl;
      std::cerr << "b" << std::endl << b << std::endl;
      std::cerr << "C" << std::endl << C << std::endl;
      std::cerr << "dl" << std::endl << dl << std::endl;
      std::cerr << "du" << std::endl << du << std::endl;
    }

    // initialize solver
    // lbM <= Mx <= ubM
    // lb <= x <= ub
    Eigen::SparseMatrix<double,Eigen::RowMajor> M(A.rows()+C.rows(),A.cols());
    M.topRows(A.rows()) = A;
    M.bottomRows(C.rows()) = C;
    Eigen::VectorXd lbM(A.rows()+C.rows());
    lbM.head(b.rows()) = b;
    lbM.tail(dl.rows()) = dl;
    Eigen::VectorXd ubM(A.rows()+C.rows());
    ubM.head(b.rows()) = b;
    ubM.tail(du.rows()) = du;
    Eigen::VectorXd lb(A.cols());
    for(size_t i=0;i<lb.rows();i++) lb[i] = -std::numeric_limits<double>::max();
    Eigen::VectorXd ub(A.cols());
    for(size_t i=0;i<ub.rows();i++) ub[i] = std::numeric_limits<double>::max();

    Eigen::VectorXd o = Eigen::VectorXd::Zero(M.cols());
    clpeigen::solver solver;
    solver.initialize(o,M,lbM,ubM,lb,ub,param.debugLevel);
    solver.model().setPrimalTolerance(param.lpTolerance);//default 1e-7. vertexを見逃さないようにする
    solver.model().setDualTolerance(param.lpTolerance);

    Eigen::VectorXd solution;
    o.head<2>() = Eigen::Vector2d(1,0); // 適当
    solver.updateObjective(o);
    if (solver.solve()) {
      return true;
    } else {
      std::cerr << "[" <<__FUNCTION__ << "] failed" << std::endl;
      return false;
    }

  }

}
