#ifndef IK_CONSTRAINT2_SCFRCONSTRAINT_H
#define IK_CONSTRAINT2_SCFRCONSTRAINT_H

#include <ik_constraint2/ik_constraint2.h>
#include <scfr_solver/scfr_solver.h>

namespace ik_constraint2_scfr{
  class ScfrConstraint : public ik_constraint2::COMConstraint {
  public:
    ScfrConstraint() {
      this->weight_ = cnoid::Vector3::Zero();
    }

    const std::vector<cnoid::Isometry3>& poses() const { return poses_;}
    std::vector<cnoid::Isometry3>& poses() { return poses_;}
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As() const { return As_;}
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As() { return As_;}
    const std::vector<cnoid::VectorX>& bs() const { return bs_;}
    std::vector<cnoid::VectorX>& bs() { return bs_;}
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs() const { return Cs_;}
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs() { return Cs_;}
    const std::vector<cnoid::VectorX>& dls() const { return dls_;}
    std::vector<cnoid::VectorX>& dls() { return dls_;}
    const std::vector<cnoid::VectorX>& dus() const { return dus_;}
    std::vector<cnoid::VectorX>& dus() { return dus_;}
    const scfr_solver::SCFRParam& SCFRParam() const { return SCFRParam_;}
    scfr_solver::SCFRParam& SCFRParam() { return SCFRParam_;}

    // A_robotのCOMをSCFRに留める
    virtual void updateBounds() override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<ik_constraint2::IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<ScfrConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;
  protected:
    std::vector<cnoid::Isometry3> poses_;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As_;
    std::vector<cnoid::VectorX> bs_;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs_;
    std::vector<cnoid::VectorX> dls_;
    std::vector<cnoid::VectorX> dus_;
    scfr_solver::SCFRParam SCFRParam_;

    std::vector<cnoid::Isometry3> prevPoses_;
  };
}

#endif
