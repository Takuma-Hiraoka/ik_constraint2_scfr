#ifndef IK_CONSTRAINT2_SCFRCONSTRAINT_H
#define IK_CONSTRAINT2_SCFRCONSTRAINT_H

#include <ik_constraint2/COMConstraint.h>
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
    const scfr_solver::SCFRParam& SCFRparam() const { return SCFRparam_;}
    scfr_solver::SCFRParam& SCFRparam() { return SCFRparam_;}

    // A_robotのCOMをSCFRに留める
    virtual void updateBounds() override;
  protected:
    std::vector<cnoid::Isometry3> poses_;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As_;
    std::vector<cnoid::VectorX> bs_;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs_;
    std::vector<cnoid::VectorX> dls_;
    std::vector<cnoid::VectorX> dus_;
    scfr_solver::SCFRParam SCFRparam_;

    std::vector<cnoid::Isometry3> prevPoses_;
  };
}

#endif
