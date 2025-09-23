#ifndef __BUGGER_INTERFACES_HIGHSINTERFACE_HPP__
#define __BUGGER_INTERFACES_HIGHSINTERFACE_HPP__

#include "Highs.h"
#include "bugger/interfaces/SolverInterface.hpp"

namespace bugger {
template <typename REAL> class HighsInterface : public SolverInterface<REAL> {
	Highs highs;

public:
  void doSetUp(SolverSettings &settings, const Problem<REAL> &problem,
               const Solution<REAL> &solution) override {
    this->adjustment = &settings;
    this->model = &problem;
    this->reference = &solution;
    bool solution_exists = this->reference->status == SolutionStatus::kFeasible;
    int ncols = this->model->getNCols();
    int nrows = this->model->getNRows();
    const auto &varNames = this->model->getVariableNames();
    const auto &consNames = this->model->getConstraintNames();
    const auto &domains = this->model->getVariableDomains();
    const auto &obj = this->model->getObjective();
    const auto &consMatrix = this->model->getConstraintMatrix();
    const auto &lhs_values = consMatrix.getLeftHandSides();
    const auto &rhs_values = consMatrix.getRightHandSides();
    const auto &cflags = this->model->getColFlags();
    const auto &rflags = this->model->getRowFlags();

    this->set_parameters();

    HighsModel model;

    model.lp_.sense_ = obj.sense ? ObjSense::kMinimize : ObjSense::kMaximize;
    model.lp_.offset_ = obj.offset;

    std::vector<HighsVarType> integrality;
    std::vector<double> col_lower, col_upper, row_lower, row_upper;
    col_lower.reserve(ncols);
    col_upper.reserve(ncols);

    if (solution_exists) {
      this->value = this->model->getPrimalObjective(solution);
    } else if (this->reference->status == SolutionStatus::kUnbounded) {
      this->value = obj.sense ? -kHighsInf : kHighsInf;
    } else if (this->reference->status == SolutionStatus::kInfeasible) {
      this->value = obj.sense ? kHighsInf : -kHighsInf;
    }

    for (int col = 0; col < ncols; ++col) {
      if (cflags[col].test(ColFlag::kFixed)) {
        continue;
      }
      double lb = cflags[col].test(ColFlag::kLbInf) ? -kHighsInf : static_cast<double>(domains.lower_bounds[col]);
      double ub = cflags[col].test(ColFlag::kLbInf) ? kHighsInf : static_cast<double>(domains.upper_bounds[col]);
      assert(!cflags[col].test(ColFlag::kInactive) || lb == ub);
      HighsVarType type;
      if (cflags[col].test(ColFlag::kIntegral)) {
        type = HighsVarType::kInteger;
      } else if (cflags[col].test(ColFlag::kImplInt)) {
        type = HighsVarType::kImplicitInteger;
      } else {
        type = HighsVarType::kContinuous;
      }
      col_lower.push_back(lb);
      col_upper.push_back(ub);
      integrality.push_back(type);
    }

    model.lp_.num_col_ = integrality.size();
    model.lp_.col_lower_ = col_lower;
    model.lp_.col_upper_ = col_upper;
    model.lp_.integrality_ = integrality;
    model.lp_.a_matrix_.format_ = MatrixFormat::kColwise;

    for (int row = 0; row < nrows; ++row){
      if (rflags[row].test(RowFlag::kRedundant)) {
        continue;
      }
      assert(!rflags[row].test(RowFlag::kLhsInf) || !rflags[row].test(RowFlag::kRhsInf));
      const auto& rowvec = consMatrix.getRowCoefficients(row);
      const auto& rowinds = rowvec.getIndices( );
      const auto& rowvals = rowvec.getValues( );
      int nrowcols = rowvec.getLength( );
      HighsSparseMatrix sparse_row;
      sparse_row.format_ = MatrixFormat::kRowwise;
      sparse_row.num_col_ = nrowcols;
      sparse_row.num_row_ = 1;
      sparse_row.start_ = {0};
      sparse_row.index_ = rowinds;
      sparse_row.value_ = rowvals;

      double lhs = rflags[row].test(RowFlag::kLhsInf) ? -kHighsInf : static_cast<double>(lhs_values[row]);
      double rhs = rflags[row].test(RowFlag::kRhsInf) ? kHighsInf : static_cast<double>(rhs_values[row]);
      for ( int col = 0; col < ncols; ++col) {
        assert(!cflags[rowinds[i]].test(ColFlag::kFixed));
        assert(rowvals[i] != 0);
      }
      model.lp_.a_matrix_.addRows(sparse_row);
    }

    // TODO: column/row names?

    // TODO: incomplete. See ScipRealInterface.hpp for what's missing.

    highs.passModel(model);
  }
  std::pair<char, SolverStatus> solve(const Vec<int> &passcodes) override {
    return {0, SolverStatus::kUnknown};
  }
};

template <typename REAL> class HighsFactory : public SolverFactory<REAL> {
public:
  void addParameters(ParameterSet &parameterset) override {}
  std::unique_ptr<SolverInterface<REAL>>
  create_solver(const Message &msg) override {
    std::unique_ptr<SolverInterface<REAL>> highs;
    return highs;
  }
};

template <typename REAL>
std::shared_ptr<SolverFactory<REAL>> load_solver_factory() {
  return std::shared_ptr<SolverFactory<REAL>>(new HighsFactory<REAL>());
}

} // namespace bugger
#endif
