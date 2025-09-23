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

    model.lp_.num_col_ = ncols;
    model.lp_.num_row_ = nrows;
    model.lp_.sense_ = obj.sense ? ObjSense::kMinimize : ObjSense::kMaximize;
    model.lp_.offset_ = obj.offset;

    this->vars.resize(ncols);
    if (solution_exists) {
      this->value = this->model->getPrimalObjective(solution);
    } else if (this->reference->status == SolutionStatus::kUnbounded) {
      this->value = obj.sense ? -kHighsInf : kHighsInf;
    } else if (this->reference->status == SolutionStatus::kInfeasible) {
      this->value = obj.sense ? kHighsInf : -kHighsInf;
    }

    // Copy the matrix across (TODO: is it really a copy?)
    model.lp_.a_matrix_.format_ = MatrixFormat::kRowwisePartitioned;
    auto extract_start = [](IndexRange r) -> int { return r.start; };
    auto extract_end = [](IndexRange r) -> int { return r.end; };
    const int nMatrixRows =
        consMatrix.getRowRangesVec().size(); // TODO: probably don't need this
    const std::vector<IndexRange> &rowRanges = consMatrix.getRowRangesVec();
    model.lp_.a_matrix_.start_.reserve(nMatrixRows);
    std::transform(rowRanges.begin(), rowRanges.end(),
                   model.lp_.a_matrix_.start_.begin(), extract_start);
    model.lp_.a_matrix_.p_end_.reserve(nMatrixRows);
    std::transform(rowRanges.begin(), rowRanges.end(),
                   model.lp_.a_matrix_.p_end_.begin(), extract_end);
    model.lp_.a_matrix_.index_ = consMatrix.getColumnsVec();
    model.lp_.a_matrix_.value_ = consMatrix.getValuesVec();

    // Copy the integrality across
    model.lp_.integrality_.resize(model.lp_.num_col_);
	  for (int col = 0; col < ncols; ++col) {
      HighsVarType type;
      if (cflags[col].test(ColFlag::kIntegral)) {
        type = HighsVarType::kInteger;
      } else if (cflags[col].test(ColFlag::kImplInt)){
        type = HighsVarType::kImplicitInteger;
      } else {
        type = HighsVarType::kContinuous;
      }
      model.lp_.integrality_[col] = type;
	  }

    // TODO: column/row names?
    // TODO: copy col and row bounds across

    // Delete unnecessary columns and rows

    HighsIndexCollection cols_to_delete, rows_to_delete;
    cols_to_delete.is_set_ = true;
    rows_to_delete.is_set_ = true;

    for (int col = 0; col < ncols; ++col) {
      if (cflags[col].test(ColFlag::kFixed)){
        cols_to_delete.set_.push_back(col);
      }
    }

    for (int row = 0; row < nrows; ++row) {
      assert(!rflags[row].test(RowFlag::kLhsInf) || !rflags[row].test(RowFlag::kRhsInf));
      if (rflags[row].test(RowFlag::kRedundant)) {
        rows_to_delete.set_.push_back(row);
      }
    }

    // Do rows first because it's already row-wise
    model.lp_.deleteRows(rows_to_delete);
    model.lp_.ensureColwise(); // deleteCols can't handle row-wise "yet"
    model.lp_.deleteCols(cols_to_delete);

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
