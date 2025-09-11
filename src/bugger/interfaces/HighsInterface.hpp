#ifndef __BUGGER_INTERFACES_HIGHSINTERFACE_HPP__
#define __BUGGER_INTERFACES_HIGHSINTERFACE_HPP__

#include "bugger/interfaces/SolverInterface.hpp"

namespace bugger {
template <typename REAL>
class HighsInterface : public SolverInterface<REAL>{
public:
	void doSetUp(SolverSettings& settings, const Problem<REAL>& problem, const Solution<REAL>& solution) override{
	}
	std::pair<char, SolverStatus>
	solve(const Vec<int>& passcodes) override {
return {0, SolverStatus::kUnknown};
	}
};

template <typename REAL>
class HighsFactory : public SolverFactory<REAL>{
public:
	void addParameters(ParameterSet& parameterset) override {}
	std::unique_ptr<SolverInterface<REAL>> create_solver(const Message& msg) override{
		std::unique_ptr<SolverInterface<REAL>> highs;
		return highs;
	}
};

   template <typename REAL>                                                                                                       std::shared_ptr<SolverFactory<REAL>>                                                                                           load_solver_factory( )                                                                                                         {                                                                                                                                 return std::shared_ptr<SolverFactory<REAL>>( new HighsFactory<REAL>( ) );                                                
   }

} // namespace bugger
#endif 
