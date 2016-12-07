/**
 @file    optimization_variables.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 23, 2016
 @brief   Declares a class to publish the current optimization variables.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_

#include "a_subject.h"
#include "nlp_structure.h"

namespace xpp {
namespace opt {


/** @brief Publishes the current value of the optimization variables.
  *
  * This class is responsible for publishing the up-to-date values of the
  * optimization variables to all the observers (cost function,
  * constraints, visualizers,...) that registered to this. This class should
  * fit all types of optimization problems and does not have a specific set
  * of variables. The only problem specific information this class holds is the
  * std::string id of what the variables represent.
  *
  * https://sourcemaking.com/design_patterns/observer
  */
class OptimizationVariables : public ASubject {
public:
  using VectorXd = Eigen::VectorXd;
  using VecBound = VariableSet::VecBound;
  using VarBound = VariableSet::VarBound;
  using VariableSetVector = std::vector<VariableSet>;

  OptimizationVariables ();
  virtual ~OptimizationVariables ();

  void ClearVariables();

  VectorXd GetVariables(std::string id) const;
  VectorXd GetOptimizationVariables() const;
  VecBound GetOptimizationVariableBounds() const;
  int GetOptimizationVariableCount() const;
  VariableSetVector GetVarSets() const;

  void AddVariableSet(std::string id, const VectorXd& values,
                      const VarBound& bound  = AConstraint::kNoBound_);
  void SetVariables(const VectorXd& x);

private:
  VariableSetVector variable_sets_;
//  NlpStructure nlp_structure_; ///< this class holds all the structural information of the NLP

  VariableSet& GetSet(std::string id);
  const VariableSet& GetSet(std::string id) const;

};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_OPTIMIZATION_VARIABLES_H_ */
