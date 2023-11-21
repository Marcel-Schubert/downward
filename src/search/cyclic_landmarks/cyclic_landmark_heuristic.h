#ifndef CYCLIC_LANDMARKS_CYCLIC_LANDMARK_HEURISTIC_H
#define CYCLIC_LANDMARKS_CYCLIC_LANDMARK_HEURISTIC_H

#include "../heuristic.h"

#include "../lp/lp_solver.h"
#include "../operator_counting/cycle_constraints.h"
#include "../operator_counting/lm_constraints.h"

namespace operator_counting {
class ConstraintGenerator;
class CycleConstraints;
class LandmarkConstraints;
}
namespace options {
class Options;
}

namespace cyclic_landmark_heuristic {
class CyclicLandmarkHeuristic : public Heuristic {
    lp::LPSolver lp_solver;
    const bool ip;
    const bool path_dependent;
    std::vector<std::shared_ptr<operator_counting::ConstraintGenerator>>
    constraint_generators;

    void prepare_constraint_generators(const options::Options &opts);
    void prepare_linear_program();
    named_vector::NamedVector<lp::LPVariable> prepare_lp_variables();
    named_vector::NamedVector<lp::LPConstraint> prepare_lp_constraints();

    bool update_constraints_dead_end_free(const State &global_state);
    int compute_heuristic_value();
protected:
    virtual int compute_heuristic(const State &global_state) override;
public:
    explicit CyclicLandmarkHeuristic(const options::Options &opts);

    void print_statistics() const override;
    virtual void get_path_dependent_evaluators(
        std::set<Evaluator *> &evals) override;
    virtual void notify_initial_state(
        const State &initial_state) override;
    virtual void notify_state_transition(
        const State &parent_state, OperatorID op_id,
        const State &current_state) override;
};
}

#endif
