#ifndef CYCLIC_LANDMARKS_LANDMARK_CONSTRAINTS_H
#define CYCLIC_LANDMARKS_LANDMARK_CONSTRAINTS_H

#include "../algorithms/named_vector.h"
#include "../lp/lp_solver.h"
#include "constraint_generator.h"

#include <set>

namespace options {
class Options;
}

namespace landmarks {
class LandmarkFactory;
class LandmarkGraph;
}

namespace operator_counting {
void add_lm_constraint_option_to_parser(options::OptionParser &parser);

class LandmarkConstraints : public ConstraintGenerator {
    const options::Options &opts;

    static lp::LPConstraint compute_constraint(
        const std::set<int> &actions, double infinity);
public:
    explicit LandmarkConstraints(const options::Options &opts);
    virtual void initialize_constraints(
        const std::shared_ptr<AbstractTask> &task,
        named_vector::NamedVector<lp::LPConstraint> &constraints,
        double infinity) override;
    virtual bool update_constraints(const State &state,
                                    lp::LPSolver &lp_solver) override;

    static void add_lm_constraint_option_to_parser(
        options::OptionParser &parser);
};
}

#endif
