#include "lm_constraints.h"

#include "../cyclic_landmarks/disj_action_landmark_graph.h"
#include "../cyclic_landmarks/landmark_constraint_handler.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace landmarks;
using namespace std;

namespace operator_counting {
LandmarkConstraints::LandmarkConstraints(const options::Options &opts)
    : opts(opts) {
}

void LandmarkConstraints::initialize_constraints(
    const shared_ptr<AbstractTask> &task,
    named_vector::NamedVector<lp::LPConstraint> & /*constraints*/,
    double /*infinity*/) {
    LandmarkConstraintHandler::get_instance().initialize(opts, task);
}

bool LandmarkConstraints::update_constraints(const State &state,
                                             lp::LPSolver &lp_solver) {
    shared_ptr<DisjActionLandmarkGraph> lm_graph =
        LandmarkConstraintHandler::get_instance().get_lm_graph(state);
    vector<lp::LPConstraint> constraints{};
    constraints.reserve(lm_graph->get_number_of_landmarks());
    for (size_t id = 0; id < lm_graph->get_number_of_landmarks(); ++id) {
        constraints.push_back(
            compute_constraint(lm_graph->get_actions(id),
                               lp_solver.get_infinity()));
    }

    lp_solver.add_temporary_constraints(constraints);
    return false;
}

lp::LPConstraint LandmarkConstraints::compute_constraint(
    const set<int> &actions, double infinity) {
    lp::LPConstraint constraint(1.0, infinity);
    for (int op_id : actions) {
        assert(op_id >= 0);
        constraint.insert(op_id, 1.0);
    }
    return constraint;
}

void add_lm_constraint_option_to_parser(OptionParser &parser) {
    parser.add_option<shared_ptr<LandmarkFactory>>(
        "lm", "Method to produce landmarks");
}

static shared_ptr<ConstraintGenerator> _parse(OptionParser &parser) {
    parser.document_synopsis("Landmark constraints", "");

    operator_counting::add_lm_constraint_option_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }
    return make_shared<LandmarkConstraints>(opts);
}

static Plugin<ConstraintGenerator> _plugin("lm_constraints", _parse);
}
