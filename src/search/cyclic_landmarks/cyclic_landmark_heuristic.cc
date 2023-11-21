#include "cyclic_landmark_heuristic.h"

#include "landmark_constraint_handler.h"

#include "../landmarks/landmark_status_manager.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace landmarks;
using namespace lp;
using namespace named_vector;
using namespace operator_counting;
using namespace std;

namespace cyclic_landmark_heuristic {
CyclicLandmarkHeuristic::CyclicLandmarkHeuristic(const options::Options &opts)
    : Heuristic(opts),
      lp_solver(opts.get<lp::LPSolverType>("lpsolver")),
      ip(opts.get<bool>("use_integer_operator_counts")),
      path_dependent(opts.get<bool>("path_dependent")) {
    LandmarkConstraintHandler::get_instance().initialize(opts, task);
    prepare_constraint_generators(opts);
    prepare_linear_program();
}

void CyclicLandmarkHeuristic::prepare_constraint_generators(
    const options::Options &opts) {
    constraint_generators.emplace_back(make_shared<LandmarkConstraints>(opts));
    if (opts.get<CycleGenerator>("cycle_generator")
        == CycleGenerator::NONE) {
        utils::g_log << "Computing landmark heuristic without cycles."
                     << endl;
    } else if (path_dependent && LandmarkConstraintHandler::get_instance()
        .initial_fact_landmark_graph_is_acyclic()) {
        utils::g_log << "Initial landmark graph is acyclic. The cycle "
                        "generator is discarded because path-dependent "
                        "landmark progression is used." << endl;
    } else {
        constraint_generators.emplace_back(
            make_shared<CycleConstraints>(opts));
    }
}

void CyclicLandmarkHeuristic::prepare_linear_program() {
    utils::g_log << "Preparing " << (ip ? "IP" : "LP") << "..." << endl;
    NamedVector<LPVariable> variables = prepare_lp_variables();
    NamedVector<LPConstraint> constraints = prepare_lp_constraints();
    lp_solver.load_problem(LinearProgram(
                               LPObjectiveSense::MINIMIZE, move(variables), move(constraints)));
}

NamedVector<lp::LPVariable> CyclicLandmarkHeuristic::prepare_lp_variables() {
    double infinity = lp_solver.get_infinity();
    NamedVector<LPVariable> variables;
    for (OperatorProxy op : task_proxy.get_operators()) {
        int op_cost = op.get_cost();
        variables.emplace_back(0, infinity, op_cost, ip);
    }
    return variables;
}

NamedVector<LPConstraint> CyclicLandmarkHeuristic::prepare_lp_constraints() {
    double infinity = lp_solver.get_infinity();
    NamedVector<LPConstraint> constraints;
    for (const auto &generator : constraint_generators) {
        generator->initialize_constraints(task, constraints, infinity);
    }
    return constraints;
}

int CyclicLandmarkHeuristic::compute_heuristic(const State &state) {
    assert(!lp_solver.has_temporary_constraints());
    int h = DEAD_END;
    if (update_constraints_dead_end_free(state)) {
        h = compute_heuristic_value();
    }
    lp_solver.clear_temporary_constraints();
    return h;
}

bool CyclicLandmarkHeuristic::update_constraints_dead_end_free(
    const State &state) {
    auto update_denotes_dead_end =
        [&](shared_ptr<ConstraintGenerator> &generator) {
            return generator->update_constraints(state, lp_solver);
        };
    return none_of(constraint_generators.begin(),
                   constraint_generators.end(),
                   update_denotes_dead_end);
}

int CyclicLandmarkHeuristic::compute_heuristic_value() {
    lp_solver.solve();
    if (lp_solver.has_optimal_solution()) {
        double epsilon = 0.01;
        double objective_value = lp_solver.get_objective_value();
        return ceil(objective_value - epsilon);
    } else {
        return DEAD_END;
    }
}

void CyclicLandmarkHeuristic::print_statistics() const {
    LandmarkConstraintHandler::get_instance().print_statistics();
}

void CyclicLandmarkHeuristic::get_path_dependent_evaluators(
    set<Evaluator *> &evals) {
    if (path_dependent) {
        evals.insert(this);
    }
}

void CyclicLandmarkHeuristic::notify_initial_state(
    const State & /*initial_state*/) {
    assert(path_dependent);
    // Nothing to do here, initialization happens later.
}

void CyclicLandmarkHeuristic::notify_state_transition(
    const State &parent_state, OperatorID op_id,
    const State &current_state) {
    assert(path_dependent);
    LandmarkStatusManager &manager =
        LandmarkConstraintHandler::get_instance().get_lm_status_manager();
    manager.update_accepted_landmarks(parent_state, op_id, current_state);
    manager.update_lm_status(current_state);
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis("Cyclic landmark heuristic", "");
    parser.add_option<bool>(
        "use_integer_operator_counts",
        "restrict operator counting variables to integer values. Computing the "
        "heuristic with integer variables can produce higher values but "
        "requires solving a MIP instead of an LP which is generally more "
        "computationally expensive. Turning this option on can thus drastically "
        "increase the runtime.",
        "false");
    parser.add_option<bool>(
        "path_dependent",
        "if true, only one LM-graph is computed in the beginning and updated "
        "based on paths expanded during search.",
        "true");

    landmarks::add_status_manager_options_to_parser(parser);
    operator_counting::add_lm_constraint_option_to_parser(parser);
    operator_counting::add_cycle_constraint_option_to_parser(parser);

    parser.document_language_support("action costs", "supported");
    // TODO: interesting line of future work
    parser.document_language_support("conditional effects",
                                     "not supported");
    parser.document_language_support("axioms", "not supported");
    parser.document_property("admissible", "yes");
    parser.document_property("consistent",
                             "no"); // Because of LM-generators
    parser.document_property("safe", "?"); // TODO: check
    // TODO: prefer operators that are non-zero in the solution
    parser.document_property("preferred operators", "no");

    lp::add_lp_solver_option_to_parser(parser);
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();

    if (parser.help_mode() || parser.dry_run()) {
        return nullptr;
    }
    return make_shared<CyclicLandmarkHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("cycle", _parse);
}
