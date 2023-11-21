#include "cycle_constraints.h"

#include "../cyclic_landmarks/landmark_constraint_handler.h"

#include "../cyclic_landmarks/cycle_oracle.h"
#include "../cyclic_landmarks/depth_first_oracle.h"
#include "../cyclic_landmarks/disj_action_landmark_graph.h"
#include "../cyclic_landmarks/floyd_warshall_oracle.h"

#include "../algorithms/johnson_cycle_detection.h"
#include "../utils/logging.h"
#include "../utils/system.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace landmarks;
using namespace std;

namespace operator_counting {
CycleConstraints::CycleConstraints(const options::Options &opts)
    : opts(opts),
      cycle_generator(opts.get<CycleGenerator>("cycle_generator")),
      strong(opts.get<bool>("strong")) {
}

void CycleConstraints::initialize_constraints(
    const shared_ptr<AbstractTask> &task,
    named_vector::NamedVector<lp::LPConstraint> & /*constraints*/,
    double /*infinity*/) {
    if (cycle_generator == CycleGenerator::NONE) {
        cerr << "Input error: cycle_generator missing in search call."
             << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
    LandmarkConstraintHandler::get_instance().initialize(opts, task);
}

bool CycleConstraints::update_constraints(const State &state,
                                          lp::LPSolver &lp_solver) {
    dalm_graph lm_graph =
        LandmarkConstraintHandler::get_instance().get_lm_graph(state);
    if (cycle_generator == CycleGenerator::johnson) {
        add_constraints_for_all_cycles(lm_graph, lp_solver);
        return false;
    } else {
        add_constraints_implicit_hitting_set_approach(lm_graph, lp_solver);
        return !lp_solver.has_optimal_solution();
    }
}

void CycleConstraints::add_constraints_for_all_cycles(
    const dalm_graph &lm_graph, lp::LPSolver &lp_solver) {
    adj_list adj = compute_adj_list(lm_graph);
    for (auto &list : adj) {
        sort(list.begin(), list.end());
    }
    vector<vector<int>> cycles =
        johnson_cycles::compute_elementary_cycles(adj);

    vector<lp::LPConstraint> constraints{};
    constraints.reserve(cycles.size());
    for (const vector<int> &cycle : cycles) {
        constraints.push_back(
            compute_constraint(lm_graph, cycle,
                               lp_solver.get_infinity()));
    }
    LandmarkConstraintHandler::num_cycles += cycles.size();

    if (first_call) {
        utils::g_log << "Cycle constraints in initial state: " << cycles.size() << endl;
        first_call = false;
    }
    lp_solver.add_temporary_constraints(constraints);
}

adj_list CycleConstraints::compute_adj_list(const dalm_graph &lm_graph) {
    size_t n = lm_graph->get_number_of_landmarks();
    adj_list adj(n, vector<int>{});
    for (size_t id = 0; id < n; ++id) {
        for (auto &parent : lm_graph->get_dependencies(id)) {
            adj[parent.first].push_back(id);
        }
    }
    return adj;
}

void CycleConstraints::add_constraints_implicit_hitting_set_approach(
    const dalm_graph &lm_graph, lp::LPSolver &lp_solver) {
    vector<map<int, bool>> adj = lm_graph->to_adj_list();
    shared_ptr<CycleOracle> oracle;
    switch (cycle_generator) {
    case CycleGenerator::floyd_warshall:
        oracle = make_shared<FloydWarshallOracle>(adj, !strong);
        break;
    case CycleGenerator::depth_first:
        oracle = make_shared<DepthFirstOracle>(adj, !strong);
        break;
    default:
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }

    vector<int> cycle;
    for (size_t iteration = 0; /* TODO: max iterations? */; ++iteration) {
        lp_solver.solve();
        if (!lp_solver.has_optimal_solution()) {
            if (first_call) {
                utils::g_log << "Cycle constraints in initial state: " << iteration << endl;
                first_call = false;
            }
            return;
        }
        vector<float> lm_count =
            compute_landmark_weights(lm_graph, lp_solver.extract_solution());
        cycle = oracle->find_cycle(lm_count);
        if (cycle.empty()) {
            if (first_call) {
                utils::g_log << "Cycle constraints in initial state: " << iteration << endl;
                first_call = false;
            }
            return;
        }
        lp::LPConstraint constraint = compute_constraint(
            lm_graph, cycle, lp_solver.get_infinity());
        ++LandmarkConstraintHandler::num_cycles;
        lp_solver.add_temporary_constraints({constraint});
    }
}

vector<float> CycleConstraints::compute_landmark_weights(
    const dalm_graph &lm_graph, const vector<double> &counts) {
    size_t n = lm_graph->get_number_of_landmarks();
    vector<float> weights(n, 0);
    for (size_t i = 0; i < n; ++i) {
        for (int op : lm_graph->get_actions(i)) {
            weights[i] += counts[op];
        }
    }
    return weights;
}

lp::LPConstraint CycleConstraints::compute_constraint(
    const dalm_graph &lm_graph, const vector<int> &cycle,
    double infinity) {
    map<int, size_t> counts = count_lm_occurrences(lm_graph, cycle);

    lp::LPConstraint constraint(counts[-1] + 1.0, infinity);
    counts.erase(-1);
    for (pair<int, size_t> op : counts) {
        constraint.insert(op.first, op.second);
    }
    return constraint;
}

map<int, size_t> CycleConstraints::count_lm_occurrences(
    const dalm_graph &lm_graph, const vector<int> &cycle) const {
    map<int, size_t> occurrences{};
    occurrences[-1] = 0; // Counts how many landmarks make up the constraint.
    for (size_t i = 0; i < cycle.size(); ++i) {
        int id = cycle[i];
        int succ = cycle[(i + 1) % cycle.size()];

        if (!strong ||
            lm_graph->get_ordering_type(id, succ) == OrderingType::weak) {
            occurrences[-1]++;
            for (int op : lm_graph->get_actions(succ)) {
                occurrences[op] =
                    occurrences.count(op) ? occurrences[op] + 1 : 1;
            }
        }
    }
    return occurrences;
}

void add_cycle_constraint_option_to_parser(OptionParser &parser) {
    parser.add_option<shared_ptr<LandmarkFactory>>(
        "lm", "Method to produce landmarks");
    parser.add_option<bool>("strong", "strong cycle constraints", "true");
    vector<string> cycle_generator;
    cycle_generator.push_back("NONE");
    cycle_generator.push_back("johnson");
    cycle_generator.push_back("floyd_warshall");
    cycle_generator.push_back("depth_first");
    parser.add_enum_option<CycleGenerator>(
        "cycle_generator", cycle_generator,
        "choose which algorithm is used to find cycles in the LM-graphs.",
        "NONE");
}

static shared_ptr<ConstraintGenerator> _parse(OptionParser &parser) {
    parser.document_synopsis("Cyclic landmark constraints", "");

    operator_counting::add_cycle_constraint_option_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }
    return make_shared<CycleConstraints>(opts);
}

static Plugin<ConstraintGenerator> _plugin("cycle_constraints", _parse);
}
