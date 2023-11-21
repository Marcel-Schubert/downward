#include "landmark_constraint_handler.h"

#include "disj_action_landmark_graph.h"

#include "../landmarks/landmark_factory.h"
#include "../landmarks/status_manager_lama.h"
#include "../landmarks/status_manager_multi_path.h"
#include "../landmarks/status_manager_multi_path_consistent.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

using namespace std;

namespace landmarks {
long LandmarkConstraintHandler::num_cycles = 0;

LandmarkConstraintHandler &LandmarkConstraintHandler::get_instance() {
    static LandmarkConstraintHandler instance;
    return instance;
}

void LandmarkConstraintHandler::initialize(
    const options::Options &opts,
    const std::shared_ptr<AbstractTask> &original_task) {
    if (!initialized) {
        lm = opts.get<shared_ptr<LandmarkFactory>>("lm");
        task = original_task;
        task_proxy = make_shared<TaskProxy>(*task);
        path_dependent = opts.get<bool>("path_dependent");
        task_properties::verify_no_axioms(*task_proxy);
        task_properties::verify_no_conditional_effects(*task_proxy);
        initialized = true;
        if (path_dependent) {
            lm_status_manager_type =
                opts.get<LandmarkStatusManagerType>("status_manager");
            add_goal_atoms = opts.get<bool>("add_goal_atoms");
            add_gn_parents = opts.get<bool>("add_gn_parents");
            add_reasonable_children =
                opts.get<bool>("add_reasonable_children");
            lm_graph = lm->compute_lm_graph(task);
        }
    }
}

shared_ptr<DisjActionLandmarkGraph> LandmarkConstraintHandler::get_lm_graph(
    const State &state) {
    state.unpack();
    if (state.get_id() != last_state_id) {
        compute_lm_graph(state);
    }
    return dalm_graph;
}

void LandmarkConstraintHandler::compute_lm_graph(const State &state) {
    assert(state.get_id() != last_state_id);

    if (last_state_id == StateID::no_state) {
        compute_initial_lm_graph(state);
    } else {
        recompute_lm_graph(state);
    }
    last_state_id = state.get_id();
}

void LandmarkConstraintHandler::compute_initial_lm_graph(const State &state) {
    assert(last_state_id == StateID::no_state);

    if (path_dependent) {
        switch (lm_status_manager_type) {
        case LAMA:
            lm_status_manager = make_shared<StatusManagerLama>(*lm_graph);
            break;
        case MULTI_PATH:
            lm_status_manager = make_shared<StatusManagerMultiPath>(
                *lm_graph, add_goal_atoms, add_gn_parents,
                add_reasonable_children);
            break;
        case CONSISTENT:
            lm_status_manager = make_shared<StatusManagerMultiPathConsistent>(
                *lm_graph, add_goal_atoms, add_gn_parents,
                add_reasonable_children);
        }
        lm_status_manager->set_landmarks_for_initial_state(state);
        lm_status_manager->update_lm_status(state);
        dalm_graph = make_shared<DisjActionLandmarkGraph>(
            *lm_graph, *lm_status_manager, state);
    } else {
        dalm_graph = make_shared<DisjActionLandmarkGraph>(
            *lm_graph, state);
        lm_graph.reset(); // not needed anymore
    }

    num_landmarks = dalm_graph->get_number_of_landmarks();
    num_orderings = dalm_graph->get_number_of_orderings();
    num_weak_orderings = dalm_graph->get_number_of_weak_orderings();
    num_lm_graphs = 1;

    utils::g_log << "Landmark graph contains " << num_landmarks
                 << " landmarks." << endl;
    utils::g_log << "Landmark graph contains " << num_orderings
                 << " orderings of which "
                 << dalm_graph->get_number_of_strong_orderings()
                 << " are strong and " << num_weak_orderings
                 << " are weak." << endl;
}

void LandmarkConstraintHandler::recompute_lm_graph(const State &state) {
    if (path_dependent) {
        dalm_graph = make_shared<DisjActionLandmarkGraph>(
            *lm_graph, *lm_status_manager, state);
    } else {
        dalm_graph = make_shared<DisjActionLandmarkGraph>(
            *lm->recompute_lm_graph(state), state);
    }

    num_landmarks += dalm_graph->get_number_of_landmarks();
    num_orderings += dalm_graph->get_number_of_orderings();
    num_weak_orderings += dalm_graph->get_number_of_weak_orderings();
    ++num_lm_graphs;
}

void LandmarkConstraintHandler::print_statistics() const {
    cout << "Number of computed LM Graphs: " << num_lm_graphs << endl;
    cout << "Average number of landmarks: "
         << (static_cast<double>(num_landmarks) / num_lm_graphs) << endl;
    cout << "Average number of orderings: "
         << (static_cast<double>(num_orderings) / num_lm_graphs) << endl;
    cout << "Average number of weak orderings: "
         << (static_cast<double>(num_weak_orderings) / num_lm_graphs) << endl;
    cout << "Average number of strong orderings: "
         << (static_cast<double>(
            num_orderings - num_weak_orderings) / num_lm_graphs) << endl;

    cout << "Average number of cycle constraints: "
         << (static_cast<double>(num_cycles) / num_lm_graphs) << endl;
}

bool LandmarkConstraintHandler::initial_fact_landmark_graph_is_acyclic() {
    assert(lm_graph);

    /*
      Kahn's algorithm: try to sort topologically which is only possible
      for acyclic graphs.
    */
    int n = lm_graph->get_num_landmarks();
    vector<bool> sorted(n, false);
    queue<int> open{};
    for (int id = 0; id < n; ++id) {
        if (lm_graph->get_node(id)->parents.empty()) {
            open.push(id);
        }
    }

    while (!open.empty()) {
        int id = open.front();
        open.pop();
        sorted[id] = true;

        for (auto succ : lm_graph->get_node(id)->children) {
            int succ_id = succ.first->get_id();
            unordered_map<LandmarkNode*, EdgeType> &predecessors =
                lm_graph->get_node(succ_id)->parents;
            if (all_of(predecessors.begin(), predecessors.end(),
                       [sorted](pair<LandmarkNode*, EdgeType> pred) {
                return sorted[pred.first->get_id()];
            })) {
                open.push(succ_id);
            }
        }
    }

    for (int id = 0; id < n; ++id) {
        if (!sorted[id]) {
            return false;
        }
    }
    return true;
}
}
