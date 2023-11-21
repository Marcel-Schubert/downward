#include "landmark_factory.h"

#include "landmark.h"
#include "landmark_graph.h"
#include "util.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"

#include "../tasks/modified_initial_state_task.h"
#include "../utils/logging.h"
#include "../utils/memory.h"
#include "../utils/timer.h"

#include <fstream>
#include <limits>

using namespace std;

namespace landmarks {
/*
  TODO: Update this comment

  Note: To allow reusing landmark graphs, we use the following temporary
  solution.

  Landmark factories cache the first landmark graph they compute, so
  each call to this function returns the same graph. Asking for landmark graphs
  of different tasks is an error and will exit with SEARCH_UNSUPPORTED.

  If you want to compute different landmark graphs for different
  Exploration objects, you have to use separate landmark factories.

  This solution remains temporary as long as the question of when and
  how to reuse landmark graphs is open.

  As all heuristics will work on task transformations in the future,
  this function will also get access to a TaskProxy. Then we need to
  ensure that the TaskProxy used by the Exploration object is the same
  as the TaskProxy object passed to this function.
*/
shared_ptr<LandmarkGraph> LandmarkFactory::compute_lm_graph(
    const shared_ptr<AbstractTask> &task) {
    if (lm_graph) {
        if (lm_graph_task != task) {
            cerr << "LandmarkFactory was asked to compute landmark graphs for "
                 << "two different tasks. This is currently not supported."
                 << endl;
            utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
        }
        return lm_graph;
    }
    lm_graph_task = task;
    utils::Timer lm_generation_timer;

    lm_graph = make_shared<LandmarkGraph>();

    TaskProxy task_proxy(*task);
    generate_operators_lookups(task_proxy);
    generate_landmarks(task);

    utils::g_log << "Landmarks generation time: " << lm_generation_timer << endl;
    if (lm_graph->get_num_landmarks() == 0)
        utils::g_log << "Warning! No landmarks found. Task unsolvable?" << endl;
    else {
        utils::g_log << "Discovered " << lm_graph->get_num_landmarks()
                     << " landmarks, of which " << lm_graph->get_num_disjunctive_landmarks()
                     << " are disjunctive and "
                     << lm_graph->get_num_conjunctive_landmarks() << " are conjunctive." << endl;
        utils::g_log << lm_graph->get_num_edges() << " edges" << endl;
    }
    return lm_graph;
}

shared_ptr<LandmarkGraph> LandmarkFactory::recompute_lm_graph(
    const State &state) {
    shared_ptr<AbstractTask> task =
        make_shared<extra_tasks::ModifiedInitialStateTask>(
            lm_graph_task, vector<int>{state.get_unpacked_values()});
    lm_graph = make_shared<LandmarkGraph>();

    generate_landmarks(task);

    return lm_graph;
}

bool LandmarkFactory::is_landmark_precondition(
    const OperatorProxy &op, const Landmark &landmark) const {
    /* Test whether the landmark is used by the operator as a precondition.
    A disjunctive landmarks is used if one of its disjuncts is used. */
    for (FactProxy pre : op.get_preconditions()) {
        for (const FactPair &lm_fact : landmark.facts) {
            if (pre.get_pair() == lm_fact)
                return true;
        }
    }
    return false;
}

void LandmarkFactory::edge_add(LandmarkNode &from, LandmarkNode &to,
                               EdgeType type) {
    /* Adds an edge in the landmarks graph if there is no contradicting edge (simple measure to
    reduce cycles. If the edge is already present, the stronger edge type wins.
    */
    assert(&from != &to);
    assert(from.parents.find(&to) == from.parents.end() || type <= EdgeType::REASONABLE);
    assert(to.children.find(&from) == to.children.end() || type <= EdgeType::REASONABLE);

    // If edge already exists, remove if weaker
    if (from.children.find(&to) != from.children.end() && from.children.find(
            &to)->second < type) {
        from.children.erase(&to);
        assert(to.parents.find(&from) != to.parents.end());
        to.parents.erase(&from);

        assert(to.parents.find(&from) == to.parents.end());
        assert(from.children.find(&to) == from.children.end());
    }
    // If edge does not exist (or has just been removed), insert
    if (from.children.find(&to) == from.children.end()) {
        assert(to.parents.find(&from) == to.parents.end());
        from.children.emplace(&to, type);
        to.parents.emplace(&from, type);
        //utils::g_log << "added parent with address " << &from << endl;
    }
    assert(from.children.find(&to) != from.children.end());
    assert(to.parents.find(&from) != to.parents.end());
}

void LandmarkFactory::discard_all_orderings() {
    //utils::g_log << "Removing all orderings." << endl;
    for (auto &node : lm_graph->get_nodes()) {
        node->children.clear();
        node->parents.clear();
    }
}

void LandmarkFactory::generate_operators_lookups(const TaskProxy &task_proxy) {
    /* Build datastructures for efficient landmark computation. Map propositions
    to the operators that achieve them or have them as preconditions */

    operators_eff_lookup.clear();
    VariablesProxy variables = task_proxy.get_variables();
    operators_eff_lookup.resize(variables.size());
    for (VariableProxy var : variables) {
        operators_eff_lookup[var.get_id()].resize(var.get_domain_size());
    }
    OperatorsProxy operators = task_proxy.get_operators();
    for (OperatorProxy op : operators) {
        const EffectsProxy effects = op.get_effects();
        for (EffectProxy effect : effects) {
            const FactProxy effect_fact = effect.get_fact();
            operators_eff_lookup[effect_fact.get_variable().get_id()][effect_fact.get_value()].push_back(
                get_operator_or_axiom_id(op));
        }
    }
    for (OperatorProxy axiom : task_proxy.get_axioms()) {
        const EffectsProxy effects = axiom.get_effects();
        for (EffectProxy effect : effects) {
            const FactProxy effect_fact = effect.get_fact();
            operators_eff_lookup[effect_fact.get_variable().get_id()][effect_fact.get_value()].push_back(
                get_operator_or_axiom_id(axiom));
        }
    }
}

void _add_use_orders_option_to_parser(OptionParser &parser) {
    parser.add_option<bool>("use_orders",
                            "use orders between landmarks",
                            "true");
}

void _add_only_causal_landmarks_option_to_parser(OptionParser &parser) {
    parser.add_option<bool>("only_causal_landmarks",
                            "keep only causal landmarks",
                            "false");
}


static PluginTypePlugin<LandmarkFactory> _type_plugin(
    "LandmarkFactory",
    "A landmark factory specification is either a newly created "
    "instance or a landmark factory that has been defined previously. "
    "This page describes how one can specify a new landmark factory instance. "
    "For re-using landmark factories, see OptionSyntax#Landmark_Predefinitions.",
    "landmarks");
}
