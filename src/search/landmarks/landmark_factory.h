#ifndef LANDMARKS_LANDMARK_FACTORY_H
#define LANDMARKS_LANDMARK_FACTORY_H

#include "landmark_graph.h"

#include <list>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class TaskProxy;

namespace options {
class OptionParser;
class Options;
}

namespace landmarks {
/*
  TODO: Change order to private -> protected -> public
   (omitted so far to minimize diff)
*/
class LandmarkFactory {
public:
    virtual ~LandmarkFactory() = default;
    LandmarkFactory(const LandmarkFactory &) = delete;

    std::shared_ptr<LandmarkGraph> compute_lm_graph(
        const std::shared_ptr<AbstractTask> &task);
    std::shared_ptr<LandmarkGraph> recompute_lm_graph(const State &state);

    /*
      TODO: Currently reasonable orders are not supported for admissible landmark count
      heuristics, which is why the heuristic needs to know whether the factory computes
      reasonable orders. Once issue383 is dealt with we should be able to use reasonable
      orders for admissible heuristics and this method can be removed.
    */
    virtual bool computes_reasonable_orders() const = 0;
    virtual bool supports_conditional_effects() const = 0;

protected:
    LandmarkFactory() = default;
    std::shared_ptr<LandmarkGraph> lm_graph;

    void edge_add(LandmarkNode &from, LandmarkNode &to, EdgeType type);

    void discard_all_orderings();

    bool is_landmark_precondition(const OperatorProxy &op,
                                  const Landmark &landmark) const;

    const std::vector<int> &get_operators_including_eff(const FactPair &eff) const {
        return operators_eff_lookup[eff.var][eff.value];
    }

private:
    std::shared_ptr<AbstractTask> lm_graph_task;

    virtual void generate_landmarks(const std::shared_ptr<AbstractTask> &task) = 0;

    std::vector<std::vector<std::vector<int>>> operators_eff_lookup;

    void generate_operators_lookups(const TaskProxy &task_proxy);
};

extern void _add_use_orders_option_to_parser(options::OptionParser &parser);
extern void _add_only_causal_landmarks_option_to_parser(options::OptionParser &parser);
}

#endif
