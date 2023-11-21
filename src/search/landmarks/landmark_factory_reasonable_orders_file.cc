#include "landmark_factory_reasonable_orders_file.h"

#include "landmark.h"
#include "landmark_graph.h"

#include "util.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"
#include "../utils/markup.h"

#include <fstream>

using namespace std;
namespace landmarks {
LandmarkFactoryReasonableOrdersFile::LandmarkFactoryReasonableOrdersFile(
    const Options &opts)
    : lm_factory(opts.get<shared_ptr<LandmarkFactory>>("lm_factory")) {
}

void LandmarkFactoryReasonableOrdersFile::generate_landmarks(
    const shared_ptr<AbstractTask> &task) {
    //utils::g_log << "Building a landmark graph with reasonable orders." << endl;

    TaskProxy task_proxy(*task);
    // FIXME: this is a bit hacky with the nested *generate_landmarks*.
    if (recomp) {
        lm_graph = lm_factory->recompute_lm_graph(
            task_proxy.get_initial_state());
    } else {
        lm_graph = lm_factory->compute_lm_graph(task);
        recomp = true;
    }

    //utils::g_log << "approx. reasonable orders" << endl;
    std::string line;
    std::fstream file("ros.txt");
    std::string delim = " -r-> ";
    if (file.is_open()) {
        while (getline(file, line)) {
            int pos = line.find(delim);
            int a = stoi(line.substr(0, pos));
            int b = stoi(line.substr(pos + delim.length()));
            edge_add(*lm_graph->get_nodes().at(a), *lm_graph->get_nodes().at(b), EdgeType::REASONABLE);
        }
    } else {
        utils::g_log << "File ros.txt not found!" << endl;
        exit(EXIT_FAILURE);
    }
}

bool LandmarkFactoryReasonableOrdersFile::computes_reasonable_orders() const {
    return true;
}

bool LandmarkFactoryReasonableOrdersFile::supports_conditional_effects() const {
    return lm_factory->supports_conditional_effects();
}

static shared_ptr<LandmarkFactory> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Reasonable Orders from File",
        "Adds reasonable orders and obedient reasonable orders "
        "from file");
    parser.add_option<shared_ptr<LandmarkFactory>>("lm_factory");
    Options opts = parser.parse();

    // TODO: correct?
    parser.document_language_support("conditional_effects",
                                     "supported if subcomponent supports them");

    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<LandmarkFactoryReasonableOrdersFile>(opts);
}

static Plugin<LandmarkFactory> _plugin("lm_reasonable_orders_file", _parse);
}
