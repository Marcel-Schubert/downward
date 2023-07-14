#include "landmark_factory_reasonable_orders_from_file.h"

#include "landmark.h"

#include "util.h"

#include "../plugins/plugin.h"
#include "../utils/logging.h"

#include <fstream>


using namespace std;
namespace landmarks {
LandmarkFactoryReasonableOrdersFromFile::LandmarkFactoryReasonableOrdersFromFile(const plugins::Options &opts)
    : LandmarkFactory(opts),
      lm_factory(opts.get<shared_ptr<LandmarkFactory>>("lm_factory")) {
}

void LandmarkFactoryReasonableOrdersFromFile::generate_landmarks(const shared_ptr<AbstractTask> &task) {
    if (log.is_at_least_normal()) {
        log << "Building a landmark graph with reasonable orders." << endl;
    }

    lm_graph = lm_factory->compute_lm_graph(task);

    TaskProxy task_proxy(*task);
    if (log.is_at_least_normal()) {
        log << "reading reasonable orders" << endl;
    }


    string line;
    fstream file("ros.txt");
    string delim = " -r-> ";
    if (file.is_open()) {
        while (getline(file, line)) {
            int pos = line.find(delim);
            int a = stoi(line.substr(0, pos));
            int b = stoi(line.substr(pos + delim.length()));
            edge_add_force(*lm_graph->get_node(a), *lm_graph->get_node(b), EdgeType::REASONABLE);
        }
    }
}

bool LandmarkFactoryReasonableOrdersFromFile::computes_reasonable_orders() const {
    return true;
}

bool LandmarkFactoryReasonableOrdersFromFile::supports_conditional_effects() const {
    return lm_factory->supports_conditional_effects();
}

class LandmarkFactoryReasonableOrdersFromFileFeature : public plugins::TypedFeature<LandmarkFactory, LandmarkFactoryReasonableOrdersFromFile> {
public:
    LandmarkFactoryReasonableOrdersFromFileFeature() : TypedFeature("lm_reasonable_orders_file") {
        document_title("Reasonable Orders from file");
        document_synopsis(
            "Adds reasonable orders from a file");

        add_option<shared_ptr<LandmarkFactory>>("lm_factory");
        add_landmark_factory_options_to_feature(*this);

        // TODO: correct?
        document_language_support(
            "conditional_effects",
            "supported if subcomponent supports them");
    }
};

static plugins::FeaturePlugin<LandmarkFactoryReasonableOrdersFromFileFeature> _plugin;
}
