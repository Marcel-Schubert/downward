#ifndef LANDMARKS_LANDMARK_FACTORY_REASONABLE_ORDERS_FROM_FILE_H
#define LANDMARKS_LANDMARK_FACTORY_REASONABLE_ORDERS_FROM_FILE_H

#include "landmark_factory.h"

namespace landmarks {
class LandmarkFactoryReasonableOrdersFromFile : public LandmarkFactory {
    std::shared_ptr<LandmarkFactory> lm_factory;

    virtual void generate_landmarks(const std::shared_ptr<AbstractTask> &task) override;

public:
    LandmarkFactoryReasonableOrdersFromFile(const plugins::Options &opts);

    virtual bool computes_reasonable_orders() const override;
    virtual bool supports_conditional_effects() const override;
};
}

#endif
