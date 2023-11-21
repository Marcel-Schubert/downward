#ifndef LANDMARKS_LANDMARK_FACTORY_REASONABLE_ORDERS_FILE_H
#define LANDMARKS_LANDMARK_FACTORY_REASONABLE_ORDERS_FILE_H

#include "landmark_factory.h"

namespace landmarks {
class LandmarkFactoryReasonableOrdersFile : public LandmarkFactory {
    std::shared_ptr<LandmarkFactory> lm_factory;
    bool recomp = false;

    virtual void generate_landmarks(const std::shared_ptr<AbstractTask> &task) override;


public:
    LandmarkFactoryReasonableOrdersFile(const options::Options &opts);

    virtual bool computes_reasonable_orders() const override;
    virtual bool supports_conditional_effects() const override;
};
}

#endif
