#ifndef FAST_DOWNWARD_MODIFIED_INITIAL_STATE_TASK_H
#define FAST_DOWNWARD_MODIFIED_INITIAL_STATE_TASK_H

#include "delegating_task.h"

#include <vector>

namespace extra_tasks {
class ModifiedInitialStateTask : public tasks::DelegatingTask {
    const std::vector<int> init_state_values;

public:
    ModifiedInitialStateTask(
        const std::shared_ptr<AbstractTask> &parent,
        std::vector<int> &&init_state_values);
    ~ModifiedInitialStateTask() = default;

    virtual std::vector<int> get_initial_state_values() const override;
};
}

#endif
