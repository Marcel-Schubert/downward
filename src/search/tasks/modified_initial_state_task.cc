#include "modified_initial_state_task.h"

using namespace std;

namespace extra_tasks {
ModifiedInitialStateTask::ModifiedInitialStateTask(
    const shared_ptr<AbstractTask> &parent,
    vector<int> &&init_state_values)
    : DelegatingTask(parent),
      init_state_values(move(init_state_values)) {
}

vector<int> ModifiedInitialStateTask::get_initial_state_values() const {
    return init_state_values;
}
}
