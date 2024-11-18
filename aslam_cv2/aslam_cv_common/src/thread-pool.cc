#include <algorithm>

#include <aslam/common/thread-pool.h>
#include <pthread.h>  // thread priority
#include <sched.h>  // thread priority
#include <iostream>

namespace {
void printThreadPriority(const std::string& thread_name_for_print) {
  int policy;
  struct sched_param param;
  int result = pthread_getschedparam(pthread_self(), &policy, &param);
  if (result != 0) {
    std::cerr << "Error getting sched param: " << strerror(result) << std::endl;
    return;
  }
  std::cout << "ThreadPriority: get priority for " << thread_name_for_print
            << " = " << param.sched_priority << " (policy = " << policy
            << ", min = " << sched_get_priority_min(policy)
            << ", max = " << sched_get_priority_max(policy)
            << "), thread_id = " << pthread_self() << std::endl;
}

void setCurrentThreadRealtime(
    const std::string& thread_name_for_print, int sched_priority = 1,
    int policy = SCHED_RR) {
  printThreadPriority(thread_name_for_print + " (before set priority)");
  struct sched_param param;
  param.sched_priority = sched_priority;
  int result = pthread_setschedparam(pthread_self(), policy, &param);
  if (result != 0) {
    std::cerr << "ThreadPriority: Fail to set priority for "
              << thread_name_for_print << " = " << param.sched_priority
              << " (min = " << sched_get_priority_min(policy)
              << ", max = " << sched_get_priority_max(policy)
              << "), thread_id = " << pthread_self()
              << ", error info: " << strerror(result) << std::endl;
    return;
  }
  std::cout << "ThreadPriority: set priority for " << thread_name_for_print
            << " = " << param.sched_priority
            << " (min = " << sched_get_priority_min(policy)
            << ", max = " << sched_get_priority_max(policy)
            << "), thread_id = " << pthread_self() << std::endl;
  printThreadPriority(thread_name_for_print + " (after set priority)");
}
}  // namespace

namespace aslam {

// The constructor just launches some amount of workers.
ThreadPool::ThreadPool(const size_t threads, const std::string& thread_name, bool realtime)
    : active_threads_(0),
      thread_name_(thread_name),
      stop_(false) {
  for (size_t i = 0; i < threads; ++i)
    workers_.emplace_back(std::bind(&ThreadPool::run, this, realtime));
}

// The destructor joins all threads.
ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(tasks_mutex_);
    stop_ = true;
  }
  tasks_queue_change_.notify_all();
  for (size_t i = 0u; i < workers_.size(); ++i) {
    workers_[i].join();
  }
}

void ThreadPool::run(bool realtime) {
  if (!thread_name_.empty()) {
    pthread_setname_np(pthread_self(), thread_name_.substr(0, 15).c_str());
  }

  if (realtime) {
    setCurrentThreadRealtime(thread_name_);
  }

  while (true) {
    std::unique_lock<std::mutex> lock(this->tasks_mutex_);

    // Here we need to select the next task from a queue that is not already
    // serviced.
    std::function<void()> task;
    size_t group_id = kGroupdIdNonExclusiveTask;
    while (true) {
      const bool all_guards_active = std::all_of(
          groupid_exclusivity_guards_.begin(),
          groupid_exclusivity_guards_.end(),
          [](const GuardMap::value_type& value) {
            CHECK_NE(value.second, kGroupdIdNonExclusiveTask)
              << "There should never be a guard for a non-exclusive task.";
            return value.second;
          });

      if (!all_guards_active || num_queued_nonexclusive_tasks > 0u) {
        // If not all guards are active, we select a task to process; otherwise
        // we can go back to sleep until a thread reports back for work.
        size_t index = 0u;
        for (const TaskDeque::value_type& groupid_task : groupid_tasks_) {
          // We have found a task to process if no thread is already working on
          // this group id.
          const bool is_exclusive_task =
              groupid_task.first != kGroupdIdNonExclusiveTask;
          bool guard_active = false;
          if (is_exclusive_task) {
            guard_active = groupid_exclusivity_guards_[groupid_task.first];
          }

          if (!(is_exclusive_task && guard_active)) {
            group_id = groupid_task.first;
            task = groupid_task.second;

            groupid_tasks_.erase(groupid_tasks_.begin() + index);
            if (!is_exclusive_task) {
              --num_queued_nonexclusive_tasks;
            }

            // We jump out of the nested for-structure here, because we have
            // found a task to process.
            break;
          }
          ++index;
        }
      }
      if (task) {
        break;
      }

      // Wait until the queue has changed (addition/removal) before re-checking
      // for new tasks to process.
      if (this->stop_ && groupid_tasks_.size() == 0u) {
        return;
      }

      this->tasks_queue_change_.wait(lock);
    }

    // We jump here if we found a task.
    CHECK(task);
    ++active_threads_;

    // Make sure the no other thread is currently working on this exclusivity
    // group.
    if (group_id != kGroupdIdNonExclusiveTask) {
      const GuardMap::iterator it_group_id_serviced =
          groupid_exclusivity_guards_.find(group_id);
      CHECK(it_group_id_serviced == groupid_exclusivity_guards_.end() ||
            it_group_id_serviced->second == false);
      it_group_id_serviced->second = true;
    }

    // Unlock the queue while we execute the task.
    lock.unlock();
    task();
    lock.lock();

    // Release the group for other threads.
    if (group_id != kGroupdIdNonExclusiveTask) {
      const GuardMap::iterator it_group_id_servied =
          groupid_exclusivity_guards_.find(group_id);
      CHECK(it_group_id_servied != groupid_exclusivity_guards_.end() &&
            it_group_id_servied->second == true);
      it_group_id_servied->second = false;
    }

    --active_threads_;

    // This is the secret to making the waitForEmptyQueue() function work.
    // After finishing a task, notify that this work is done.
    tasks_queue_change_.notify_all();
  }
}

size_t ThreadPool::numActiveThreads() const {
  std::unique_lock<std::mutex> lock(this->tasks_mutex_);
  return active_threads_;
}

void ThreadPool::waitForEmptyQueue() const {
  std::unique_lock<std::mutex> lock(this->tasks_mutex_);
  // Only exit if all tasks are complete by tracking the number of
  // active threads.
  while (active_threads_ > 0u || groupid_tasks_.size() > 0u) {
    this->tasks_queue_change_.wait(lock);
  }
}
}  // namespace aslam
