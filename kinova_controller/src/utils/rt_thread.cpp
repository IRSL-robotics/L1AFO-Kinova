#include "utils/rt_thread.hpp"

namespace utils
{
int set_thread_priority(pid_t pid, size_t sched_priority, int policy)
{
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = sched_priority;
  return sched_setscheduler(pid, policy, &param);
}

int set_this_thread_priority(size_t sched_priority, int policy)
{
  return set_thread_priority(getpid(), sched_priority, policy);
}

int set_thread_cpu_affinity(pid_t pid, uint32_t cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0U;
  CPU_ZERO(&set);
  while (cpu_bit_mask > 0U) {
    if ((cpu_bit_mask & 0x1U) > 0) {
      CPU_SET(cpu_cnt, &set);
    }
    cpu_bit_mask = (cpu_bit_mask >> 1U);
    cpu_cnt++;
  }
  return sched_setaffinity(pid, sizeof(set), &set);
}

int set_this_thread_cpu_affinity(uint32_t cpu_bit_mask)
{
  return set_thread_cpu_affinity(getpid(), cpu_bit_mask);
}
}  // namespace utils