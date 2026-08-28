#pragma once

#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>

#include <stdlib.h>
#include <limits.h>
#include <malloc.h>
#include <sys/resource.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>

#include <iostream>

namespace utils
{
int set_thread_priority(pid_t pid, size_t sched_priority, int policy);
int set_this_thread_priority(size_t sched_priority, int policy);
int set_thread_cpu_affinity(pid_t pid, uint32_t cpu_bit_mask);
int set_this_thread_cpu_affinity(uint32_t cpu_bit_mask);
}  // namespace utils
