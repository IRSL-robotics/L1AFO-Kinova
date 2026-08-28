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

#include <iostream>
#include <cerrno>
#include <cstring>
#include <memory>


namespace utils
{
/// \brief Lock currently paged memory using mlockall.
/// \return Error code to propagate to main
int lock_memory();

/// \brief Commit a pool of dynamic memory based on the memory already cached
/// by this process by checking the number of pagefaults.
/// \return Error code to propagate to main
int lock_and_prefault_dynamic();

/// \brief Commit a pool of dynamic memory based on a prefixed size
/// \return Error code to propagate to main
int lock_and_prefault_dynamic(size_t process_max_dynamic_memory);
}  // namespace utils

