#ifndef _DRAMSIM_WRAPPER_C_CONNECTOR_H_
#define _DRAMSIM_WRAPPER_C_CONNECTOR_H_

#include "memory_controller_utils.h"
#include "riscv_sim_macros.h"
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

void dramsim_wrapper_init(const char *dram_ini_file,
                          const char *system_ini_file, const char *stats_dir,
                          const char *app_name, int size_mb,
                          StageMemAccessQueue *frontend_mem_access_queue,
                          StageMemAccessQueue *backend_mem_access_queue);
void dramsim_wrapper_destroy();
int dramsim_wrapper_can_add_transaction(target_ulong addr);
int dramsim_wrapper_add_transaction(target_ulong addr, int isWrite);
void dramsim_wrapper_update();
void dramsim_wrapper_print_stats();

#ifdef __cplusplus
}
#endif

#endif // End include guard