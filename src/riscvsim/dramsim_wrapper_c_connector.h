#ifndef _DRAMSIM_WRAPPER_C_CONNECTOR_H_
#define _DRAMSIM_WRAPPER_C_CONNECTOR_H_

#include <inttypes.h>
typedef uint64_t target_ulong;

#ifdef __cplusplus
extern "C" {
#endif

void dramsim_wrapper_init(const char *dram_ini_file,
                          const char *system_ini_file, const char *stats_dir,
                          const char *app_name, int size_mb);
void dramsim_wrapper_destroy();
int dramsim_wrapper_can_add_transaction(target_ulong addr);
int dramsim_wrapper_add_transaction(target_ulong addr, int isWrite);
void dramsim_wrapper_update();
void dramsim_wrapper_print_stats();

#ifdef __cplusplus
}
#endif

#endif // End include guard