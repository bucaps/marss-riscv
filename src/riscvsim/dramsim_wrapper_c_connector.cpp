#include <cstdlib>

#include "dramsim_wrapper.h"
#include "dramsim_wrapper_c_connector.h"

#ifdef __cplusplus
extern "C" {
#endif

static dramsim_wrapper *dramsim_wrapper_obj = NULL;

void
dramsim_wrapper_init(const char *dram_ini_file, const char *system_ini_file,
                     const char *stats_dir, const char *app_name, int size_mb)
{
    if (dramsim_wrapper_obj == NULL)
    {
        dramsim_wrapper_obj = new dramsim_wrapper(
            dram_ini_file, system_ini_file, stats_dir, app_name, size_mb);
    }
}

void
dramsim_wrapper_destroy()
{
    delete dramsim_wrapper_obj;
}

int
dramsim_wrapper_can_add_transaction(target_ulong addr)
{
    return dramsim_wrapper_obj->can_add_transaction(addr);
}

int
dramsim_wrapper_add_transaction(target_ulong addr, int isWrite)
{
    return dramsim_wrapper_obj->add_transaction(addr, (bool)isWrite);
}

void
dramsim_wrapper_update()
{
    dramsim_wrapper_obj->update();
}

void
dramsim_wrapper_print_stats()
{
    dramsim_wrapper_obj->print_stats();
}

#ifdef __cplusplus
}
#endif