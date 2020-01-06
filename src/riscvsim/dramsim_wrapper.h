#ifndef _DRAMSIM_WRAPPER_H_
#define _DRAMSIM_WRAPPER_H_

#include "riscv_sim_macros.h"
#include "memory_controller_utils.h"
#include <DRAMSim.h>

#include <cstdint>

using namespace DRAMSim;

class dramsim_wrapper
{
  public:
    dramsim_wrapper();
    dramsim_wrapper(const char *dram_ini_file, const char *system_ini_file,
                    const char *stats_dir, const char *app_name, int size_mb,
                    StageMemAccessQueue *frontend_mem_access_queue,
                    StageMemAccessQueue *backend_mem_access_queue);
    ~dramsim_wrapper();
    bool can_add_transaction(target_ulong addr);
    bool add_transaction(target_ulong addr, bool isWrite);
    void update();
    void print_stats();
    MultiChannelMemorySystem *dramsim;
    TransactionCompleteCB *read_cb;
    TransactionCompleteCB *write_cb;
    StageMemAccessQueue *frontend_mem_access_queue;
    StageMemAccessQueue *backend_mem_access_queue;

    /* Callback functions */
    void read_complete(unsigned, uint64_t, uint64_t);
    void write_complete(unsigned, uint64_t, uint64_t);

    /* stage queues will come here */
};
#endif