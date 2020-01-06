#ifndef _DRAMSIM_WRAPPER_H_
#define _DRAMSIM_WRAPPER_H_

#include <DRAMSim.h>

#include <cstdint>

using namespace DRAMSim;

typedef uint64_t target_ulong;

class dramsim_wrapper
{
  public:
    dramsim_wrapper();
    dramsim_wrapper(const char *dram_ini_file,
                                     const char *system_ini_file,
                                     const char *stats_dir,
                                     const char *app_name, int size_mb);
    ~dramsim_wrapper();
    bool can_add_transaction(target_ulong addr);
    bool add_transaction(target_ulong addr, bool isWrite);
    void update();
    void print_stats();
    MultiChannelMemorySystem *dramsim;
    TransactionCompleteCB *read_cb;
    TransactionCompleteCB *write_cb;

    /* Callback functions */
    void read_complete(unsigned, uint64_t, uint64_t);
    void write_complete(unsigned, uint64_t, uint64_t);

    /* stage queues will come here */
};
#endif