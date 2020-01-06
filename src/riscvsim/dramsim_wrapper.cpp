#include "dramsim_wrapper.h"

#include <cstdio>
#include <string>

dramsim_wrapper::dramsim_wrapper(const char *dram_ini_file,
                                 const char *system_ini_file,
                                 const char *stats_dir, const char *app_name,
                                 int size_mb)
{
    read_cb = new Callback<dramsim_wrapper, void, unsigned, uint64_t, uint64_t>(
        this, &dramsim_wrapper::read_complete);

    write_cb
        = new Callback<dramsim_wrapper, void, unsigned, uint64_t, uint64_t>(
            this, &dramsim_wrapper::write_complete);

    dramsim = getMemorySystemInstance(
        std::string(dram_ini_file), std::string(system_ini_file),
        std::string(stats_dir), std::string(app_name), size_mb);

    dramsim->RegisterCallbacks(read_cb, write_cb, NULL);

    dramsim->setCPUClockSpeed(0);
}

dramsim_wrapper::~dramsim_wrapper()
{
    delete dramsim;
}

void
dramsim_wrapper::read_complete(unsigned id, uint64_t address,
                               uint64_t clock_cycle)
{
    printf("[Callback] read complete: %d 0x%lx cycle=%lu\n", id, address,
           clock_cycle);
}

void
dramsim_wrapper::write_complete(unsigned id, uint64_t address,
                                uint64_t clock_cycle)
{
    printf("[Callback] write complete: %d 0x%lx cycle=%lu\n", id, address,
           clock_cycle);
}

bool
dramsim_wrapper::can_add_transaction(target_ulong addr)
{
    return dramsim->willAcceptTransaction(addr);
}

bool
dramsim_wrapper::add_transaction(target_ulong addr, bool isWrite)
{
    return dramsim->addTransaction(isWrite, addr);
}

void
dramsim_wrapper::update()
{
    dramsim->update();
}

void
dramsim_wrapper::print_stats()
{
    dramsim->printStats(true);
}