#include "dramsim_wrapper.h"

#include <cstdio>
#include <string>

dramsim_wrapper::dramsim_wrapper(const char *dram_ini_file,
                                 const char *system_ini_file,
                                 const char *stats_dir, const char *app_name,
                                 int size_mb,
                                 StageMemAccessQueue *frontend_queue,
                                 StageMemAccessQueue *backend_queue)
{
    read_cb = new Callback<dramsim_wrapper, void, unsigned, uint64_t, uint64_t>(
        this, &dramsim_wrapper::read_complete);

    write_cb
        = new Callback<dramsim_wrapper, void, unsigned, uint64_t, uint64_t>(
            this, &dramsim_wrapper::write_complete);

    frontend_mem_access_queue = frontend_queue;
    backend_mem_access_queue = backend_queue;

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
dramsim_wrapper::read_complete(unsigned id, uint64_t addr,
                               uint64_t clock_cycle)
{
    int i;

    for (i = 0; i < frontend_mem_access_queue->cur_idx; ++i)
    {
        if ((frontend_mem_access_queue->entry[i].valid)
            && (frontend_mem_access_queue->entry[i].addr == addr)
            && (frontend_mem_access_queue->entry[i].type == Read))
        {
            frontend_mem_access_queue->entry[i].valid = 0;
            --frontend_mem_access_queue->cur_size;
            return;
        }
    }

    for (i = 0; i < backend_mem_access_queue->cur_idx; ++i)
    {
        if ((backend_mem_access_queue->entry[i].valid)
            && (backend_mem_access_queue->entry[i].addr == addr)
            && (backend_mem_access_queue->entry[i].type == Read))
        {
            backend_mem_access_queue->entry[i].valid = 0;
            --backend_mem_access_queue->cur_size;
            return;
        }
    }
}

void
dramsim_wrapper::write_complete(unsigned id, uint64_t addr,
                                uint64_t clock_cycle)
{
    int i;

    for (i = 0; i < frontend_mem_access_queue->cur_idx; ++i)
    {
        if ((frontend_mem_access_queue->entry[i].valid)
            && (frontend_mem_access_queue->entry[i].addr == addr)
            && (frontend_mem_access_queue->entry[i].type == Write))
        {
            frontend_mem_access_queue->entry[i].valid = 0;
            --frontend_mem_access_queue->cur_size;
            return;
        }
    }

    for (i = 0; i < backend_mem_access_queue->cur_idx; ++i)
    {
        if ((backend_mem_access_queue->entry[i].valid)
            && (backend_mem_access_queue->entry[i].addr == addr)
            && (backend_mem_access_queue->entry[i].type == Write))
        {
            backend_mem_access_queue->entry[i].valid = 0;
            --backend_mem_access_queue->cur_size;
            return;
        }
    }
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