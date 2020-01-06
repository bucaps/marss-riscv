#ifndef _MEM_CONTROLLER_UTILS_H_
#define _MEM_CONTROLLER_UTILS_H_

/* Memory operation type */
typedef enum MemAccessType
{
    Read = 0x0,
    Write = 0x1,
} MemAccessType;

typedef struct PendingMemAccessEntry
{
    int valid;
    int bytes_to_access;
    target_ulong addr;
    MemAccessType type;
} PendingMemAccessEntry;

typedef struct StageMemAccessQueue
{
    int cur_idx;
    int max_size;
    int cur_size;
    PendingMemAccessEntry *entry;
} StageMemAccessQueue;
#endif