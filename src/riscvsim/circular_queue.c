/**
 * Generic Circular Queue
 *
 * MARSS-RISCV : Micro-Architectural System Simulator for RISC-V
 *
 * Copyright (c) 2017-2019 Gaurav Kothari {gkothar1@binghamton.edu}
 * State University of New York at Binghamton
 *
 * Copyright (c) 2018-2019 Parikshit Sarnaik {psarnai1@binghamton.edu}
 * State University of New York at Binghamton
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "circular_queue.h"

void
cq_init(CQ *p, int max_size_val)
{
    p->front = -1;
    p->rear = -1;
    p->max_size = max_size_val;
}

int
cq_enqueue(CQ *p)
{
    if (cq_full(p))
    {
        /* Queue full */
        return -1;
    }
    else if (cq_empty(p))
    {
        /* Empty Queue, insert first element */
        p->front = 0;
        p->rear = 0;
        return p->front;
    }
    else if ((p->rear == (p->max_size - 1)) && (p->front != 0))
    {
        /* Wrap around */
        p->rear = 0;
        return p->rear;
    }
    else
    {
        (p->rear)++;
        return p->rear;
    }
}

int
cq_dequeue(CQ *p)
{
    int old_front;

    if (cq_empty(p))
    {
        /* Empty Queue */
        return -1;
    }

    old_front = p->front;

    if (p->front == p->rear)
    {
        /* Remove the only element in the queue */
        p->front = -1;
        p->rear = -1;
    }
    else if (p->front == (p->max_size - 1))
    {
        /* Wrap around */
        p->front = 0;
    }
    else
    {
        (p->front)++;
    }

    return old_front;
}

int
cq_empty(CQ *p)
{
    if (p->front == -1)
    {
        return 1;
    }
    return 0;
}

int
cq_full(CQ *p)
{
    if ((p->front == 0 && (p->rear == (p->max_size - 1)))
        || ((p->max_size > 1)
            && (p->rear == ((p->front - 1) % (p->max_size - 1)))))
    {
        return 1;
    }
    return 0;
}

void
cq_reset(CQ *p)
{
    p->front = -1;
    p->rear = -1;
}

int
cq_front(CQ *p)
{
    return p->front;
}

int
cq_rear(CQ *p)
{
    return p->rear;
}

void
cq_set_rear(CQ *p, int rear)
{
    p->rear = rear;
}