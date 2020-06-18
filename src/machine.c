/*
 * VM utilities
 *
 * Copyright (c) 2017 Fabrice Bellard
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
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include "cutils.h"
#include "iomem.h"
#include "virtio.h"
#include "machine.h"
#include "fs_utils.h"
#ifdef CONFIG_FS_NET
#include "fs_wget.h"
#endif

#include "riscvsim/utils/sim_params_stats.h"

void __attribute__((format(printf, 1, 2))) vm_error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
#ifdef EMSCRIPTEN
    vprintf(fmt, ap);
#else
    vfprintf(stderr, fmt, ap);
#endif
    va_end(ap);
}

int vm_get_int(JSONValue obj, const char *name, int *pval)
{ 
    JSONValue val;
    val = json_object_get(obj, name);
    if (json_is_undefined(val)) {
        vm_error("expecting '%s' property\n", name);
        return -1;
    }
    if (val.type != JSON_INT) {
        vm_error("%s: integer expected\n", name);
        return -1;
    }
    *pval = val.u.int32;
    return 0;
}

int vm_get_int_opt(JSONValue obj, const char *name, int *pval, int def_val)
{ 
    JSONValue val;
    val = json_object_get(obj, name);
    if (json_is_undefined(val)) {
        *pval = def_val;
        return 0;
    }
    if (val.type != JSON_INT) {
        vm_error("%s: integer expected\n", name);
        return -1;
    }
    *pval = val.u.int32;
    return 0;
}

static int vm_get_str2(JSONValue obj, const char *name, const char **pstr,
                      BOOL is_opt)
{ 
    JSONValue val;
    val = json_object_get(obj, name);
    if (json_is_undefined(val)) {
        if (is_opt) {
            *pstr = NULL;
            return 0;
        } else {
            vm_error("expecting '%s' property\n", name);
            return -1;
        }
    }
    if (val.type != JSON_STR) {
        vm_error("%s: string expected\n", name);
        return -1;
    }
    *pstr = val.u.str->data;
    return 0;
}

static int vm_get_str(JSONValue obj, const char *name, const char **pstr)
{ 
    return vm_get_str2(obj, name, pstr, FALSE);
}

static int vm_get_str_opt(JSONValue obj, const char *name, const char **pstr)
{ 
    return vm_get_str2(obj, name, pstr, TRUE);
}

static char *strdup_null(const char *str)
{
    if (!str)
        return NULL;
    else
        return strdup(str);
}

/* currently only for "TZ" */
static char *cmdline_subst(const char *cmdline)
{
    DynBuf dbuf;
    const char *p;
    char var_name[32], *q, buf[32];
    
    dbuf_init(&dbuf);
    p = cmdline;
    while (*p != '\0') {
        if (p[0] == '$' && p[1] == '{') {
            p += 2;
            q = var_name;
            while (*p != '\0' && *p != '}') {
                if ((q - var_name) < sizeof(var_name) - 1)
                    *q++ = *p;
                p++;
            }
            *q = '\0';
            if (*p == '}')
                p++;
            if (!strcmp(var_name, "TZ")) {
                time_t ti;
                struct tm tm;
                int n, sg;
                /* get the offset to UTC */
                time(&ti);
                localtime_r(&ti, &tm);
                n = tm.tm_gmtoff / 60;
                sg = '-';
                if (n < 0) {
                    sg = '+';
                    n = -n;
                }
                snprintf(buf, sizeof(buf), "UTC%c%02d:%02d",
                         sg, n / 60, n % 60);
                dbuf_putstr(&dbuf, buf);
            }
        } else {
            dbuf_putc(&dbuf, *p++);
        }
    }
    dbuf_putc(&dbuf, 0);
    return (char *)dbuf.buf;
}

/* Max length for comma separated latency string for FU stages specified
in RISCVEMU config file */
#define LATENCY_STRING_MAX_LENGTH 256

static void parse_stage_latency_str(int **dest, int max_stage_count, char *str)
{
    int pos;
    char *ptr;

    /* Re-allocate default allocated latency array with the new size parsed from config file */ 
    *dest = (int *)realloc((*dest), max_stage_count * sizeof(int));
    assert(*dest);

    pos = 0;
    ptr = strtok(str, ",");
    while (ptr != NULL)
    {
        (*dest)[pos++] = atoi(ptr);
        ptr = strtok(NULL, ",");
        if (pos >= max_stage_count)
        {
            break;
        }
    }

    /* if latency string is shorter than max_stage_count, we must exit */
    if (pos != max_stage_count)
    {
        fprintf(stderr, "error: latency string %s does not specify all the latencies\n", str);
        exit(1);
    }
}

static BOOL find_name(const char *name, const char *name_list)
{
    size_t len;
    const char *p, *r;
    
    p = name_list;
    for(;;) {
        r = strchr(p, ',');
        if (!r) {
            if (!strcmp(name, p))
                return TRUE;
            break;
        } else {
            len = r - p;
            if (len == strlen(name) && !memcmp(name, p, len))
                return TRUE;
            p = r + 1;
        }
    }
    return FALSE;
}

static const VirtMachineClass *virt_machine_list[] = {
#if defined(EMSCRIPTEN)
    /* only a single machine in the EMSCRIPTEN target */
#ifndef CONFIG_X86EMU
    &riscv_machine_class,
#endif    
#else
    &riscv_machine_class,
#endif /* !EMSCRIPTEN */
#ifdef CONFIG_X86EMU
    &pc_machine_class,
#endif
    NULL,
};

static const VirtMachineClass *virt_machine_find_class(const char *machine_name)
{
    const VirtMachineClass *vmc, **pvmc;
    
    for(pvmc = virt_machine_list; *pvmc != NULL; pvmc++) {
        vmc = *pvmc;
        if (find_name(machine_name, vmc->machine_names))
            return vmc;
    }
    return NULL;
}

static int virt_machine_parse_config(VirtMachineParams *p,
                                     char *config_file_str, int len)
{
    int version, val;
    const char *tag_name, *str;
    char buf1[256];
    JSONValue cfg, obj, obj1, el;
    char stage_latency_str[LATENCY_STRING_MAX_LENGTH];
    
    cfg = json_parse_value_len(config_file_str, len);
    if (json_is_error(cfg)) {
        vm_error("error: %s\n", json_get_error(cfg));
        json_free(cfg);
        return -1;
    }

    if (vm_get_int(cfg, "version", &version) < 0)
        goto tag_fail;
    if (version != VM_CONFIG_VERSION) {
        if (version > VM_CONFIG_VERSION) {
            vm_error("The emulator is too old to run this VM: please upgrade\n");
            return -1;
        } else {
            vm_error("The VM configuration file is too old for this emulator version: please upgrade the VM configuration file\n");
            return -1;
        }
    }
    
    if (vm_get_str(cfg, "machine", &str) < 0)
        goto tag_fail;
    p->machine_name = strdup(str);
    p->vmc = virt_machine_find_class(p->machine_name);
    if (!p->vmc) {
        vm_error("Unknown machine name: %s\n", p->machine_name);
        goto tag_fail;
    }
    p->vmc->virt_machine_set_defaults(p);

    tag_name = "memory_size";
    if (vm_get_int(cfg, tag_name, &val) < 0)
        goto tag_fail;
    p->ram_size = (uint64_t)val << 20;
    
    tag_name = "bios";
    if (vm_get_str_opt(cfg, tag_name, &str) < 0)
        goto tag_fail;
    if (str) {
        p->files[VM_FILE_BIOS].filename = strdup(str);
    }

    tag_name = "kernel";
    if (vm_get_str_opt(cfg, tag_name, &str) < 0)
        goto tag_fail;
    if (str) {
        p->files[VM_FILE_KERNEL].filename = strdup(str);
    }

    tag_name = "initrd";
    if (vm_get_str_opt(cfg, tag_name, &str) < 0)
        goto tag_fail;
    if (str) {
        p->files[VM_FILE_INITRD].filename = strdup(str);
    }

    if (vm_get_str_opt(cfg, "cmdline", &str) < 0)
        goto tag_fail;
    if (str) {
        p->cmdline = cmdline_subst(str);
    }
    
    for(;;) {
        snprintf(buf1, sizeof(buf1), "drive%d", p->drive_count);
        obj = json_object_get(cfg, buf1);
        if (json_is_undefined(obj))
            break;
        if (p->drive_count >= MAX_DRIVE_DEVICE) {
            vm_error("Too many drives\n");
            return -1;
        }
        if (vm_get_str(obj, "file", &str) < 0)
            goto tag_fail;
        p->tab_drive[p->drive_count].filename = strdup(str);
        if (vm_get_str_opt(obj, "device", &str) < 0)
            goto tag_fail;
        p->tab_drive[p->drive_count].device = strdup_null(str);
        p->drive_count++;
    }

    for(;;) {
        snprintf(buf1, sizeof(buf1), "fs%d", p->fs_count);
        obj = json_object_get(cfg, buf1);
        if (json_is_undefined(obj))
            break;
        if (p->fs_count >= MAX_DRIVE_DEVICE) {
            vm_error("Too many filesystems\n");
            return -1;
        }
        if (vm_get_str(obj, "file", &str) < 0)
            goto tag_fail;
        p->tab_fs[p->fs_count].filename = strdup(str);
        if (vm_get_str_opt(obj, "tag", &str) < 0)
            goto tag_fail;
        if (!str) {
            if (p->fs_count == 0)
                strcpy(buf1, "/dev/root");
            else
                snprintf(buf1, sizeof(buf1), "/dev/root%d", p->fs_count);
            str = buf1;
        }
        p->tab_fs[p->fs_count].tag = strdup(str);
        p->fs_count++;
    }

    for(;;) {
        snprintf(buf1, sizeof(buf1), "eth%d", p->eth_count);
        obj = json_object_get(cfg, buf1);
        if (json_is_undefined(obj))
            break;
        if (p->eth_count >= MAX_ETH_DEVICE) {
            vm_error("Too many ethernet interfaces\n");
            return -1;
        }
        if (vm_get_str(obj, "driver", &str) < 0)
            goto tag_fail;
        p->tab_eth[p->eth_count].driver = strdup(str);
        if (!strcmp(str, "tap")) {
            if (vm_get_str(obj, "ifname", &str) < 0)
                goto tag_fail;
            p->tab_eth[p->eth_count].ifname = strdup(str);
        }
        p->eth_count++;
    }

    p->display_device = NULL;
    obj = json_object_get(cfg, "display0");
    if (!json_is_undefined(obj)) {
        if (vm_get_str(obj, "device", &str) < 0)
            goto tag_fail;
        p->display_device = strdup(str);
        if (vm_get_int(obj, "width", &p->width) < 0)
            goto tag_fail;
        if (vm_get_int(obj, "height", &p->height) < 0)
            goto tag_fail;
        if (vm_get_str_opt(obj, "vga_bios", &str) < 0)
            goto tag_fail;
        if (str) {
            p->files[VM_FILE_VGA_BIOS].filename = strdup(str);
        }
    }

    if (vm_get_str_opt(cfg, "input_device", &str) < 0)
        goto tag_fail;
    p->input_device = strdup_null(str);

    if (vm_get_str_opt(cfg, "accel", &str) < 0)
        goto tag_fail;
    if (str) {
        if (!strcmp(str, "none")) {
            p->accel_enable = FALSE;
        } else if (!strcmp(str, "auto")) {
            p->accel_enable = TRUE;
        } else {
            vm_error("unsupported 'accel' config: %s\n", str);
            return -1;
        }
    }

    tag_name = "rtc_local_time";
    el = json_object_get(cfg, tag_name);
    if (!json_is_undefined(el)) {
        if (el.type != JSON_BOOL) {
            vm_error("%s: boolean expected\n", tag_name);
            goto tag_fail;
        }
        p->rtc_local_time = el.u.b;
    }

    /* Parse simulation options */
    tag_name = "core_name";
    if (vm_get_str(cfg, tag_name, &str) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %s\n", tag_name,
            p->sim_params->core_name);
    } else {
        free(p->sim_params->core_name);
        p->sim_params->core_name = strdup(str);
    }

    tag_name = "core_type";
    if (vm_get_str(cfg, tag_name, &str) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %s\n", tag_name,
            core_type_str[p->sim_params->core_type]);
    } else {
        if (strcmp(str, "incore") == 0){
            p->sim_params->core_type = CORE_TYPE_INCORE;
        } else if (strcmp(str, "oocore") == 0){
            p->sim_params->core_type = CORE_TYPE_OOCORE;
        }
    }

    if (p->sim_params->core_type == CORE_TYPE_INCORE) {
        tag_name = "num_cpu_stages";
        if (vm_get_int(cfg, tag_name, &p->sim_params->num_cpu_stages) < 0) {
            fprintf(stderr, "%s not found, selecting default value: %d\n",
                    tag_name, p->sim_params->num_cpu_stages);
        }

        tag_name = "enable_parallel_fu";
        if (vm_get_str(cfg, tag_name, &str) < 0) {
            fprintf(stderr, "%s not found, selecting default value %s\n", tag_name,
                    sim_param_status[p->sim_params->enable_parallel_fu]);
        }
        else {
            if (strcmp(str, "false") == 0) {
                p->sim_params->enable_parallel_fu = DISABLE;
            } else if (strcmp(str, "true") == 0) {
                p->sim_params->enable_parallel_fu = ENABLE;
            } else {
                fprintf(stderr, "error: option %s has invalid value\n", tag_name);
                exit(1);
            }
        }

    } else if (p->sim_params->core_type == CORE_TYPE_OOCORE) {
        tag_name = "iq_size";
        if (vm_get_int(cfg, tag_name, &p->sim_params->iq_size) < 0) {
            fprintf(stderr, "%s not found, selecting default value: %d\n",
                    tag_name, p->sim_params->iq_size);
        }

        tag_name = "iq_issue_ports";
        if (vm_get_int(cfg, tag_name, &p->sim_params->iq_issue_ports) < 0) {
            fprintf(stderr, "%s not found, selecting default value: %d\n",
                    tag_name, p->sim_params->iq_issue_ports);
        }

        tag_name = "rob_size";
        if (vm_get_int(cfg, tag_name, &p->sim_params->rob_size) < 0) {
            fprintf(stderr, "%s not found, selecting default value: %d\n",
                    tag_name, p->sim_params->rob_size);
        }

        tag_name = "rob_commit_ports";
        if (vm_get_int(cfg, tag_name, &p->sim_params->rob_commit_ports) < 0) {
            fprintf(stderr, "%s not found, selecting default value: %d\n",
                    tag_name, p->sim_params->rob_commit_ports);
        }


        tag_name = "lsq_size";
        if (vm_get_int(cfg, tag_name, &p->sim_params->lsq_size) < 0) {
            fprintf(stderr, "%s not found, selecting default value: %d\n",
                    tag_name, p->sim_params->lsq_size);
        }
    }

    tag_name = "sim_stats_path";
    if (vm_get_str(cfg, tag_name, &str) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %s\n",
                tag_name, p->sim_params->sim_stats_path);
    } else {
        free(p->sim_params->sim_stats_path);
        p->sim_params->sim_stats_path = strdup(str);
    }

    snprintf(buf1, sizeof(buf1), "%s", "execution_units");
    obj = json_object_get(cfg, buf1);

    if (json_is_undefined(obj)) {
    fprintf(stderr, "%s object not found, selecting default values\n",
            buf1);
    }

    tag_name = "num_alu_stages";
    if (vm_get_int(obj, tag_name, &p->sim_params->num_alu_stages) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->num_alu_stages);
    }

    tag_name = "alu_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0) {
      fprintf(stderr, "%s not found, selecting default value\n", tag_name);
    } else {
      strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
      stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
      parse_stage_latency_str(&p->sim_params->alu_stage_latency,
                              p->sim_params->num_alu_stages, stage_latency_str);
    }

    tag_name = "num_mul_stages";
    if (vm_get_int(obj, tag_name, &p->sim_params->num_mul_stages) < 0) {
      fprintf(stderr, "%s not found, selecting default value: %d\n", tag_name,
              p->sim_params->num_mul_stages);
    }

    tag_name = "mul_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0) {
      fprintf(stderr, "%s not found, selecting default value\n", tag_name);
    } else {
      strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
      stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
      parse_stage_latency_str(&p->sim_params->mul_stage_latency,
                              p->sim_params->num_mul_stages, stage_latency_str);
    }

    tag_name = "num_div_stages";
    if (vm_get_int(obj, tag_name, &p->sim_params->num_div_stages) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->num_div_stages);
    }

    tag_name = "div_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0) {
      fprintf(stderr, "%s not found, selecting default value\n", tag_name);
    } else {
      strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
      stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
      parse_stage_latency_str(&p->sim_params->div_stage_latency,
                              p->sim_params->num_div_stages, stage_latency_str);
    }

    tag_name = "num_fpu_fma_stages";
    if (vm_get_int(obj, tag_name, &p->sim_params->num_fpu_fma_stages) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->num_fpu_fma_stages);
    }

    tag_name = "fpu_fma_stage_latency";
    if (vm_get_str(obj, tag_name, &str) < 0) {
      fprintf(stderr, "%s not found, selecting default value\n", tag_name);
    } else {
      strncpy(stage_latency_str, str, LATENCY_STRING_MAX_LENGTH - 1);
      stage_latency_str[LATENCY_STRING_MAX_LENGTH - 1] = '\0';
      parse_stage_latency_str(&p->sim_params->fpu_fma_stage_latency,
                              p->sim_params->num_fpu_fma_stages,
                              stage_latency_str);
    }

    snprintf(buf1, sizeof(buf1), "%s", "fpu_alu");
    obj1 = json_object_get(obj, buf1);

    tag_name = "fadd";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FADD]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FADD]);
    }

    tag_name = "fsub";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FSUB]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FSUB]);
    }

    tag_name = "fmul";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMUL]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMUL]);
    }

    tag_name = "fdiv";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FDIV]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FDIV]);
    }

    tag_name = "fsqrt";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FSQRT]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FSQRT]);
    }

    tag_name = "fsgnj";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FSGNJ]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FSGNJ]);
    }

    tag_name = "fmin";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMIN]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMIN]);
    }

    tag_name = "fmax";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMAX]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMAX]);
    }

    tag_name = "feq";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FEQ]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FEQ]);
    }

    tag_name = "flt";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FLT]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FLT]);
    }

    tag_name = "fle";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FLE]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FLE]);
    }

    tag_name = "fcvt";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FCVT]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FCVT]);
    }

    tag_name = "cvt";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_CVT]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_CVT]);
    }

    tag_name = "fmv";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMV]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FMV]);
    }

    tag_name = "fclass";
    if (vm_get_int(obj1, tag_name, &p->sim_params->fpu_alu_latency[FU_FPU_ALU_FCLASS]) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->fpu_alu_latency[FU_FPU_ALU_FCLASS]);
    }

    tag_name = "sim_trace_file";
    if (vm_get_str(cfg, tag_name, &str) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %s\n",
                tag_name, p->sim_params->sim_trace_file);
    } else {
        free(p->sim_params->sim_trace_file);
        p->sim_params->sim_trace_file = strdup(str);
    }

    tag_name = "tlb_size";
    if (vm_get_int(cfg, tag_name, &p->sim_params->tlb_size) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->tlb_size);
    }

    tag_name = "pte_rw_latency";
    if (vm_get_int(cfg, tag_name, &p->sim_params->pte_rw_latency) < 0) {
        fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->pte_rw_latency);
    }

    /* BPU */
    tag_name = "enable_bpu";
    if (vm_get_str(cfg, tag_name, &str) < 0) {
        fprintf(stderr, "%s not found, selecting default value %s\n", tag_name,
                sim_param_status[p->sim_params->enable_bpu]);
    }
    else {
        if (strcmp(str, "false") == 0) {
            p->sim_params->enable_bpu = DISABLE;
        } else if (strcmp(str, "true") == 0) {
            p->sim_params->enable_bpu = ENABLE;
        } else {
            fprintf(stderr, "error: option %s has invalid value\n", tag_name);
            exit(1);
        }
    }

    if (p->sim_params->enable_bpu == ENABLE) {
        tag_name = "btb_size";
        if (vm_get_int(cfg, tag_name, &p->sim_params->btb_size) < 0) {
          fprintf(stderr, "%s not found, selecting default value: %d\n",
                  tag_name, p->sim_params->btb_size);
        }

        tag_name = "btb_ways";
        if (vm_get_int(cfg, tag_name, &p->sim_params->btb_ways) < 0) {
          fprintf(stderr, "%s not found, selecting default value: %d\n",
                  tag_name, p->sim_params->btb_ways);
        }

        tag_name = "bpu_type";
        if (vm_get_str(cfg, tag_name, &str) < 0) {
            fprintf(stderr, "%s not found, selecting default value %s\n", tag_name,
                    bpu_type_str[p->sim_params->bpu_type]);
        }
        else {
            if (strcmp(str, "bimodal") == 0) {
                p->sim_params->bpu_type = BPU_TYPE_BIMODAL;
            } else if (strcmp(str, "adaptive") == 0) {
                p->sim_params->bpu_type = BPU_TYPE_ADAPTIVE;
            } else {
                fprintf(stderr, "error: option %s has invalid value\n", tag_name);
                exit(1);
            }
        }

        tag_name = "btb_eviction_policy";
        if (vm_get_str(cfg, tag_name, &str) < 0) {
            fprintf(stderr, "%s not found, selecting default value %s\n", tag_name,
                    evict_policy_str[p->sim_params->btb_eviction_policy]);
        }
        else {
            if (strcmp(str, "random") == 0) {
                p->sim_params->btb_eviction_policy  = EVICT_POLICY_RANDOM;
            } else if (strcmp(str, "lru") == 0) {
                p->sim_params->btb_eviction_policy = EVICT_POLICY_BIT_PLRU;
            } else {
                fprintf(stderr, "error: option %s has invalid value\n", tag_name);
                exit(1);
            }
        }

        if (p->sim_params->bpu_type == BPU_TYPE_ADAPTIVE) {
            tag_name = "bpu_ght_size";
            if (vm_get_int(cfg, tag_name, &p->sim_params->bpu_ght_size) < 0) {
              fprintf(stderr, "%s not found, selecting default value: %d\n",
                      tag_name, p->sim_params->bpu_ght_size);
            }

            tag_name = "bpu_pht_size";
            if (vm_get_int(cfg, tag_name, &p->sim_params->bpu_pht_size) < 0) {
              fprintf(stderr, "%s not found, selecting default value: %d\n",
                      tag_name, p->sim_params->bpu_pht_size);
            }

            tag_name = "bpu_history_bits";
            if (vm_get_int(cfg, tag_name, &p->sim_params->bpu_history_bits) < 0) {
              fprintf(stderr, "%s not found, selecting default value: %d\n",
                      tag_name, p->sim_params->bpu_history_bits);
            }

            if ((p->sim_params->bpu_ght_size == 1) && (p->sim_params->bpu_pht_size == 1))
            {
                tag_name = "bpu_aliasing_func_type";
                if (vm_get_str(cfg, tag_name, &str) < 0) {
                    fprintf(stderr, "%s not found, selecting default value %s\n", tag_name,
                            bpu_aliasing_func_type_str[p->sim_params->bpu_aliasing_func_type]);
                }
                else {
                    if (strcmp(str, "xor") == 0) {
                        p->sim_params->bpu_aliasing_func_type  = BPU_ALIAS_FUNC_XOR;
                    } else if (strcmp(str, "and") == 0) {
                        p->sim_params->bpu_aliasing_func_type = BPU_ALIAS_FUNC_AND;
                    } else if (strcmp(str, "none") == 0) {
                        p->sim_params->bpu_aliasing_func_type = BPU_ALIAS_FUNC_NONE;
                    } else {
                        fprintf(stderr, "error: option %s has invalid value\n", tag_name);
                        exit(1);
                    }
                }
            }
        } else {
            tag_name = "bht_size";
            if (vm_get_int(cfg, tag_name, &p->sim_params->bht_size) < 0) {
              fprintf(stderr, "%s not found, selecting default value: %d\n",
                      tag_name, p->sim_params->bht_size);
            }
        }

        tag_name = "ras_size";
        if (vm_get_int(cfg, tag_name, &p->sim_params->ras_size) < 0) {
              fprintf(stderr, "%s not found, selecting default value: %d\n",
                tag_name, p->sim_params->ras_size);
        }

    }

    /* L1 Caches */
    tag_name = "enable_l1_caches";
    if (vm_get_str(cfg, tag_name, &str) < 0) {
        fprintf(stderr, "%s not found, selecting default value %s\n", tag_name,
                sim_param_status[p->sim_params->enable_l1_caches]);
    }
    else {
        if (strcmp(str, "false") == 0) {
            p->sim_params->enable_l1_caches = DISABLE;
        } else if (strcmp(str, "true") == 0) {
            p->sim_params->enable_l1_caches = ENABLE;
        } else {
            fprintf(stderr, "error: option %s has invalid value\n", tag_name);
            exit(1);
        }
    }

    if (p->sim_params->enable_l1_caches) {
      snprintf(buf1, sizeof(buf1), "%s", "icache");
      obj = json_object_get(cfg, buf1);

      if (json_is_undefined(obj)) {
        fprintf(stderr, "%s object not found, selecting default values\n",
                buf1);
      }

      tag_name = "read_latency";
      if (vm_get_int(obj, tag_name, &p->sim_params->l1_code_cache_read_latency) < 0) {
        fprintf(stderr, "icache %s not found, selecting default value: %d\n",
                tag_name, p->sim_params->l1_code_cache_read_latency);
      }

      tag_name = "size";
      if (vm_get_int(obj, tag_name, &p->sim_params->l1_code_cache_size) < 0) {
        fprintf(stderr, "icache %s not found, selecting default value: %d\n",
                tag_name, p->sim_params->l1_code_cache_size);
      }

      tag_name = "ways";
      if (vm_get_int(obj, tag_name, &p->sim_params->l1_code_cache_ways) < 0) {
        fprintf(stderr, "icache %s not found, selecting default value: %d\n",
                tag_name, p->sim_params->l1_code_cache_ways);
      }

      tag_name = "eviction";
      if (vm_get_str(obj, tag_name, &str) < 0) {
          fprintf(stderr, "icache %s policy not found, selecting default value %s\n",
                  tag_name, evict_policy_str[p->sim_params->l1_code_cache_evict]);
      }
      else {
         if (strcmp(str, "lru") == 0) {
             p->sim_params->l1_code_cache_evict = EVICT_POLICY_BIT_PLRU;
         } else if (strcmp(str, "random") == 0) {
             p->sim_params->l1_code_cache_evict = EVICT_POLICY_RANDOM;
         } else {
             fprintf(stderr, "error: option icache %s policy has invalid value\n", tag_name);
             exit(1);
         }
      }

      snprintf(buf1, sizeof(buf1), "%s", "dcache");
      obj = json_object_get(cfg, buf1);

      if (json_is_undefined(obj)) {
        fprintf(stderr, "%s object not found, selecting default values\n",
                buf1);
      }

      tag_name = "read_latency";
      if (vm_get_int(obj, tag_name, &p->sim_params->l1_data_cache_read_latency) < 0) {
        fprintf(stderr, "dcache %s not found, selecting default value: %d\n",
                tag_name, p->sim_params->l1_data_cache_read_latency);
      }

      tag_name = "write_latency";
      if (vm_get_int(obj, tag_name, &p->sim_params->l1_data_cache_write_latency) < 0) {
        fprintf(stderr, "dcache %s not found, selecting default value: %d\n",
                tag_name, p->sim_params->l1_data_cache_write_latency);
      }

      tag_name = "size";
      if (vm_get_int(obj, tag_name, &p->sim_params->l1_data_cache_size) < 0) {
        fprintf(stderr, "dcache %s not found, selecting default value: %d\n",
                tag_name, p->sim_params->l1_data_cache_size);
      }

      tag_name = "ways";
      if (vm_get_int(obj, tag_name, &p->sim_params->l1_data_cache_ways) < 0) {
        fprintf(stderr, "dcache %s not found, selecting default value: %d\n",
                tag_name, p->sim_params->l1_data_cache_ways);
      }

      tag_name = "eviction";
      if (vm_get_str(obj, tag_name, &str) < 0) {
          fprintf(stderr, "dcache %s policy not found, selecting default value %s\n",
                  tag_name, evict_policy_str[p->sim_params->l1_data_cache_evict]);
      }
      else {
         if (strcmp(str, "lru") == 0) {
             p->sim_params->l1_data_cache_evict = EVICT_POLICY_BIT_PLRU;
         } else if (strcmp(str, "random") == 0) {
             p->sim_params->l1_data_cache_evict = EVICT_POLICY_RANDOM;
         } else {
             fprintf(stderr, "error: option dcache %s policy has invalid value\n", tag_name);
             exit(1);
         }
      }

      tag_name = "words_per_cache_line";
      if (vm_get_int(cfg, tag_name, &p->sim_params->words_per_cache_line) < 0) {
          fprintf(stderr, "%s not found, selecting default value: %d\n", tag_name,
                p->sim_params->words_per_cache_line);
      }

      // tag_name = "cache_allocate_on_read_miss";
      // if (vm_get_str(cfg, tag_name, &str) < 0) {
      //     fprintf(stderr, "%s not found, selecting default value %s\n",
      //     tag_name, cache_ra_str[p->sim_params->cache_read_allocate_policy]);
      // }
      // else {
      //     if (strcmp(str, "true") == 0) {
      //         p->sim_params->cache_read_allocate_policy = CACHE_READ_ALLOC;
      //     } else if (strcmp(str, "false") == 0) {
      //         p->sim_params->cache_read_allocate_policy = CACHE_READ_NO_ALLOC;
      //     } else {
      //        fprintf(stderr, "error: option %s has invalid value\n", tag_name);
      //        exit(1);
      //     }
      // }

      tag_name = "cache_allocate_on_write_miss";
      if (vm_get_str(cfg, tag_name, &str) < 0) {
          fprintf(stderr, "%s not found, selecting default value %s\n",
          tag_name, cache_wa_str[p->sim_params->cache_write_allocate_policy]);
      }
      else {
          if (strcmp(str, "true") == 0) {
              p->sim_params->cache_write_allocate_policy = CACHE_WRITE_ALLOC;
          } else if (strcmp(str, "false") == 0) {
              p->sim_params->cache_write_allocate_policy = CACHE_WRITE_NO_ALLOC;
          } else {
             fprintf(stderr, "error: option %s has invalid value\n", tag_name);
             exit(1);
          }
      }

      tag_name = "cache_write_policy";
      if (vm_get_str(cfg, tag_name, &str) < 0) {
          fprintf(stderr, "%s not found, selecting default value %s\n",
          tag_name, cache_wp_str[p->sim_params->cache_write_policy]);
      }
      else {
          if (strcmp(str, "writeback") == 0) {
              p->sim_params->cache_write_policy = CACHE_WRITEBACK;
          } else if (strcmp(str, "writethrough") == 0) {
              p->sim_params->cache_write_policy = CACHE_WRITETHROUGH;
          } else {
             fprintf(stderr, "error: option %s has invalid value\n", tag_name);
             exit(1);
          }
      }

      tag_name = "enable_l2_cache";
      if (vm_get_str(cfg, tag_name, &str) < 0) {
          fprintf(stderr, "%s not found, selecting default value %s\n",
                  tag_name, sim_param_status[p->sim_params->enable_l2_cache]);
      }
      else {
          if (strcmp(str, "false") == 0) {
              p->sim_params->enable_l2_cache = DISABLE;
          } else if (strcmp(str, "true") == 0) {
              p->sim_params->enable_l2_cache = ENABLE;
          } else {
              fprintf(stderr, "error: option %s has invalid value\n", tag_name);
              exit(1);
          }
      }

      if (p->sim_params->enable_l2_cache) {
        snprintf(buf1, sizeof(buf1), "%s", "l2_shared_cache");
        obj = json_object_get(cfg, buf1);

        if (json_is_undefined(obj)) {
          fprintf(stderr, "%s object not found, selecting default values\n",
                  buf1);
        }

       tag_name = "read_latency";
       if (vm_get_int(obj, tag_name, &p->sim_params->l2_shared_cache_read_latency) < 0) {
         fprintf(stderr, "dcache %s not found, selecting default value: %d\n",
                 tag_name, p->sim_params->l2_shared_cache_read_latency);
       }

       tag_name = "write_latency";
       if (vm_get_int(obj, tag_name, &p->sim_params->l2_shared_cache_write_latency) < 0) {
         fprintf(stderr, "dcache %s not found, selecting default value: %d\n",
                 tag_name, p->sim_params->l2_shared_cache_write_latency);
       }

        tag_name = "size";
        if (vm_get_int(obj, tag_name, &p->sim_params->l2_shared_cache_size) < 0) {
          fprintf(stderr, "l2_shared_cache %s not found, selecting default value: %d\n",
                  tag_name, p->sim_params->l2_shared_cache_size);
        }

        tag_name = "ways";
        if (vm_get_int(obj, tag_name, &p->sim_params->l2_shared_cache_ways) < 0) {
          fprintf(stderr, "l2_shared_cache %s not found, selecting default value: %d\n",
                  tag_name, p->sim_params->l2_shared_cache_ways);
        }

        tag_name = "eviction";
        if (vm_get_str(obj, tag_name, &str) < 0) {
            fprintf(stderr, "l2-cache %s policy not found, selecting default value %s\n",
                    tag_name, evict_policy_str[p->sim_params->l2_shared_cache_evict]);
        }
        else {
            if (strcmp(str, "lru") == 0) {
                p->sim_params->l2_shared_cache_evict = EVICT_POLICY_BIT_PLRU;
            } else if (strcmp(str, "random") == 0) {
                p->sim_params->l2_shared_cache_evict = EVICT_POLICY_RANDOM;
            } else {
                fprintf(stderr, "error: option l2-cache %s policy has invalid value\n", tag_name);
                exit(1);
            }
         }
       }
    }


    switch (p->sim_params->dram_model_type)
    {
        case MEM_MODEL_BASE:
        {
            tag_name = "mem_access_latency";
            if (vm_get_int(cfg, tag_name, &p->sim_params->mem_access_latency) < 0) {
              fprintf(stderr, "%s not found, selecting default value: %d\n", tag_name,
                      p->sim_params->mem_access_latency);
            }
            tag_name = "burst_length";
            if (vm_get_int(cfg, tag_name, (int *)&p->sim_params->burst_length) < 0) {
              fprintf(stderr, "%s not found, selecting default value: %u\n", tag_name,
                      p->sim_params->burst_length);
            }
            break;
        }
        case MEM_MODEL_DRAMSIM:
        {
            tag_name = "dramsim_ini_file";
            if (vm_get_str(cfg, tag_name, &str) < 0) {
                fprintf(stderr, "%s not found, selecting default value: %s\n", tag_name,
                    p->sim_params->dramsim_ini_file);
            } else {
                free(p->sim_params->dramsim_ini_file);
                p->sim_params->dramsim_ini_file = strdup(str);
            }

            tag_name = "dramsim_system_ini_file";
            if (vm_get_str(cfg, tag_name, &str) < 0) {
                fprintf(stderr, "%s not found, selecting default value: %s\n", tag_name,
                    p->sim_params->dramsim_system_ini_file);
            } else {
                free(p->sim_params->dramsim_system_ini_file);
                p->sim_params->dramsim_system_ini_file = strdup(str);
            }

            tag_name = "dramsim_stats_dir";
            if (vm_get_str(cfg, tag_name, &str) < 0) {
                fprintf(stderr, "%s not found, selecting default value: %s\n", tag_name,
                    p->sim_params->dramsim_stats_dir);
            } else {
                free(p->sim_params->dramsim_stats_dir);
                p->sim_params->dramsim_stats_dir = strdup(str);
            }
            break;
        }
        default:
        {
            fprintf(stderr, "error: invalid memory model\n");
            exit(1);
        }
    }

    p->sim_params->guest_ram_size = p->ram_size >> 20;

    json_free(cfg);
    return 0;
 tag_fail:
    json_free(cfg);
    return -1;
}

typedef void FSLoadFileCB(void *opaque, uint8_t *buf, int buf_len);

typedef struct {
    VirtMachineParams *vm_params;
    void (*start_cb)(void *opaque);
    void *opaque;
    
    FSLoadFileCB *file_load_cb;
    void *file_load_opaque;
    int file_index;
} VMConfigLoadState;

static void config_file_loaded(void *opaque, uint8_t *buf, int buf_len);
static void config_additional_file_load(VMConfigLoadState *s);
static void config_additional_file_load_cb(void *opaque,
                                           uint8_t *buf, int buf_len);

/* XXX: win32, URL */
char *get_file_path(const char *base_filename, const char *filename)
{
    int len, len1;
    char *fname, *p;
    
    if (!base_filename)
        goto done;
    if (strchr(filename, ':'))
        goto done; /* full URL */
    if (filename[0] == '/')
        goto done;
    p = strrchr(base_filename, '/');
    if (!p) {
    done:
        return strdup(filename);
    }
    len = p + 1 - base_filename;
    len1 = strlen(filename);
    fname = malloc(len + len1 + 1);
    memcpy(fname, base_filename, len);
    memcpy(fname + len, filename, len1 + 1);
    return fname;
}


#ifdef EMSCRIPTEN
static int load_file(uint8_t **pbuf, const char *filename)
{
    abort();
}
#else
/* return -1 if error. */
static int load_file(uint8_t **pbuf, const char *filename)
{
    FILE *f;
    int size;
    uint8_t *buf;
    
    f = fopen(filename, "rb");
    if (!f) {
        perror(filename);
        exit(1);
    }
    fseek(f, 0, SEEK_END);
    size = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc(size);
    if (fread(buf, 1, size, f) != size) {
        fprintf(stderr, "%s: read error\n", filename);
        exit(1);
    }
    fclose(f);
    *pbuf = buf;
    return size;
}
#endif

#ifdef CONFIG_FS_NET
static void config_load_file_cb(void *opaque, int err, void *data, size_t size)
{
    VMConfigLoadState *s = opaque;
    
    //    printf("err=%d data=%p size=%ld\n", err, data, size);
    if (err < 0) {
        vm_error("Error %d while loading file\n", -err);
        exit(1);
    }
    s->file_load_cb(s->file_load_opaque, data, size);
}
#endif

static void config_load_file(VMConfigLoadState *s, const char *filename,
                             FSLoadFileCB *cb, void *opaque)
{
    //    printf("loading %s\n", filename);
#ifdef CONFIG_FS_NET
    if (is_url(filename)) {
        s->file_load_cb = cb;
        s->file_load_opaque = opaque;
        fs_wget(filename, NULL, NULL, s, config_load_file_cb, TRUE);
    } else
#endif
    {
        uint8_t *buf;
        int size;
        size = load_file(&buf, filename);
        cb(opaque, buf, size);
        free(buf);
    }
}

void virt_machine_load_config_file(VirtMachineParams *p,
                                   const char *filename,
                                   void (*start_cb)(void *opaque),
                                   void *opaque)
{
    VMConfigLoadState *s;
    
    s = mallocz(sizeof(*s));
    s->vm_params = p;
    s->start_cb = start_cb;
    s->opaque = opaque;
    p->cfg_filename = strdup(filename);

    config_load_file(s, filename, config_file_loaded, s);
}

static void config_file_loaded(void *opaque, uint8_t *buf, int buf_len)
{
    VMConfigLoadState *s = opaque;
    VirtMachineParams *p = s->vm_params;

    if (virt_machine_parse_config(p, (char *)buf, buf_len) < 0)
        exit(1);
    
    /* load the additional files */
    s->file_index = 0;
    config_additional_file_load(s);
}

static void config_additional_file_load(VMConfigLoadState *s)
{
    VirtMachineParams *p = s->vm_params;
    while (s->file_index < VM_FILE_COUNT &&
           p->files[s->file_index].filename == NULL) {
        s->file_index++;
    }
    if (s->file_index == VM_FILE_COUNT) {
        if (s->start_cb)
            s->start_cb(s->opaque);
        free(s);
    } else {
        char *fname;
        
        fname = get_file_path(p->cfg_filename,
                              p->files[s->file_index].filename);
        config_load_file(s, fname,
                         config_additional_file_load_cb, s);
        free(fname);
    }
}

static void config_additional_file_load_cb(void *opaque,
                                           uint8_t *buf, int buf_len)
{
    VMConfigLoadState *s = opaque;
    VirtMachineParams *p = s->vm_params;

    p->files[s->file_index].buf = malloc(buf_len);
    memcpy(p->files[s->file_index].buf, buf, buf_len);
    p->files[s->file_index].len = buf_len;

    /* load the next files */
    s->file_index++;
    config_additional_file_load(s);
}

void vm_add_cmdline(VirtMachineParams *p, const char *cmdline)
{
    char *new_cmdline, *old_cmdline;
    if (cmdline[0] == '!') {
        new_cmdline = strdup(cmdline + 1);
    } else {
        old_cmdline = p->cmdline;
        if (!old_cmdline)
            old_cmdline = "";
        new_cmdline = malloc(strlen(old_cmdline) + 1 + strlen(cmdline) + 1);
        strcpy(new_cmdline, old_cmdline);
        strcat(new_cmdline, " ");
        strcat(new_cmdline, cmdline);
    }
    free(p->cmdline);
    p->cmdline = new_cmdline;
}

void virt_machine_free_config(VirtMachineParams *p)
{
    int i;
    
    free(p->machine_name);
    free(p->cmdline);
    for(i = 0; i < VM_FILE_COUNT; i++) {
        free(p->files[i].filename);
        free(p->files[i].buf);
    }
    for(i = 0; i < p->drive_count; i++) {
        free(p->tab_drive[i].filename);
        free(p->tab_drive[i].device);
    }
    for(i = 0; i < p->fs_count; i++) {
        free(p->tab_fs[i].filename);
        free(p->tab_fs[i].tag);
    }
    for(i = 0; i < p->eth_count; i++) {
        free(p->tab_eth[i].driver);
        free(p->tab_eth[i].ifname);
    }
    free(p->input_device);
    free(p->display_device);
    free(p->cfg_filename);
}

VirtMachine *virt_machine_init(const VirtMachineParams *p)
{
    const VirtMachineClass *vmc = p->vmc;
    return vmc->virt_machine_init(p);
}

void virt_machine_set_defaults(VirtMachineParams *p)
{
    memset(p, 0, sizeof(*p));
    p->sim_params = sim_params_init();
}

void virt_machine_end(VirtMachine *s)
{
    s->vmc->virt_machine_end(s);
}
