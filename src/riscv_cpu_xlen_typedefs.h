#ifndef _RISCV_CPU_XLEN_TYPEDEFS_
#define _RISCV_CPU_XLEN_TYPEDEFS_

#ifndef FLEN
#if MAX_XLEN == 128
#define FLEN 128
#else
#define FLEN 64
#define SIM_FLEN 64
#endif
#endif /* !FLEN */

#if MAX_XLEN == 32
typedef uint32_t target_ulong;
typedef int32_t target_long;
#define PR_target_ulong "08x"
#elif MAX_XLEN == 64
typedef uint64_t target_ulong;
typedef int64_t target_long;
#define PR_target_ulong "016" PRIx64
#elif MAX_XLEN == 128
typedef uint128_t target_ulong;
typedef int128_t target_long;
#define PR_target_ulong "016" PRIx64 /* XXX */
#else
#error unsupported MAX_XLEN
#endif

/* FLEN is the floating point register width */
#if FLEN > 0
#if FLEN == 32
typedef uint32_t fp_uint;
#define F32_HIGH 0
#elif FLEN == 64
typedef uint64_t fp_uint;
#define F32_HIGH ((fp_uint)-1 << 32)
#define F64_HIGH 0
#elif FLEN == 128
typedef uint128_t fp_uint;
#define F32_HIGH ((fp_uint)-1 << 32)
#define F64_HIGH ((fp_uint)-1 << 64)
#else
#error unsupported FLEN
#endif
#endif

/* MLEN is the maximum memory access width */
#if MAX_XLEN <= 32 && FLEN <= 32
#define MLEN 32
#elif MAX_XLEN <= 64 && FLEN <= 64
#define MLEN 64
#else
#define MLEN 128
#endif

#if MLEN == 32
typedef uint32_t mem_uint_t;
#elif MLEN == 64
typedef uint64_t mem_uint_t;
#elif MLEN == 128
typedef uint128_t mem_uint_t;
#else
#unsupported MLEN
#endif

#endif