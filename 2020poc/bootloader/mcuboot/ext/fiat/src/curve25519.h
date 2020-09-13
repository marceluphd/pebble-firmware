/* Autogenerated */
/* curve description: 25519 */
/* requested operations: carry_mul, carry_square, carry_scmul121666, carry, add, sub, opp, selectznz, to_bytes, from_bytes */
/* n = 10 (from "10") */
/* s = 0x8000000000000000000000000000000000000000000000000000000000000000 (from "2^255") */
/* c = [(1, 19)] (from "1,19") */
/* machine_wordsize = 32 (from "32") */

#include <stdint.h>

#ifndef UINT64_C
#define UINT64_C(x) x##ULL
#endif
#ifndef UINT32_C
#define UINT32_C(x) x##UL
#endif
#ifndef UINT8_C
#define UINT8_C(x) (x)
#endif

// fe means field element. Here the field is \Z/(2^255-19). An element t,
// entries t[0]...t[9], represents the integer t[0]+2^26 t[1]+2^51 t[2]+2^77
// t[3]+2^102 t[4]+...+2^230 t[9].
// fe limbs are bounded by 1.125*2^26,1.125*2^25,1.125*2^26,1.125*2^25,etc.
// Multiplication and carrying produce fe from fe_loose.
typedef struct fe { uint32_t v[10]; } fe;

// fe_loose limbs are bounded by 3.375*2^26,3.375*2^25,3.375*2^26,3.375*2^25,etc.
// Addition and subtraction produce fe_loose from (fe, fe).
typedef struct fe_loose { uint32_t v[10]; } fe_loose;

// ge means group element.
//
// Here the group is the set of pairs (x,y) of field elements (see fe.h)
// satisfying -x^2 + y^2 = 1 + d x^2y^2
// where d = -121665/121666.
//
// Representations:
//   ge_p2 (projective): (X:Y:Z) satisfying x=X/Z, y=Y/Z
//   ge_p3 (extended): (X:Y:Z:T) satisfying x=X/Z, y=Y/Z, XY=ZT
//   ge_p1p1 (completed): ((X:Z),(Y:T)) satisfying x=X/Z, y=Y/T
//   ge_precomp (Duif): (y+x,y-x,2dxy)

typedef struct {
  fe X;
  fe Y;
  fe Z;
} ge_p2;

typedef struct {
  fe X;
  fe Y;
  fe Z;
  fe T;
} ge_p3;

typedef struct {
  fe_loose X;
  fe_loose Y;
  fe_loose Z;
  fe_loose T;
} ge_p1p1;

typedef struct {
  fe_loose yplusx;
  fe_loose yminusx;
  fe_loose xy2d;
} ge_precomp;

typedef struct {
  fe_loose YplusX;
  fe_loose YminusX;
  fe_loose Z;
  fe_loose T2d;
} ge_cached;

typedef unsigned char fiat_25519_uint1;
typedef signed char fiat_25519_int1;

/*
 * Input Bounds:
 *   arg1: [0x0 ~> 0x1]
 *   arg2: [0x0 ~> 0x3ffffff]
 *   arg3: [0x0 ~> 0x3ffffff]
 * Output Bounds:
 *   out1: [0x0 ~> 0x3ffffff]
 *   out2: [0x0 ~> 0x1]
 */
static void fiat_25519_addcarryx_u26(uint32_t* out1, fiat_25519_uint1* out2, fiat_25519_uint1 arg1, uint32_t arg2, uint32_t arg3) {
  uint32_t x1 = ((arg1 + arg2) + arg3);
  uint32_t x2 = (x1 & UINT32_C(0x3ffffff));
  fiat_25519_uint1 x3 = (fiat_25519_uint1)(x1 >> 26);
  *out1 = x2;
  *out2 = x3;
}

/*
 * Input Bounds:
 *   arg1: [0x0 ~> 0x1]
 *   arg2: [0x0 ~> 0x3ffffff]
 *   arg3: [0x0 ~> 0x3ffffff]
 * Output Bounds:
 *   out1: [0x0 ~> 0x3ffffff]
 *   out2: [0x0 ~> 0x1]
 */
static void fiat_25519_subborrowx_u26(uint32_t* out1, fiat_25519_uint1* out2, fiat_25519_uint1 arg1, uint32_t arg2, uint32_t arg3) {
  int32_t x1 = ((int32_t)(arg2 - arg1) - (int32_t)arg3);
  fiat_25519_int1 x2 = (fiat_25519_int1)(x1 >> 26);
  uint32_t x3 = (x1 & UINT32_C(0x3ffffff));
  *out1 = x3;
  *out2 = (fiat_25519_uint1)(0x0 - x2);
}

/*
 * Input Bounds:
 *   arg1: [0x0 ~> 0x1]
 *   arg2: [0x0 ~> 0x1ffffff]
 *   arg3: [0x0 ~> 0x1ffffff]
 * Output Bounds:
 *   out1: [0x0 ~> 0x1ffffff]
 *   out2: [0x0 ~> 0x1]
 */
static void fiat_25519_addcarryx_u25(uint32_t* out1, fiat_25519_uint1* out2, fiat_25519_uint1 arg1, uint32_t arg2, uint32_t arg3) {
  uint32_t x1 = ((arg1 + arg2) + arg3);
  uint32_t x2 = (x1 & UINT32_C(0x1ffffff));
  fiat_25519_uint1 x3 = (fiat_25519_uint1)(x1 >> 25);
  *out1 = x2;
  *out2 = x3;
}

/*
 * Input Bounds:
 *   arg1: [0x0 ~> 0x1]
 *   arg2: [0x0 ~> 0x1ffffff]
 *   arg3: [0x0 ~> 0x1ffffff]
 * Output Bounds:
 *   out1: [0x0 ~> 0x1ffffff]
 *   out2: [0x0 ~> 0x1]
 */
static void fiat_25519_subborrowx_u25(uint32_t* out1, fiat_25519_uint1* out2, fiat_25519_uint1 arg1, uint32_t arg2, uint32_t arg3) {
  int32_t x1 = ((int32_t)(arg2 - arg1) - (int32_t)arg3);
  fiat_25519_int1 x2 = (fiat_25519_int1)(x1 >> 25);
  uint32_t x3 = (x1 & UINT32_C(0x1ffffff));
  *out1 = x3;
  *out2 = (fiat_25519_uint1)(0x0 - x2);
}

// value_barrier_u32 returns |a|, but prevents GCC and Clang from reasoning about
// the returned value. This is used to mitigate compilers undoing constant-time
// code, until we can express our requirements directly in the language.
//
// Note the compiler is aware that |value_barrier_u32| has no side effects and
// always has the same output for a given input. This allows it to eliminate
// dead code, move computations across loops, and vectorize.
static inline uint32_t value_barrier_u32(uint32_t a) {
#if !defined(OPENSSL_NO_ASM) && (defined(__GNUC__) || defined(__clang__))
  __asm__("" : "+r"(a) : /* no inputs */);
#endif
  return a;
}

/*
 * Input Bounds:
 *   arg1: [0x0 ~> 0x1]
 *   arg2: [0x0 ~> 0xffffffff]
 *   arg3: [0x0 ~> 0xffffffff]
 * Output Bounds:
 *   out1: [0x0 ~> 0xffffffff]
 */
static void fiat_25519_cmovznz_u32(uint32_t* out1, fiat_25519_uint1 arg1, uint32_t arg2, uint32_t arg3) {
  fiat_25519_uint1 x1 = (!(!arg1));
  uint32_t x2 = ((fiat_25519_int1)(0x0 - x1) & UINT32_C(0xffffffff));
  // Note this line has been patched from the synthesized code to add value
  // barriers.
  //
  // Clang recognizes this pattern as a select. While it usually transforms it
  // to a cmov, it sometimes further transforms it into a branch, which we do
  // not want.
  uint32_t x3 = ((value_barrier_u32(x2) & arg3) | (value_barrier_u32(~x2) & arg2));
  *out1 = x3;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
 *   arg2: [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 */
static void fiat_25519_carry_mul(uint32_t out1[10], const uint32_t arg1[10], const uint32_t arg2[10]) {
  uint64_t x1 = ((uint64_t)(arg1[9]) * ((arg2[9]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x2 = ((uint64_t)(arg1[9]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x3 = ((uint64_t)(arg1[9]) * ((arg2[7]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x4 = ((uint64_t)(arg1[9]) * ((arg2[6]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x5 = ((uint64_t)(arg1[9]) * ((arg2[5]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x6 = ((uint64_t)(arg1[9]) * ((arg2[4]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x7 = ((uint64_t)(arg1[9]) * ((arg2[3]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x8 = ((uint64_t)(arg1[9]) * ((arg2[2]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x9 = ((uint64_t)(arg1[9]) * ((arg2[1]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x10 = ((uint64_t)(arg1[8]) * ((arg2[9]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x11 = ((uint64_t)(arg1[8]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x12 = ((uint64_t)(arg1[8]) * ((arg2[7]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x13 = ((uint64_t)(arg1[8]) * ((arg2[6]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x14 = ((uint64_t)(arg1[8]) * ((arg2[5]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x15 = ((uint64_t)(arg1[8]) * ((arg2[4]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x16 = ((uint64_t)(arg1[8]) * ((arg2[3]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x17 = ((uint64_t)(arg1[8]) * ((arg2[2]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x18 = ((uint64_t)(arg1[7]) * ((arg2[9]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x19 = ((uint64_t)(arg1[7]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x20 = ((uint64_t)(arg1[7]) * ((arg2[7]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x21 = ((uint64_t)(arg1[7]) * ((arg2[6]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x22 = ((uint64_t)(arg1[7]) * ((arg2[5]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x23 = ((uint64_t)(arg1[7]) * ((arg2[4]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x24 = ((uint64_t)(arg1[7]) * ((arg2[3]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x25 = ((uint64_t)(arg1[6]) * ((arg2[9]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x26 = ((uint64_t)(arg1[6]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x27 = ((uint64_t)(arg1[6]) * ((arg2[7]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x28 = ((uint64_t)(arg1[6]) * ((arg2[6]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x29 = ((uint64_t)(arg1[6]) * ((arg2[5]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x30 = ((uint64_t)(arg1[6]) * ((arg2[4]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x31 = ((uint64_t)(arg1[5]) * ((arg2[9]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x32 = ((uint64_t)(arg1[5]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x33 = ((uint64_t)(arg1[5]) * ((arg2[7]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x34 = ((uint64_t)(arg1[5]) * ((arg2[6]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x35 = ((uint64_t)(arg1[5]) * ((arg2[5]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x36 = ((uint64_t)(arg1[4]) * ((arg2[9]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x37 = ((uint64_t)(arg1[4]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x38 = ((uint64_t)(arg1[4]) * ((arg2[7]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x39 = ((uint64_t)(arg1[4]) * ((arg2[6]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x40 = ((uint64_t)(arg1[3]) * ((arg2[9]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x41 = ((uint64_t)(arg1[3]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x42 = ((uint64_t)(arg1[3]) * ((arg2[7]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x43 = ((uint64_t)(arg1[2]) * ((arg2[9]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x44 = ((uint64_t)(arg1[2]) * ((arg2[8]) * (uint32_t)UINT8_C(0x13)));
  uint64_t x45 = ((uint64_t)(arg1[1]) * ((arg2[9]) * ((uint32_t)0x2 * UINT8_C(0x13))));
  uint64_t x46 = ((uint64_t)(arg1[9]) * (arg2[0]));
  uint64_t x47 = ((uint64_t)(arg1[8]) * (arg2[1]));
  uint64_t x48 = ((uint64_t)(arg1[8]) * (arg2[0]));
  uint64_t x49 = ((uint64_t)(arg1[7]) * (arg2[2]));
  uint64_t x50 = ((uint64_t)(arg1[7]) * ((arg2[1]) * (uint32_t)0x2));
  uint64_t x51 = ((uint64_t)(arg1[7]) * (arg2[0]));
  uint64_t x52 = ((uint64_t)(arg1[6]) * (arg2[3]));
  uint64_t x53 = ((uint64_t)(arg1[6]) * (arg2[2]));
  uint64_t x54 = ((uint64_t)(arg1[6]) * (arg2[1]));
  uint64_t x55 = ((uint64_t)(arg1[6]) * (arg2[0]));
  uint64_t x56 = ((uint64_t)(arg1[5]) * (arg2[4]));
  uint64_t x57 = ((uint64_t)(arg1[5]) * ((arg2[3]) * (uint32_t)0x2));
  uint64_t x58 = ((uint64_t)(arg1[5]) * (arg2[2]));
  uint64_t x59 = ((uint64_t)(arg1[5]) * ((arg2[1]) * (uint32_t)0x2));
  uint64_t x60 = ((uint64_t)(arg1[5]) * (arg2[0]));
  uint64_t x61 = ((uint64_t)(arg1[4]) * (arg2[5]));
  uint64_t x62 = ((uint64_t)(arg1[4]) * (arg2[4]));
  uint64_t x63 = ((uint64_t)(arg1[4]) * (arg2[3]));
  uint64_t x64 = ((uint64_t)(arg1[4]) * (arg2[2]));
  uint64_t x65 = ((uint64_t)(arg1[4]) * (arg2[1]));
  uint64_t x66 = ((uint64_t)(arg1[4]) * (arg2[0]));
  uint64_t x67 = ((uint64_t)(arg1[3]) * (arg2[6]));
  uint64_t x68 = ((uint64_t)(arg1[3]) * ((arg2[5]) * (uint32_t)0x2));
  uint64_t x69 = ((uint64_t)(arg1[3]) * (arg2[4]));
  uint64_t x70 = ((uint64_t)(arg1[3]) * ((arg2[3]) * (uint32_t)0x2));
  uint64_t x71 = ((uint64_t)(arg1[3]) * (arg2[2]));
  uint64_t x72 = ((uint64_t)(arg1[3]) * ((arg2[1]) * (uint32_t)0x2));
  uint64_t x73 = ((uint64_t)(arg1[3]) * (arg2[0]));
  uint64_t x74 = ((uint64_t)(arg1[2]) * (arg2[7]));
  uint64_t x75 = ((uint64_t)(arg1[2]) * (arg2[6]));
  uint64_t x76 = ((uint64_t)(arg1[2]) * (arg2[5]));
  uint64_t x77 = ((uint64_t)(arg1[2]) * (arg2[4]));
  uint64_t x78 = ((uint64_t)(arg1[2]) * (arg2[3]));
  uint64_t x79 = ((uint64_t)(arg1[2]) * (arg2[2]));
  uint64_t x80 = ((uint64_t)(arg1[2]) * (arg2[1]));
  uint64_t x81 = ((uint64_t)(arg1[2]) * (arg2[0]));
  uint64_t x82 = ((uint64_t)(arg1[1]) * (arg2[8]));
  uint64_t x83 = ((uint64_t)(arg1[1]) * ((arg2[7]) * (uint32_t)0x2));
  uint64_t x84 = ((uint64_t)(arg1[1]) * (arg2[6]));
  uint64_t x85 = ((uint64_t)(arg1[1]) * ((arg2[5]) * (uint32_t)0x2));
  uint64_t x86 = ((uint64_t)(arg1[1]) * (arg2[4]));
  uint64_t x87 = ((uint64_t)(arg1[1]) * ((arg2[3]) * (uint32_t)0x2));
  uint64_t x88 = ((uint64_t)(arg1[1]) * (arg2[2]));
  uint64_t x89 = ((uint64_t)(arg1[1]) * ((arg2[1]) * (uint32_t)0x2));
  uint64_t x90 = ((uint64_t)(arg1[1]) * (arg2[0]));
  uint64_t x91 = ((uint64_t)(arg1[0]) * (arg2[9]));
  uint64_t x92 = ((uint64_t)(arg1[0]) * (arg2[8]));
  uint64_t x93 = ((uint64_t)(arg1[0]) * (arg2[7]));
  uint64_t x94 = ((uint64_t)(arg1[0]) * (arg2[6]));
  uint64_t x95 = ((uint64_t)(arg1[0]) * (arg2[5]));
  uint64_t x96 = ((uint64_t)(arg1[0]) * (arg2[4]));
  uint64_t x97 = ((uint64_t)(arg1[0]) * (arg2[3]));
  uint64_t x98 = ((uint64_t)(arg1[0]) * (arg2[2]));
  uint64_t x99 = ((uint64_t)(arg1[0]) * (arg2[1]));
  uint64_t x100 = ((uint64_t)(arg1[0]) * (arg2[0]));
  uint64_t x101 = (x100 + (x45 + (x44 + (x42 + (x39 + (x35 + (x30 + (x24 + (x17 + x9)))))))));
  uint64_t x102 = (x101 >> 26);
  uint32_t x103 = (uint32_t)(x101 & UINT32_C(0x3ffffff));
  uint64_t x104 = (x91 + (x82 + (x74 + (x67 + (x61 + (x56 + (x52 + (x49 + (x47 + x46)))))))));
  uint64_t x105 = (x92 + (x83 + (x75 + (x68 + (x62 + (x57 + (x53 + (x50 + (x48 + x1)))))))));
  uint64_t x106 = (x93 + (x84 + (x76 + (x69 + (x63 + (x58 + (x54 + (x51 + (x10 + x2)))))))));
  uint64_t x107 = (x94 + (x85 + (x77 + (x70 + (x64 + (x59 + (x55 + (x18 + (x11 + x3)))))))));
  uint64_t x108 = (x95 + (x86 + (x78 + (x71 + (x65 + (x60 + (x25 + (x19 + (x12 + x4)))))))));
  uint64_t x109 = (x96 + (x87 + (x79 + (x72 + (x66 + (x31 + (x26 + (x20 + (x13 + x5)))))))));
  uint64_t x110 = (x97 + (x88 + (x80 + (x73 + (x36 + (x32 + (x27 + (x21 + (x14 + x6)))))))));
  uint64_t x111 = (x98 + (x89 + (x81 + (x40 + (x37 + (x33 + (x28 + (x22 + (x15 + x7)))))))));
  uint64_t x112 = (x99 + (x90 + (x43 + (x41 + (x38 + (x34 + (x29 + (x23 + (x16 + x8)))))))));
  uint64_t x113 = (x102 + x112);
  uint64_t x114 = (x113 >> 25);
  uint32_t x115 = (uint32_t)(x113 & UINT32_C(0x1ffffff));
  uint64_t x116 = (x114 + x111);
  uint64_t x117 = (x116 >> 26);
  uint32_t x118 = (uint32_t)(x116 & UINT32_C(0x3ffffff));
  uint64_t x119 = (x117 + x110);
  uint64_t x120 = (x119 >> 25);
  uint32_t x121 = (uint32_t)(x119 & UINT32_C(0x1ffffff));
  uint64_t x122 = (x120 + x109);
  uint64_t x123 = (x122 >> 26);
  uint32_t x124 = (uint32_t)(x122 & UINT32_C(0x3ffffff));
  uint64_t x125 = (x123 + x108);
  uint64_t x126 = (x125 >> 25);
  uint32_t x127 = (uint32_t)(x125 & UINT32_C(0x1ffffff));
  uint64_t x128 = (x126 + x107);
  uint64_t x129 = (x128 >> 26);
  uint32_t x130 = (uint32_t)(x128 & UINT32_C(0x3ffffff));
  uint64_t x131 = (x129 + x106);
  uint64_t x132 = (x131 >> 25);
  uint32_t x133 = (uint32_t)(x131 & UINT32_C(0x1ffffff));
  uint64_t x134 = (x132 + x105);
  uint64_t x135 = (x134 >> 26);
  uint32_t x136 = (uint32_t)(x134 & UINT32_C(0x3ffffff));
  uint64_t x137 = (x135 + x104);
  uint64_t x138 = (x137 >> 25);
  uint32_t x139 = (uint32_t)(x137 & UINT32_C(0x1ffffff));
  uint64_t x140 = (x138 * (uint64_t)UINT8_C(0x13));
  uint64_t x141 = (x103 + x140);
  uint32_t x142 = (uint32_t)(x141 >> 26);
  uint32_t x143 = (uint32_t)(x141 & UINT32_C(0x3ffffff));
  uint32_t x144 = (x142 + x115);
  uint32_t x145 = (x144 >> 25);
  uint32_t x146 = (x144 & UINT32_C(0x1ffffff));
  uint32_t x147 = (x145 + x118);
  out1[0] = x143;
  out1[1] = x146;
  out1[2] = x147;
  out1[3] = x121;
  out1[4] = x124;
  out1[5] = x127;
  out1[6] = x130;
  out1[7] = x133;
  out1[8] = x136;
  out1[9] = x139;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 */
static void fiat_25519_carry_square(uint32_t out1[10], const uint32_t arg1[10]) {
  uint32_t x1 = ((arg1[9]) * (uint32_t)UINT8_C(0x13));
  uint32_t x2 = (x1 * (uint32_t)0x2);
  uint32_t x3 = ((arg1[9]) * (uint32_t)0x2);
  uint32_t x4 = ((arg1[8]) * (uint32_t)UINT8_C(0x13));
  uint64_t x5 = (x4 * (uint64_t)0x2);
  uint32_t x6 = ((arg1[8]) * (uint32_t)0x2);
  uint32_t x7 = ((arg1[7]) * (uint32_t)UINT8_C(0x13));
  uint32_t x8 = (x7 * (uint32_t)0x2);
  uint32_t x9 = ((arg1[7]) * (uint32_t)0x2);
  uint32_t x10 = ((arg1[6]) * (uint32_t)UINT8_C(0x13));
  uint64_t x11 = (x10 * (uint64_t)0x2);
  uint32_t x12 = ((arg1[6]) * (uint32_t)0x2);
  uint32_t x13 = ((arg1[5]) * (uint32_t)UINT8_C(0x13));
  uint32_t x14 = ((arg1[5]) * (uint32_t)0x2);
  uint32_t x15 = ((arg1[4]) * (uint32_t)0x2);
  uint32_t x16 = ((arg1[3]) * (uint32_t)0x2);
  uint32_t x17 = ((arg1[2]) * (uint32_t)0x2);
  uint32_t x18 = ((arg1[1]) * (uint32_t)0x2);
  uint64_t x19 = ((uint64_t)(arg1[9]) * (x1 * (uint32_t)0x2));
  uint64_t x20 = ((uint64_t)(arg1[8]) * x2);
  uint64_t x21 = ((uint64_t)(arg1[8]) * x4);
  uint64_t x22 = ((arg1[7]) * (x2 * (uint64_t)0x2));
  uint64_t x23 = ((arg1[7]) * x5);
  uint64_t x24 = ((uint64_t)(arg1[7]) * (x7 * (uint32_t)0x2));
  uint64_t x25 = ((uint64_t)(arg1[6]) * x2);
  uint64_t x26 = ((arg1[6]) * x5);
  uint64_t x27 = ((uint64_t)(arg1[6]) * x8);
  uint64_t x28 = ((uint64_t)(arg1[6]) * x10);
  uint64_t x29 = ((arg1[5]) * (x2 * (uint64_t)0x2));
  uint64_t x30 = ((arg1[5]) * x5);
  uint64_t x31 = ((arg1[5]) * (x8 * (uint64_t)0x2));
  uint64_t x32 = ((arg1[5]) * x11);
  uint64_t x33 = ((uint64_t)(arg1[5]) * (x13 * (uint32_t)0x2));
  uint64_t x34 = ((uint64_t)(arg1[4]) * x2);
  uint64_t x35 = ((arg1[4]) * x5);
  uint64_t x36 = ((uint64_t)(arg1[4]) * x8);
  uint64_t x37 = ((arg1[4]) * x11);
  uint64_t x38 = ((uint64_t)(arg1[4]) * x14);
  uint64_t x39 = ((uint64_t)(arg1[4]) * (arg1[4]));
  uint64_t x40 = ((arg1[3]) * (x2 * (uint64_t)0x2));
  uint64_t x41 = ((arg1[3]) * x5);
  uint64_t x42 = ((arg1[3]) * (x8 * (uint64_t)0x2));
  uint64_t x43 = ((uint64_t)(arg1[3]) * x12);
  uint64_t x44 = ((uint64_t)(arg1[3]) * (x14 * (uint32_t)0x2));
  uint64_t x45 = ((uint64_t)(arg1[3]) * x15);
  uint64_t x46 = ((uint64_t)(arg1[3]) * ((arg1[3]) * (uint32_t)0x2));
  uint64_t x47 = ((uint64_t)(arg1[2]) * x2);
  uint64_t x48 = ((arg1[2]) * x5);
  uint64_t x49 = ((uint64_t)(arg1[2]) * x9);
  uint64_t x50 = ((uint64_t)(arg1[2]) * x12);
  uint64_t x51 = ((uint64_t)(arg1[2]) * x14);
  uint64_t x52 = ((uint64_t)(arg1[2]) * x15);
  uint64_t x53 = ((uint64_t)(arg1[2]) * x16);
  uint64_t x54 = ((uint64_t)(arg1[2]) * (arg1[2]));
  uint64_t x55 = ((arg1[1]) * (x2 * (uint64_t)0x2));
  uint64_t x56 = ((uint64_t)(arg1[1]) * x6);
  uint64_t x57 = ((uint64_t)(arg1[1]) * (x9 * (uint32_t)0x2));
  uint64_t x58 = ((uint64_t)(arg1[1]) * x12);
  uint64_t x59 = ((uint64_t)(arg1[1]) * (x14 * (uint32_t)0x2));
  uint64_t x60 = ((uint64_t)(arg1[1]) * x15);
  uint64_t x61 = ((uint64_t)(arg1[1]) * (x16 * (uint32_t)0x2));
  uint64_t x62 = ((uint64_t)(arg1[1]) * x17);
  uint64_t x63 = ((uint64_t)(arg1[1]) * ((arg1[1]) * (uint32_t)0x2));
  uint64_t x64 = ((uint64_t)(arg1[0]) * x3);
  uint64_t x65 = ((uint64_t)(arg1[0]) * x6);
  uint64_t x66 = ((uint64_t)(arg1[0]) * x9);
  uint64_t x67 = ((uint64_t)(arg1[0]) * x12);
  uint64_t x68 = ((uint64_t)(arg1[0]) * x14);
  uint64_t x69 = ((uint64_t)(arg1[0]) * x15);
  uint64_t x70 = ((uint64_t)(arg1[0]) * x16);
  uint64_t x71 = ((uint64_t)(arg1[0]) * x17);
  uint64_t x72 = ((uint64_t)(arg1[0]) * x18);
  uint64_t x73 = ((uint64_t)(arg1[0]) * (arg1[0]));
  uint64_t x74 = (x73 + (x55 + (x48 + (x42 + (x37 + x33)))));
  uint64_t x75 = (x74 >> 26);
  uint32_t x76 = (uint32_t)(x74 & UINT32_C(0x3ffffff));
  uint64_t x77 = (x64 + (x56 + (x49 + (x43 + x38))));
  uint64_t x78 = (x65 + (x57 + (x50 + (x44 + (x39 + x19)))));
  uint64_t x79 = (x66 + (x58 + (x51 + (x45 + x20))));
  uint64_t x80 = (x67 + (x59 + (x52 + (x46 + (x22 + x21)))));
  uint64_t x81 = (x68 + (x60 + (x53 + (x25 + x23))));
  uint64_t x82 = (x69 + (x61 + (x54 + (x29 + (x26 + x24)))));
  uint64_t x83 = (x70 + (x62 + (x34 + (x30 + x27))));
  uint64_t x84 = (x71 + (x63 + (x40 + (x35 + (x31 + x28)))));
  uint64_t x85 = (x72 + (x47 + (x41 + (x36 + x32))));
  uint64_t x86 = (x75 + x85);
  uint64_t x87 = (x86 >> 25);
  uint32_t x88 = (uint32_t)(x86 & UINT32_C(0x1ffffff));
  uint64_t x89 = (x87 + x84);
  uint64_t x90 = (x89 >> 26);
  uint32_t x91 = (uint32_t)(x89 & UINT32_C(0x3ffffff));
  uint64_t x92 = (x90 + x83);
  uint64_t x93 = (x92 >> 25);
  uint32_t x94 = (uint32_t)(x92 & UINT32_C(0x1ffffff));
  uint64_t x95 = (x93 + x82);
  uint64_t x96 = (x95 >> 26);
  uint32_t x97 = (uint32_t)(x95 & UINT32_C(0x3ffffff));
  uint64_t x98 = (x96 + x81);
  uint64_t x99 = (x98 >> 25);
  uint32_t x100 = (uint32_t)(x98 & UINT32_C(0x1ffffff));
  uint64_t x101 = (x99 + x80);
  uint64_t x102 = (x101 >> 26);
  uint32_t x103 = (uint32_t)(x101 & UINT32_C(0x3ffffff));
  uint64_t x104 = (x102 + x79);
  uint64_t x105 = (x104 >> 25);
  uint32_t x106 = (uint32_t)(x104 & UINT32_C(0x1ffffff));
  uint64_t x107 = (x105 + x78);
  uint64_t x108 = (x107 >> 26);
  uint32_t x109 = (uint32_t)(x107 & UINT32_C(0x3ffffff));
  uint64_t x110 = (x108 + x77);
  uint64_t x111 = (x110 >> 25);
  uint32_t x112 = (uint32_t)(x110 & UINT32_C(0x1ffffff));
  uint64_t x113 = (x111 * (uint64_t)UINT8_C(0x13));
  uint64_t x114 = (x76 + x113);
  uint32_t x115 = (uint32_t)(x114 >> 26);
  uint32_t x116 = (uint32_t)(x114 & UINT32_C(0x3ffffff));
  uint32_t x117 = (x115 + x88);
  uint32_t x118 = (x117 >> 25);
  uint32_t x119 = (x117 & UINT32_C(0x1ffffff));
  uint32_t x120 = (x118 + x91);
  out1[0] = x116;
  out1[1] = x119;
  out1[2] = x120;
  out1[3] = x94;
  out1[4] = x97;
  out1[5] = x100;
  out1[6] = x103;
  out1[7] = x106;
  out1[8] = x109;
  out1[9] = x112;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 */
static void fiat_25519_carry(uint32_t out1[10], const uint32_t arg1[10]) {
  uint32_t x1 = (arg1[0]);
  uint32_t x2 = ((x1 >> 26) + (arg1[1]));
  uint32_t x3 = ((x2 >> 25) + (arg1[2]));
  uint32_t x4 = ((x3 >> 26) + (arg1[3]));
  uint32_t x5 = ((x4 >> 25) + (arg1[4]));
  uint32_t x6 = ((x5 >> 26) + (arg1[5]));
  uint32_t x7 = ((x6 >> 25) + (arg1[6]));
  uint32_t x8 = ((x7 >> 26) + (arg1[7]));
  uint32_t x9 = ((x8 >> 25) + (arg1[8]));
  uint32_t x10 = ((x9 >> 26) + (arg1[9]));
  uint32_t x11 = ((x1 & UINT32_C(0x3ffffff)) + ((x10 >> 25) * (uint32_t)UINT8_C(0x13)));
  uint32_t x12 = ((x11 >> 26) + (x2 & UINT32_C(0x1ffffff)));
  uint32_t x13 = (x11 & UINT32_C(0x3ffffff));
  uint32_t x14 = (x12 & UINT32_C(0x1ffffff));
  uint32_t x15 = ((x12 >> 25) + (x3 & UINT32_C(0x3ffffff)));
  uint32_t x16 = (x4 & UINT32_C(0x1ffffff));
  uint32_t x17 = (x5 & UINT32_C(0x3ffffff));
  uint32_t x18 = (x6 & UINT32_C(0x1ffffff));
  uint32_t x19 = (x7 & UINT32_C(0x3ffffff));
  uint32_t x20 = (x8 & UINT32_C(0x1ffffff));
  uint32_t x21 = (x9 & UINT32_C(0x3ffffff));
  uint32_t x22 = (x10 & UINT32_C(0x1ffffff));
  out1[0] = x13;
  out1[1] = x14;
  out1[2] = x15;
  out1[3] = x16;
  out1[4] = x17;
  out1[5] = x18;
  out1[6] = x19;
  out1[7] = x20;
  out1[8] = x21;
  out1[9] = x22;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 *   arg2: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
 */
static void fiat_25519_add(uint32_t out1[10], const uint32_t arg1[10], const uint32_t arg2[10]) {
  uint32_t x1 = ((arg1[0]) + (arg2[0]));
  uint32_t x2 = ((arg1[1]) + (arg2[1]));
  uint32_t x3 = ((arg1[2]) + (arg2[2]));
  uint32_t x4 = ((arg1[3]) + (arg2[3]));
  uint32_t x5 = ((arg1[4]) + (arg2[4]));
  uint32_t x6 = ((arg1[5]) + (arg2[5]));
  uint32_t x7 = ((arg1[6]) + (arg2[6]));
  uint32_t x8 = ((arg1[7]) + (arg2[7]));
  uint32_t x9 = ((arg1[8]) + (arg2[8]));
  uint32_t x10 = ((arg1[9]) + (arg2[9]));
  out1[0] = x1;
  out1[1] = x2;
  out1[2] = x3;
  out1[3] = x4;
  out1[4] = x5;
  out1[5] = x6;
  out1[6] = x7;
  out1[7] = x8;
  out1[8] = x9;
  out1[9] = x10;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 *   arg2: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
 */
static void fiat_25519_sub(uint32_t out1[10], const uint32_t arg1[10], const uint32_t arg2[10]) {
  uint32_t x1 = ((UINT32_C(0x7ffffda) + (arg1[0])) - (arg2[0]));
  uint32_t x2 = ((UINT32_C(0x3fffffe) + (arg1[1])) - (arg2[1]));
  uint32_t x3 = ((UINT32_C(0x7fffffe) + (arg1[2])) - (arg2[2]));
  uint32_t x4 = ((UINT32_C(0x3fffffe) + (arg1[3])) - (arg2[3]));
  uint32_t x5 = ((UINT32_C(0x7fffffe) + (arg1[4])) - (arg2[4]));
  uint32_t x6 = ((UINT32_C(0x3fffffe) + (arg1[5])) - (arg2[5]));
  uint32_t x7 = ((UINT32_C(0x7fffffe) + (arg1[6])) - (arg2[6]));
  uint32_t x8 = ((UINT32_C(0x3fffffe) + (arg1[7])) - (arg2[7]));
  uint32_t x9 = ((UINT32_C(0x7fffffe) + (arg1[8])) - (arg2[8]));
  uint32_t x10 = ((UINT32_C(0x3fffffe) + (arg1[9])) - (arg2[9]));
  out1[0] = x1;
  out1[1] = x2;
  out1[2] = x3;
  out1[3] = x4;
  out1[4] = x5;
  out1[5] = x6;
  out1[6] = x7;
  out1[7] = x8;
  out1[8] = x9;
  out1[9] = x10;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999], [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
 */
static void fiat_25519_opp(uint32_t out1[10], const uint32_t arg1[10]) {
  uint32_t x1 = (UINT32_C(0x7ffffda) - (arg1[0]));
  uint32_t x2 = (UINT32_C(0x3fffffe) - (arg1[1]));
  uint32_t x3 = (UINT32_C(0x7fffffe) - (arg1[2]));
  uint32_t x4 = (UINT32_C(0x3fffffe) - (arg1[3]));
  uint32_t x5 = (UINT32_C(0x7fffffe) - (arg1[4]));
  uint32_t x6 = (UINT32_C(0x3fffffe) - (arg1[5]));
  uint32_t x7 = (UINT32_C(0x7fffffe) - (arg1[6]));
  uint32_t x8 = (UINT32_C(0x3fffffe) - (arg1[7]));
  uint32_t x9 = (UINT32_C(0x7fffffe) - (arg1[8]));
  uint32_t x10 = (UINT32_C(0x3fffffe) - (arg1[9]));
  out1[0] = x1;
  out1[1] = x2;
  out1[2] = x3;
  out1[3] = x4;
  out1[4] = x5;
  out1[5] = x6;
  out1[6] = x7;
  out1[7] = x8;
  out1[8] = x9;
  out1[9] = x10;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0x7f]]
 */
static void fiat_25519_to_bytes(uint8_t out1[32], const uint32_t arg1[10]) {
  uint32_t x1;
  fiat_25519_uint1 x2;
  fiat_25519_subborrowx_u26(&x1, &x2, 0x0, (arg1[0]), UINT32_C(0x3ffffed));
  uint32_t x3;
  fiat_25519_uint1 x4;
  fiat_25519_subborrowx_u25(&x3, &x4, x2, (arg1[1]), UINT32_C(0x1ffffff));
  uint32_t x5;
  fiat_25519_uint1 x6;
  fiat_25519_subborrowx_u26(&x5, &x6, x4, (arg1[2]), UINT32_C(0x3ffffff));
  uint32_t x7;
  fiat_25519_uint1 x8;
  fiat_25519_subborrowx_u25(&x7, &x8, x6, (arg1[3]), UINT32_C(0x1ffffff));
  uint32_t x9;
  fiat_25519_uint1 x10;
  fiat_25519_subborrowx_u26(&x9, &x10, x8, (arg1[4]), UINT32_C(0x3ffffff));
  uint32_t x11;
  fiat_25519_uint1 x12;
  fiat_25519_subborrowx_u25(&x11, &x12, x10, (arg1[5]), UINT32_C(0x1ffffff));
  uint32_t x13;
  fiat_25519_uint1 x14;
  fiat_25519_subborrowx_u26(&x13, &x14, x12, (arg1[6]), UINT32_C(0x3ffffff));
  uint32_t x15;
  fiat_25519_uint1 x16;
  fiat_25519_subborrowx_u25(&x15, &x16, x14, (arg1[7]), UINT32_C(0x1ffffff));
  uint32_t x17;
  fiat_25519_uint1 x18;
  fiat_25519_subborrowx_u26(&x17, &x18, x16, (arg1[8]), UINT32_C(0x3ffffff));
  uint32_t x19;
  fiat_25519_uint1 x20;
  fiat_25519_subborrowx_u25(&x19, &x20, x18, (arg1[9]), UINT32_C(0x1ffffff));
  uint32_t x21;
  fiat_25519_cmovznz_u32(&x21, x20, 0x0, UINT32_C(0xffffffff));
  uint32_t x22;
  fiat_25519_uint1 x23;
  fiat_25519_addcarryx_u26(&x22, &x23, 0x0, (x21 & UINT32_C(0x3ffffed)), x1);
  uint32_t x24;
  fiat_25519_uint1 x25;
  fiat_25519_addcarryx_u25(&x24, &x25, x23, (x21 & UINT32_C(0x1ffffff)), x3);
  uint32_t x26;
  fiat_25519_uint1 x27;
  fiat_25519_addcarryx_u26(&x26, &x27, x25, (x21 & UINT32_C(0x3ffffff)), x5);
  uint32_t x28;
  fiat_25519_uint1 x29;
  fiat_25519_addcarryx_u25(&x28, &x29, x27, (x21 & UINT32_C(0x1ffffff)), x7);
  uint32_t x30;
  fiat_25519_uint1 x31;
  fiat_25519_addcarryx_u26(&x30, &x31, x29, (x21 & UINT32_C(0x3ffffff)), x9);
  uint32_t x32;
  fiat_25519_uint1 x33;
  fiat_25519_addcarryx_u25(&x32, &x33, x31, (x21 & UINT32_C(0x1ffffff)), x11);
  uint32_t x34;
  fiat_25519_uint1 x35;
  fiat_25519_addcarryx_u26(&x34, &x35, x33, (x21 & UINT32_C(0x3ffffff)), x13);
  uint32_t x36;
  fiat_25519_uint1 x37;
  fiat_25519_addcarryx_u25(&x36, &x37, x35, (x21 & UINT32_C(0x1ffffff)), x15);
  uint32_t x38;
  fiat_25519_uint1 x39;
  fiat_25519_addcarryx_u26(&x38, &x39, x37, (x21 & UINT32_C(0x3ffffff)), x17);
  uint32_t x40;
  fiat_25519_uint1 x41;
  fiat_25519_addcarryx_u25(&x40, &x41, x39, (x21 & UINT32_C(0x1ffffff)), x19);
  uint32_t x42 = (x40 << 6);
  uint32_t x43 = (x38 << 4);
  uint32_t x44 = (x36 << 3);
  uint32_t x45 = (x34 * (uint32_t)0x2);
  uint32_t x46 = (x30 << 6);
  uint32_t x47 = (x28 << 5);
  uint32_t x48 = (x26 << 3);
  uint32_t x49 = (x24 << 2);
  uint32_t x50 = (x22 >> 8);
  uint8_t x51 = (uint8_t)(x22 & UINT8_C(0xff));
  uint32_t x52 = (x50 >> 8);
  uint8_t x53 = (uint8_t)(x50 & UINT8_C(0xff));
  uint8_t x54 = (uint8_t)(x52 >> 8);
  uint8_t x55 = (uint8_t)(x52 & UINT8_C(0xff));
  uint32_t x56 = (x54 + x49);
  uint32_t x57 = (x56 >> 8);
  uint8_t x58 = (uint8_t)(x56 & UINT8_C(0xff));
  uint32_t x59 = (x57 >> 8);
  uint8_t x60 = (uint8_t)(x57 & UINT8_C(0xff));
  uint8_t x61 = (uint8_t)(x59 >> 8);
  uint8_t x62 = (uint8_t)(x59 & UINT8_C(0xff));
  uint32_t x63 = (x61 + x48);
  uint32_t x64 = (x63 >> 8);
  uint8_t x65 = (uint8_t)(x63 & UINT8_C(0xff));
  uint32_t x66 = (x64 >> 8);
  uint8_t x67 = (uint8_t)(x64 & UINT8_C(0xff));
  uint8_t x68 = (uint8_t)(x66 >> 8);
  uint8_t x69 = (uint8_t)(x66 & UINT8_C(0xff));
  uint32_t x70 = (x68 + x47);
  uint32_t x71 = (x70 >> 8);
  uint8_t x72 = (uint8_t)(x70 & UINT8_C(0xff));
  uint32_t x73 = (x71 >> 8);
  uint8_t x74 = (uint8_t)(x71 & UINT8_C(0xff));
  uint8_t x75 = (uint8_t)(x73 >> 8);
  uint8_t x76 = (uint8_t)(x73 & UINT8_C(0xff));
  uint32_t x77 = (x75 + x46);
  uint32_t x78 = (x77 >> 8);
  uint8_t x79 = (uint8_t)(x77 & UINT8_C(0xff));
  uint32_t x80 = (x78 >> 8);
  uint8_t x81 = (uint8_t)(x78 & UINT8_C(0xff));
  uint8_t x82 = (uint8_t)(x80 >> 8);
  uint8_t x83 = (uint8_t)(x80 & UINT8_C(0xff));
  uint8_t x84 = (uint8_t)(x82 & UINT8_C(0xff));
  uint32_t x85 = (x32 >> 8);
  uint8_t x86 = (uint8_t)(x32 & UINT8_C(0xff));
  uint32_t x87 = (x85 >> 8);
  uint8_t x88 = (uint8_t)(x85 & UINT8_C(0xff));
  fiat_25519_uint1 x89 = (fiat_25519_uint1)(x87 >> 8);
  uint8_t x90 = (uint8_t)(x87 & UINT8_C(0xff));
  uint32_t x91 = (x89 + x45);
  uint32_t x92 = (x91 >> 8);
  uint8_t x93 = (uint8_t)(x91 & UINT8_C(0xff));
  uint32_t x94 = (x92 >> 8);
  uint8_t x95 = (uint8_t)(x92 & UINT8_C(0xff));
  uint8_t x96 = (uint8_t)(x94 >> 8);
  uint8_t x97 = (uint8_t)(x94 & UINT8_C(0xff));
  uint32_t x98 = (x96 + x44);
  uint32_t x99 = (x98 >> 8);
  uint8_t x100 = (uint8_t)(x98 & UINT8_C(0xff));
  uint32_t x101 = (x99 >> 8);
  uint8_t x102 = (uint8_t)(x99 & UINT8_C(0xff));
  uint8_t x103 = (uint8_t)(x101 >> 8);
  uint8_t x104 = (uint8_t)(x101 & UINT8_C(0xff));
  uint32_t x105 = (x103 + x43);
  uint32_t x106 = (x105 >> 8);
  uint8_t x107 = (uint8_t)(x105 & UINT8_C(0xff));
  uint32_t x108 = (x106 >> 8);
  uint8_t x109 = (uint8_t)(x106 & UINT8_C(0xff));
  uint8_t x110 = (uint8_t)(x108 >> 8);
  uint8_t x111 = (uint8_t)(x108 & UINT8_C(0xff));
  uint32_t x112 = (x110 + x42);
  uint32_t x113 = (x112 >> 8);
  uint8_t x114 = (uint8_t)(x112 & UINT8_C(0xff));
  uint32_t x115 = (x113 >> 8);
  uint8_t x116 = (uint8_t)(x113 & UINT8_C(0xff));
  uint8_t x117 = (uint8_t)(x115 >> 8);
  uint8_t x118 = (uint8_t)(x115 & UINT8_C(0xff));
  out1[0] = x51;
  out1[1] = x53;
  out1[2] = x55;
  out1[3] = x58;
  out1[4] = x60;
  out1[5] = x62;
  out1[6] = x65;
  out1[7] = x67;
  out1[8] = x69;
  out1[9] = x72;
  out1[10] = x74;
  out1[11] = x76;
  out1[12] = x79;
  out1[13] = x81;
  out1[14] = x83;
  out1[15] = x84;
  out1[16] = x86;
  out1[17] = x88;
  out1[18] = x90;
  out1[19] = x93;
  out1[20] = x95;
  out1[21] = x97;
  out1[22] = x100;
  out1[23] = x102;
  out1[24] = x104;
  out1[25] = x107;
  out1[26] = x109;
  out1[27] = x111;
  out1[28] = x114;
  out1[29] = x116;
  out1[30] = x118;
  out1[31] = x117;
}

/*
 * Input Bounds:
 *   arg1: [[0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0xff], [0x0 ~> 0x7f]]
 * Output Bounds:
 *   out1: [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333], [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
 */
static void fiat_25519_from_bytes(uint32_t out1[10], const uint8_t arg1[32]) {
  uint32_t x1 = ((uint32_t)(arg1[31]) << 18);
  uint32_t x2 = ((uint32_t)(arg1[30]) << 10);
  uint32_t x3 = ((uint32_t)(arg1[29]) << 2);
  uint32_t x4 = ((uint32_t)(arg1[28]) << 20);
  uint32_t x5 = ((uint32_t)(arg1[27]) << 12);
  uint32_t x6 = ((uint32_t)(arg1[26]) << 4);
  uint32_t x7 = ((uint32_t)(arg1[25]) << 21);
  uint32_t x8 = ((uint32_t)(arg1[24]) << 13);
  uint32_t x9 = ((uint32_t)(arg1[23]) << 5);
  uint32_t x10 = ((uint32_t)(arg1[22]) << 23);
  uint32_t x11 = ((uint32_t)(arg1[21]) << 15);
  uint32_t x12 = ((uint32_t)(arg1[20]) << 7);
  uint32_t x13 = ((uint32_t)(arg1[19]) << 24);
  uint32_t x14 = ((uint32_t)(arg1[18]) << 16);
  uint32_t x15 = ((uint32_t)(arg1[17]) << 8);
  uint8_t x16 = (arg1[16]);
  uint32_t x17 = ((uint32_t)(arg1[15]) << 18);
  uint32_t x18 = ((uint32_t)(arg1[14]) << 10);
  uint32_t x19 = ((uint32_t)(arg1[13]) << 2);
  uint32_t x20 = ((uint32_t)(arg1[12]) << 19);
  uint32_t x21 = ((uint32_t)(arg1[11]) << 11);
  uint32_t x22 = ((uint32_t)(arg1[10]) << 3);
  uint32_t x23 = ((uint32_t)(arg1[9]) << 21);
  uint32_t x24 = ((uint32_t)(arg1[8]) << 13);
  uint32_t x25 = ((uint32_t)(arg1[7]) << 5);
  uint32_t x26 = ((uint32_t)(arg1[6]) << 22);
  uint32_t x27 = ((uint32_t)(arg1[5]) << 14);
  uint32_t x28 = ((uint32_t)(arg1[4]) << 6);
  uint32_t x29 = ((uint32_t)(arg1[3]) << 24);
  uint32_t x30 = ((uint32_t)(arg1[2]) << 16);
  uint32_t x31 = ((uint32_t)(arg1[1]) << 8);
  uint8_t x32 = (arg1[0]);
  uint32_t x33 = (x32 + (x31 + (x30 + x29)));
  uint8_t x34 = (uint8_t)(x33 >> 26);
  uint32_t x35 = (x33 & UINT32_C(0x3ffffff));
  uint32_t x36 = (x3 + (x2 + x1));
  uint32_t x37 = (x6 + (x5 + x4));
  uint32_t x38 = (x9 + (x8 + x7));
  uint32_t x39 = (x12 + (x11 + x10));
  uint32_t x40 = (x16 + (x15 + (x14 + x13)));
  uint32_t x41 = (x19 + (x18 + x17));
  uint32_t x42 = (x22 + (x21 + x20));
  uint32_t x43 = (x25 + (x24 + x23));
  uint32_t x44 = (x28 + (x27 + x26));
  uint32_t x45 = (x34 + x44);
  uint8_t x46 = (uint8_t)(x45 >> 25);
  uint32_t x47 = (x45 & UINT32_C(0x1ffffff));
  uint32_t x48 = (x46 + x43);
  uint8_t x49 = (uint8_t)(x48 >> 26);
  uint32_t x50 = (x48 & UINT32_C(0x3ffffff));
  uint32_t x51 = (x49 + x42);
  uint8_t x52 = (uint8_t)(x51 >> 25);
  uint32_t x53 = (x51 & UINT32_C(0x1ffffff));
  uint32_t x54 = (x52 + x41);
  uint32_t x55 = (x54 & UINT32_C(0x3ffffff));
  uint8_t x56 = (uint8_t)(x40 >> 25);
  uint32_t x57 = (x40 & UINT32_C(0x1ffffff));
  uint32_t x58 = (x56 + x39);
  uint8_t x59 = (uint8_t)(x58 >> 26);
  uint32_t x60 = (x58 & UINT32_C(0x3ffffff));
  uint32_t x61 = (x59 + x38);
  uint8_t x62 = (uint8_t)(x61 >> 25);
  uint32_t x63 = (x61 & UINT32_C(0x1ffffff));
  uint32_t x64 = (x62 + x37);
  uint8_t x65 = (uint8_t)(x64 >> 26);
  uint32_t x66 = (x64 & UINT32_C(0x3ffffff));
  uint32_t x67 = (x65 + x36);
  out1[0] = x35;
  out1[1] = x47;
  out1[2] = x50;
  out1[3] = x53;
  out1[4] = x55;
  out1[5] = x57;
  out1[6] = x60;
  out1[7] = x63;
  out1[8] = x66;
  out1[9] = x67;
}

