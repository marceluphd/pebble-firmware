// The MIT License (MIT)
//
// Copyright (c) 2015-2016 the fiat-crypto authors (see the AUTHORS file).
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Some of this code is taken from the ref10 version of Ed25519 in SUPERCOP
// 20141124 (http://bench.cr.yp.to/supercop.html). That code is released as
// public domain but parts have been replaced with code generated by Fiat
// (https://github.com/mit-plv/fiat-crypto), which is MIT licensed.
//
// The field functions are shared by Ed25519 and X25519 where possible.

#include <assert.h>
#include <string.h>
#include <stdint.h>

#include <mcuboot_config/mcuboot_config.h>

#if defined(MCUBOOT_USE_MBED_TLS)
#include <mbedtls/platform_util.h>
#include <mbedtls/sha512.h>
#else
#include <tinycrypt/constants.h>
#include <tinycrypt/utils.h>
#include <tinycrypt/sha512.h>
#endif

#include "curve25519.h"
// Various pre-computed constants.
#include "curve25519_tables.h"

#define SHA512_DIGEST_LENGTH 64

// Low-level intrinsic operations

static uint64_t load_3(const uint8_t *in) {
  uint64_t result;
  result = (uint64_t)in[0];
  result |= ((uint64_t)in[1]) << 8;
  result |= ((uint64_t)in[2]) << 16;
  return result;
}

static uint64_t load_4(const uint8_t *in) {
  uint64_t result;
  result = (uint64_t)in[0];
  result |= ((uint64_t)in[1]) << 8;
  result |= ((uint64_t)in[2]) << 16;
  result |= ((uint64_t)in[3]) << 24;
  return result;
}


// Field operations.

typedef uint32_t fe_limb_t;
#define FE_NUM_LIMBS 10

// assert_fe asserts that |f| satisfies bounds:
//
//  [[0x0 ~> 0x4666666], [0x0 ~> 0x2333333],
//   [0x0 ~> 0x4666666], [0x0 ~> 0x2333333],
//   [0x0 ~> 0x4666666], [0x0 ~> 0x2333333],
//   [0x0 ~> 0x4666666], [0x0 ~> 0x2333333],
//   [0x0 ~> 0x4666666], [0x0 ~> 0x2333333]]
//
// See comments in curve25519_32.h for which functions use these bounds for
// inputs or outputs.
#define assert_fe(f)                                                     \
  do {                                                                   \
    for (unsigned _assert_fe_i = 0; _assert_fe_i < 10; _assert_fe_i++) { \
      assert(f[_assert_fe_i] <=                                          \
             ((_assert_fe_i & 1) ? 0x2333333u : 0x4666666u));            \
    }                                                                    \
  } while (0)

// assert_fe_loose asserts that |f| satisfies bounds:
//
//  [[0x0 ~> 0xd333332], [0x0 ~> 0x6999999],
//   [0x0 ~> 0xd333332], [0x0 ~> 0x6999999],
//   [0x0 ~> 0xd333332], [0x0 ~> 0x6999999],
//   [0x0 ~> 0xd333332], [0x0 ~> 0x6999999],
//   [0x0 ~> 0xd333332], [0x0 ~> 0x6999999]]
//
// See comments in curve25519_32.h for which functions use these bounds for
// inputs or outputs.
#define assert_fe_loose(f)                                               \
  do {                                                                   \
    for (unsigned _assert_fe_i = 0; _assert_fe_i < 10; _assert_fe_i++) { \
      assert(f[_assert_fe_i] <=                                          \
             ((_assert_fe_i & 1) ? 0x6999999u : 0xd333332u));            \
    }                                                                    \
  } while (0)

//FIXME: use Zephyr macro
_Static_assert(sizeof(fe) == sizeof(fe_limb_t) * FE_NUM_LIMBS,
               "fe_limb_t[FE_NUM_LIMBS] is inconsistent with fe");

static void fe_frombytes_strict(fe *h, const uint8_t s[32]) {
  // |fiat_25519_from_bytes| requires the top-most bit be clear.
  assert((s[31] & 0x80) == 0);
  fiat_25519_from_bytes(h->v, s);
  assert_fe(h->v);
}

static void fe_frombytes(fe *h, const uint8_t s[32]) {
  uint8_t s_copy[32];
  memcpy(s_copy, s, 32);
  s_copy[31] &= 0x7f;
  fe_frombytes_strict(h, s_copy);
}

static void fe_tobytes(uint8_t s[32], const fe *f) {
  assert_fe(f->v);
  fiat_25519_to_bytes(s, f->v);
}

// h = 0
static void fe_0(fe *h) {
#if defined(MCUBOOT_USE_MBED_TLS)
  mbedtls_platform_zeroize(h, sizeof(fe));
#else
  _set(h, 0, sizeof(fe));
#endif
}

// h = 1
static void fe_1(fe *h) {
#if defined(MCUBOOT_USE_MBED_TLS)
  mbedtls_platform_zeroize(h, sizeof(fe));
#else
  _set(h, 0, sizeof(fe));
#endif
  h->v[0] = 1;
}

// h = f + g
// Can overlap h with f or g.
static void fe_add(fe_loose *h, const fe *f, const fe *g) {
  assert_fe(f->v);
  assert_fe(g->v);
  fiat_25519_add(h->v, f->v, g->v);
  assert_fe_loose(h->v);
}

// h = f - g
// Can overlap h with f or g.
static void fe_sub(fe_loose *h, const fe *f, const fe *g) {
  assert_fe(f->v);
  assert_fe(g->v);
  fiat_25519_sub(h->v, f->v, g->v);
  assert_fe_loose(h->v);
}

static void fe_carry(fe *h, const fe_loose* f) {
  assert_fe_loose(f->v);
  fiat_25519_carry(h->v, f->v);
  assert_fe(h->v);
}

static void fe_mul_impl(fe_limb_t out[FE_NUM_LIMBS],
                        const fe_limb_t in1[FE_NUM_LIMBS],
                        const fe_limb_t in2[FE_NUM_LIMBS]) {
  assert_fe_loose(in1);
  assert_fe_loose(in2);
  fiat_25519_carry_mul(out, in1, in2);
  assert_fe(out);
}

static void fe_mul_ltt(fe_loose *h, const fe *f, const fe *g) {
  fe_mul_impl(h->v, f->v, g->v);
}

static void fe_mul_ttt(fe *h, const fe *f, const fe *g) {
  fe_mul_impl(h->v, f->v, g->v);
}

static void fe_mul_tlt(fe *h, const fe_loose *f, const fe *g) {
  fe_mul_impl(h->v, f->v, g->v);
}

static void fe_mul_ttl(fe *h, const fe *f, const fe_loose *g) {
  fe_mul_impl(h->v, f->v, g->v);
}

static void fe_mul_tll(fe *h, const fe_loose *f, const fe_loose *g) {
  fe_mul_impl(h->v, f->v, g->v);
}

static void fe_sq_tl(fe *h, const fe_loose *f) {
  assert_fe_loose(f->v);
  fiat_25519_carry_square(h->v, f->v);
  assert_fe(h->v);
}

static void fe_sq_tt(fe *h, const fe *f) {
  assert_fe_loose(f->v);
  fiat_25519_carry_square(h->v, f->v);
  assert_fe(h->v);
}

// h = -f
static void fe_neg(fe_loose *h, const fe *f) {
  assert_fe(f->v);
  fiat_25519_opp(h->v, f->v);
  assert_fe_loose(h->v);
}

// h = f
static void fe_copy(fe *h, const fe *f) {
  memmove(h, f, sizeof(fe));
}

static void fe_copy_lt(fe_loose *h, const fe *f) {
  //FIXME: use Zephyr macro
  _Static_assert(sizeof(fe_loose) == sizeof(fe), "fe and fe_loose mismatch");
  memmove(h, f, sizeof(fe));
}

static void fe_loose_invert(fe *out, const fe_loose *z) {
  fe t0;
  fe t1;
  fe t2;
  fe t3;
  int i;

  fe_sq_tl(&t0, z);
  fe_sq_tt(&t1, &t0);
  for (i = 1; i < 2; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_tlt(&t1, z, &t1);
  fe_mul_ttt(&t0, &t0, &t1);
  fe_sq_tt(&t2, &t0);
  fe_mul_ttt(&t1, &t1, &t2);
  fe_sq_tt(&t2, &t1);
  for (i = 1; i < 5; ++i) {
    fe_sq_tt(&t2, &t2);
  }
  fe_mul_ttt(&t1, &t2, &t1);
  fe_sq_tt(&t2, &t1);
  for (i = 1; i < 10; ++i) {
    fe_sq_tt(&t2, &t2);
  }
  fe_mul_ttt(&t2, &t2, &t1);
  fe_sq_tt(&t3, &t2);
  for (i = 1; i < 20; ++i) {
    fe_sq_tt(&t3, &t3);
  }
  fe_mul_ttt(&t2, &t3, &t2);
  fe_sq_tt(&t2, &t2);
  for (i = 1; i < 10; ++i) {
    fe_sq_tt(&t2, &t2);
  }
  fe_mul_ttt(&t1, &t2, &t1);
  fe_sq_tt(&t2, &t1);
  for (i = 1; i < 50; ++i) {
    fe_sq_tt(&t2, &t2);
  }
  fe_mul_ttt(&t2, &t2, &t1);
  fe_sq_tt(&t3, &t2);
  for (i = 1; i < 100; ++i) {
    fe_sq_tt(&t3, &t3);
  }
  fe_mul_ttt(&t2, &t3, &t2);
  fe_sq_tt(&t2, &t2);
  for (i = 1; i < 50; ++i) {
    fe_sq_tt(&t2, &t2);
  }
  fe_mul_ttt(&t1, &t2, &t1);
  fe_sq_tt(&t1, &t1);
  for (i = 1; i < 5; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_ttt(out, &t1, &t0);
}

static void fe_invert(fe *out, const fe *z) {
  fe_loose l;
  fe_copy_lt(&l, z);
  fe_loose_invert(out, &l);
}

static int CRYPTO_memcmp(const void *in_a, const void *in_b, size_t len) {
  const uint8_t *a = in_a;
  const uint8_t *b = in_b;
  uint8_t x = 0;

  for (size_t i = 0; i < len; i++) {
    x |= a[i] ^ b[i];
  }

  return x;
}

// return 0 if f == 0
// return 1 if f != 0
static int fe_isnonzero(const fe_loose *f) {
  fe tight;
  fe_carry(&tight, f);
  uint8_t s[32];
  fe_tobytes(s, &tight);

  static const uint8_t zero[32] = {0};
  return CRYPTO_memcmp(s, zero, sizeof(zero)) != 0;
}

// return 1 if f is in {1,3,5,...,q-2}
// return 0 if f is in {0,2,4,...,q-1}
static int fe_isnegative(const fe *f) {
  uint8_t s[32];
  fe_tobytes(s, f);
  return s[0] & 1;
}

static void fe_sq2_tt(fe *h, const fe *f) {
  // h = f^2
  fe_sq_tt(h, f);

  // h = h + h
  fe_loose tmp;
  fe_add(&tmp, h, h);
  fe_carry(h, &tmp);
}

static void fe_pow22523(fe *out, const fe *z) {
  fe t0;
  fe t1;
  fe t2;
  int i;

  fe_sq_tt(&t0, z);
  fe_sq_tt(&t1, &t0);
  for (i = 1; i < 2; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_ttt(&t1, z, &t1);
  fe_mul_ttt(&t0, &t0, &t1);
  fe_sq_tt(&t0, &t0);
  fe_mul_ttt(&t0, &t1, &t0);
  fe_sq_tt(&t1, &t0);
  for (i = 1; i < 5; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_ttt(&t0, &t1, &t0);
  fe_sq_tt(&t1, &t0);
  for (i = 1; i < 10; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_ttt(&t1, &t1, &t0);
  fe_sq_tt(&t2, &t1);
  for (i = 1; i < 20; ++i) {
    fe_sq_tt(&t2, &t2);
  }
  fe_mul_ttt(&t1, &t2, &t1);
  fe_sq_tt(&t1, &t1);
  for (i = 1; i < 10; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_ttt(&t0, &t1, &t0);
  fe_sq_tt(&t1, &t0);
  for (i = 1; i < 50; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_ttt(&t1, &t1, &t0);
  fe_sq_tt(&t2, &t1);
  for (i = 1; i < 100; ++i) {
    fe_sq_tt(&t2, &t2);
  }
  fe_mul_ttt(&t1, &t2, &t1);
  fe_sq_tt(&t1, &t1);
  for (i = 1; i < 50; ++i) {
    fe_sq_tt(&t1, &t1);
  }
  fe_mul_ttt(&t0, &t1, &t0);
  fe_sq_tt(&t0, &t0);
  for (i = 1; i < 2; ++i) {
    fe_sq_tt(&t0, &t0);
  }
  fe_mul_ttt(out, &t0, z);
}


// Group operations.

void x25519_ge_tobytes(uint8_t s[32], const ge_p2 *h) {
  fe recip;
  fe x;
  fe y;

  fe_invert(&recip, &h->Z);
  fe_mul_ttt(&x, &h->X, &recip);
  fe_mul_ttt(&y, &h->Y, &recip);
  fe_tobytes(s, &y);
  s[31] ^= fe_isnegative(&x) << 7;
}

int x25519_ge_frombytes_vartime(ge_p3 *h, const uint8_t s[32]) {
  fe u;
  fe_loose v;
  fe v3;
  fe vxx;
  fe_loose check;

  fe_frombytes(&h->Y, s);
  fe_1(&h->Z);
  fe_sq_tt(&v3, &h->Y);
  fe_mul_ttt(&vxx, &v3, &d);
  fe_sub(&v, &v3, &h->Z);  // u = y^2-1
  fe_carry(&u, &v);
  fe_add(&v, &vxx, &h->Z);  // v = dy^2+1

  fe_sq_tl(&v3, &v);
  fe_mul_ttl(&v3, &v3, &v);  // v3 = v^3
  fe_sq_tt(&h->X, &v3);
  fe_mul_ttl(&h->X, &h->X, &v);
  fe_mul_ttt(&h->X, &h->X, &u);  // x = uv^7

  fe_pow22523(&h->X, &h->X);  // x = (uv^7)^((q-5)/8)
  fe_mul_ttt(&h->X, &h->X, &v3);
  fe_mul_ttt(&h->X, &h->X, &u);  // x = uv^3(uv^7)^((q-5)/8)

  fe_sq_tt(&vxx, &h->X);
  fe_mul_ttl(&vxx, &vxx, &v);
  fe_sub(&check, &vxx, &u);
  if (fe_isnonzero(&check)) {
    fe_add(&check, &vxx, &u);
    if (fe_isnonzero(&check)) {
      return 0;
    }
    fe_mul_ttt(&h->X, &h->X, &sqrtm1);
  }

  if (fe_isnegative(&h->X) != (s[31] >> 7)) {
    fe_loose t;
    fe_neg(&t, &h->X);
    fe_carry(&h->X, &t);
  }

  fe_mul_ttt(&h->T, &h->X, &h->Y);
  return 1;
}

static void ge_p2_0(ge_p2 *h) {
  fe_0(&h->X);
  fe_1(&h->Y);
  fe_1(&h->Z);
}

// r = p
static void ge_p3_to_p2(ge_p2 *r, const ge_p3 *p) {
  fe_copy(&r->X, &p->X);
  fe_copy(&r->Y, &p->Y);
  fe_copy(&r->Z, &p->Z);
}

// r = p
void x25519_ge_p3_to_cached(ge_cached *r, const ge_p3 *p) {
  fe_add(&r->YplusX, &p->Y, &p->X);
  fe_sub(&r->YminusX, &p->Y, &p->X);
  fe_copy_lt(&r->Z, &p->Z);
  fe_mul_ltt(&r->T2d, &p->T, &d2);
}

// r = p
void x25519_ge_p1p1_to_p2(ge_p2 *r, const ge_p1p1 *p) {
  fe_mul_tll(&r->X, &p->X, &p->T);
  fe_mul_tll(&r->Y, &p->Y, &p->Z);
  fe_mul_tll(&r->Z, &p->Z, &p->T);
}

// r = p
void x25519_ge_p1p1_to_p3(ge_p3 *r, const ge_p1p1 *p) {
  fe_mul_tll(&r->X, &p->X, &p->T);
  fe_mul_tll(&r->Y, &p->Y, &p->Z);
  fe_mul_tll(&r->Z, &p->Z, &p->T);
  fe_mul_tll(&r->T, &p->X, &p->Y);
}

// r = 2 * p
static void ge_p2_dbl(ge_p1p1 *r, const ge_p2 *p) {
  fe trX, trZ, trT;
  fe t0;

  fe_sq_tt(&trX, &p->X);
  fe_sq_tt(&trZ, &p->Y);
  fe_sq2_tt(&trT, &p->Z);
  fe_add(&r->Y, &p->X, &p->Y);
  fe_sq_tl(&t0, &r->Y);

  fe_add(&r->Y, &trZ, &trX);
  fe_sub(&r->Z, &trZ, &trX);
  fe_carry(&trZ, &r->Y);
  fe_sub(&r->X, &t0, &trZ);
  fe_carry(&trZ, &r->Z);
  fe_sub(&r->T, &trT, &trZ);
}

// r = 2 * p
static void ge_p3_dbl(ge_p1p1 *r, const ge_p3 *p) {
  ge_p2 q;
  ge_p3_to_p2(&q, p);
  ge_p2_dbl(r, &q);
}

// r = p + q
static void ge_madd(ge_p1p1 *r, const ge_p3 *p, const ge_precomp *q) {
  fe trY, trZ, trT;

  fe_add(&r->X, &p->Y, &p->X);
  fe_sub(&r->Y, &p->Y, &p->X);
  fe_mul_tll(&trZ, &r->X, &q->yplusx);
  fe_mul_tll(&trY, &r->Y, &q->yminusx);
  fe_mul_tlt(&trT, &q->xy2d, &p->T);
  fe_add(&r->T, &p->Z, &p->Z);
  fe_sub(&r->X, &trZ, &trY);
  fe_add(&r->Y, &trZ, &trY);
  fe_carry(&trZ, &r->T);
  fe_add(&r->Z, &trZ, &trT);
  fe_sub(&r->T, &trZ, &trT);
}

// r = p - q
static void ge_msub(ge_p1p1 *r, const ge_p3 *p, const ge_precomp *q) {
  fe trY, trZ, trT;

  fe_add(&r->X, &p->Y, &p->X);
  fe_sub(&r->Y, &p->Y, &p->X);
  fe_mul_tll(&trZ, &r->X, &q->yminusx);
  fe_mul_tll(&trY, &r->Y, &q->yplusx);
  fe_mul_tlt(&trT, &q->xy2d, &p->T);
  fe_add(&r->T, &p->Z, &p->Z);
  fe_sub(&r->X, &trZ, &trY);
  fe_add(&r->Y, &trZ, &trY);
  fe_carry(&trZ, &r->T);
  fe_sub(&r->Z, &trZ, &trT);
  fe_add(&r->T, &trZ, &trT);
}

// r = p + q
void x25519_ge_add(ge_p1p1 *r, const ge_p3 *p, const ge_cached *q) {
  fe trX, trY, trZ, trT;

  fe_add(&r->X, &p->Y, &p->X);
  fe_sub(&r->Y, &p->Y, &p->X);
  fe_mul_tll(&trZ, &r->X, &q->YplusX);
  fe_mul_tll(&trY, &r->Y, &q->YminusX);
  fe_mul_tlt(&trT, &q->T2d, &p->T);
  fe_mul_ttl(&trX, &p->Z, &q->Z);
  fe_add(&r->T, &trX, &trX);
  fe_sub(&r->X, &trZ, &trY);
  fe_add(&r->Y, &trZ, &trY);
  fe_carry(&trZ, &r->T);
  fe_add(&r->Z, &trZ, &trT);
  fe_sub(&r->T, &trZ, &trT);
}

// r = p - q
void x25519_ge_sub(ge_p1p1 *r, const ge_p3 *p, const ge_cached *q) {
  fe trX, trY, trZ, trT;

  fe_add(&r->X, &p->Y, &p->X);
  fe_sub(&r->Y, &p->Y, &p->X);
  fe_mul_tll(&trZ, &r->X, &q->YminusX);
  fe_mul_tll(&trY, &r->Y, &q->YplusX);
  fe_mul_tlt(&trT, &q->T2d, &p->T);
  fe_mul_ttl(&trX, &p->Z, &q->Z);
  fe_add(&r->T, &trX, &trX);
  fe_sub(&r->X, &trZ, &trY);
  fe_add(&r->Y, &trZ, &trY);
  fe_carry(&trZ, &r->T);
  fe_sub(&r->Z, &trZ, &trT);
  fe_add(&r->T, &trZ, &trT);
}

static void slide(signed char *r, const uint8_t *a) {
  int i;
  int b;
  int k;

  for (i = 0; i < 256; ++i) {
    r[i] = 1 & (a[i >> 3] >> (i & 7));
  }

  for (i = 0; i < 256; ++i) {
    if (r[i]) {
      for (b = 1; b <= 6 && i + b < 256; ++b) {
        if (r[i + b]) {
          if (r[i] + (r[i + b] << b) <= 15) {
            r[i] += r[i + b] << b;
            r[i + b] = 0;
          } else if (r[i] - (r[i + b] << b) >= -15) {
            r[i] -= r[i + b] << b;
            for (k = i + b; k < 256; ++k) {
              if (!r[k]) {
                r[k] = 1;
                break;
              }
              r[k] = 0;
            }
          } else {
            break;
          }
        }
      }
    }
  }
}

// r = a * A + b * B
// where a = a[0]+256*a[1]+...+256^31 a[31].
// and b = b[0]+256*b[1]+...+256^31 b[31].
// B is the Ed25519 base point (x,4/5) with x positive.
static void ge_double_scalarmult_vartime(ge_p2 *r, const uint8_t *a,
                                         const ge_p3 *A, const uint8_t *b) {
  signed char aslide[256];
  signed char bslide[256];
  ge_cached Ai[8];  // A,3A,5A,7A,9A,11A,13A,15A
  ge_p1p1 t;
  ge_p3 u;
  ge_p3 A2;
  int i;

  slide(aslide, a);
  slide(bslide, b);

  x25519_ge_p3_to_cached(&Ai[0], A);
  ge_p3_dbl(&t, A);
  x25519_ge_p1p1_to_p3(&A2, &t);
  x25519_ge_add(&t, &A2, &Ai[0]);
  x25519_ge_p1p1_to_p3(&u, &t);
  x25519_ge_p3_to_cached(&Ai[1], &u);
  x25519_ge_add(&t, &A2, &Ai[1]);
  x25519_ge_p1p1_to_p3(&u, &t);
  x25519_ge_p3_to_cached(&Ai[2], &u);
  x25519_ge_add(&t, &A2, &Ai[2]);
  x25519_ge_p1p1_to_p3(&u, &t);
  x25519_ge_p3_to_cached(&Ai[3], &u);
  x25519_ge_add(&t, &A2, &Ai[3]);
  x25519_ge_p1p1_to_p3(&u, &t);
  x25519_ge_p3_to_cached(&Ai[4], &u);
  x25519_ge_add(&t, &A2, &Ai[4]);
  x25519_ge_p1p1_to_p3(&u, &t);
  x25519_ge_p3_to_cached(&Ai[5], &u);
  x25519_ge_add(&t, &A2, &Ai[5]);
  x25519_ge_p1p1_to_p3(&u, &t);
  x25519_ge_p3_to_cached(&Ai[6], &u);
  x25519_ge_add(&t, &A2, &Ai[6]);
  x25519_ge_p1p1_to_p3(&u, &t);
  x25519_ge_p3_to_cached(&Ai[7], &u);

  ge_p2_0(r);

  for (i = 255; i >= 0; --i) {
    if (aslide[i] || bslide[i]) {
      break;
    }
  }

  for (; i >= 0; --i) {
    ge_p2_dbl(&t, r);

    if (aslide[i] > 0) {
      x25519_ge_p1p1_to_p3(&u, &t);
      x25519_ge_add(&t, &u, &Ai[aslide[i] / 2]);
    } else if (aslide[i] < 0) {
      x25519_ge_p1p1_to_p3(&u, &t);
      x25519_ge_sub(&t, &u, &Ai[(-aslide[i]) / 2]);
    }

    if (bslide[i] > 0) {
      x25519_ge_p1p1_to_p3(&u, &t);
      ge_madd(&t, &u, &Bi[bslide[i] / 2]);
    } else if (bslide[i] < 0) {
      x25519_ge_p1p1_to_p3(&u, &t);
      ge_msub(&t, &u, &Bi[(-bslide[i]) / 2]);
    }

    x25519_ge_p1p1_to_p2(r, &t);
  }
}

// int64_lshift21 returns |a << 21| but is defined when shifting bits into the
// sign bit. This works around a language flaw in C.
static inline int64_t int64_lshift21(int64_t a) {
  return (int64_t)((uint64_t)a << 21);
}

// The set of scalars is \Z/l
// where l = 2^252 + 27742317777372353535851937790883648493.

// Input:
//   s[0]+256*s[1]+...+256^63*s[63] = s
//
// Output:
//   s[0]+256*s[1]+...+256^31*s[31] = s mod l
//   where l = 2^252 + 27742317777372353535851937790883648493.
//   Overwrites s in place.
void x25519_sc_reduce(uint8_t s[64]) {
  int64_t s0 = 2097151 & load_3(s);
  int64_t s1 = 2097151 & (load_4(s + 2) >> 5);
  int64_t s2 = 2097151 & (load_3(s + 5) >> 2);
  int64_t s3 = 2097151 & (load_4(s + 7) >> 7);
  int64_t s4 = 2097151 & (load_4(s + 10) >> 4);
  int64_t s5 = 2097151 & (load_3(s + 13) >> 1);
  int64_t s6 = 2097151 & (load_4(s + 15) >> 6);
  int64_t s7 = 2097151 & (load_3(s + 18) >> 3);
  int64_t s8 = 2097151 & load_3(s + 21);
  int64_t s9 = 2097151 & (load_4(s + 23) >> 5);
  int64_t s10 = 2097151 & (load_3(s + 26) >> 2);
  int64_t s11 = 2097151 & (load_4(s + 28) >> 7);
  int64_t s12 = 2097151 & (load_4(s + 31) >> 4);
  int64_t s13 = 2097151 & (load_3(s + 34) >> 1);
  int64_t s14 = 2097151 & (load_4(s + 36) >> 6);
  int64_t s15 = 2097151 & (load_3(s + 39) >> 3);
  int64_t s16 = 2097151 & load_3(s + 42);
  int64_t s17 = 2097151 & (load_4(s + 44) >> 5);
  int64_t s18 = 2097151 & (load_3(s + 47) >> 2);
  int64_t s19 = 2097151 & (load_4(s + 49) >> 7);
  int64_t s20 = 2097151 & (load_4(s + 52) >> 4);
  int64_t s21 = 2097151 & (load_3(s + 55) >> 1);
  int64_t s22 = 2097151 & (load_4(s + 57) >> 6);
  int64_t s23 = (load_4(s + 60) >> 3);
  int64_t carry0;
  int64_t carry1;
  int64_t carry2;
  int64_t carry3;
  int64_t carry4;
  int64_t carry5;
  int64_t carry6;
  int64_t carry7;
  int64_t carry8;
  int64_t carry9;
  int64_t carry10;
  int64_t carry11;
  int64_t carry12;
  int64_t carry13;
  int64_t carry14;
  int64_t carry15;
  int64_t carry16;

  s11 += s23 * 666643;
  s12 += s23 * 470296;
  s13 += s23 * 654183;
  s14 -= s23 * 997805;
  s15 += s23 * 136657;
  s16 -= s23 * 683901;
  s23 = 0;

  s10 += s22 * 666643;
  s11 += s22 * 470296;
  s12 += s22 * 654183;
  s13 -= s22 * 997805;
  s14 += s22 * 136657;
  s15 -= s22 * 683901;
  s22 = 0;

  s9 += s21 * 666643;
  s10 += s21 * 470296;
  s11 += s21 * 654183;
  s12 -= s21 * 997805;
  s13 += s21 * 136657;
  s14 -= s21 * 683901;
  s21 = 0;

  s8 += s20 * 666643;
  s9 += s20 * 470296;
  s10 += s20 * 654183;
  s11 -= s20 * 997805;
  s12 += s20 * 136657;
  s13 -= s20 * 683901;
  s20 = 0;

  s7 += s19 * 666643;
  s8 += s19 * 470296;
  s9 += s19 * 654183;
  s10 -= s19 * 997805;
  s11 += s19 * 136657;
  s12 -= s19 * 683901;
  s19 = 0;

  s6 += s18 * 666643;
  s7 += s18 * 470296;
  s8 += s18 * 654183;
  s9 -= s18 * 997805;
  s10 += s18 * 136657;
  s11 -= s18 * 683901;
  s18 = 0;

  carry6 = (s6 + (1 << 20)) >> 21;
  s7 += carry6;
  s6 -= int64_lshift21(carry6);
  carry8 = (s8 + (1 << 20)) >> 21;
  s9 += carry8;
  s8 -= int64_lshift21(carry8);
  carry10 = (s10 + (1 << 20)) >> 21;
  s11 += carry10;
  s10 -= int64_lshift21(carry10);
  carry12 = (s12 + (1 << 20)) >> 21;
  s13 += carry12;
  s12 -= int64_lshift21(carry12);
  carry14 = (s14 + (1 << 20)) >> 21;
  s15 += carry14;
  s14 -= int64_lshift21(carry14);
  carry16 = (s16 + (1 << 20)) >> 21;
  s17 += carry16;
  s16 -= int64_lshift21(carry16);

  carry7 = (s7 + (1 << 20)) >> 21;
  s8 += carry7;
  s7 -= int64_lshift21(carry7);
  carry9 = (s9 + (1 << 20)) >> 21;
  s10 += carry9;
  s9 -= int64_lshift21(carry9);
  carry11 = (s11 + (1 << 20)) >> 21;
  s12 += carry11;
  s11 -= int64_lshift21(carry11);
  carry13 = (s13 + (1 << 20)) >> 21;
  s14 += carry13;
  s13 -= int64_lshift21(carry13);
  carry15 = (s15 + (1 << 20)) >> 21;
  s16 += carry15;
  s15 -= int64_lshift21(carry15);

  s5 += s17 * 666643;
  s6 += s17 * 470296;
  s7 += s17 * 654183;
  s8 -= s17 * 997805;
  s9 += s17 * 136657;
  s10 -= s17 * 683901;
  s17 = 0;

  s4 += s16 * 666643;
  s5 += s16 * 470296;
  s6 += s16 * 654183;
  s7 -= s16 * 997805;
  s8 += s16 * 136657;
  s9 -= s16 * 683901;
  s16 = 0;

  s3 += s15 * 666643;
  s4 += s15 * 470296;
  s5 += s15 * 654183;
  s6 -= s15 * 997805;
  s7 += s15 * 136657;
  s8 -= s15 * 683901;
  s15 = 0;

  s2 += s14 * 666643;
  s3 += s14 * 470296;
  s4 += s14 * 654183;
  s5 -= s14 * 997805;
  s6 += s14 * 136657;
  s7 -= s14 * 683901;
  s14 = 0;

  s1 += s13 * 666643;
  s2 += s13 * 470296;
  s3 += s13 * 654183;
  s4 -= s13 * 997805;
  s5 += s13 * 136657;
  s6 -= s13 * 683901;
  s13 = 0;

  s0 += s12 * 666643;
  s1 += s12 * 470296;
  s2 += s12 * 654183;
  s3 -= s12 * 997805;
  s4 += s12 * 136657;
  s5 -= s12 * 683901;
  s12 = 0;

  carry0 = (s0 + (1 << 20)) >> 21;
  s1 += carry0;
  s0 -= int64_lshift21(carry0);
  carry2 = (s2 + (1 << 20)) >> 21;
  s3 += carry2;
  s2 -= int64_lshift21(carry2);
  carry4 = (s4 + (1 << 20)) >> 21;
  s5 += carry4;
  s4 -= int64_lshift21(carry4);
  carry6 = (s6 + (1 << 20)) >> 21;
  s7 += carry6;
  s6 -= int64_lshift21(carry6);
  carry8 = (s8 + (1 << 20)) >> 21;
  s9 += carry8;
  s8 -= int64_lshift21(carry8);
  carry10 = (s10 + (1 << 20)) >> 21;
  s11 += carry10;
  s10 -= int64_lshift21(carry10);

  carry1 = (s1 + (1 << 20)) >> 21;
  s2 += carry1;
  s1 -= int64_lshift21(carry1);
  carry3 = (s3 + (1 << 20)) >> 21;
  s4 += carry3;
  s3 -= int64_lshift21(carry3);
  carry5 = (s5 + (1 << 20)) >> 21;
  s6 += carry5;
  s5 -= int64_lshift21(carry5);
  carry7 = (s7 + (1 << 20)) >> 21;
  s8 += carry7;
  s7 -= int64_lshift21(carry7);
  carry9 = (s9 + (1 << 20)) >> 21;
  s10 += carry9;
  s9 -= int64_lshift21(carry9);
  carry11 = (s11 + (1 << 20)) >> 21;
  s12 += carry11;
  s11 -= int64_lshift21(carry11);

  s0 += s12 * 666643;
  s1 += s12 * 470296;
  s2 += s12 * 654183;
  s3 -= s12 * 997805;
  s4 += s12 * 136657;
  s5 -= s12 * 683901;
  s12 = 0;

  carry0 = s0 >> 21;
  s1 += carry0;
  s0 -= int64_lshift21(carry0);
  carry1 = s1 >> 21;
  s2 += carry1;
  s1 -= int64_lshift21(carry1);
  carry2 = s2 >> 21;
  s3 += carry2;
  s2 -= int64_lshift21(carry2);
  carry3 = s3 >> 21;
  s4 += carry3;
  s3 -= int64_lshift21(carry3);
  carry4 = s4 >> 21;
  s5 += carry4;
  s4 -= int64_lshift21(carry4);
  carry5 = s5 >> 21;
  s6 += carry5;
  s5 -= int64_lshift21(carry5);
  carry6 = s6 >> 21;
  s7 += carry6;
  s6 -= int64_lshift21(carry6);
  carry7 = s7 >> 21;
  s8 += carry7;
  s7 -= int64_lshift21(carry7);
  carry8 = s8 >> 21;
  s9 += carry8;
  s8 -= int64_lshift21(carry8);
  carry9 = s9 >> 21;
  s10 += carry9;
  s9 -= int64_lshift21(carry9);
  carry10 = s10 >> 21;
  s11 += carry10;
  s10 -= int64_lshift21(carry10);
  carry11 = s11 >> 21;
  s12 += carry11;
  s11 -= int64_lshift21(carry11);

  s0 += s12 * 666643;
  s1 += s12 * 470296;
  s2 += s12 * 654183;
  s3 -= s12 * 997805;
  s4 += s12 * 136657;
  s5 -= s12 * 683901;
  s12 = 0;

  carry0 = s0 >> 21;
  s1 += carry0;
  s0 -= int64_lshift21(carry0);
  carry1 = s1 >> 21;
  s2 += carry1;
  s1 -= int64_lshift21(carry1);
  carry2 = s2 >> 21;
  s3 += carry2;
  s2 -= int64_lshift21(carry2);
  carry3 = s3 >> 21;
  s4 += carry3;
  s3 -= int64_lshift21(carry3);
  carry4 = s4 >> 21;
  s5 += carry4;
  s4 -= int64_lshift21(carry4);
  carry5 = s5 >> 21;
  s6 += carry5;
  s5 -= int64_lshift21(carry5);
  carry6 = s6 >> 21;
  s7 += carry6;
  s6 -= int64_lshift21(carry6);
  carry7 = s7 >> 21;
  s8 += carry7;
  s7 -= int64_lshift21(carry7);
  carry8 = s8 >> 21;
  s9 += carry8;
  s8 -= int64_lshift21(carry8);
  carry9 = s9 >> 21;
  s10 += carry9;
  s9 -= int64_lshift21(carry9);
  carry10 = s10 >> 21;
  s11 += carry10;
  s10 -= int64_lshift21(carry10);

  s[0] = s0 >> 0;
  s[1] = s0 >> 8;
  s[2] = (s0 >> 16) | (s1 << 5);
  s[3] = s1 >> 3;
  s[4] = s1 >> 11;
  s[5] = (s1 >> 19) | (s2 << 2);
  s[6] = s2 >> 6;
  s[7] = (s2 >> 14) | (s3 << 7);
  s[8] = s3 >> 1;
  s[9] = s3 >> 9;
  s[10] = (s3 >> 17) | (s4 << 4);
  s[11] = s4 >> 4;
  s[12] = s4 >> 12;
  s[13] = (s4 >> 20) | (s5 << 1);
  s[14] = s5 >> 7;
  s[15] = (s5 >> 15) | (s6 << 6);
  s[16] = s6 >> 2;
  s[17] = s6 >> 10;
  s[18] = (s6 >> 18) | (s7 << 3);
  s[19] = s7 >> 5;
  s[20] = s7 >> 13;
  s[21] = s8 >> 0;
  s[22] = s8 >> 8;
  s[23] = (s8 >> 16) | (s9 << 5);
  s[24] = s9 >> 3;
  s[25] = s9 >> 11;
  s[26] = (s9 >> 19) | (s10 << 2);
  s[27] = s10 >> 6;
  s[28] = (s10 >> 14) | (s11 << 7);
  s[29] = s11 >> 1;
  s[30] = s11 >> 9;
  s[31] = s11 >> 17;
}

int ED25519_verify(const uint8_t *message, size_t message_len,
                   const uint8_t signature[64], const uint8_t public_key[32]) {
  ge_p3 A;
  if ((signature[63] & 224) != 0 ||
      !x25519_ge_frombytes_vartime(&A, public_key)) {
    return 0;
  }

  fe_loose t;
  fe_neg(&t, &A.X);
  fe_carry(&A.X, &t);
  fe_neg(&t, &A.T);
  fe_carry(&A.T, &t);

  uint8_t pkcopy[32];
  memcpy(pkcopy, public_key, 32);
  uint8_t rcopy[32];
  memcpy(rcopy, signature, 32);
  union {
    uint64_t u64[4];
    uint8_t u8[32];
  } scopy;
  memcpy(&scopy.u8[0], signature + 32, 32);

  // https://tools.ietf.org/html/rfc8032#section-5.1.7 requires that s be in
  // the range [0, order) in order to prevent signature malleability.

  // kOrder is the order of Curve25519 in little-endian form.
  static const uint64_t kOrder[4] = {
    UINT64_C(0x5812631a5cf5d3ed),
    UINT64_C(0x14def9dea2f79cd6),
    0,
    UINT64_C(0x1000000000000000),
  };
  for (size_t i = 3;; i--) {
    if (scopy.u64[i] > kOrder[i]) {
      return 0;
    } else if (scopy.u64[i] < kOrder[i]) {
      break;
    } else if (i == 0) {
      return 0;
    }
  }

#if defined(MCUBOOT_USE_MBED_TLS)

  mbedtls_sha512_context ctx;
  int ret;

  mbedtls_sha512_init(&ctx);

  ret = mbedtls_sha512_starts_ret(&ctx, 0);
  assert(ret == 0);

  ret = mbedtls_sha512_update_ret(&ctx, signature, 32);
  assert(ret == 0);
  ret = mbedtls_sha512_update_ret(&ctx, public_key, 32);
  assert(ret == 0);
  ret = mbedtls_sha512_update_ret(&ctx, message, message_len);
  assert(ret == 0);

  uint8_t h[SHA512_DIGEST_LENGTH];
  ret = mbedtls_sha512_finish_ret(&ctx, h);
  assert(ret == 0);
  mbedtls_sha512_free(&ctx);

#else

  struct tc_sha512_state_struct s;
  int rc;

  rc = tc_sha512_init(&s);
  assert(rc == TC_CRYPTO_SUCCESS);

  rc = tc_sha512_update(&s, signature, 32);
  assert(rc == TC_CRYPTO_SUCCESS);
  rc = tc_sha512_update(&s, public_key, 32);
  assert(rc == TC_CRYPTO_SUCCESS);
  rc = tc_sha512_update(&s, message, message_len);
  assert(rc == TC_CRYPTO_SUCCESS);

  uint8_t h[TC_SHA512_DIGEST_SIZE];
  rc = tc_sha512_final(h, &s);
  assert(rc == TC_CRYPTO_SUCCESS);

#endif

  x25519_sc_reduce(h);

  ge_p2 R;
  ge_double_scalarmult_vartime(&R, h, &A, scopy.u8);

  uint8_t rcheck[32];
  x25519_ge_tobytes(rcheck, &R);

  return CRYPTO_memcmp(rcheck, rcopy, sizeof(rcheck)) == 0;
}

static void fe_cswap(fe *f, fe *g, fe_limb_t b) {
  b = 0-b;
  for (unsigned i = 0; i < FE_NUM_LIMBS; i++) {
    fe_limb_t x = f->v[i] ^ g->v[i];
    x &= b;
    f->v[i] ^= x;
    g->v[i] ^= x;
  }
}

static void fiat_25519_carry_scmul_121666(uint32_t out1[10], const uint32_t arg1[10]) {
  uint64_t x1 = ((uint64_t)UINT32_C(0x1db42) * (arg1[9]));
  uint64_t x2 = ((uint64_t)UINT32_C(0x1db42) * (arg1[8]));
  uint64_t x3 = ((uint64_t)UINT32_C(0x1db42) * (arg1[7]));
  uint64_t x4 = ((uint64_t)UINT32_C(0x1db42) * (arg1[6]));
  uint64_t x5 = ((uint64_t)UINT32_C(0x1db42) * (arg1[5]));
  uint64_t x6 = ((uint64_t)UINT32_C(0x1db42) * (arg1[4]));
  uint64_t x7 = ((uint64_t)UINT32_C(0x1db42) * (arg1[3]));
  uint64_t x8 = ((uint64_t)UINT32_C(0x1db42) * (arg1[2]));
  uint64_t x9 = ((uint64_t)UINT32_C(0x1db42) * (arg1[1]));
  uint64_t x10 = ((uint64_t)UINT32_C(0x1db42) * (arg1[0]));
  uint32_t x11 = (uint32_t)(x10 >> 26);
  uint32_t x12 = (uint32_t)(x10 & UINT32_C(0x3ffffff));
  uint64_t x13 = (x11 + x9);
  uint32_t x14 = (uint32_t)(x13 >> 25);
  uint32_t x15 = (uint32_t)(x13 & UINT32_C(0x1ffffff));
  uint64_t x16 = (x14 + x8);
  uint32_t x17 = (uint32_t)(x16 >> 26);
  uint32_t x18 = (uint32_t)(x16 & UINT32_C(0x3ffffff));
  uint64_t x19 = (x17 + x7);
  uint32_t x20 = (uint32_t)(x19 >> 25);
  uint32_t x21 = (uint32_t)(x19 & UINT32_C(0x1ffffff));
  uint64_t x22 = (x20 + x6);
  uint32_t x23 = (uint32_t)(x22 >> 26);
  uint32_t x24 = (uint32_t)(x22 & UINT32_C(0x3ffffff));
  uint64_t x25 = (x23 + x5);
  uint32_t x26 = (uint32_t)(x25 >> 25);
  uint32_t x27 = (uint32_t)(x25 & UINT32_C(0x1ffffff));
  uint64_t x28 = (x26 + x4);
  uint32_t x29 = (uint32_t)(x28 >> 26);
  uint32_t x30 = (uint32_t)(x28 & UINT32_C(0x3ffffff));
  uint64_t x31 = (x29 + x3);
  uint32_t x32 = (uint32_t)(x31 >> 25);
  uint32_t x33 = (uint32_t)(x31 & UINT32_C(0x1ffffff));
  uint64_t x34 = (x32 + x2);
  uint32_t x35 = (uint32_t)(x34 >> 26);
  uint32_t x36 = (uint32_t)(x34 & UINT32_C(0x3ffffff));
  uint64_t x37 = (x35 + x1);
  uint32_t x38 = (uint32_t)(x37 >> 25);
  uint32_t x39 = (uint32_t)(x37 & UINT32_C(0x1ffffff));
  uint32_t x40 = (x38 * (uint32_t)UINT8_C(0x13));
  uint32_t x41 = (x12 + x40);
  uint32_t x42 = (x41 >> 26);
  uint32_t x43 = (x41 & UINT32_C(0x3ffffff));
  uint32_t x44 = (x42 + x15);
  uint32_t x45 = (x44 >> 25);
  uint32_t x46 = (x44 & UINT32_C(0x1ffffff));
  uint32_t x47 = (x45 + x18);
  out1[0] = x43;
  out1[1] = x46;
  out1[2] = x47;
  out1[3] = x21;
  out1[4] = x24;
  out1[5] = x27;
  out1[6] = x30;
  out1[7] = x33;
  out1[8] = x36;
  out1[9] = x39;
}

static void fe_mul121666(fe *h, const fe_loose *f) {
  assert_fe_loose(f->v);
  fiat_25519_carry_scmul_121666(h->v, f->v);
  assert_fe(h->v);
}

static void x25519_scalar_mult_generic(uint8_t out[32],
                                       const uint8_t scalar[32],
                                       const uint8_t point[32]) {
  fe x1, x2, z2, x3, z3, tmp0, tmp1;
  fe_loose x2l, z2l, x3l, tmp0l, tmp1l;

  uint8_t e[32];
  memcpy(e, scalar, 32);
  e[0] &= 248;
  e[31] &= 127;
  e[31] |= 64;

  // The following implementation was transcribed to Coq and proven to
  // correspond to unary scalar multiplication in affine coordinates given that
  // x1 != 0 is the x coordinate of some point on the curve. It was also checked
  // in Coq that doing a ladderstep with x1 = x3 = 0 gives z2' = z3' = 0, and z2
  // = z3 = 0 gives z2' = z3' = 0. The statement was quantified over the
  // underlying field, so it applies to Curve25519 itself and the quadratic
  // twist of Curve25519. It was not proven in Coq that prime-field arithmetic
  // correctly simulates extension-field arithmetic on prime-field values.
  // The decoding of the byte array representation of e was not considered.
  // Specification of Montgomery curves in affine coordinates:
  // <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Spec/MontgomeryCurve.v#L27>
  // Proof that these form a group that is isomorphic to a Weierstrass curve:
  // <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Curves/Montgomery/AffineProofs.v#L35>
  // Coq transcription and correctness proof of the loop (where scalarbits=255):
  // <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Curves/Montgomery/XZ.v#L118>
  // <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Curves/Montgomery/XZProofs.v#L278>
  // preconditions: 0 <= e < 2^255 (not necessarily e < order), fe_invert(0) = 0
  fe_frombytes(&x1, point);
  fe_1(&x2);
  fe_0(&z2);
  fe_copy(&x3, &x1);
  fe_1(&z3);

  unsigned swap = 0;
  int pos;
  for (pos = 254; pos >= 0; --pos) {
    // loop invariant as of right before the test, for the case where x1 != 0:
    //   pos >= -1; if z2 = 0 then x2 is nonzero; if z3 = 0 then x3 is nonzero
    //   let r := e >> (pos+1) in the following equalities of projective points:
    //   to_xz (r*P)     === if swap then (x3, z3) else (x2, z2)
    //   to_xz ((r+1)*P) === if swap then (x2, z2) else (x3, z3)
    //   x1 is the nonzero x coordinate of the nonzero point (r*P-(r+1)*P)
    unsigned b = 1 & (e[pos / 8] >> (pos & 7));
    swap ^= b;
    fe_cswap(&x2, &x3, swap);
    fe_cswap(&z2, &z3, swap);
    swap = b;
    // Coq transcription of ladderstep formula (called from transcribed loop):
    // <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Curves/Montgomery/XZ.v#L89>
    // <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Curves/Montgomery/XZProofs.v#L131>
    // x1 != 0 <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Curves/Montgomery/XZProofs.v#L217>
    // x1  = 0 <https://github.com/mit-plv/fiat-crypto/blob/2456d821825521f7e03e65882cc3521795b0320f/src/Curves/Montgomery/XZProofs.v#L147>
    fe_sub(&tmp0l, &x3, &z3);
    fe_sub(&tmp1l, &x2, &z2);
    fe_add(&x2l, &x2, &z2);
    fe_add(&z2l, &x3, &z3);
    fe_mul_tll(&z3, &tmp0l, &x2l);
    fe_mul_tll(&z2, &z2l, &tmp1l);
    fe_sq_tl(&tmp0, &tmp1l);
    fe_sq_tl(&tmp1, &x2l);
    fe_add(&x3l, &z3, &z2);
    fe_sub(&z2l, &z3, &z2);
    fe_mul_ttt(&x2, &tmp1, &tmp0);
    fe_sub(&tmp1l, &tmp1, &tmp0);
    fe_sq_tl(&z2, &z2l);
    fe_mul121666(&z3, &tmp1l);
    fe_sq_tl(&x3, &x3l);
    fe_add(&tmp0l, &tmp0, &z3);
    fe_mul_ttt(&z3, &x1, &z2);
    fe_mul_tll(&z2, &tmp1l, &tmp0l);
  }
  // here pos=-1, so r=e, so to_xz (e*P) === if swap then (x3, z3) else (x2, z2)
  fe_cswap(&x2, &x3, swap);
  fe_cswap(&z2, &z3, swap);

  fe_invert(&z2, &z2);
  fe_mul_ttt(&x2, &x2, &z2);
  fe_tobytes(out, &x2);
}

int X25519(uint8_t out_shared_key[32], const uint8_t private_key[32],
           const uint8_t peer_public_value[32]) {
  static const uint8_t kZeros[32] = {0};
  x25519_scalar_mult_generic(out_shared_key, private_key, peer_public_value);
  // The all-zero output results when the input is a point of small order.
  return CRYPTO_memcmp(kZeros, out_shared_key, 32) != 0;
}
