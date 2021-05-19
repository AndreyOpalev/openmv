/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * AprilTags library.
 */
#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include "imlib.h"

// Enable new code optimizations
#define OPTIMIZED

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/
#define fprintf(format, ...)
#define free(ptr) ({ umm_free(ptr); })
#define malloc(size) ({ void *_r = umm_malloc(size); if(!_r) umm_alloc_fail(); _r; })
#define realloc(ptr, size) ({ void *_r = umm_realloc((ptr), (size)); if(!_r) umm_alloc_fail(); _r; })
#define calloc(num, item_size) ({ void *_r = umm_calloc((num), (item_size)); if(!_r) umm_alloc_fail(); _r; })
#define assert(expression)
#define sqrt(x) fast_sqrtf(x)
#define sqrtf(x) fast_sqrtf(x)
#define floor(x) fast_floorf(x)
#define floorf(x) fast_floorf(x)
#define ceil(x) fast_ceilf(x)
#define ceilf(x) fast_ceilf(x)
#define round(x) fast_roundf(x)
#define roundf(x) fast_roundf(x)
#define atan(x) fast_atanf(x)
#define atanf(x) fast_atanf(x)
#define atan2(y, x) fast_atan2f((y), (x))
#define atan2f(y, x) fast_atan2f((y), (x))
#define exp(x) fast_expf(x)
#define expf(x) fast_expf(x)
#define cbrt(x) fast_cbrtf(x)
#define cbrtf(x) fast_cbrtf(x)
#define fabs(x) fast_fabsf(x)
#define fabsf(x) fast_fabsf(x)
#define log(x) fast_log(x)
#define logf(x) fast_log(x)
#undef log2
#define log2(x) fast_log2(x)
#undef log2f
#define log2f(x) fast_log2(x)
#define sin(x) arm_sin_f32(x)
#define cos(x) arm_cos_f32(x)
#define fmin(a, b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define fminf(a, b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define fmax(a, b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define fmaxf(a, b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "zarray.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a structure which acts as a resize-able array ala Java's ArrayList.
 */
typedef struct zarray zarray_t;
struct zarray
{
    size_t el_sz; // size of each element

    int size; // how many elements?
    int alloc; // we've allocated storage for how many elements?
    char *data;
};

/**
 * Creates and returns a variable array structure capable of holding elements of
 * the specified size. It is the caller's responsibility to call zarray_destroy()
 * on the returned array when it is no longer needed.
 */
static inline zarray_t *zarray_create(size_t el_sz)
{
    assert(el_sz > 0);

    zarray_t *za = (zarray_t*) calloc(1, sizeof(zarray_t));
    za->el_sz = el_sz;
    return za;
}

/**
 * Creates and returns a variable array structure capable of holding elements of
 * the specified size. It is the caller's responsibility to call zarray_destroy()
 * on the returned array when it is no longer needed.
 */
static inline zarray_t *zarray_create_fail_ok(size_t el_sz)
{
    assert(el_sz > 0);

    zarray_t *za = (zarray_t*) umm_calloc(1, sizeof(zarray_t));
    if (za) za->el_sz = el_sz;
    return za;
}

/**
 * Frees all resources associated with the variable array structure which was
 * created by zarray_create(). After calling, 'za' will no longer be valid for storage.
 */
static inline void zarray_destroy(zarray_t *za)
{
    if (za == NULL)
        return;

    if (za->data != NULL)
        free(za->data);
    memset(za, 0, sizeof(zarray_t));
    free(za);
}

/** Allocate a new zarray that contains a copy of the data in the argument. **/
static inline zarray_t *zarray_copy(const zarray_t *za)
{
    assert(za != NULL);

    zarray_t *zb = (zarray_t*) calloc(1, sizeof(zarray_t));
    zb->el_sz = za->el_sz;
    zb->size = za->size;
    zb->alloc = za->alloc;
    zb->data = (char*) malloc(zb->alloc * zb->el_sz);
    memcpy(zb->data, za->data, za->size * za->el_sz);
    return zb;
}

static int iceillog2(int v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

/**
 * Allocate a new zarray that contains a subset of the original
 * elements. NOTE: end index is EXCLUSIVE, that is one past the last
 * element you want.
 */
static inline zarray_t *zarray_copy_subset(const zarray_t *za,
                             int start_idx,
                             int end_idx_exclusive)
{
    zarray_t *out = (zarray_t*) calloc(1, sizeof(zarray_t));
    out->el_sz = za->el_sz;
    out->size = end_idx_exclusive - start_idx;
    out->alloc = iceillog2(out->size); // round up pow 2
    out->data = (char*) malloc(out->alloc * out->el_sz);
    memcpy(out->data,  za->data +(start_idx*out->el_sz), out->size*out->el_sz);
    return out;
}

/**
 * Retrieves the number of elements currently being contained by the passed
 * array, which may be different from its capacity. The index of the last element
 * in the array will be one less than the returned value.
 */
static inline int zarray_size(const zarray_t *za)
{
    assert(za != NULL);

    return za->size;
}

/**
 * Returns 1 if zarray_size(za) == 0,
 * returns 0 otherwise.
 */
/*
JUST CALL zarray_size
int zarray_isempty(const zarray_t *za)
{
    assert(za != NULL);
    if (za->size <= 0)
        return 1;
    else
        return 0;
}
*/


/**
 * Allocates enough internal storage in the supplied variable array structure to
 * guarantee that the supplied number of elements (capacity) can be safely stored.
 */
static inline void zarray_ensure_capacity(zarray_t *za, int capacity)
{
    assert(za != NULL);

    if (capacity <= za->alloc)
        return;

    while (za->alloc < capacity) {
        za->alloc += 8; // use less memory // *= 2;
        if (za->alloc < 8)
            za->alloc = 8;
    }

    za->data = (char*) realloc(za->data, za->alloc * za->el_sz);
}

/**
 * Adds a new element to the end of the supplied array, and sets its value
 * (by copying) from the data pointed to by the supplied pointer 'p'.
 * Automatically ensures that enough storage space is available for the new element.
 */
static inline void zarray_add(zarray_t *za, const void *p)
{
    assert(za != NULL);
    assert(p != NULL);

    zarray_ensure_capacity(za, za->size + 1);

    memcpy(&za->data[za->size*za->el_sz], p, za->el_sz);
    za->size++;
}

/**
 * Adds a new element to the end of the supplied array, and sets its value
 * (by copying) from the data pointed to by the supplied pointer 'p'.
 * Automatically ensures that enough storage space is available for the new element.
 */
static inline void zarray_add_fail_ok(zarray_t *za, const void *p)
{
    assert(za != NULL);
    assert(p != NULL);

    if ((za->size + 1) > za->alloc)
    {
        char *old_data = za->data;
        int old_alloc = za->alloc;

        while (za->alloc < (za->size + 1)) {
            za->alloc += 8; // use less memory // *= 2;
            if (za->alloc < 8)
                za->alloc = 8;
        }

        za->data = (char*) umm_realloc(za->data, za->alloc * za->el_sz);

        if (!za->data) {
            za->data = old_data;
            za->alloc = old_alloc;
            return;
        }
    }

    memcpy(&za->data[za->size*za->el_sz], p, za->el_sz);
    za->size++;
}

/**
 * Retrieves the element from the supplied array located at the zero-based
 * index of 'idx' and copies its value into the variable pointed to by the pointer
 * 'p'.
 */
static inline void zarray_get(const zarray_t *za, int idx, void *p)
{
    assert(za != NULL);
    assert(p != NULL);
    assert(idx >= 0);
    assert(idx < za->size);

    memcpy(p, &za->data[idx*za->el_sz], za->el_sz);
}

/**
 * Similar to zarray_get(), but returns a "live" pointer to the internal
 * storage, avoiding a memcpy. This pointer is not valid across
 * operations which might move memory around (i.e. zarray_remove_value(),
 * zarray_remove_index(), zarray_insert(), zarray_sort(), zarray_clear()).
 * 'p' should be a pointer to the pointer which will be set to the internal address.
 */
inline static void zarray_get_volatile(const zarray_t *za, int idx, void *p)
{
    assert(za != NULL);
    assert(p != NULL);
    assert(idx >= 0);
    assert(idx < za->size);

    *((void**) p) = &za->data[idx*za->el_sz];
}

inline static void zarray_truncate(zarray_t *za, int sz)
{
   assert(za != NULL);
   assert(sz <= za->size);
   za->size = sz;
}

/**
 * Copies the memory array used internally by zarray to store its owned
 * elements to the address pointed by 'buffer'. It is the caller's responsibility
 * to allocate zarray_size()*el_sz bytes for the copy to be stored and
 * to free the memory when no longer needed. The memory allocated at 'buffer'
 * and the internal zarray storage must not overlap. 'buffer_bytes' should be
 * the size of the 'buffer' memory space, in bytes, and must be at least
 * zarray_size()*el_sz.
 *
 * Returns the number of bytes copied into 'buffer'.
 */
static inline size_t zarray_copy_data(const zarray_t *za, void *buffer, size_t buffer_bytes)
{
    assert(za != NULL);
    assert(buffer != NULL);
    assert(buffer_bytes >= za->el_sz * za->size);
    memcpy(buffer, za->data, za->el_sz * za->size);
    return za->el_sz * za->size;
}

/**
 * Removes the entry at index 'idx'.
 * If shuffle is true, the last element in the array will be placed in
 * the newly-open space; if false, the zarray is compacted.
 */
static inline void zarray_remove_index(zarray_t *za, int idx, int shuffle)
{
    assert(za != NULL);
    assert(idx >= 0);
    assert(idx < za->size);

    if (shuffle) {
        if (idx < za->size-1)
            memcpy(&za->data[idx*za->el_sz], &za->data[(za->size-1)*za->el_sz], za->el_sz);
        za->size--;
        return;
    } else {
        // size = 10, idx = 7. Should copy 2 entries (at idx=8 and idx=9).
        // size = 10, idx = 9. Should copy 0 entries.
        int ncopy = za->size - idx - 1;
        if (ncopy > 0)
            memmove(&za->data[idx*za->el_sz], &za->data[(idx+1)*za->el_sz], ncopy*za->el_sz);
        za->size--;
        return;
    }
}

/**
 * Remove the entry whose value is equal to the value pointed to by 'p'.
 * If shuffle is true, the last element in the array will be placed in
 * the newly-open space; if false, the zarray is compacted. At most
 * one element will be removed.
 *
 * Note that objects will be compared using memcmp over the full size
 * of the value. If the value is a struct that contains padding,
 * differences in the padding bytes can cause comparisons to
 * fail. Thus, it remains best practice to bzero all structs so that
 * the padding is set to zero.
 *
 * Returns the number of elements removed (0 or 1).
 */
// remove the entry whose value is equal to the value pointed to by p.
// if shuffle is true, the last element in the array will be placed in
// the newly-open space; if false, the zarray is compacted.
static inline int zarray_remove_value(zarray_t *za, const void *p, int shuffle)
{
    assert(za != NULL);
    assert(p != NULL);

    for (int idx = 0; idx < za->size; idx++) {
        if (!memcmp(p, &za->data[idx*za->el_sz], za->el_sz)) {
            zarray_remove_index(za, idx, shuffle);
            return 1;
        }
    }

    return 0;
}


/**
 * Creates a new entry and inserts it into the array so that it will have the
 * index 'idx' (i.e. before the item which currently has that index). The value
 * of the new entry is set to (copied from) the data pointed to by 'p'. 'idx'
 * can be one larger than the current max index to place the new item at the end
 * of the array, or zero to add it to an empty array.
 */
static inline void zarray_insert(zarray_t *za, int idx, const void *p)
{
    assert(za != NULL);
    assert(p != NULL);
    assert(idx >= 0);
    assert(idx <= za->size);

    zarray_ensure_capacity(za, za->size + 1);
    // size = 10, idx = 7. Should copy three entries (idx=7, idx=8, idx=9)
    int ncopy = za->size - idx;

    memmove(&za->data[(idx+1)*za->el_sz], &za->data[idx*za->el_sz], ncopy*za->el_sz);
    memcpy(&za->data[idx*za->el_sz], p, za->el_sz);

    za->size++;
}


/**
 * Sets the value of the current element at index 'idx' by copying its value from
 * the data pointed to by 'p'. The previous value of the changed element will be
 * copied into the data pointed to by 'outp' if it is not null.
 */
static inline void zarray_set(zarray_t *za, int idx, const void *p, void *outp)
{
    assert(za != NULL);
    assert(p != NULL);
    assert(idx >= 0);
    assert(idx < za->size);

    if (outp != NULL)
        memcpy(outp, &za->data[idx*za->el_sz], za->el_sz);

    memcpy(&za->data[idx*za->el_sz], p, za->el_sz);
}

/**
 * Calls the supplied function for every element in the array in index order.
 * The map function will be passed a pointer to each element in turn and must
 * have the following format:
 *
 * void map_function(element_type *element)
 */
static inline void zarray_map(zarray_t *za, void (*f)(void*))
{
    assert(za != NULL);
    assert(f != NULL);

    for (int idx = 0; idx < za->size; idx++)
        f(&za->data[idx*za->el_sz]);
}

/**
 * Calls the supplied function for every element in the array in index order.
 * HOWEVER values are passed to the function, not pointers to values. In the
 * case where the zarray stores object pointers, zarray_vmap allows you to
 * pass in the object's destroy function (or free) directly. Can only be used
 * with zarray's which contain pointer data. The map function should have the
 * following format:
 *
 * void map_function(element_type *element)
 */
    void zarray_vmap(zarray_t *za, void (*f)());

/**
 * Removes all elements from the array and sets its size to zero. Pointers to
 * any data elements obtained i.e. by zarray_get_volatile() will no longer be
 * valid.
 */
static inline void zarray_clear(zarray_t *za)
{
    assert(za != NULL);
    za->size = 0;
}

/**
 * Determines whether any element in the array has a value which matches the
 * data pointed to by 'p'.
 *
 * Returns 1 if a match was found anywhere in the array, else 0.
 */
static inline int zarray_contains(const zarray_t *za, const void *p)
{
    assert(za != NULL);
    assert(p != NULL);

    for (int idx = 0; idx < za->size; idx++) {
        if (!memcmp(p, &za->data[idx*za->el_sz], za->el_sz)) {
            return 1;
        }
    }

    return 0;
}

/**
 * Uses qsort() to sort the elements contained by the array in ascending order.
 * Uses the supplied comparison function to determine the appropriate order.
 *
 * The comparison function will be passed a pointer to two elements to be compared
 * and should return a measure of the difference between them (see strcmp()).
 * I.e. it should return a negative number if the first element is 'less than'
 * the second, zero if they are equivalent, and a positive number if the first
 * element is 'greater than' the second. The function should have the following format:
 *
 * int comparison_function(const element_type *first, const element_type *second)
 *
 * zstrcmp() can be used as the comparison function for string elements, which
 * will call strcmp() internally.
 */
static inline void zarray_sort(zarray_t *za, int (*compar)(const void*, const void*))
{
    assert(za != NULL);
    assert(compar != NULL);
    if (za->size == 0)
        return;

    qsort(za->data, za->size, za->el_sz, compar);
}

/**
 * A comparison function for comparing strings which can be used by zarray_sort()
 * to sort arrays with char* elements.
 */
    int zstrcmp(const void * a_pp, const void * b_pp);

/**
  * Find the index of an element, or return -1 if not found. Remember that p is
  * a pointer to the element.
 **/
// returns -1 if not in array. Remember p is a pointer to the item.
static inline int zarray_index_of(const zarray_t *za, const void *p)
{
    assert(za != NULL);
    assert(p != NULL);

    for (int i = 0; i < za->size; i++) {
        if (!memcmp(p, &za->data[i*za->el_sz], za->el_sz))
            return i;
    }

    return -1;
}



/**
 * Add all elements from 'source' into 'dest'. el_size must be the same
 * for both lists
 **/
static inline void zarray_add_all(zarray_t * dest, const zarray_t * source)
{
    assert(dest->el_sz == source->el_sz);

    // Don't allocate on stack because el_sz could be larger than ~8 MB
    // stack size
    char *tmp = (char*)calloc(1, dest->el_sz);

    for (int i = 0; i < zarray_size(source); i++) {
        zarray_get(source, i, tmp);
        zarray_add(dest, tmp);
   }

    free(tmp);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "zarray.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

int zstrcmp(const void * a_pp, const void * b_pp)
{
    assert(a_pp != NULL);
    assert(b_pp != NULL);

    char * a = *(void**)a_pp;
    char * b = *(void**)b_pp;

    return strcmp(a,b);
}

void zarray_vmap(zarray_t *za, void (*f)())
{
    assert(za != NULL);
    assert(f != NULL);
    assert(za->el_sz == sizeof(void*));

    for (int idx = 0; idx < za->size; idx++) {
        void *pp = &za->data[idx*za->el_sz];
        void *p = *(void**) pp;
        f(p);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "math_util.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_TWOPI
# define M_TWOPI       6.2831853071795862319959  /* 2*pi */
#endif

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

#define to_radians(x) ( (x) * (M_PI / 180.0 ))
#define to_degrees(x) ( (x) * (180.0 / M_PI ))

#define max(A, B) (A < B ? B : A)
#define min(A, B) (A < B ? A : B)

  /* DEPRECATE, threshold meaningless without context.
static inline int dequals(float a, float b)
{
    float thresh = 1e-9;
    return (fabs(a-b) < thresh);
}
  */

static inline int dequals_mag(float a, float b, float thresh)
{
    return (fabs(a-b) < thresh);
}

static inline int isq(int v)
{
    return v*v;
}

static inline float fsq(float v)
{
    return v*v;
}

static inline float sq(float v)
{
    return v*v;
}

static inline float sgn(float v)
{
    return (v>=0) ? 1 : -1;
}

// random number between [0, 1)
static inline float randf()
{
    return ((float) rand()) / (RAND_MAX + 1.0);
}


static inline float signed_randf()
{
    return randf()*2 - 1;
}

// return a random integer between [0, bound)
static inline int irand(int bound)
{
    int v = (int) (randf()*bound);
    if (v == bound)
        return (bound-1);
    //assert(v >= 0);
    //assert(v < bound);
    return v;
}

/** Map vin to [0, 2*PI) **/
static inline float mod2pi_positive(float vin)
{
    return vin - M_TWOPI * floor(vin / M_TWOPI);
}

/** Map vin to [-PI, PI) **/
static inline float mod2pi(float vin)
{
    return mod2pi_positive(vin + M_PI) - M_PI;
}

/** Return vin such that it is within PI degrees of ref **/
static inline float mod2pi_ref(float ref, float vin)
{
    return ref + mod2pi(vin - ref);
}

/** Map vin to [0, 360) **/
static inline float mod360_positive(float vin)
{
    return vin - 360 * floor(vin / 360);
}

/** Map vin to [-180, 180) **/
static inline float mod360(float vin)
{
    return mod360_positive(vin + 180) - 180;
}

static inline int mod_positive(int vin, int mod) {
    return (vin % mod + mod) % mod;
}

static inline int theta_to_int(float theta, int max)
{
    theta = mod2pi_ref(M_PI, theta);
    int v = (int) (theta / M_TWOPI * max);

    if (v == max)
        v = 0;

    assert (v >= 0 && v < max);

    return v;
}

static inline int imin(int a, int b)
{
    return (a < b) ? a : b;
}

static inline int imax(int a, int b)
{
    return (a > b) ? a : b;
}

static inline int64_t imin64(int64_t a, int64_t b)
{
    return (a < b) ? a : b;
}

static inline int64_t imax64(int64_t a, int64_t b)
{
    return (a > b) ? a : b;
}

static inline int iclamp(int v, int minv, int maxv)
{
    return imax(minv, imin(v, maxv));
}

static inline float dclamp(float a, float min, float max)
{
    if (a < min)
        return min;
    if (a > max)
        return max;
    return a;
}

static inline int fltcmp (float f1, float f2)
{
    float epsilon = f1-f2;
    if (epsilon < 0.0)
        return -1;
    else if (epsilon > 0.0)
        return  1;
    else
        return  0;
}

static inline int dblcmp (float d1, float d2)
{
    float epsilon = d1-d2;
    if (epsilon < 0.0)
        return -1;
    else if (epsilon > 0.0)
        return  1;
    else
        return  0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "svd22.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

void svd22(const float A[4], float U[4], float S[2], float V[4]);

// for the matrix [a b; b d]
void svd_sym_singular_values(float A00, float A01, float A11,
                             float *Lmin, float *Lmax);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "svd22.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

/** SVD 2x2.

    Computes singular values and vectors without squaring the input
    matrix. With double precision math, results are accurate to about
    1E-16.

    U = [ cos(theta) -sin(theta) ]
        [ sin(theta)  cos(theta) ]

    S = [ e  0 ]
        [ 0  f ]

    V = [ cos(phi)   -sin(phi) ]
        [ sin(phi)   cos(phi)  ]


    Our strategy is basically to analytically multiply everything out
    and then rearrange so that we can solve for theta, phi, e, and
    f. (Derivation by ebolson@umich.edu 5/2016)

   V' = [ CP  SP ]
        [ -SP CP ]

USV' = [ CT -ST ][  e*CP  e*SP ]
       [ ST  CT ][ -f*SP  f*CP ]

     = [e*CT*CP + f*ST*SP     e*CT*SP - f*ST*CP ]
       [e*ST*CP - f*SP*CT     e*SP*ST + f*CP*CT ]

A00+A11 = e*CT*CP + f*ST*SP + e*SP*ST + f*CP*CT
        = e*(CP*CT + SP*ST) + f*(SP*ST + CP*CT)
        = (e+f)(CP*CT + SP*ST)
B0	    = (e+f)*cos(P-T)

A00-A11 = e*CT*CP + f*ST*SP - e*SP*ST - f*CP*CT
        = e*(CP*CT - SP*ST) - f*(-ST*SP + CP*CT)
        = (e-f)(CP*CT - SP*ST)
B1	    = (e-f)*cos(P+T)

A01+A10 = e*CT*SP - f*ST*CP + e*ST*CP - f*SP*CT
        = e(CT*SP + ST*CP) - f*(ST*CP + SP*CT)
        = (e-f)*(CT*SP + ST*CP)
B2	    = (e-f)*sin(P+T)

A01-A10 = e*CT*SP - f*ST*CP - e*ST*CP + f*SP*CT
    = e*(CT*SP - ST*CP) + f(SP*CT - ST*CP)
    = (e+f)*(CT*SP - ST*CP)
B3	= (e+f)*sin(P-T)

B0 = (e+f)*cos(P-T)
B1 = (e-f)*cos(P+T)
B2 = (e-f)*sin(P+T)
B3 = (e+f)*sin(P-T)

B3/B0 = tan(P-T)

B2/B1 = tan(P+T)
 **/
void svd22(const float A[4], float U[4], float S[2], float V[4])
{
    float A00 = A[0];
    float A01 = A[1];
    float A10 = A[2];
    float A11 = A[3];

    float B0 = A00 + A11;
    float B1 = A00 - A11;
    float B2 = A01 + A10;
    float B3 = A01 - A10;

    float PminusT = atan2(B3, B0);
    float PplusT = atan2(B2, B1);

    float P = (PminusT + PplusT) / 2;
    float T = (-PminusT + PplusT) / 2;

    float CP = cos(P), SP = sin(P);
    float CT = cos(T), ST = sin(T);

    U[0] = CT;
    U[1] = -ST;
    U[2] = ST;
    U[3] = CT;

    V[0] = CP;
    V[1] = -SP;
    V[2] = SP;
    V[3] = CP;

    // C0 = e+f. There are two ways to compute C0; we pick the one
    // that is better conditioned.
    float CPmT = cos(P-T), SPmT = sin(P-T);
    float C0 = 0;
    if (fabs(CPmT) > fabs(SPmT))
        C0 = B0 / CPmT;
    else
        C0 = B3 / SPmT;

    // C1 = e-f. There are two ways to compute C1; we pick the one
    // that is better conditioned.
    float CPpT = cos(P+T), SPpT = sin(P+T);
    float C1 = 0;
    if (fabs(CPpT) > fabs(SPpT))
        C1 = B1 / CPpT;
    else
        C1 = B2 / SPpT;

    // e and f are the singular values
    float e = (C0 + C1) / 2;
    float f = (C0 - C1) / 2;

    if (e < 0) {
        e = -e;
        U[0] = -U[0];
        U[2] = -U[2];
    }

    if (f < 0) {
        f = -f;
        U[1] = -U[1];
        U[3] = -U[3];
    }

    // sort singular values.
    if (e > f) {
        // already in big-to-small order.
        S[0] = e;
        S[1] = f;
    } else {
        // Curiously, this code never seems to get invoked.  Why is it
        // that S[0] always ends up the dominant vector?  However,
        // this code has been tested (flipping the logic forces us to
        // sort the singular values in ascending order).
        //
        // P = [ 0 1 ; 1 0 ]
        // USV' = (UP)(PSP)(PV')
        //      = (UP)(PSP)(VP)'
        //      = (UP)(PSP)(P'V')'
        S[0] = f;
        S[1] = e;

        // exchange columns of U and V
        float tmp[2];
        tmp[0] = U[0];
        tmp[1] = U[2];
        U[0] = U[1];
        U[2] = U[3];
        U[1] = tmp[0];
        U[3] = tmp[1];

        tmp[0] = V[0];
        tmp[1] = V[2];
        V[0] = V[1];
        V[2] = V[3];
        V[1] = tmp[0];
        V[3] = tmp[1];
    }

    /*
    float SM[4] = { S[0], 0, 0, S[1] };

    doubles_print_mat(U, 2, 2, "%20.10g");
    doubles_print_mat(SM, 2, 2, "%20.10g");
    doubles_print_mat(V, 2, 2, "%20.10g");
    printf("A:\n");
    doubles_print_mat(A, 2, 2, "%20.10g");

    float SVt[4];
    doubles_mat_ABt(SM, 2, 2, V, 2, 2, SVt, 2, 2);
    float USVt[4];
    doubles_mat_AB(U, 2, 2, SVt, 2, 2, USVt, 2, 2);

    printf("USVt\n");
    doubles_print_mat(USVt, 2, 2, "%20.10g");

    float diff[4];
    for (int i = 0; i < 4; i++)
        diff[i] = A[i] - USVt[i];

    printf("diff\n");
    doubles_print_mat(diff, 2, 2, "%20.10g");

    */

}


// for the matrix [a b; b d]
void svd_sym_singular_values(float A00, float A01, float A11,
                             float *Lmin, float *Lmax)
{
    float A10 = A01;

    float B0 = A00 + A11;
    float B1 = A00 - A11;
    float B2 = A01 + A10;
    float B3 = A01 - A10;

    float PminusT = atan2(B3, B0);
    float PplusT = atan2(B2, B1);

    float P = (PminusT + PplusT) / 2;
    float T = (-PminusT + PplusT) / 2;

    // C0 = e+f. There are two ways to compute C0; we pick the one
    // that is better conditioned.
    float CPmT = cos(P-T), SPmT = sin(P-T);
    float C0 = 0;
    if (fabs(CPmT) > fabs(SPmT))
        C0 = B0 / CPmT;
    else
        C0 = B3 / SPmT;

    // C1 = e-f. There are two ways to compute C1; we pick the one
    // that is better conditioned.
    float CPpT = cos(P+T), SPpT = sin(P+T);
    float C1 = 0;
    if (fabs(CPpT) > fabs(SPpT))
        C1 = B1 / CPpT;
    else
        C1 = B2 / SPpT;

    // e and f are the singular values
    float e = (C0 + C1) / 2;
    float f = (C0 - C1) / 2;

    *Lmin = fmin(e, f);
    *Lmax = fmax(e, f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "matd.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a matrix structure for holding float-precision values with
 * data in row-major order (i.e. index = row*ncols + col).
 *
 * nrows and ncols are 1-based counts with the exception that a scalar (non-matrix)
 *   is represented with nrows=0 and/or ncols=0.
 */
typedef struct
{
    unsigned int nrows, ncols;
    float data[];
//    float *data;
} matd_t;

#define MATD_ALLOC(name, nrows, ncols) float name ## _storage [nrows*ncols]; matd_t name = { .nrows = nrows, .ncols = ncols, .data = &name ## _storage };

/**
 * Defines a small value which can be used in place of zero for approximating
 * calculations which are singular at zero values (i.e. inverting a matrix with
 * a zero or near-zero determinant).
 */
#define MATD_EPS 1e-8

/**
 * A macro to reference a specific matd_t data element given it's zero-based
 * row and column indexes. Suitable for both retrieval and assignment.
 */
#define MATD_EL(m, row, col) (m)->data[((row)*(m)->ncols + (col))]

/**
 * Creates a float matrix with the given number of rows and columns (or a scalar
 * in the case where rows=0 and/or cols=0). All data elements will be initialized
 * to zero. It is the caller's responsibility to call matd_destroy() on the
 * returned matrix.
 */
matd_t *matd_create(int rows, int cols);

/**
 * Creates a float matrix with the given number of rows and columns (or a scalar
 * in the case where rows=0 and/or cols=0). All data elements will be initialized
 * using the supplied array of data, which must contain at least rows*cols elements,
 * arranged in row-major order (i.e. index = row*ncols + col). It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 */
matd_t *matd_create_data(int rows, int cols, const float *data);

/**
 * Creates a float matrix with the given number of rows and columns (or a scalar
 * in the case where rows=0 and/or cols=0). All data elements will be initialized
 * using the supplied array of float data, which must contain at least rows*cols elements,
 * arranged in row-major order (i.e. index = row*ncols + col). It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 */
matd_t *matd_create_dataf(int rows, int cols, const float *data);

/**
 * Creates a square identity matrix with the given number of rows (and
 * therefore columns), or a scalar with value 1 in the case where dim=0.
 * It is the caller's responsibility to call matd_destroy() on the
 * returned matrix.
 */
matd_t *matd_identity(int dim);

/**
 * Creates a scalar with the supplied value 'v'. It is the caller's responsibility
 * to call matd_destroy() on the returned matrix.
 *
 * NOTE: Scalars are different than 1x1 matrices (implementation note:
 * they are encoded as 0x0 matrices). For example: for matrices A*B, A
 * and B must both have specific dimensions. However, if A is a
 * scalar, there are no restrictions on the size of B.
 */
matd_t *matd_create_scalar(float v);

/**
 * Retrieves the cell value for matrix 'm' at the given zero-based row and column index.
 * Performs more thorough validation checking than MATD_EL().
 */
float matd_get(const matd_t *m, int row, int col);

/**
 * Assigns the given value to the matrix cell at the given zero-based row and
 * column index. Performs more thorough validation checking than MATD_EL().
 */
void matd_put(matd_t *m, int row, int col, float value);

/**
 * Retrieves the scalar value of the given element ('m' must be a scalar).
 * Performs more thorough validation checking than MATD_EL().
 */
float matd_get_scalar(const matd_t *m);

/**
 * Assigns the given value to the supplied scalar element ('m' must be a scalar).
 * Performs more thorough validation checking than MATD_EL().
 */
void matd_put_scalar(matd_t *m, float value);

/**
 * Creates an exact copy of the supplied matrix 'm'. It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 */
matd_t *matd_copy(const matd_t *m);

/**
 * Creates a copy of a subset of the supplied matrix 'a'. The subset will include
 * rows 'r0' through 'r1', inclusive ('r1' >= 'r0'), and columns 'c0' through 'c1',
 * inclusive ('c1' >= 'c0'). All parameters are zero-based (i.e. matd_select(a, 0, 0, 0, 0)
 * will return only the first cell). Cannot be used on scalars or to extend
 * beyond the number of rows/columns of 'a'. It is the caller's  responsibility to
 * call matd_destroy() on the returned matrix.
 */
matd_t *matd_select(const matd_t *a, int r0, int r1, int c0, int c1);

/**
 * Prints the supplied matrix 'm' to standard output by applying the supplied
 * printf format specifier 'fmt' for each individual element. Each row will
 * be printed on a separate newline.
 */
void matd_print(const matd_t *m, const char *fmt);

/**
 * Prints the transpose of the supplied matrix 'm' to standard output by applying
 * the supplied printf format specifier 'fmt' for each individual element. Each
 * row will be printed on a separate newline.
 */
void matd_print_transpose(const matd_t *m, const char *fmt);

/**
 * Adds the two supplied matrices together, cell-by-cell, and returns the results
 * as a new matrix of the same dimensions. The supplied matrices must have
 * identical dimensions.  It is the caller's responsibility to call matd_destroy()
 * on the returned matrix.
 */
matd_t *matd_add(const matd_t *a, const matd_t *b);

/**
 * Adds the values of 'b' to matrix 'a', cell-by-cell, and overwrites the
 * contents of 'a' with the results. The supplied matrices must have
 * identical dimensions.
 */
void matd_add_inplace(matd_t *a, const matd_t *b);

/**
 * Subtracts matrix 'b' from matrix 'a', cell-by-cell, and returns the results
 * as a new matrix of the same dimensions. The supplied matrices must have
 * identical dimensions.  It is the caller's responsibility to call matd_destroy()
 * on the returned matrix.
 */
matd_t *matd_subtract(const matd_t *a, const matd_t *b);

/**
 * Subtracts the values of 'b' from matrix 'a', cell-by-cell, and overwrites the
 * contents of 'a' with the results. The supplied matrices must have
 * identical dimensions.
 */
void matd_subtract_inplace(matd_t *a, const matd_t *b);

/**
 * Scales all cell values of matrix 'a' by the given scale factor 's' and
 * returns the result as a new matrix of the same dimensions. It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 */
matd_t *matd_scale(const matd_t *a, float s);

/**
 * Scales all cell values of matrix 'a' by the given scale factor 's' and
 * overwrites the contents of 'a' with the results.
 */
void matd_scale_inplace(matd_t *a, float s);

/**
 * Multiplies the two supplied matrices together (matrix product), and returns the
 * results as a new matrix. The supplied matrices must have dimensions such that
 * columns(a) = rows(b). The returned matrix will have a row count of rows(a)
 * and a column count of columns(b). It is the caller's responsibility to call
 * matd_destroy() on the returned matrix.
 */
matd_t *matd_multiply(const matd_t *a, const matd_t *b);

/**
 * Creates a matrix which is the transpose of the supplied matrix 'a'. It is the
 * caller's responsibility to call matd_destroy() on the returned matrix.
 */
matd_t *matd_transpose(const matd_t *a);

/**
 * Calculates the determinant of the supplied matrix 'a'.
 */
float matd_det(const matd_t *a);

/**
 * Attempts to compute an inverse of the supplied matrix 'a' and return it as
 * a new matrix. This is strictly only possible if the determinant of 'a' is
 * non-zero (matd_det(a) != 0).
 *
 * If the determinant is zero, NULL is returned. It is otherwise the
 * caller's responsibility to cope with the results caused by poorly
 * conditioned matrices. (E.g.., if such a situation is likely to arise, compute
 * the pseudo-inverse from the SVD.)
 **/
matd_t *matd_inverse(const matd_t *a);

static inline void matd_set_data(matd_t *m, const float *data)
{
    memcpy(m->data, data, m->nrows * m->ncols * sizeof(float));
}

/**
 * Determines whether the supplied matrix 'a' is a scalar (positive return) or
 * not (zero return, indicating a matrix of dimensions at least 1x1).
 */
static inline int matd_is_scalar(const matd_t *a)
{
    assert(a != NULL);
    return a->ncols == 0 || a->nrows == 0;
}

/**
 * Determines whether the supplied matrix 'a' is a row or column vector
 * (positive return) or not (zero return, indicating either 'a' is a scalar or a
 * matrix with at least one dimension > 1).
 */
static inline int matd_is_vector(const matd_t *a)
{
    assert(a != NULL);
    return a->ncols == 1 || a->nrows == 1;
}

/**
 * Determines whether the supplied matrix 'a' is a row or column vector
 * with a dimension of 'len' (positive return) or not (zero return).
 */
static inline int matd_is_vector_len(const matd_t *a, int len)
{
    assert(a != NULL);
    return (a->ncols == 1 && a->nrows == len) || (a->ncols == len && a->nrows == 1);
}

/**
 * Calculates the magnitude of the supplied matrix 'a'.
 */
float matd_vec_mag(const matd_t *a);

/**
 * Calculates the magnitude of the distance between the points represented by
 * matrices 'a' and 'b'. Both 'a' and 'b' must be vectors and have the same
 * dimension (although one may be a row vector and one may be a column vector).
 */
float matd_vec_dist(const matd_t *a, const matd_t *b);


/**
 * Same as matd_vec_dist, but only uses the first 'n' terms to compute distance
 */
float matd_vec_dist_n(const matd_t *a, const matd_t *b, int n);

/**
 * Calculates the dot product of two vectors. Both 'a' and 'b' must be vectors
 * and have the same dimension (although one may be a row vector and one may be
 * a column vector).
 */
float matd_vec_dot_product(const matd_t *a, const matd_t *b);

/**
 * Calculates the normalization of the supplied vector 'a' (i.e. a unit vector
 * of the same dimension and orientation as 'a' with a magnitude of 1) and returns
 * it as a new vector. 'a' must be a vector of any dimension and must have a
 * non-zero magnitude. It is the caller's responsibility to call matd_destroy()
 * on the returned matrix.
 */
matd_t *matd_vec_normalize(const matd_t *a);

/**
 * Calculates the cross product of supplied matrices 'a' and 'b' (i.e. a x b)
 * and returns it as a new matrix. Both 'a' and 'b' must be vectors of dimension
 * 3, but can be either row or column vectors. It is the caller's responsibility
 * to call matd_destroy() on the returned matrix.
 */
matd_t *matd_crossproduct(const matd_t *a, const matd_t *b);

float matd_err_inf(const matd_t *a, const matd_t *b);

/**
 * Creates a new matrix by applying a series of matrix operations, as expressed
 * in 'expr', to the supplied list of matrices. Each matrix to be operated upon
 * must be represented in the expression by a separate matrix placeholder, 'M',
 * and there must be one matrix supplied as an argument for each matrix
 * placeholder in the expression. All rules and caveats of the corresponding
 * matrix operations apply to the operated-on matrices. It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 *
 * Available operators (in order of increasing precedence):
 *   M+M   add two matrices together
 *   M-M   subtract one matrix from another
 *   M*M   multiply to matrices together (matrix product)
 *   MM    multiply to matrices together (matrix product)
 *   -M    negate a matrix
 *   M^-1  take the inverse of a matrix
 *   M'    take the transpose of a matrix
 *
 * Expressions can be combined together and grouped by enclosing them in
 * parenthesis, i.e.:
 *   -M(M+M+M)-(M*M)^-1
 *
 * Scalar values can be generated on-the-fly, i.e.:
 *   M*2.2  scales M by 2.2
 *   -2+M   adds -2 to all elements of M
 *
 * All whitespace in the expression is ignored.
 */
matd_t *matd_op(const char *expr, ...);

/**
 * Frees the memory associated with matrix 'm', being the result of an earlier
 * call to a matd_*() function, after which 'm' will no longer be usable.
 */
void matd_destroy(matd_t *m);

typedef struct
{
    matd_t *U;
    matd_t *S;
    matd_t *V;
} matd_svd_t;

/** Compute a complete SVD of a matrix. The SVD exists for all
 * matrices. For a matrix MxN, we will have:
 *
 * A = U*S*V'
 *
 * where A is MxN, U is MxM (and is an orthonormal basis), S is MxN
 * (and is diagonal up to machine precision), and V is NxN (and is an
 * orthonormal basis).
 *
 * The caller is responsible for destroying U, S, and V.
 **/
matd_svd_t matd_svd(matd_t *A);

#define MATD_SVD_NO_WARNINGS 1
    matd_svd_t matd_svd_flags(matd_t *A, int flags);

////////////////////////////////
// PLU Decomposition

// All square matrices (even singular ones) have a partially-pivoted
// LU decomposition such that A = PLU, where P is a permutation
// matrix, L is a lower triangular matrix, and U is an upper
// triangular matrix.
//
typedef struct
{
    // was the input matrix singular? When a zero pivot is found, this
    // flag is set to indicate that this has happened.
    int singular;

    unsigned int *piv; // permutation indices
    int pivsign; // either +1 or -1

    // The matd_plu_t object returned "owns" the enclosed LU matrix. It
    // is not expected that the returned object is itself useful to
    // users: it contains the L and U information all smushed
    // together.
    matd_t *lu; // combined L and U matrices, permuted so they can be triangular.
} matd_plu_t;

matd_plu_t *matd_plu(const matd_t *a);
void matd_plu_destroy(matd_plu_t *mlu);
float matd_plu_det(const matd_plu_t *lu);
matd_t *matd_plu_p(const matd_plu_t *lu);
matd_t *matd_plu_l(const matd_plu_t *lu);
matd_t *matd_plu_u(const matd_plu_t *lu);
matd_t *matd_plu_solve(const matd_plu_t *mlu, const matd_t *b);

// uses LU decomposition internally.
matd_t *matd_solve(matd_t *A, matd_t *b);

////////////////////////////////
// Cholesky Factorization

/**
 * Creates a float matrix with the Cholesky lower triangular matrix
 * of A. A must be symmetric, positive definite. It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 */
//matd_t *matd_cholesky(const matd_t *A);

typedef struct
{
    int is_spd;
    matd_t *u;
} matd_chol_t;

matd_chol_t *matd_chol(matd_t *A);
matd_t *matd_chol_solve(const matd_chol_t *chol, const matd_t *b);
void matd_chol_destroy(matd_chol_t *chol);
// only sensible on PSD matrices
matd_t *matd_chol_inverse(matd_t *a);

void matd_ltransposetriangle_solve(matd_t *u, const float *b, float *x);
void matd_ltriangle_solve(matd_t *u, const float *b, float *x);
void matd_utriangle_solve(matd_t *u, const float *b, float *x);


float matd_max(matd_t *m);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "matd.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

// a matd_t with rows=0 cols=0 is a SCALAR.

// to ease creating mati, matf, etc. in the future.
#define TYPE float

matd_t *matd_create(int rows, int cols)
{
    assert(rows >= 0);
    assert(cols >= 0);

    if (rows == 0 || cols == 0)
        return matd_create_scalar(0);

    matd_t *m = calloc(1, sizeof(matd_t) + (rows*cols*sizeof(float)));
    m->nrows = rows;
    m->ncols = cols;

    return m;
}

matd_t *matd_create_scalar(TYPE v)
{
    matd_t *m = calloc(1, sizeof(matd_t) + sizeof(float));
    m->nrows = 0;
    m->ncols = 0;
    m->data[0] = v;

    return m;
}

matd_t *matd_create_data(int rows, int cols, const TYPE *data)
{
    if (rows == 0 || cols == 0)
        return matd_create_scalar(data[0]);

    matd_t *m = matd_create(rows, cols);
    for (int i = 0; i < rows * cols; i++)
        m->data[i] = data[i];

    return m;
}

matd_t *matd_create_dataf(int rows, int cols, const float *data)
{
    if (rows == 0 || cols == 0)
        return matd_create_scalar(data[0]);

    matd_t *m = matd_create(rows, cols);
    for (int i = 0; i < rows * cols; i++)
        m->data[i] = (float)data[i];

    return m;
}

matd_t *matd_identity(int dim)
{
    if (dim == 0)
        return matd_create_scalar(1);

    matd_t *m = matd_create(dim, dim);
    for (int i = 0; i < dim; i++)
        MATD_EL(m, i, i) = 1;

    return m;
}

// row and col are zero-based
TYPE matd_get(const matd_t *m, int row, int col)
{
    assert(m != NULL);
    assert(!matd_is_scalar(m));
    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    return MATD_EL(m, row, col);
}

// row and col are zero-based
void matd_put(matd_t *m, int row, int col, TYPE value)
{
    assert(m != NULL);

    if (matd_is_scalar(m)) {
        matd_put_scalar(m, value);
        return;
    }

    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    MATD_EL(m, row, col) = value;
}

TYPE matd_get_scalar(const matd_t *m)
{
    assert(m != NULL);
    assert(matd_is_scalar(m));

    return (m->data[0]);
}

void matd_put_scalar(matd_t *m, TYPE value)
{
    assert(m != NULL);
    assert(matd_is_scalar(m));

    m->data[0] = value;
}

matd_t *matd_copy(const matd_t *m)
{
    assert(m != NULL);

    matd_t *x = matd_create(m->nrows, m->ncols);
    if (matd_is_scalar(m))
        x->data[0] = m->data[0];
    else
        memcpy(x->data, m->data, sizeof(TYPE)*m->ncols*m->nrows);

    return x;
}

matd_t *matd_select(const matd_t * a, int r0, int r1, int c0, int c1)
{
    assert(a != NULL);

    assert(r0 >= 0 && r0 < a->nrows);
    assert(c0 >= 0 && c0 < a->ncols);

    int nrows = r1 - r0 + 1;
    int ncols = c1 - c0 + 1;

    matd_t * r = matd_create(nrows, ncols);

    for (int row = r0; row <= r1; row++)
        for (int col = c0; col <= c1; col++)
            MATD_EL(r,row-r0,col-c0) = MATD_EL(a,row,col);

    return r;
}

void matd_print(const matd_t *m, const char *fmt)
{
    assert(m != NULL);
    assert(fmt != NULL);

    if (matd_is_scalar(m)) {
        printf(fmt, (double) MATD_EL(m, 0, 0));
        printf("\n");
    } else {
        for (int i = 0; i < m->nrows; i++) {
            for (int j = 0; j < m->ncols; j++) {
                printf(fmt, (double) MATD_EL(m, i, j));
            }
            printf("\n");
        }
    }
}

void matd_print_transpose(const matd_t *m, const char *fmt)
{
    assert(m != NULL);
    assert(fmt != NULL);

    if (matd_is_scalar(m)) {
        printf(fmt, (double) MATD_EL(m, 0, 0));
        printf("\n");
    } else {
        for (int j = 0; j < m->ncols; j++) {
            for (int i = 0; i < m->nrows; i++) {
                printf(fmt, (double) MATD_EL(m, i, j));
            }
            printf("\n");
        }
    }
}

void matd_destroy(matd_t *m)
{
    if (!m)
        return;

    assert(m != NULL);
    free(m);
}

matd_t *matd_multiply(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);

    if (matd_is_scalar(a))
        return matd_scale(b, a->data[0]);
    if (matd_is_scalar(b))
        return matd_scale(a, b->data[0]);

    assert(a->ncols == b->nrows);
    matd_t *m = matd_create(a->nrows, b->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            TYPE acc = 0;
            for (int k = 0; k < a->ncols; k++) {
                acc += MATD_EL(a, i, k) * MATD_EL(b, k, j);
            }
            MATD_EL(m, i, j) = acc;
        }
    }

    return m;
}

matd_t *matd_scale(const matd_t *a, float s)
{
    assert(a != NULL);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] * s);

    matd_t *m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = s * MATD_EL(a, i, j);
        }
    }

    return m;
}

void matd_scale_inplace(matd_t *a, float s)
{
    assert(a != NULL);

    if (matd_is_scalar(a)) {
        a->data[0] *= s;
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) *= s;
        }
    }
}

matd_t *matd_add(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] + b->data[0]);

    matd_t *m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = MATD_EL(a, i, j) + MATD_EL(b, i, j);
        }
    }

    return m;
}

void matd_add_inplace(matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a)) {
        a->data[0] += b->data[0];
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) += MATD_EL(b, i, j);
        }
    }
}


matd_t *matd_subtract(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0] - b->data[0]);

    matd_t *m = matd_create(a->nrows, a->ncols);

    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            MATD_EL(m, i, j) = MATD_EL(a, i, j) - MATD_EL(b, i, j);
        }
    }

    return m;
}

void matd_subtract_inplace(matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    if (matd_is_scalar(a)) {
        a->data[0] -= b->data[0];
        return;
    }

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(a, i, j) -= MATD_EL(b, i, j);
        }
    }
}


matd_t *matd_transpose(const matd_t *a)
{
    assert(a != NULL);

    if (matd_is_scalar(a))
        return matd_create_scalar(a->data[0]);

    matd_t *m = matd_create(a->ncols, a->nrows);

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            MATD_EL(m, j, i) = MATD_EL(a, i, j);
        }
    }
    return m;
}

static
float matd_det_general(const matd_t *a)
{
    // Use LU decompositon to calculate the determinant
    matd_plu_t *mlu = matd_plu(a);
    matd_t *L = matd_plu_l(mlu);
    matd_t *U = matd_plu_u(mlu);

    // The determinants of the L and U matrices are the products of
    // their respective diagonal elements
    float detL = 1; float detU = 1;
    for (int i = 0; i < a->nrows; i++) {
        detL *= matd_get(L, i, i);
        detU *= matd_get(U, i, i);
    }

    // The determinant of a can be calculated as
    //     epsilon*det(L)*det(U),
    // where epsilon is just the sign of the corresponding permutation
    // (which is +1 for an even number of permutations and is −1
    // for an uneven number of permutations).
    float det = mlu->pivsign * detL * detU;

    // Cleanup
    matd_plu_destroy(mlu);
    matd_destroy(L);
    matd_destroy(U);

    return det;
}

float matd_det(const matd_t *a)
{
    assert(a != NULL);
    assert(a->nrows == a->ncols);

    switch(a->nrows) {
        case 0:
            // scalar: invalid
            assert(a->nrows > 0);
            break;

        case 1:
            // 1x1 matrix
            return a->data[0];

        case 2:
            // 2x2 matrix
            return a->data[0] * a->data[3] - a->data[1] * a->data[2];

        case 3:
            // 3x3 matrix
            return  a->data[0]*a->data[4]*a->data[8]
                - a->data[0]*a->data[5]*a->data[7]
                + a->data[1]*a->data[5]*a->data[6]
                - a->data[1]*a->data[3]*a->data[8]
                + a->data[2]*a->data[3]*a->data[7]
                - a->data[2]*a->data[4]*a->data[6];

        case 4: {
            // 4x4 matrix
            float m00 = MATD_EL(a,0,0), m01 = MATD_EL(a,0,1), m02 = MATD_EL(a,0,2), m03 = MATD_EL(a,0,3);
            float m10 = MATD_EL(a,1,0), m11 = MATD_EL(a,1,1), m12 = MATD_EL(a,1,2), m13 = MATD_EL(a,1,3);
            float m20 = MATD_EL(a,2,0), m21 = MATD_EL(a,2,1), m22 = MATD_EL(a,2,2), m23 = MATD_EL(a,2,3);
            float m30 = MATD_EL(a,3,0), m31 = MATD_EL(a,3,1), m32 = MATD_EL(a,3,2), m33 = MATD_EL(a,3,3);

            return m00 * m11 * m22 * m33 - m00 * m11 * m23 * m32 -
                m00 * m21 * m12 * m33 + m00 * m21 * m13 * m32 + m00 * m31 * m12 * m23 -
                m00 * m31 * m13 * m22 - m10 * m01 * m22 * m33 +
                m10 * m01 * m23 * m32 + m10 * m21 * m02 * m33 -
                m10 * m21 * m03 * m32 - m10 * m31 * m02 * m23 +
                m10 * m31 * m03 * m22 + m20 * m01 * m12 * m33 -
                m20 * m01 * m13 * m32 - m20 * m11 * m02 * m33 +
                m20 * m11 * m03 * m32 + m20 * m31 * m02 * m13 -
                m20 * m31 * m03 * m12 - m30 * m01 * m12 * m23 +
                m30 * m01 * m13 * m22 + m30 * m11 * m02 * m23 -
                m30 * m11 * m03 * m22 - m30 * m21 * m02 * m13 +
                m30 * m21 * m03 * m12;
        }

        default:
            return matd_det_general(a);
    }

    assert(0);
    return 0;
}

// returns NULL if the matrix is (exactly) singular. Caller is
// otherwise responsible for knowing how to cope with badly
// conditioned matrices.
matd_t *matd_inverse(const matd_t *x)
{
    matd_t *m = NULL;

    assert(x != NULL);
    assert(x->nrows == x->ncols);

    if (matd_is_scalar(x)) {
        if (x->data[0] == 0)
            return NULL;

        return matd_create_scalar(1.0 / x->data[0]);
    }

    switch(x->nrows) {
        case 1: {
            float det = x->data[0];
            if (det == 0)
                return NULL;

            float invdet = 1.0 / det;

            m = matd_create(x->nrows, x->nrows);
            MATD_EL(m, 0, 0) = 1.0 * invdet;
            return m;
        }

        case 2: {
            float det = x->data[0] * x->data[3] - x->data[1] * x->data[2];
            if (det == 0)
                return NULL;

            float invdet = 1.0 / det;

            m = matd_create(x->nrows, x->nrows);
            MATD_EL(m, 0, 0) = MATD_EL(x, 1, 1) * invdet;
            MATD_EL(m, 0, 1) = - MATD_EL(x, 0, 1) * invdet;
            MATD_EL(m, 1, 0) = - MATD_EL(x, 1, 0) * invdet;
            MATD_EL(m, 1, 1) = MATD_EL(x, 0, 0) * invdet;
            return m;
        }

        default: {
            matd_plu_t *plu = matd_plu(x);

            matd_t *inv = NULL;
            if (!plu->singular) {
                matd_t *ident = matd_identity(x->nrows);
                inv = matd_plu_solve(plu, ident);
                matd_destroy(ident);
            }

            matd_plu_destroy(plu);

            return inv;
        }
    }

    return NULL; // unreachable
}



// TODO Optimization: Some operations we could perform in-place,
// saving some memory allocation work. E.g., ADD, SUBTRACT. Just need
// to make sure that we don't do an in-place modification on a matrix
// that was an input argument!

// handle right-associative operators, greedily consuming them. These
// include transpose and inverse. This is called by the main recursion
// method.
static inline matd_t *matd_op_gobble_right(const char *expr, int *pos, matd_t *acc, matd_t **garb, int *garbpos)
{
    while (expr[*pos] != 0) {

        switch (expr[*pos]) {

            case '\'': {
                assert(acc != NULL); // either a syntax error or a math op failed, producing null
                matd_t *res = matd_transpose(acc);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;

                (*pos)++;
                break;
            }

                // handle inverse ^-1. No other exponents are allowed.
            case '^': {
                assert(acc != NULL);
                assert(expr[*pos+1] == '-');
                assert(expr[*pos+2] == '1');

                matd_t *res = matd_inverse(acc);
                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;

                (*pos)+=3;
                break;
            }

            default:
                return acc;
        }
    }

    return acc;
}

// @garb, garbpos  A list of every matrix allocated during evaluation... used to assist cleanup.
// @oneterm: we should return at the end of this term (i.e., stop at a PLUS, MINUS, LPAREN).
static matd_t *matd_op_recurse(const char *expr, int *pos, matd_t *acc, matd_t **args, int *argpos,
                               matd_t **garb, int *garbpos, int oneterm)
{
    while (expr[*pos] != 0) {

        switch (expr[*pos]) {

            case '(': {
                if (oneterm && acc != NULL)
                    return acc;
                (*pos)++;
                matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 0);
                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                if (acc == NULL) {
                    acc = rhs;
                } else {
                    matd_t *res = matd_multiply(acc, rhs);
                    garb[*garbpos] = res;
                    (*garbpos)++;
                    acc = res;
                }

                break;
            }

            case ')': {
                if (oneterm)
                    return acc;

                (*pos)++;
                return acc;
            }

            case '*': {
                (*pos)++;

                matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                if (acc == NULL) {
                    acc = rhs;
                } else {
                    matd_t *res = matd_multiply(acc, rhs);
                    garb[*garbpos] = res;
                    (*garbpos)++;
                    acc = res;
                }

                break;
            }

            case 'F': {
                matd_t *rhs = args[*argpos];
                garb[*garbpos] = rhs;
                (*garbpos)++;

                (*pos)++;
                (*argpos)++;

                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                if (acc == NULL) {
                    acc = rhs;
                } else {
                    matd_t *res = matd_multiply(acc, rhs);
                    garb[*garbpos] = res;
                    (*garbpos)++;
                    acc = res;
                }

                break;
            }

            case 'M': {
                matd_t *rhs = args[*argpos];

                (*pos)++;
                (*argpos)++;

                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                if (acc == NULL) {
                    acc = rhs;
                } else {
                    matd_t *res = matd_multiply(acc, rhs);
                    garb[*garbpos] = res;
                    (*garbpos)++;
                    acc = res;
                }

                break;
            }

/*
  case 'D': {
  int rows = expr[*pos+1]-'0';
  int cols = expr[*pos+2]-'0';

  matd_t *rhs = matd_create(rows, cols);

  break;
  }
*/
                // a constant (SCALAR) defined inline. Treat just like M, creating a matd_t on the fly.
//            case '0':
//            case '1':
//            case '2':
//            case '3':
//            case '4':
//            case '5':
//            case '6':
//            case '7':
//            case '8':
//            case '9':
//            case '.': {
//                const char *start = &expr[*pos];
//                char *end;
//                float s = strtod(start, &end);
//                (*pos) += (end - start);
//                matd_t *rhs = matd_create_scalar(s);
//                garb[*garbpos] = rhs;
//                (*garbpos)++;

//                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

//                if (acc == NULL) {
//                    acc = rhs;
//                } else {
//                    matd_t *res = matd_multiply(acc, rhs);
//                    garb[*garbpos] = res;
//                    (*garbpos)++;
//                    acc = res;
//                }

//                break;
//            }

            case '+': {
                if (oneterm && acc != NULL)
                    return acc;

                // don't support unary plus
                assert(acc != NULL);
                (*pos)++;
                matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
                rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                matd_t *res = matd_add(acc, rhs);

                garb[*garbpos] = res;
                (*garbpos)++;
                acc = res;
                break;
            }

            case '-': {
                if (oneterm && acc != NULL)
                    return acc;

                if (acc == NULL) {
                    // unary minus
                    (*pos)++;
                    matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
                    rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                    matd_t *res = matd_scale(rhs, -1);
                    garb[*garbpos] = res;
                    (*garbpos)++;
                    acc = res;
                } else {
                    // subtract
                    (*pos)++;
                    matd_t *rhs = matd_op_recurse(expr, pos, NULL, args, argpos, garb, garbpos, 1);
                    rhs = matd_op_gobble_right(expr, pos, rhs, garb, garbpos);

                    matd_t *res = matd_subtract(acc, rhs);
                    garb[*garbpos] = res;
                    (*garbpos)++;
                    acc = res;
                }
                break;
            }

            case ' ': {
                // nothing to do. spaces are meaningless.
                (*pos)++;
                break;
            }

            default: {
                fprintf(stderr, "matd_op(): Unknown character: '%c'\n", expr[*pos]);
                assert(expr[*pos] != expr[*pos]);
            }
        }
    }
    return acc;
}

// always returns a new matrix.
matd_t *matd_op(const char *expr, ...)
{
    int nargs = 0;
    int exprlen = 0;

    assert(expr != NULL);

    for (const char *p = expr; *p != 0; p++) {
        if (*p == 'M' || *p == 'F')
            nargs++;
        exprlen++;
    }

    assert(nargs > 0);

    if (!exprlen) // expr = ""
        return NULL;

    va_list ap;
    va_start(ap, expr);

    matd_t *args[nargs];
    for (int i = 0; i < nargs; i++) {
        args[i] = va_arg(ap, matd_t*);
        // XXX: sanity check argument; emit warning/error if args[i]
        // doesn't look like a matd_t*.
    }

    va_end(ap);

    int pos = 0;
    int argpos = 0;
    int garbpos = 0;

    matd_t *garb[2*exprlen]; // can't create more than 2 new result per character
                             // one result, and possibly one argument to free

    matd_t *res = matd_op_recurse(expr, &pos, NULL, args, &argpos, garb, &garbpos, 0);

    // 'res' may need to be freed as part of garbage collection (i.e. expr = "F")
    matd_t *res_copy = (res ? matd_copy(res) : NULL);

    for (int i = 0; i < garbpos; i++) {
        matd_destroy(garb[i]);
    }

    return res_copy;
}

float matd_vec_mag(const matd_t *a)
{
    assert(a != NULL);
    assert(matd_is_vector(a));

    float mag = 0.0;
    int len = a->nrows*a->ncols;
    for (int i = 0; i < len; i++)
        mag += sq(a->data[i]);
    return sqrt(mag);
}

float matd_vec_dist(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(matd_is_vector(a) && matd_is_vector(b));
    assert(a->nrows*a->ncols == b->nrows*b->ncols);

    int lena = a->nrows*a->ncols;
    return matd_vec_dist_n(a, b, lena);
}

float matd_vec_dist_n(const matd_t *a, const matd_t *b, int n)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(matd_is_vector(a) && matd_is_vector(b));

    int lena = a->nrows*a->ncols;
    int lenb = b->nrows*b->ncols;

    assert(n <= lena && n <= lenb);

    float mag = 0.0;
    for (int i = 0; i < n; i++)
        mag += sq(a->data[i] - b->data[i]);
    return sqrt(mag);
}

// find the index of the off-diagonal element with the largest mag
static inline int max_idx(const matd_t *A, int row, int maxcol)
{
    int maxi = 0;
    float maxv = -1;

    for (int i = 0; i < maxcol; i++) {
        if (i == row)
            continue;
        float v = fabs(MATD_EL(A, row, i));
        if (v > maxv) {
            maxi = i;
            maxv = v;
        }
    }

    return maxi;
}

float matd_vec_dot_product(const matd_t *a, const matd_t *b)
{
    assert(a != NULL);
    assert(b != NULL);
    assert(matd_is_vector(a) && matd_is_vector(b));
    int adim = a->ncols*a->nrows;
    int bdim = b->ncols*b->nrows;
    assert(adim == bdim);

    float acc = 0;
    for (int i = 0; i < adim; i++) {
        acc += a->data[i] * b->data[i];
    }
    return acc;
}


matd_t *matd_vec_normalize(const matd_t *a)
{
    assert(a != NULL);
    assert(matd_is_vector(a));

    float mag = matd_vec_mag(a);
    assert(mag > 0);

    matd_t *b = matd_create(a->nrows, a->ncols);

    int len = a->nrows*a->ncols;
    for(int i = 0; i < len; i++)
        b->data[i] = a->data[i] / mag;

    return b;
}

matd_t *matd_crossproduct(const matd_t *a, const matd_t *b)
{ // only defined for vecs (col or row) of length 3
    assert(a != NULL);
    assert(b != NULL);
    assert(matd_is_vector_len(a, 3) && matd_is_vector_len(b, 3));

    matd_t * r = matd_create(a->nrows, a->ncols);

    r->data[0] = a->data[1] * b->data[2] - a->data[2] * b->data[1];
    r->data[1] = a->data[2] * b->data[0] - a->data[0] * b->data[2];
    r->data[2] = a->data[0] * b->data[1] - a->data[1] * b->data[0];

    return r;
}

TYPE matd_err_inf(const matd_t *a, const matd_t *b)
{
    assert(a->nrows == b->nrows);
    assert(a->ncols == b->ncols);

    TYPE maxf = 0;

    for (int i = 0; i < a->nrows; i++) {
        for (int j = 0; j < a->ncols; j++) {
            TYPE av = MATD_EL(a, i, j);
            TYPE bv = MATD_EL(b, i, j);

            TYPE err = fabs(av - bv);
            maxf = fmax(maxf, err);
        }
    }

    return maxf;
}

// Computes an SVD for square or tall matrices. This code doesn't work
// for wide matrices, because the bidiagonalization results in one
// non-zero element too far to the right for us to rotate away.
//
// Caller is responsible for destroying U, S, and V.
static matd_svd_t matd_svd_tall(matd_t *A, int flags)
{
    matd_t *B = matd_copy(A);

    // Apply householder reflections on each side to reduce A to
    // bidiagonal form. Specifically:
    //
    // A = LS*B*RS'
    //
    // Where B is bidiagonal, and LS/RS are unitary.
    //
    // Why are we doing this? Some sort of transformation is necessary
    // to reduce the matrix's nz elements to a square region. QR could
    // work too. We need nzs confined to a square region so that the
    // subsequent iterative process, which is based on rotations, can
    // work. (To zero out a term at (i,j), our rotations will also
    // affect (j,i).
    //
    // We prefer bidiagonalization over QR because it gets us "closer"
    // to the SVD, which should mean fewer iterations.

    // LS: cumulative left-handed transformations
    matd_t *LS = matd_identity(A->nrows);

    // RS: cumulative right-handed transformations.
    matd_t *RS = matd_identity(A->ncols);

    for (int hhidx = 0; hhidx < A->nrows; hhidx++)  {

        if (hhidx < A->ncols) {
            // We construct the normal of the reflection plane: let u
            // be the vector to reflect, x =[ M 0 0 0 ] the target
            // location for u (u') after reflection (with M = ||u||).
            //
            // The normal vector is then n = (u - x), but since we
            // could equally have the target location be x = [-M 0 0 0
            // ], we could use n = (u + x).
            //
            // We then normalize n. To ensure a reasonable magnitude,
            // we select the sign of M so as to maximize the magnitude
            // of the first element of (x +/- M). (Otherwise, we could
            // end up with a divide-by-zero if u[0] and M cancel.)
            //
            // The householder reflection matrix is then H=(I - nn'), and
            // u' = Hu.
            //
            //
            int vlen = A->nrows - hhidx;

            float v[vlen];

            float mag2 = 0;
            for (int i = 0; i < vlen; i++) {
                v[i] = MATD_EL(B, hhidx+i, hhidx);
                mag2 += v[i]*v[i];
            }

            float oldv0 = v[0];
            if (oldv0 < 0)
                v[0] -= sqrt(mag2);
            else
                v[0] += sqrt(mag2);

            mag2 += -oldv0*oldv0 + v[0]*v[0];

            // normalize v
            float mag = sqrt(mag2);

            // this case arises with matrices of all zeros, for example.
            if (mag == 0)
                continue;

            for (int i = 0; i < vlen; i++)
                v[i] /= mag;

            // Q = I - 2vv'
            //matd_t *Q = matd_identity(A->nrows);
            //for (int i = 0; i < vlen; i++)
            //  for (int j = 0; j < vlen; j++)
            //    MATD_EL(Q, i+hhidx, j+hhidx) -= 2*v[i]*v[j];


            // LS = matd_op("F*M", LS, Q);
            // Implementation: take each row of LS, compute dot product with n,
            // subtract n (scaled by dot product) from it.
            for (int i = 0; i < LS->nrows; i++) {
                float dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(LS, i, hhidx+j) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(LS, i, hhidx+j) -= 2*dot*v[j];
            }

            //  B = matd_op("M*F", Q, B); // should be Q', but Q is symmetric.
            for (int i = 0; i < B->ncols; i++) {
                float dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(B, hhidx+j, i) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(B, hhidx+j, i) -= 2*dot*v[j];
            }
        }

        if (hhidx+2 < A->ncols) {
            int vlen = A->ncols - hhidx - 1;

            float v[vlen];

            float mag2 = 0;
            for (int i = 0; i < vlen; i++) {
                v[i] = MATD_EL(B, hhidx, hhidx+i+1);
                mag2 += v[i]*v[i];
            }

            float oldv0 = v[0];
            if (oldv0 < 0)
                v[0] -= sqrt(mag2);
            else
                v[0] += sqrt(mag2);

            mag2 += -oldv0*oldv0 + v[0]*v[0];

            // compute magnitude of ([1 0 0..]+v)
            float mag = sqrt(mag2);

            // this case can occur when the vectors are already perpendicular
            if (mag == 0)
                continue;

            for (int i = 0; i < vlen; i++)
                v[i] /= mag;

            // TODO: optimize these multiplications
            // matd_t *Q = matd_identity(A->ncols);
            //  for (int i = 0; i < vlen; i++)
            //    for (int j = 0; j < vlen; j++)
            //       MATD_EL(Q, i+1+hhidx, j+1+hhidx) -= 2*v[i]*v[j];

            //  RS = matd_op("F*M", RS, Q);
            for (int i = 0; i < RS->nrows; i++) {
                float dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(RS, i, hhidx+1+j) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(RS, i, hhidx+1+j) -= 2*dot*v[j];
            }

            //   B = matd_op("F*M", B, Q); // should be Q', but Q is symmetric.
            for (int i = 0; i < B->nrows; i++) {
                float dot = 0;
                for (int j = 0; j < vlen; j++)
                    dot += MATD_EL(B, i, hhidx+1+j) * v[j];
                for (int j = 0; j < vlen; j++)
                    MATD_EL(B, i, hhidx+1+j) -= 2*dot*v[j];
            }
        }
    }

    // maxiters used to be smaller to prevent us from looping forever,
    // but this doesn't seem to happen any more with our more stable
    // svd22 implementation.
    int maxiters = 1UL << 5; // 1UL << 30;
    assert(maxiters > 0); // reassure clang
    int iter;

    float maxv; // maximum non-zero value being reduced this iteration

    float tol = 1E-5; // 1E-10;

    // which method will we use to find the largest off-diagonal
    // element of B?
    const int find_max_method = 1; //(B->ncols < 6) ? 2 : 1;

    // for each of the first B->ncols rows, which index has the
    // maximum absolute value? (used by method 1)
    int maxrowidx[B->ncols];
    int lastmaxi, lastmaxj;

    if (find_max_method == 1) {
        for (int i = 2; i < B->ncols; i++)
            maxrowidx[i] = max_idx(B, i, B->ncols);

        // note that we started the array at 2. That's because by setting
        // these values below, we'll recompute first two entries on the
        // first iteration!
        lastmaxi = 0, lastmaxj = 1;
    }

    for (iter = 0; iter < maxiters; iter++) {

        // No diagonalization required for 0x0 and 1x1 matrices.
        if (B->ncols < 2)
            break;

        // find the largest off-diagonal element of B, and put its
        // coordinates in maxi, maxj.
        int maxi, maxj;

        if (find_max_method == 1) {
            // method 1 is the "smarter" method which does at least
            // 4*ncols work. More work might be needed (up to
            // ncols*ncols), depending on data. Thus, this might be a
            // bit slower than the default method for very small
            // matrices.
            maxi = -1;
            maxv = -1;

            // every iteration, we must deal with the fact that rows
            // and columns lastmaxi and lastmaxj have been
            // modified. Update maxrowidx accordingly.

            // now, EVERY row also had columns lastmaxi and lastmaxj modified.
            for (int rowi = 0; rowi < B->ncols; rowi++) {

                // the magnitude of the largest off-diagonal element
                // in this row.
                float thismaxv;

                // row 'lastmaxi' and 'lastmaxj' have been completely
                // changed. compute from scratch.
                if (rowi == lastmaxi || rowi == lastmaxj) {
                    maxrowidx[rowi] = max_idx(B, rowi, B->ncols);
                    thismaxv = fabs(MATD_EL(B, rowi, maxrowidx[rowi]));
                    goto endrowi;
                }

                // our maximum entry was just modified. We don't know
                // if it went up or down, and so we don't know if it
                // is still the maximum. We have to update from
                // scratch.
                if (maxrowidx[rowi] == lastmaxi || maxrowidx[rowi] == lastmaxj) {
                    maxrowidx[rowi] = max_idx(B, rowi, B->ncols);
                    thismaxv = fabs(MATD_EL(B, rowi, maxrowidx[rowi]));
                    goto endrowi;
                }

                // This row is unchanged, except for columns
                // 'lastmaxi' and 'lastmaxj', and those columns were
                // not previously the largest entry...  just check to
                // see if they are now the maximum entry in their
                // row. (Remembering to consider off-diagonal entries
                // only!)
                thismaxv = fabs(MATD_EL(B, rowi, maxrowidx[rowi]));

                // check column lastmaxi. Is it now the maximum?
                if (lastmaxi != rowi) {
                    float v = fabs(MATD_EL(B, rowi, lastmaxi));
                    if (v > thismaxv) {
                        thismaxv = v;
                        maxrowidx[rowi] = lastmaxi;
                    }
                }

                // check column lastmaxj
                if (lastmaxj != rowi) {
                    float v = fabs(MATD_EL(B, rowi, lastmaxj));
                    if (v > thismaxv) {
                        thismaxv = v;
                        maxrowidx[rowi] = lastmaxj;
                    }
                }

                // does this row have the largest value we've seen so far?
              endrowi:
                if (thismaxv > maxv) {
                    maxv = thismaxv;
                    maxi = rowi;
                }
            }

            assert(maxi >= 0);
            maxj = maxrowidx[maxi];

            // save these for the next iteration.
            lastmaxi = maxi;
            lastmaxj = maxj;

            if (maxv < tol)
                break;

        } else if (find_max_method == 2) {
            // brute-force (reference) version.
            maxv = -1;

            // only search top "square" portion
            for (int i = 0; i < B->ncols; i++) {
                for (int j = 0; j < B->ncols; j++) {
                    if (i == j)
                        continue;

                    float v = fabs(MATD_EL(B, i, j));

                    if (v > maxv) {
                        maxi = i;
                        maxj = j;
                        maxv = v;
                    }
                }
            }

            // termination condition.
            if (maxv < tol)
                break;
        } else {
            assert(0);
        }

//        printf(">>> %5d %3d, %3d %15g\n", maxi, maxj, iter, maxv);

        // Now, solve the 2x2 SVD problem for the matrix
        // [ A0 A1 ]
        // [ A2 A3 ]
        float A0 = MATD_EL(B, maxi, maxi);
        float A1 = MATD_EL(B, maxi, maxj);
        float A2 = MATD_EL(B, maxj, maxi);
        float A3 = MATD_EL(B, maxj, maxj);

        if (1) {
            float AQ[4];
            AQ[0] = A0;
            AQ[1] = A1;
            AQ[2] = A2;
            AQ[3] = A3;

            float U[4], S[2], V[4];
            svd22(AQ, U, S, V);

/*  Reference (slow) implementation...

            // LS = LS * ROT(theta) = LS * QL
            matd_t *QL = matd_identity(A->nrows);
            MATD_EL(QL, maxi, maxi) = U[0];
            MATD_EL(QL, maxi, maxj) = U[1];
            MATD_EL(QL, maxj, maxi) = U[2];
            MATD_EL(QL, maxj, maxj) = U[3];

            matd_t *QR = matd_identity(A->ncols);
            MATD_EL(QR, maxi, maxi) = V[0];
            MATD_EL(QR, maxi, maxj) = V[1];
            MATD_EL(QR, maxj, maxi) = V[2];
            MATD_EL(QR, maxj, maxj) = V[3];

            LS = matd_op("F*M", LS, QL);
            RS = matd_op("F*M", RS, QR); // remember we'll transpose RS.
            B = matd_op("M'*F*M", QL, B, QR);

            matd_destroy(QL);
            matd_destroy(QR);
*/

            //  LS = matd_op("F*M", LS, QL);
            for (int i = 0; i < LS->nrows; i++) {
                float vi = MATD_EL(LS, i, maxi);
                float vj = MATD_EL(LS, i, maxj);

                MATD_EL(LS, i, maxi) = U[0]*vi + U[2]*vj;
                MATD_EL(LS, i, maxj) = U[1]*vi + U[3]*vj;
            }

            //  RS = matd_op("F*M", RS, QR); // remember we'll transpose RS.
            for (int i = 0; i < RS->nrows; i++) {
                float vi = MATD_EL(RS, i, maxi);
                float vj = MATD_EL(RS, i, maxj);

                MATD_EL(RS, i, maxi) = V[0]*vi + V[2]*vj;
                MATD_EL(RS, i, maxj) = V[1]*vi + V[3]*vj;
            }

            // B = matd_op("M'*F*M", QL, B, QR);
            // The QL matrix mixes rows of B.
            for (int i = 0; i < B->ncols; i++) {
                float vi = MATD_EL(B, maxi, i);
                float vj = MATD_EL(B, maxj, i);

                MATD_EL(B, maxi, i) = U[0]*vi + U[2]*vj;
                MATD_EL(B, maxj, i) = U[1]*vi + U[3]*vj;
            }

            // The QR matrix mixes columns of B.
            for (int i = 0; i < B->nrows; i++) {
                float vi = MATD_EL(B, i, maxi);
                float vj = MATD_EL(B, i, maxj);

                MATD_EL(B, i, maxi) = V[0]*vi + V[2]*vj;
                MATD_EL(B, i, maxj) = V[1]*vi + V[3]*vj;
            }
        }
    }

    if (!(flags & MATD_SVD_NO_WARNINGS) && iter == maxiters) {
        printf("WARNING: maximum iters (maximum = %d, matrix %d x %d, max=%.15f)\n",
               iter, A->nrows, A->ncols, (double) maxv);

//        matd_print(A, "%15f");
    }

    // them all positive by flipping the corresponding columns of
    // U/LS.
    int idxs[A->ncols];
    float vals[A->ncols];
    for (int i = 0; i < A->ncols; i++) {
        idxs[i] = i;
        vals[i] = MATD_EL(B, i, i);
    }

    // A bubble sort. Seriously.
    int changed;
    do {
        changed = 0;

        for (int i = 0; i + 1 < A->ncols; i++) {
            if (fabs(vals[i+1]) > fabs(vals[i])) {
                int tmpi = idxs[i];
                idxs[i] = idxs[i+1];
                idxs[i+1] = tmpi;

                float tmpv = vals[i];
                vals[i] = vals[i+1];
                vals[i+1] = tmpv;

                changed = 1;
            }
        }
    } while (changed);

    matd_t *LP = matd_identity(A->nrows);
    matd_t *RP = matd_identity(A->ncols);

    for (int i = 0; i < A->ncols; i++) {
        MATD_EL(LP, idxs[i], idxs[i]) = 0; // undo the identity above
        MATD_EL(RP, idxs[i], idxs[i]) = 0;

        MATD_EL(LP, idxs[i], i) = vals[i] < 0 ? -1 : 1;
        MATD_EL(RP, idxs[i], i) = 1; //vals[i] < 0 ? -1 : 1;
    }

    // we've factored:
    // LP*(something)*RP'

    // solve for (something)
    B = matd_op("M'*F*M", LP, B, RP);

    // update LS and RS, remembering that RS will be transposed.
    LS = matd_op("F*M", LS, LP);
    RS = matd_op("F*M", RS, RP);

    matd_destroy(LP);
    matd_destroy(RP);

    matd_svd_t res;
    memset(&res, 0, sizeof(res));

    // make B exactly diagonal

    for (int i = 0; i < B->nrows; i++) {
        for (int j = 0; j < B->ncols; j++) {
            if (i != j)
                MATD_EL(B, i, j) = 0;
        }
    }

    res.U = LS;
    res.S = B;
    res.V = RS;

    return res;
}

matd_svd_t matd_svd(matd_t *A)
{
    return matd_svd_flags(A, 0);
}

matd_svd_t matd_svd_flags(matd_t *A, int flags)
{
    matd_svd_t res;

    if (A->ncols <= A->nrows) {
        res = matd_svd_tall(A, flags);
    } else {
        matd_t *At = matd_transpose(A);

        // A =U  S  V'
        // A'=V  S' U'

        matd_svd_t tmp = matd_svd_tall(At, flags);

        memset(&res, 0, sizeof(res));
        res.U = tmp.V; //matd_transpose(tmp.V);
        res.S = matd_transpose(tmp.S);
        res.V = tmp.U; //matd_transpose(tmp.U);

        matd_destroy(tmp.S);
        matd_destroy(At);
    }

/*
  matd_t *check = matd_op("M*M*M'-M", res.U, res.S, res.V, A);
  float maxerr = 0;

  for (int i = 0; i < check->nrows; i++)
  for (int j = 0; j < check->ncols; j++)
  maxerr = fmax(maxerr, fabs(MATD_EL(check, i, j)));

  matd_destroy(check);

  if (maxerr > 1e-7) {
  printf("bad maxerr: %15f\n", maxerr);
  }

  if (maxerr > 1e-5) {
  printf("bad maxerr: %15f\n", maxerr);
  matd_print(A, "%15f");
  assert(0);
  }

*/
    return res;
}


matd_plu_t *matd_plu(const matd_t *a)
{
    unsigned int *piv = calloc(a->nrows, sizeof(unsigned int));
    int pivsign = 1;
    matd_t *lu = matd_copy(a);

    // only for square matrices.
    assert(a->nrows == a->ncols);

    matd_plu_t *mlu = calloc(1, sizeof(matd_plu_t));

    for (int i = 0; i < a->nrows; i++)
        piv[i] = i;

    for (int j = 0; j < a->ncols; j++) {
        for (int i = 0; i < a->nrows; i++) {
            int kmax = i < j ? i : j; // min(i,j)

            // compute dot product of row i with column j (up through element kmax)
            float acc = 0;
            for (int k = 0; k < kmax; k++)
                acc += MATD_EL(lu, i, k) * MATD_EL(lu, k, j);

            MATD_EL(lu, i, j) -= acc;
        }

        // find pivot and exchange if necessary.
        int p = j;
        if (1) {
            for (int i = j+1; i < lu->nrows; i++) {
                if (fabs(MATD_EL(lu,i,j)) > fabs(MATD_EL(lu, p, j))) {
                    p = i;
                }
            }
        }

        // swap rows p and j?
        if (p != j) {
            TYPE tmp[lu->ncols];
            memcpy(tmp, &MATD_EL(lu, p, 0), sizeof(TYPE) * lu->ncols);
            memcpy(&MATD_EL(lu, p, 0), &MATD_EL(lu, j, 0), sizeof(TYPE) * lu->ncols);
            memcpy(&MATD_EL(lu, j, 0), tmp, sizeof(TYPE) * lu->ncols);
            int k = piv[p];
            piv[p] = piv[j];
            piv[j] = k;
            pivsign = -pivsign;
        }

        float LUjj = MATD_EL(lu, j, j);

        // If our pivot is very small (which means the matrix is
        // singular or nearly singular), replace with a new pivot of the
        // right sign.
        if (fabs(LUjj) < MATD_EPS) {
/*
            if (LUjj < 0)
                LUjj = -MATD_EPS;
            else
                LUjj = MATD_EPS;

            MATD_EL(lu, j, j) = LUjj;
*/
            mlu->singular = 1;
        }

        if (j < lu->ncols && j < lu->nrows && LUjj != 0) {
            LUjj = 1.0 / LUjj;
            for (int i = j+1; i < lu->nrows; i++)
                MATD_EL(lu, i, j) *= LUjj;
        }
    }

    mlu->lu = lu;
    mlu->piv = piv;
    mlu->pivsign = pivsign;

    return mlu;
}

void matd_plu_destroy(matd_plu_t *mlu)
{
    matd_destroy(mlu->lu);
    free(mlu->piv);
    memset(mlu, 0, sizeof(matd_plu_t));
    free(mlu);
}

float matd_plu_det(const matd_plu_t *mlu)
{
    matd_t *lu = mlu->lu;
    float det = mlu->pivsign;

    if (lu->nrows == lu->ncols) {
        for (int i = 0; i < lu->ncols; i++)
            det *= MATD_EL(lu, i, i);
    }

    return det;
}

matd_t *matd_plu_p(const matd_plu_t *mlu)
{
    matd_t *lu = mlu->lu;
    matd_t *P = matd_create(lu->nrows, lu->nrows);

    for (int i = 0; i < lu->nrows; i++) {
        MATD_EL(P, mlu->piv[i], i) = 1;
    }

    return P;
}

matd_t *matd_plu_l(const matd_plu_t *mlu)
{
    matd_t *lu = mlu->lu;

    matd_t *L = matd_create(lu->nrows, lu->ncols);
    for (int i = 0; i < lu->nrows; i++) {
        MATD_EL(L, i, i) = 1;

        for (int j = 0; j < i; j++) {
            MATD_EL(L, i, j) = MATD_EL(lu, i, j);
        }
    }

    return L;
}

matd_t *matd_plu_u(const matd_plu_t *mlu)
{
    matd_t *lu = mlu->lu;

    matd_t *U = matd_create(lu->ncols, lu->ncols);
    for (int i = 0; i < lu->ncols; i++) {
        for (int j = 0; j < lu->ncols; j++) {
            if (i <= j)
                MATD_EL(U, i, j) = MATD_EL(lu, i, j);
        }
    }

    return U;
}

// PLU = A
// Ax = B
// PLUx = B
// LUx = P'B
matd_t *matd_plu_solve(const matd_plu_t *mlu, const matd_t *b)
{
    matd_t *x = matd_copy(b);

    // permute right hand side
    for (int i = 0; i < mlu->lu->nrows; i++)
        memcpy(&MATD_EL(x, i, 0), &MATD_EL(b, mlu->piv[i], 0), sizeof(TYPE) * b->ncols);

    // solve Ly = b
    for (int k = 0; k < mlu->lu->nrows; k++) {
        for (int i = k+1; i < mlu->lu->nrows; i++) {
            float LUik = -MATD_EL(mlu->lu, i, k);
            for (int t = 0; t < b->ncols; t++)
                MATD_EL(x, i, t) += MATD_EL(x, k, t) * LUik;
        }
    }

    // solve Ux = y
    for (int k = mlu->lu->ncols-1; k >= 0; k--) {
        float LUkk = 1.0 / MATD_EL(mlu->lu, k, k);
        for (int t = 0; t < b->ncols; t++)
            MATD_EL(x, k, t) *= LUkk;

        for (int i = 0; i < k; i++) {
            float LUik = -MATD_EL(mlu->lu, i, k);
            for (int t = 0; t < b->ncols; t++)
                MATD_EL(x, i, t) += MATD_EL(x, k, t) *LUik;
        }
    }

    return x;
}

matd_t *matd_solve(matd_t *A, matd_t *b)
{
    matd_plu_t *mlu = matd_plu(A);
    matd_t *x = matd_plu_solve(mlu, b);

    matd_plu_destroy(mlu);
    return x;
}

#if 0

static int randi()
{
    int v = random()&31;
    v -= 15;
    return v;
}

static float randf()
{
    float v = 1.0 *random() / RAND_MAX;
    return 2*v - 1;
}

int main(int argc, char *argv[])
{
    if (1) {
        int maxdim = 16;
        matd_t *A = matd_create(maxdim, maxdim);

        for (int iter = 0; 1; iter++) {
            srand(iter);

            if (iter % 1000 == 0)
                printf("%d\n", iter);

            int m = 1 + (random()%(maxdim-1));
            int n = 1 + (random()%(maxdim-1));

            for (int i = 0; i < m*n; i++)
                A->data[i] = randi();

            A->nrows = m;
            A->ncols = n;

//            printf("%d %d ", m, n);
            matd_svd_t svd = matd_svd(A);
            matd_destroy(svd.U);
            matd_destroy(svd.S);
            matd_destroy(svd.V);

        }

/*        matd_t *A = matd_create_data(2, 5, (float[]) { 1, 5, 2, 6,
          3, 3, 0, 7,
          1, 1, 0, -2,
          4, 0, 9, 9, 2, 6, 1, 3, 2, 5, 5, 4, -1, 2, 5, 9, 8, 2 });

          matd_svd(A);
*/
        return 0;
    }


    struct svd22 s;

    srand(0);

    matd_t *A = matd_create(2, 2);
    MATD_EL(A,0,0) = 4;
    MATD_EL(A,0,1) = 7;
    MATD_EL(A,1,0) = 2;
    MATD_EL(A,1,1) = 6;

    matd_t *U = matd_create(2, 2);
    matd_t *V = matd_create(2, 2);
    matd_t *S = matd_create(2, 2);

    for (int iter = 0; 1; iter++) {
        if (iter % 100000 == 0)
            printf("%d\n", iter);

        MATD_EL(A,0,0) = randf();
        MATD_EL(A,0,1) = randf();
        MATD_EL(A,1,0) = randf();
        MATD_EL(A,1,1) = randf();

        matd_svd22_impl(A->data, &s);

        memcpy(U->data, s.U, 4*sizeof(float));
        memcpy(V->data, s.V, 4*sizeof(float));
        MATD_EL(S,0,0) = s.S[0];
        MATD_EL(S,1,1) = s.S[1];

        assert(s.S[0] >= s.S[1]);
        assert(s.S[0] >= 0);
        assert(s.S[1] >= 0);
        if (s.S[0] == 0) {
//            printf("*"); fflush(NULL);
//            printf("%15f %15f %15f %15f\n", MATD_EL(A,0,0), MATD_EL(A,0,1), MATD_EL(A,1,0), MATD_EL(A,1,1));
        }
        if (s.S[1] == 0) {
//            printf("#"); fflush(NULL);
        }

        matd_t *USV = matd_op("M*M*M'", U, S, V);

        float maxerr = 0;
        for (int i = 0; i < 4; i++)
            maxerr = fmax(maxerr, fabs(USV->data[i] - A->data[i]));

        if (0) {
            printf("------------------------------------\n");
            printf("A:\n");
            matd_print(A, "%15f");
            printf("\nUSV':\n");
            matd_print(USV, "%15f");
            printf("maxerr: %.15f\n", maxerr);
            printf("\n\n");
        }

        matd_destroy(USV);

        assert(maxerr < 0.00001);
    }
}

#endif

// XXX NGV Cholesky
/*static float *matd_cholesky_raw(float *A, int n)
  {
  float *L = (float*)calloc(n * n, sizeof(float));

  for (int i = 0; i < n; i++) {
  for (int j = 0; j < (i+1); j++) {
  float s = 0;
  for (int k = 0; k < j; k++)
  s += L[i * n + k] * L[j * n + k];
  L[i * n + j] = (i == j) ?
  sqrt(A[i * n + i] - s) :
  (1.0 / L[j * n + j] * (A[i * n + j] - s));
  }
  }

  return L;
  }

  matd_t *matd_cholesky(const matd_t *A)
  {
  assert(A->nrows == A->ncols);
  float *L_data = matd_cholesky_raw(A->data, A->nrows);
  matd_t *L = matd_create_data(A->nrows, A->ncols, L_data);
  free(L_data);
  return L;
  }*/

// NOTE: The below implementation of Cholesky is different from the one
// used in NGV.
matd_chol_t *matd_chol(matd_t *A)
{
    assert(A->nrows == A->ncols);
    int N = A->nrows;

    // make upper right
    matd_t *U = matd_copy(A);

    // don't actually need to clear lower-left... we won't touch it.
/*    for (int i = 0; i < U->nrows; i++) {
      for (int j = 0; j < i; j++) {
//            assert(MATD_EL(U, i, j) == MATD_EL(U, j, i));
MATD_EL(U, i, j) = 0;
}
}
*/
    int is_spd = 1; // (A->nrows == A->ncols);

    for (int i = 0; i < N; i++) {
        float d = MATD_EL(U, i, i);
        is_spd &= (d > 0);

        if (d < MATD_EPS)
            d = MATD_EPS;
        d = 1.0 / sqrt(d);

        for (int j = i; j < N; j++)
            MATD_EL(U, i, j) *= d;

        for (int j = i+1; j < N; j++) {
            float s = MATD_EL(U, i, j);

            if (s == 0)
                continue;

            for (int k = j; k < N; k++) {
                MATD_EL(U, j, k) -= MATD_EL(U, i, k)*s;
            }
        }
    }

    matd_chol_t *chol = calloc(1, sizeof(matd_chol_t));
    chol->is_spd = is_spd;
    chol->u = U;
    return chol;
}

void matd_chol_destroy(matd_chol_t *chol)
{
    matd_destroy(chol->u);
    free(chol);
}

// Solve: (U')x = b, U is upper triangular
void matd_ltransposetriangle_solve(matd_t *u, const TYPE *b, TYPE *x)
{
    int n = u->ncols;
    memcpy(x, b, n*sizeof(TYPE));
    for (int i = 0; i < n; i++) {
        x[i] /= MATD_EL(u, i, i);

        for (int j = i+1; j < u->ncols; j++) {
            x[j] -= x[i] * MATD_EL(u, i, j);
        }
    }
}

// Solve: Lx = b, L is lower triangular
void matd_ltriangle_solve(matd_t *L, const TYPE *b, TYPE *x)
{
    int n = L->ncols;

    for (int i = 0; i < n; i++) {
        float acc = b[i];

        for (int j = 0; j < i; j++) {
            acc -= MATD_EL(L, i, j)*x[j];
        }

        x[i] = acc / MATD_EL(L, i, i);
    }
}

// solve Ux = b, U is upper triangular
void matd_utriangle_solve(matd_t *u, const TYPE *b, TYPE *x)
{
    for (int i = u->ncols-1; i >= 0; i--) {
        float bi = b[i];

        float diag = MATD_EL(u, i, i);

        for (int j = i+1; j < u->ncols; j++)
            bi -= MATD_EL(u, i, j)*x[j];

        x[i] = bi / diag;
    }
}

matd_t *matd_chol_solve(const matd_chol_t *chol, const matd_t *b)
{
    matd_t *u = chol->u;

    matd_t *x = matd_copy(b);

    // LUx = b

    // solve Ly = b ==> (U')y = b

    for (int i = 0; i < u->nrows; i++) {
        for (int j = 0; j < i; j++) {
            // b[i] -= L[i,j]*x[j]... replicated across columns of b
            //   ==> i.e., ==>
            // b[i,k] -= L[i,j]*x[j,k]
            for (int k = 0; k < b->ncols; k++) {
                MATD_EL(x, i, k) -= MATD_EL(u, j, i)*MATD_EL(x, j, k);
            }
        }
        // x[i] = b[i] / L[i,i]
        for (int k = 0; k < b->ncols; k++) {
            MATD_EL(x, i, k) /= MATD_EL(u, i, i);
        }
    }

    // solve Ux = y
    for (int k = u->ncols-1; k >= 0; k--) {
        float LUkk = 1.0 / MATD_EL(u, k, k);
        for (int t = 0; t < b->ncols; t++)
            MATD_EL(x, k, t) *= LUkk;

        for (int i = 0; i < k; i++) {
            float LUik = -MATD_EL(u, i, k);
            for (int t = 0; t < b->ncols; t++)
                MATD_EL(x, i, t) += MATD_EL(x, k, t) *LUik;
        }
    }

    return x;
}

/*void matd_chol_solve(matd_chol_t *chol, const TYPE *b, TYPE *x)
  {
  matd_t *u = chol->u;

  TYPE y[u->ncols];
  matd_ltransposetriangle_solve(u, b, y);
  matd_utriangle_solve(u, y, x);
  }
*/
// only sensible on PSD matrices. had expected it to be faster than
// inverse via LU... for now, doesn't seem to be.
matd_t *matd_chol_inverse(matd_t *a)
{
    assert(a->nrows == a->ncols);

    matd_chol_t *chol = matd_chol(a);

    matd_t *eye = matd_identity(a->nrows);
    matd_t *inv = matd_chol_solve(chol, eye);
    matd_destroy(eye);
    matd_chol_destroy(chol);

    return inv;
}

float matd_max(matd_t *m)
{
    float d = -FLT_MAX;
    for(int x=0; x<m->nrows; x++) {
        for(int y=0; y<m->ncols; y++) {
            if(MATD_EL(m, x, y) > d)
                d = MATD_EL(m, x, y);
        }
    }

    return d;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "homography.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Given a 3x3 homography matrix and the focal lengths of the
     * camera, compute the pose of the tag. The focal lengths should
     * be given in pixels. For example, if the camera's focal length
     * is twice the width of the sensor, and the sensor is 600 pixels
     * across, the focal length in pixels is 2*600. Note that the
     * focal lengths in the fx and fy direction will be approximately
     * equal for most lenses, and is not a function of aspect ratio.
     *
     * Theory: The homography matrix is the product of the camera
     * projection matrix and the tag's pose matrix (the matrix that
     * projects points from the tag's local coordinate system to the
     * camera's coordinate frame).
     *
    * [ h00 h01 h02 h03] = [ fx   0     cx 0 ] [ R00 R01 R02 TX ]
     * [ h10 h11 h12 h13] = [ 0    fy    cy 0 ] [ R10 R11 R12 TY ]
     * [ h20 h21 h22 h23] = [ 0    0      s 0 ] [ R20 R21 R22 TZ ]
     *                                          [ 0   0   0   1  ]
     *
     * fx is the focal length in the x direction of the camera
     * (typically measured in pixels), fy is the focal length. cx and
     * cy give the focal center (usually the middle of the image), and
     * s is either +1 or -1, depending on the conventions you use. (We
     * use 1.)

     * When observing a tag, the points we project in world space all
     * have z=0, so we can form a 3x3 matrix by eliminating the 3rd
     * column of the pose matrix.
     *
     * [ h00 h01 h02 ] = [ fx   0    cx 0 ] [ R00 R01 TX ]
     * [ h10 h11 h12 ] = [ 0    fy   cy 0 ] [ R10 R11 TY ]
     * [ h20 h21 h22 ] = [ 0    0     s 0 ] [ R20 R21 TZ ]
     *                                      [ 0   0   1  ]
     *
     * (note that these h's are different from the ones above.)
     *
     * We can multiply the right-hand side to yield a set of equations
     * relating the values of h to the values of the pose matrix.
     *
     * There are two wrinkles. The first is that the homography matrix
     * is known only up to scale. We recover the unknown scale by
     * constraining the magnitude of the first two columns of the pose
     * matrix to be 1. We use the geometric average scale. The sign of
     * the scale factor is recovered by constraining the observed tag
     * to be in front of the camera. Once scaled, we recover the first
     * two colmuns of the rotation matrix. The third column is the
     * cross product of these.
     *
     * The second wrinkle is that the computed rotation matrix might
     * not be exactly orthogonal, so we perform a polar decomposition
     * to find a good pure rotation approximation.
     *
     * Tagsize is the size of the tag in your desired units. I.e., if
     * your tag measures 0.25m along the side, your tag size is
     * 0.25. (The homography is computed in terms of *half* the tag
     * size, i.e., that a tag is 2 units wide as it spans from -1 to
     * +1, but this code makes the appropriate adjustment.)
     *
     * A note on signs:
     *
     * The code below incorporates no additional negative signs, but
     * respects the sign of any parameters that you pass in. Flipping
     * the signs allows you to modify the projection to suit a wide
     * variety of conditions.
     *
     * In the "pure geometry" projection matrix, the image appears
     * upside down; i.e., the x and y coordinates on the left hand
     * side are the opposite of those on the right of the camera
     * projection matrix. This would happen for all parameters
     * positive: recall that points in front of the camera have
     * negative Z values, which will cause the sign of all points to
     * flip.
     *
     * However, most cameras flip things so that the image appears
     * "right side up" as though you were looking through the lens
     * directly. This means that the projected points should have the
     * same sign as the points on the right of the camera projection
     * matrix. To achieve this, flip fx and fy.
     *
     * One further complication: cameras typically put y=0 at the top
     * of the image, instead of the bottom. Thus you generally want to
     * flip y yet again (so it's now positive again).
     *
     * General advice: you probably want fx negative, fy positive, cx
     * and cy positive, and s=1.
     **/

// correspondences is a list of float[4]s, consisting of the points x
// and y concatenated. We will compute a homography such that y = Hx
// Specifically, float [] { a, b, c, d } where x = [a b], y = [c d].


#define HOMOGRAPHY_COMPUTE_FLAG_INVERSE 1
#define HOMOGRAPHY_COMPUTE_FLAG_SVD 0

matd_t *homography_compute(zarray_t *correspondences, int flags);

//void homography_project(const matd_t *H, float x, float y, float *ox, float *oy);
static inline void homography_project(const matd_t *H, float x, float y, float *ox, float *oy)
{
    float xx = MATD_EL(H, 0, 0)*x + MATD_EL(H, 0, 1)*y + MATD_EL(H, 0, 2);
    float yy = MATD_EL(H, 1, 0)*x + MATD_EL(H, 1, 1)*y + MATD_EL(H, 1, 2);
    float zz = MATD_EL(H, 2, 0)*x + MATD_EL(H, 2, 1)*y + MATD_EL(H, 2, 2);

    *ox = xx / zz;
    *oy = yy / zz;
}

// assuming that the projection matrix is:
// [ fx 0  cx 0 ]
// [  0 fy cy 0 ]
// [  0  0  1 0 ]
//
// And that the homography is equal to the projection matrix times the model matrix,
// recover the model matrix (which is returned). Note that the third column of the model
// matrix is missing in the expresison below, reflecting the fact that the homography assumes
// all points are at z=0 (i.e., planar) and that the element of z is thus omitted.
// (3x1 instead of 4x1).
//
// [ fx 0  cx 0 ] [ R00  R01  TX ]    [ H00 H01 H02 ]
// [  0 fy cy 0 ] [ R10  R11  TY ] =  [ H10 H11 H12 ]
// [  0  0  1 0 ] [ R20  R21  TZ ] =  [ H20 H21 H22 ]
//                [  0    0    1 ]
//
// fx*R00 + cx*R20 = H00   (note, H only known up to scale; some additional adjustments required; see code.)
// fx*R01 + cx*R21 = H01
// fx*TX  + cx*TZ  = H02
// fy*R10 + cy*R20 = H10
// fy*R11 + cy*R21 = H11
// fy*TY  + cy*TZ  = H12
// R20 = H20
// R21 = H21
// TZ  = H22
matd_t *homography_to_pose(const matd_t *H, float fx, float fy, float cx, float cy);

// Similar to above
// Recover the model view matrix assuming that the projection matrix is:
//
// [ F  0  A  0 ]     (see glFrustrum)
// [ 0  G  B  0 ]
// [ 0  0  C  D ]
// [ 0  0 -1  0 ]

matd_t *homography_to_model_view(const matd_t *H, float F, float G, float A, float B, float C, float D);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "homography.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

// correspondences is a list of float[4]s, consisting of the points x
// and y concatenated. We will compute a homography such that y = Hx
matd_t *homography_compute(zarray_t *correspondences, int flags)
{
    // compute centroids of both sets of points (yields a better
    // conditioned information matrix)
    float x_cx = 0, x_cy = 0;
    float y_cx = 0, y_cy = 0;

    for (int i = 0; i < zarray_size(correspondences); i++) {
        float *c;
        zarray_get_volatile(correspondences, i, &c);

        x_cx += c[0];
        x_cy += c[1];
        y_cx += c[2];
        y_cy += c[3];
    }

    int sz = zarray_size(correspondences);
    x_cx /= sz;
    x_cy /= sz;
    y_cx /= sz;
    y_cy /= sz;

    // NB We don't normalize scale; it seems implausible that it could
    // possibly make any difference given the dynamic range of IEEE
    // doubles.

    matd_t *A = matd_create(9,9);
    for (int i = 0; i < zarray_size(correspondences); i++) {
        float *c;
        zarray_get_volatile(correspondences, i, &c);

        // (below world is "x", and image is "y")
        float worldx = c[0] - x_cx;
        float worldy = c[1] - x_cy;
        float imagex = c[2] - y_cx;
        float imagey = c[3] - y_cy;

        float a03 = -worldx;
        float a04 = -worldy;
        float a05 = -1;
        float a06 = worldx*imagey;
        float a07 = worldy*imagey;
        float a08 = imagey;

        MATD_EL(A, 3, 3) += a03*a03;
        MATD_EL(A, 3, 4) += a03*a04;
        MATD_EL(A, 3, 5) += a03*a05;
        MATD_EL(A, 3, 6) += a03*a06;
        MATD_EL(A, 3, 7) += a03*a07;
        MATD_EL(A, 3, 8) += a03*a08;
        MATD_EL(A, 4, 4) += a04*a04;
        MATD_EL(A, 4, 5) += a04*a05;
        MATD_EL(A, 4, 6) += a04*a06;
        MATD_EL(A, 4, 7) += a04*a07;
        MATD_EL(A, 4, 8) += a04*a08;
        MATD_EL(A, 5, 5) += a05*a05;
        MATD_EL(A, 5, 6) += a05*a06;
        MATD_EL(A, 5, 7) += a05*a07;
        MATD_EL(A, 5, 8) += a05*a08;
        MATD_EL(A, 6, 6) += a06*a06;
        MATD_EL(A, 6, 7) += a06*a07;
        MATD_EL(A, 6, 8) += a06*a08;
        MATD_EL(A, 7, 7) += a07*a07;
        MATD_EL(A, 7, 8) += a07*a08;
        MATD_EL(A, 8, 8) += a08*a08;

        float a10 = worldx;
        float a11 = worldy;
        float a12 = 1;
        float a16 = -worldx*imagex;
        float a17 = -worldy*imagex;
        float a18 = -imagex;

        MATD_EL(A, 0, 0) += a10*a10;
        MATD_EL(A, 0, 1) += a10*a11;
        MATD_EL(A, 0, 2) += a10*a12;
        MATD_EL(A, 0, 6) += a10*a16;
        MATD_EL(A, 0, 7) += a10*a17;
        MATD_EL(A, 0, 8) += a10*a18;
        MATD_EL(A, 1, 1) += a11*a11;
        MATD_EL(A, 1, 2) += a11*a12;
        MATD_EL(A, 1, 6) += a11*a16;
        MATD_EL(A, 1, 7) += a11*a17;
        MATD_EL(A, 1, 8) += a11*a18;
        MATD_EL(A, 2, 2) += a12*a12;
        MATD_EL(A, 2, 6) += a12*a16;
        MATD_EL(A, 2, 7) += a12*a17;
        MATD_EL(A, 2, 8) += a12*a18;
        MATD_EL(A, 6, 6) += a16*a16;
        MATD_EL(A, 6, 7) += a16*a17;
        MATD_EL(A, 6, 8) += a16*a18;
        MATD_EL(A, 7, 7) += a17*a17;
        MATD_EL(A, 7, 8) += a17*a18;
        MATD_EL(A, 8, 8) += a18*a18;

        float a20 = -worldx*imagey;
        float a21 = -worldy*imagey;
        float a22 = -imagey;
        float a23 = worldx*imagex;
        float a24 = worldy*imagex;
        float a25 = imagex;

        MATD_EL(A, 0, 0) += a20*a20;
        MATD_EL(A, 0, 1) += a20*a21;
        MATD_EL(A, 0, 2) += a20*a22;
        MATD_EL(A, 0, 3) += a20*a23;
        MATD_EL(A, 0, 4) += a20*a24;
        MATD_EL(A, 0, 5) += a20*a25;
        MATD_EL(A, 1, 1) += a21*a21;
        MATD_EL(A, 1, 2) += a21*a22;
        MATD_EL(A, 1, 3) += a21*a23;
        MATD_EL(A, 1, 4) += a21*a24;
        MATD_EL(A, 1, 5) += a21*a25;
        MATD_EL(A, 2, 2) += a22*a22;
        MATD_EL(A, 2, 3) += a22*a23;
        MATD_EL(A, 2, 4) += a22*a24;
        MATD_EL(A, 2, 5) += a22*a25;
        MATD_EL(A, 3, 3) += a23*a23;
        MATD_EL(A, 3, 4) += a23*a24;
        MATD_EL(A, 3, 5) += a23*a25;
        MATD_EL(A, 4, 4) += a24*a24;
        MATD_EL(A, 4, 5) += a24*a25;
        MATD_EL(A, 5, 5) += a25*a25;
    }

    // make symmetric
    for (int i = 0; i < 9; i++)
        for (int j = i+1; j < 9; j++)
            MATD_EL(A, j, i) = MATD_EL(A, i, j);

    matd_t *H = matd_create(3,3);

    if (flags & HOMOGRAPHY_COMPUTE_FLAG_INVERSE) {
        // compute singular vector by (carefully) inverting the rank-deficient matrix.

        if (1) {
            matd_t *Ainv = matd_inverse(A);
            float scale = 0;

            for (int i = 0; i < 9; i++)
                scale += sq(MATD_EL(Ainv, i, 0));
            scale = sqrt(scale);

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    MATD_EL(H, i, j) = MATD_EL(Ainv, 3*i+j, 0) / scale;

            matd_destroy(Ainv);
        } else {

            matd_t *b = matd_create_data(9, 1, (float[]) { 1, 0, 0, 0, 0, 0, 0, 0, 0 });
            matd_t *Ainv = NULL;

            if (0) {
                matd_plu_t *lu = matd_plu(A);
                Ainv = matd_plu_solve(lu, b);
                matd_plu_destroy(lu);
            } else {
                matd_chol_t *chol = matd_chol(A);
                Ainv = matd_chol_solve(chol, b);
                matd_chol_destroy(chol);
            }

            float scale = 0;

            for (int i = 0; i < 9; i++)
                scale += sq(MATD_EL(Ainv, i, 0));
            scale = sqrt(scale);

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    MATD_EL(H, i, j) = MATD_EL(Ainv, 3*i+j, 0) / scale;

            matd_destroy(b);
            matd_destroy(Ainv);
        }

    } else {
        // compute singular vector using SVD. A bit slower, but more accurate.
        matd_svd_t svd = matd_svd_flags(A, MATD_SVD_NO_WARNINGS);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                MATD_EL(H, i, j) = MATD_EL(svd.U, 3*i+j, 8);

        matd_destroy(svd.U);
        matd_destroy(svd.S);
        matd_destroy(svd.V);

    }

    matd_t *Tx = matd_identity(3);
    MATD_EL(Tx,0,2) = -x_cx;
    MATD_EL(Tx,1,2) = -x_cy;

    matd_t *Ty = matd_identity(3);
    MATD_EL(Ty,0,2) = y_cx;
    MATD_EL(Ty,1,2) = y_cy;

    matd_t *H2 = matd_op("M*M*M", Ty, H, Tx);

    matd_destroy(A);
    matd_destroy(Tx);
    matd_destroy(Ty);
    matd_destroy(H);

    return H2;
}


// assuming that the projection matrix is:
// [ fx 0  cx 0 ]
// [  0 fy cy 0 ]
// [  0  0  1 0 ]
//
// And that the homography is equal to the projection matrix times the
// model matrix, recover the model matrix (which is returned). Note
// that the third column of the model matrix is missing in the
// expresison below, reflecting the fact that the homography assumes
// all points are at z=0 (i.e., planar) and that the element of z is
// thus omitted.  (3x1 instead of 4x1).
//
// [ fx 0  cx 0 ] [ R00  R01  TX ]    [ H00 H01 H02 ]
// [  0 fy cy 0 ] [ R10  R11  TY ] =  [ H10 H11 H12 ]
// [  0  0  1 0 ] [ R20  R21  TZ ] =  [ H20 H21 H22 ]
//                [  0    0    1 ]
//
// fx*R00 + cx*R20 = H00   (note, H only known up to scale; some additional adjustments required; see code.)
// fx*R01 + cx*R21 = H01
// fx*TX  + cx*TZ  = H02
// fy*R10 + cy*R20 = H10
// fy*R11 + cy*R21 = H11
// fy*TY  + cy*TZ  = H12
// R20 = H20
// R21 = H21
// TZ  = H22

matd_t *homography_to_pose(const matd_t *H, float fx, float fy, float cx, float cy)
{
    // Note that every variable that we compute is proportional to the scale factor of H.
    float R20 = MATD_EL(H, 2, 0);
    float R21 = MATD_EL(H, 2, 1);
    float TZ  = MATD_EL(H, 2, 2);
    float R00 = (MATD_EL(H, 0, 0) - cx*R20) / fx;
    float R01 = (MATD_EL(H, 0, 1) - cx*R21) / fx;
    float TX  = (MATD_EL(H, 0, 2) - cx*TZ)  / fx;
    float R10 = (MATD_EL(H, 1, 0) - cy*R20) / fy;
    float R11 = (MATD_EL(H, 1, 1) - cy*R21) / fy;
    float TY  = (MATD_EL(H, 1, 2) - cy*TZ)  / fy;

    // compute the scale by requiring that the rotation columns are unit length
    // (Use geometric average of the two length vectors we have)
    float length1 = sqrtf(R00*R00 + R10*R10 + R20*R20);
    float length2 = sqrtf(R01*R01 + R11*R11 + R21*R21);
    float s = 1.0 / sqrtf(length1 * length2);

    // get sign of S by requiring the tag to be in front the camera;
    // we assume camera looks in the -Z direction.
    if (TZ > 0)
        s *= -1;

    R20 *= s;
    R21 *= s;
    TZ  *= s;
    R00 *= s;
    R01 *= s;
    TX  *= s;
    R10 *= s;
    R11 *= s;
    TY  *= s;

    // now recover [R02 R12 R22] by noting that it is the cross product of the other two columns.
    float R02 = R10*R21 - R20*R11;
    float R12 = R20*R01 - R00*R21;
    float R22 = R00*R11 - R10*R01;

    // Improve rotation matrix by applying polar decomposition.
    if (1) {
        // do polar decomposition. This makes the rotation matrix
        // "proper", but probably increases the reprojection error. An
        // iterative alignment step would be superior.

        matd_t *R = matd_create_data(3, 3, (float[]) { R00, R01, R02,
                                                       R10, R11, R12,
                                                       R20, R21, R22 });

        matd_svd_t svd = matd_svd(R);
        matd_destroy(R);

        R = matd_op("M*M'", svd.U, svd.V);

        matd_destroy(svd.U);
        matd_destroy(svd.S);
        matd_destroy(svd.V);

        R00 = MATD_EL(R, 0, 0);
        R01 = MATD_EL(R, 0, 1);
        R02 = MATD_EL(R, 0, 2);
        R10 = MATD_EL(R, 1, 0);
        R11 = MATD_EL(R, 1, 1);
        R12 = MATD_EL(R, 1, 2);
        R20 = MATD_EL(R, 2, 0);
        R21 = MATD_EL(R, 2, 1);
        R22 = MATD_EL(R, 2, 2);

        matd_destroy(R);
    }

    return matd_create_data(4, 4, (float[]) { R00, R01, R02, TX,
                                               R10, R11, R12, TY,
                                               R20, R21, R22, TZ,
                                                0, 0, 0, 1 });
}

// Similar to above
// Recover the model view matrix assuming that the projection matrix is:
//
// [ F  0  A  0 ]     (see glFrustrum)
// [ 0  G  B  0 ]
// [ 0  0  C  D ]
// [ 0  0 -1  0 ]

matd_t *homography_to_model_view(const matd_t *H, float F, float G, float A, float B, float C, float D)
{
    // Note that every variable that we compute is proportional to the scale factor of H.
    float R20 = -MATD_EL(H, 2, 0);
    float R21 = -MATD_EL(H, 2, 1);
    float TZ  = -MATD_EL(H, 2, 2);
    float R00 = (MATD_EL(H, 0, 0) - A*R20) / F;
    float R01 = (MATD_EL(H, 0, 1) - A*R21) / F;
    float TX  = (MATD_EL(H, 0, 2) - A*TZ)  / F;
    float R10 = (MATD_EL(H, 1, 0) - B*R20) / G;
    float R11 = (MATD_EL(H, 1, 1) - B*R21) / G;
    float TY  = (MATD_EL(H, 1, 2) - B*TZ)  / G;

    // compute the scale by requiring that the rotation columns are unit length
    // (Use geometric average of the two length vectors we have)
    float length1 = sqrtf(R00*R00 + R10*R10 + R20*R20);
    float length2 = sqrtf(R01*R01 + R11*R11 + R21*R21);
    float s = 1.0 / sqrtf(length1 * length2);

    // get sign of S by requiring the tag to be in front of the camera
    // (which is Z < 0) for our conventions.
    if (TZ > 0)
        s *= -1;

    R20 *= s;
    R21 *= s;
    TZ  *= s;
    R00 *= s;
    R01 *= s;
    TX  *= s;
    R10 *= s;
    R11 *= s;
    TY  *= s;

    // now recover [R02 R12 R22] by noting that it is the cross product of the other two columns.
    float R02 = R10*R21 - R20*R11;
    float R12 = R20*R01 - R00*R21;
    float R22 = R00*R11 - R10*R01;

    // TODO XXX: Improve rotation matrix by applying polar decomposition.

    return matd_create_data(4, 4, (float[]) { R00, R01, R02, TX,
        R10, R11, R12, TY,
        R20, R21, R22, TZ,
        0, 0, 0, 1 });
}

// Only uses the upper 3x3 matrix.
/*
static void matrix_to_quat(const matd_t *R, float q[4])
{
    // see: "from quaternion to matrix and back"

    // trace: get the same result if R is 4x4 or 3x3:
    float T = MATD_EL(R, 0, 0) + MATD_EL(R, 1, 1) + MATD_EL(R, 2, 2) + 1;
    float S = 0;

    float m0  = MATD_EL(R, 0, 0);
    float m1  = MATD_EL(R, 1, 0);
    float m2  = MATD_EL(R, 2, 0);
    float m4  = MATD_EL(R, 0, 1);
    float m5  = MATD_EL(R, 1, 1);
    float m6  = MATD_EL(R, 2, 1);
    float m8  = MATD_EL(R, 0, 2);
    float m9  = MATD_EL(R, 1, 2);
    float m10 = MATD_EL(R, 2, 2);

    if (T > 0.0000001) {
        S = sqrtf(T) * 2;
        q[1] = -( m9 - m6 ) / S;
        q[2] = -( m2 - m8 ) / S;
        q[3] = -( m4 - m1 ) / S;
        q[0] = 0.25 * S;
    } else if ( m0 > m5 && m0 > m10 )  {	// Column 0:
        S  = sqrtf( 1.0 + m0 - m5 - m10 ) * 2;
        q[1] = -0.25 * S;
        q[2] = -(m4 + m1 ) / S;
        q[3] = -(m2 + m8 ) / S;
        q[0] = (m9 - m6 ) / S;
    } else if ( m5 > m10 ) {			// Column 1:
        S  = sqrtf( 1.0 + m5 - m0 - m10 ) * 2;
        q[1] = -(m4 + m1 ) / S;
        q[2] = -0.25 * S;
        q[3] = -(m9 + m6 ) / S;
        q[0] = (m2 - m8 ) / S;
    } else {
        // Column 2:
        S  = sqrtf( 1.0 + m10 - m0 - m5 ) * 2;
        q[1] = -(m2 + m8 ) / S;
        q[2] = -(m9 + m6 ) / S;
        q[3] = -0.25 * S;
        q[0] = (m4 - m1 ) / S;
    }

    float mag2 = 0;
    for (int i = 0; i < 4; i++)
        mag2 += q[i]*q[i];
    float norm = 1.0 / sqrtf(mag2);
    for (int i = 0; i < 4; i++)
        q[i] *= norm;
}
*/

// overwrites upper 3x3 area of matrix M. Doesn't touch any other elements of M.
void quat_to_matrix(const float q[4], matd_t *M)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    MATD_EL(M, 0, 0) = w*w + x*x - y*y - z*z;
    MATD_EL(M, 0, 1) = 2*x*y - 2*w*z;
    MATD_EL(M, 0, 2) = 2*x*z + 2*w*y;

    MATD_EL(M, 1, 0) = 2*x*y + 2*w*z;
    MATD_EL(M, 1, 1) = w*w - x*x + y*y - z*z;
    MATD_EL(M, 1, 2) = 2*y*z - 2*w*x;

    MATD_EL(M, 2, 0) = 2*x*z - 2*w*y;
    MATD_EL(M, 2, 1) = 2*y*z + 2*w*x;
    MATD_EL(M, 2, 2) = w*w - x*x - y*y + z*z;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "g2d.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

// This library tries to avoid needless proliferation of types.
//
// A point is a float[2]. (Note that when passing a float[2] as an
// argument, it is passed by pointer, not by value.)
//
// A polygon is a zarray_t of float[2]. (Note that in this case, the
// zarray contains the actual vertex data, and not merely a pointer to
// some other data. IMPORTANT: A polygon must be specified in CCW
// order.  It is implicitly closed (do not list the same point at the
// beginning at the end.
//
// Where sensible, it is assumed that objects should be allocated
// sparingly; consequently "init" style methods, rather than "create"
// methods are used.

////////////////////////////////////////////////////////////////////
// Lines

typedef struct
{
    // Internal representation: a point that the line goes through (p) and
    // the direction of the line (u).
    float p[2];
    float u[2]; // always a unit vector
} g2d_line_t;

// initialize a line object.
void g2d_line_init_from_points(g2d_line_t *line, const float p0[2], const float p1[2]);

// The line defines a one-dimensional coordinate system whose origin
// is p. Where is q? (If q is not on the line, the point nearest q is
// returned.
float g2d_line_get_coordinate(const g2d_line_t *line, const float q[2]);

// Intersect two lines. The intersection, if it exists, is written to
// p (if not NULL), and 1 is returned. Else, zero is returned.
int g2d_line_intersect_line(const g2d_line_t *linea, const g2d_line_t *lineb, float *p);

////////////////////////////////////////////////////////////////////
// Line Segments. line.p is always one endpoint; p1 is the other
// endpoint.
typedef struct
{
    g2d_line_t line;
    float p1[2];
} g2d_line_segment_t;

void g2d_line_segment_init_from_points(g2d_line_segment_t *seg, const float p0[2], const float p1[2]);

// Intersect two segments. The intersection, if it exists, is written
// to p (if not NULL), and 1 is returned. Else, zero is returned.
int g2d_line_segment_intersect_segment(const g2d_line_segment_t *sega, const g2d_line_segment_t *segb, float *p);

void g2d_line_segment_closest_point(const g2d_line_segment_t *seg, const float *q, float *p);
float g2d_line_segment_closest_point_distance(const g2d_line_segment_t *seg, const float *q);

////////////////////////////////////////////////////////////////////
// Polygons

zarray_t *g2d_polygon_create_data(float v[][2], int sz);

zarray_t *g2d_polygon_create_zeros(int sz);

zarray_t *g2d_polygon_create_empty();

void g2d_polygon_add(zarray_t *poly, float v[2]);

// Takes a polygon in either CW or CCW and modifies it (if necessary)
// to be CCW.
void g2d_polygon_make_ccw(zarray_t *poly);

// Return 1 if point q lies within poly.
int g2d_polygon_contains_point(const zarray_t *poly, float q[2]);

// Do the edges of the polygons cross? (Does not test for containment).
int g2d_polygon_intersects_polygon(const zarray_t *polya, const zarray_t *polyb);

// Does polya completely contain polyb?
int g2d_polygon_contains_polygon(const zarray_t *polya, const zarray_t *polyb);

// Is there some point which is in both polya and polyb?
int g2d_polygon_overlaps_polygon(const zarray_t *polya, const zarray_t *polyb);

// returns the number of points written to x. see comments.
int g2d_polygon_rasterize(const zarray_t *poly, float y, float *x);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "g2d.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

float g2d_distance(const float a[2], const float b[2])
{
    return sqrtf(sq(a[0]-b[0]) + sq(a[1]-b[1]));
}

zarray_t *g2d_polygon_create_empty()
{
    return zarray_create(sizeof(float[2]));
}

void g2d_polygon_add(zarray_t *poly, float v[2])
{
    zarray_add(poly, v);
}

zarray_t *g2d_polygon_create_data(float v[][2], int sz)
{
    zarray_t *points = g2d_polygon_create_empty();

    for (int i = 0; i < sz; i++)
        g2d_polygon_add(points, v[i]);

    return points;
}

zarray_t *g2d_polygon_create_zeros(int sz)
{
    zarray_t *points = zarray_create(sizeof(float[2]));

    float z[2] = { 0, 0 };

    for (int i = 0; i < sz; i++)
        zarray_add(points, z);

    return points;
}

void g2d_polygon_make_ccw(zarray_t *poly)
{
    // Step one: we want the points in counter-clockwise order.
    // If the points are in clockwise order, we'll reverse them.
    float total_theta = 0;
    float last_theta = 0;

    // Count the angle accumulated going around the polygon. If
    // the sum is +2pi, it's CCW. Otherwise, we'll get -2pi.
    int sz = zarray_size(poly);

    for (int i = 0; i <= sz; i++) {
        float p0[2], p1[2];
        zarray_get(poly, i % sz, &p0);
        zarray_get(poly, (i+1) % sz, &p1);

        float this_theta = atan2(p1[1]-p0[1], p1[0]-p0[0]);

        if (i > 0) {
            float dtheta = mod2pi(this_theta-last_theta);
            total_theta += dtheta;
        }

        last_theta = this_theta;
    }

    int ccw = (total_theta > 0);

    // reverse order if necessary.
    if (!ccw) {
        for (int i = 0; i < sz / 2; i++) {
            float a[2], b[2];

            zarray_get(poly, i, a);
            zarray_get(poly, sz-1-i, b);
            zarray_set(poly, i, b, NULL);
            zarray_set(poly, sz-1-i, a, NULL);
        }
    }
}

int g2d_polygon_contains_point_ref(const zarray_t *poly, float q[2])
{
    // use winding. If the point is inside the polygon, we'll wrap
    // around it (accumulating 6.28 radians). If we're outside the
    // polygon, we'll accumulate zero.
    int psz = zarray_size(poly);

    float acc_theta = 0;

    float last_theta;

    for (int i = 0; i <= psz; i++) {
        float p[2];

        zarray_get(poly, i % psz, &p);

        float this_theta = atan2(q[1]-p[1], q[0]-p[0]);

        if (i != 0)
            acc_theta += mod2pi(this_theta - last_theta);

        last_theta = this_theta;
    }

    return acc_theta > M_PI;
}

/*
// sort by x coordinate, ascending
static int g2d_convex_hull_sort(const void *_a, const void *_b)
{
    float *a = (float*) _a;
    float *b = (float*) _b;

    if (a[0] < b[0])
        return -1;
    if (a[0] == b[0])
        return 0;
    return 1;
}
*/

/*
zarray_t *g2d_convex_hull2(const zarray_t *points)
{
    zarray_t *hull = zarray_copy(points);

    zarray_sort(hull, g2d_convex_hull_sort);

    int hsz = zarray_size(hull);
    int hout = 0;

    for (int hin = 1; hin < hsz; hin++) {
        float *p;
        zarray_get_volatile(hull, i, &p);

        // Everything to the right of hin is already convex. We now
        // add one point, p, which begins "connected" by two
        // (coincident) edges from the last right-most point to p.
        float *last;
        zarray_get_volatile(hull, hout, &last);

        // We now remove points from the convex hull by moving
    }

    return hull;
}
*/

// creates and returns a zarray(float[2]). The resulting polygon is
// CCW and implicitly closed. Unnecessary colinear points are omitted.
zarray_t *g2d_convex_hull(const zarray_t *points)
{
    zarray_t *hull = zarray_create(sizeof(float[2]));

    // gift-wrap algorithm.

    // step 1: find left most point.
    int insz = zarray_size(points);

    // must have at least 2 points. (XXX need 3?)
    assert(insz >= 2);

    float *pleft = NULL;
    for (int i = 0; i < insz; i++) {
        float *p;
        zarray_get_volatile(points, i, &p);

        if (pleft == NULL || p[0] < pleft[0])
            pleft = p;
    }

    // cannot be NULL since there must be at least one point.
    assert(pleft != NULL);

    zarray_add(hull, pleft);

    // step 2. gift wrap. Keep searching for points that make the
    // smallest-angle left-hand turn. This implementation is carefully
    // written to use only addition/subtraction/multiply. No division
    // or sqrts. This guarantees exact results for integer-coordinate
    // polygons (no rounding/precision problems).
    float *p = pleft;

    while (1) {
        assert(p != NULL);

        float *q = NULL;
        float n0 = 0, n1 = 0; // the normal to the line (p, q) (not
                       // necessarily unit length).

        // Search for the point q for which the line (p,q) is most "to
        // the right of" the other points. (i.e., every time we find a
        // point that is to the right of our current line, we change
        // lines.)
        for (int i = 0; i < insz; i++) {
            float *thisq;
            zarray_get_volatile(points, i, &thisq);

            if (thisq == p)
                continue;

            // the first time we find another point, we initialize our
            // value of q, forming the line (p,q)
            if (q == NULL) {
                q = thisq;
                n0 = q[1] - p[1];
                n1 = -q[0] + p[0];
            } else {
                // we already have a line (p,q). is point thisq RIGHT OF line (p, q)?
                float e0 = thisq[0] - p[0], e1 = thisq[1] - p[1];
                float dot = e0*n0 + e1*n1;

                if (dot > 0) {
                    // it is. change our line.
                    q = thisq;
                    n0 = q[1] - p[1];
                    n1 = -q[0] + p[0];
                }
            }
        }

        // we must have elected *some* line, so long as there are at
        // least 2 points in the polygon.
        assert(q != NULL);

        // loop completed?
        if (q == pleft)
            break;

        int colinear = 0;

        // is this new point colinear with the last two?
        if (zarray_size(hull) > 1) {
            float *o;
            zarray_get_volatile(hull, zarray_size(hull) - 2, &o);

            float e0 = o[0] - p[0];
            float e1 = o[1] - p[1];

            if (n0*e0 + n1*e1 == 0)
                colinear = 1;
        }

        // if it is colinear, overwrite the last one.
        if (colinear)
            zarray_set(hull, zarray_size(hull)-1, q, NULL);
        else
            zarray_add(hull, q);

        p = q;
    }

    return hull;
}

// Find point p on the boundary of poly that is closest to q.
void g2d_polygon_closest_boundary_point(const zarray_t *poly, const float q[2], float *p)
{
    int psz = zarray_size(poly);
    float min_dist = HUGE_VALF;

    for (int i = 0; i < psz; i++) {
        float *p0, *p1;

        zarray_get_volatile(poly, i, &p0);
        zarray_get_volatile(poly, (i+1) % psz, &p1);

        g2d_line_segment_t seg;
        g2d_line_segment_init_from_points(&seg, p0, p1);

        float thisp[2];
        g2d_line_segment_closest_point(&seg, q, thisp);

        float dist = g2d_distance(q, thisp);
        if (dist < min_dist) {
            memcpy(p, thisp, sizeof(float[2]));
            min_dist = dist;
        }
    }
}

int g2d_polygon_contains_point(const zarray_t *poly, float q[2])
{
    // use winding. If the point is inside the polygon, we'll wrap
    // around it (accumulating 6.28 radians). If we're outside the
    // polygon, we'll accumulate zero.
    int psz = zarray_size(poly);
    assert(psz > 0);

    int last_quadrant;
    int quad_acc = 0;

    for (int i = 0; i <= psz; i++) {
        float *p;

        zarray_get_volatile(poly, i % psz, &p);

        // p[0] < q[0]       p[1] < q[1]    quadrant
        //     0                 0              0
        //     0                 1              3
        //     1                 0              1
        //     1                 1              2

        // p[1] < q[1]       p[0] < q[0]    quadrant
        //     0                 0              0
        //     0                 1              1
        //     1                 0              3
        //     1                 1              2

        int quadrant;
        if (p[0] < q[0])
            quadrant = (p[1] < q[1]) ? 2 : 1;
        else
            quadrant = (p[1] < q[1]) ? 3 : 0;

        if (i > 0) {
            int dquadrant = quadrant - last_quadrant;

            // encourage a jump table by mapping to small positive integers.
            switch (dquadrant) {
                case -3:
                case 1:
                    quad_acc ++;
                    break;
                case -1:
                case 3:
                    quad_acc --;
                    break;
                case 0:
                    break;
                case -2:
                case 2:
                {
                    // get the previous point.
                    float *p0;
                    zarray_get_volatile(poly, i-1, &p0);

                    // Consider the points p0 and p (the points around the
                    //polygon that we are tracing) and the query point q.
                    //
                    // If we've moved diagonally across quadrants, we want
                    // to measure whether we have rotated +PI radians or
                    // -PI radians. We can test this by computing the dot
                    // product of vector (p0-q) with the vector
                    // perpendicular to vector (p-q)
                    float nx = p[1] - q[1];
                    float ny = -p[0] + q[0];

                    float dot = nx*(p0[0]-q[0]) + ny*(p0[1]-q[1]);
                    if (dot < 0)
                        quad_acc -= 2;
                    else
                        quad_acc += 2;

                    break;
                }
            }
        }

        last_quadrant = quadrant;
    }

    int v = (quad_acc >= 2) || (quad_acc <= -2);

    if (0 && v != g2d_polygon_contains_point_ref(poly, q)) {
        printf("FAILURE %d %d\n", v, quad_acc);
        exit(-1);
    }

    return v;
}

void g2d_line_init_from_points(g2d_line_t *line, const float p0[2], const float p1[2])
{
    line->p[0] = p0[0];
    line->p[1] = p0[1];
    line->u[0] = p1[0]-p0[0];
    line->u[1] = p1[1]-p0[1];
    float mag = sqrtf(sq(line->u[0]) + sq(line->u[1]));

    line->u[0] /= mag;
    line->u[1] /= mag;
}

float g2d_line_get_coordinate(const g2d_line_t *line, const float q[2])
{
    return (q[0]-line->p[0])*line->u[0] + (q[1]-line->p[1])*line->u[1];
}

// Compute intersection of two line segments. If they intersect,
// result is stored in p and 1 is returned. Otherwise, zero is
// returned. p may be NULL.
int g2d_line_intersect_line(const g2d_line_t *linea, const g2d_line_t *lineb, float *p)
{
    // this implementation is many times faster than the original,
    // mostly due to avoiding a general-purpose LU decomposition in
    // Matrix.inverse().
    float m00, m01, m10, m11;
    float i00, i01;
    float b00, b10;

    m00 = linea->u[0];
    m01= -lineb->u[0];
    m10 = linea->u[1];
    m11= -lineb->u[1];

    // determinant of m
    float det = m00*m11-m01*m10;

    // parallel lines?
    if (fabs(det) < 0.00000001)
        return 0;

    // inverse of m
    i00 = m11/det;
    i01 = -m01/det;

    b00 = lineb->p[0] - linea->p[0];
    b10 = lineb->p[1] - linea->p[1];

    float x00; //, x10;
    x00 = i00*b00+i01*b10;

    if (p != NULL) {
        p[0] = linea->u[0]*x00 + linea->p[0];
        p[1] = linea->u[1]*x00 + linea->p[1];
    }

    return 1;
}


void g2d_line_segment_init_from_points(g2d_line_segment_t *seg, const float p0[2], const float p1[2])
{
    g2d_line_init_from_points(&seg->line, p0, p1);
    seg->p1[0] = p1[0];
    seg->p1[1] = p1[1];
}

// Find the point p on segment seg that is closest to point q.
void g2d_line_segment_closest_point(const g2d_line_segment_t *seg, const float *q, float *p)
{
    float a = g2d_line_get_coordinate(&seg->line, seg->line.p);
    float b = g2d_line_get_coordinate(&seg->line, seg->p1);
    float c = g2d_line_get_coordinate(&seg->line, q);

    if (a < b)
        c = dclamp(c, a, b);
    else
        c = dclamp(c, b, a);

    p[0] = seg->line.p[0] + c * seg->line.u[0];
    p[1] = seg->line.p[1] + c * seg->line.u[1];
}

// Compute intersection of two line segments. If they intersect,
// result is stored in p and 1 is returned. Otherwise, zero is
// returned. p may be NULL.
int g2d_line_segment_intersect_segment(const g2d_line_segment_t *sega, const g2d_line_segment_t *segb, float *p)
{
    float tmp[2];

    if (!g2d_line_intersect_line(&sega->line, &segb->line, tmp))
        return 0;

    float a = g2d_line_get_coordinate(&sega->line, sega->line.p);
    float b = g2d_line_get_coordinate(&sega->line, sega->p1);
    float c = g2d_line_get_coordinate(&sega->line, tmp);

    // does intersection lie on the first line?
    if ((c<a && c<b) || (c>a && c>b))
        return 0;

    a = g2d_line_get_coordinate(&segb->line, segb->line.p);
    b = g2d_line_get_coordinate(&segb->line, segb->p1);
    c = g2d_line_get_coordinate(&segb->line, tmp);

    // does intersection lie on second line?
    if ((c<a && c<b) || (c>a && c>b))
        return 0;

    if (p != NULL) {
        p[0] = tmp[0];
        p[1] = tmp[1];
    }

    return 1;
}

// Compute intersection of a line segment and a line. If they
// intersect, result is stored in p and 1 is returned. Otherwise, zero
// is returned. p may be NULL.
int g2d_line_segment_intersect_line(const g2d_line_segment_t *seg, const g2d_line_t *line, float *p)
{
    float tmp[2];

    if (!g2d_line_intersect_line(&seg->line, line, tmp))
        return 0;

    float a = g2d_line_get_coordinate(&seg->line, seg->line.p);
    float b = g2d_line_get_coordinate(&seg->line, seg->p1);
    float c = g2d_line_get_coordinate(&seg->line, tmp);

    // does intersection lie on the first line?
    if ((c<a && c<b) || (c>a && c>b))
        return 0;

    if (p != NULL) {
        p[0] = tmp[0];
        p[1] = tmp[1];
    }

    return 1;
}

// do the edges of polya and polyb collide? (Does NOT test for containment).
int g2d_polygon_intersects_polygon(const zarray_t *polya, const zarray_t *polyb)
{
    // do any of the line segments collide? If so, the answer is no.

    // dumb N^2 method.
    for (int ia = 0; ia < zarray_size(polya); ia++) {
        float pa0[2], pa1[2];
        zarray_get(polya, ia, pa0);
        zarray_get(polya, (ia+1)%zarray_size(polya), pa1);

        g2d_line_segment_t sega;
        g2d_line_segment_init_from_points(&sega, pa0, pa1);

        for (int ib = 0; ib < zarray_size(polyb); ib++) {
            float pb0[2], pb1[2];
            zarray_get(polyb, ib, pb0);
            zarray_get(polyb, (ib+1)%zarray_size(polyb), pb1);

            g2d_line_segment_t segb;
            g2d_line_segment_init_from_points(&segb, pb0, pb1);

            if (g2d_line_segment_intersect_segment(&sega, &segb, NULL))
                return 1;
        }
    }

    return 0;
}

// does polya completely contain polyb?
int g2d_polygon_contains_polygon(const zarray_t *polya, const zarray_t *polyb)
{
    // do any of the line segments collide? If so, the answer is no.
    if (g2d_polygon_intersects_polygon(polya, polyb))
        return 0;

    // if none of the edges cross, then the polygon is either fully
    // contained or fully outside.
    float p[2];
    zarray_get(polyb, 0, p);

    return g2d_polygon_contains_point(polya, p);
}

// compute a point that is inside the polygon. (It may not be *far* inside though)
void g2d_polygon_get_interior_point(const zarray_t *poly, float *p)
{
    // take the first three points, which form a triangle. Find the middle point
    float a[2], b[2], c[2];

    zarray_get(poly, 0, a);
    zarray_get(poly, 1, b);
    zarray_get(poly, 2, c);

    p[0] = (a[0]+b[0]+c[0])/3;
    p[1] = (a[1]+b[1]+c[1])/3;
}

int g2d_polygon_overlaps_polygon(const zarray_t *polya, const zarray_t *polyb)
{
    // do any of the line segments collide? If so, the answer is yes.
    if (g2d_polygon_intersects_polygon(polya, polyb))
        return 1;

    // if none of the edges cross, then the polygon is either fully
    // contained or fully outside.
    float p[2];
    g2d_polygon_get_interior_point(polyb, p);

    if (g2d_polygon_contains_point(polya, p))
        return 1;

    g2d_polygon_get_interior_point(polya, p);

    if (g2d_polygon_contains_point(polyb, p))
        return 1;

    return 0;
}

static int double_sort_up(const void *_a, const void *_b)
{
    float a = *((float*) _a);
    float b = *((float*) _b);

    if (a < b)
        return -1;

    if (a == b)
        return 0;

    return 1;
}

// Compute the crossings of the polygon along line y, storing them in
// the array x. X must be allocated to be at least as long as
// zarray_size(poly). X will be sorted, ready for
// rasterization. Returns the number of intersections (and elements
// written to x).
/*
  To rasterize, do something like this:

  float res = 0.099;
  for (float y = y0; y < y1; y += res) {
  float xs[zarray_size(poly)];

  int xsz = g2d_polygon_rasterize(poly, y, xs);
  int xpos = 0;
  int inout = 0; // start off "out"

  for (float x = x0; x < x1; x += res) {
      while (x > xs[xpos] && xpos < xsz) {
        xpos++;
        inout ^= 1;
      }

    if (inout)
       printf("y");
    else
       printf(" ");
  }
  printf("\n");
*/

// returns the number of x intercepts
int g2d_polygon_rasterize(const zarray_t *poly, float y, float *x)
{
    int sz = zarray_size(poly);

    g2d_line_t line;
    if (1) {
        float p0[2] = { 0, y };
        float p1[2] = { 1, y };

        g2d_line_init_from_points(&line, p0, p1);
    }

    int xpos = 0;

    for (int i = 0; i < sz; i++) {
        g2d_line_segment_t seg;
        float *p0, *p1;
        zarray_get_volatile(poly, i, &p0);
        zarray_get_volatile(poly, (i+1)%sz, &p1);

        g2d_line_segment_init_from_points(&seg, p0, p1);

        float q[2];
        if (g2d_line_segment_intersect_line(&seg, &line, q))
            x[xpos++] = q[0];
    }

    qsort(x, xpos, sizeof(float), double_sort_up);

    return xpos;
}

/*
  /---(1,5)
  (-2,4)-/        |
  \          |
  \        (1,2)--(2,2)\
  \                     \
  \                      \
  (0,0)------------------(4,0)
*/
#if 0

#include "timeprofile.h"

int main(int argc, char *argv[])
{
    timeprofile_t *tp = timeprofile_create();

    zarray_t *polya = g2d_polygon_create_data((float[][2]) {
            { 0, 0},
            { 4, 0},
            { 2, 2},
            { 1, 2},
            { 1, 5},
            { -2,4} }, 6);

    zarray_t *polyb = g2d_polygon_create_data((float[][2]) {
            { .1, .1},
            { .5, .1},
            { .1, .5 } }, 3);

    zarray_t *polyc = g2d_polygon_create_data((float[][2]) {
            { 3, 0},
            { 5, 0},
            { 5, 1} }, 3);

    zarray_t *polyd = g2d_polygon_create_data((float[][2]) {
            { 5, 5},
            { 6, 6},
            { 5, 6} }, 3);

/*
  5      L---K
  4      |I--J
  3      |H-G
  2      |E-F
  1      |D--C
  0      A---B
  01234
*/
    zarray_t *polyE = g2d_polygon_create_data((float[][2]) {
            {0,0}, {4,0}, {4, 1}, {1,1},
                                  {1,2}, {3,2}, {3,3}, {1,3},
                                                       {1,4}, {4,4}, {4,5}, {0,5}}, 12);

    srand(0);

    timeprofile_stamp(tp, "begin");

    if (1) {
        int niters = 100000;

        for (int i = 0; i < niters; i++) {
            float q[2];
            q[0] = 10.0f * random() / RAND_MAX - 2;
            q[1] = 10.0f * random() / RAND_MAX - 2;

            g2d_polygon_contains_point(polyE, q);
        }

        timeprofile_stamp(tp, "fast");

        for (int i = 0; i < niters; i++) {
            float q[2];
            q[0] = 10.0f * random() / RAND_MAX - 2;
            q[1] = 10.0f * random() / RAND_MAX - 2;

            g2d_polygon_contains_point_ref(polyE, q);
        }

        timeprofile_stamp(tp, "slow");

        for (int i = 0; i < niters; i++) {
            float q[2];
            q[0] = 10.0f * random() / RAND_MAX - 2;
            q[1] = 10.0f * random() / RAND_MAX - 2;

            int v0 = g2d_polygon_contains_point(polyE, q);
            int v1 = g2d_polygon_contains_point_ref(polyE, q);
            assert(v0 == v1);
        }

        timeprofile_stamp(tp, "both");
        timeprofile_display(tp);
    }

    if (1) {
        zarray_t *poly = polyE;

        float res = 0.399;
        for (float y = 5.2; y >= -.5; y -= res) {
            float xs[zarray_size(poly)];

            int xsz = g2d_polygon_rasterize(poly, y, xs);
            int xpos = 0;
            int inout = 0; // start off "out"
            for (float x = -3; x < 6; x += res) {
                while (x > xs[xpos] && xpos < xsz) {
                    xpos++;
                    inout ^= 1;
                }

                if (inout)
                    printf("y");
                else
                    printf(" ");
            }
            printf("\n");

            for (float x = -3; x < 6; x += res) {
                float q[2] = {x, y};
                if (g2d_polygon_contains_point(poly, q))
                    printf("X");
                else
                    printf(" ");
            }
            printf("\n");
        }
    }



/*
// CW order
float p[][2] =  { { 0, 0},
{ -2, 4},
{1, 5},
{1, 2},
{2, 2},
{4, 0} };
*/

     float q[2] = { 10, 10 };
     printf("0==%d\n", g2d_polygon_contains_point(polya, q));

     q[0] = 1; q[1] = 1;
     printf("1==%d\n", g2d_polygon_contains_point(polya, q));

     q[0] = 3; q[1] = .5;
     printf("1==%d\n", g2d_polygon_contains_point(polya, q));

     q[0] = 1.2; q[1] = 2.1;
     printf("0==%d\n", g2d_polygon_contains_point(polya, q));

     printf("0==%d\n", g2d_polygon_contains_polygon(polya, polyb));

     printf("0==%d\n", g2d_polygon_contains_polygon(polya, polyc));

     printf("0==%d\n", g2d_polygon_contains_polygon(polya, polyd));

     ////////////////////////////////////////////////////////
     // Test convex hull
     if (1) {
         zarray_t *hull = g2d_convex_hull(polyE);

         for (int k = 0; k < zarray_size(hull); k++) {
             float *h;
             zarray_get_volatile(hull, k, &h);

             printf("%15f, %15f\n", h[0], h[1]);
         }
     }

     for (int i = 0; i < 100000; i++) {
         zarray_t *points = zarray_create(sizeof(float[2]));

         for (int j = 0; j < 100; j++) {
             float q[2];
             q[0] = 10.0f * random() / RAND_MAX - 2;
             q[1] = 10.0f * random() / RAND_MAX - 2;

             zarray_add(points, q);
         }

         zarray_t *hull = g2d_convex_hull(points);
         for (int j = 0; j < zarray_size(points); j++) {
             float *q;
             zarray_get_volatile(points, j, &q);

             int on_edge;

             float p[2];
             g2d_polygon_closest_boundary_point(hull, q, p);
             if (g2d_distance(q, p) < .00001)
                 on_edge = 1;

             assert(on_edge || g2d_polygon_contains_point(hull, q));
         }

         zarray_destroy(hull);
         zarray_destroy(points);
     }
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "image_types.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

// to support conversions between different types, we define all image
// types at once. Type-specific implementations can then #include this
// file, assured that the basic types of each image are known.

typedef struct image_u8 image_u8_t;
struct image_u8
{
    int32_t width;
    int32_t height;
    int32_t stride;

    uint8_t *buf;
};

typedef struct image_u8x3 image_u8x3_t;
struct image_u8x3
{
    const int32_t width;
    const int32_t height;
    const int32_t stride; // bytes per line

    uint8_t *buf;
};

typedef struct image_u8x4 image_u8x4_t;
struct image_u8x4
{
    const int32_t width;
    const int32_t height;
    const int32_t stride; // bytes per line

    uint8_t *buf;
};

typedef struct image_f32 image_f32_t;
struct image_f32
{
    const int32_t width;
    const int32_t height;
    const int32_t stride; // floats per line

    float *buf; // indexed as buf[y*stride + x]
};

typedef struct image_u32 image_u32_t;
struct image_u32
{
    const int32_t width;
    const int32_t height;
    const int32_t stride; // int32_ts per line

    uint32_t *buf; // indexed as buf[y*stride + x]
};

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "apriltag_math.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

// Computes the cholesky factorization of A, putting the lower
// triangular matrix into R.
static inline void mat33_chol(const float *A,
                              float *R)
{
    // A[0] = R[0]*R[0]
    R[0] = sqrt(A[0]);

    // A[1] = R[0]*R[3];
    R[3] = A[1] / R[0];

    // A[2] = R[0]*R[6];
    R[6] = A[2] / R[0];

    // A[4] = R[3]*R[3] + R[4]*R[4]
    R[4] = sqrt(A[4] - R[3]*R[3]);

    // A[5] = R[3]*R[6] + R[4]*R[7]
    R[7] = (A[5] - R[3]*R[6]) / R[4];

    // A[8] = R[6]*R[6] + R[7]*R[7] + R[8]*R[8]
    R[8] = sqrt(A[8] - R[6]*R[6] - R[7]*R[7]);

    R[1] = 0;
    R[2] = 0;
    R[5] = 0;
}

static inline void mat33_lower_tri_inv(const float *A,
                                       float *R)
{
    // A[0]*R[0] = 1
    R[0] = 1 / A[0];

    // A[3]*R[0] + A[4]*R[3] = 0
    R[3] = -A[3]*R[0] / A[4];

    // A[4]*R[4] = 1
    R[4] = 1 / A[4];

    // A[6]*R[0] + A[7]*R[3] + A[8]*R[6] = 0
    R[6] = (-A[6]*R[0] - A[7]*R[3]) / A[8];

    // A[7]*R[4] + A[8]*R[7] = 0
    R[7] = -A[7]*R[4] / A[8];

    // A[8]*R[8] = 1
    R[8] = 1 / A[8];
}


static inline void mat33_sym_solve(const float *A,
                                   const float *B,
                                   float *R)
{
    float L[9];
    mat33_chol(A, L);

    float M[9];
    mat33_lower_tri_inv(L, M);

    float tmp[3];
    tmp[0] = M[0]*B[0];
    tmp[1] = M[3]*B[0] + M[4]*B[1];
    tmp[2] = M[6]*B[0] + M[7]*B[1] + M[8]*B[2];

    R[0] = M[0]*tmp[0] + M[3]*tmp[1] + M[6]*tmp[2];
    R[1] = M[4]*tmp[1] + M[7]*tmp[2];
    R[2] = M[8]*tmp[2];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "apriltag.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

struct quad
{
    float p[4][2]; // corners

    bool reversed_border;

    // H: tag coordinates ([-1,1] at the black corners) to pixels
    // Hinv: pixels to tag
    matd_t *H, *Hinv;
};

// Represents a tag family. Every tag belongs to a tag family. Tag
// families are generated by the Java tool
// april.tag.TagFamilyGenerator and can be converted to C using
// april.tag.TagToC.
typedef struct apriltag_family apriltag_family_t;
struct apriltag_family
{
    // How many codes are there in this tag family?
    uint32_t ncodes;

    int32_t width_at_border;
    int32_t total_width;
    int32_t d;
    bool reversed_border;

    // The bit locations.
    uint32_t nbits;
    const uint32_t *bit_x;
    const uint32_t *bit_y;

    // minimum hamming distance between any two codes. (e.g. 36h11 => 11)
    uint32_t h;

    // some detector implementations may preprocess codes in order to
    // accelerate decoding.  They put their data here. (Do not use the
    // same apriltag_family instance in more than one implementation)
    void *impl;

    // The codes in the family.
    const uint64_t codes[];
};


struct apriltag_quad_thresh_params
{
    // reject quads containing too few pixels
    int min_cluster_pixels;

    // how many corner candidates to consider when segmenting a group
    // of pixels into a quad.
    int max_nmaxima;

    // Reject quads where pairs of edges have angles that are close to
    // straight or close to 180 degrees. Zero means that no quads are
    // rejected. (In radians).
    float critical_rad;
    float cos_critical_rad;

    // When fitting lines to the contours, what is the maximum mean
    // squared error allowed?  This is useful in rejecting contours
    // that are far from being quad shaped; rejecting these quads "early"
    // saves expensive decoding processing.
    float max_line_fit_mse;

    // When we build our model of black & white pixels, we add an
    // extra check that the white model must be (overall) brighter
    // than the black model.  How much brighter? (in pixel values,
    // [0,255]). .
    int min_white_black_diff;

    // should the thresholded image be deglitched? Only useful for
    // very noisy images
    int deglitch;
};

// Represents a detector object. Upon creating a detector, all fields
// are set to reasonable values, but can be overridden by accessing
// these fields.
typedef struct apriltag_detector apriltag_detector_t;
struct apriltag_detector
{
    ///////////////////////////////////////////////////////////////
    // User-configurable parameters.

    // detection of quads can be done on a lower-resolution image,
    // improving speed at a cost of pose accuracy and a slight
    // decrease in detection rate. Decoding the binary payload is
    // still done at full resolution. .
    float quad_decimate;

    // What Gaussian blur should be applied to the segmented image
    // (used for quad detection?)  Parameter is the standard deviation
    // in pixels.  Very noisy images benefit from non-zero values
    // (e.g. 0.8).
    float quad_sigma;

    // When non-zero, the edges of the each quad are adjusted to "snap
    // to" strong gradients nearby. This is useful when decimation is
    // employed, as it can increase the quality of the initial quad
    // estimate substantially. Generally recommended to be on (1).
    //
    // Very computationally inexpensive. Option is ignored if
    // quad_decimate = 1.
    int refine_edges;

    // How much sharpening should be done to decoded images? This
    // can help decode small tags but may or may not help in odd
    // lighting conditions or low light conditions.
    //
    // The default value is 0.25.
    double decode_sharpening;

    struct apriltag_quad_thresh_params qtp;

    ///////////////////////////////////////////////////////////////

    uint32_t nedges;
    uint32_t nsegments;
    uint32_t nquads;

    ///////////////////////////////////////////////////////////////
    // Internal variables below

    // Not freed on apriltag_destroy; a tag family can be shared
    // between multiple users. The user should ultimately destroy the
    // tag family passed into the constructor.
    zarray_t *tag_families;
};

// Represents the detection of a tag. These are returned to the user
// and must be individually destroyed by the user.
typedef struct apriltag_detection apriltag_detection_t;
struct apriltag_detection
{
    // a pointer for convenience. not freed by apriltag_detection_destroy.
    apriltag_family_t *family;

    // The decoded ID of the tag
    int id;

    // How many error bits were corrected? Note: accepting large numbers of
    // corrected errors leads to greatly increased false positive rates.
    // NOTE: As of this implementation, the detector cannot detect tags with
    // a hamming distance greater than 2.
    int hamming;

    // A measure of the quality of the binary decoding process: the
    // average difference between the intensity of a data bit versus
    // the decision threshold. Higher numbers roughly indicate better
    // decodes. This is a reasonable measure of detection accuracy
    // only for very small tags-- not effective for larger tags (where
    // we could have sampled anywhere within a bit cell and still
    // gotten a good detection.)
    float decision_margin;

    // The 3x3 homography matrix describing the projection from an
    // "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
    // -1)) to pixels in the image. This matrix will be freed by
    // apriltag_detection_destroy.
    matd_t *H;

    // The center of the detection in image pixel coordinates.
    double c[2];

    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    double p[4][2];
};

// don't forget to add a family!
apriltag_detector_t *apriltag_detector_create();

// add a family to the apriltag detector. caller still "owns" the family.
// a single instance should only be provided to one apriltag detector instance.
void apriltag_detector_add_family_bits(apriltag_detector_t *td, apriltag_family_t *fam, int bits_corrected);

// Tunable, but really, 2 is a good choice. Values of >=3
// consume prohibitively large amounts of memory, and otherwise
// you want the largest value possible.
static inline void apriltag_detector_add_family(apriltag_detector_t *td, apriltag_family_t *fam)
{
    apriltag_detector_add_family_bits(td, fam, 2);
}

// does not deallocate the family.
void apriltag_detector_remove_family(apriltag_detector_t *td, apriltag_family_t *fam);

// unregister all families, but does not deallocate the underlying tag family objects.
void apriltag_detector_clear_families(apriltag_detector_t *td);

// Destroy the april tag detector (but not the underlying
// apriltag_family_t used to initialize it.)
void apriltag_detector_destroy(apriltag_detector_t *td);

// Detect tags from an image and return an array of
// apriltag_detection_t*. You can use apriltag_detections_destroy to
// free the array and the detections it contains, or call
// _detection_destroy and zarray_destroy yourself.
zarray_t *apriltag_detector_detect(apriltag_detector_t *td, image_u8_t *im_orig);

// Call this method on each of the tags returned by apriltag_detector_detect
void apriltag_detection_destroy(apriltag_detection_t *det);

// destroys the array AND the detections within it.
void apriltag_detections_destroy(zarray_t *detections);

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "tag16h5"
////////////////////////////////////////////////////////////////////////////////////////////////////

#define N_BITS 16

static const uint32_t tag16h5_bit_x[N_BITS] = {1, 2, 3, 2, 4, 4, 4, 3, 4, 3, 2, 3, 1, 1, 1, 2};
static const uint32_t tag16h5_bit_y[N_BITS] = {1, 1, 1, 2, 1, 2, 3, 2, 4, 4, 4, 3, 4, 3, 2, 3};

const apriltag_family_t tag16h5 = {
    .ncodes = 30,
    .width_at_border = 6,
    .total_width = 8,
    .d = 4,
    .reversed_border = false,
    .nbits = N_BITS,
    .bit_x = tag16h5_bit_x,
    .bit_y = tag16h5_bit_y,
    .h = 5,
    .codes = {
       0x00000000000027c8UL,
       0x00000000000031b6UL,
       0x0000000000003859UL,
       0x000000000000569cUL,
       0x0000000000006c76UL,
       0x0000000000007ddbUL,
       0x000000000000af09UL,
       0x000000000000f5a1UL,
       0x000000000000fb8bUL,
       0x0000000000001cb9UL,
       0x00000000000028caUL,
       0x000000000000e8dcUL,
       0x0000000000001426UL,
       0x0000000000005770UL,
       0x0000000000009253UL,
       0x000000000000b702UL,
       0x000000000000063aUL,
       0x0000000000008f34UL,
       0x000000000000b4c0UL,
       0x00000000000051ecUL,
       0x000000000000e6f0UL,
       0x0000000000005fa4UL,
       0x000000000000dd43UL,
       0x0000000000001aaaUL,
       0x000000000000e62fUL,
       0x0000000000006dbcUL,
       0x000000000000b6ebUL,
       0x000000000000de10UL,
       0x000000000000154dUL,
       0x000000000000b57aUL,
    }
};

#undef N_BITS

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "tag25h7"
////////////////////////////////////////////////////////////////////////////////////////////////////

#define N_BITS 25

static const uint32_t tag25h7_bit_x[N_BITS] = {1, 2, 3, 4, 2, 3, 5, 5, 5, 5, 4, 4, 5, 4, 3, 2, 4, 3, 1, 1, 1, 1, 2, 2, 3};
static const uint32_t tag25h7_bit_y[N_BITS] = {1, 1, 1, 1, 2, 2, 1, 2, 3, 4, 2, 3, 5, 5, 5, 5, 4, 4, 5, 4, 3, 2, 4, 3, 3};

iltag_family_t tag25h7 = {
    .ncodes = 279,
    .width_at_border = 7,
    .total_width = 9,
    .d = 5,
    .reversed_border = false,
    .nbits = N_BITS,
    .bit_x = tag25h7_bit_x,
    .bit_y = tag25h7_bit_y,
    .h = 7,
    .codes = {
        0x0000000000d0f164UL,
        0x00000000015ff729UL,
        0x00000000007e02b3UL,
        0x00000000010d0878UL,
        0x00000000019c0e3dUL,
        0x00000000002b1402UL,
        0x0000000000ba19c7UL,
        0x0000000001491f8cUL,
        0x0000000001d82551UL,
        0x0000000000143c65UL,
        0x0000000000a3422aUL,
        0x0000000001c14db4UL,
        0x0000000000505379UL,
        0x00000000016e5f03UL,
        0x0000000001fd64c8UL,
        0x00000000011b7052UL,
        0x0000000001aa7617UL,
        0x0000000001e68d2bUL,
        0x00000000007592f0UL,
        0x00000000010498b5UL,
        0x0000000001939e7aUL,
        0x000000000140afc9UL,
        0x0000000001cfb58eUL,
        0x0000000000edc118UL,
        0x00000000000bcca2UL,
        0x000000000129d82cUL,
        0x0000000001f4f505UL,
        0x000000000083facaUL,
        0x0000000001a20654UL,
        0x0000000000310c19UL,
        0x0000000000c011deUL,
        0x00000000006d232dUL,
        0x0000000000fc28f2UL,
        0x0000000000564b90UL,
        0x0000000001ec8542UL,
        0x00000000007b8b07UL,
        0x0000000000b7a21bUL,
        0x0000000001d5ada5UL,
        0x000000000064b36aUL,
        0x0000000000f3b92fUL,
        0x000000000182bef4UL,
        0x0000000000a0ca7eUL,
        0x0000000001bed608UL,
        0x00000000004ddbcdUL,
        0x0000000000dce192UL,
        0x00000000013e381dUL,
        0x0000000000985abbUL,
        0x00000000019f8ea8UL,
        0x00000000002e946dUL,
        0x000000000188b70bUL,
        0x0000000001ea0d96UL,
        0x00000000002624aaUL,
        0x0000000001d335f9UL,
        0x0000000000623bbeUL,
        0x00000000009e52d2UL,
        0x00000000012d5897UL,
        0x000000000070a398UL,
        0x0000000001cac636UL,
        0x000000000124e8d4UL,
        0x000000000042f45eUL,
        0x0000000001589025UL,
        0x00000000009bdb26UL,
        0x0000000000c11a9dUL,
        0x0000000001df2627UL,
        0x0000000000a1d33dUL,
        0x0000000001e51e3eUL,
        0x00000000005d4c66UL,
        0x0000000000d57a8eUL,
        0x0000000000828bddUL,
        0x0000000001bd671bUL,
        0x000000000153a0cdUL,
        0x0000000001e2a692UL,
        0x0000000000164de3UL,
        0x0000000000efd296UL,
        0x00000000017ed85bUL,
        0x000000000115120dUL,
        0x00000000011b0a24UL,
        0x0000000001b877c3UL,
        0x0000000001658912UL,
        0x00000000006cbcffUL,
        0x000000000091fc76UL,
        0x0000000000a65c67UL,
        0x0000000001717940UL,
        0x000000000132ea80UL,
        0x00000000008d0d1eUL,
        0x0000000000b24c95UL,
        0x00000000001ad70dUL,
        0x0000000000a16d0fUL,
        0x0000000001a03139UL,
        0x0000000001dc484dUL,
        0x0000000001366aebUL,
        0x00000000011f934eUL,
        0x000000000079b5ecUL,
        0x0000000001aab27aUL,
        0x000000000182fb57UL,
        0x00000000019fcb0bUL,
        0x00000000003ac0feUL,
        0x00000000001309dbUL,
        0x0000000000bedf54UL,
        0x00000000000e1a83UL,
        0x000000000005aac0UL,
        0x0000000000a3185fUL,
        0x000000000047b9ebUL,
        0x0000000001ddf39dUL,
        0x00000000018b04ecUL,
        0x0000000000b77850UL,
        0x0000000001cd1417UL,
        0x00000000009bb15bUL,
        0x0000000000d7c86fUL,
        0x00000000013c9f65UL,
        0x0000000001e874deUL,
        0x00000000009cba1aUL,
        0x000000000167d6f3UL,
        0x00000000011e60c4UL,
        0x000000000168dfb2UL,
        0x0000000001ef75b4UL,
        0x0000000001bf4eceUL,
        0x00000000006c2d06UL,
        0x00000000017360f3UL,
        0x0000000001f5079dUL,
        0x00000000008a0579UL,
        0x0000000000c3a4e1UL,
        0x0000000001df38bfUL,
        0x0000000001fd1132UL,
        0x00000000014ec40dUL,
        0x0000000000c44772UL,
        0x00000000007f5a6dUL,
        0x0000000001d10d48UL,
        0x000000000004b499UL,
        0x00000000007472feUL,
        0x0000000001b15fbaUL,
        0x000000000136b9e6UL,
        0x00000000019429d8UL,
        0x0000000001013d7aUL,
        0x0000000001ae1bb2UL,
        0x000000000177fcb5UL,
        0x0000000001612518UL,
        0x00000000011a2695UL,
        0x0000000000275299UL,
        0x0000000000e9ffafUL,
        0x000000000072da46UL,
        0x0000000001f0cd6eUL,
        0x000000000130d467UL,
        0x00000000014b2c6fUL,
        0x0000000000ca8e84UL,
        0x0000000001512486UL,
        0x0000000001e02a4bUL,
        0x00000000018ac3eeUL,
        0x0000000001463d17UL,
        0x0000000001592e1bUL,
        0x0000000000a482b1UL,
        0x000000000002c84bUL,
        0x0000000001029534UL,
        0x000000000074982eUL,
        0x0000000001a31d10UL,
        0x00000000015d274cUL,
        0x0000000000127547UL,
        0x00000000001dff47UL,
        0x00000000001abb3fUL,
        0x0000000000987b50UL,
        0x0000000000c42ba4UL,
        0x0000000001cd0ae1UL,
        0x000000000150f620UL,
        0x000000000124f236UL,
        0x00000000007de24aUL,
        0x0000000000e530ecUL,
        0x00000000018f6dadUL,
        0x0000000000de42aeUL,
        0x0000000000af2487UL,
        0x00000000016c15e9UL,
        0x0000000000e72b37UL,
        0x0000000001c25b3aUL,
        0x0000000000234b97UL,
        0x0000000000707e94UL,
        0x0000000000631f79UL,
        0x00000000011c2a42UL,
        0x00000000009a9630UL,
        0x000000000178e070UL,
        0x00000000015e5e9dUL,
        0x0000000000b222f6UL,
        0x000000000169e89dUL,
        0x00000000009cf6a9UL,
        0x0000000001abc962UL,
        0x00000000004bc145UL,
        0x000000000007dcffUL,
        0x000000000109a433UL,
        0x000000000027afbdUL,
        0x00000000013b103bUL,
        0x000000000190c593UL,
        0x0000000000097f19UL,
        0x00000000011bd6d8UL,
        0x00000000004a981dUL,
        0x00000000014cb2e7UL,
        0x00000000014f1360UL,
        0x0000000000f36156UL,
        0x0000000001fa5445UL,
        0x0000000001f74ca0UL,
        0x0000000001763154UL,
        0x00000000000549b1UL,
        0x0000000001f3d0fcUL,
        0x0000000001df59d8UL,
        0x0000000001e117c0UL,
        0x00000000017d4989UL,
        0x0000000000ed7768UL,
        0x00000000006cd047UL,
        0x0000000000873ae7UL,
        0x00000000006308f6UL,
        0x000000000068e9daUL,
        0x0000000001948c47UL,
        0x0000000001102cf3UL,
        0x0000000000dbcbdeUL,
        0x0000000001517dbfUL,
        0x000000000133897eUL,
        0x000000000088cac1UL,
        0x00000000001fa269UL,
        0x0000000001081a54UL,
        0x00000000011bade9UL,
        0x0000000000df06dcUL,
        0x0000000000a0781cUL,
        0x00000000003c8e9bUL,
        0x00000000011f25a2UL,
        0x00000000018d87b2UL,
        0x00000000009969f9UL,
        0x000000000011738bUL,
        0x0000000000380abcUL,
        0x000000000077e365UL,
        0x0000000001bc2defUL,
        0x00000000003e7bf1UL,
        0x00000000005d131bUL,
        0x0000000000fa9352UL,
        0x0000000001eae1d1UL,
        0x0000000000e314b5UL,
        0x000000000113c2b6UL,
        0x0000000000e03c52UL,
        0x0000000001ca23daUL,
        0x00000000000748ccUL,
        0x00000000001b4756UL,
        0x0000000000e6e657UL,
        0x00000000015cebceUL,
        0x0000000000e0a8a7UL,
        0x0000000000b009abUL,
        0x0000000001b78cbfUL,
        0x0000000000f732d5UL,
        0x0000000001170ef5UL,
        0x0000000001bb539fUL,
        0x0000000001e6007fUL,
        0x0000000001f5d2abUL,
        0x00000000007ee520UL,
        0x000000000010f2ccUL,
        0x0000000000557508UL,
        0x0000000001eb9c4eUL,
        0x000000000015d99eUL,
        0x00000000017a7da9UL,
        0x0000000000f60df9UL,
        0x0000000001e61218UL,
        0x0000000000e9073bUL,
        0x0000000001700ccdUL,
        0x0000000000558b3cUL,
        0x00000000006c687cUL,
        0x0000000001062826UL,
        0x0000000000cd75dbUL,
        0x000000000023963eUL,
        0x0000000000024f51UL,
        0x0000000001869605UL,
        0x00000000004d47beUL,
        0x0000000000ae2114UL,
        0x0000000000c4c89cUL,
        0x0000000000ee558aUL,
        0x0000000001cc7240UL,
        0x00000000010c9c0aUL,
        0x000000000005274bUL,
        0x0000000000eab6cfUL,
        0x00000000002a0e77UL,
        0x0000000000c0630eUL,
        0x00000000016d8f6eUL,
        0x0000000000d3e2bdUL,
        0x0000000001c4d128UL,
        0x0000000001cfac90UL,
        0x0000000001206b66UL,
    };
};


////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "tag25h9"
////////////////////////////////////////////////////////////////////////////////////////////////////

#define N_BITS 25

static const uint32_t tag25h9_bit_x[N_BITS] = {1, 2, 3, 4, 2, 3, 5, 5, 5, 5, 4, 4, 5, 4, 3, 2, 4, 3, 1, 1, 1, 1, 2, 2, 3};
static const uint32_t tag25h9_bit_y[N_BITS] = {1, 1, 1, 1, 2, 2, 1, 2, 3, 4, 2, 3, 5, 5, 5, 5, 4, 4, 5, 4, 3, 2, 4, 3, 3};

const apriltag_family_t tag16h5 = {
    .ncodes = 35,
    .width_at_border = 7,
    .total_width = 9,
    .d = 5,
    .reversed_border = false,
    .nbits = N_BITS,
    .bit_x = tag25h9_bit_x,
    .bit_y = tag25h9_bit_y,
    .h = 9,
    .codes = {
        0x000000000156f1f4UL,
        0x0000000001f28cd5UL,
        0x00000000016ce32cUL,
        0x0000000001ea379cUL,
        0x0000000001390f89UL,
        0x000000000034fad0UL,
        0x00000000007dcdb5UL,
        0x000000000119ba95UL,
        0x0000000001ae9daaUL,
        0x0000000000df02aaUL,
        0x000000000082fc15UL,
        0x0000000000465123UL,
        0x0000000000ceee98UL,
        0x0000000001f17260UL,
        0x00000000014429cdUL,
        0x00000000017248a8UL,
        0x00000000016ad452UL,
        0x00000000009670adUL,
        0x00000000016f65b2UL,
        0x0000000000b8322bUL,
        0x00000000005d715bUL,
        0x0000000001a1c7e7UL,
        0x0000000000d7890dUL,
        0x0000000001813522UL,
        0x0000000001c9c611UL,
        0x000000000099e4a4UL,
        0x0000000000855234UL,
        0x00000000017b81c0UL,
        0x0000000000c294bbUL,
        0x000000000089fae3UL,
        0x000000000044df5fUL,
        0x0000000001360159UL,
        0x0000000000ec31e8UL,
        0x0000000001bcc0f6UL,
        0x0000000000a64f8dUL,
    }
};

#undef N_BITS

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "tag36h10"
////////////////////////////////////////////////////////////////////////////////////////////////////

#define N_BITS 36

static const uint32_t tag36h10_bit_x[N_BITS] = {1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3};
static const uint32_t tag36h10_bit_y[N_BITS] = {1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4};

const apriltag_family_t tag36h10 = {
    .ncodes = 2320,
    .width_at_border = 8,
    .total_width = 10,
    .d = 6,
    .reversed_border = false,
    .nbits = N_BITS,
    .bit_x = tag36h10_bit_x,
    .bit_y = tag36h10_bit_y,
    .h = 10,
    .codes = {
        0x00000001a42f9469UL,
        0x000000021a48c08dUL,
        0x000000026dfdbc5dUL,
        0x00000002d19b78fbUL,
        0x000000031c5557dfUL,
        0x00000003f2b3d349UL,
        0x00000003e6d4a7a5UL,
        0x000000043d6dfb6dUL,
        0x00000005688d2affUL,
        0x000000054663b474UL,
        0x000000067fd759e4UL,
        0x000000073b196d82UL,
        0x00000007de71c630UL,
        0x00000008492722d4UL,
        0x0000000835407afaUL,
        0x0000000893a6fe16UL,
        0x000000095668d348UL,
        0x00000009c20e37a4UL,
        0x0000000a19bb676cUL,
        0x0000000ad1355c1bUL,
        0x0000000b2ab4ac53UL,
        0x0000000b14f27095UL,
        0x0000000bef6b014dUL,
        0x0000000c9ecbadc7UL,
        0x0000000e1996a6bdUL,
        0x0000000edc588fcbUL,
        0x0000000f26d9d7a1UL,
        0x0000000035e79c1aUL,
        0x00000001644f74a8UL,
        0x00000001bfda85e0UL,
        0x000000020573db2aUL,
        0x000000027614bfd6UL,
        0x00000002daf26b72UL,
        0x0000000332930a98UL,
        0x00000003176c5674UL,
        0x00000003892ad2d0UL,
        0x00000003fd4d26acUL,
        0x00000004b88a1fcaUL,
        0x0000000555b6b579UL,
        0x00000006103880d7UL,
        0x00000006045e58fbUL,
        0x00000006dedf28a3UL,
        0x000000072766f249UL,
        0x00000007e4a0c77fUL,
        0x0000000973ba8e61UL,
        0x0000000965fd5285UL,
        0x00000009d99bd6a9UL,
        0x0000000a221264e6UL,
        0x0000000a1455fccaUL,
        0x0000000a7cb3193eUL,
        0x0000000aead44d92UL,
        0x0000000ad50ac954UL,
        0x0000000b2f0bb19cUL,
        0x0000000cba043aa2UL,
        0x0000000d73d7f6b4UL,
        0x0000000d6d189310UL,
        0x0000000f41278885UL,
        0x00000001c45306afUL,
        0x0000000222b4420bUL,
        0x00000002d70ceeb1UL,
        0x0000000392cb439fUL,
        0x000000045d1179eeUL,
        0x000000057a79d05cUL,
        0x000000062bc91dd6UL,
        0x000000073ca773c0UL,
        0x00000008b3da6eb0UL,
        0x00000009c5a521c7UL,
        0x0000000a812a1881UL,
        0x0000000adeab4859UL,
        0x0000000b3402b213UL,
        0x0000000b9171667fUL,
        0x0000000f5ac6918cUL,
        0x00000000549b1cfcUL,
        0x00000000b2744a58UL,
        0x00000000ae12ceb4UL,
        0x00000002f8aaab94UL,
        0x000000038db1c43bUL,
        0x00000004be190989UL,
        0x000000051bef9565UL,
        0x0000000505a8d1c3UL,
        0x0000000575ce25bfUL,
        0x000000068ac14eb1UL,
        0x00000006fd1bca45UL,
        0x0000000854752994UL,
        0x00000009e96b442aUL,
        0x0000000afb308f74UL,
        0x0000000cd9c8ca44UL,
        0x0000000d23c9b28eUL,
        0x0000000d07277423UL,
        0x0000000e8b3a6959UL,
        0x0000000f44bc546fUL,
        0x000000010e4aa7f3UL,
        0x000000029971cc68UL,
        0x000000062d7b0b94UL,
        0x00000008c22aac77UL,
        0x00000008b46d70d3UL,
        0x0000000a43373b0dUL,
        0x0000000e2efd2c74UL,
        0x0000000f5f4587e6UL,
        0x000000001b82bfa0UL,
        0x00000000642bef7aUL,
        0x00000001e576f881UL,
        0x00000002a290cda7UL,
        0x00000003b1ae80b1UL,
        0x000000053b9317e3UL,
        0x00000005833ac709UL,
        0x00000005f25d3ff5UL,
        0x0000000648fcef3dUL,
        0x000000068d6350d6UL,
        0x00000008064a4da6UL,
        0x000000092b76e736UL,
        0x00000009157102b0UL,
        0x00000009eef0de60UL,
        0x0000000b098e9306UL,
        0x0000000b63e94718UL,
        0x0000000c1241e99bUL,
        0x0000000eab42b39fUL,
        0x0000000fca2a422dUL,
        0x0000000007f86a4bUL,
        0x0000000124962550UL,
        0x00000001fe977988UL,
        0x0000000300a8b4c4UL,
        0x000000076c2b21a1UL,
        0x0000000932f76619UL,
        0x00000009994e9b51UL,
        0x0000000a634fcb8bUL,
        0x0000000b6d92c074UL,
        0x0000000bcbf53858UL,
        0x0000000c0c3a0d7eUL,
        0x000000001c31a9c3UL,
        0x00000000d5a694d5UL,
        0x000000012f2fc41dUL,
        0x00000005a342598aUL,
        0x00000006466af138UL,
        0x000000081eb68f22UL,
        0x000000098e885eb6UL,
        0x0000000ab594f405UL,
        0x0000000b19f201a1UL,
        0x0000000d44832643UL,
        0x0000000fdc915802UL,
        0x00000000ee4f05dcUL,
        0x00000003043aa2a6UL,
        0x00000003765d7a9aUL,
        0x00000005468500abUL,
        0x000000059d38dc6bUL,
        0x000000060b5e288fUL,
        0x00000009539ba6dfUL,
        0x00000009a93219d2UL,
        0x0000000ad81ab160UL,
        0x0000000abe7d650cUL,
        0x0000000beb8dd89eUL,
        0x0000000bd5432e10UL,
        0x000000009aafb509UL,
        0x000000014f535a9bUL,
        0x0000000202956e9dUL,
        0x00000003d64508f8UL,
        0x00000005babe994eUL,
        0x00000006d1c1e752UL,
        0x000000094cb57d7bUL,
        0x00000009a45c8c11UL,
        0x0000000a7f4dd4c9UL,
        0x0000000d6ab9426bUL,
        0x0000000da6766828UL,
        0x0000000f1608f9bcUL,
        0x000000016ad49fb6UL,
        0x00000006da68e61fUL,
        0x00000006ce0e03bbUL,
        0x0000000a0404d752UL,
        0x0000000e066e102fUL,
        0x00000002f9d00546UL,
        0x000000039878a9e4UL,
        0x000000049521229cUL,
        0x00000008a70b85a9UL,
        0x000000096ecdad9dUL,
        0x00000009b5300ed7UL,
        0x0000000d08e19cd0UL,
        0x0000000e1dbbf0a6UL,
        0x0000000ef012435cUL,
        0x00000006ecd2fc10UL,
        0x0000000749a519f4UL,
        0x000000091b715e4cUL,
        0x000000090f17dae8UL,
        0x0000000975d0ae16UL,
        0x00000009ced1f6deUL,
        0x0000000ba3660c0bUL,
        0x0000000d1b1c9d9dUL,
        0x0000000d77fa492bUL,
        0x0000000de0bdcdc7UL,
        0x0000000e2472e389UL,
        0x000000039ee6b70eUL,
        0x00000007a8ad48b3UL,
        0x0000000b4ee1682eUL,
        0x0000000bb8a7ec82UL,
        0x000000007cd0404dUL,
        0x000000011d68ecc7UL,
        0x000000046cbd039dUL,
        0x00000004b634dd1aUL,
        0x000000063c6d44e0UL,
        0x0000000981a82ea0UL,
        0x0000000a93f25d5fUL,
        0x0000000a8795d9fbUL,
        0x0000000b5d3c69a3UL,
        0x0000000f56912854UL,
        0x00000001f86096b4UL,
        0x00000001ee865210UL,
        0x000000024bf5c67cUL,
        0x000000037a5d0beeUL,
        0x00000003c2fcd384UL,
        0x00000005324bc851UL,
        0x00000005886a7099UL,
        0x0000000b72fa2c2cUL,
        0x0000000fc92eb5b5UL,
        0x0000000422e4a0c6UL,
        0x000000059b17b310UL,
        0x00000008e7d2e941UL,
        0x0000000943f50c2dUL,
        0x000000093db38889UL,
        0x0000000d3c56115cUL,
        0x0000000ec3097940UL,
        0x0000000f0c8e4c64UL,
        0x000000015a1a2b44UL,
        0x000000031885e859UL,
        0x00000003c77ec56fUL,
        0x0000000665489b09UL,
        0x0000000ac8831e70UL,
        0x0000000b0559325eUL,
        0x0000000d9d4e401bUL,
        0x00000001618bca4bUL,
        0x00000006cd54c849UL,
        0x0000000717d5b003UL,
        0x0000000acb7e0f3dUL,
        0x0000000db2c90514UL,
        0x0000000edb75ab84UL,
        0x00000002d2d82857UL,
        0x000000095fb613c6UL,
        0x00000009c1f187c8UL,
        0x0000000b1521d0f9UL,
        0x0000000f252b178dUL,
        0x00000003fe0f8640UL,
        0x00000005ca93c171UL,
        0x00000009dad88605UL,
        0x0000000b32a795d4UL,
        0x0000000d0273d2e4UL,
        0x0000000d7034b6d8UL,
        0x000000021d211a8bUL,
        0x000000032f7b66d5UL,
        0x000000097f46e061UL,
        0x0000000a3a80d947UL,
        0x0000000af24f7511UL,
        0x000000017760c682UL,
        0x00000002f17d53f8UL,
        0x00000005768d8553UL,
        0x00000005ccac399bUL,
        0x0000000b363443b6UL,
        0x000000086e868835UL,
        0x0000000d1f31c586UL,
        0x0000000e3c1f8c18UL,
        0x0000000dfb29b8f1UL,
        0x000000012545997cUL,
        0x00000003c9b87c9cUL,
        0x0000000650afcedaUL,
        0x00000007cc5d2d4dUL,
        0x0000000e97f2c6b4UL,
        0x0000000bcf094d17UL,
        0x0000000c2b6fd1b3UL,
        0x000000055b7539e1UL,
        0x00000006563c3099UL,
        0x00000006cbcaa475UL,
        0x0000000711ebf8bfUL,
        0x0000000beacf0872UL,
        0x0000000152bf7dd3UL,
        0x00000002d9e666a9UL,
        0x00000008aadf624cUL,
        0x0000000ae04840c3UL,
        0x0000000b89fceccbUL,
        0x0000000391534881UL,
        0x00000006d3b626f3UL,
        0x0000000b8cd3f384UL,
        0x000000056e622ea0UL,
        0x000000072da9effcUL,
        0x00000009d9dc3195UL,
        0x00000009b5bab5bbUL,
        0x0000000a2255635fUL,
        0x0000000b3d6b2a41UL,
        0x0000000e1e78a0caUL,
        0x00000006f166d850UL,
        0x00000006ed013c3cUL,
        0x0000000974124e7aUL,
        0x0000000b9c237103UL,
        0x000000012a55c948UL,
        0x00000006e6e02db3UL,
        0x0000000bb970b4aeUL,
        0x0000000cb5e92956UL,
        0x000000024a8a8519UL,
        0x00000005e4e6a504UL,
        0x0000000cce402a5bUL,
        0x0000000ead45ac6bUL,
        0x0000000b7e065329UL,
        0x00000002c9b38934UL,
        0x0000000550a5f272UL,
        0x00000009af15026bUL,
        0x0000000c43acbba9UL,
        0x000000003c4f2ed0UL,
        0x00000005a6df5a79UL,
        0x0000000acf8a733eUL,
        0x0000000e05408ed3UL,
        0x000000060787293bUL,
        0x000000089e95533fUL,
        0x0000000dcb673abcUL,
        0x0000000e0ea10fdaUL,
        0x0000000efa4f932eUL,
        0x000000045e988b2bUL,
        0x00000005d2c11c5eUL,
        0x0000000b84be5c13UL,
        0x000000050629a003UL,
        0x00000005e11c1dd3UL,
        0x000000012111629fUL,
        0x00000002365f3381UL,
        0x00000004cf5d41c0UL,
        0x00000008d957e5b5UL,
        0x000000025b88ce2cUL,
        0x000000041ec563d3UL,
        0x0000000c3884919fUL,
        0x00000004ffd91f28UL,
        0x0000000d6211be2eUL,
        0x00000002c6c33e2bUL,
        0x00000009b062517dUL,
        0x00000009a605d5d1UL,
        0x0000000c93615ffbUL,
        0x00000000587aa508UL,
        0x0000000ae741eba5UL,
        0x00000004b232ba75UL,
        0x00000009d68187b1UL,
        0x0000000a253c3b79UL,
        0x0000000bae55c0a6UL,
        0x0000000d0eaa3172UL,
        0x0000000c0c2dbd49UL,
        0x0000000d1d37d23fUL,
        0x0000000e952ecf4fUL,
        0x0000000a51239d5eUL,
        0x0000000f6e416099UL,
        0x0000000da4923cf8UL,
        0x000000055cd5bbbfUL,
        0x000000067fad4a0dUL,
        0x00000006f802d5abUL,
        0x00000004c90df6abUL,
        0x0000000cc0aad0e1UL,
        0x0000000745207f2bUL,
        0x0000000b516ad892UL,
        0x00000008da9ca7d8UL,
        0x0000000ba7517799UL,
        0x0000000b13811fcdUL,
        0x00000000328436caUL,
        0x00000005965ad68fUL,
        0x0000000e6b65823fUL,
        0x0000000951a1abf3UL,
        0x000000095cce7616UL,
        0x0000000ca003fe57UL,
        0x0000000d5ee5d32bUL,
        0x00000002d6d58f8eUL,
        0x0000000bee25c778UL,
        0x000000021cac7ed2UL,
        0x00000007b982415fUL,
        0x0000000237a92d3eUL,
        0x000000065c2a898aUL,
        0x00000002a67c67d0UL,
        0x0000000492c37445UL,
        0x0000000e66713585UL,
        0x0000000adb041611UL,
        0x00000009b5ed4c88UL,
        0x0000000f2f751a83UL,
        0x000000005a5db231UL,
        0x00000008478d5037UL,
        0x00000005e2954c13UL,
        0x00000000d8597d1dUL,
        0x0000000537116e24UL,
        0x00000007ce063461UL,
        0x0000000a79a20302UL,
        0x0000000b8016f46aUL,
        0x0000000ebeb44bdcUL,
        0x0000000206e7cff7UL,
        0x00000000d2870a69UL,
        0x00000004e21e723aUL,
        0x00000006a59590e7UL,
        0x0000000a522a2e9bUL,
        0x0000000c2ec8f44fUL,
        0x000000003e51273cUL,
        0x000000056bab7c9bUL,
        0x0000000b3a92fa7eUL,
        0x0000000f4a993d4bUL,
        0x00000008b80884b7UL,
        0x00000008a5339f30UL,
        0x0000000690c7389dUL,
        0x00000009f34cb283UL,
        0x0000000cd0febc20UL,
        0x0000000f579a2a82UL,
        0x00000003ba18373fUL,
        0x000000068fddad38UL,
        0x00000007ca529286UL,
        0x0000000df93cd25bUL,
        0x00000007d18313a9UL,
        0x0000000febd825c0UL,
        0x0000000898d72da2UL,
        0x0000000c2d8c5adeUL,
        0x000000060b797eeaUL,
        0x00000007281731bdUL,
        0x0000000b48a4940aUL,
        0x0000000d7e28a20dUL,
        0x000000006d0aac8eUL,
        0x00000002f418d6e8UL,
        0x000000045ce7d55fUL,
        0x0000000bb3415aa6UL,
        0x0000000f592d643dUL,
        0x00000000c556e7e9UL,
        0x0000000dede56c4eUL,
        0x0000000075f63628UL,
        0x0000000778f69c5bUL,
        0x0000000b0add7ab7UL,
        0x0000000d86f0b737UL,
        0x00000006d3092589UL,
        0x0000000776618a1bUL,
        0x0000000ecec99b8cUL,
        0x000000038a56d5b3UL,
        0x0000000e0cb464d1UL,
        0x00000000b9385f3aUL,
        0x0000000ba42a64f5UL,
        0x0000000938bb73f3UL,
        0x0000000a7691cb01UL,
        0x000000086a986dadUL,
        0x0000000447ed963bUL,
        0x0000000b6bcab900UL,
        0x0000000a348c9079UL,
        0x000000071812b90bUL,
        0x0000000be867b1a9UL,
        0x00000007807672aeUL,
        0x0000000f52c97520UL,
        0x000000005f6dabb7UL,
        0x00000001321e1472UL,
        0x0000000e44ab0b16UL,
        0x000000031b1b920fUL,
        0x0000000e7a916c37UL,
        0x0000000ec3a095ddUL,
        0x0000000e1cc9b481UL,
        0x000000046209b37bUL,
        0x000000047f0aa439UL,
        0x00000000d7b31befUL,
        0x000000052db78a32UL,
        0x0000000ed3b35ea8UL,
        0x00000005b1f58572UL,
        0x0000000acf866c7dUL,
        0x0000000fa2be6544UL,
        0x0000000774737a28UL,
        0x0000000bcf8fe9b1UL,
        0x0000000fdfc4ac80UL,
        0x0000000a0aef3457UL,
        0x00000005d93f4f4aUL,
        0x0000000a314ac7faUL,
        0x0000000b52221849UL,
        0x0000000b5d908dacUL,
        0x0000000ec20c9ce2UL,
        0x0000000da5563b12UL,
        0x000000057257c59bUL,
        0x0000000ef0e8f782UL,
        0x0000000f9d2b3dc4UL,
        0x0000000ab8097620UL,
        0x0000000b99a08acaUL,
        0x0000000064fe5b8bUL,
        0x000000019255e746UL,
        0x0000000aab6d88b0UL,
        0x0000000e011adc1bUL,
        0x00000003214f657cUL,
        0x0000000093f917b4UL,
        0x000000091f0c7859UL,
        0x0000000bcd3a5171UL,
        0x00000000afd4cb29UL,
        0x00000008787d5924UL,
        0x000000024f3a8b55UL,
        0x00000002ae32d017UL,
        0x000000067af72a45UL,
        0x00000008fe87d42aUL,
        0x0000000902c5df3fUL,
        0x00000002f2500693UL,
        0x0000000868a4733cUL,
        0x00000005dd4c95d9UL,
        0x000000097210f5f2UL,
        0x000000023f0de89fUL,
        0x000000084ce8a4b5UL,
        0x000000034eb80e3dUL,
        0x0000000532c70decUL,
        0x000000013b90cb54UL,
        0x0000000c3dfe6cb7UL,
        0x00000008b03d0badUL,
        0x000000090bacd3e5UL,
        0x0000000afa0bc824UL,
        0x0000000a38fe08f8UL,
        0x0000000fa758ea36UL,
        0x0000000fe0862d3cUL,
        0x00000001ace4f7edUL,
        0x0000000e04bac135UL,
        0x000000063e99b95aUL,
        0x0000000b010d68e1UL,
        0x0000000ce9352b2bUL,
        0x000000082838db6fUL,
        0x0000000d3bf04d59UL,
        0x0000000c9df5aec3UL,
        0x0000000a4e8aafeeUL,
        0x00000006b1db1459UL,
        0x00000006ba3217d0UL,
        0x0000000e5c73549cUL,
        0x000000037c2679dfUL,
        0x0000000732d69874UL,
        0x000000073ce98c9eUL,
        0x00000004cc0f5c5eUL,
        0x000000008f1b8c73UL,
        0x0000000ef1b22865UL,
        0x000000006f5eb759UL,
        0x00000001476bea11UL,
        0x000000078ba48850UL,
        0x000000074ac468d5UL,
        0x00000008ed2a2869UL,
        0x0000000a0d4f4fcdUL,
        0x00000002401bcfacUL,
        0x0000000af4852356UL,
        0x0000000e49487115UL,
        0x0000000678ebe495UL,
        0x00000003afec50b5UL,
        0x0000000ff853144eUL,
        0x0000000ca35def53UL,
        0x000000087328bda4UL,
        0x0000000d258703b5UL,
        0x000000071152a781UL,
        0x00000004dd323e43UL,
        0x00000004812c5a59UL,
        0x000000057bd1d082UL,
        0x0000000456266e15UL,
        0x00000006b9fccad6UL,
        0x00000004abe3b07bUL,
        0x0000000baf59cfceUL,
        0x000000056d6932fcUL,
        0x0000000fd158446cUL,
        0x0000000363f31ca6UL,
        0x000000093c9eecceUL,
        0x000000033930d57fUL,
        0x000000062c9dcfbbUL,
        0x00000006f52475c4UL,
        0x0000000950d51d55UL,
        0x000000095c430b37UL,
        0x00000007424acdf9UL,
        0x00000005e59f504fUL,
        0x00000007abe9fe84UL,
        0x00000001462e4fb3UL,
        0x00000005b346387fUL,
        0x0000000adcf80432UL,
        0x0000000dbcf4c50eUL,
        0x0000000213274059UL,
        0x00000007d0da7a11UL,
        0x0000000d9c5de950UL,
        0x0000000334edcf91UL,
        0x0000000eaadbf8b8UL,
        0x00000004fd450926UL,
        0x00000009a8af56b0UL,
        0x000000028b062348UL,
        0x000000086bd1c31dUL,
        0x00000000d15dd6c2UL,
        0x00000008cc9d14c6UL,
        0x0000000d0b9038c4UL,
        0x0000000b445f90b9UL,
        0x0000000c3378c977UL,
        0x0000000197aa5976UL,
        0x0000000f5879d3e2UL,
        0x00000002ec330e97UL,
        0x00000004bcab4ba7UL,
        0x0000000185b91b99UL,
        0x00000009a64ecebfUL,
        0x0000000e8b66c585UL,
        0x00000005e1e848ecUL,
        0x0000000a24ca255bUL,
        0x00000006a0d7dfc7UL,
        0x0000000770e1711aUL,
        0x00000007e34402b4UL,
        0x0000000525ceff8cUL,
        0x000000089a56e0c9UL,
        0x0000000ee1401ffcUL,
        0x0000000b13cfa633UL,
        0x0000000327aa9b9aUL,
        0x000000077383d86dUL,
        0x0000000eb6d7283eUL,
        0x00000004ba294b54UL,
        0x000000026c84d5d9UL,
        0x00000001c2c5adc0UL,
        0x0000000694e6ec3cUL,
        0x00000000f3064d10UL,
        0x00000006b7c9d416UL,
        0x0000000d7d74bd89UL,
        0x0000000e11232372UL,
        0x00000004d84369e0UL,
        0x0000000f7de40249UL,
        0x0000000b64950611UL,
        0x0000000a0724a19bUL,
        0x0000000688eef9abUL,
        0x00000003c7a3a2eaUL,
        0x000000049cd79585UL,
        0x00000006c35bff78UL,
        0x0000000456a31970UL,
        0x00000004205eb8ecUL,
        0x00000002717c22e3UL,
        0x0000000763d127fbUL,
        0x0000000865548e6fUL,
        0x00000004f1dcc597UL,
        0x0000000279dcfca4UL,
        0x0000000aad1f2e05UL,
        0x00000006437570c6UL,
        0x0000000d68b74cd9UL,
        0x0000000ec82e7216UL,
        0x00000004223b2229UL,
        0x00000009c12a8582UL,
        0x0000000c073eb63dUL,
        0x000000068b779e29UL,
        0x000000061157ce5fUL,
        0x000000050f6ebec2UL,
        0x000000035f4e4b1cUL,
        0x0000000f909c56d6UL,
        0x00000009c4b78474UL,
        0x0000000b8160eb1eUL,
        0x00000005e3d256f8UL,
        0x000000021919b529UL,
        0x0000000d4c7b6226UL,
        0x00000005190049a8UL,
        0x0000000ad869eab6UL,
        0x0000000a10b3759aUL,
        0x0000000917560e43UL,
        0x00000002c5d651d7UL,
        0x0000000eb2c8e47cUL,
        0x0000000bc131902dUL,
        0x00000007e883469bUL,
        0x000000091d991bfeUL,
        0x000000020c3a7704UL,
        0x00000002b3b53f8fUL,
        0x00000008978b2c6aUL,
        0x000000091138b228UL,
        0x0000000529f08e2cUL,
        0x0000000979342793UL,
        0x000000098137dcc3UL,
        0x00000003a9c6e139UL,
        0x0000000f898abd5eUL,
        0x0000000ce2585f36UL,
        0x0000000968312f45UL,
        0x00000002994a6ec2UL,
        0x0000000ca6685871UL,
        0x0000000a3af94337UL,
        0x000000003f628d34UL,
        0x00000001e9617f6cUL,
        0x0000000a34ed1a9cUL,
        0x00000002c22028b4UL,
        0x0000000fdeb13f3aUL,
        0x000000002d85e3acUL,
        0x0000000957491696UL,
        0x0000000ac42bab89UL,
        0x0000000e3753d50fUL,
        0x0000000b712a307aUL,
        0x00000008b7c1c2caUL,
        0x00000008893cb8f8UL,
        0x00000005abc4467fUL,
        0x00000005f1aa96c0UL,
        0x0000000b7dcde504UL,
        0x0000000c3cd9ca20UL,
        0x00000000af48c055UL,
        0x0000000f4e10477bUL,
        0x000000012b45078cUL,
        0x000000010af764c6UL,
        0x000000023d8b32d5UL,
        0x000000036f1e9c47UL,
        0x0000000e847bd756UL,
        0x00000004c3b4e455UL,
        0x000000064d42e688UL,
        0x0000000cf5218b7aUL,
        0x0000000cdc1605e4UL,
        0x00000006250bba26UL,
        0x0000000be6c4770aUL,
        0x0000000982331a23UL,
        0x0000000dfd090dcaUL,
        0x0000000506bc18d5UL,
        0x0000000109d7375cUL,
        0x0000000680ca4a7eUL,
        0x0000000a6e1c91e1UL,
        0x0000000688f44350UL,
        0x0000000ad470ae05UL,
        0x0000000f3ff0a79eUL,
        0x0000000326f26fbeUL,
        0x00000004105901beUL,
        0x000000032ed82cd9UL,
        0x000000009916c638UL,
        0x00000001d0025ed6UL,
        0x00000004aeaec114UL,
        0x000000098af27931UL,
        0x0000000968f79b3bUL,
        0x0000000ed4b906d5UL,
        0x000000061ae9b2e4UL,
        0x000000059b82a3f5UL,
        0x0000000550eb833bUL,
        0x000000017b69fd4bUL,
        0x000000073d569e02UL,
        0x000000063dba43f8UL,
        0x0000000f6cb4cb8aUL,
        0x0000000d6258181bUL,
        0x0000000e86109784UL,
        0x0000000fa5a660eaUL,
        0x00000004f9815064UL,
        0x000000075d8db35dUL,
        0x0000000546f94888UL,
        0x000000058df2d00aUL,
        0x00000006a779d19bUL,
        0x0000000ea1b4e76bUL,
        0x000000014870eef0UL,
        0x000000055429f855UL,
        0x0000000e86b71b9fUL,
        0x0000000d333c2d43UL,
        0x000000042733ecd4UL,
        0x0000000d16ba12efUL,
        0x00000003b69c3d5cUL,
        0x00000004a5c07666UL,
        0x0000000a84634048UL,
        0x000000021a2c4b26UL,
        0x0000000573be1892UL,
        0x0000000730bd1d40UL,
        0x0000000eed055d81UL,
        0x00000007e815e7a6UL,
        0x00000008ab11c883UL,
        0x000000050283e7d2UL,
        0x0000000c8a795b85UL,
        0x000000047a229dc5UL,
        0x000000012b9e108cUL,
        0x0000000caa2034dbUL,
        0x0000000a62f9ae47UL,
        0x000000093979dcfaUL,
        0x00000005fb8fd985UL,
        0x000000025bf32ccbUL,
        0x00000007ae9a5ee5UL,
        0x0000000b10665d2eUL,
        0x0000000ed0c2b0e6UL,
        0x00000002c72c25fdUL,
        0x0000000d74f444d2UL,
        0x00000009b0db80cbUL,
        0x000000055c8e43deUL,
        0x0000000d77253912UL,
        0x000000084cfd9ea0UL,
        0x00000001615ba8c5UL,
        0x0000000503884d62UL,
        0x0000000fc53328a8UL,
        0x0000000f547a6153UL,
        0x0000000ca97160cfUL,
        0x000000095e440434UL,
        0x00000006fa4c125dUL,
        0x0000000a420cfe45UL,
        0x0000000af8147dfbUL,
        0x000000012e90ddeaUL,
        0x0000000f14c3cb5eUL,
        0x0000000966579d6aUL,
        0x00000004447c94d0UL,
        0x000000032336aceaUL,
        0x0000000c75d077dfUL,
        0x0000000b7e72837eUL,
        0x000000051d24ed1eUL,
        0x000000093fb67f5cUL,
        0x000000085cedc1b2UL,
        0x00000003ca6c5d2bUL,
        0x000000032269be93UL,
        0x0000000120ed410cUL,
        0x0000000fe115d3d4UL,
        0x00000000a045ed79UL,
        0x000000083e2846deUL,
        0x00000004e7a34f3bUL,
        0x00000005078cae37UL,
        0x0000000c831aa1f0UL,
        0x000000079de6b9b6UL,
        0x00000008e39b1ad2UL,
        0x000000066bdb37a5UL,
        0x00000004a79f5da7UL,
        0x0000000cd9693b30UL,
        0x00000000dc69a07aUL,
        0x00000008649a002fUL,
        0x0000000932887d37UL,
        0x00000003926b6a74UL,
        0x0000000c5d4f8ff8UL,
        0x0000000ea290f911UL,
        0x0000000360e344e9UL,
        0x00000007fa9e8a19UL,
        0x0000000b1d0c16c3UL,
        0x00000001bbe55ddcUL,
        0x0000000825cdaea9UL,
        0x000000004f67f00fUL,
        0x0000000d3df43a25UL,
        0x0000000b07b349a2UL,
        0x0000000716a39611UL,
        0x0000000aa0fc66b7UL,
        0x0000000f928fca14UL,
        0x0000000697c299d3UL,
        0x00000006dea5d945UL,
        0x000000038cee4ea7UL,
        0x00000004e67ea7b6UL,
        0x000000024deb4062UL,
        0x0000000e4428a670UL,
        0x00000008fdb1ad13UL,
        0x0000000318ff0e1cUL,
        0x000000008c148b5dUL,
        0x0000000dbf90b839UL,
        0x000000003ea5b027UL,
        0x000000042b077ba9UL,
        0x0000000b528450e0UL,
        0x0000000a1256cbfdUL,
        0x00000004798a8e57UL,
        0x00000001fdc0d38fUL,
        0x0000000146329edcUL,
        0x0000000fa0f94b03UL,
        0x000000056c6f0e19UL,
        0x000000006db661a7UL,
        0x0000000681addf27UL,
        0x00000000d1eded30UL,
        0x0000000765c3d6e7UL,
        0x00000001ba6ae5adUL,
        0x00000005e0b49af4UL,
        0x000000033176fa1cUL,
        0x000000064ca46a3aUL,
        0x0000000389363b6fUL,
        0x00000008cc072db9UL,
        0x0000000b25f03d8fUL,
        0x000000056aa81ab1UL,
        0x00000002b1c37b54UL,
        0x0000000ac1c3279dUL,
        0x0000000669af74edUL,
        0x000000064816db93UL,
        0x0000000254f44a17UL,
        0x0000000c1f5fc290UL,
        0x0000000ebca3a222UL,
        0x000000020e75e68bUL,
        0x0000000d3bb4dfc2UL,
        0x00000008d9167a74UL,
        0x0000000b6de37939UL,
        0x0000000f7a9a93d6UL,
        0x00000009aab08164UL,
        0x00000006d186ddfcUL,
        0x000000004d3e529cUL,
        0x0000000a53e2a5c4UL,
        0x0000000d5e40b964UL,
        0x00000009a1d76463UL,
        0x00000007ad6a8eccUL,
        0x000000030b0c9279UL,
        0x0000000351e5b489UL,
        0x0000000bba967fe0UL,
        0x0000000ab3d8bf17UL,
        0x000000016c42b17aUL,
        0x00000009744fae63UL,
        0x000000077a3f2d39UL,
        0x000000083c0190acUL,
        0x00000008d016cba0UL,
        0x00000005eb9f1e35UL,
        0x00000006349c2c93UL,
        0x0000000f5a15e641UL,
        0x00000007a5bc8c86UL,
        0x000000065b6b9834UL,
        0x0000000b756618a7UL,
        0x0000000d297dc208UL,
        0x00000003d6cfca2aUL,
        0x0000000c26e4a930UL,
        0x00000003afd2df57UL,
        0x00000007f990decdUL,
        0x00000006665a2f43UL,
        0x0000000345d16f4fUL,
        0x0000000322e392bdUL,
        0x0000000567baa5deUL,
        0x00000004a6412121UL,
        0x0000000a102338a6UL,
        0x0000000b130fd768UL,
        0x000000072a30a317UL,
        0x0000000e4b71cd71UL,
        0x0000000ad4b2ceebUL,
        0x00000001da4e11cfUL,
        0x0000000d5df39b45UL,
        0x000000081622faa8UL,
        0x000000098f38af5cUL,
        0x000000025397c292UL,
        0x00000002cfcdb0d7UL,
        0x0000000d24542679UL,
        0x000000004f04f5f4UL,
        0x0000000254842f2cUL,
        0x0000000c6fce4a81UL,
        0x00000001518648bfUL,
        0x000000029ead8e85UL,
        0x0000000dc1435a6bUL,
        0x0000000c8ebc91afUL,
        0x0000000bcc94c093UL,
        0x00000008285d6a81UL,
        0x00000005b5cca263UL,
        0x000000005279a4fdUL,
        0x00000008c7dc69a7UL,
        0x0000000963e0b3fcUL,
        0x0000000e38519cb6UL,
        0x0000000e84af68a5UL,
        0x0000000d20ee188dUL,
        0x000000074fdc8660UL,
        0x000000056f95f633UL,
        0x0000000f5d2233bfUL,
        0x0000000ecb3c9815UL,
        0x0000000c21a3cee6UL,
        0x00000001558f2f83UL,
        0x00000008ea10ea50UL,
        0x00000007ededa109UL,
        0x00000004d4c5e25dUL,
        0x0000000130e15fa7UL,
        0x000000064c873cc7UL,
        0x000000035a0225a3UL,
        0x0000000d37aefb27UL,
        0x0000000b441d8903UL,
        0x00000002339f650bUL,
        0x0000000c52d79fa9UL,
        0x0000000a28badc40UL,
        0x0000000e2b99967cUL,
        0x00000008726656b3UL,
        0x0000000124e07029UL,
        0x00000008582b1dabUL,
        0x00000004eb90cbeeUL,
        0x0000000bff16b8acUL,
        0x0000000a408251fdUL,
        0x00000004feccced5UL,
        0x0000000edda7bf1cUL,
        0x000000055a0d37b2UL,
        0x00000000dd9dc90eUL,
        0x0000000a537c9ad8UL,
        0x00000008b007aeb4UL,
        0x00000006e17fbab4UL,
        0x00000000d03559e3UL,
        0x000000066fb4aaacUL,
        0x0000000d06449a6bUL,
        0x0000000528bb1818UL,
        0x000000048f0d8baeUL,
        0x0000000834d540b7UL,
        0x000000023d5d9ab9UL,
        0x00000001a13b04d1UL,
        0x0000000bd30cc9afUL,
        0x0000000e392c0fb4UL,
        0x0000000fd5d4e8bdUL,
        0x0000000cbc646d45UL,
        0x0000000348db96b7UL,
        0x0000000e32e54c5fUL,
        0x000000062446b63fUL,
        0x0000000ea47a2c4dUL,
        0x00000008d4d99969UL,
        0x0000000d01d6d0d2UL,
        0x00000001862cef24UL,
        0x0000000ed64fe4baUL,
        0x0000000d48b40fecUL,
        0x000000030aa27a14UL,
        0x00000008b2fcfdceUL,
        0x0000000943824f9aUL,
        0x0000000bab9f8d09UL,
        0x00000003204c129fUL,
        0x0000000823a36283UL,
        0x00000005a0b07693UL,
        0x0000000437949450UL,
        0x00000004ad8ca4c5UL,
        0x0000000ec979f418UL,
        0x0000000e4ed18627UL,
        0x00000008cd5d978dUL,
        0x00000005b33248f0UL,
        0x0000000bf5ac9f64UL,
        0x0000000bd427f795UL,
        0x0000000d19d39539UL,
        0x00000009f4a46c35UL,
        0x0000000532019548UL,
        0x0000000b0ebce22cUL,
        0x0000000b91133fbcUL,
        0x00000006f1983636UL,
        0x00000008f898fed7UL,
        0x0000000e642bf2ccUL,
        0x0000000a8841dfe9UL,
        0x0000000c6b879b48UL,
        0x00000007ad82cc2eUL,
        0x0000000651977fc1UL,
        0x00000004dcf4f025UL,
        0x0000000c94950e6cUL,
        0x00000002595c6eddUL,
        0x0000000d4490ebe6UL,
        0x00000004105ad270UL,
        0x0000000903d8c8efUL,
        0x000000062531ad0bUL,
        0x00000001c1fd55b3UL,
        0x0000000d27d4ced8UL,
        0x0000000dc62f305bUL,
        0x00000001e2557e49UL,
        0x0000000495c28cedUL,
        0x00000009c279d9dbUL,
        0x000000052aa6cef5UL,
        0x00000007ea533ab1UL,
        0x0000000d7d49e37bUL,
        0x0000000e8c5c7424UL,
        0x00000008137f4da1UL,
        0x0000000ae13670b3UL,
        0x00000000de97bd57UL,
        0x00000000759a6643UL,
        0x0000000d646248dfUL,
        0x0000000f5f13b6a3UL,
        0x0000000d12b2ecadUL,
        0x0000000205cccbb8UL,
        0x0000000fe6a5b5f2UL,
        0x0000000d1995e567UL,
        0x000000091d0f2174UL,
        0x0000000360e9ebefUL,
        0x0000000d304c9653UL,
        0x00000007eb43d360UL,
        0x00000009b24edb22UL,
        0x00000005aeb87d49UL,
        0x000000073a745811UL,
        0x0000000e1fd08936UL,
        0x000000033eeb68a9UL,
        0x00000006367edc20UL,
        0x000000036fd414d2UL,
        0x00000002e6722e78UL,
        0x00000006c7a9c15eUL,
        0x00000002330d46f7UL,
        0x0000000b6bc7a6e8UL,
        0x000000043eb954aaUL,
        0x000000005eadcf50UL,
        0x000000063e43b4eaUL,
        0x00000000351ca1f1UL,
        0x00000008b5682648UL,
        0x0000000c28f03f9aUL,
        0x0000000636bfe005UL,
        0x00000004ca1054e5UL,
        0x0000000440ed66c9UL,
        0x00000006e60f6fbcUL,
        0x00000006d94e25eeUL,
        0x0000000adfe9ec25UL,
        0x0000000f0cb53616UL,
        0x0000000cb19d69aaUL,
        0x0000000ff898685eUL,
        0x000000066ea208d6UL,
        0x00000005dadebaf0UL,
        0x0000000f3f7afea5UL,
        0x0000000a96739522UL,
        0x000000039a0ef88eUL,
        0x0000000626acbb6eUL,
        0x0000000ce0e42f8dUL,
        0x0000000883ae2d8fUL,
        0x0000000619bd4ce7UL,
        0x0000000be466b665UL,
        0x000000079e154eaeUL,
        0x000000073e2a0a7aUL,
        0x00000005ec94e92cUL,
        0x0000000898bc6465UL,
        0x0000000abce956ebUL,
        0x00000002b21e8995UL,
        0x000000086364e87dUL,
        0x0000000d58212c13UL,
        0x00000008b8c39e93UL,
        0x00000000a1546c8bUL,
        0x00000008abbe32b5UL,
        0x0000000e8a4a2512UL,
        0x0000000ad5ee178bUL,
        0x000000069cc1e886UL,
        0x00000005bc2f8d35UL,
        0x00000001bbcd0982UL,
        0x00000001fd969b1aUL,
        0x0000000ae9e4251bUL,
        0x00000009afc001adUL,
        0x000000069e9d82f9UL,
        0x000000092f852ed7UL,
        0x0000000adb6d7fe1UL,
        0x000000067b91999dUL,
        0x0000000fa2600b99UL,
        0x0000000d6735277eUL,
        0x0000000f836268edUL,
        0x0000000b7911c708UL,
        0x000000013427930eUL,
        0x0000000f24a44da1UL,
        0x0000000ed2f760e0UL,
        0x0000000920508584UL,
        0x00000009f1b07e2bUL,
        0x000000032c15b482UL,
        0x0000000ffcd64b43UL,
        0x0000000a055b6498UL,
        0x0000000b2a107a8bUL,
        0x0000000191147253UL,
        0x00000009c214bb01UL,
        0x00000008bb616abdUL,
        0x00000001ef7dea4aUL,
        0x00000009dd6d225dUL,
        0x000000095563112cUL,
        0x000000001e4d0569UL,
        0x0000000f21a51b18UL,
        0x0000000686d2a5ebUL,
        0x000000032032674aUL,
        0x0000000c3d902590UL,
        0x0000000856c92019UL,
        0x000000058ebdcd91UL,
        0x0000000df2c7f9aeUL,
        0x000000089d852225UL,
        0x0000000862c4110dUL,
        0x0000000dc7df8185UL,
        0x0000000c16e1da36UL,
        0x00000005a6749140UL,
        0x00000000e94d9865UL,
        0x000000032ec4b2feUL,
        0x0000000c80777c94UL,
        0x00000006869a7d57UL,
        0x00000008de18dc8eUL,
        0x0000000c98cc7379UL,
        0x000000079a6fa3a2UL,
        0x00000008b854287dUL,
        0x000000055bcd493bUL,
        0x0000000f3ddb56ecUL,
        0x0000000d15ec6405UL,
        0x0000000316f82362UL,
        0x000000039676e2a1UL,
        0x00000004044e831aUL,
        0x00000008444cf83bUL,
        0x000000064658814cUL,
        0x0000000d7ad48a68UL,
        0x0000000d5eb72583UL,
        0x0000000e11057b5dUL,
        0x0000000b7e3cd622UL,
        0x0000000a6a2f9c78UL,
        0x0000000a5924afe8UL,
        0x000000040a92e9baUL,
        0x0000000367a3950fUL,
        0x00000008dee1ebe9UL,
        0x000000052b717d05UL,
        0x00000009a5691e7dUL,
        0x0000000d454d2e0fUL,
        0x0000000fd8b299e9UL,
        0x0000000b2b1b84c2UL,
        0x00000004eacf400bUL,
        0x000000081eb4687eUL,
        0x00000005cd851fa7UL,
        0x00000001c285ea35UL,
        0x00000008ac9fdc95UL,
        0x00000001b612a6c3UL,
        0x000000094f92a63aUL,
        0x00000007debaf7c8UL,
        0x00000008cbcb7d94UL,
        0x0000000e439689eaUL,
        0x00000000932e63d2UL,
        0x0000000f3f641704UL,
        0x00000001acb5259eUL,
        0x0000000321ea6e39UL,
        0x000000061c65c02fUL,
        0x00000004282e42f2UL,
        0x00000001dd4ff338UL,
        0x0000000d91e27631UL,
        0x0000000d93927fceUL,
        0x00000004d40d6ae7UL,
        0x00000005bbfd1359UL,
        0x000000088bd393feUL,
        0x00000002535b3fd7UL,
        0x0000000b705488d3UL,
        0x00000009aa688843UL,
        0x00000009fa55ccf0UL,
        0x000000002cf10ca5UL,
        0x00000008235e2861UL,
        0x00000002610f1f02UL,
        0x0000000b203fbac1UL,
        0x0000000e05f02f3cUL,
        0x0000000b0d0e50adUL,
        0x0000000244eee955UL,
        0x00000004c5beb90dUL,
        0x000000013343b3c4UL,
        0x000000021bf9a392UL,
        0x0000000be95ecb69UL,
        0x0000000bc65857e2UL,
        0x00000008185173f4UL,
        0x0000000c3f6d059dUL,
        0x000000035c94e74aUL,
        0x0000000f72a083a3UL,
        0x00000005b36eab8fUL,
        0x0000000aa28606bbUL,
        0x0000000447d407bdUL,
        0x0000000d47eafc61UL,
        0x00000004e4dd872fUL,
        0x0000000abd276950UL,
        0x0000000b999bc572UL,
        0x00000008e0c5cee3UL,
        0x00000009fd2442d0UL,
        0x0000000bcb568051UL,
        0x0000000b87a1635bUL,
        0x0000000b3557c1e5UL,
        0x0000000ce9b07f46UL,
        0x00000000b6dfcfd2UL,
        0x000000028a540e7bUL,
        0x0000000bb04666c4UL,
        0x0000000b7baf38c6UL,
        0x0000000c37168124UL,
        0x0000000ca3c9589dUL,
        0x00000003529b99c0UL,
        0x0000000e8afab685UL,
        0x0000000bdc566027UL,
        0x00000003f1ded61bUL,
        0x0000000ce9930acdUL,
        0x000000008aa447dbUL,
        0x000000077a696e73UL,
        0x00000000eb44d17bUL,
        0x0000000c5d9c164dUL,
        0x00000001f98d7237UL,
        0x000000056a327041UL,
        0x0000000e9ecd1e58UL,
        0x00000000c2e0c7f4UL,
        0x0000000abf4a1a5eUL,
        0x00000009395c523eUL,
        0x0000000d014134faUL,
        0x00000005fbd62cadUL,
        0x0000000b247b4af6UL,
        0x0000000413f6fb3bUL,
        0x000000061c6ea7c4UL,
        0x00000008015ac6a5UL,
        0x00000000baf0e3ddUL,
        0x0000000ba1215e4aUL,
        0x0000000b01a9b92eUL,
        0x0000000c1516251fUL,
        0x000000001d0eeda9UL,
        0x0000000cf17f566fUL,
        0x00000003203947b3UL,
        0x00000007c92be543UL,
        0x0000000c320f34b8UL,
        0x00000006b1ff7e93UL,
        0x000000029d39ef02UL,
        0x0000000eda7b1139UL,
        0x0000000cca47d853UL,
        0x00000007ad24144dUL,
        0x00000007b082f234UL,
        0x0000000a6ec32320UL,
        0x0000000cf89eda25UL,
        0x0000000b64064998UL,
        0x0000000fa2314616UL,
        0x0000000d21789ac2UL,
        0x0000000d3c8d0f5dUL,
        0x00000006bac87d8dUL,
        0x00000001ff1311bdUL,
        0x00000002a1584b86UL,
        0x0000000b23dc8745UL,
        0x0000000eda5fcd76UL,
        0x0000000a7a9261d0UL,
        0x0000000512745cf6UL,
        0x00000002fc13fa8bUL,
        0x0000000e733958d3UL,
        0x0000000a057fc27dUL,
        0x0000000b4ffd8bbcUL,
        0x00000002abff4fb6UL,
        0x00000001d148a214UL,
        0x0000000d3c3c41fbUL,
        0x00000000cf542a81UL,
        0x0000000ee518b8dbUL,
        0x00000005d97028faUL,
        0x00000000dd153679UL,
        0x0000000d8e1610b5UL,
        0x0000000575122babUL,
        0x0000000090fec0a7UL,
        0x00000005aa2ee7deUL,
        0x0000000116ae8273UL,
        0x000000016991092eUL,
        0x00000003ab370aa5UL,
        0x00000003f0c514e6UL,
        0x0000000340ce20adUL,
        0x0000000d1cbbebb8UL,
        0x0000000aff5b363dUL,
        0x00000002f31fb255UL,
        0x00000005b7e53a8aUL,
        0x000000058d91de7cUL,
        0x00000006fdf2521fUL,
        0x000000080d755cb9UL,
        0x0000000cf8c9903bUL,
        0x00000000ab7ef0c7UL,
        0x00000003c61bf489UL,
        0x0000000fbfd14815UL,
        0x0000000baa647e90UL,
        0x0000000daf22defeUL,
        0x00000001a54cfe57UL,
        0x000000054d061818UL,
        0x0000000158deb36dUL,
        0x0000000a7955893aUL,
        0x0000000409b1960bUL,
        0x000000021cfed67eUL,
        0x0000000b478f7f56UL,
        0x00000002ff42f95dUL,
        0x000000048a06d3ecUL,
        0x0000000170957703UL,
        0x00000003925ece98UL,
        0x0000000d929cec5bUL,
        0x0000000aa5813ee5UL,
        0x00000009fb913a1eUL,
        0x0000000465854b47UL,
        0x00000008ac8a7343UL,
        0x000000084451853bUL,
        0x000000049987e033UL,
        0x000000075cd03c35UL,
        0x0000000cf326e2c1UL,
        0x0000000ed14fdb95UL,
        0x000000031a390e60UL,
        0x0000000a1959d956UL,
        0x00000001985d18b3UL,
        0x00000000b6b8bb85UL,
        0x0000000dc3fa4c95UL,
        0x00000003645fcf45UL,
        0x00000006c2146e60UL,
        0x00000008bdeaf7d6UL,
        0x00000005d0d268cfUL,
        0x00000008df1d5b62UL,
        0x000000077506eed4UL,
        0x00000005bebc26a3UL,
        0x0000000c78584caaUL,
        0x00000007ee6685bdUL,
        0x00000000d8af0a56UL,
        0x0000000e3a0db991UL,
        0x0000000625de434dUL,
        0x0000000582318bceUL,
        0x00000003ebe2bc3cUL,
        0x0000000ebd301053UL,
        0x00000005f579e6d7UL,
        0x0000000e9810d2aeUL,
        0x0000000b893145b4UL,
        0x0000000a7e7836d1UL,
        0x0000000679dac13bUL,
        0x00000008d99e4704UL,
        0x00000001c7234de5UL,
        0x0000000f23b8b6dbUL,
        0x0000000de8a6f02fUL,
        0x0000000bd2026bf8UL,
        0x0000000bb3903264UL,
        0x0000000ede8048afUL,
        0x000000060161ef64UL,
        0x00000008fc791a5bUL,
        0x00000006a4546243UL,
        0x0000000bc8187b47UL,
        0x0000000ff2ca8dcaUL,
        0x00000006bf71b837UL,
        0x000000078b8d1fe8UL,
        0x0000000e738cdd2aUL,
        0x00000007aa80d490UL,
        0x0000000fd1026293UL,
        0x0000000c96fe6ad3UL,
        0x000000083f0f4f7bUL,
        0x000000075cb39786UL,
        0x0000000b6ef13bddUL,
        0x0000000f6a89f8a5UL,
        0x000000099a14349eUL,
        0x00000001ce6c16e5UL,
        0x0000000a7597498fUL,
        0x0000000145206424UL,
        0x0000000013c9c5b3UL,
        0x0000000d25775fc6UL,
        0x000000072311ca63UL,
        0x0000000936d4cfe7UL,
        0x00000009ce43a505UL,
        0x0000000663594c01UL,
        0x000000042d1be7b6UL,
        0x000000080b1dc92dUL,
        0x000000020e8e5b75UL,
        0x00000000c2337a9eUL,
        0x000000058ae29f6bUL,
        0x000000090c553143UL,
        0x00000005b06da028UL,
        0x00000009415b2520UL,
        0x00000005dabcde8bUL,
        0x0000000d3c804332UL,
        0x0000000028d103dbUL,
        0x00000007be4cf210UL,
        0x000000056f76dd52UL,
        0x00000003877e1a3bUL,
        0x00000001f93eb430UL,
        0x00000000588972acUL,
        0x0000000c76cd7eb1UL,
        0x0000000852596e61UL,
        0x00000006fa0ead03UL,
        0x0000000d78be2bb6UL,
        0x000000056fdd6fd1UL,
        0x0000000e265b0ebdUL,
        0x0000000cc715a846UL,
        0x0000000ffb1e87a8UL,
        0x0000000d9893a834UL,
        0x0000000054c56923UL,
        0x000000063bbaac88UL,
        0x000000076a5585d4UL,
        0x0000000607b37e59UL,
        0x0000000ee4d4018bUL,
        0x00000007ef2d8dc5UL,
        0x0000000c1c2d4285UL,
        0x00000001e0be0a7bUL,
        0x00000009fbdac67eUL,
        0x0000000dc5c74ca1UL,
        0x00000008366e2f1cUL,
        0x0000000abf1e4843UL,
        0x00000007b55f4dacUL,
        0x00000003d6150197UL,
        0x00000004e034c8dbUL,
        0x000000063df500dfUL,
        0x000000034b89afd5UL,
        0x00000000a5ddabc7UL,
        0x00000009ee6ff8ecUL,
        0x00000003cdd9a682UL,
        0x0000000a8d46f842UL,
        0x00000005fec25f7dUL,
        0x00000004216835cfUL,
        0x00000001624430f9UL,
        0x000000018afb5b6cUL,
        0x00000009ffd34c8eUL,
        0x000000095d5882edUL,
        0x00000002e28d07cfUL,
        0x0000000fd9163bc2UL,
        0x0000000a5d0833d8UL,
        0x00000003ebaf5bfdUL,
        0x000000089a7a3e54UL,
        0x000000085d3f7dd8UL,
        0x00000008af8d1ba1UL,
        0x0000000a5b4e01b7UL,
        0x00000009f1accb0aUL,
        0x00000002fe5fcb31UL,
        0x00000002c4484bedUL,
        0x0000000a124de2eeUL,
        0x000000051471f700UL,
        0x000000014f7195f8UL,
        0x00000003ab7e9700UL,
        0x00000004542a74bfUL,
        0x00000009a01158f7UL,
        0x00000001f288d502UL,
        0x0000000af0834285UL,
        0x0000000c936df23fUL,
        0x00000002d38ea826UL,
        0x0000000b1966249aUL,
        0x0000000bc43458d4UL,
        0x000000011eb6a09cUL,
        0x0000000d49d9986cUL,
        0x0000000dd9067c0eUL,
        0x0000000ce383aa71UL,
        0x00000000739c5a2fUL,
        0x0000000770f58eaaUL,
        0x0000000a5d87da4dUL,
        0x0000000401b685acUL,
        0x0000000c65512bc1UL,
        0x00000006999d7387UL,
        0x0000000e23d36997UL,
        0x0000000e30f1d174UL,
        0x000000016e46fc04UL,
        0x0000000200efa090UL,
        0x000000018d86ea94UL,
        0x0000000b75de5761UL,
        0x00000009e50de39dUL,
        0x0000000d7a0a0161UL,
        0x0000000537a63844UL,
        0x0000000e4f4de023UL,
        0x0000000c5b83066fUL,
        0x0000000b9a2f9eb6UL,
        0x000000082a761d43UL,
        0x00000007088ad873UL,
        0x00000005b512f418UL,
        0x0000000a373cf2e4UL,
        0x00000005937d7c89UL,
        0x0000000cca051304UL,
        0x00000007a84e3efbUL,
        0x000000074d80c91bUL,
        0x00000004ad73e91dUL,
        0x0000000490cea84aUL,
        0x000000052e4f1987UL,
        0x0000000ec811379eUL,
        0x0000000b4f570589UL,
        0x00000005a6c46e81UL,
        0x0000000ca857badaUL,
        0x0000000bb9c86860UL,
        0x0000000b2632ee04UL,
        0x00000003f597b324UL,
        0x00000007df14d546UL,
        0x00000005978d3925UL,
        0x000000098498767bUL,
        0x000000015966a839UL,
        0x00000006923743efUL,
        0x000000052f38b460UL,
        0x00000006a2df69edUL,
        0x000000084151f66cUL,
        0x00000001ee304eabUL,
        0x0000000a082ae4afUL,
        0x00000000de4e6afaUL,
        0x00000008131b4166UL,
        0x0000000af5b7967aUL,
        0x000000016d942f84UL,
        0x0000000dab10053eUL,
        0x0000000cce886f54UL,
        0x000000039bc89ef4UL,
        0x0000000a5b7d9067UL,
        0x00000002fb039e23UL,
        0x000000006d82231dUL,
        0x000000003a946dd7UL,
        0x00000008b719555cUL,
        0x00000005352ece13UL,
        0x0000000904fb9151UL,
        0x0000000c7d7cc8d1UL,
        0x0000000d20511b70UL,
        0x0000000f2411eeb8UL,
        0x00000005b4a24ed9UL,
        0x0000000c00fe866fUL,
        0x0000000eb118811aUL,
        0x0000000cabf21b14UL,
        0x0000000052e3a95fUL,
        0x0000000506d306f1UL,
        0x000000022cd662d4UL,
        0x000000089161da55UL,
        0x00000007db947827UL,
        0x0000000b70b47976UL,
        0x0000000814e4e0edUL,
        0x00000008cb8f07f3UL,
        0x0000000b6979fdc4UL,
        0x0000000b0eebb6b0UL,
        0x00000005ec980ef2UL,
        0x00000004b53ccab8UL,
        0x00000009c80770e3UL,
        0x0000000b7ee43a52UL,
        0x0000000a8ef24d04UL,
        0x00000009ec4f67e6UL,
        0x0000000d5f2fb080UL,
        0x0000000efc528d35UL,
        0x0000000db181ccfbUL,
        0x000000039330efd0UL,
        0x00000001a7dd101aUL,
        0x00000004ac3982ecUL,
        0x00000007037e17dcUL,
        0x0000000366188495UL,
        0x000000002458d8a7UL,
        0x00000005f004f923UL,
        0x00000004fe549703UL,
        0x00000004d33fc825UL,
        0x0000000016f29207UL,
        0x0000000a5b2b61a0UL,
        0x0000000d9ea2cb3fUL,
        0x000000009d5e3b9bUL,
        0x00000004e473ff93UL,
        0x000000051e981524UL,
        0x000000068db475adUL,
        0x0000000def9cdca4UL,
        0x000000044a257b51UL,
        0x000000050a9ee66aUL,
        0x00000007b40b799bUL,
        0x00000004111f2de4UL,
        0x0000000cfcfb6b09UL,
        0x0000000f2412713aUL,
        0x00000008d799e9ddUL,
        0x0000000534748e45UL,
        0x000000032f291f51UL,
        0x0000000030363bd3UL,
        0x000000012906d84dUL,
        0x00000006bd9344a9UL,
        0x000000061a5524f1UL,
        0x0000000a94d8e02bUL,
        0x000000082d03a557UL,
        0x0000000468cdf94aUL,
        0x0000000bdd69de40UL,
        0x0000000ea8853fd9UL,
        0x0000000d6caf9343UL,
        0x000000001755e839UL,
        0x0000000badf446deUL,
        0x0000000fe70eae5aUL,
        0x00000001f3817ad1UL,
        0x000000091db0de1aUL,
        0x0000000c1fc0df55UL,
        0x0000000b55de01ccUL,
        0x000000066c5829bdUL,
        0x000000033cb31ff9UL,
        0x0000000ad671f856UL,
        0x0000000534431aefUL,
        0x0000000d30fd939aUL,
        0x0000000826d24c57UL,
        0x0000000d40d5f1b1UL,
        0x000000058f22448cUL,
        0x000000083611d663UL,
        0x0000000f4df3f635UL,
        0x00000001c1071941UL,
        0x00000000f870bccbUL,
        0x000000013a62329eUL,
        0x00000006d6e094a0UL,
        0x000000013c44a6a7UL,
        0x0000000129bc445bUL,
        0x000000059ee90dfdUL,
        0x00000000e0a5171aUL,
        0x00000005f92372baUL,
        0x00000008638592efUL,
        0x000000000d72f3b5UL,
        0x0000000bc7e60a8cUL,
        0x0000000ce5895c63UL,
        0x0000000e41b1ca5dUL,
        0x0000000e615ea62cUL,
        0x00000006993e2180UL,
        0x00000000cc37d82cUL,
        0x000000026af573f3UL,
        0x00000001eb95a424UL,
        0x000000050e5fbc99UL,
        0x0000000fb89b34b2UL,
        0x00000009571a8d0fUL,
        0x00000009d1a63a99UL,
        0x00000009a033e7e7UL,
        0x0000000d77b20f8cUL,
        0x0000000ed2ce19b0UL,
        0x00000005731543c1UL,
        0x0000000a59b97f53UL,
        0x00000001b3503950UL,
        0x00000007cc1f4899UL,
        0x00000003b5a20f14UL,
        0x00000003fb33bf95UL,
        0x000000024850c92fUL,
        0x0000000e3e322563UL,
        0x0000000ba303ab82UL,
        0x00000005a004a3cbUL,
        0x00000001bc3a579dUL,
        0x0000000bc3dd3ba0UL,
        0x0000000df582df03UL,
        0x0000000dfd57af2cUL,
        0x00000003863e1da0UL,
        0x0000000dac52e26fUL,
        0x00000008eb863556UL,
        0x000000094d19e451UL,
        0x00000001d50d8f5fUL,
        0x000000027d1987deUL,
        0x00000000aadc2f1eUL,
        0x0000000d55af6b1dUL,
        0x0000000737fe1509UL,
        0x0000000a49e0fe3bUL,
        0x00000002a49eeaf9UL,
        0x00000000c946b05cUL,
        0x0000000ca43c3583UL,
        0x000000019f868513UL,
        0x000000069cbd7558UL,
        0x00000005043e7034UL,
        0x0000000616e82885UL,
        0x0000000fd919ba26UL,
        0x0000000b50ca391fUL,
        0x0000000c1c578673UL,
        0x0000000d4cb35af7UL,
        0x0000000f82970fb0UL,
        0x00000002fe746c03UL,
        0x00000008a89c5009UL,
        0x00000006228cb500UL,
        0x000000027ea5af6bUL,
        0x0000000dbce2be47UL,
        0x0000000bd304a07cUL,
        0x0000000436d1ffecUL,
        0x0000000e616c764aUL,
        0x0000000e50868951UL,
        0x000000008cb63318UL,
        0x000000006a39fbc8UL,
        0x0000000d42cede82UL,
        0x0000000814c745ceUL,
        0x0000000d9032244bUL,
        0x0000000e20bc0e5aUL,
        0x00000002ce95450bUL,
        0x000000015cae0ea8UL,
        0x00000004a2ff168fUL,
        0x00000003c6b4fe2fUL,
        0x00000001999a6d47UL,
        0x0000000df7755c04UL,
        0x00000000e2da75e1UL,
        0x0000000cfb8748c2UL,
        0x00000006472c5913UL,
        0x0000000ca51e4ce0UL,
        0x000000034f2ca384UL,
        0x00000007b305ce7cUL,
        0x0000000f3704894aUL,
        0x000000055945c075UL,
        0x0000000104252729UL,
        0x0000000b16e409ffUL,
        0x00000003676f5fc8UL,
        0x00000008b9ba144fUL,
        0x0000000a322964e9UL,
        0x000000009841268bUL,
        0x00000008c8912053UL,
        0x0000000db2e4f8d1UL,
        0x000000062a58cbf4UL,
        0x000000034a15524cUL,
        0x00000000e73ea403UL,
        0x000000038492c5dcUL,
        0x0000000865ab5b6dUL,
        0x0000000b638c0fbdUL,
        0x000000085ce2ca0eUL,
        0x0000000e45949e30UL,
        0x00000000431f7881UL,
        0x00000004f8626706UL,
        0x0000000f59c896ceUL,
        0x00000008d4e37dd7UL,
        0x000000072e5e016bUL,
        0x000000019c9570a6UL,
        0x0000000eb0e55828UL,
        0x0000000f3cb02a1bUL,
        0x0000000b609ff814UL,
        0x0000000dab87f876UL,
        0x000000076d547356UL,
        0x0000000b9f95ae4eUL,
        0x000000063a65e9e3UL,
        0x0000000b6838a991UL,
        0x000000012b64eb81UL,
        0x0000000c59ced5a6UL,
        0x0000000d715aecdbUL,
        0x0000000b4291fe13UL,
        0x000000006d0d0693UL,
        0x0000000f98794c86UL,
        0x00000005c816d4d8UL,
        0x0000000c947590ffUL,
        0x000000080bb44a94UL,
        0x0000000f59bd80c8UL,
        0x0000000df761d2bcUL,
        0x0000000217a8fb51UL,
        0x000000093d47240dUL,
        0x0000000761920193UL,
        0x0000000e4e5eba29UL,
        0x0000000eeca76549UL,
        0x0000000a1bbfc9c4UL,
        0x0000000c711b2477UL,
        0x0000000325b5a666UL,
        0x0000000ce05f41f2UL,
        0x00000003da9c4cffUL,
        0x0000000120c47b51UL,
        0x0000000cd32263aeUL,
        0x0000000e799e7548UL,
        0x0000000ae372d335UL,
        0x0000000063ddbda3UL,
        0x0000000267fac28cUL,
        0x000000001fc9d86bUL,
        0x000000017641543bUL,
        0x0000000c72e347daUL,
        0x00000006ffd0717aUL,
        0x00000003c30cb70fUL,
        0x00000000820ea2bdUL,
        0x00000003e9d4bf2bUL,
        0x0000000a3c30b85cUL,
        0x0000000f9f0118dfUL,
        0x0000000e4b9fe4d3UL,
        0x000000029c0611aeUL,
        0x0000000a8c35fa15UL,
        0x00000001b9e934caUL,
        0x0000000480cb9d1bUL,
        0x0000000bf800f039UL,
        0x0000000c84dca5d6UL,
        0x00000007bc72f527UL,
        0x00000007b9239faeUL,
        0x00000007d89ef157UL,
        0x00000007ac85a437UL,
        0x0000000d16f6d1beUL,
        0x0000000edf98b6ffUL,
        0x0000000e8686dc27UL,
        0x00000004859dce4bUL,
        0x0000000041627612UL,
        0x0000000fc09cbd90UL,
        0x0000000fdae828b9UL,
        0x00000008e8ab9aacUL,
        0x0000000e3c3a5acbUL,
        0x0000000df138b3f6UL,
        0x000000068b6499ceUL,
        0x0000000bf229a907UL,
        0x0000000c5565d691UL,
        0x00000008f8119c05UL,
        0x000000096751f8e6UL,
        0x0000000ac2f319d4UL,
        0x0000000dc9a8a43bUL,
        0x000000023e877e11UL,
        0x00000006bd59226fUL,
        0x000000059360ec26UL,
        0x0000000e9afc1dd5UL,
        0x0000000de69624a6UL,
        0x0000000de54a22a7UL,
        0x0000000ea27ba143UL,
        0x00000008bcdef6a3UL,
        0x00000004779f40beUL,
        0x000000065a1a6cbcUL,
        0x0000000f31b71b67UL,
        0x00000001dee9ec9bUL,
        0x0000000a8bbd546bUL,
        0x00000004d77af897UL,
        0x000000053be52417UL,
        0x00000009ab39f2e2UL,
        0x0000000c7672be23UL,
        0x000000085bb59414UL,
        0x0000000f77b6f781UL,
        0x0000000aecf29f03UL,
        0x00000003ea7deeb5UL,
        0x000000053154616bUL,
        0x0000000f0ad4172eUL,
        0x0000000ac97900c4UL,
        0x000000024af96431UL,
        0x0000000742dcac09UL,
        0x0000000b77612a7cUL,
        0x000000043f8e0b06UL,
        0x0000000b9af4d04cUL,
        0x00000005f0c0eeb7UL,
        0x0000000beed5d839UL,
        0x0000000594bc055fUL,
        0x0000000bf3d4ee51UL,
        0x000000036dc329e7UL,
        0x00000006292a913dUL,
        0x0000000853afc28dUL,
        0x00000008f669cd4dUL,
        0x000000035d67967dUL,
        0x0000000a1ba1cda9UL,
        0x0000000475e70d85UL,
        0x0000000db124355eUL,
        0x0000000b7464de4bUL,
        0x0000000210a5d6ecUL,
        0x00000003f3b26a00UL,
        0x00000003b654ab0bUL,
        0x00000005461eabb8UL,
        0x0000000add3bc40fUL,
        0x00000000223eae50UL,
        0x0000000d0a8f290dUL,
        0x000000070ec2982dUL,
        0x00000000e98ba15aUL,
        0x00000001bf087c2eUL,
        0x00000006e98233f2UL,
        0x0000000b06e2c22bUL,
        0x000000045483b62dUL,
        0x00000002fadab9aeUL,
        0x0000000112d8a187UL,
        0x0000000c1a9043c6UL,
        0x000000007abad130UL,
        0x000000092fd1710eUL,
        0x0000000f55801505UL,
        0x0000000f42daa0b3UL,
        0x000000063e7b4d45UL,
        0x0000000f976ab37bUL,
        0x000000059c9833d0UL,
        0x000000098cbdb021UL,
        0x0000000da67a27caUL,
        0x0000000d0e6edf39UL,
        0x0000000d8fdd70b9UL,
        0x00000006b86d3b06UL,
        0x00000003979ed2b7UL,
        0x000000046a8a0a6cUL,
        0x0000000fba0b5f5fUL,
        0x0000000c34a9713dUL,
        0x0000000840e7387cUL,
        0x0000000ca04d9f6aUL,
        0x000000076073e651UL,
        0x0000000e6f2fca96UL,
        0x0000000daef51fa4UL,
        0x00000007824e6073UL,
        0x0000000dbbb9e428UL,
        0x000000064bda5a5dUL,
        0x0000000d3944a8beUL,
        0x0000000895b7c247UL,
        0x0000000a8a092b5eUL,
        0x00000008454ae0f6UL,
        0x00000001a3109cc6UL,
        0x0000000411731a19UL,
        0x0000000ee3ccd9e5UL,
        0x0000000865688814UL,
        0x000000084c59bdd7UL,
        0x0000000bdf4f34e2UL,
        0x0000000192b2633cUL,
        0x0000000f7c30ff94UL,
        0x0000000bbd0187c5UL,
        0x000000036858be42UL,
        0x00000007cf38933fUL,
        0x0000000b1c48417eUL,
        0x000000077b63288cUL,
        0x0000000794af1c0eUL,
        0x0000000b5afd4a02UL,
        0x000000092e67eb6bUL,
        0x00000006253569e5UL,
        0x00000001405ca65fUL,
        0x00000004ca61bba3UL,
        0x000000006fe876e9UL,
        0x0000000c25b98680UL,
        0x00000000bd52f241UL,
        0x0000000de92bdbc6UL,
        0x00000007114244f7UL,
        0x0000000c92109927UL,
        0x000000011246d25fUL,
        0x0000000878963531UL,
        0x0000000b8f639289UL,
        0x0000000edd1ee2dcUL,
        0x00000002dd27bd83UL,
        0x00000005b77fb633UL,
        0x000000092d244feaUL,
        0x0000000ff29f378bUL,
        0x0000000cd74b01dfUL,
        0x0000000caee75d99UL,
        0x000000032a7950e7UL,
        0x00000008ab7143d0UL,
        0x00000004a396d396UL,
        0x000000027cc982aaUL,
        0x0000000748ffbcacUL,
        0x00000007dc1212dbUL,
        0x000000060491f30fUL,
        0x00000006462a44c5UL,
        0x0000000f693b3934UL,
        0x00000002c85c32c8UL,
        0x000000058842e73aUL,
        0x000000039c5c936aUL,
        0x000000059da18b06UL,
        0x0000000b0c4be569UL,
        0x0000000b38429749UL,
        0x000000014d177c42UL,
        0x0000000b2890755cUL,
        0x0000000c316f3c01UL,
        0x000000097a978118UL,
        0x00000006e3af9c97UL,
        0x0000000b16bfcf5bUL,
        0x0000000fbdedeaa1UL,
        0x000000062d2efff3UL,
        0x0000000221e13b03UL,
        0x00000006f189a8f4UL,
        0x0000000ba3c855cfUL,
        0x00000004d9b593f1UL,
        0x000000070bdb83c4UL,
        0x00000007be402a75UL,
        0x0000000acbb1dcf6UL,
        0x00000006d76dd708UL,
        0x00000004210a8723UL,
        0x0000000e4a9e3182UL,
        0x00000000b86ea177UL,
        0x0000000589c7c8dfUL,
        0x0000000728f649ccUL,
        0x0000000235daf936UL,
        0x0000000ed5b13b07UL,
        0x00000009838cc62bUL,
        0x0000000699ea0fb3UL,
        0x0000000533af329dUL,
        0x00000000486ec47dUL,
        0x0000000ea7e7874dUL,
        0x0000000ae99ea9c5UL,
        0x000000037e02d2d5UL,
        0x0000000b92cf5ea1UL,
        0x0000000e8c6fa527UL,
        0x000000093ac59f00UL,
        0x0000000a5876d0fbUL,
        0x000000042664d4abUL,
        0x0000000672c67c2fUL,
        0x0000000a499d1a9bUL,
        0x00000001c55c99a8UL,
        0x00000005d5644f76UL,
        0x0000000149b6cd96UL,
        0x00000005d19a752aUL,
        0x0000000fa5fce591UL,
        0x00000005380ec50cUL,
        0x0000000dea0d752dUL,
        0x00000001faf5309bUL,
        0x0000000e3b7a775eUL,
        0x0000000173d59edbUL,
        0x0000000b8e38cca5UL,
        0x00000001e43cad4eUL,
        0x000000027dedf623UL,
        0x0000000da72d480fUL,
        0x000000099434d76fUL,
        0x00000008e06320e5UL,
        0x000000012cc5d9b3UL,
        0x0000000d4ec00d0aUL,
        0x00000008fa256ec8UL,
        0x00000002a7755b41UL,
        0x0000000649598f17UL,
        0x0000000048a9ea61UL,
        0x0000000b749bbe4dUL,
        0x000000059e4c9da2UL,
        0x000000050395f36cUL,
        0x00000006b07149d2UL,
        0x00000004f9406d7fUL,
        0x000000092febe987UL,
        0x0000000d7bf93ed2UL,
        0x00000001e66cc0f3UL,
        0x00000006c48e37c0UL,
        0x00000003ad8159d5UL,
        0x000000048f41fbc8UL,
        0x0000000cd5f40d31UL,
        0x00000004d9d50e50UL,
        0x0000000377552d44UL,
        0x0000000d6400b5bfUL,
        0x0000000205260a56UL,
        0x0000000c319eaf12UL,
        0x0000000bb3016818UL,
        0x0000000194c9caa5UL,
        0x00000003205df053UL,
        0x00000008dbf64dc8UL,
        0x00000005cc34b609UL,
        0x00000007d2367f18UL,
        0x000000020d6cee2eUL,
        0x0000000c9f86c18eUL,
        0x0000000e2b1f4620UL,
        0x000000009c52f03eUL,
        0x000000007d8b9b31UL,
        0x00000002b85f049fUL,
        0x0000000e0b542614UL,
        0x0000000d0f06b745UL,
        0x0000000df7b0f962UL,
        0x0000000d7b41890fUL,
        0x000000024baf7927UL,
        0x0000000cc63c5347UL,
        0x0000000c9dbe3e8eUL,
        0x000000052b1c7225UL,
        0x000000073f558c8dUL,
        0x0000000542702b85UL,
        0x0000000578f18fddUL,
        0x0000000a76cf36efUL,
        0x0000000a3cb92407UL,
        0x0000000b7ad6cd05UL,
        0x00000006d0f34533UL,
        0x0000000416083d1bUL,
        0x00000002bdb7fab0UL,
        0x000000029619e1d2UL,
        0x0000000fe824f4c1UL,
        0x00000007a5ff22b9UL,
        0x00000005be7f7ce2UL,
        0x00000004140ddebeUL,
        0x0000000ec104f338UL,
        0x00000003c6ea6417UL,
        0x00000004c68c4841UL,
        0x0000000738550624UL,
        0x000000095d52ffbeUL,
        0x00000001211a3407UL,
        0x000000085c2667c7UL,
        0x0000000d3fcfc70aUL,
        0x000000062c79fa03UL,
        0x0000000e5a01d1deUL,
        0x000000021a6f78c0UL,
        0x000000067a9d08d0UL,
        0x00000003521d86a9UL,
        0x0000000821858962UL,
        0x00000000238c9ab4UL,
        0x0000000d377e444eUL,
        0x00000009b4db4370UL,
        0x0000000326d40d28UL,
        0x0000000cca399d64UL,
        0x000000029e717061UL,
        0x000000081c257536UL,
        0x0000000b5398ff6fUL,
        0x00000006246d505dUL,
        0x000000052de44206UL,
        0x0000000d4bfe70a7UL,
        0x00000001256f3bbfUL,
        0x0000000e18dca845UL,
        0x0000000cadc6270eUL,
        0x0000000ceecebcf9UL,
        0x000000030490b131UL,
        0x00000003499eae73UL,
        0x0000000e407687b5UL,
        0x00000000d16a1b5cUL,
        0x00000000aed4fc7cUL,
        0x0000000977235aceUL,
        0x0000000e45bc089cUL,
        0x0000000d1c7e8b0fUL,
        0x0000000c801f2a62UL,
        0x00000009bb982febUL,
        0x0000000c2ade21d5UL,
        0x0000000e664c4cdcUL,
        0x0000000b5b1b7d8dUL,
        0x000000057e319e42UL,
        0x0000000a62708260UL,
        0x00000004d49e8e90UL,
        0x00000000cb5f349aUL,
        0x000000097f8c8487UL,
        0x0000000bf26ecbd1UL,
        0x00000006a22fc93fUL,
        0x0000000f40123ad5UL,
        0x0000000f2864b84fUL,
        0x000000052b894393UL,
        0x0000000aeac3e443UL,
        0x000000064a6e37e9UL,
        0x00000005ff905511UL,
        0x00000000f9368ae9UL,
        0x0000000304c961d1UL,
        0x0000000901e3d5efUL,
        0x00000003d9c9e1eaUL,
        0x000000014af08310UL,
        0x0000000f7d5f48faUL,
        0x00000004cc90c712UL,
        0x000000093eb3c4f2UL,
        0x0000000c64af2ee0UL,
        0x0000000ee9b0b880UL,
        0x000000059e9f00c4UL,
        0x0000000f0e385a56UL,
        0x0000000013e3176aUL,
        0x000000022cefe2bfUL,
        0x0000000e8d646da2UL,
        0x0000000fa07e9914UL,
        0x000000064a342dc3UL,
        0x0000000029436fdeUL,
        0x00000003f436a788UL,
        0x0000000ebce59d33UL,
        0x0000000d1cc417e5UL,
        0x0000000bdabb3afcUL,
        0x00000002b075aa91UL,
        0x0000000cf479bc86UL,
        0x00000001bc69c922UL,
        0x0000000f70bed102UL,
        0x000000073fba7412UL,
        0x0000000dad289c99UL,
        0x000000025fa19a7eUL,
        0x0000000a5a248105UL,
        0x000000059c85904fUL,
        0x0000000587368f7aUL,
        0x00000006ddf6cd9bUL,
        0x0000000276aaeac2UL,
        0x000000039793390eUL,
        0x0000000ee79121bdUL,
        0x000000043b22f813UL,
        0x0000000137d42357UL,
        0x000000067857c8bdUL,
        0x000000046cf4e741UL,
        0x00000001d18cd171UL,
        0x0000000a9806c31fUL,
        0x000000006605176cUL,
        0x00000000f592c1acUL,
        0x00000007265a10a4UL,
        0x0000000e89d3f153UL,
        0x0000000b2c1d25d5UL,
        0x0000000fc221a68cUL,
        0x000000021bc5cb0fUL,
        0x00000007d8865a48UL,
        0x000000014fb6a9c1UL,
        0x0000000826702ff5UL,
        0x0000000c875eecddUL,
        0x0000000334906e55UL,
        0x0000000e75f38920UL,
        0x0000000e68cc8585UL,
        0x0000000e21989dadUL,
        0x0000000991c0a54fUL,
        0x0000000afc2e4db2UL,
        0x000000049215ae09UL,
        0x00000004414dd21cUL,
        0x0000000270102c2bUL,
        0x0000000d74d46724UL,
        0x0000000d8ba0e6cfUL,
        0x00000004924c591cUL,
        0x000000014a218727UL,
        0x0000000ec649a317UL,
        0x000000040506e864UL,
        0x00000004a5fce324UL,
        0x0000000432da3920UL,
        0x0000000be6c87867UL,
        0x000000096d47ab50UL,
        0x000000013b26554aUL,
        0x000000042d294c1aUL,
        0x000000043a8f4537UL,
        0x0000000ec9981703UL,
        0x00000009d23e9feaUL,
        0x0000000a5e63c741UL,
        0x0000000b590555a7UL,
        0x0000000af5cc76d0UL,
        0x000000036f6807beUL,
        0x0000000d9e7509baUL,
        0x000000008e7485e6UL,
        0x0000000ca410c173UL,
        0x000000034c0efac9UL,
        0x0000000aa8dd95f8UL,
        0x000000092ff8e340UL,
        0x00000002fcf617e5UL,
        0x0000000f4ee3deeaUL,
        0x0000000ec3675f06UL,
        0x000000011d3034ccUL,
        0x000000053d80da67UL,
        0x00000002ffd3e2edUL,
        0x0000000701c5dbcaUL,
        0x0000000fd41bd301UL,
        0x0000000176d0d999UL,
        0x000000084a7a607aUL,
        0x00000003a4cd4f6eUL,
        0x000000029fd854f3UL,
        0x0000000239143be4UL,
        0x0000000c1d83b7ceUL,
        0x00000003998e35f9UL,
        0x0000000c9cade2feUL,
        0x000000067a7ce5f8UL,
        0x000000058096eb5eUL,
        0x0000000129003b1aUL,
        0x000000058cd60dd3UL,
        0x00000007ec9c978cUL,
        0x00000008209b7fc6UL,
        0x0000000b1a07b065UL,
        0x00000002f8c5d980UL,
        0x0000000ec68f5262UL,
        0x00000009f6738ae8UL,
        0x0000000e240da32dUL,
        0x000000029a692dd3UL,
        0x000000093a2fc212UL,
        0x000000090a7ad415UL,
        0x000000038462d719UL,
        0x000000088d09ed64UL,
        0x0000000629414143UL,
        0x0000000f37f54627UL,
        0x0000000f654223daUL,
        0x00000009866d0462UL,
        0x000000039b7e344bUL,
        0x0000000e29b0d31eUL,
        0x00000004d26f996bUL,
        0x0000000ca999e2d4UL,
        0x000000048244cecaUL,
        0x0000000db430a091UL,
        0x0000000b20a09f71UL,
        0x000000089d7a977dUL,
        0x0000000d0c7d781fUL,
        0x000000045dbd608bUL,
        0x00000005f0eaf573UL,
        0x00000004b72a881eUL,
        0x00000006721d71acUL,
        0x0000000aea5b5257UL,
        0x0000000729da786eUL,
        0x0000000b7bbb055eUL,
        0x000000033c26e035UL,
        0x0000000bbb1faaf3UL,
        0x0000000a7acc30bcUL,
        0x00000001ce0841a4UL,
        0x0000000244d1d408UL,
        0x0000000cde684816UL,
        0x00000000f8fb4cdaUL,
        0x0000000706c97e43UL,
        0x0000000c971d3afbUL,
        0x0000000fda0d9c09UL,
        0x0000000016b86da3UL,
        0x0000000fad3fd2e5UL,
        0x0000000bac1633c9UL,
        0x00000002309feea7UL,
        0x00000000033737bbUL,
        0x00000009aed89097UL,
        0x000000068538abb5UL,
        0x0000000828f7850dUL,
        0x000000072432d95dUL,
        0x0000000261a8e5a9UL,
        0x0000000a200eb1b2UL,
        0x0000000c3de7f043UL,
        0x0000000bf74c5970UL,
        0x00000006dd1481b8UL,
        0x0000000aa94bbef6UL,
        0x0000000da7e65697UL,
        0x0000000bc3ab5693UL,
        0x0000000fac70dbecUL,
        0x000000011c7b546dUL,
        0x0000000f2f1877aeUL,
        0x0000000ddfd559cbUL,
        0x0000000213843237UL,
        0x0000000d35629a71UL,
        0x000000066b966ff8UL,
        0x0000000f7c3d2b42UL,
        0x0000000d2a9bf013UL,
        0x000000082aade491UL,
        0x000000080a74d66aUL,
        0x0000000c23180cd5UL,
        0x000000013bc699afUL,
        0x0000000568f373afUL,
        0x00000004e8f6cb22UL,
        0x00000000236593daUL,
        0x0000000c06d419daUL,
        0x0000000fb63c9338UL,
        0x0000000c58ea0e62UL,
        0x00000003f129af28UL,
        0x00000001bc5d8e7cUL,
        0x00000005a0cce5a2UL,
        0x000000012138fcc9UL,
        0x00000006475b47b3UL,
        0x000000036127061cUL,
        0x0000000a66ea1c90UL,
        0x0000000c7622d06fUL,
        0x00000007b06f9883UL,
        0x000000017f9dd0f5UL,
        0x00000004f7d20a59UL,
        0x000000080ca1778dUL,
        0x000000034bf99f09UL,
        0x0000000b1f6b20ffUL,
        0x0000000ddc248171UL,
        0x000000079adedf2dUL,
        0x0000000e7b8ec414UL,
        0x00000001660b1a2aUL,
        0x0000000596131143UL,
        0x0000000ed5876f22UL,
        0x0000000ccf196212UL,
        0x00000005a7da3ed3UL,
        0x00000005d7c1c564UL,
        0x0000000ebf3b80d4UL,
        0x00000000fbb5da08UL,
        0x0000000751b26b7fUL,
        0x0000000f646e15e0UL,
        0x000000048cf268f5UL,
        0x0000000681ff913eUL,
        0x00000008a60709eeUL,
        0x0000000ee8aa3c19UL,
        0x0000000d74c7190bUL,
        0x0000000403a516f6UL,
        0x000000043859f5e7UL,
        0x0000000674b05601UL,
        0x000000029dc78f7bUL,
        0x0000000f871a0d9cUL,
        0x0000000149cd7e80UL,
        0x0000000e0ebb3dadUL,
        0x0000000cfd60dc21UL,
        0x0000000fcac192f3UL,
        0x0000000da0bb9f49UL,
        0x0000000377891815UL,
        0x00000008cf831888UL,
        0x0000000d8ad9cff2UL,
        0x000000066ef37c63UL,
        0x0000000289532c5dUL,
        0x000000045d8a5400UL,
        0x0000000452111b4dUL,
        0x0000000be9b62af0UL,
        0x00000004a7fa60afUL,
        0x0000000f38ab400eUL,
        0x0000000329b05789UL,
        0x00000005edba3023UL,
        0x00000005f48632e5UL,
        0x0000000c44562244UL,
        0x0000000781f865e7UL,
        0x00000004868de42dUL,
        0x0000000b7d2aafddUL,
        0x00000000652863c6UL,
        0x00000009c8c0a919UL,
        0x0000000a566fd21eUL,
        0x0000000d42a60c7aUL,
        0x00000003b2fa8afeUL,
        0x00000004feead808UL,
        0x00000004e08205a5UL,
        0x00000002ca3d8a2fUL,
        0x0000000983fc5a57UL,
        0x0000000c23db45c9UL,
        0x0000000a37e0ba04UL,
        0x00000005f510ba05UL,
        0x00000001601a2ff1UL,
        0x000000030cccbfe0UL,
        0x00000003880f5176UL,
        0x00000000ea5aee4fUL,
        0x0000000b1b166fdbUL,
        0x00000007b3ca3761UL,
        0x000000064fae3ebfUL,
        0x0000000447b9e7acUL,
        0x0000000d9f564f30UL,
    }
};

#undef N_BITS

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "tag36h11"
////////////////////////////////////////////////////////////////////////////////////////////////////

#define N_BITS 36

const static uint32_t tag36h11_bit_x[N_BITS] = {1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5,
   5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3};
const static uint32_t tag36h11_bit_y[N_BITS] = { 1, 1, 1, 1, 1, 2, 2, 2, 3, 1,
   2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4 };

const apriltag_family_t tag36h11 = {
    .ncodes = 587,
    .width_at_border = 8,
    .total_width = 10,
    .d = 6,
    .reversed_border = false,
    .nbits = N_BITS,
    .bit_x = tag36h11_bit_x,
    .bit_y = tag36h11_bit_y,
    .h = 11,
    .codes = {
        0x0000000d7e00984bUL,
        0x0000000dda664ca7UL,
        0x0000000dc4a1c821UL,
        0x0000000e17b470e9UL,
        0x0000000ef91d01b1UL,
        0x0000000f429cdd73UL,
        0x000000005da29225UL,
        0x00000001106cba43UL,
        0x0000000223bed79dUL,
        0x000000021f51213cUL,
        0x000000033eb19ca6UL,
        0x00000003f76eb0f8UL,
        0x0000000469a97414UL,
        0x000000045dcfe0b0UL,
        0x00000004a6465f72UL,
        0x000000051801db96UL,
        0x00000005eb946b4eUL,
        0x000000068a7cc2ecUL,
        0x00000006f0ba2652UL,
        0x000000078765559dUL,
        0x000000087b83d129UL,
        0x000000086cc4a5c5UL,
        0x00000008b64df90fUL,
        0x00000009c577b611UL,
        0x0000000a3810f2f5UL,
        0x0000000af4d75b83UL,
        0x0000000b59a03fefUL,
        0x0000000bb1096f85UL,
        0x0000000d1b92fc76UL,
        0x0000000d0dd509d2UL,
        0x0000000e2cfda160UL,
        0x00000002ff497c63UL,
        0x000000047240671bUL,
        0x00000005047a2e55UL,
        0x0000000635ca87c7UL,
        0x0000000691254166UL,
        0x000000068f43d94aUL,
        0x00000006ef24bdb6UL,
        0x00000008cdd8f886UL,
        0x00000009de96b718UL,
        0x0000000aff6e5a8aUL,
        0x0000000bae46f029UL,
        0x0000000d225b6d59UL,
        0x0000000df8ba8c01UL,
        0x0000000e3744a22fUL,
        0x0000000fbb59375dUL,
        0x000000018a916828UL,
        0x000000022f29c1baUL,
        0x0000000286887d58UL,
        0x000000041392322eUL,
        0x000000075d18ecd1UL,
        0x000000087c302743UL,
        0x00000008c6317ba9UL,
        0x00000009e40f36d7UL,
        0x0000000c0e5a806aUL,
        0x0000000cc78cb87cUL,
        0x000000012d2f2d01UL,
        0x0000000379f36a21UL,
        0x00000006973f59acUL,
        0x00000007789ea9f4UL,
        0x00000008f1c73e84UL,
        0x00000008dd287a20UL,
        0x000000094a4eee4cUL,
        0x0000000a455379b5UL,
        0x0000000a9e92987dUL,
        0x0000000bd25cb40bUL,
        0x0000000be98d3582UL,
        0x0000000d3d5972b2UL,
        0x000000014c53d7c7UL,
        0x00000004f1796936UL,
        0x00000004e71fed1aUL,
        0x000000066d46fae0UL,
        0x0000000a55abb933UL,
        0x0000000ebee1accaUL,
        0x00000001ad4ba6a4UL,
        0x0000000305b17571UL,
        0x0000000553611351UL,
        0x000000059ca62775UL,
        0x00000007819cb6a1UL,
        0x0000000edb7bc9ebUL,
        0x00000005b2694212UL,
        0x000000072e12d185UL,
        0x0000000ed6152e2cUL,
        0x00000005bcdadbf3UL,
        0x000000078e0aa0c6UL,
        0x0000000c60a0b909UL,
        0x0000000ef9a34b0dUL,
        0x0000000398a6621aUL,
        0x0000000a8a27c944UL,
        0x00000004b564304eUL,
        0x000000052902b4e2UL,
        0x0000000857280b56UL,
        0x0000000a91b2c84bUL,
        0x0000000e91df939bUL,
        0x00000001fa405f28UL,
        0x000000023793ab86UL,
        0x000000068c17729fUL,
        0x00000009fbf3b840UL,
        0x000000036922413cUL,
        0x00000004eb5f946eUL,
        0x0000000533fe2404UL,
        0x000000063de7d35eUL,
        0x0000000925eddc72UL,
        0x000000099b8b3896UL,
        0x0000000aace4c708UL,
        0x0000000c22994af0UL,
        0x00000008f1eae41bUL,
        0x0000000d95fb486cUL,
        0x000000013fb77857UL,
        0x00000004fe0983a3UL,
        0x0000000d559bf8a9UL,
        0x0000000e1855d78dUL,
        0x0000000fec8daaadUL,
        0x000000071ecb6d95UL,
        0x0000000dc9e50e4cUL,
        0x0000000ca3a4c259UL,
        0x0000000740d12bbfUL,
        0x0000000aeedd18e0UL,
        0x0000000b509b9c8eUL,
        0x00000005232fea1cUL,
        0x000000019282d18bUL,
        0x000000076c22d67bUL,
        0x0000000936beb34bUL,
        0x000000008a5ea8ddUL,
        0x0000000679eadc28UL,
        0x0000000a08e119c5UL,
        0x000000020a6e3e24UL,
        0x00000007eab9c239UL,
        0x000000096632c32eUL,
        0x0000000470d06e44UL,
        0x00000008a70212fbUL,
        0x00000000a7e4251bUL,
        0x00000009ec762cc0UL,
        0x0000000d8a3a1f48UL,
        0x0000000db680f346UL,
        0x00000004a1e93a9dUL,
        0x0000000638ddc04fUL,
        0x00000004c2fcc993UL,
        0x000000001ef28c95UL,
        0x0000000bf0d9792dUL,
        0x00000006d27557c3UL,
        0x0000000623f977f4UL,
        0x000000035b43be57UL,
        0x0000000bb0c428d5UL,
        0x0000000a6f01474dUL,
        0x00000005a70c9749UL,
        0x000000020ddabc3bUL,
        0x00000002eabd78cfUL,
        0x000000090aa18f88UL,
        0x0000000a9ea89350UL,
        0x00000003cdb39b22UL,
        0x0000000839a08f34UL,
        0x0000000169bb814eUL,
        0x00000001a575ab08UL,
        0x0000000a04d3d5a2UL,
        0x0000000bf7902f2bUL,
        0x0000000095a5e65cUL,
        0x000000092e8fce94UL,
        0x000000067ef48d12UL,
        0x00000006400dbcacUL,
        0x0000000b12d8fb9fUL,
        0x00000000347f45d3UL,
        0x0000000b35826f56UL,
        0x0000000c546ac6e4UL,
        0x000000081cc35b66UL,
        0x000000041d14bd57UL,
        0x00000000c052b168UL,
        0x00000007d6ce5018UL,
        0x0000000ab4ed5edeUL,
        0x00000005af817119UL,
        0x0000000d1454b182UL,
        0x00000002badb090bUL,
        0x000000003fcb4c0cUL,
        0x00000002f1c28fd8UL,
        0x000000093608c6f7UL,
        0x00000004c93ba2b5UL,
        0x000000007d950a5dUL,
        0x0000000e54b3d3fcUL,
        0x000000015560cf9dUL,
        0x0000000189e4958aUL,
        0x000000062140e9d2UL,
        0x0000000723bc1cdbUL,
        0x00000002063f26faUL,
        0x0000000fa08ab19fUL,
        0x00000007955641dbUL,
        0x0000000646b01daaUL,
        0x000000071cd427ccUL,
        0x000000009a42f7d4UL,
        0x0000000717edc643UL,
        0x000000015eb94367UL,
        0x00000008392e6bb2UL,
        0x0000000832408542UL,
        0x00000002b9b874beUL,
        0x0000000b21f4730dUL,
        0x0000000b5d8f24c9UL,
        0x00000007dbaf6931UL,
        0x00000001b4e33629UL,
        0x000000013452e710UL,
        0x0000000e974af612UL,
        0x00000001df61d29aUL,
        0x000000099f2532adUL,
        0x0000000e50ec71b4UL,
        0x00000005df0a36e8UL,
        0x00000004934e4ceaUL,
        0x0000000e34a0b4bdUL,
        0x0000000b7b26b588UL,
        0x00000000f255118dUL,
        0x0000000d0c8fa31eUL,
        0x000000006a50c94fUL,
        0x0000000f28aa9f06UL,
        0x0000000131d194d8UL,
        0x0000000622e3da79UL,
        0x0000000ac7478303UL,
        0x0000000c8f2521d7UL,
        0x00000006c9c881f5UL,
        0x000000049e38b60aUL,
        0x0000000513d8df65UL,
        0x0000000d7c2b0785UL,
        0x00000009f6f9d75aUL,
        0x00000009f6966020UL,
        0x00000001e1a54e33UL,
        0x0000000c04d63419UL,
        0x0000000946e04cd7UL,
        0x00000001bdac5902UL,
        0x000000056469b830UL,
        0x0000000ffad59569UL,
        0x000000086970e7d8UL,
        0x00000008a4b41e12UL,
        0x0000000ad4688e3bUL,
        0x000000085f8f5df4UL,
        0x0000000d833a0893UL,
        0x00000002a36fdd7cUL,
        0x0000000d6a857cf2UL,
        0x00000008829bc35cUL,
        0x00000005e50d79bcUL,
        0x0000000fbb8035e4UL,
        0x0000000c1a95bebfUL,
        0x0000000036b0baf8UL,
        0x0000000e0da964eaUL,
        0x0000000b6483689bUL,
        0x00000007c8e2f4c1UL,
        0x00000005b856a23bUL,
        0x00000002fc183995UL,
        0x0000000e914b6d70UL,
        0x0000000b31041969UL,
        0x00000001bb478493UL,
        0x0000000063e2b456UL,
        0x0000000f2a082b9cUL,
        0x00000008e5e646eaUL,
        0x000000008172f8f6UL,
        0x00000000dacd923eUL,
        0x0000000e5dcf0e2eUL,
        0x0000000bf9446baeUL,
        0x00000004822d50d1UL,
        0x000000026e710bf5UL,
        0x0000000b90ba2a24UL,
        0x0000000f3b25aa73UL,
        0x0000000809ad589bUL,
        0x000000094cc1e254UL,
        0x00000005334a3adbUL,
        0x0000000592886b2fUL,
        0x0000000bf64704aaUL,
        0x0000000566dbf24cUL,
        0x000000072203e692UL,
        0x000000064e61e809UL,
        0x0000000d7259aad6UL,
        0x00000007b924aedcUL,
        0x00000002df2184e8UL,
        0x0000000353d1eca7UL,
        0x0000000fce30d7ceUL,
        0x0000000f7b0f436eUL,
        0x000000057e8d8f68UL,
        0x00000008c79e60dbUL,
        0x00000009c8362b2bUL,
        0x000000063a5804f2UL,
        0x00000009298353dcUL,
        0x00000006f98a71c8UL,
        0x0000000a5731f693UL,
        0x000000021ca5c870UL,
        0x00000001c2107fd3UL,
        0x00000006181f6c39UL,
        0x000000019e574304UL,
        0x0000000329937606UL,
        0x0000000043d5c70dUL,
        0x00000009b18ff162UL,
        0x00000008e2ccfebfUL,
        0x000000072b7b9b54UL,
        0x00000009b71f4f3cUL,
        0x0000000935d7393eUL,
        0x000000065938881aUL,
        0x00000006a5bd6f2dUL,
        0x0000000a19783306UL,
        0x0000000e6472f4d7UL,
        0x000000081163df5aUL,
        0x0000000a838e1cbdUL,
        0x0000000982748477UL,
        0x0000000050c54febUL,
        0x00000000d82fbb58UL,
        0x00000002c4c72799UL,
        0x000000097d259ad6UL,
        0x000000022d9a43edUL,
        0x0000000fdb162a9fUL,
        0x00000000cb4a727dUL,
        0x00000004fae2e371UL,
        0x0000000535b5be8bUL,
        0x000000048795908aUL,
        0x0000000ce7c18962UL,
        0x00000004ea154d80UL,
        0x000000050c064889UL,
        0x00000008d97fc75dUL,
        0x0000000c8bd9ec61UL,
        0x000000083ee8e8bbUL,
        0x0000000c8431419aUL,
        0x00000001aa78079dUL,
        0x00000008111aa4a5UL,
        0x0000000dfa3a69feUL,
        0x000000051630d83fUL,
        0x00000002d930fb3fUL,
        0x00000002133116e5UL,
        0x0000000ae5395522UL,
        0x0000000bc07a4e8aUL,
        0x000000057bf08ba0UL,
        0x00000006cb18036aUL,
        0x0000000f0e2e4b75UL,
        0x00000003eb692b6fUL,
        0x0000000d8178a3faUL,
        0x0000000238cce6a6UL,
        0x0000000e97d5cdd7UL,
        0x0000000fe10d8d5eUL,
        0x0000000b39584a1dUL,
        0x0000000ca03536fdUL,
        0x0000000aa61f3998UL,
        0x000000072ff23ec2UL,
        0x000000015aa7d770UL,
        0x000000057a3a1282UL,
        0x0000000d1f3902dcUL,
        0x00000006554c9388UL,
        0x0000000fd01283c7UL,
        0x0000000e8baa42c5UL,
        0x000000072cee6adfUL,
        0x0000000f6614b3faUL,
        0x000000095c3778a2UL,
        0x00000007da4cea7aUL,
        0x0000000d18a5912cUL,
        0x0000000d116426e5UL,
        0x000000027c17bc1cUL,
        0x0000000b95b53bc1UL,
        0x0000000c8f937a05UL,
        0x0000000ed220c9bdUL,
        0x00000000c97d72abUL,
        0x00000008fb1217aeUL,
        0x000000025ca8a5a1UL,
        0x0000000b261b871bUL,
        0x00000001bef0a056UL,
        0x0000000806a51179UL,
        0x0000000eed249145UL,
        0x00000003f82aecebUL,
        0x0000000cc56e9acfUL,
        0x00000002e78d01ebUL,
        0x0000000102cee17fUL,
        0x000000037caad3d5UL,
        0x000000016ac5b1eeUL,
        0x00000002af164eceUL,
        0x0000000d4cd81dc9UL,
        0x000000012263a7e7UL,
        0x000000057ac7d117UL,
        0x00000009391d9740UL,
        0x00000007aedaa77fUL,
        0x00000009675a3c72UL,
        0x0000000277f25191UL,
        0x0000000ebb6e64b9UL,
        0x00000007ad3ef747UL,
        0x000000012759b181UL,
        0x0000000948257d4dUL,
        0x0000000b63a850f6UL,
        0x00000003a52a8f75UL,
        0x00000004a019532cUL,
        0x0000000a021a7529UL,
        0x0000000cc661876dUL,
        0x00000004085afd05UL,
        0x0000000e7048e089UL,
        0x00000003f979cdc6UL,
        0x0000000d9da9071bUL,
        0x0000000ed2fc5b68UL,
        0x000000079d64c3a1UL,
        0x0000000fd44e2361UL,
        0x00000008eea46a74UL,
        0x000000042233b9c2UL,
        0x0000000ae4d1765dUL,
        0x00000007303a094cUL,
        0x00000002d7033abeUL,
        0x00000003dcc2b0b4UL,
        0x00000000f0967d09UL,
        0x000000006f0cd7deUL,
        0x000000009807aca0UL,
        0x00000003a295cad3UL,
        0x00000002b106b202UL,
        0x00000003f38a828eUL,
        0x000000078af46596UL,
        0x0000000bda2dc713UL,
        0x00000009a8c8c9d9UL,
        0x00000006a0f2ddceUL,
        0x0000000a76af6fe2UL,
        0x0000000086f66fa4UL,
        0x0000000d52d63f8dUL,
        0x000000089f7a6e73UL,
        0x0000000cc6b23362UL,
        0x0000000b4ebf3c39UL,
        0x0000000564f300faUL,
        0x0000000e8de3a706UL,
        0x000000079a033b61UL,
        0x0000000765e160c5UL,
        0x0000000a266a4f85UL,
        0x0000000a68c38c24UL,
        0x0000000dca0711fbUL,
        0x000000085fba85baUL,
        0x000000037a207b46UL,
        0x0000000158fcc4d0UL,
        0x00000000569d79b3UL,
        0x00000007b1a25555UL,
        0x0000000a8ae22468UL,
        0x00000007c592bdfdUL,
        0x00000000c59a5f66UL,
        0x0000000b1115daa3UL,
        0x0000000f17c87177UL,
        0x00000006769d766bUL,
        0x00000002b637356dUL,
        0x000000013d8685acUL,
        0x0000000f24cb6ec0UL,
        0x00000000bd0b56d1UL,
        0x000000042ff0e26dUL,
        0x0000000b41609267UL,
        0x000000096f9518afUL,
        0x0000000c56f96636UL,
        0x00000004a8e10349UL,
        0x0000000863512171UL,
        0x0000000ea455d86cUL,
        0x0000000bd0e25279UL,
        0x0000000e65e3f761UL,
        0x000000036c84a922UL,
        0x000000085fd1b38fUL,
        0x0000000657c91539UL,
        0x000000015033fe04UL,
        0x000000009051c921UL,
        0x0000000ab27d80d8UL,
        0x0000000f92f7d0a1UL,
        0x00000008eb6bb737UL,
        0x000000010b5b0f63UL,
        0x00000006c9c7ad63UL,
        0x0000000f66fe70aeUL,
        0x0000000ca579bd92UL,
        0x0000000956198e4dUL,
        0x000000029e4405e5UL,
        0x0000000e44eb885cUL,
        0x000000041612456cUL,
        0x0000000ea45e0abfUL,
        0x0000000d326529bdUL,
        0x00000007b2c33cefUL,
        0x000000080bc9b558UL,
        0x00000007169b9740UL,
        0x0000000c37f99209UL,
        0x000000031ff6dab9UL,
        0x0000000c795190edUL,
        0x0000000a7636e95fUL,
        0x00000009df075841UL,
        0x000000055a083932UL,
        0x0000000a7cbdf630UL,
        0x0000000409ea4ef0UL,
        0x000000092a1991b6UL,
        0x00000004b078dee9UL,
        0x0000000ae18ce9e4UL,
        0x00000005a6e1ef35UL,
        0x00000001a403bd59UL,
        0x000000031ea70a83UL,
        0x00000002bc3c4f3aUL,
        0x00000005c921b3cbUL,
        0x0000000042da05c5UL,
        0x00000001f667d16bUL,
        0x0000000416a368cfUL,
        0x0000000fbc0a7a3bUL,
        0x00000009419f0c7cUL,
        0x000000081be2fa03UL,
        0x000000034e2c172fUL,
        0x000000028648d8aeUL,
        0x0000000c7acbb885UL,
        0x000000045f31eb6aUL,
        0x0000000d1cfc0a7bUL,
        0x000000042c4d260dUL,
        0x0000000cf6584097UL,
        0x000000094b132b14UL,
        0x00000003c5c5df75UL,
        0x00000008ae596fefUL,
        0x0000000aea8054ebUL,
        0x00000000ae9cc573UL,
        0x0000000496fb731bUL,
        0x0000000ebf105662UL,
        0x0000000af9c83a37UL,
        0x0000000c0d64cd6bUL,
        0x00000007b608159aUL,
        0x0000000e74431642UL,
        0x0000000d6fb9d900UL,
        0x0000000291e99de0UL,
        0x000000010500ba9aUL,
        0x00000005cd05d037UL,
        0x0000000a87254fb2UL,
        0x00000009d7824a37UL,
        0x00000008b2c7b47cUL,
        0x000000030c788145UL,
        0x00000002f4e5a8beUL,
        0x0000000badb884daUL,
        0x0000000026e0d5c9UL,
        0x00000006fdbaa32eUL,
        0x000000034758eb31UL,
        0x0000000565cd1b4fUL,
        0x00000002bfd90fb0UL,
        0x0000000093052a6bUL,
        0x0000000d3c13c4b9UL,
        0x00000002daea43bfUL,
        0x0000000a279762bcUL,
        0x0000000f1bd9f22cUL,
        0x00000004b7fec94fUL,
        0x0000000545761d5aUL,
        0x00000007327df411UL,
        0x00000001b52a442eUL,
        0x000000049b0ce108UL,
        0x000000024c764bc8UL,
        0x0000000374563045UL,
        0x0000000a3e8f91c6UL,
        0x00000000e6bd2241UL,
        0x0000000e0e52ee3cUL,
        0x000000007e8e3caaUL,
        0x000000096c2b7372UL,
        0x000000033acbdfdaUL,
        0x0000000b15d91e54UL,
        0x0000000464759ac1UL,
        0x00000006886a1998UL,
        0x000000057f5d3958UL,
        0x00000005a1f5c1f5UL,
        0x00000000b58158adUL,
        0x0000000e712053fbUL,
        0x00000005352ddb25UL,
        0x0000000414b98ea0UL,
        0x000000074f89f546UL,
        0x000000038a56b3c3UL,
        0x000000038db0dc17UL,
        0x0000000aa016a755UL,
        0x0000000dc72366f5UL,
        0x00000000cee93d75UL,
        0x0000000b2fe7a56bUL,
        0x0000000a847ed390UL,
        0x00000008713ef88cUL,
        0x0000000a217cc861UL,
        0x00000008bca25d7bUL,
        0x0000000455526818UL,
        0x0000000ea3a7a180UL,
        0x0000000a9536e5e0UL,
        0x00000009b64a1975UL,
        0x00000005bfc756bcUL,
        0x0000000046aa169bUL,
        0x000000053a17f76fUL,
        0x00000004d6815274UL,
        0x0000000cca9cf3f6UL,
        0x00000004013fcb8bUL,
        0x00000003d26cdfa5UL,
        0x00000005786231f7UL,
        0x00000007d4ab09abUL,
        0x0000000960b5ffbcUL,
        0x00000008914df0d4UL,
        0x00000002fc6f2213UL,
        0x0000000ac235637eUL,
        0x0000000151b28ed3UL,
        0x000000046f79b6dbUL,
        0x00000001382e0c9fUL,
        0x000000053abf983aUL,
        0x0000000383c47adeUL,
        0x00000003fcf88978UL,
        0x0000000eb9079df7UL,
        0x000000009af0714dUL,
        0x0000000da19d1bb7UL,
        0x00000009a02749f8UL,
        0x00000001c62dab9bUL,
        0x00000001a137e44bUL,
        0x00000002867718c7UL,
        0x000000035815525bUL,
        0x00000007cd35c550UL,
        0x00000002164f73a0UL,
        0x0000000e8b772fe0UL,
    }
};

#undef N_BITS
////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "artoolkit"
////////////////////////////////////////////////////////////////////////////////////////////////////

#define N_BITS 36

static const uint32_t tagArtoolkit_bit_x[N_BITS] = {1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3};
static const uint32_t tagArtoolkit_bit_y[N_BITS] = { 1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4};

const apriltag_family_t tagArtoolkit = {
    .ncodes = 512,
    .width_at_border = 8,
    .total_width = 10,
    .d = 7,
    .reversed_border = false,
    .nbits = N_BITS,
    .bit_x = tagArtoolkit_bit_x,
    .bit_y = tagArtoolkit_bit_y,
    .h = 7,
    .codes = {
        0x00000006fd8381c9UL,
        0x00000006dc8185c9UL,
        0x00000006bd9281c1UL,
        0x000000069c9085c1UL,
        0x00000006fd870159UL,
        0x00000006dc850559UL,
        0x00000006bd960151UL,
        0x000000069c940551UL,
        0x00000006f103c1c9UL,
        0x00000006d001c5c9UL,
        0x00000006b112c1c1UL,
        0x000000069010c5c1UL,
        0x00000006f1074159UL,
        0x00000006d0054559UL,
        0x00000006b1164151UL,
        0x0000000690144551UL,
        0x000000067d8ba1cbUL,
        0x000000065c89a5cbUL,
        0x000000063d9aa1c3UL,
        0x000000061c98a5c3UL,
        0x000000067d8f215bUL,
        0x000000065c8d255bUL,
        0x000000063d9e2153UL,
        0x000000061c9c2553UL,
        0x00000006710be1cbUL,
        0x000000065009e5cbUL,
        0x00000006311ae1c3UL,
        0x000000061018e5c3UL,
        0x00000006710f615bUL,
        0x00000006500d655bUL,
        0x00000006311e6153UL,
        0x00000006101c6553UL,
        0x00000007fd8382e9UL,
        0x00000007dc8186e9UL,
        0x00000007bd9282e1UL,
        0x000000079c9086e1UL,
        0x00000007fd870279UL,
        0x00000007dc850679UL,
        0x00000007bd960271UL,
        0x000000079c940671UL,
        0x00000007f103c2e9UL,
        0x00000007d001c6e9UL,
        0x00000007b112c2e1UL,
        0x000000079010c6e1UL,
        0x00000007f1074279UL,
        0x00000007d0054679UL,
        0x00000007b1164271UL,
        0x0000000790144671UL,
        0x000000077d8ba2ebUL,
        0x000000075c89a6ebUL,
        0x000000073d9aa2e3UL,
        0x000000071c98a6e3UL,
        0x000000077d8f227bUL,
        0x000000075c8d267bUL,
        0x000000073d9e2273UL,
        0x000000071c9c2673UL,
        0x00000007710be2ebUL,
        0x000000075009e6ebUL,
        0x00000007311ae2e3UL,
        0x000000071018e6e3UL,
        0x00000007710f627bUL,
        0x00000007500d667bUL,
        0x00000007311e6273UL,
        0x00000007101c6673UL,
        0x00000004ffc381c8UL,
        0x00000004dec185c8UL,
        0x00000004bfd281c0UL,
        0x000000049ed085c0UL,
        0x00000004ffc70158UL,
        0x00000004dec50558UL,
        0x00000004bfd60150UL,
        0x000000049ed40550UL,
        0x00000004f343c1c8UL,
        0x00000004d241c5c8UL,
        0x00000004b352c1c0UL,
        0x000000049250c5c0UL,
        0x00000004f3474158UL,
        0x00000004d2454558UL,
        0x00000004b3564150UL,
        0x0000000492544550UL,
        0x000000047fcba1caUL,
        0x000000045ec9a5caUL,
        0x000000043fdaa1c2UL,
        0x000000041ed8a5c2UL,
        0x000000047fcf215aUL,
        0x000000045ecd255aUL,
        0x000000043fde2152UL,
        0x000000041edc2552UL,
        0x00000004734be1caUL,
        0x000000045249e5caUL,
        0x00000004335ae1c2UL,
        0x000000041258e5c2UL,
        0x00000004734f615aUL,
        0x00000004524d655aUL,
        0x00000004335e6152UL,
        0x00000004125c6552UL,
        0x00000005ffc382e8UL,
        0x00000005dec186e8UL,
        0x00000005bfd282e0UL,
        0x000000059ed086e0UL,
        0x00000005ffc70278UL,
        0x00000005dec50678UL,
        0x00000005bfd60270UL,
        0x000000059ed40670UL,
        0x00000005f343c2e8UL,
        0x00000005d241c6e8UL,
        0x00000005b352c2e0UL,
        0x000000059250c6e0UL,
        0x00000005f3474278UL,
        0x00000005d2454678UL,
        0x00000005b3564270UL,
        0x0000000592544670UL,
        0x000000057fcba2eaUL,
        0x000000055ec9a6eaUL,
        0x000000053fdaa2e2UL,
        0x000000051ed8a6e2UL,
        0x000000057fcf227aUL,
        0x000000055ecd267aUL,
        0x000000053fde2272UL,
        0x000000051edc2672UL,
        0x00000005734be2eaUL,
        0x000000055249e6eaUL,
        0x00000005335ae2e2UL,
        0x000000051258e6e2UL,
        0x00000005734f627aUL,
        0x00000005524d667aUL,
        0x00000005335e6272UL,
        0x00000005125c6672UL,
        0x00000002fda391cdUL,
        0x00000002dca195cdUL,
        0x00000002bdb291c5UL,
        0x000000029cb095c5UL,
        0x00000002fda7115dUL,
        0x00000002dca5155dUL,
        0x00000002bdb61155UL,
        0x000000029cb41555UL,
        0x00000002f123d1cdUL,
        0x00000002d021d5cdUL,
        0x00000002b132d1c5UL,
        0x000000029030d5c5UL,
        0x00000002f127515dUL,
        0x00000002d025555dUL,
        0x00000002b1365155UL,
        0x0000000290345555UL,
        0x000000027dabb1cfUL,
        0x000000025ca9b5cfUL,
        0x000000023dbab1c7UL,
        0x000000021cb8b5c7UL,
        0x000000027daf315fUL,
        0x000000025cad355fUL,
        0x000000023dbe3157UL,
        0x000000021cbc3557UL,
        0x00000002712bf1cfUL,
        0x000000025029f5cfUL,
        0x00000002313af1c7UL,
        0x000000021038f5c7UL,
        0x00000002712f715fUL,
        0x00000002502d755fUL,
        0x00000002313e7157UL,
        0x00000002103c7557UL,
        0x00000003fda392edUL,
        0x00000003dca196edUL,
        0x00000003bdb292e5UL,
        0x000000039cb096e5UL,
        0x00000003fda7127dUL,
        0x00000003dca5167dUL,
        0x00000003bdb61275UL,
        0x000000039cb41675UL,
        0x00000003f123d2edUL,
        0x00000003d021d6edUL,
        0x00000003b132d2e5UL,
        0x000000039030d6e5UL,
        0x00000003f127527dUL,
        0x00000003d025567dUL,
        0x00000003b1365275UL,
        0x0000000390345675UL,
        0x000000037dabb2efUL,
        0x000000035ca9b6efUL,
        0x000000033dbab2e7UL,
        0x000000031cb8b6e7UL,
        0x000000037daf327fUL,
        0x000000035cad367fUL,
        0x000000033dbe3277UL,
        0x000000031cbc3677UL,
        0x00000003712bf2efUL,
        0x000000035029f6efUL,
        0x00000003313af2e7UL,
        0x000000031038f6e7UL,
        0x00000003712f727fUL,
        0x00000003502d767fUL,
        0x00000003313e7277UL,
        0x00000003103c7677UL,
        0x00000000ffe391ccUL,
        0x00000000dee195ccUL,
        0x00000000bff291c4UL,
        0x000000009ef095c4UL,
        0x00000000ffe7115cUL,
        0x00000000dee5155cUL,
        0x00000000bff61154UL,
        0x000000009ef41554UL,
        0x00000000f363d1ccUL,
        0x00000000d261d5ccUL,
        0x00000000b372d1c4UL,
        0x000000009270d5c4UL,
        0x00000000f367515cUL,
        0x00000000d265555cUL,
        0x00000000b3765154UL,
        0x0000000092745554UL,
        0x000000007febb1ceUL,
        0x000000005ee9b5ceUL,
        0x000000003ffab1c6UL,
        0x000000001ef8b5c6UL,
        0x000000007fef315eUL,
        0x000000005eed355eUL,
        0x000000003ffe3156UL,
        0x000000001efc3556UL,
        0x00000000736bf1ceUL,
        0x000000005269f5ceUL,
        0x00000000337af1c6UL,
        0x000000001278f5c6UL,
        0x00000000736f715eUL,
        0x00000000526d755eUL,
        0x00000000337e7156UL,
        0x00000000127c7556UL,
        0x00000001ffe392ecUL,
        0x00000001dee196ecUL,
        0x00000001bff292e4UL,
        0x000000019ef096e4UL,
        0x00000001ffe7127cUL,
        0x00000001dee5167cUL,
        0x00000001bff61274UL,
        0x000000019ef41674UL,
        0x00000001f363d2ecUL,
        0x00000001d261d6ecUL,
        0x00000001b372d2e4UL,
        0x000000019270d6e4UL,
        0x00000001f367527cUL,
        0x00000001d265567cUL,
        0x00000001b3765274UL,
        0x0000000192745674UL,
        0x000000017febb2eeUL,
        0x000000015ee9b6eeUL,
        0x000000013ffab2e6UL,
        0x000000011ef8b6e6UL,
        0x000000017fef327eUL,
        0x000000015eed367eUL,
        0x000000013ffe3276UL,
        0x000000011efc3676UL,
        0x00000001736bf2eeUL,
        0x000000015269f6eeUL,
        0x00000001337af2e6UL,
        0x000000011278f6e6UL,
        0x00000001736f727eUL,
        0x00000001526d767eUL,
        0x00000001337e7276UL,
        0x00000001127c7676UL,
        0x0000000eed838989UL,
        0x0000000ecc818d89UL,
        0x0000000ead928981UL,
        0x0000000e8c908d81UL,
        0x0000000eed870919UL,
        0x0000000ecc850d19UL,
        0x0000000ead960911UL,
        0x0000000e8c940d11UL,
        0x0000000ee103c989UL,
        0x0000000ec001cd89UL,
        0x0000000ea112c981UL,
        0x0000000e8010cd81UL,
        0x0000000ee1074919UL,
        0x0000000ec0054d19UL,
        0x0000000ea1164911UL,
        0x0000000e80144d11UL,
        0x0000000e6d8ba98bUL,
        0x0000000e4c89ad8bUL,
        0x0000000e2d9aa983UL,
        0x0000000e0c98ad83UL,
        0x0000000e6d8f291bUL,
        0x0000000e4c8d2d1bUL,
        0x0000000e2d9e2913UL,
        0x0000000e0c9c2d13UL,
        0x0000000e610be98bUL,
        0x0000000e4009ed8bUL,
        0x0000000e211ae983UL,
        0x0000000e0018ed83UL,
        0x0000000e610f691bUL,
        0x0000000e400d6d1bUL,
        0x0000000e211e6913UL,
        0x0000000e001c6d13UL,
        0x0000000fed838aa9UL,
        0x0000000fcc818ea9UL,
        0x0000000fad928aa1UL,
        0x0000000f8c908ea1UL,
        0x0000000fed870a39UL,
        0x0000000fcc850e39UL,
        0x0000000fad960a31UL,
        0x0000000f8c940e31UL,
        0x0000000fe103caa9UL,
        0x0000000fc001cea9UL,
        0x0000000fa112caa1UL,
        0x0000000f8010cea1UL,
        0x0000000fe1074a39UL,
        0x0000000fc0054e39UL,
        0x0000000fa1164a31UL,
        0x0000000f80144e31UL,
        0x0000000f6d8baaabUL,
        0x0000000f4c89aeabUL,
        0x0000000f2d9aaaa3UL,
        0x0000000f0c98aea3UL,
        0x0000000f6d8f2a3bUL,
        0x0000000f4c8d2e3bUL,
        0x0000000f2d9e2a33UL,
        0x0000000f0c9c2e33UL,
        0x0000000f610beaabUL,
        0x0000000f4009eeabUL,
        0x0000000f211aeaa3UL,
        0x0000000f0018eea3UL,
        0x0000000f610f6a3bUL,
        0x0000000f400d6e3bUL,
        0x0000000f211e6a33UL,
        0x0000000f001c6e33UL,
        0x0000000cefc38988UL,
        0x0000000ccec18d88UL,
        0x0000000cafd28980UL,
        0x0000000c8ed08d80UL,
        0x0000000cefc70918UL,
        0x0000000ccec50d18UL,
        0x0000000cafd60910UL,
        0x0000000c8ed40d10UL,
        0x0000000ce343c988UL,
        0x0000000cc241cd88UL,
        0x0000000ca352c980UL,
        0x0000000c8250cd80UL,
        0x0000000ce3474918UL,
        0x0000000cc2454d18UL,
        0x0000000ca3564910UL,
        0x0000000c82544d10UL,
        0x0000000c6fcba98aUL,
        0x0000000c4ec9ad8aUL,
        0x0000000c2fdaa982UL,
        0x0000000c0ed8ad82UL,
        0x0000000c6fcf291aUL,
        0x0000000c4ecd2d1aUL,
        0x0000000c2fde2912UL,
        0x0000000c0edc2d12UL,
        0x0000000c634be98aUL,
        0x0000000c4249ed8aUL,
        0x0000000c235ae982UL,
        0x0000000c0258ed82UL,
        0x0000000c634f691aUL,
        0x0000000c424d6d1aUL,
        0x0000000c235e6912UL,
        0x0000000c025c6d12UL,
        0x0000000defc38aa8UL,
        0x0000000dcec18ea8UL,
        0x0000000dafd28aa0UL,
        0x0000000d8ed08ea0UL,
        0x0000000defc70a38UL,
        0x0000000dcec50e38UL,
        0x0000000dafd60a30UL,
        0x0000000d8ed40e30UL,
        0x0000000de343caa8UL,
        0x0000000dc241cea8UL,
        0x0000000da352caa0UL,
        0x0000000d8250cea0UL,
        0x0000000de3474a38UL,
        0x0000000dc2454e38UL,
        0x0000000da3564a30UL,
        0x0000000d82544e30UL,
        0x0000000d6fcbaaaaUL,
        0x0000000d4ec9aeaaUL,
        0x0000000d2fdaaaa2UL,
        0x0000000d0ed8aea2UL,
        0x0000000d6fcf2a3aUL,
        0x0000000d4ecd2e3aUL,
        0x0000000d2fde2a32UL,
        0x0000000d0edc2e32UL,
        0x0000000d634beaaaUL,
        0x0000000d4249eeaaUL,
        0x0000000d235aeaa2UL,
        0x0000000d0258eea2UL,
        0x0000000d634f6a3aUL,
        0x0000000d424d6e3aUL,
        0x0000000d235e6a32UL,
        0x0000000d025c6e32UL,
        0x0000000aeda3998dUL,
        0x0000000acca19d8dUL,
        0x0000000aadb29985UL,
        0x0000000a8cb09d85UL,
        0x0000000aeda7191dUL,
        0x0000000acca51d1dUL,
        0x0000000aadb61915UL,
        0x0000000a8cb41d15UL,
        0x0000000ae123d98dUL,
        0x0000000ac021dd8dUL,
        0x0000000aa132d985UL,
        0x0000000a8030dd85UL,
        0x0000000ae127591dUL,
        0x0000000ac0255d1dUL,
        0x0000000aa1365915UL,
        0x0000000a80345d15UL,
        0x0000000a6dabb98fUL,
        0x0000000a4ca9bd8fUL,
        0x0000000a2dbab987UL,
        0x0000000a0cb8bd87UL,
        0x0000000a6daf391fUL,
        0x0000000a4cad3d1fUL,
        0x0000000a2dbe3917UL,
        0x0000000a0cbc3d17UL,
        0x0000000a612bf98fUL,
        0x0000000a4029fd8fUL,
        0x0000000a213af987UL,
        0x0000000a0038fd87UL,
        0x0000000a612f791fUL,
        0x0000000a402d7d1fUL,
        0x0000000a213e7917UL,
        0x0000000a003c7d17UL,
        0x0000000beda39aadUL,
        0x0000000bcca19eadUL,
        0x0000000badb29aa5UL,
        0x0000000b8cb09ea5UL,
        0x0000000beda71a3dUL,
        0x0000000bcca51e3dUL,
        0x0000000badb61a35UL,
        0x0000000b8cb41e35UL,
        0x0000000be123daadUL,
        0x0000000bc021deadUL,
        0x0000000ba132daa5UL,
        0x0000000b8030dea5UL,
        0x0000000be1275a3dUL,
        0x0000000bc0255e3dUL,
        0x0000000ba1365a35UL,
        0x0000000b80345e35UL,
        0x0000000b6dabbaafUL,
        0x0000000b4ca9beafUL,
        0x0000000b2dbabaa7UL,
        0x0000000b0cb8bea7UL,
        0x0000000b6daf3a3fUL,
        0x0000000b4cad3e3fUL,
        0x0000000b2dbe3a37UL,
        0x0000000b0cbc3e37UL,
        0x0000000b612bfaafUL,
        0x0000000b4029feafUL,
        0x0000000b213afaa7UL,
        0x0000000b0038fea7UL,
        0x0000000b612f7a3fUL,
        0x0000000b402d7e3fUL,
        0x0000000b213e7a37UL,
        0x0000000b003c7e37UL,
        0x00000008efe3998cUL,
        0x00000008cee19d8cUL,
        0x00000008aff29984UL,
        0x000000088ef09d84UL,
        0x00000008efe7191cUL,
        0x00000008cee51d1cUL,
        0x00000008aff61914UL,
        0x000000088ef41d14UL,
        0x00000008e363d98cUL,
        0x00000008c261dd8cUL,
        0x00000008a372d984UL,
        0x000000088270dd84UL,
        0x00000008e367591cUL,
        0x00000008c2655d1cUL,
        0x00000008a3765914UL,
        0x0000000882745d14UL,
        0x000000086febb98eUL,
        0x000000084ee9bd8eUL,
        0x000000082ffab986UL,
        0x000000080ef8bd86UL,
        0x000000086fef391eUL,
        0x000000084eed3d1eUL,
        0x000000082ffe3916UL,
        0x000000080efc3d16UL,
        0x00000008636bf98eUL,
        0x000000084269fd8eUL,
        0x00000008237af986UL,
        0x000000080278fd86UL,
        0x00000008636f791eUL,
        0x00000008426d7d1eUL,
        0x00000008237e7916UL,
        0x00000008027c7d16UL,
        0x00000009efe39aacUL,
        0x00000009cee19eacUL,
        0x00000009aff29aa4UL,
        0x000000098ef09ea4UL,
        0x00000009efe71a3cUL,
        0x00000009cee51e3cUL,
        0x00000009aff61a34UL,
        0x000000098ef41e34UL,
        0x00000009e363daacUL,
        0x00000009c261deacUL,
        0x00000009a372daa4UL,
        0x000000098270dea4UL,
        0x00000009e3675a3cUL,
        0x00000009c2655e3cUL,
        0x00000009a3765a34UL,
        0x0000000982745e34UL,
        0x000000096febbaaeUL,
        0x000000094ee9beaeUL,
        0x000000092ffabaa6UL,
        0x000000090ef8bea6UL,
        0x000000096fef3a3eUL,
        0x000000094eed3e3eUL,
        0x000000092ffe3a36UL,
        0x000000090efc3e36UL,
        0x00000009636bfaaeUL,
        0x000000094269feaeUL,
        0x00000009237afaa6UL,
        0x000000090278fea6UL,
        0x00000009636f7a3eUL,
        0x00000009426d7e3eUL,
        0x00000009237e7a36UL,
        0x00000009027c7e36UL,
    }
};

#undef N_BITS
////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "union_find.h"
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct unionfind unionfind_t;

struct unionfind
{
    uint32_t maxid;
    struct ufrec *data;
};

struct ufrec
{
#ifdef IMLIB_ENABLE_HIGH_RES_APRILTAGS
    // the parent of this node. If a node's parent is its own index,
    // then it is a root.
    uint32_t parent;

    // for the root of a connected component, the number of components
    // connected to it. For intermediate values, it's not meaningful.
    uint32_t size;
#else
    uint16_t parent;
    uint16_t size;
#endif
};

static inline unionfind_t *unionfind_create(uint32_t maxid)
{
    unionfind_t *uf = (unionfind_t*) fb_alloc(sizeof (unionfind_t), FB_ALLOC_NO_HINT);
    uf->maxid = maxid;
    uf->data = (struct ufrec*) fb_alloc((maxid+1) * sizeof(struct ufrec), FB_ALLOC_NO_HINT);
    for (int i = 0; i <= maxid; i++) {
        uf->data[i].size = 1;
        uf->data[i].parent = i;
    }
    return uf;
}

static inline void unionfind_destroy(unionfind_t *uf)
{
    fb_free();  //uf->data
    fb_free();  //uf
}

/*
static inline uint32_t unionfind_get_representative(unionfind_t *uf, uint32_t id)
{
    // base case: a node is its own parent
    if (uf->data[id].parent == id)
        return id;

    // otherwise, recurse
    uint32_t root = unionfind_get_representative(uf, uf->data[id].parent);

    // short circuit the path. [XXX This write prevents tail recursion]
    uf->data[id].parent = root;

    return root;
}
*/

// this one seems to be every-so-slightly faster than the recursive
// version above.
static inline uint32_t unionfind_get_representative(unionfind_t *uf, uint32_t id)
{
    uint32_t root = id;

    // chase down the root
    while (uf->data[root].parent != root) {
        root = uf->data[root].parent;
    }

    // go back and collapse the tree.
    while (uf->data[id].parent != root) {
        uint32_t tmp = uf->data[id].parent;
        uf->data[id].parent = root;
        id = tmp;
    }

    return root;
}

static inline uint32_t unionfind_get_set_size(unionfind_t *uf, uint32_t id)
{
    uint32_t repid = unionfind_get_representative(uf, id);
    return uf->data[repid].size;
}

static inline uint32_t unionfind_connect(unionfind_t *uf, uint32_t aid, uint32_t bid)
{
    uint32_t aroot = unionfind_get_representative(uf, aid);
    uint32_t broot = unionfind_get_representative(uf, bid);

    if (aroot == broot)
        return aroot;

    // we don't perform "union by rank", but we perform a similar
    // operation (but probably without the same asymptotic guarantee):
    // We join trees based on the number of *elements* (as opposed to
    // rank) contained within each tree. I.e., we use size as a proxy
    // for rank.  In my testing, it's often *faster* to use size than
    // rank, perhaps because the rank of the tree isn't that critical
    // if there are very few nodes in it.
    uint32_t asize = uf->data[aroot].size;
    uint32_t bsize = uf->data[broot].size;

    // optimization idea: We could shortcut some or all of the tree
    // that is grafted onto the other tree. Pro: those nodes were just
    // read and so are probably in cache. Con: it might end up being
    // wasted effort -- the tree might be grafted onto another tree in
    // a moment!
    if (asize > bsize) {
        uf->data[broot].parent = aroot;
        uf->data[aroot].size += bsize;
        return aroot;
    } else {
        uf->data[aroot].parent = broot;
        uf->data[broot].size += asize;
        return broot;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "union_find.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "apriltag_quad_thresh.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

// limitation: image size must be <32768 in width and height. This is
// because we use a fixed-point 16 bit integer representation with one
// fractional bit.

static inline uint32_t u64hash_2(uint64_t x) {
    return (2654435761 * x) >> 32;
    return (uint32_t) x;
}

struct uint32_zarray_entry
{
    uint32_t id;
    zarray_t *cluster;

    struct uint32_zarray_entry *next;
};

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

struct pt
{
    // Note: these represent 2*actual value.
    uint16_t x, y;
    float theta;
    int16_t gx, gy;
};

struct remove_vertex
{
    int i;           // which vertex to remove?
    int left, right; // left vertex, right vertex

    float err;
};

struct segment
{
    int is_vertex;

    // always greater than zero, but right can be > size, which denotes
    // a wrap around back to the beginning of the points. and left < right.
    int left, right;
};

struct line_fit_pt
{
    float Mx, My;
    float Mxx, Myy, Mxy;
    float W; // total weight
};

static inline void ptsort(struct pt *pts, int sz)
{
#define MAYBE_SWAP(arr,apos,bpos)                                   \
    if (arr[apos].theta > arr[bpos].theta) {                        \
        tmp = arr[apos]; arr[apos] = arr[bpos]; arr[bpos] = tmp;    \
    };

    if (sz <= 1)
        return;

    if (sz == 2) {
        struct pt tmp;
        MAYBE_SWAP(pts, 0, 1);
        return;
    }

    // NB: Using less-branch-intensive sorting networks here on the
    // hunch that it's better for performance.
    if (sz == 3) { // 3 element bubble sort is optimal
        struct pt tmp;
        MAYBE_SWAP(pts, 0, 1);
        MAYBE_SWAP(pts, 1, 2);
        MAYBE_SWAP(pts, 0, 1);
        return;
    }

    if (sz == 4) { // 4 element optimal sorting network.
        struct pt tmp;
        MAYBE_SWAP(pts, 0, 1); // sort each half, like a merge sort
        MAYBE_SWAP(pts, 2, 3);
        MAYBE_SWAP(pts, 0, 2); // minimum value is now at 0.
        MAYBE_SWAP(pts, 1, 3); // maximum value is now at end.
        MAYBE_SWAP(pts, 1, 2); // that only leaves the middle two.
        return;
    }

    if (sz == 5) {
        // this 9-step swap is optimal for a sorting network, but two
        // steps slower than a generic sort.
        struct pt tmp;
        MAYBE_SWAP(pts, 0, 1); // sort each half (3+2), like a merge sort
        MAYBE_SWAP(pts, 3, 4);
        MAYBE_SWAP(pts, 1, 2);
        MAYBE_SWAP(pts, 0, 1);
        MAYBE_SWAP(pts, 0, 3); // minimum element now at 0
        MAYBE_SWAP(pts, 2, 4); // maximum element now at end
        MAYBE_SWAP(pts, 1, 2); // now resort the three elements 1-3.
        MAYBE_SWAP(pts, 2, 3);
        MAYBE_SWAP(pts, 1, 2);
        return;
    }

#undef MAYBE_SWAP

    // a merge sort with temp storage.

    struct pt *tmp = fb_alloc(sizeof(struct pt) * sz, FB_ALLOC_NO_HINT);

    memcpy(tmp, pts, sizeof(struct pt) * sz);

    int asz = sz/2;
    int bsz = sz - asz;

    struct pt *as = &tmp[0];
    struct pt *bs = &tmp[asz];

    ptsort(as, asz);
    ptsort(bs, bsz);

#define MERGE(apos,bpos)                        \
    if (as[apos].theta < bs[bpos].theta)        \
        pts[outpos++] = as[apos++];             \
    else                                        \
        pts[outpos++] = bs[bpos++];

    int apos = 0, bpos = 0, outpos = 0;
    while (apos + 8 < asz && bpos + 8 < bsz) {
        MERGE(apos,bpos); MERGE(apos,bpos); MERGE(apos,bpos); MERGE(apos,bpos);
        MERGE(apos,bpos); MERGE(apos,bpos); MERGE(apos,bpos); MERGE(apos,bpos);
    }

    while (apos < asz && bpos < bsz) {
        MERGE(apos,bpos);
    }

    if (apos < asz)
        memcpy(&pts[outpos], &as[apos], (asz-apos)*sizeof(struct pt));
    if (bpos < bsz)
        memcpy(&pts[outpos], &bs[bpos], (bsz-bpos)*sizeof(struct pt));

    fb_free(); // tmp

#undef MERGE
}

// lfps contains *cumulative* moments for N points, with
// index j reflecting points [0,j] (inclusive).
//
// fit a line to the points [i0, i1] (inclusive). i0, i1 are both [0,
// sz) if i1 < i0, we treat this as a wrap around.
void fit_line(struct line_fit_pt *lfps, int sz, int i0, int i1, float *lineparm, float *err, float *mse)
{
    assert(i0 != i1);
    assert(i0 >= 0 && i1 >= 0 && i0 < sz && i1 < sz);

    float Mx, My, Mxx, Myy, Mxy, W;
    int N; // how many points are included in the set?

    if (i0 < i1) {
        N = i1 - i0 + 1;

        Mx  = lfps[i1].Mx;
        My  = lfps[i1].My;
        Mxx = lfps[i1].Mxx;
        Mxy = lfps[i1].Mxy;
        Myy = lfps[i1].Myy;
        W   = lfps[i1].W;

        if (i0 > 0) {
            Mx  -= lfps[i0-1].Mx;
            My  -= lfps[i0-1].My;
            Mxx -= lfps[i0-1].Mxx;
            Mxy -= lfps[i0-1].Mxy;
            Myy -= lfps[i0-1].Myy;
            W   -= lfps[i0-1].W;
        }

    } else {
        // i0 > i1, e.g. [15, 2]. Wrap around.
        assert(i0 > 0);

        Mx  = lfps[sz-1].Mx   - lfps[i0-1].Mx;
        My  = lfps[sz-1].My   - lfps[i0-1].My;
        Mxx = lfps[sz-1].Mxx  - lfps[i0-1].Mxx;
        Mxy = lfps[sz-1].Mxy  - lfps[i0-1].Mxy;
        Myy = lfps[sz-1].Myy  - lfps[i0-1].Myy;
        W   = lfps[sz-1].W    - lfps[i0-1].W;

        Mx  += lfps[i1].Mx;
        My  += lfps[i1].My;
        Mxx += lfps[i1].Mxx;
        Mxy += lfps[i1].Mxy;
        Myy += lfps[i1].Myy;
        W   += lfps[i1].W;

        N = sz - i0 + i1 + 1;
    }

    assert(N >= 2);

    float Ex = Mx / W;
    float Ey = My / W;
    float Cxx = Mxx / W - Ex*Ex;
    float Cxy = Mxy / W - Ex*Ey;
    float Cyy = Myy / W - Ey*Ey;

    float nx, ny;

    if (1) {
        // on iOS about 5% of total CPU spent in these trig functions.
        // 85 ms per frame on 5S, example.pnm
        //
        // XXX this was using the float-precision atan2. Was there a case where
        // we needed that precision? Seems doubtful.
        float normal_theta = .5 * atan2f(-2*Cxy, (Cyy - Cxx));
        nx = cosf(normal_theta);
        ny = sinf(normal_theta);
    } else {
        // 73.5 ms per frame on 5S, example.pnm
        float ty = -2*Cxy;
        float tx = (Cyy - Cxx);
        float mag = ty*ty + tx*tx;

        if (mag == 0) {
            nx = 1;
            ny = 0;
        } else {
            float norm = sqrtf(ty*ty + tx*tx);
            tx /= norm;

            // ty is now sin(2theta)
            // tx is now cos(2theta). We want sin(theta) and cos(theta)

            // due to precision err, tx could still have slightly too large magnitude.
            if (tx > 1) {
                ny = 0;
                nx = 1;
            } else if (tx < -1) {
                ny = 1;
                nx = 0;
            } else {
                // half angle formula
                ny = sqrtf((1 - tx)/2);
                nx = sqrtf((1 + tx)/2);

                // pick a consistent branch cut
                if (ty < 0)
                    ny = - ny;
            }
        }
    }

    if (lineparm) {
        lineparm[0] = Ex;
        lineparm[1] = Ey;
        lineparm[2] = nx;
        lineparm[3] = ny;
    }

    // sum of squared errors =
    //
    // SUM_i ((p_x - ux)*nx + (p_y - uy)*ny)^2
    // SUM_i  nx*nx*(p_x - ux)^2 + 2nx*ny(p_x -ux)(p_y-uy) + ny*ny*(p_y-uy)*(p_y-uy)
    //  nx*nx*SUM_i((p_x -ux)^2) + 2nx*ny*SUM_i((p_x-ux)(p_y-uy)) + ny*ny*SUM_i((p_y-uy)^2)
    //
    //  nx*nx*N*Cxx + 2nx*ny*N*Cxy + ny*ny*N*Cyy

    // sum of squared errors
    if (err)
        *err = nx*nx*N*Cxx + 2*nx*ny*N*Cxy + ny*ny*N*Cyy;

    // mean squared error
    if (mse)
        *mse = nx*nx*Cxx + 2*nx*ny*Cxy + ny*ny*Cyy;
}

int pt_compare_theta(const void *_a, const void *_b)
{
    struct pt *a = (struct pt*) _a;
    struct pt *b = (struct pt*) _b;

    return (a->theta < b->theta) ? -1 : 1;
}

int err_compare_descending(const void *_a, const void *_b)
{
    const float *a =  _a;
    const float *b =  _b;

    return ((*a) < (*b)) ? 1 : -1;
}

/*

  1. Identify A) white points near a black point and B) black points near a white point.

  2. Find the connected components within each of the classes above,
  yielding clusters of "white-near-black" and
  "black-near-white". (These two classes are kept separate). Each
  segment has a unique id.

  3. For every pair of "white-near-black" and "black-near-white"
  clusters, find the set of points that are in one and adjacent to the
  other. In other words, a "boundary" layer between the two
  clusters. (This is actually performed by iterating over the pixels,
  rather than pairs of clusters.) Critically, this helps keep nearby
  edges from becoming connected.
*/
int quad_segment_maxima(apriltag_detector_t *td, zarray_t *cluster, struct line_fit_pt *lfps, int indices[4])
{
    int sz = zarray_size(cluster);

    // ksz: when fitting points, how many points on either side do we consider?
    // (actual "kernel" width is 2ksz).
    //
    // This value should be about: 0.5 * (points along shortest edge).
    //
    // If all edges were equally-sized, that would give a value of
    // sz/8. We make it somewhat smaller to account for tags at high
    // aspects.

    // XXX Tunable. Maybe make a multiple of JPEG block size to increase robustness
    // to JPEG compression artifacts?
    int ksz = imin(20, sz / 12);

    // can't fit a quad if there are too few points.
    if (ksz < 2)
        return 0;

//    printf("sz %5d, ksz %3d\n", sz, ksz);

    float *errs = fb_alloc(sz * sizeof(float), FB_ALLOC_NO_HINT);

    for (int i = 0; i < sz; i++) {
        fit_line(lfps, sz, (i + sz - ksz) % sz, (i + ksz) % sz, NULL, &errs[i], NULL);
    }

    // apply a low-pass filter to errs
    if (1) {
        float *y = fb_alloc(sz * sizeof(float), FB_ALLOC_NO_HINT);

        // how much filter to apply?

        // XXX Tunable
        float sigma = 1; // was 3

        // cutoff = exp(-j*j/(2*sigma*sigma));
        // log(cutoff) = -j*j / (2*sigma*sigma)
        // log(cutoff)*2*sigma*sigma = -j*j;

        // how big a filter should we use? We make our kernel big
        // enough such that we represent any values larger than
        // 'cutoff'.

        // XXX Tunable (though not super useful to change)
        float cutoff = 0.05;
        int fsz = sqrt(-log(cutoff)*2*sigma*sigma) + 1;
        fsz = 2*fsz + 1;

        // For default values of cutoff = 0.05, sigma = 3,
        // we have fsz = 17.
        float *f = fb_alloc(fsz * sizeof(float), FB_ALLOC_NO_HINT);

        for (int i = 0; i < fsz; i++) {
            int j = i - fsz / 2;
            f[i] = exp(-j*j/(2*sigma*sigma));
        }

        for (int iy = 0; iy < sz; iy++) {
            float acc = 0;
#ifdef OPTIMIZED
            int index = (iy - fsz/2 + sz) % sz;
            for (int i = 0; i < fsz; i++) {
                acc += errs[index] * f[i];
                index++;
                if (index >= sz) // faster to compare than divide (%)
                   index -= sz;
            }
#else
            for (int i = 0; i < fsz; i++) {
                acc += errs[(iy + i - fsz / 2 + sz) % sz] * f[i];
            }
#endif
            y[iy] = acc;
        }

        fb_free(); // f
        memcpy(errs, y, sz * sizeof(float));
        fb_free(); // y
    }

    int *maxima = fb_alloc(sz * sizeof(int), FB_ALLOC_NO_HINT);
    float *maxima_errs = fb_alloc(sz * sizeof(float), FB_ALLOC_NO_HINT);
    int nmaxima = 0;

    for (int i = 0; i < sz; i++) {
        if (errs[i] > errs[(i+1)%sz] && errs[i] > errs[(i+sz-1)%sz]) {
            maxima[nmaxima] = i;
            maxima_errs[nmaxima] = errs[i];
            nmaxima++;
        }
    }

    // if we didn't get at least 4 maxima, we can't fit a quad.
    if (nmaxima < 4)
        return 0;

    // select only the best maxima if we have too many
    int max_nmaxima = td->qtp.max_nmaxima;

    if (nmaxima > max_nmaxima) {
        float *maxima_errs_copy = fb_alloc(nmaxima * sizeof(float), FB_ALLOC_NO_HINT);
        memcpy(maxima_errs_copy, maxima_errs, nmaxima * sizeof(float));

        // throw out all but the best handful of maxima. Sorts descending.
        qsort(maxima_errs_copy, nmaxima, sizeof(float), err_compare_descending);

        float maxima_thresh = maxima_errs_copy[max_nmaxima];
        int out = 0;
        for (int in = 0; in < nmaxima; in++) {
            if (maxima_errs[in] <= maxima_thresh)
                continue;
            maxima[out++] = maxima[in];
        }
        nmaxima = out;

        fb_free(); // maxima_errs_copy
    }

    fb_free(); // maxima_errs
    fb_free(); // maxima
    fb_free(); // errs

    int best_indices[4];
    float best_error = HUGE_VALF;

    float err01, err12, err23, err30;
    float mse01, mse12, mse23, mse30;
    float params01[4], params12[4], params23[4], params30[4];

    // disallow quads where the angle is less than a critical value.
    float max_dot = cos(td->qtp.critical_rad); //25*M_PI/180);

    for (int m0 = 0; m0 < nmaxima - 3; m0++) {
        int i0 = maxima[m0];

        for (int m1 = m0+1; m1 < nmaxima - 2; m1++) {
            int i1 = maxima[m1];

            fit_line(lfps, sz, i0, i1, params01, &err01, &mse01);

            if (mse01 > td->qtp.max_line_fit_mse)
                continue;

            for (int m2 = m1+1; m2 < nmaxima - 1; m2++) {
                int i2 = maxima[m2];

                fit_line(lfps, sz, i1, i2, params12, &err12, &mse12);
                if (mse12 > td->qtp.max_line_fit_mse)
                    continue;

                float dot = params01[2]*params12[2] + params01[3]*params12[3];
                if (fabs(dot) > max_dot)
                    continue;

                for (int m3 = m2+1; m3 < nmaxima; m3++) {
                    int i3 = maxima[m3];

                    fit_line(lfps, sz, i2, i3, params23, &err23, &mse23);
                    if (mse23 > td->qtp.max_line_fit_mse)
                        continue;

                    fit_line(lfps, sz, i3, i0, params30, &err30, &mse30);
                    if (mse30 > td->qtp.max_line_fit_mse)
                        continue;

                    float err = err01 + err12 + err23 + err30;
                    if (err < best_error) {
                        best_error = err;
                        best_indices[0] = i0;
                        best_indices[1] = i1;
                        best_indices[2] = i2;
                        best_indices[3] = i3;
                    }
                }
            }
        }
    }

    if (best_error == HUGE_VALF)
        return 0;

    for (int i = 0; i < 4; i++)
        indices[i] = best_indices[i];

    if (best_error / sz < td->qtp.max_line_fit_mse)
        return 1;
    return 0;
}

// return 1 if the quad looks okay, 0 if it should be discarded
int fit_quad(apriltag_detector_t *td, image_u8_t *im, zarray_t *cluster, struct quad *quad, bool overrideMode)
{
    int res = 0;

    int sz = zarray_size(cluster);
    if (sz < 4) // can't fit a quad to less than 4 points
        return 0;

    /////////////////////////////////////////////////////////////
    // Step 1. Sort points so they wrap around the center of the
    // quad. We will constrain our quad fit to simply partition this
    // ordered set into 4 groups.

    // compute a bounding box so that we can order the points
    // according to their angle WRT the center.
    int32_t xmax = 0, xmin = INT32_MAX, ymax = 0, ymin = INT32_MAX;

    for (int pidx = 0; pidx < zarray_size(cluster); pidx++) {
        struct pt *p;
        zarray_get_volatile(cluster, pidx, &p);

        xmax = imax(xmax, p->x);
        xmin = imin(xmin, p->x);

        ymax = imax(ymax, p->y);
        ymin = imin(ymin, p->y);
    }

    // add some noise to (cx,cy) so that pixels get a more diverse set
    // of theta estimates. This will help us remove more points.
    // (Only helps a small amount. The actual noise values here don't
    // matter much at all, but we want them [-1, 1]. (XXX with
    // fixed-point, should range be bigger?)
    float cx = (xmin + xmax) * 0.5 + 0.05118;
    float cy = (ymin + ymax) * 0.5 + -0.028581;

    float dot = 0;

    for (int pidx = 0; pidx < zarray_size(cluster); pidx++) {
        struct pt *p;
        zarray_get_volatile(cluster, pidx, &p);

        float dx = p->x - cx;
        float dy = p->y - cy;

        p->theta = atan2f(dy, dx);

        dot += dx*p->gx + dy*p->gy;
//        p->theta = terrible_atan2(dy, dx);
    }

    // Ensure that the black border is inside the white border.
    if ((!overrideMode) && (dot < 0))
        return 0;

    // we now sort the points according to theta. This is a prepatory
    // step for segmenting them into four lines.
    if (1) {
        //        zarray_sort(cluster, pt_compare_theta);
        ptsort((struct pt*) cluster->data, zarray_size(cluster));

        // remove duplicate points. (A byproduct of our segmentation system.)
        if (1) {
            int outpos = 1;

            struct pt *last;
            zarray_get_volatile(cluster, 0, &last);

            for (int i = 1; i < sz; i++) {

                struct pt *p;
                zarray_get_volatile(cluster, i, &p);

                if (p->x != last->x || p->y != last->y) {

                    if (i != outpos)  {
                        struct pt *out;
                        zarray_get_volatile(cluster, outpos, &out);
                        memcpy(out, p, sizeof(struct pt));
                    }

                    outpos++;
                }

                last = p;
            }

            cluster->size = outpos;
            sz = outpos;
        }

    } else {
        // This is a counting sort in which we retain at most one
        // point for every bucket; the bucket index is computed from
        // theta. Since a good quad completes a complete revolution,
        // there's reason to think that we should get a good
        // distribution of thetas.  We might "lose" a few points due
        // to collisions, but this shouldn't affect quality very much.

        // XXX tunable. Increase to reduce the likelihood of "losing"
        // points due to collisions.
        int nbuckets = 4*sz;

#define ASSOC 2
        struct pt v[nbuckets][ASSOC];
        memset(v, 0, sizeof(v));

        // put each point into a bucket.
        for (int i = 0; i < sz; i++) {
            struct pt *p;
            zarray_get_volatile(cluster, i, &p);

            assert(p->theta >= -M_PI && p->theta <= M_PI);

            int bucket = (nbuckets - 1) * (p->theta + M_PI) / (2*M_PI);
            assert(bucket >= 0 && bucket < nbuckets);

            for (int i = 0; i < ASSOC; i++) {
                if (v[bucket][i].theta == 0) {
                    v[bucket][i] = *p;
                    break;
                }
            }
        }

        // collect the points from the buckets and put them back into the array.
        int outsz = 0;
        for (int i = 0; i < nbuckets; i++) {
            for (int j = 0; j < ASSOC; j++) {
                if (v[i][j].theta != 0) {
                    zarray_set(cluster, outsz, &v[i][j], NULL);
                    outsz++;
                }
            }
        }

        zarray_truncate(cluster, outsz);
        sz = outsz;
    }

    if (sz < 4)
        return 0;

    /////////////////////////////////////////////////////////////
    // Step 2. Precompute statistics that allow line fit queries to be
    // efficiently computed for any contiguous range of indices.

    struct line_fit_pt *lfps = fb_alloc0(sz * sizeof(struct line_fit_pt), FB_ALLOC_NO_HINT);

    for (int i = 0; i < sz; i++) {
        struct pt *p;
        zarray_get_volatile(cluster, i, &p);

        if (i > 0) {
            memcpy(&lfps[i], &lfps[i-1], sizeof(struct line_fit_pt));
        }

        if (0) {
            // we now undo our fixed-point arithmetic.
            float delta = 0.5;
            float x = p->x * .5 + delta;
            float y = p->y * .5 + delta;
            float W;

            for (int dy = -1; dy <= 1; dy++) {
                int iy = y + dy;

                if (iy < 0 || iy + 1 >= im->height)
                    continue;

                for (int dx = -1; dx <= 1; dx++) {
                    int ix = x + dx;

                    if (ix < 0 || ix + 1 >= im->width)
                        continue;

                    int grad_x = im->buf[iy * im->stride + ix + 1] -
                        im->buf[iy * im->stride + ix - 1];

                    int grad_y = im->buf[(iy+1) * im->stride + ix] -
                        im->buf[(iy-1) * im->stride + ix];

                    W = sqrtf(grad_x*grad_x + grad_y*grad_y) + 1;

//                    float fx = x + dx, fy = y + dy;
                    float fx = ix + .5, fy = iy + .5;
                    lfps[i].Mx  += W * fx;
                    lfps[i].My  += W * fy;
                    lfps[i].Mxx += W * fx * fx;
                    lfps[i].Mxy += W * fx * fy;
                    lfps[i].Myy += W * fy * fy;
                    lfps[i].W   += W;
                }
            }
        } else {
            // we now undo our fixed-point arithmetic.
            float delta = 0.5; // adjust for pixel center bias
            float x = p->x * .5 + delta;
            float y = p->y * .5 + delta;
            int ix = x, iy = y;
            float W = 1;

            if (ix > 0 && ix+1 < im->width && iy > 0 && iy+1 < im->height) {
                int grad_x = im->buf[iy * im->stride + ix + 1] -
                    im->buf[iy * im->stride + ix - 1];

                int grad_y = im->buf[(iy+1) * im->stride + ix] -
                    im->buf[(iy-1) * im->stride + ix];

                // XXX Tunable. How to shape the gradient magnitude?
                W = sqrt(grad_x*grad_x + grad_y*grad_y) + 1;
            }

            float fx = x, fy = y;
            lfps[i].Mx  += W * fx;
            lfps[i].My  += W * fy;
            lfps[i].Mxx += W * fx * fx;
            lfps[i].Mxy += W * fx * fy;
            lfps[i].Myy += W * fy * fy;
            lfps[i].W   += W;
        }
    }

    int indices[4];
    if (1) {
        if (!quad_segment_maxima(td, cluster, lfps, indices))
            goto finish;
    }

//    printf("%d %d %d %d\n", indices[0], indices[1], indices[2], indices[3]);

    if (0) {
        // no refitting here; just use those points as the vertices.
        // Note, this is useful for debugging, but pretty bad in
        // practice since this code path also omits several
        // plausibility checks that save us tons of time in quad
        // decoding.
        for (int i = 0; i < 4; i++) {
            struct pt *p;
            zarray_get_volatile(cluster, indices[i], &p);

            quad->p[i][0] = .5*p->x; // undo fixed-point arith.
            quad->p[i][1] = .5*p->y;
        }

        res = 1;

    } else {
        float lines[4][4];

        for (int i = 0; i < 4; i++) {
            int i0 = indices[i];
            int i1 = indices[(i+1)&3];

            if (0) {
                // if there are enough points, skip the points near the corners
                // (because those tend not to be very good.)
                if (i1-i0 > 8) {
                    int t = (i1-i0)/6;
                    if (t < 0)
                        t = -t;

                    i0 = (i0 + t) % sz;
                    i1 = (i1 + sz - t) % sz;
                }
            }

            float err;
            fit_line(lfps, sz, i0, i1, lines[i], NULL, &err);

            if (err > td->qtp.max_line_fit_mse) {
                res = 0;
                goto finish;
            }
        }

        for (int i = 0; i < 4; i++) {
            // solve for the intersection of lines (i) and (i+1)&3.
            // p0 + lambda0*u0 = p1 + lambda1*u1, where u0 and u1
            // are the line directions.
            //
            // lambda0*u0 - lambda1*u1 = (p1 - p0)
            //
            // rearrange (solve for lambdas)
            //
            // [u0_x   -u1_x ] [lambda0] = [ p1_x - p0_x ]
            // [u0_y   -u1_y ] [lambda1]   [ p1_y - p0_y ]
            //
            // remember that lines[i][0,1] = p, lines[i][2,3] = NORMAL vector.
            // We want the unit vector, so we need the perpendiculars. Thus, below
            // we have swapped the x and y components and flipped the y components.

            float A00 =  lines[i][3],  A01 = -lines[(i+1)&3][3];
            float A10 =  -lines[i][2],  A11 = lines[(i+1)&3][2];
            float B0 = -lines[i][0] + lines[(i+1)&3][0];
            float B1 = -lines[i][1] + lines[(i+1)&3][1];

            float det = A00 * A11 - A10 * A01;

            // inverse.
            float W00 = A11 / det, W01 = -A01 / det;
            if (fabs(det) < 0.001) {
                res = 0;
                goto finish;
            }

            // solve
            float L0 = W00*B0 + W01*B1;

            // compute intersection
            quad->p[i][0] = lines[i][0] + L0*A00;
            quad->p[i][1] = lines[i][1] + L0*A10;

            if (0) {
                // we should get the same intersection starting
                // from point p1 and moving L1*u1.
                float W10 = -A10 / det, W11 = A00 / det;
                float L1 = W10*B0 + W11*B1;

                float x = lines[(i+1)&3][0] - L1*A10;
                float y = lines[(i+1)&3][1] - L1*A11;
                assert(fabs(x - quad->p[i][0]) < 0.001 &&
                       fabs(y - quad->p[i][1]) < 0.001);
            }

            res = 1;
        }
    }

    // reject quads that are too small
    if (1) {
        float area = 0;

        // get area of triangle formed by points 0, 1, 2, 0
        float length[3], p;
        for (int i = 0; i < 3; i++) {
            int idxa = i; // 0, 1, 2,
            int idxb = (i+1) % 3; // 1, 2, 0
            length[i] = sqrt(sq(quad->p[idxb][0] - quad->p[idxa][0]) +
                             sq(quad->p[idxb][1] - quad->p[idxa][1]));
        }
        p = (length[0] + length[1] + length[2]) / 2;

        area += sqrt(p*(p-length[0])*(p-length[1])*(p-length[2]));

        // get area of triangle formed by points 2, 3, 0, 2
        for (int i = 0; i < 3; i++) {
            int idxs[] = { 2, 3, 0, 2 };
            int idxa = idxs[i];
            int idxb = idxs[i+1];
            length[i] = sqrt(sq(quad->p[idxb][0] - quad->p[idxa][0]) +
                             sq(quad->p[idxb][1] - quad->p[idxa][1]));
        }
        p = (length[0] + length[1] + length[2]) / 2;

        area += sqrt(p*(p-length[0])*(p-length[1])*(p-length[2]));

        // we don't actually know the family yet (quad detection is generic.)
        // This threshold is based on a 6x6 tag (which is actually 8x8)
//        int d = fam->d + fam->black_border*2;
        int d = 8;
        if (area < d*d) {
            res = 0;
            goto finish;
        }
    }

    // reject quads whose cumulative angle change isn't equal to 2PI
    if (1) {
        float total = 0;

        for (int i = 0; i < 4; i++) {
            int i0 = i, i1 = (i+1)&3, i2 = (i+2)&3;

            float theta0 = atan2f(quad->p[i0][1] - quad->p[i1][1],
                                   quad->p[i0][0] - quad->p[i1][0]);
            float theta1 = atan2f(quad->p[i2][1] - quad->p[i1][1],
                                   quad->p[i2][0] - quad->p[i1][0]);

            float dtheta = theta0 - theta1;
            if (dtheta < 0)
                dtheta += 2*M_PI;

            if (dtheta < td->qtp.critical_rad || dtheta > (M_PI - td->qtp.critical_rad))
                res = 0;

            total += dtheta;
        }

        // looking for 2PI
        if (total < 6.2 || total > 6.4) {
            res = 0;
            goto finish;
        }
    }

    // adjust pixel coordinates; all math up 'til now uses pixel
    // coordinates in which (0,0) is the lower left corner. But each
    // pixel actually spans from to [x, x+1), [y, y+1) the mean value of which
    // is +.5 higher than x & y.
/*    float delta = .5;
      for (int i = 0; i < 4; i++) {
      quad->p[i][0] += delta;
      quad->p[i][1] += delta;
      }
*/
  finish:

    fb_free(); // lfps

    return res;
}

#ifdef OPTIMIZED
#define DO_UNIONFIND(dx, dy) if (im->buf[y*s + dy*s + x + dx] == v) { broot = unionfind_get_representative(uf, y*w + dy*w + x + dx); if (aroot != broot) uf->data[broot].parent = aroot; }

static void do_unionfind_line(unionfind_t *uf, image_u8_t *im, int h, int w, int s, int y)
{
    assert(y+1 < im->height);
    uint8_t v, *p;
    p = &im->buf[y*s + 1];
    for (int x = 1; x < w - 1; x++) {
        v = *p++; //im->buf[y*s + x];

        if (v == 127)
            continue;
        uint32_t broot;
        uint32_t aroot = unionfind_get_representative(uf, y*w+x);
        // (dx,dy) pairs for 8 connectivity:
        //          (REFERENCE) (1, 0)
        // (-1, 1)    (0, 1)    (1, 1)
        //
        DO_UNIONFIND(1, 0);
        DO_UNIONFIND(0, 1);
        if (v == 255) {
            DO_UNIONFIND(-1, 1);
            DO_UNIONFIND(1, 1);
        }
    }
}
#else // not optimized
#define DO_UNIONFIND(dx, dy) if (im->buf[y*s + dy*s + x + dx] == v) unionfind_connect(uf, y*w + x, y*w + dy*w + x + dx);

static void do_unionfind_line(unionfind_t *uf, image_u8_t *im, int h, int w, int s, int y)
{
    assert(y+1 < im->height);

    for (int x = 1; x < w - 1; x++) {
        uint8_t v = im->buf[y*s + x];

        if (v == 127)
            continue;

        // (dx,dy) pairs for 8 connectivity:
        //          (REFERENCE) (1, 0)
        // (-1, 1)    (0, 1)    (1, 1)
        //
        DO_UNIONFIND(1, 0);
        DO_UNIONFIND(0, 1);
        if (v == 255) {
            DO_UNIONFIND(-1, 1);
            DO_UNIONFIND(1, 1);
        }
    }
}
#undef DO_UNIONFIND
#endif // OPTIMIZED

image_u8_t *threshold(apriltag_detector_t *td, image_u8_t *im)
{
    int w = im->width, h = im->height, s = im->stride;
    assert(w < 32768);
    assert(h < 32768);

    image_u8_t *threshim = fb_alloc(sizeof(image_u8_t), FB_ALLOC_NO_HINT);
    threshim->width = w;
    threshim->height = h;
    threshim->stride = s;
    threshim->buf = fb_alloc(w * h, FB_ALLOC_NO_HINT);
    assert(threshim->stride == s);

    // The idea is to find the maximum and minimum values in a
    // window around each pixel. If it's a contrast-free region
    // (max-min is small), don't try to binarize. Otherwise,
    // threshold according to (max+min)/2.
    //
    // Mark low-contrast regions with value 127 so that we can skip
    // future work on these areas too.

    // however, computing max/min around every pixel is needlessly
    // expensive. We compute max/min for tiles. To avoid artifacts
    // that arise when high-contrast features appear near a tile
    // edge (and thus moving from one tile to another results in a
    // large change in max/min value), the max/min values used for
    // any pixel are computed from all 3x3 surrounding tiles. Thus,
    // the max/min sampling area for nearby pixels overlap by at least
    // one tile.
    //
    // The important thing is that the windows be large enough to
    // capture edge transitions; the tag does not need to fit into
    // a tile.

    // XXX Tunable. Generally, small tile sizes--- so long as they're
    // large enough to span a single tag edge--- seem to be a winner.
    const int tilesz = 4;

    // the last (possibly partial) tiles along each row and column will
    // just use the min/max value from the last full tile.
    int tw = w / tilesz;
    int th = h / tilesz;

    uint8_t *im_max = fb_alloc(tw*th*sizeof(uint8_t), FB_ALLOC_NO_HINT);
    uint8_t *im_min = fb_alloc(tw*th*sizeof(uint8_t), FB_ALLOC_NO_HINT);

    // first, collect min/max statistics for each tile
    for (int ty = 0; ty < th; ty++) {
        for (int tx = 0; tx < tw; tx++) {
#if defined( OPTIMIZED ) && (defined(ARM_MATH_CM7) || defined(ARM_MATH_CM4))
        uint32_t tmp, max32 = 0, min32 = 0xffffffff;
        for (int dy=0; dy < tilesz; dy++) {
            uint32_t v = *(uint32_t *)&im->buf[(ty*tilesz+dy)*s + tx*tilesz];
            tmp = __USUB8(v, max32);
            max32 = __SEL(v, max32);
            tmp = __USUB8(min32, v);
            min32 = __SEL(v, min32);
        }
        // find the min/max of the 4 remaining values
        tmp = max32 >> 16;
        __USUB8(max32, tmp); // 4->2
        max32 = __SEL(max32, tmp);
        tmp = max32 >> 8;
        __USUB8(max32, tmp); // 2->1
        max32 = __SEL(max32, tmp);
        tmp = min32 >> 16;
        __USUB8(min32, tmp);
        min32 = __SEL(tmp, min32); // 4-->2
        tmp = min32 >> 8;
        __USUB8(min32, tmp);
        min32 = __SEL(tmp, min32); // 2-->1
        im_max[ty*tw+tx] = (uint8_t)max32;
        im_min[ty*tw+tx] = (uint8_t)min32;
#else
        uint8_t max = 0, min = 255;
        for (int dy = 0; dy < tilesz; dy++) {
            for (int dx = 0; dx < tilesz; dx++) {
                uint8_t v = im->buf[(ty*tilesz+dy)*s + tx*tilesz + dx];
                if (v < min)
                    min = v;
                if (v > max)
                    max = v;
            }
        }
        im_max[ty*tw+tx] = max;
        im_min[ty*tw+tx] = min;
#endif
        }
    }

    // second, apply 3x3 max/min convolution to "blur" these values
    // over larger areas. This reduces artifacts due to abrupt changes
    // in the threshold value.
    if (1) {
        uint8_t *im_max_tmp = fb_alloc(tw*th*sizeof(uint8_t), FB_ALLOC_NO_HINT);
        uint8_t *im_min_tmp = fb_alloc(tw*th*sizeof(uint8_t), FB_ALLOC_NO_HINT);

#ifdef OPTIMIZED
        // Checking boundaries on every pixel wastes significant time; just break it into 5 pieces
        // (center, top, bottom, left right)
        // First pass does the entire center area
        int ty, tx, dy, dx;
        for (ty = 1; ty < th-1; ty++) {
            for (tx = 1; tx < tw-1; tx++) {
                uint8_t max = 0, min = 255;
                for (dy = -1; dy <= 1; dy++) {
                    for (dx = -1; dx <= 1; dx++) {
                        uint8_t m = im_max[(ty+dy)*tw+tx+dx];
                        if (m > max)
                            max = m;
                        m = im_min[(ty+dy)*tw+tx+dx];
                        if (m < min)
                            min = m;
                    }
                }
                im_max_tmp[ty*tw + tx] = max;
                im_min_tmp[ty*tw + tx] = min;
            }
        }
        // top edge
        ty = 0;
        for (tx = 1; tx < tw-1; tx++) {
            uint8_t max = 0, min = 255;
            for (dy = 0; dy <= 1; dy++) {
                for (dx = -1; dx <= 1; dx++) {
                    uint8_t m = im_max[(ty+dy)*tw+tx+dx];
                    if (m > max)
                        max = m;
                    m = im_min[(ty+dy)*tw+tx+dx];
                    if (m < min)
                        min = m;
                }
            }
            im_max_tmp[ty*tw + tx] = max;
            im_min_tmp[ty*tw + tx] = min;
        }
        // bottom edge
        ty = th-1;
        for (tx = 1; tx < tw-1; tx++) {
            uint8_t max = 0, min = 255;
            for (dy = -1; dy <= 0; dy++) {
                for (dx = -1; dx <= 1; dx++) {
                    uint8_t m = im_max[(ty+dy)*tw+tx+dx];
                    if (m > max)
                        max = m;
                    m = im_min[(ty+dy)*tw+tx+dx];
                    if (m < min)
                        min = m;
                }
            }
            im_max_tmp[ty*tw + tx] = max;
            im_min_tmp[ty*tw + tx] = min;
        }
        // left edge
        tx = 0;
        for (ty = 1; ty < th-1; ty++) {
            uint8_t max = 0, min = 255;
            for (dy = -1; dy <= 1; dy++) {
                for (dx = 0; dx <= 1; dx++) {
                    uint8_t m = im_max[(ty+dy)*tw+tx+dx];
                    if (m > max)
                        max = m;
                    m = im_min[(ty+dy)*tw+tx+dx];
                    if (m < min)
                        min = m;
                }
            }
            im_max_tmp[ty*tw + tx] = max;
            im_min_tmp[ty*tw + tx] = min;
        }
        // right edge
        tx = tw-1;
        for (ty = 1; ty < th-1; ty++) {
            uint8_t max = 0, min = 255;
            for (dy = -1; dy <= 1; dy++) {
                for (dx = -1; dx <= 0; dx++) {
                    uint8_t m = im_max[(ty+dy)*tw+tx+dx];
                    if (m > max)
                        max = m;
                    m = im_min[(ty+dy)*tw+tx+dx];
                    if (m < min)
                        min = m;
                }
            }
            im_max_tmp[ty*tw + tx] = max;
            im_min_tmp[ty*tw + tx] = min;
        }
#else
        for (int ty = 0; ty < th; ty++) {
            for (int tx = 0; tx < tw; tx++) {
                uint8_t max = 0, min = 255;

                for (int dy = -1; dy <= 1; dy++) {
                    if (ty+dy < 0 || ty+dy >= th)
                        continue;
                    for (int dx = -1; dx <= 1; dx++) {
                        if (tx+dx < 0 || tx+dx >= tw)
                            continue;

                        uint8_t m = im_max[(ty+dy)*tw+tx+dx];
                        if (m > max)
                            max = m;
                        m = im_min[(ty+dy)*tw+tx+dx];
                        if (m < min)
                            min = m;
                    }
                }

                im_max_tmp[ty*tw + tx] = max;
                im_min_tmp[ty*tw + tx] = min;
            }
        }
#endif
        memcpy(im_max, im_max_tmp, tw*th*sizeof(uint8_t));
        memcpy(im_min, im_min_tmp, tw*th*sizeof(uint8_t));
        fb_free(); // im_min_tmp
        fb_free(); // im_max_tmp
    }
#if defined( OPTIMIZED ) && (defined(ARM_MATH_CM7) || defined(ARM_MATH_CM4))
    if ((s & 0x3) == 0 && tilesz == 4) // if each line is a multiple of 4, we can do this faster
    {
        const uint32_t lowcontrast = 0x7f7f7f7f;
        const int s32 = s/4; // pitch for 32-bit values
        const int minmax = td->qtp.min_white_black_diff; // local var to avoid constant dereferencing of the pointer
        for (int ty = 0; ty < th; ty++) {
            for (int tx = 0; tx < tw; tx++) {

                int min = im_min[ty*tw + tx];
                int max = im_max[ty*tw + tx];

                // low contrast region? (no edges)
                if (max - min < minmax) {
                    uint32_t *d32 = (uint32_t *)&threshim->buf[ty*tilesz*s + tx*tilesz];
                    d32[0] = d32[s32] = d32[s32*2] = d32[s32*3] = lowcontrast;
                    continue;
                } // if low contrast
                    // otherwise, actually threshold this tile.

                    // argument for biasing towards dark; specular highlights
                    // can be substantially brighter than white tag parts
                    uint32_t thresh32 = (min + (max - min) / 2) + 1; // plus 1 to make GT become GE for the __USUB8 and __SEL instructions
                    uint32_t u32tmp;
                    thresh32 *= 0x01010101; // spread value to all 4 slots
                        for (int dy = 0; dy < tilesz; dy++) {
                        uint32_t *d32 = (uint32_t *)&threshim->buf[(ty*tilesz+dy)*s + tx*tilesz];
                            uint32_t *s32 = (uint32_t *)&im->buf[(ty*tilesz+dy)*s + tx*tilesz];
                            // process 4 pixels at a time
                            u32tmp = s32[0];
                            u32tmp = __USUB8(u32tmp, thresh32);
                            u32tmp = __SEL(0xffffffff, 0x00000000); // 4 thresholded pixels
                            d32[0] = u32tmp;
                    } // dy
            } // tx
        } // ty
    }
    else // need to do it the slow way
#endif // OPTIMIZED
    {
    for (int ty = 0; ty < th; ty++) {
        for (int tx = 0; tx < tw; tx++) {

            int min = im_min[ty*tw + tx];
            int max = im_max[ty*tw + tx];

            // low contrast region? (no edges)
            if (max - min < td->qtp.min_white_black_diff) {
                for (int dy = 0; dy < tilesz; dy++) {
                    int y = ty*tilesz + dy;

                    for (int dx = 0; dx < tilesz; dx++) {
                        int x = tx*tilesz + dx;

                        threshim->buf[y*s+x] = 127;
                    }
                }
                continue;
            }

            // otherwise, actually threshold this tile.

            // argument for biasing towards dark; specular highlights
            // can be substantially brighter than white tag parts
            uint8_t thresh = min + (max - min) / 2;

            for (int dy = 0; dy < tilesz; dy++) {
                int y = ty*tilesz + dy;

                for (int dx = 0; dx < tilesz; dx++) {
                    int x = tx*tilesz + dx;

                    uint8_t v = im->buf[y*s+x];
                    if (v > thresh)
                        threshim->buf[y*s+x] = 255;
                    else
                        threshim->buf[y*s+x] = 0;
                }
            }
        }
    }
    }

    // we skipped over the non-full-sized tiles above. Fix those now.
    if (1) {
        for (int y = 0; y < h; y++) {

            // what is the first x coordinate we need to process in this row?

            int x0;

            if (y >= th*tilesz) {
                x0 = 0; // we're at the bottom; do the whole row.
            } else {
                x0 = tw*tilesz; // we only need to do the right most part.
            }

            // compute tile coordinates and clamp.
            int ty = y / tilesz;
            if (ty >= th)
                ty = th - 1;

            for (int x = x0; x < w; x++) {
                int tx = x / tilesz;
                if (tx >= tw)
                    tx = tw - 1;

                int max = im_max[ty*tw + tx];
                int min = im_min[ty*tw + tx];
                int thresh = min + (max - min) / 2;

                uint8_t v = im->buf[y*s+x];
                if (v > thresh)
                    threshim->buf[y*s+x] = 255;
                else
                    threshim->buf[y*s+x] = 0;
            }
        }
    }

    fb_free(); // im_min
    fb_free(); // im_max

    // this is a dilate/erode deglitching scheme that does not improve
    // anything as far as I can tell.
    if (0 || td->qtp.deglitch) {
        image_u8_t *tmp = fb_alloc(sizeof(image_u8_t), FB_ALLOC_NO_HINT);
        tmp->width = w;
        tmp->height = h;
        tmp->stride = s;
        tmp->buf = fb_alloc(w * h, FB_ALLOC_NO_HINT);

        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t max = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = threshim->buf[(y+dy)*s + x + dx];
                        if (v > max)
                            max = v;
                    }
                }
                tmp->buf[y*s+x] = max;
            }
        }

        for (int y = 1; y + 1 < h; y++) {
            for (int x = 1; x + 1 < w; x++) {
                uint8_t min = 255;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        uint8_t v = tmp->buf[(y+dy)*s + x + dx];
                        if (v < min)
                            min = v;
                    }
                }
                threshim->buf[y*s+x] = min;
            }
        }

        fb_free(); // tmp->buf
        fb_free(); // tmp
    }

    return threshim;
}

zarray_t *apriltag_quad_thresh(apriltag_detector_t *td, image_u8_t *im, bool overrideMode)
{
    ////////////////////////////////////////////////////////
    // step 1. threshold the image, creating the edge image.

    int w = im->width, h = im->height;

    image_u8_t *threshim = threshold(td, im);
    int ts = threshim->stride;

    ////////////////////////////////////////////////////////
    // step 2. find connected components.

    unionfind_t *uf = unionfind_create(w * h);

    for (int y = 0; y < h - 1; y++) {
        do_unionfind_line(uf, threshim, h, w, ts, y);
    }

    uint32_t nclustermap;
    struct uint32_zarray_entry **clustermap = fb_alloc0_all(&nclustermap, FB_ALLOC_PREFER_SPEED);
    nclustermap /= sizeof(struct uint32_zarray_entry*);
    if (!nclustermap) fb_alloc_fail();

    for (int y = 1; y < h-1; y++) {
        for (int x = 1; x < w-1; x++) {

            uint8_t v0 = threshim->buf[y*ts + x];
            if (v0 == 127)
                continue;

            // XXX don't query this until we know we need it?
            uint32_t rep0 = unionfind_get_representative(uf, y*w + x);

            // whenever we find two adjacent pixels such that one is
            // white and the other black, we add the point half-way
            // between them to a cluster associated with the unique
            // ids of the white and black regions.
            //
            // We additionally compute the gradient direction (i.e., which
            // direction was the white pixel?) Note: if (v1-v0) == 255, then
            // (dx,dy) points towards the white pixel. if (v1-v0) == -255, then
            // (dx,dy) points towards the black pixel. p.gx and p.gy will thus
            // be -255, 0, or 255.
            //
            // Note that any given pixel might be added to multiple
            // different clusters. But in the common case, a given
            // pixel will be added multiple times to the same cluster,
            // which increases the size of the cluster and thus the
            // computational costs.
            //
            // A possible optimization would be to combine entries
            // within the same cluster.

#define DO_CONN(dx, dy)                                                 \
            if (1) {                                                    \
                uint8_t v1 = threshim->buf[y*ts + dy*ts + x + dx];      \
                                                                        \
                while (v0 + v1 == 255) {                                   \
                    uint32_t rep1 = unionfind_get_representative(uf, y*w + dy*w + x + dx); \
                    uint32_t clusterid;                                 \
                    if (rep0 < rep1)                                    \
                        clusterid = (rep1 << 16) + rep0;                \
                    else                                                \
                        clusterid = (rep0 << 16) + rep1;                \
                                                                        \
                    /* XXX lousy hash function */                       \
                    uint32_t clustermap_bucket = u64hash_2(clusterid) % nclustermap; \
                    struct uint32_zarray_entry *entry = clustermap[clustermap_bucket]; \
                    while (entry && entry->id != clusterid)     {       \
                        entry = entry->next;                            \
                    }                                                   \
                                                                        \
                    if (!entry) {                                       \
                        entry = umm_calloc(1, sizeof(struct uint32_zarray_entry)); \
                        if (!entry) break;                              \
                        entry->id = clusterid;                          \
                        entry->cluster = zarray_create_fail_ok(sizeof(struct pt)); \
                        if (!entry->cluster) {                          \
                            free(entry);                                \
                            break;                                      \
                        }                                               \
                        entry->next = clustermap[clustermap_bucket];    \
                        clustermap[clustermap_bucket] = entry;          \
                    }                                                   \
                                                                        \
                    struct pt p = { .x = 2*x + dx, .y = 2*y + dy, .gx = dx*((int) v1-v0), .gy = dy*((int) v1-v0)}; \
                    zarray_add_fail_ok(entry->cluster, &p);             \
                    break;                                              \
                }                                                       \
            }

            // do 4 connectivity. NB: Arguments must be [-1, 1] or we'll overflow .gx, .gy
            DO_CONN(1, 0);
            DO_CONN(0, 1);

#ifdef IMLIB_ENABLE_FINE_APRILTAGS
            // do 8 connectivity
            DO_CONN(-1, 1);
            DO_CONN(1, 1);
#endif
        }
    }
#undef DO_CONN

    ////////////////////////////////////////////////////////
    // step 3. process each connected component.
    zarray_t *clusters = zarray_create_fail_ok(sizeof(zarray_t*)); //, uint32_zarray_hash_size(clustermap));
    if (clusters) {
        for (int i = 0; i < nclustermap; i++) {

            for (struct uint32_zarray_entry *entry = clustermap[i]; entry; entry = entry->next) {
                // XXX reject clusters here?
                zarray_add_fail_ok(clusters, &entry->cluster);
            }
        }
    }


    int sz = clusters ? zarray_size(clusters) : 0;

    if (1) {
      for (int i = 0; i < nclustermap; i++) {
        struct uint32_zarray_entry *entry = clustermap[i];
        while (entry) {
          // free any leaked cluster (zarray_add_fail_ok)
          bool leaked = true;
          for (int j = 0; j < sz && leaked; j++) {
              zarray_t *cluster;
              zarray_get(clusters, j, &cluster);
              leaked &= entry->cluster != cluster;
          }
          if (leaked) free(entry->cluster);
          struct uint32_zarray_entry *tmp = entry->next;
          free(entry);
          entry = tmp;
        }
      }
      fb_free(); // clustermap
    }

    unionfind_destroy();

    fb_free(); // threshim->buf
    fb_free(); // threshim

    zarray_t *quads = zarray_create_fail_ok(sizeof(struct quad));

    if (quads) {
        for (int i = 0; i < sz; i++) {

            zarray_t *cluster;
            zarray_get(clusters, i, &cluster);

            if (zarray_size(cluster) < td->qtp.min_cluster_pixels)
                continue;

            // a cluster should contain only boundary points around the
            // tag. it cannot be bigger than the whole screen. (Reject
            // large connected blobs that will be prohibitively slow to
            // fit quads to.) A typical point along an edge is added three
            // times (because it has 3 neighbors). The maximum perimeter
            // is 2w+2h.
            if (zarray_size(cluster) > 3*(2*w+2*h)) {
                continue;
            }

            struct quad quad;
            memset(&quad, 0, sizeof(struct quad));

            if (fit_quad(td, im, cluster, &quad, overrideMode)) {

                zarray_add_fail_ok(quads, &quad);
            }
        }
    }

    //        printf("  %d %d %d %d\n", indices[0], indices[1], indices[2], indices[3]);

    for (int i = 0; i < sz; i++) {
        zarray_t *cluster;
        zarray_get(clusters, i, &cluster);
        zarray_destroy(cluster);
    }

    if (clusters) zarray_destroy(clusters);


    if (!quads) {
        // we should have enough memory now
        quads = zarray_create(sizeof(struct quad));
    }
    return quads;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////// "apriltag.c"
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

// Regresses a model of the form:
// intensity(x,y) = C0*x + C1*y + CC2
// The J matrix is the:
//    J = [ x1 y1 1 ]
//        [ x2 y2 1 ]
//        [ ...     ]
//  The A matrix is J'J

struct graymodel
{
    float A[3][3];
    float B[3];
    float C[3];
};

void graymodel_init(struct graymodel *gm)
{
    memset(gm, 0, sizeof(struct graymodel));
}

void graymodel_add(struct graymodel *gm, float x, float y, float gray)
{
    // update upper right entries of A = J'J
    gm->A[0][0] += x*x;
    gm->A[0][1] += x*y;
    gm->A[0][2] += x;
    gm->A[1][1] += y*y;
    gm->A[1][2] += y;
    gm->A[2][2] += 1;

    // update B = J'gray
    gm->B[0] += x * gray;
    gm->B[1] += y * gray;
    gm->B[2] += gray;
}

void graymodel_solve(struct graymodel *gm)
{
    mat33_sym_solve((float*) gm->A, gm->B, gm->C);
}

float graymodel_interpolate(struct graymodel *gm, float x, float y)
{
    return gm->C[0]*x + gm->C[1]*y + gm->C[2];
}

struct quick_decode_entry
{
    uint64_t rcode;   // the queried code
    uint16_t id;      // the tag ID (a small integer)
    uint8_t hamming;  // how many errors corrected?
    uint8_t rotation; // number of rotations [0, 3]
    bool hmirror;
    bool vflip;
};

struct quick_decode
{
    int nentries;
    struct quick_decode_entry *entries;
};

/** if the bits in w were arranged in a d*d grid and that grid was
 * rotated, what would the new bits in w be?
 * The bits are organized like this (for d = 3):
 *
 *  8 7 6       2 5 8      0 1 2
 *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
 *  2 1 0       0 3 6      6 7 8
 **/
static uint64_t rotate90(uint64_t w, uint32_t d)
{
    uint64_t wr = 0;

    for (int32_t r = d-1; r >=0; r--) {
        for (int32_t c = 0; c < d; c++) {
            int32_t b = r + d*c;

            wr = wr << 1;

            if ((w & (((uint64_t) 1) << b))!=0)
                wr |= 1;
        }
    }

    return wr;
}

static uint64_t hmirror_code(uint64_t w, uint32_t d)
{
    uint64_t wr = 0;

    for (int32_t r = d-1; r >=0; r--) {
        for (int32_t c = 0; c < d; c++) {
            int32_t b = c + d*r;

            wr = wr << 1;

            if ((w & (((uint64_t) 1) << b))!=0)
                wr |= 1;
        }
    }

    return wr;
}

static uint64_t vflip_code(uint64_t w, uint32_t d)
{
    uint64_t wr = 0;

    for (int32_t r = 0; r < d; r++) {
        for (int32_t c = d-1; c >=0; c--) {
            int32_t b = c + d*r;

            wr = wr << 1;

            if ((w & (((uint64_t) 1) << b))!=0)
                wr |= 1;
        }
    }

    return wr;
}

void quad_destroy(struct quad *quad)
{
    if (!quad)
        return;

    matd_destroy(quad->H);
    matd_destroy(quad->Hinv);
    free(quad);
}

struct quad *quad_copy(struct quad *quad)
{
    struct quad *q = calloc(1, sizeof(struct quad));
    memcpy(q, quad, sizeof(struct quad));
    if (quad->H)
        q->H = matd_copy(quad->H);
    if (quad->Hinv)
        q->Hinv = matd_copy(quad->Hinv);
    return q;
}

// http://en.wikipedia.org/wiki/Hamming_weight

//types and constants used in the functions below
//uint64_t is an unsigned 64-bit integer variable type (defined in C99 version of C language)
const uint64_t m1  = 0x5555555555555555; //binary: 0101...
const uint64_t m2  = 0x3333333333333333; //binary: 00110011..
const uint64_t m4  = 0x0f0f0f0f0f0f0f0f; //binary:  4 zeros,  4 ones ...
const uint64_t m8  = 0x00ff00ff00ff00ff; //binary:  8 zeros,  8 ones ...
const uint64_t m16 = 0x0000ffff0000ffff; //binary: 16 zeros, 16 ones ...
const uint64_t m32 = 0x00000000ffffffff; //binary: 32 zeros, 32 ones
const uint64_t hff = 0xffffffffffffffff; //binary: all ones
const uint64_t h01 = 0x0101010101010101; //the sum of 256 to the power of 0,1,2,3...

//This is a naive implementation, shown for comparison,
//and to help in understanding the better functions.
//This algorithm uses 24 arithmetic operations (shift, add, and).
int popcount64a(uint64_t x)
{
    x = (x & m1 ) + ((x >>  1) & m1 ); //put count of each  2 bits into those  2 bits
    x = (x & m2 ) + ((x >>  2) & m2 ); //put count of each  4 bits into those  4 bits
    x = (x & m4 ) + ((x >>  4) & m4 ); //put count of each  8 bits into those  8 bits
    x = (x & m8 ) + ((x >>  8) & m8 ); //put count of each 16 bits into those 16 bits
    x = (x & m16) + ((x >> 16) & m16); //put count of each 32 bits into those 32 bits
    x = (x & m32) + ((x >> 32) & m32); //put count of each 64 bits into those 64 bits
    return x;
}

//This uses fewer arithmetic operations than any other known
//implementation on machines with slow multiplication.
//This algorithm uses 17 arithmetic operations.
int popcount64b(uint64_t x)
{
    x -= (x >> 1) & m1;             //put count of each 2 bits into those 2 bits
    x = (x & m2) + ((x >> 2) & m2); //put count of each 4 bits into those 4 bits
    x = (x + (x >> 4)) & m4;        //put count of each 8 bits into those 8 bits
    x += x >>  8;  //put count of each 16 bits into their lowest 8 bits
    x += x >> 16;  //put count of each 32 bits into their lowest 8 bits
    x += x >> 32;  //put count of each 64 bits into their lowest 8 bits
    return x & 0x7f;
}

//This uses fewer arithmetic operations than any other known
//implementation on machines with fast multiplication.
//This algorithm uses 12 arithmetic operations, one of which is a multiply.
int popcount64c(uint64_t x)
{
    x -= (x >> 1) & m1;             //put count of each 2 bits into those 2 bits
    x = (x & m2) + ((x >> 2) & m2); //put count of each 4 bits into those 4 bits
    x = (x + (x >> 4)) & m4;        //put count of each 8 bits into those 8 bits
    return (x * h01) >> 56;  //returns left 8 bits of x + (x<<8) + (x<<16) + (x<<24) + ...
}

// returns an entry with hamming set to 255 if no decode was found.
static void quick_decode_codeword(apriltag_family_t *tf, uint64_t rcode,
                                  struct quick_decode_entry *entry)
{
    int threshold = imax(tf->h - tf->d - 1, 0);

    for (int ridx = 0; ridx < 4; ridx++) {

        for (int i = 0, j = tf->ncodes; i < j; i++) {
            int hamming = popcount64c(tf->codes[i] ^ rcode);
            if(hamming <= threshold) {
                entry->rcode = rcode;
                entry->id = i;
                entry->hamming = hamming;
                entry->rotation = ridx;
                entry->hmirror = false;
                entry->vflip = false;
                return;
            }
        }

        rcode = rotate90(rcode, tf->d);
    }

    rcode = hmirror_code(rcode, tf->d); // handle hmirror

    for (int ridx = 0; ridx < 4; ridx++) {

        for (int i = 0, j = tf->ncodes; i < j; i++) {
            int hamming = popcount64c(tf->codes[i] ^ rcode);
            if(hamming <= threshold) {
                entry->rcode = rcode;
                entry->id = i;
                entry->hamming = hamming;
                entry->rotation = ridx;
                entry->hmirror = true;
                entry->vflip = false;
                return;
            }
        }

        rcode = rotate90(rcode, tf->d);
    }

    rcode = vflip_code(rcode, tf->d); // handle hmirror+vflip

    for (int ridx = 0; ridx < 4; ridx++) {

        for (int i = 0, j = tf->ncodes; i < j; i++) {
            int hamming = popcount64c(tf->codes[i] ^ rcode);
            if(hamming <= threshold) {
                entry->rcode = rcode;
                entry->id = i;
                entry->hamming = hamming;
                entry->rotation = ridx;
                entry->hmirror = true;
                entry->vflip = true;
                return;
            }
        }

        rcode = rotate90(rcode, tf->d);
    }

    rcode = hmirror_code(rcode, tf->d); // handle vflip

    for (int ridx = 0; ridx < 4; ridx++) {

        for (int i = 0, j = tf->ncodes; i < j; i++) {
            int hamming = popcount64c(tf->codes[i] ^ rcode);
            if(hamming <= threshold) {
                entry->rcode = rcode;
                entry->id = i;
                entry->hamming = hamming;
                entry->rotation = ridx;
                entry->hmirror = false;
                entry->vflip = true;
                return;
            }
        }

        rcode = rotate90(rcode, tf->d);
    }

    entry->rcode = 0;
    entry->id = 65535;
    entry->hamming = 255;
    entry->rotation = 0;
    entry->hmirror = false;
    entry->vflip = false;
}

static inline int detection_compare_function(const void *_a, const void *_b)
{
    apriltag_detection_t *a = *(apriltag_detection_t**) _a;
    apriltag_detection_t *b = *(apriltag_detection_t**) _b;

    return a->id - b->id;
}

void apriltag_detector_remove_family(apriltag_detector_t *td, apriltag_family_t *fam)
{
    zarray_remove_value(td->tag_families, &fam, 0);
}

void apriltag_detector_add_family_bits(apriltag_detector_t *td, apriltag_family_t *fam, int bits_corrected)
{
    zarray_add(td->tag_families, &fam);
}

void apriltag_detector_clear_families(apriltag_detector_t *td)
{
    zarray_clear(td->tag_families);
}

apriltag_detector_t *apriltag_detector_create()
{
    apriltag_detector_t *td = (apriltag_detector_t*) calloc(1, sizeof(apriltag_detector_t));

    td->qtp.max_nmaxima = 10;
    td->qtp.min_cluster_pixels = 5;

    td->qtp.max_line_fit_mse = 10.0;
    td->qtp.critical_rad = 10 * M_PI / 180;
    td->qtp.deglitch = 0;
    td->qtp.min_white_black_diff = 5;

    td->tag_families = zarray_create(sizeof(apriltag_family_t*));

    td->refine_edges = 1;
    td->refine_pose = 0;
    td->refine_decode = 0;

    return td;
}

void apriltag_detector_destroy(apriltag_detector_t *td)
{
    apriltag_detector_clear_families(td);

    zarray_destroy(td->tag_families);
    free(td);
}

struct evaluate_quad_ret
{
    int64_t rcode;
    float  score;
    matd_t  *H, *Hinv;

    int decode_status;
    struct quick_decode_entry e;
};

// returns non-zero if an error occurs (i.e., H has no inverse)
int quad_update_homographies(struct quad *quad)
{
    zarray_t *correspondences = zarray_create(sizeof(float[4]));

    for (int i = 0; i < 4; i++) {
        float corr[4];

        // At this stage of the pipeline, we have not attempted to decode the
        // quad into an oriented tag. Thus, just act as if the quad is facing
        // "up" with respect to our desired corners. We'll fix the rotation
        // later.
        // [-1, -1], [1, -1], [1, 1], [-1, 1]
        corr[0] = (i==0 || i==3) ? -1 : 1;
        corr[1] = (i==0 || i==1) ? -1 : 1;

        corr[2] = quad->p[i][0];
        corr[3] = quad->p[i][1];

        zarray_add(correspondences, &corr);
    }

    if (quad->H)
        matd_destroy(quad->H);
    if (quad->Hinv)
        matd_destroy(quad->Hinv);

    // XXX Tunable
    quad->H = homography_compute(correspondences, HOMOGRAPHY_COMPUTE_FLAG_SVD);
    quad->Hinv = matd_inverse(quad->H);
    zarray_destroy(correspondences);

    if (quad->H && quad->Hinv)
        return 0;

    return -1;
}

// compute a "score" for a quad that is independent of tag family
// encoding (but dependent upon the tag geometry) by considering the
// contrast around the exterior of the tag.
float quad_goodness(apriltag_family_t *family, image_u8_t *im, struct quad *quad)
{
    // when sampling from the white border, how much white border do
    // we actually consider valid, measured in bit-cell units? (the
    // outside portions are often intruded upon, so it could be advantageous to use
    // less than the "nominal" 1.0. (Less than 1.0 not well tested.)

    // XXX Tunable
    float white_border = 1;

    // in tag coordinates, how big is each bit cell?
    float bit_size = 2.0 / (2*family->black_border + family->d);
//    float inv_bit_size = 1.0 / bit_size;

    int32_t xmin = INT32_MAX, xmax = 0, ymin = INT32_MAX, ymax = 0;

    for (int i = 0; i < 4; i++) {
        float tx = (i == 0 || i == 3) ? -1 - bit_size : 1 + bit_size;
        float ty = (i == 0 || i == 1) ? -1 - bit_size : 1 + bit_size;
        float x, y;

        homography_project(quad->H, tx, ty, &x, &y);
        xmin = imin(xmin, x);
        xmax = imax(xmax, x);
        ymin = imin(ymin, y);
        ymax = imax(ymax, y);
    }

    // clamp bounding box to image dimensions
    xmin = imax(0, xmin);
    xmax = imin(im->width-1, xmax);
    ymin = imax(0, ymin);
    ymax = imin(im->height-1, ymax);

//    int nbits = family->d * family->d;

    int32_t W1 = 0, B1 = 0, Wn = 0, Bn = 0; // int64_t W1 = 0, B1 = 0, Wn = 0, Bn = 0;

    float wsz = bit_size*white_border;
    float bsz = bit_size*family->black_border;

    matd_t *Hinv = quad->Hinv;
//    matd_t *H = quad->H;

    // iterate over all the pixels in the tag. (Iterating in pixel space)
    for (int y = ymin; y <= ymax; y++) {

        // we'll incrementally compute the homography
        // projections. Begin by evaluating the homogeneous position
        // [(xmin - .5f), y, 1]. Then, we'll update as we stride in
        // the +x direction.
        float Hx = MATD_EL(Hinv, 0, 0) * (.5 + (int) xmin) +
            MATD_EL(Hinv, 0, 1) * (y + .5) + MATD_EL(Hinv, 0, 2);
        float Hy = MATD_EL(Hinv, 1, 0) * (.5 + (int) xmin) +
            MATD_EL(Hinv, 1, 1) * (y + .5) + MATD_EL(Hinv, 1, 2);
        float Hh = MATD_EL(Hinv, 2, 0) * (.5 + (int) xmin) +
            MATD_EL(Hinv, 2, 1) * (y + .5) + MATD_EL(Hinv, 2, 2);

        for (int x = xmin; x <= xmax;  x++) {
            // project the pixel center.
            float tx, ty;

            // divide by homogeneous coordinate
            tx = Hx / Hh;
            ty = Hy / Hh;

            // if we move x one pixel to the right, here's what
            // happens to our three pre-normalized coordinates.
            Hx += MATD_EL(Hinv, 0, 0);
            Hy += MATD_EL(Hinv, 1, 0);
            Hh += MATD_EL(Hinv, 2, 0);

            float txa = fabsf((float) tx), tya = fabsf((float) ty);
            float xymax = fmaxf(txa, tya);

//            if (txa >= 1 + wsz || tya >= 1 + wsz)
            if (xymax >= 1 + wsz)
                continue;

            uint8_t v = im->buf[y*im->stride + x];

            // it's within the white border?
//            if (txa >= 1 || tya >= 1) {
            if (xymax >= 1) {
                W1 += v;
                Wn ++;
                continue;
            }

            // it's within the black border?
//            if (txa >= 1 - bsz || tya >= 1 - bsz) {
            if (xymax >= 1 - bsz) {
                B1 += v;
                Bn ++;
                continue;
            }

            // it must be a data bit. We don't do anything with these.
            continue;
        }
    }


    // score = average margin between white and black pixels near border.
    float margin = 1.0 * W1 / Wn - 1.0 * B1 / Bn;
//    printf("margin %f: W1 %f, B1 %f\n", margin, W1, B1);

    return margin;
}

// returns the decision margin. Return < 0 if the detection should be rejected.
float quad_decode(apriltag_family_t *family, image_u8_t *im, struct quad *quad, struct quick_decode_entry *entry, image_u8_t *im_samples)
{
    // decode the tag binary contents by sampling the pixel
    // closest to the center of each bit cell.

    int64_t rcode = 0;

    // how wide do we assume the white border is?
    float white_border = 1.0;

    // We will compute a threshold by sampling known white/black cells around this tag.
    // This sampling is achieved by considering a set of samples along lines.
    //
    // coordinates are given in bit coordinates. ([0, fam->d]).
    //
    // { initial x, initial y, delta x, delta y, WHITE=1 }
    float patterns[] = {
        // left white column
        0 - white_border / 2.0, 0.5,
        0, 1,
        1,

        // left black column
        0 + family->black_border / 2.0, 0.5,
        0, 1,
        0,

        // right white column
        2*family->black_border + family->d + white_border / 2.0, .5,
        0, 1,
        1,

        // right black column
        2*family->black_border + family->d - family->black_border / 2.0, .5,
        0, 1,
        0,

        // top white row
        0.5, -white_border / 2.0,
        1, 0,
        1,

        // top black row
        0.5, family->black_border / 2.0,
        1, 0,
        0,

        // bottom white row
        0.5, 2*family->black_border + family->d + white_border / 2.0,
        1, 0,
        1,

        // bottom black row
        0.5, 2*family->black_border + family->d - family->black_border / 2.0,
        1, 0,
        0

        // XXX float-counts the corners.
    };

    struct graymodel whitemodel, blackmodel;
    graymodel_init(&whitemodel);
    graymodel_init(&blackmodel);

    for (int pattern_idx = 0; pattern_idx < sizeof(patterns)/(5*sizeof(float)); pattern_idx ++) {
        float *pattern = &patterns[pattern_idx * 5];

        int is_white = pattern[4];

        for (int i = 0; i < 2*family->black_border + family->d; i++) {
            float tagx01 = (pattern[0] + i*pattern[2]) / (2*family->black_border + family->d);
            float tagy01 = (pattern[1] + i*pattern[3]) / (2*family->black_border + family->d);

            float tagx = 2*(tagx01-0.5);
            float tagy = 2*(tagy01-0.5);

            float px, py;
            homography_project(quad->H, tagx, tagy, &px, &py);

            // don't round
            int ix = px;
            int iy = py;
            if (ix < 0 || iy < 0 || ix >= im->width || iy >= im->height)
                continue;

            int v = im->buf[iy*im->stride + ix];

            if (im_samples) {
                im_samples->buf[iy*im_samples->stride + ix] = (1-is_white)*255;
            }

            if (is_white)
                graymodel_add(&whitemodel, tagx, tagy, v);
            else
                graymodel_add(&blackmodel, tagx, tagy, v);
        }
    }

    graymodel_solve(&whitemodel);
    graymodel_solve(&blackmodel);

    // XXX Tunable
    if (graymodel_interpolate(&whitemodel, 0, 0) - graymodel_interpolate(&blackmodel, 0, 0) < 0)
        return -1;

    // compute the average decision margin (how far was each bit from
    // the decision boundary?
    //
    // we score this separately for white and black pixels and return
    // the minimum average threshold for black/white pixels. This is
    // to penalize thresholds that are too close to an extreme.
    float black_score = 0, white_score = 0;
    float black_score_count = 1, white_score_count = 1;

    for (int bitidx = 0; bitidx < family->d * family->d; bitidx++) {
        int bitx = bitidx % family->d;
        int bity = bitidx / family->d;

        float tagx01 = (family->black_border + bitx + 0.5) / (2*family->black_border + family->d);
        float tagy01 = (family->black_border + bity + 0.5) / (2*family->black_border + family->d);

        // scale to [-1, 1]
        float tagx = 2*(tagx01-0.5);
        float tagy = 2*(tagy01-0.5);

        float px, py;
        homography_project(quad->H, tagx, tagy, &px, &py);

        rcode = (rcode << 1);

        // don't round.
        int ix = px;
        int iy = py;

        if (ix < 0 || iy < 0 || ix >= im->width || iy >= im->height)
            continue;

        int v = im->buf[iy*im->stride + ix];

        float thresh = (graymodel_interpolate(&blackmodel, tagx, tagy) + graymodel_interpolate(&whitemodel, tagx, tagy)) / 2.0;
        if (v > thresh) {
            white_score += (v - thresh);
            white_score_count ++;
            rcode |= 1;
        } else {
            black_score += (thresh - v);
            black_score_count ++;
        }

        if (im_samples)
            im_samples->buf[iy*im_samples->stride + ix] = (1 - (rcode & 1)) * 255;
    }

    quick_decode_codeword(family, rcode, entry);

    return fmin(white_score / white_score_count, black_score / black_score_count);
}

float score_goodness(apriltag_family_t *family, image_u8_t *im, struct quad *quad, void *user)
{
    return quad_goodness(family, im, quad);
}

float score_decodability(apriltag_family_t *family, image_u8_t *im, struct quad *quad, void *user)
{
    struct quick_decode_entry entry;

    float decision_margin = quad_decode(family, im, quad, &entry, NULL);

    // hamming trumps decision margin; maximum value for decision_margin is 255.
    return decision_margin - entry.hamming*1000;
}

// returns score of best quad
float optimize_quad_generic(apriltag_family_t *family, image_u8_t *im, struct quad *quad0,
                             float *stepsizes, int nstepsizes,
                             float (*score)(apriltag_family_t *family, image_u8_t *im, struct quad *quad, void *user),
                             void *user)
{
    struct quad *best_quad = quad_copy(quad0);
    float best_score = score(family, im, best_quad, user);

    for (int stepsize_idx = 0; stepsize_idx < nstepsizes; stepsize_idx++)  {

        int improved = 1;

        // when we make progress with a particular step size, how many
        // times will we try to perform that same step size again?
        // (max_repeat = 0 means ("don't repeat--- just move to the
        // next step size").
        // XXX Tunable
        int max_repeat = 1;

        for (int repeat = 0; repeat <= max_repeat && improved; repeat++) {

            improved = 0;

            // wiggle point i
            for (int i = 0; i < 4; i++) {

                float stepsize = stepsizes[stepsize_idx];

                // XXX Tunable (really 1 makes the best sense since)
                int nsteps = 1;

                struct quad *this_best_quad = NULL;
                float this_best_score = best_score;

                for (int sx = -nsteps; sx <= nsteps; sx++) {
                    for (int sy = -nsteps; sy <= nsteps; sy++) {
                        if (sx==0 && sy==0)
                            continue;

                        struct quad *this_quad = quad_copy(best_quad);
                        this_quad->p[i][0] = best_quad->p[i][0] + sx*stepsize;
                        this_quad->p[i][1] = best_quad->p[i][1] + sy*stepsize;
                        if (quad_update_homographies(this_quad))
                            continue;

                        float this_score = score(family, im, this_quad, user);

                        if (this_score > this_best_score) {
                            quad_destroy(this_best_quad);

                            this_best_quad = this_quad;
                            this_best_score = this_score;
                        } else {
                            quad_destroy(this_quad);
                        }
                    }
                }

                if (this_best_score > best_score) {
                    quad_destroy(best_quad);
                    best_quad = this_best_quad;
                    best_score = this_best_score;
                    improved = 1;
                }
            }
        }
    }

    matd_destroy(quad0->H);
    matd_destroy(quad0->Hinv);
    memcpy(quad0, best_quad, sizeof(struct quad)); // copy pointers
    free(best_quad);
    return best_score;
}

static void refine_edges(apriltag_detector_t *td, image_u8_t *im_orig, struct quad *quad)
{
    float lines[4][4]; // for each line, [Ex Ey nx ny]

    for (int edge = 0; edge < 4; edge++) {
        int a = edge, b = (edge + 1) & 3; // indices of the end points.

        // compute the normal to the current line estimate
        float nx = quad->p[b][1] - quad->p[a][1];
        float ny = -quad->p[b][0] + quad->p[a][0];
        float mag = sqrt(nx*nx + ny*ny);
        nx /= mag;
        ny /= mag;

        // we will now fit a NEW line by sampling points near
        // our original line that have large gradients. On really big tags,
        // we're willing to sample more to get an even better estimate.
        int nsamples = imax(16, mag / 8); // XXX tunable

        // stats for fitting a line...
        float Mx = 0, My = 0, Mxx = 0, Mxy = 0, Myy = 0, N = 0;

        for (int s = 0; s < nsamples; s++) {
            // compute a point along the line... Note, we're avoiding
            // sampling *right* at the corners, since those points are
            // the least reliable.
            float alpha = (1.0 + s) / (nsamples + 1);
            float x0 = alpha*quad->p[a][0] + (1-alpha)*quad->p[b][0];
            float y0 = alpha*quad->p[a][1] + (1-alpha)*quad->p[b][1];

            // search along the normal to this line, looking at the
            // gradients along the way. We're looking for a strong
            // response.
            float Mn = 0;
            float Mcount = 0;

            // XXX tunable: how far to search?  We want to search far
            // enough that we find the best edge, but not so far that
            // we hit other edges that aren't part of the tag. We
            // shouldn't ever have to search more than quad_decimate,
            // since otherwise we would (ideally) have started our
            // search on another pixel in the first place. Likewise,
            // for very small tags, we don't want the range to be too
            // big.
            float range = 1.0 + 1;

            // XXX tunable step size.
            for (float n = -range; n <= range; n +=  0.25) {
                // Because of the guaranteed winding order of the
                // points in the quad, we will start inside the white
                // portion of the quad and work our way outward.
                //
                // sample to points (x1,y1) and (x2,y2) XXX tunable:
                // how far +/- to look? Small values compute the
                // gradient more precisely, but are more sensitive to
                // noise.
                float grange = 1;
                int x1 = x0 + (n + grange)*nx;
                int y1 = y0 + (n + grange)*ny;
                if (x1 < 0 || x1 >= im_orig->width || y1 < 0 || y1 >= im_orig->height)
                    continue;

                int x2 = x0 + (n - grange)*nx;
                int y2 = y0 + (n - grange)*ny;
                if (x2 < 0 || x2 >= im_orig->width || y2 < 0 || y2 >= im_orig->height)
                    continue;

                int g1 = im_orig->buf[y1*im_orig->stride + x1];
                int g2 = im_orig->buf[y2*im_orig->stride + x2];

                if (g1 < g2) // reject points whose gradient is "backwards". They can only hurt us.
                    continue;

                float weight = (g2 - g1)*(g2 - g1); // XXX tunable. What shape for weight=f(g2-g1)?

                // compute weighted average of the gradient at this point.
                Mn += weight*n;
                Mcount += weight;
            }

            // what was the average point along the line?
            if (Mcount == 0)
                continue;

            float n0 = Mn / Mcount;

            // where is the point along the line?
            float bestx = x0 + n0*nx;
            float besty = y0 + n0*ny;

            // update our line fit statistics
            Mx += bestx;
            My += besty;
            Mxx += bestx*bestx;
            Mxy += bestx*besty;
            Myy += besty*besty;
            N++;
        }

        // fit a line
        float Ex = Mx / N, Ey = My / N;
        float Cxx = Mxx / N - Ex*Ex;
        float Cxy = Mxy / N - Ex*Ey;
        float Cyy = Myy / N - Ey*Ey;

        float normal_theta = .5 * atan2f(-2*Cxy, (Cyy - Cxx));
        nx = cosf(normal_theta);
        ny = sinf(normal_theta);
        lines[edge][0] = Ex;
        lines[edge][1] = Ey;
        lines[edge][2] = nx;
        lines[edge][3] = ny;
    }

    // now refit the corners of the quad
    for (int i = 0; i < 4; i++) {

        // solve for the intersection of lines (i) and (i+1)&3.
        float A00 =  lines[i][3],  A01 = -lines[(i+1)&3][3];
        float A10 =  -lines[i][2],  A11 = lines[(i+1)&3][2];
        float B0 = -lines[i][0] + lines[(i+1)&3][0];
        float B1 = -lines[i][1] + lines[(i+1)&3][1];

        float det = A00 * A11 - A10 * A01;

        // inverse.
        if (fabs(det) > 0.001) {
            // solve
            float W00 = A11 / det, W01 = -A01 / det;

            float L0 = W00*B0 + W01*B1;

            // compute intersection
            quad->p[i][0] = lines[i][0] + L0*A00;
            quad->p[i][1] = lines[i][1] + L0*A10;
        } else {
            // this is a bad sign. We'll just keep the corner we had.
//            printf("bad det: %15f %15f %15f %15f %15f\n", A00, A11, A10, A01, det);
        }
    }
}

void apriltag_detection_destroy(apriltag_detection_t *det)
{
    if (det == NULL)
        return;

    matd_destroy(det->H);
    free(det);
}

int prefer_smaller(int pref, float q0, float q1)
{
    if (pref)     // already prefer something? exit.
        return pref;

    if (q0 < q1)
        return -1; // we now prefer q0
    if (q1 < q0)
        return 1; // we now prefer q1

    // no preference
    return 0;
}

zarray_t *apriltag_detector_detect(apriltag_detector_t *td, image_u8_t *im_orig)
{
    if (zarray_size(td->tag_families) == 0) {
        zarray_t *s = zarray_create(sizeof(apriltag_detection_t*));
        printf("apriltag.c: No tag families enabled.");
        return s;
    }

    ///////////////////////////////////////////////////////////
    // Step 1. Detect quads according to requested image decimation
    // and blurring parameters.

//    zarray_t *quads = apriltag_quad_gradient(td, im_orig);
    zarray_t *quads = apriltag_quad_thresh(td, im_orig, false);

    zarray_t *detections = zarray_create(sizeof(apriltag_detection_t*));

    td->nquads = zarray_size(quads);

    ////////////////////////////////////////////////////////////////
    // Step 2. Decode tags from each quad.
    if (1) {
        for (int i = 0; i < zarray_size(quads); i++) {
            struct quad *quad_original;
            zarray_get_volatile(quads, i, &quad_original);

            // refine edges is not dependent upon the tag family, thus
            // apply this optimization BEFORE the other work.
            //if (td->quad_decimate > 1 && td->refine_edges) {
            if (td->refine_edges) {
                refine_edges(td, im_orig, quad_original);
            }

            // make sure the homographies are computed...
            if (quad_update_homographies(quad_original))
                continue;

            for (int famidx = 0; famidx < zarray_size(td->tag_families); famidx++) {
                apriltag_family_t *family;
                zarray_get(td->tag_families, famidx, &family);

                float goodness = 0;

                // since the geometry of tag families can vary, start any
                // optimization process over with the original quad.
                struct quad *quad = quad_copy(quad_original);

                // improve the quad corner positions by minimizing the
                // variance within each intra-bit area.
                if (td->refine_pose) {
                    // NB: We potentially step an integer
                    // number of times in each direction. To make each
                    // sample as useful as possible, the step sizes should
                    // not be integer multiples of each other. (I.e.,
                    // probably don't use 1, 0.5, 0.25, etc.)

                    // XXX Tunable
                    float stepsizes[] = { 1, .4, .16, .064 };
                    int nstepsizes = sizeof(stepsizes)/sizeof(float);

                    goodness = optimize_quad_generic(family, im_orig, quad, stepsizes, nstepsizes, score_goodness, NULL);
                }

                if (td->refine_decode) {
                    // this optimizes decodability, but we don't report
                    // that value to the user.  (so discard return value.)
                    // XXX Tunable
                    float stepsizes[] = { .4 };
                    int nstepsizes = sizeof(stepsizes)/sizeof(float);

                    optimize_quad_generic(family, im_orig, quad, stepsizes, nstepsizes, score_decodability, NULL);
                }

                struct quick_decode_entry entry;

                float decision_margin = quad_decode(family, im_orig, quad, &entry, NULL);

                if (entry.hamming < 255 && decision_margin >= 0) {
                    apriltag_detection_t *det = calloc(1, sizeof(apriltag_detection_t));

                    det->family = family;
                    det->id = entry.id;
                    det->hamming = entry.hamming;
                    det->goodness = goodness;
                    det->decision_margin = decision_margin;

                    float theta = -entry.rotation * M_PI / 2.0;
                    float c = cos(theta), s = sin(theta);

                    // Fix the rotation of our homography to properly orient the tag
                    matd_t *R = matd_create(3,3);
                    MATD_EL(R, 0, 0) = c;
                    MATD_EL(R, 0, 1) = -s;
                    MATD_EL(R, 1, 0) = s;
                    MATD_EL(R, 1, 1) = c;
                    MATD_EL(R, 2, 2) = 1;

                    matd_t *RHMirror = matd_create(3,3);
                    MATD_EL(RHMirror, 0, 0) = entry.hmirror ? -1 : 1;
                    MATD_EL(RHMirror, 1, 1) = 1;
                    MATD_EL(RHMirror, 2, 2) = entry.hmirror ? -1 : 1;

                    matd_t *RVFlip = matd_create(3,3);
                    MATD_EL(RVFlip, 0, 0) = 1;
                    MATD_EL(RVFlip, 1, 1) = entry.vflip ? -1 : 1;
                    MATD_EL(RVFlip, 2, 2) = entry.vflip ? -1 : 1;

                    det->H = matd_op("M*M*M*M", quad->H, R, RHMirror, RVFlip);

                    matd_destroy(R);
                    matd_destroy(RHMirror);
                    matd_destroy(RVFlip);

                    homography_project(det->H, 0, 0, &det->c[0], &det->c[1]);

                    // [-1, -1], [1, -1], [1, 1], [-1, 1], Desired points
                    // [-1, 1], [1, 1], [1, -1], [-1, -1], FLIP Y
                    // adjust the points in det->p so that they correspond to
                    // counter-clockwise around the quad, starting at -1,-1.
                    for (int i = 0; i < 4; i++) {
                        int tcx = (i == 1 || i == 2) ? 1 : -1;
                        int tcy = (i < 2) ? 1 : -1;

                        float p[2];

                        homography_project(det->H, tcx, tcy, &p[0], &p[1]);

                        det->p[i][0] = p[0];
                        det->p[i][1] = p[1];
                    }

                    zarray_add(detections, &det);
                }

                quad_destroy(quad);
            }
        }
    }

    ////////////////////////////////////////////////////////////////
    // Step 3. Reconcile detections--- don't report the same tag more
    // than once. (Allow non-overlapping duplicate detections.)
    if (1) {
        zarray_t *poly0 = g2d_polygon_create_zeros(4);
        zarray_t *poly1 = g2d_polygon_create_zeros(4);

        for (int i0 = 0; i0 < zarray_size(detections); i0++) {

            apriltag_detection_t *det0;
            zarray_get(detections, i0, &det0);

            for (int k = 0; k < 4; k++)
                zarray_set(poly0, k, det0->p[k], NULL);

            for (int i1 = i0+1; i1 < zarray_size(detections); i1++) {

                apriltag_detection_t *det1;
                zarray_get(detections, i1, &det1);

                if (det0->id != det1->id || det0->family != det1->family)
                    continue;

                for (int k = 0; k < 4; k++)
                    zarray_set(poly1, k, det1->p[k], NULL);

                if (g2d_polygon_overlaps_polygon(poly0, poly1)) {
                    // the tags overlap. Delete one, keep the other.

                    int pref = 0; // 0 means undecided which one we'll keep.
                    pref = prefer_smaller(pref, det0->hamming, det1->hamming);     // want small hamming
                    pref = prefer_smaller(pref, -det0->decision_margin, -det1->decision_margin);      // want bigger margins
                    pref = prefer_smaller(pref, -det0->goodness, -det1->goodness); // want bigger goodness

                    // if we STILL don't prefer one detection over the other, then pick
                    // any deterministic criterion.
                    for (int i = 0; i < 4; i++) {
                        pref = prefer_smaller(pref, det0->p[i][0], det1->p[i][0]);
                        pref = prefer_smaller(pref, det0->p[i][1], det1->p[i][1]);
                    }

                    if (pref == 0) {
                        // at this point, we should only be undecided if the tag detections
                        // are *exactly* the same. How would that happen?
                        // printf("uh oh, no preference for overlappingdetection\n");
                    }

                    if (pref < 0) {
                        // keep det0, destroy det1
                        apriltag_detection_destroy(det1);
                        zarray_remove_index(detections, i1, 1);
                        i1--; // retry the same index
                        goto retry1;
                    } else {
                        // keep det1, destroy det0
                        apriltag_detection_destroy(det0);
                        zarray_remove_index(detections, i0, 1);
                        i0--; // retry the same index.
                        goto retry0;
                    }
                }

              retry1: ;
            }

          retry0: ;
        }

        zarray_destroy(poly0);
        zarray_destroy(poly1);
    }

    for (int i = 0; i < zarray_size(quads); i++) {
        struct quad *quad;
        zarray_get_volatile(quads, i, &quad);
        matd_destroy(quad->H);
        matd_destroy(quad->Hinv);
    }

    zarray_destroy(quads);

    zarray_sort(detections, detection_compare_function);

    return detections;
}


// Call this method on each of the tags returned by apriltag_detector_detect
void apriltag_detections_destroy(zarray_t *detections)
{
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        apriltag_detection_destroy(det);
    }

    zarray_destroy(detections);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

void imlib_find_apriltags(list_t *out, image_t *ptr, rectangle_t *roi, apriltag_families_t families,
                          float fx, float fy, float cx, float cy)
{
    // Frame Buffer Memory Usage...
    // -> GRAYSCALE Input Image = w*h*1
    // -> GRAYSCALE Threhsolded Image = w*h*1
    // -> UnionFind = w*h*2 (+w*h*1 for hash table)
    size_t resolution = roi->w * roi->h;
    size_t fb_alloc_need = resolution * (1 + 1 + 2 + 1); // read above...
    umm_init_x(((fb_avail() - fb_alloc_need) / resolution) * resolution);
    apriltag_detector_t *td = apriltag_detector_create();

    if (families & TAG16H5) {
        apriltag_detector_add_family(td, (apriltag_family_t *) &tag16h5);
    }

    if (families & TAG25H7) {
        apriltag_detector_add_family(td, (apriltag_family_t *) &tag25h7);
    }

    if (families & TAG25H9) {
        apriltag_detector_add_family(td, (apriltag_family_t *) &tag25h9);
    }

    if (families & TAG36H10) {
        apriltag_detector_add_family(td, (apriltag_family_t *) &tag36h10);
    }

    if (families & TAG36H11) {
        apriltag_detector_add_family(td, (apriltag_family_t *) &tag36h11);
    }

    if (families & ARTOOLKIT) {
        apriltag_detector_add_family(td, (apriltag_family_t *) &artoolkit);
    }

    uint8_t *grayscale_image = fb_alloc(roi->w * roi->h, FB_ALLOC_NO_HINT);

    image_u8_t im;
    im.width = roi->w;
    im.height = roi->h;
    im.stride = roi->w;
    im.buf = grayscale_image;

    switch(ptr->bpp) {
        case IMAGE_BPP_BINARY: {
            for (int y = roi->y, yy = roi->y + roi->h; y < yy; y++) {
                uint32_t *row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(ptr, y);
                for (int x = roi->x, xx = roi->x + roi->w; x < xx; x++) {
                    *(grayscale_image++) = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, x));
                }
            }
            break;
        }
        case IMAGE_BPP_GRAYSCALE: {
            for (int y = roi->y, yy = roi->y + roi->h; y < yy; y++) {
                uint8_t *row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(ptr, y);
                for (int x = roi->x, xx = roi->x + roi->w; x < xx; x++) {
                    *(grayscale_image++) = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, x);
                }
            }
            break;
        }
        case IMAGE_BPP_RGB565: {
            for (int y = roi->y, yy = roi->y + roi->h; y < yy; y++) {
                uint16_t *row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(ptr, y);
                for (int x = roi->x, xx = roi->x + roi->w; x < xx; x++) {
                    *(grayscale_image++) = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, x));
                }
            }
            break;
        }
        default: {
            memset(grayscale_image, 0, roi->w * roi->h);
            break;
        }
    }

    zarray_t *detections = apriltag_detector_detect(td, &im);
    list_init(out, sizeof(find_apriltags_list_lnk_data_t));

    for (int i = 0, j = zarray_size(detections); i < j; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        find_apriltags_list_lnk_data_t lnk_data;
        rectangle_init(&(lnk_data.rect), fast_roundf(det->p[0][0]) + roi->x, fast_roundf(det->p[0][1]) + roi->y, 0, 0);

        for (size_t k = 1, l = (sizeof(det->p) / sizeof(det->p[0])); k < l; k++) {
            rectangle_t temp;
            rectangle_init(&temp, fast_roundf(det->p[k][0]) + roi->x, fast_roundf(det->p[k][1]) + roi->y, 0, 0);
            rectangle_united(&(lnk_data.rect), &temp);
        }

        // Add corners...
        lnk_data.corners[0].x = fast_roundf(det->p[3][0]) + roi->x; // top-left
        lnk_data.corners[0].y = fast_roundf(det->p[3][1]) + roi->y; // top-left
        lnk_data.corners[1].x = fast_roundf(det->p[2][0]) + roi->x; // top-right
        lnk_data.corners[1].y = fast_roundf(det->p[2][1]) + roi->y; // top-right
        lnk_data.corners[2].x = fast_roundf(det->p[1][0]) + roi->x; // bottom-right
        lnk_data.corners[2].y = fast_roundf(det->p[1][1]) + roi->y; // bottom-right
        lnk_data.corners[3].x = fast_roundf(det->p[0][0]) + roi->x; // bottom-left
        lnk_data.corners[3].y = fast_roundf(det->p[0][1]) + roi->y; // bottom-left

        lnk_data.id = det->id;
        lnk_data.family = 0;

        if(det->family == &tag16h5) {
            lnk_data.family |= TAG16H5;
        }

        if(det->family == &tag25h7) {
            lnk_data.family |= TAG25H7;
        }

        if(det->family == &tag25h9) {
            lnk_data.family |= TAG25H9;
        }

        if(det->family == &tag36h10) {
            lnk_data.family |= TAG36H10;
        }

        if(det->family == &tag36h11) {
            lnk_data.family |= TAG36H11;
        }

        if(det->family == &artoolkit) {
            lnk_data.family |= ARTOOLKIT;
        }

        lnk_data.hamming = det->hamming;
        lnk_data.centroid.x = fast_roundf(det->c[0]) + roi->x;
        lnk_data.centroid.y = fast_roundf(det->c[1]) + roi->y;
        lnk_data.goodness = det->goodness / 255.0; // scale to [0:1]
        lnk_data.decision_margin = det->decision_margin / 255.0; // scale to [0:1]

        matd_t *pose = homography_to_pose(det->H, -fx, fy, cx, cy);

        lnk_data.x_translation = MATD_EL(pose, 0, 3);
        lnk_data.y_translation = MATD_EL(pose, 1, 3);
        lnk_data.z_translation = MATD_EL(pose, 2, 3);
        lnk_data.x_rotation = fast_atan2f(MATD_EL(pose, 2, 1), MATD_EL(pose, 2, 2));
        lnk_data.y_rotation = fast_atan2f(-MATD_EL(pose, 2, 0), fast_sqrtf(sq(MATD_EL(pose, 2, 1)) + sq(MATD_EL(pose, 2, 2))));
        lnk_data.z_rotation = fast_atan2f(MATD_EL(pose, 1, 0), MATD_EL(pose, 0, 0));

        matd_destroy(pose);

        list_push_back(out, &lnk_data);
    }

    apriltag_detections_destroy(detections);
    fb_free(); // grayscale_image;
    apriltag_detector_destroy(td);
    fb_free(); // umm_init_x();
}

#ifdef IMLIB_ENABLE_FIND_RECTS
void imlib_find_rects(list_t *out, image_t *ptr, rectangle_t *roi, uint32_t threshold)
{
    // Frame Buffer Memory Usage...
    // -> GRAYSCALE Input Image = w*h*1
    // -> GRAYSCALE Threhsolded Image = w*h*1
    // -> UnionFind = w*h*4 (+w*h*2 for hash table)
    size_t resolution = roi->w * roi->h;
    size_t fb_alloc_need = resolution * (1 + 1 + 4 + 2); // read above...
    umm_init_x(((fb_avail() - fb_alloc_need) / resolution) * resolution);
    apriltag_detector_t *td = apriltag_detector_create();

    image_t img;
    img.w = roi->w;
    img.h = roi->h;
    img.bpp = IMAGE_BPP_GRAYSCALE;
    img.data = fb_alloc(image_size(&img), FB_ALLOC_NO_HINT);
    imlib_draw_image(&img, ptr, 0, 0, 1.f, 1.f, roi, -1, 256, NULL, NULL, 0, NULL, NULL);

    image_u8_t im;
    im.width = roi->w;
    im.height = roi->h;
    im.stride = roi->w;
    im.buf = img.data;

    ///////////////////////////////////////////////////////////
    // Detect quads according to requested image decimation
    // and blurring parameters.

//    zarray_t *detections = apriltag_quad_gradient(td, &im, true);
    zarray_t *detections = apriltag_quad_thresh(td, &im, true);

    td->nquads = zarray_size(detections);

    ////////////////////////////////////////////////////////////////
    // Decode tags from each quad.
    if (1) {
        for (int i = 0; i < zarray_size(detections); i++) {
            struct quad *quad_original;
            zarray_get_volatile(detections, i, &quad_original);

            // refine edges is not dependent upon the tag family, thus
            // apply this optimization BEFORE the other work.
            //if (td->quad_decimate > 1 && td->refine_edges) {
            if (td->refine_edges) {
                refine_edges(td, &im, quad_original);
            }

            // make sure the homographies are computed...
            if (quad_update_homographies(quad_original))
                continue;
        }
    }

    ////////////////////////////////////////////////////////////////
    // Reconcile detections--- don't report the same tag more
    // than once. (Allow non-overlapping duplicate detections.)
    if (1) {
        zarray_t *poly0 = g2d_polygon_create_zeros(4);
        zarray_t *poly1 = g2d_polygon_create_zeros(4);

        for (int i0 = 0; i0 < zarray_size(detections); i0++) {

            struct quad *det0;
            zarray_get_volatile(detections, i0, &det0);

            for (int k = 0; k < 4; k++)
                zarray_set(poly0, k, det0->p[k], NULL);

            for (int i1 = i0+1; i1 < zarray_size(detections); i1++) {

                struct quad *det1;
                zarray_get_volatile(detections, i1, &det1);

                for (int k = 0; k < 4; k++)
                    zarray_set(poly1, k, det1->p[k], NULL);

                if (g2d_polygon_overlaps_polygon(poly0, poly1)) {
                    // the tags overlap. Delete one, keep the other.

                    int pref = 0; // 0 means undecided which one we'll keep.

                    // if we STILL don't prefer one detection over the other, then pick
                    // any deterministic criterion.
                    for (int i = 0; i < 4; i++) {
                        pref = prefer_smaller(pref, det0->p[i][0], det1->p[i][0]);
                        pref = prefer_smaller(pref, det0->p[i][1], det1->p[i][1]);
                    }

                    if (pref == 0) {
                        // at this point, we should only be undecided if the tag detections
                        // are *exactly* the same. How would that happen?
                        // printf("uh oh, no preference for overlappingdetection\n");
                    }

                    if (pref < 0) {
                        // keep det0, destroy det1
                        matd_destroy(det1->H);
                        matd_destroy(det1->Hinv);
                        zarray_remove_index(detections, i1, 1);
                        i1--; // retry the same index
                        goto retry1;
                    } else {
                        // keep det1, destroy det0
                        matd_destroy(det0->H);
                        matd_destroy(det0->Hinv);
                        zarray_remove_index(detections, i0, 1);
                        i0--; // retry the same index.
                        goto retry0;
                    }
                }

              retry1: ;
            }

          retry0: ;
        }

        zarray_destroy(poly0);
        zarray_destroy(poly1);
    }

    list_init(out, sizeof(find_rects_list_lnk_data_t));

    const int r_diag_len = fast_roundf(fast_sqrtf((roi->w * roi->w) + (roi->h * roi->h))) * 2;
    int *theta_buffer = fb_alloc(sizeof(int) * r_diag_len, FB_ALLOC_NO_HINT);
    uint32_t *mag_buffer = fb_alloc(sizeof(uint32_t) * r_diag_len, FB_ALLOC_NO_HINT);
    point_t *point_buffer = fb_alloc(sizeof(point_t) * r_diag_len, FB_ALLOC_NO_HINT);

    for (int i = 0, j = zarray_size(detections); i < j; i++) {
        struct quad *det;
        zarray_get_volatile(detections, i, &det);

        line_t lines[4];
        lines[0].x1 = fast_roundf(det->p[0][0]) + roi->x; lines[0].y1 = fast_roundf(det->p[0][1]) + roi->y;
        lines[0].x2 = fast_roundf(det->p[1][0]) + roi->x; lines[0].y2 = fast_roundf(det->p[1][1]) + roi->y;
        lines[1].x1 = fast_roundf(det->p[1][0]) + roi->x; lines[1].y1 = fast_roundf(det->p[1][1]) + roi->y;
        lines[1].x2 = fast_roundf(det->p[2][0]) + roi->x; lines[1].y2 = fast_roundf(det->p[2][1]) + roi->y;
        lines[2].x1 = fast_roundf(det->p[2][0]) + roi->x; lines[2].y1 = fast_roundf(det->p[2][1]) + roi->y;
        lines[2].x2 = fast_roundf(det->p[3][0]) + roi->x; lines[2].y2 = fast_roundf(det->p[3][1]) + roi->y;
        lines[3].x1 = fast_roundf(det->p[3][0]) + roi->x; lines[3].y1 = fast_roundf(det->p[3][1]) + roi->y;
        lines[3].x2 = fast_roundf(det->p[0][0]) + roi->x; lines[3].y2 = fast_roundf(det->p[0][1]) + roi->y;

        uint32_t magnitude = 0;

        for (int i = 0; i < 4; i++) {
            if(!lb_clip_line(&lines[i], 0, 0, roi->w, roi->h)) {
                continue;
            }

            size_t index = trace_line(ptr, &lines[i], theta_buffer, mag_buffer, point_buffer);

            for (int j = 0; j < index; j++) {
                magnitude += mag_buffer[j];
            }
        }

        if (magnitude < threshold) {
            continue;
        }

        find_rects_list_lnk_data_t lnk_data;
        rectangle_init(&(lnk_data.rect), fast_roundf(det->p[0][0]) + roi->x, fast_roundf(det->p[0][1]) + roi->y, 0, 0);

        for (size_t k = 1, l = (sizeof(det->p) / sizeof(det->p[0])); k < l; k++) {
            rectangle_t temp;
            rectangle_init(&temp, fast_roundf(det->p[k][0]) + roi->x, fast_roundf(det->p[k][1]) + roi->y, 0, 0);
            rectangle_united(&(lnk_data.rect), &temp);
        }

        // Add corners...
        lnk_data.corners[0].x = fast_roundf(det->p[3][0]) + roi->x; // top-left
        lnk_data.corners[0].y = fast_roundf(det->p[3][1]) + roi->y; // top-left
        lnk_data.corners[1].x = fast_roundf(det->p[2][0]) + roi->x; // top-right
        lnk_data.corners[1].y = fast_roundf(det->p[2][1]) + roi->y; // top-right
        lnk_data.corners[2].x = fast_roundf(det->p[1][0]) + roi->x; // bottom-right
        lnk_data.corners[2].y = fast_roundf(det->p[1][1]) + roi->y; // bottom-right
        lnk_data.corners[3].x = fast_roundf(det->p[0][0]) + roi->x; // bottom-left
        lnk_data.corners[3].y = fast_roundf(det->p[0][1]) + roi->y; // bottom-left

        lnk_data.magnitude = magnitude;

        list_push_back(out, &lnk_data);
    }

    fb_free(); // point_buffer
    fb_free(); // mag_buffer
    fb_free(); // theta_buffer

    zarray_destroy(detections);
    fb_free(); // grayscale_image;
    apriltag_detector_destroy(td);
    fb_free(); // umm_init_x();
}
#endif //IMLIB_ENABLE_FIND_RECTS

#ifdef IMLIB_ENABLE_ROTATION_CORR
// http://jepsonsblog.blogspot.com/2012/11/rotation-in-3d-using-opencvs.html
void imlib_rotation_corr(image_t *img, float x_rotation, float y_rotation, float z_rotation,
                         float x_translation, float y_translation,
                         float zoom, float fov, float *corners)
{
    // Create a tmp copy of the image to pull pixels from.
    size_t size = image_size(img);
    void *data = fb_alloc(size, FB_ALLOC_NO_HINT);
    memcpy(data, img->data, size);
    memset(img->data, 0, size);

    umm_init_x(fb_avail());

    int w = img->w;
    int h = img->h;
    float z = (fast_sqrtf((w * w) + (h * h)) / 2) / tanf(fov / 2);
    float z_z = z * zoom;

    matd_t *A1 = matd_create(4, 3);
    MATD_EL(A1, 0, 0) = 1;  MATD_EL(A1, 0, 1) = 0;  MATD_EL(A1, 0, 2) = -w / 2;
    MATD_EL(A1, 1, 0) = 0;  MATD_EL(A1, 1, 1) = 1;  MATD_EL(A1, 1, 2) = -h / 2;
    MATD_EL(A1, 2, 0) = 0;  MATD_EL(A1, 2, 1) = 0;  MATD_EL(A1, 2, 2) = 0;
    MATD_EL(A1, 3, 0) = 0;  MATD_EL(A1, 3, 1) = 0;  MATD_EL(A1, 3, 2) = 1; // needed for z translation

    matd_t *RX = matd_create(4, 4);
    MATD_EL(RX, 0, 0) = 1;  MATD_EL(RX, 0, 1) = 0;                  MATD_EL(RX, 0, 2) = 0;                  MATD_EL(RX, 0, 3) = 0;
    MATD_EL(RX, 1, 0) = 0;  MATD_EL(RX, 1, 1) = +cosf(x_rotation);  MATD_EL(RX, 1, 2) = -sinf(x_rotation);  MATD_EL(RX, 1, 3) = 0;
    MATD_EL(RX, 2, 0) = 0;  MATD_EL(RX, 2, 1) = +sinf(x_rotation);  MATD_EL(RX, 2, 2) = +cosf(x_rotation);  MATD_EL(RX, 2, 3) = 0;
    MATD_EL(RX, 3, 0) = 0;  MATD_EL(RX, 3, 1) = 0;                  MATD_EL(RX, 3, 2) = 0;                  MATD_EL(RX, 3, 3) = 1;

    matd_t *RY = matd_create(4, 4);
    MATD_EL(RY, 0, 0) = +cosf(y_rotation);  MATD_EL(RY, 0, 1) = 0;  MATD_EL(RY, 0, 2) = -sinf(y_rotation);  MATD_EL(RY, 0, 3) = 0;
    MATD_EL(RY, 1, 0) = 0;                  MATD_EL(RY, 1, 1) = 1;  MATD_EL(RY, 1, 2) = 0;                  MATD_EL(RY, 1, 3) = 0;
    MATD_EL(RY, 2, 0) = +sinf(y_rotation);  MATD_EL(RY, 2, 1) = 0;  MATD_EL(RY, 2, 2) = +cosf(y_rotation);  MATD_EL(RY, 2, 3) = 0;
    MATD_EL(RY, 3, 0) = 0;                  MATD_EL(RY, 3, 1) = 0;  MATD_EL(RY, 3, 2) = 0;                  MATD_EL(RY, 3, 3) = 1;

    matd_t *RZ = matd_create(4, 4);
    MATD_EL(RZ, 0, 0) = +cosf(z_rotation);  MATD_EL(RZ, 0, 1) = -sinf(z_rotation);  MATD_EL(RZ, 0, 2) = 0;  MATD_EL(RZ, 0, 3) = 0;
    MATD_EL(RZ, 1, 0) = +sinf(z_rotation);  MATD_EL(RZ, 1, 1) = +cosf(z_rotation);  MATD_EL(RZ, 1, 2) = 0;  MATD_EL(RZ, 1, 3) = 0;
    MATD_EL(RZ, 2, 0) = 0;                  MATD_EL(RZ, 2, 1) = 0;                  MATD_EL(RZ, 2, 2) = 1;  MATD_EL(RZ, 2, 3) = 0;
    MATD_EL(RZ, 3, 0) = 0;                  MATD_EL(RZ, 3, 1) = 0;                  MATD_EL(RZ, 3, 2) = 0;  MATD_EL(RZ, 3, 3) = 1;

    matd_t *R = matd_op("M*M*M", RX, RY, RZ);

    matd_t *T = matd_create(4, 4);
    MATD_EL(T, 0, 0) = 1;   MATD_EL(T, 0, 1) = 0;   MATD_EL(T, 0, 2) = 0;   MATD_EL(T, 0, 3) = x_translation;
    MATD_EL(T, 1, 0) = 0;   MATD_EL(T, 1, 1) = 1;   MATD_EL(T, 1, 2) = 0;   MATD_EL(T, 1, 3) = y_translation;
    MATD_EL(T, 2, 0) = 0;   MATD_EL(T, 2, 1) = 0;   MATD_EL(T, 2, 2) = 1;   MATD_EL(T, 2, 3) = z;
    MATD_EL(T, 3, 0) = 0;   MATD_EL(T, 3, 1) = 0;   MATD_EL(T, 3, 2) = 0;   MATD_EL(T, 3, 3) = 1;

    matd_t *A2 = matd_create(3, 4);
    MATD_EL(A2, 0, 0) = z_z;    MATD_EL(A2, 0, 1) = 0;      MATD_EL(A2, 0, 2) = w / 2;   MATD_EL(A2, 0, 3) = 0;
    MATD_EL(A2, 1, 0) = 0;      MATD_EL(A2, 1, 1) = z_z;    MATD_EL(A2, 1, 2) = h / 2;   MATD_EL(A2, 1, 3) = 0;
    MATD_EL(A2, 2, 0) = 0;      MATD_EL(A2, 2, 1) = 0;      MATD_EL(A2, 2, 2) = 1;       MATD_EL(A2, 2, 3) = 0;

    matd_t *T1 = matd_op("M*M", R, A1);
    matd_t *T2 = matd_op("M*M", T, T1);
    matd_t *T3 = matd_op("M*M", A2, T2);
    matd_t *T4 = matd_inverse(T3);

    if (T4 && corners) {
        float corr[4];
        zarray_t *correspondences = zarray_create(sizeof(float[4]));

        corr[0] = 0;
        corr[1] = 0;
        corr[2] = corners[0];
        corr[3] = corners[1];
        zarray_add(correspondences, &corr);

        corr[0] = w - 1;
        corr[1] = 0;
        corr[2] = corners[2];
        corr[3] = corners[3];
        zarray_add(correspondences, &corr);

        corr[0] = w - 1;
        corr[1] = h - 1;
        corr[2] = corners[4];
        corr[3] = corners[5];
        zarray_add(correspondences, &corr);

        corr[0] = 0;
        corr[1] = h - 1;
        corr[2] = corners[6];
        corr[3] = corners[7];
        zarray_add(correspondences, &corr);

        matd_t *H = homography_compute(correspondences, HOMOGRAPHY_COMPUTE_FLAG_INVERSE);

        if (!H) { // try again...
            H = homography_compute(correspondences, HOMOGRAPHY_COMPUTE_FLAG_SVD);
        }

        if (H) {
            matd_t *T5 = matd_op("M*M", H, T4);
            matd_destroy(H);
            matd_destroy(T4);
            T4 = T5;
        }

        zarray_destroy(correspondences);
    }

    if (T4) {
        float T4_00 = MATD_EL(T4, 0, 0), T4_01 = MATD_EL(T4, 0, 1), T4_02 = MATD_EL(T4, 0, 2);
        float T4_10 = MATD_EL(T4, 1, 0), T4_11 = MATD_EL(T4, 1, 1), T4_12 = MATD_EL(T4, 1, 2);
        float T4_20 = MATD_EL(T4, 2, 0), T4_21 = MATD_EL(T4, 2, 1), T4_22 = MATD_EL(T4, 2, 2);

        if ((fast_fabsf(T4_20) < MATD_EPS) && (fast_fabsf(T4_21) < MATD_EPS)) { // warp affine
            T4_00 /= T4_22;
            T4_01 /= T4_22;
            T4_02 /= T4_22;
            T4_10 /= T4_22;
            T4_11 /= T4_22;
            T4_12 /= T4_22;
            switch(img->bpp) {
                case IMAGE_BPP_BINARY: {
                    uint32_t *tmp = (uint32_t *) data;

                    for (int y = 0, yy = h; y < yy; y++) {
                        uint32_t *row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(img, y);
                        for (int x = 0, xx = w; x < xx; x++) {
                            int sourceX = fast_roundf(T4_00*x + T4_01*y + T4_02);
                            int sourceY = fast_roundf(T4_10*x + T4_11*y + T4_12);

                            if ((0 <= sourceX) && (sourceX < w) && (0 <= sourceY) && (sourceY < h)) {
                                uint32_t *ptr = tmp + (((w + UINT32_T_MASK) >> UINT32_T_SHIFT) * sourceY);
                                int pixel = IMAGE_GET_BINARY_PIXEL_FAST(ptr, sourceX);
                                IMAGE_PUT_BINARY_PIXEL_FAST(row_ptr, x, pixel);
                            }
                        }
                    }
                    break;
                }
                case IMAGE_BPP_GRAYSCALE: {
                    uint8_t *tmp = (uint8_t *) data;

                    for (int y = 0, yy = h; y < yy; y++) {
                        uint8_t *row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(img, y);
                        for (int x = 0, xx = w; x < xx; x++) {
                            int sourceX = fast_roundf(T4_00*x + T4_01*y + T4_02);
                            int sourceY = fast_roundf(T4_10*x + T4_11*y + T4_12);

                            if ((0 <= sourceX) && (sourceX < w) && (0 <= sourceY) && (sourceY < h)) {
                                uint8_t *ptr = tmp + (w * sourceY);
                                int pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(ptr, sourceX);
                                IMAGE_PUT_GRAYSCALE_PIXEL_FAST(row_ptr, x, pixel);
                            }
                        }
                    }
                    break;
                }
                case IMAGE_BPP_RGB565: {
                    uint16_t *tmp = (uint16_t *) data;

                    for (int y = 0, yy = h; y < yy; y++) {
                        uint16_t *row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(img, y);
                        for (int x = 0, xx = w; x < xx; x++) {
                            int sourceX = fast_roundf(T4_00*x + T4_01*y + T4_02);
                            int sourceY = fast_roundf(T4_10*x + T4_11*y + T4_12);

                            if ((0 <= sourceX) && (sourceX < w) && (0 <= sourceY) && (sourceY < h)) {
                                uint16_t *ptr = tmp + (w * sourceY);
                                int pixel = IMAGE_GET_RGB565_PIXEL_FAST(ptr, sourceX);
                                IMAGE_PUT_RGB565_PIXEL_FAST(row_ptr, x, pixel);
                            }
                        }
                    }
                    break;
                }
                default: {
                    break;
                }
            }
        } else { // warp persepective
            switch(img->bpp) {
                case IMAGE_BPP_BINARY: {
                    uint32_t *tmp = (uint32_t *) data;

                    for (int y = 0, yy = h; y < yy; y++) {
                        uint32_t *row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(img, y);
                        for (int x = 0, xx = w; x < xx; x++) {
                            float xxx = T4_00*x + T4_01*y + T4_02;
                            float yyy = T4_10*x + T4_11*y + T4_12;
                            float zzz = T4_20*x + T4_21*y + T4_22;
                            int sourceX = fast_roundf(xxx / zzz);
                            int sourceY = fast_roundf(yyy / zzz);

                            if ((0 <= sourceX) && (sourceX < w) && (0 <= sourceY) && (sourceY < h)) {
                                uint32_t *ptr = tmp + (((w + UINT32_T_MASK) >> UINT32_T_SHIFT) * sourceY);
                                int pixel = IMAGE_GET_BINARY_PIXEL_FAST(ptr, sourceX);
                                IMAGE_PUT_BINARY_PIXEL_FAST(row_ptr, x, pixel);
                            }
                        }
                    }
                    break;
                }
                case IMAGE_BPP_GRAYSCALE: {
                    uint8_t *tmp = (uint8_t *) data;

                    for (int y = 0, yy = h; y < yy; y++) {
                        uint8_t *row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(img, y);
                        for (int x = 0, xx = w; x < xx; x++) {
                            float xxx = T4_00*x + T4_01*y + T4_02;
                            float yyy = T4_10*x + T4_11*y + T4_12;
                            float zzz = T4_20*x + T4_21*y + T4_22;
                            int sourceX = fast_roundf(xxx / zzz);
                            int sourceY = fast_roundf(yyy / zzz);

                            if ((0 <= sourceX) && (sourceX < w) && (0 <= sourceY) && (sourceY < h)) {
                                uint8_t *ptr = tmp + (w * sourceY);
                                int pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(ptr, sourceX);
                                IMAGE_PUT_GRAYSCALE_PIXEL_FAST(row_ptr, x, pixel);
                            }
                        }
                    }
                    break;
                }
                case IMAGE_BPP_RGB565: {
                    uint16_t *tmp = (uint16_t *) data;

                    for (int y = 0, yy = h; y < yy; y++) {
                        uint16_t *row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(img, y);
                        for (int x = 0, xx = w; x < xx; x++) {
                            float xxx = T4_00*x + T4_01*y + T4_02;
                            float yyy = T4_10*x + T4_11*y + T4_12;
                            float zzz = T4_20*x + T4_21*y + T4_22;
                            int sourceX = fast_roundf(xxx / zzz);
                            int sourceY = fast_roundf(yyy / zzz);

                            if ((0 <= sourceX) && (sourceX < w) && (0 <= sourceY) && (sourceY < h)) {
                                uint16_t *ptr = tmp + (w * sourceY);
                                int pixel = IMAGE_GET_RGB565_PIXEL_FAST(ptr, sourceX);
                                IMAGE_PUT_RGB565_PIXEL_FAST(row_ptr, x, pixel);
                            }
                        }
                    }
                    break;
                }
                default: {
                    break;
                }
            }
        }

        matd_destroy(T4);
    }

    matd_destroy(T3);
    matd_destroy(T2);
    matd_destroy(T1);
    matd_destroy(A2);
    matd_destroy(T);
    matd_destroy(R);
    matd_destroy(RZ);
    matd_destroy(RY);
    matd_destroy(RX);
    matd_destroy(A1);

    fb_free(); // umm_init_x();

    fb_free();
}
#endif //IMLIB_ENABLE_ROTATION_CORR
#pragma GCC diagnostic pop
