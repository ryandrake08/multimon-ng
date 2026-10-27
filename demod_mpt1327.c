/*
 *      demod_mpt1327.c -- MPT 1327 trunked radio signalling decoder
 *
 *      Copyright (C) 2026 Ryan Drake
 *
 *      Decodes the 1200 bit/s FFSK signalling that MPT 1327 ("A Signalling
 *      Standard for Trunked Private Land Mobile Radio Systems", June 1997
 *      issue) trunking system controllers transmit, and prints one line per
 *      codeword. Section numbers below refer to that standard.
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* ---------------------------------------------------------------------- */

#include "multimon.h"
#include "filter.h"
#include <inttypes.h>
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------- */

/*
 * FFSK, 1200 bit/s (3.1)
 * "1" frequency: 1200 Hz
 * "0" frequency: 1800 Hz
 */

#define FREQ_1 1200
#define FREQ_0 1800
#define FREQ_SAMP 22050
#define BAUD 1200
#define SUBSAMP 2

#define CORRLEN ((int)(FREQ_SAMP/BAUD))
#define SPHASEINC (0x10000u*BAUD*SUBSAMP/FREQ_SAMP)

/* codeword synchronisation sequences (3.2.1) */
#define SYNC 0xC4D7u
#define SYNT 0x3B28u

/* x^15 + x^14 + x^13 + x^11 + x^4 + x^2 + 1 without the x^15 term (3.2.3) */
#define GENERATOR 0x6815u

static float corr_1_i[CORRLEN];
static float corr_1_q[CORRLEN];
static float corr_0_i[CORRLEN];
static float corr_0_q[CORRLEN];

/* ---------------------------------------------------------------------- */

/* Field of a codeword at bits first..last, numbered as in the standard (bit 1 is sent first) */
static unsigned int mpt1327_field(uint64_t cw, int first, int last)
{
    return (unsigned int)((cw >> (64 - last)) & ((1ULL << (last - first + 1)) - 1));
}

/* Remainder of info(x) * x^15 divided by the generator, for the 48 information bits */
static unsigned int mpt1327_remainder(uint64_t info)
{
    unsigned int r = 0;
    int i;

    for (i = 47; i >= 0; i--) {
        unsigned int feedback = ((unsigned int)(info >> i) ^ (r >> 14)) & 1;
        r = (r << 1) & 0x7FFF;
        if (feedback)
            r ^= GENERATOR;
    }
    return r;
}

/* Check bits with the last one inverted, and even parity over all 64 bits (3.2.3) */
static int mpt1327_valid(uint64_t cw)
{
    uint64_t v = cw;
    int parity = 0;

    if (((cw >> 1) & 0x7FFF) != (mpt1327_remainder(cw >> 16) ^ 1))
        return 0;
    while (v) {
        parity ^= (int)(v & 1);
        v >>= 1;
    }
    return !parity;
}

/* Corrects up to one bit error; returns the number of bits corrected, or -1 */
static int mpt1327_correct(uint64_t *cw)
{
    int i;

    if (mpt1327_valid(*cw))
        return 0;
    for (i = 0; i < 64; i++) {
        if (mpt1327_valid(*cw ^ (1ULL << i))) {
            *cw ^= 1ULL << i;
            return 1;
        }
    }
    return -1;
}

/* Control channel system codeword (5.1.1) */
static int mpt1327_is_ccsc(uint64_t cw)
{
    return mpt1327_field(cw, 1, 1) == 0 &&
           mpt1327_field(cw, 33, 48) == 0xAAAA &&
           mpt1327_field(cw, 49, 64) == SYNC;
}

/* ---------------------------------------------------------------------- */

/* Prints an ident, by name for the special idents (table 4-1) */
static void mpt1327_print_ident(const char *label, unsigned int ident)
{
    static const char *const special[] = { "PSTNGI", "PABXI", "DNI" };
    static const char *const special2[] = {
        "REGI", "INCI", "DIVERTI", "SDMI", "IPFIXI", "TSCI", "ALLI"
    };

    if (ident == 0)
        verbprintf(0, " %s=DUMMYI", label);
    else if (ident >= 8101 && ident <= 8103)
        verbprintf(0, " %s=%s", label, special[ident - 8101]);
    else if (ident >= 8185)
        verbprintf(0, " %s=%s", label, special2[ident - 8185]);
    else
        verbprintf(0, " %s=%u", label, ident);
}

static void mpt1327_print_address(uint64_t cw)
{
    verbprintf(0, " PFIX=%u", mpt1327_field(cw, 2, 8));
    mpt1327_print_ident("IDENT1", mpt1327_field(cw, 9, 21));
}

/* Prints one codeword: names and layouts from section 5 */
static void mpt1327_print(uint64_t cw)
{
    /* CAT '000' function names by TYPE (5.3) for messages from the trunking system
     * controller; NULL for spare and reserved functions */
    static const char *const funcs[4][8] = {
        { "ALH", "ALHS", "ALHD", "ALHE", "ALHR", "ALHX", "ALHF", NULL },
        { "ACK", "ACKI", "ACKQ", "ACKX", "ACKV", "ACKE", "ACKT", "ACKB" },
        { "AHY", NULL, "AHYX", NULL, NULL, "AHYP", "AHYQ", "AHYC" },
        { "MARK", "MAINT", "CLEAR", "MOVE", "BCAST", NULL, NULL, NULL },
    };
    unsigned int cat, type, func, sysdef;
    const char *name;

    verbprintf(0, "MPT1327:");
    if (mpt1327_field(cw, 1, 1) == 0) {
        if (mpt1327_is_ccsc(cw))
            verbprintf(0, " CCSC SYS=%04X\n", mpt1327_field(cw, 2, 16));
        else
            verbprintf(0, " DATA 0x%012" PRIx64 "\n", (uint64_t)((cw >> 16) & 0x7FFFFFFFFFFFULL));
        return;
    }
    if (mpt1327_field(cw, 22, 22) == 0) {
        verbprintf(0, " GTC");
        mpt1327_print_address(cw);
        verbprintf(0, " D=%u CHAN=%u", mpt1327_field(cw, 23, 23), mpt1327_field(cw, 24, 33));
        mpt1327_print_ident("IDENT2", mpt1327_field(cw, 34, 46));
        verbprintf(0, " N=%u\n", mpt1327_field(cw, 47, 48));
        return;
    }
    cat = mpt1327_field(cw, 23, 25);
    type = mpt1327_field(cw, 26, 27);
    func = mpt1327_field(cw, 28, 30);
    if (cat != 0) {
        verbprintf(0, " CAT=%u%u%u", cat >> 2, (cat >> 1) & 1, cat & 1);
        mpt1327_print_address(cw);
        verbprintf(0, " PARAMS=0x%06x\n", mpt1327_field(cw, 26, 48));
        return;
    }
    name = funcs[type][func];
    if (name == NULL) {
        verbprintf(0, " CAT=000 TYPE=%u%u FUNC=%u%u%u",
                   type >> 1, type & 1, func >> 2, (func >> 1) & 1, func & 1);
        mpt1327_print_address(cw);
        verbprintf(0, " PARAMS=0x%05x\n", mpt1327_field(cw, 31, 48));
        return;
    }
    verbprintf(0, " %s", name);
    switch (type) {
    case 0: /* Aloha (5.5.1) */
        mpt1327_print_address(cw);
        verbprintf(0, " CHAN4=%u WT=%u M=%u N=%u\n",
                   mpt1327_field(cw, 31, 34), mpt1327_field(cw, 35, 37),
                   mpt1327_field(cw, 40, 44), mpt1327_field(cw, 45, 48));
        return;
    case 1: /* acknowledgement (5.5.2) */
        mpt1327_print_address(cw);
        mpt1327_print_ident("IDENT2", mpt1327_field(cw, 31, 43));
        verbprintf(0, " QUAL=%u N=%u\n", mpt1327_field(cw, 44, 44), mpt1327_field(cw, 45, 48));
        return;
    case 2: /* Ahoy (5.5.3.2) */
        mpt1327_print_address(cw);
        mpt1327_print_ident("IDENT2", mpt1327_field(cw, 31, 43));
        if (func == 0)
            verbprintf(0, " D=%u POINT=%u CHECK=%u E=%u AD=%u",
                       mpt1327_field(cw, 44, 44), mpt1327_field(cw, 45, 45),
                       mpt1327_field(cw, 46, 46), mpt1327_field(cw, 47, 47),
                       mpt1327_field(cw, 48, 48));
        else if (func == 6)
            verbprintf(0, " STATUS=%u", mpt1327_field(cw, 44, 48));
        else if (func == 7)
            verbprintf(0, " SLOTS=%u DESC=%u",
                       mpt1327_field(cw, 44, 45), mpt1327_field(cw, 46, 48));
        verbprintf(0, "\n");
        return;
    }
    /* miscellaneous (5.5.4) */
    switch (func) {
    case 0: /* MARK */
        verbprintf(0, " CHAN4=%u SYS=%04X\n", mpt1327_field(cw, 2, 5), mpt1327_field(cw, 7, 21));
        return;
    case 1: /* MAINT */
        mpt1327_print_address(cw);
        verbprintf(0, " CHAN=%u OPER=%u\n", mpt1327_field(cw, 31, 40), mpt1327_field(cw, 41, 43));
        return;
    case 2: /* CLEAR */
        verbprintf(0, " CHAN=%u CONT=%u\n", mpt1327_field(cw, 2, 11), mpt1327_field(cw, 12, 21));
        return;
    case 3: /* MOVE */
        mpt1327_print_address(cw);
        verbprintf(0, " CONT=%u M=%u\n", mpt1327_field(cw, 31, 40), mpt1327_field(cw, 41, 45));
        return;
    }
    /* BCAST */
    sysdef = mpt1327_field(cw, 2, 6);
    verbprintf(0, " SYSDEF=%u SYS=%04X", sysdef, mpt1327_field(cw, 7, 21));
    if (sysdef == 0 || sysdef == 1)
        verbprintf(0, " CHAN=%u\n", mpt1327_field(cw, 31, 40));
    else if (sysdef == 2)
        verbprintf(0, " PER=%u IVAL=%u PON=%u ID=%u\n",
                   mpt1327_field(cw, 31, 31), mpt1327_field(cw, 32, 36),
                   mpt1327_field(cw, 37, 37), mpt1327_field(cw, 38, 38));
    else if (sysdef == 4 || sysdef == 5)
        verbprintf(0, " CHAN=%u ADJSITE=%u\n",
                   mpt1327_field(cw, 31, 40), mpt1327_field(cw, 45, 48));
    else
        verbprintf(0, " PARAMS=0x%05x\n", mpt1327_field(cw, 31, 48));
}

/* ---------------------------------------------------------------------- */

/* Prints a CCSC when its system identity code differs from the last one */
static void mpt1327_system(struct demod_state *s, uint64_t cw)
{
    int sys = (int)mpt1327_field(cw, 2, 16);

    if (sys != s->l1.mpt1327.sys) {
        s->l1.mpt1327.sys = sys;
        mpt1327_print(cw);
    }
}

/*
 * Finds SYNC or SYNT, then checks each following 64-bit codeword. A CCSC
 * (5.1.1) ends with the SYNC of the next slot, so slot framing carries on
 * through it. After a codeword that cannot be corrected, framing resumes
 * after the earliest SYNC or SYNT within it, since each traffic channel
 * message has its own sync (3.3.2).
 */
static void mpt1327_rxbit(struct demod_state *s, int bit)
{
    uint64_t cw;
    unsigned int sync;
    int i;

    s->l1.mpt1327.shreg = (s->l1.mpt1327.shreg << 1) | (bit & 1);
    if (s->l1.mpt1327.count >= 0 && ++s->l1.mpt1327.count == 64) {
        cw = s->l1.mpt1327.shreg;
        if (mpt1327_correct(&cw) < 0) {
            s->l1.mpt1327.count = -1;
            for (i = 48; i > 0; i--) {
                sync = (unsigned int)(s->l1.mpt1327.shreg >> i) & 0xFFFF;
                if (sync == SYNC || sync == SYNT) {
                    s->l1.mpt1327.count = i;
                    return;
                }
            }
        } else {
            s->l1.mpt1327.count = 0;
            if (mpt1327_is_ccsc(cw))
                mpt1327_system(s, cw);
            else
                mpt1327_print(cw);
            return;
        }
    }
    if (s->l1.mpt1327.count < 0) {
        sync = (unsigned int)(s->l1.mpt1327.shreg & 0xFFFF);
        if (sync == SYNC || sync == SYNT) {
            s->l1.mpt1327.count = 0;
            cw = s->l1.mpt1327.shreg;
            if (mpt1327_correct(&cw) >= 0 && mpt1327_is_ccsc(cw))
                mpt1327_system(s, cw);
        }
    }
}

/* ---------------------------------------------------------------------- */

static void mpt1327_init(struct demod_state *s)
{
    float f;
    int i;

    memset(&s->l1.mpt1327, 0, sizeof(s->l1.mpt1327));
    s->l1.mpt1327.count = -1;
    s->l1.mpt1327.sys = -1;
    for (f = 0, i = 0; i < CORRLEN; i++) {
        corr_1_i[i] = cos(f);
        corr_1_q[i] = sin(f);
        f += 2.0*M_PI*FREQ_1/FREQ_SAMP;
    }
    for (f = 0, i = 0; i < CORRLEN; i++) {
        corr_0_i[i] = cos(f);
        corr_0_q[i] = sin(f);
        f += 2.0*M_PI*FREQ_0/FREQ_SAMP;
    }
}

/* ---------------------------------------------------------------------- */

/* FFSK demodulator of demod_fmsfsk.c */
static void mpt1327_demod(struct demod_state *s, buffer_t buffer, int length)
{
    float f;

    if (s->l1.mpt1327.subsamp) {
        if (length <= (int)s->l1.mpt1327.subsamp) {
            s->l1.mpt1327.subsamp -= length;
            return;
        }
        buffer.fbuffer += s->l1.mpt1327.subsamp;
        length -= s->l1.mpt1327.subsamp;
        s->l1.mpt1327.subsamp = 0;
    }
    for (; length > 0; length -= SUBSAMP, buffer.fbuffer += SUBSAMP) {
        f = fsqr(mac(buffer.fbuffer, corr_1_i, CORRLEN)) +
            fsqr(mac(buffer.fbuffer, corr_1_q, CORRLEN)) -
            fsqr(mac(buffer.fbuffer, corr_0_i, CORRLEN)) -
            fsqr(mac(buffer.fbuffer, corr_0_q, CORRLEN));
        s->l1.mpt1327.dcd_shreg <<= 1;
        s->l1.mpt1327.dcd_shreg |= (f > 0);
        /*
         * check if transition
         */
        if ((s->l1.mpt1327.dcd_shreg ^ (s->l1.mpt1327.dcd_shreg >> 1)) & 1) {
            if (s->l1.mpt1327.sphase < (0x8000u-(SPHASEINC/2)))
                s->l1.mpt1327.sphase += SPHASEINC/8;
            else
                s->l1.mpt1327.sphase -= SPHASEINC/8;
        }
        s->l1.mpt1327.sphase += SPHASEINC;
        if (s->l1.mpt1327.sphase >= 0x10000u) {
            s->l1.mpt1327.sphase &= 0xffffu;
            mpt1327_rxbit(s, s->l1.mpt1327.dcd_shreg & 1);
        }
    }
    s->l1.mpt1327.subsamp = -length;
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_mpt1327 = {
    "MPT1327", true, FREQ_SAMP, CORRLEN, mpt1327_init, mpt1327_demod, NULL
};

/* ---------------------------------------------------------------------- */
