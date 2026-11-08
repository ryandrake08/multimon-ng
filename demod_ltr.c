/*
 *      demod_ltr.c -- LTR-Standard trunked radio signalling decoder
 *
 *      Copyright (C) 2026 Ryan Drake
 *
 *      Decodes the 300 bit/s sub-audible data that LTR-Standard (Logic
 *      Trunked Radio) repeaters send under their FM voice, and prints one
 *      line per repeater output word with a valid checksum.
 *
 *      Word, in transmission order, each field most significant bit first:
 *      sync 101011000 (9 bits), area (1), channel (5), home repeater (5),
 *      group (8), free repeater (5), checksum (7). A 1 is sent as the higher
 *      FM frequency.
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
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------- */

#define FREQ_SAMP 22050
#define BAUD 300
#define SUBSAMP 7

/* low-pass filter that keeps the data and removes the voice, computed every SUBSAMP samples */
#define FILTLEN 294
#define CUTOFF 200

/* filtered samples per bit, as a fraction: 10.5 */
#define SPB_NUM (FREQ_SAMP/SUBSAMP*2/BAUD)
#define SPB_DEN 2

/* filtered samples in one word, and half a bit */
#define WORDLEN ((40*SPB_NUM + SPB_DEN/2)/SPB_DEN)
#define HALFBIT (SPB_NUM/SPB_DEN/2)

/* length of the history of filtered samples */
#define HISTLEN (sizeof(((struct demod_state *)0)->l1.ltr.hist) / sizeof(float))

/* repeater output word synchronisation sequence */
#define SYNC 0x158u

/*
 * Smallest distance of any bit from the decision threshold, as a fraction of
 * half the distance between the mean 1 and 0 levels of the sync, for a word
 * to be accepted. Lower values let through more words whose level droops in
 * long runs of equal bits, and more false words from noise and voice.
 */
#define MIN_EYE 0.25f

extern int json_mode;

static float filt[FILTLEN];

/* checksum contribution of each data bit, from the area bit to the last free repeater bit */
static const unsigned char ltr_columns[24] = {
    0x38, 0x1C, 0x0E, 0x46, 0x23, 0x51, 0x68, 0x75, 0x7A, 0x3D, 0x1F, 0x4F,
    0x26, 0x52, 0x29, 0x15, 0x0B, 0x45, 0x62, 0x31, 0x19, 0x0D, 0x07, 0x43
};

/* ---------------------------------------------------------------------- */

/* Checksum of the 24 data bits: area, channel, home, group, free */
static unsigned int ltr_checksum(uint32_t data)
{
    unsigned int checksum = 0;
    int i;

    for (i = 0; i < 24; i++)
        if ((data >> (23 - i)) & 1)
            checksum ^= ltr_columns[i];
    return checksum;
}

/* True if the 40-bit word has the sync and a correct checksum */
static int ltr_valid(uint64_t word)
{
    return (word >> 31) == SYNC &&
           ltr_checksum((uint32_t)(word >> 7) & 0xFFFFFF) == (unsigned int)(word & 0x7F);
}

/* Prints one word as a text line, or as a JSON object with --json */
static void ltr_print(uint64_t word)
{
    cJSON *json_output = json_mode ? cJSON_CreateObject() : NULL;
    unsigned int area = (unsigned int)(word >> 30) & 1;
    unsigned int chan = (unsigned int)(word >> 25) & 0x1F;
    unsigned int home = (unsigned int)(word >> 20) & 0x1F;
    unsigned int group = (unsigned int)(word >> 12) & 0xFF;
    unsigned int free_rep = (unsigned int)(word >> 7) & 0x1F;
    const char *name = chan == 31 ? "TURNOFF" : group == 255 ? "IDLE" : "CALL";

    if (!json_mode) {
        verbprintf(0, "LTR: %s AREA=%u CHAN=%u HOME=%u GROUP=%u FREE=%u\n",
                   name, area, chan, home, group, free_rep);
        return;
    }
    cJSON_AddStringToObject(json_output, "demod_name", "LTR");
    cJSON_AddStringToObject(json_output, "message", name);
    cJSON_AddNumberToObject(json_output, "area", area);
    cJSON_AddNumberToObject(json_output, "chan", chan);
    cJSON_AddNumberToObject(json_output, "home", home);
    cJSON_AddNumberToObject(json_output, "group", group);
    cJSON_AddNumberToObject(json_output, "free", free_rep);
    addJsonTimestamp(json_output);
    printJson(json_output);
    cJSON_Delete(json_output);
}

/* ---------------------------------------------------------------------- */

/* Filtered sample at the centre of bit k (0-39) of a word whose last bit centre is the newest sample */
static float ltr_bit_sample(struct demod_state *s, int k)
{
    unsigned int back = (unsigned int)(((39 - k) * SPB_NUM + SPB_DEN/2) / SPB_DEN);

    return s->l1.ltr.hist[(s->l1.ltr.n - 1 - back) % HISTLEN];
}

/*
 * Treats the newest filtered sample as the centre of the last bit of a word.
 * The first sync bit is not checked: some repeaters lose it as they key up
 * for an idle burst. The threshold is the midpoint of the mean 1 and 0 levels
 * of the other sync bits, so a constant offset (receiver mistuning) does not
 * matter. Returns the smallest normalised distance of any checked bit from the
 * threshold, or -1 if the window does not hold a valid word.
 */
static float ltr_try_word(struct demod_state *s, uint64_t *word)
{
    float x[40], one = 0, zero = 0, threshold, half, eye = 1e9f;
    uint64_t w = 0;
    int k;

    for (k = 1; k < 40; k++)
        x[k] = ltr_bit_sample(s, k);
    for (k = 1; k < 9; k++) {
        if ((SYNC >> (8 - k)) & 1)
            one += x[k] / 3;
        else
            zero += x[k] / 5;
    }
    if (one <= zero)
        return -1;
    threshold = (one + zero) / 2;
    half = (one - zero) / 2;
    for (k = 1; k < 40; k++) {
        float d = fabsf(x[k] - threshold) / half;

        w = (w << 1) | (x[k] > threshold);
        if (d < eye)
            eye = d;
    }
    w |= 1ULL << 39;
    /* the all-zero data word (checksum 0) follows sync-like transients when a
     * transmitter keys up or down; channel and home 0 are not repeaters */
    if (eye < MIN_EYE || !ltr_valid(w) || ((w >> 7) & 0xFFFFFF) == 0)
        return -1;
    *word = w;
    return eye;
}

/*
 * Adds one filtered sample. A word is valid at several neighbouring sampling
 * phases; the one with the widest eye within half a bit is printed, once.
 * Words that overlap the last printed word are ignored.
 */
static void ltr_sample(struct demod_state *s, float value)
{
    uint64_t word;
    float eye;

    s->l1.ltr.hist[s->l1.ltr.n % HISTLEN] = value;
    s->l1.ltr.n++;
    if (s->l1.ltr.cand && s->l1.ltr.n - s->l1.ltr.cand_n > HALFBIT) {
        ltr_print(s->l1.ltr.cand_word);
        s->l1.ltr.last_n = s->l1.ltr.cand_n;
        s->l1.ltr.printed = 1;
        s->l1.ltr.cand = 0;
    }
    if (s->l1.ltr.n <= WORDLEN)
        return;
    if (s->l1.ltr.printed && s->l1.ltr.n - s->l1.ltr.last_n < WORDLEN - HALFBIT)
        return;
    if ((eye = ltr_try_word(s, &word)) < 0)
        return;
    if (!s->l1.ltr.cand || eye > s->l1.ltr.cand_eye) {
        s->l1.ltr.cand = 1;
        s->l1.ltr.cand_word = word;
        s->l1.ltr.cand_eye = eye;
        s->l1.ltr.cand_n = s->l1.ltr.n;
    }
}

/* ---------------------------------------------------------------------- */

static void ltr_init(struct demod_state *s)
{
    float sum = 0;
    int i;

    memset(&s->l1.ltr, 0, sizeof(s->l1.ltr));
    /* windowed sinc (Hamming) with unity gain at 0 Hz */
    for (i = 0; i < FILTLEN; i++) {
        float t = i - (FILTLEN - 1) / 2.0f;
        float sinc = t == 0 ? 1 : sinf(2 * M_PI * CUTOFF / FREQ_SAMP * t) / (2 * M_PI * CUTOFF / FREQ_SAMP * t);

        filt[i] = sinc * (0.54f - 0.46f * cosf(2 * M_PI * i / (FILTLEN - 1)));
        sum += filt[i];
    }
    for (i = 0; i < FILTLEN; i++)
        filt[i] /= sum;
}

/* ---------------------------------------------------------------------- */

static void ltr_demod(struct demod_state *s, buffer_t buffer, int length)
{
    if (s->l1.ltr.subsamp) {
        if (length <= (int)s->l1.ltr.subsamp) {
            s->l1.ltr.subsamp -= length;
            return;
        }
        buffer.fbuffer += s->l1.ltr.subsamp;
        length -= s->l1.ltr.subsamp;
        s->l1.ltr.subsamp = 0;
    }
    for (; length > 0; length -= SUBSAMP, buffer.fbuffer += SUBSAMP)
        ltr_sample(s, mac(buffer.fbuffer, filt, FILTLEN));
    s->l1.ltr.subsamp = -length;
}

/* ---------------------------------------------------------------------- */

const struct demod_param demod_ltr = {
    "LTR", true, FREQ_SAMP, FILTLEN, ltr_init, ltr_demod, NULL
};

/* ---------------------------------------------------------------------- */
