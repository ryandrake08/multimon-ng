/*
 *      gen_ltr.c -- generate LTR-Standard repeater signalling
 *
 *      Copyright (C) 2026 Ryan Drake
 *
 *      Generates the 300 bit/s sub-audible data an LTR-Standard repeater sends
 *      under its FM voice: the given repeater output words back to back, as
 *      the FM-demodulated audio a receiver hears (higher frequency = 1).
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

#include "gen.h"
#include <string.h>

#define LTR_BAUD 300

/* bit times at the middle level before and after the words */
#define LTR_LEAD 8

/* repeater output word synchronisation sequence */
#define LTR_SYNC 0x158u

/* checksum contribution of each data bit, from the area bit to the last free repeater bit */
static const unsigned char ltr_columns[24] = {
	0x38, 0x1C, 0x0E, 0x46, 0x23, 0x51, 0x68, 0x75, 0x7A, 0x3D, 0x1F, 0x4F,
	0x26, 0x52, 0x29, 0x15, 0x0B, 0x45, 0x62, 0x31, 0x19, 0x0D, 0x07, 0x43
};

/* ---------------------------------------------------------------------- */

/* 40-bit word: sync, the 24 data bits (area, channel, home, group, free) and the checksum */
static uint64_t ltr_word(uint32_t data)
{
	unsigned int checksum = 0;
	int i;

	for (i = 0; i < 24; i++)
		if ((data >> (23 - i)) & 1)
			checksum ^= ltr_columns[i];
	return ((uint64_t)LTR_SYNC << 31) | ((uint64_t)data << 7) | checksum;
}

/* Appends n bits of value, most significant first, as levels of +1 and -1 */
static void put_bits(struct gen_state *s, uint64_t value, int n)
{
	while (n-- > 0 && s->s.ltr.nbits < sizeof(s->s.ltr.level))
		s->s.ltr.level[s->s.ltr.nbits++] = ((value >> n) & 1) ? 1 : -1;
}

/* Appends n bit times at the middle level, as the FM carrier without data */
static void put_idle(struct gen_state *s, int n)
{
	while (n-- > 0 && s->s.ltr.nbits < sizeof(s->s.ltr.level))
		s->s.ltr.level[s->s.ltr.nbits++] = 0;
}

void gen_init_ltr(struct gen_params *p, struct gen_state *s)
{
	int i, j;

	memset(s, 0, sizeof(struct gen_state));
	put_idle(s, LTR_LEAD);
	for (i = 0; i < p->p.ltr.count; i++) {
		uint64_t word = ltr_word(p->p.ltr.data[i]);

		/* bit errors in the data and checksum (the last 31 bits), at positions
		 * that vary from word to word */
		for (j = 0; j < p->p.ltr.errors; j++)
			word ^= 1ULL << ((i * 7 + j * 13 + 3) % 31);
		put_bits(s, word, 40);
	}
	put_idle(s, LTR_LEAD);
}

/* The level moves between bit centres along a raised cosine, so each transition
 * takes one bit time and every bit reaches its full level at its centre */
int gen_ltr(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s)
{
	int num = 0;

	for (; buflen > 0; buflen--, buf++, num++) {
		/* time since the centre of bit 0, in units of 1 / (2 * SAMPLE_RATE) bit */
		int64_t t = (int64_t)s->s.ltr.sample * LTR_BAUD * 2 - SAMPLE_RATE;
		unsigned int k;
		int from, to, weight;

		if ((uint64_t)s->s.ltr.sample * LTR_BAUD >= (uint64_t)s->s.ltr.nbits * SAMPLE_RATE)
			return num;
		if (t < 0)
			t = 0;
		k = (unsigned int)(t / (2 * SAMPLE_RATE));
		from = s->s.ltr.level[k];
		to = k + 1 < s->s.ltr.nbits ? s->s.ltr.level[k + 1] : 0;
		/* (1 - cos(pi * fraction)) / 2, scaled to 0..32767 */
		weight = (32767 - COS((unsigned int)(t % (2 * SAMPLE_RATE) * 0x8000 / (2 * SAMPLE_RATE)))) / 2;
		*buf += (p->ampl * (from * 32767 + (to - from) * weight)) >> 15;
		s->s.ltr.sample++;
	}
	return num;
}
