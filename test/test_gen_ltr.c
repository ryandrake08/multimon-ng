/* Standalone: cc -std=gnu11 -I. test/test_gen_ltr.c -lm -o test_gen_ltr */
#include <stdio.h>
#include <string.h>
#include "../gen_ltr.c"
#include "../costabi.c"

static int failed;

/* Number of bits in which the levels from offset differ from the n low bits of value */
static int differences(const struct gen_state *s, int offset, uint64_t value, int n)
{
	int i, d = 0;

	for (i = 0; i < n; i++)
		d += s->s.ltr.level[offset + i] != (((value >> (n - 1 - i)) & 1) ? 1 : -1);
	return d;
}

int main(void)
{
	/* idle on LCN 1, and a turn-off word for group 112 on LCN 5 */
	static const uint64_t words[] = { 0xAC021FF0A6ULL, 0xAC3E5702D0ULL };
	struct gen_params p = {0};
	struct gen_state s;
	signed short buf[256];
	int i, n, total = 0, expected;

	p.ampl = 16384;
	p.p.ltr.count = 2;
	p.p.ltr.data[0] = 0x043FE1;
	p.p.ltr.data[1] = 0x7CAE05;

	gen_init_ltr(&p, &s);
	if (s.s.ltr.nbits != 2 * LTR_LEAD + 80) {
		fprintf(stderr, "length %u bits, expected %d\n", s.s.ltr.nbits, 2 * LTR_LEAD + 80);
		failed = 1;
	}
	for (i = 0; i < LTR_LEAD; i++)
		if (s.s.ltr.level[i] != 0 || s.s.ltr.level[LTR_LEAD + 80 + i] != 0) {
			fprintf(stderr, "lead-in or trailer bit %d is not at the middle level\n", i);
			failed = 1;
		}
	for (i = 0; i < 2; i++)
		if (differences(&s, LTR_LEAD + 40 * i, words[i], 40)) {
			fprintf(stderr, "word %d differs from %010llX\n", i, (unsigned long long)words[i]);
			failed = 1;
		}

	/* the waveform stays within the amplitude and lasts the whole transmission */
	for (;;) {
		memset(buf, 0, sizeof(buf));
		if ((n = gen_ltr(buf, 256, &p, &s)) <= 0)
			break;
		for (i = 0; i < n; i++)
			if (buf[i] > p.ampl || buf[i] < -p.ampl) {
				fprintf(stderr, "sample %d is %d, beyond the amplitude\n", total + i, buf[i]);
				failed = 1;
			}
		total += n;
	}
	expected = (int)((uint64_t)s.s.ltr.nbits * SAMPLE_RATE / LTR_BAUD);
	if (total < expected - 1 || total > expected + 1) {
		fprintf(stderr, "%d samples, expected %d\n", total, expected);
		failed = 1;
	}

	/* -e flips that many bits in the data and checksum of every word, never in the sync */
	for (n = 1; n <= 2; n++) {
		p.p.ltr.errors = n;
		gen_init_ltr(&p, &s);
		for (i = 0; i < 2; i++) {
			int offset = LTR_LEAD + 40 * i;
			if (differences(&s, offset, words[i] >> 31, 9) != 0 ||
			    differences(&s, offset, words[i], 40) != n) {
				fprintf(stderr, "-e %d: word %d has %d bit errors, %d in the sync\n", n, i,
				        differences(&s, offset, words[i], 40), differences(&s, offset, words[i] >> 31, 9));
				failed = 1;
			}
		}
	}

	if (!failed)
		printf("gen_ltr: all tests passed\n");
	return failed;
}
