/* Standalone: cc -std=gnu11 -I. test/test_ltr.c -lm -o test_ltr */
#include <stdarg.h>
#include <stdio.h>
#include "multimon.h"
#include "../demod_ltr.c"

static char printed[1024];

void _verbprintf(int level, const char *fmt, ...) {
    va_list args;
    size_t used = strlen(printed);
    (void)level;
    va_start(args, fmt);
    vsnprintf(printed + used, sizeof(printed) - used, fmt, args);
    va_end(args);
}

static int failed;

/* Words from off-air recordings: area, channel, home, group, free */
static const uint64_t idle = 0xAC021FF0A6ULL;    /* 0, 1, 1, 255, 1 */
static const uint64_t call = 0xAC0A570128ULL;    /* 0, 5, 5, 112, 2 */
static const uint64_t turnoff = 0xAC3E5702D0ULL; /* 0, 31, 5, 112, 5 */
static const uint64_t area1 = 0xAC529FF4FCULL;   /* 1, 9, 9, 255, 9 */

static void check_printed(const char *what, const char *expected) {
    if (strcmp(printed, expected) != 0) {
        fprintf(stderr, "%s: printed \"%s\", expected \"%s\"\n", what, printed, expected);
        failed = 1;
    }
}

static void check_line(uint64_t word, const char *expected) {
    printed[0] = '\0';
    ltr_print(word);
    check_printed("ltr_print", expected);
}

/*
 * Feeds filtered samples of NRZ words at 10.5 samples per bit: level +1 for 1
 * and -1 for 0, plus offset, with no data (level 0) before and after
 */
static void feed(const uint64_t *words, int count, float offset, int drop_first_bit, const char *expected) {
    static struct demod_state s;
    int nbits = 40 * count, lead = 20, i;

    ltr_init(&s);
    printed[0] = '\0';
    for (i = 0; i < (nbits + 2 * lead) * SPB_NUM / SPB_DEN; i++) {
        int bit = i * SPB_DEN / SPB_NUM - lead;
        float level = 0;

        if (bit >= 0 && bit < nbits) {
            level = (words[bit / 40] >> (39 - bit % 40)) & 1 ? 1 : -1;
            if (drop_first_bit && bit % 40 == 0)
                level = -1;
        }
        ltr_sample(&s, level + offset);
    }
    check_printed("samples", expected);
}

int main(void) {
    const uint64_t words[] = { call, call, turnoff };
    uint64_t bad;
    int a;

    if (!ltr_valid(idle) || !ltr_valid(call) || !ltr_valid(turnoff) || !ltr_valid(area1)) {
        fprintf(stderr, "recorded word not valid\n");
        failed = 1;
    }
    /* all data bits zero: the checksum is zero too */
    if (!ltr_valid(0xAC00000000ULL)) {
        fprintf(stderr, "all-zero word not valid\n");
        failed = 1;
    }
    for (a = 0; a < 31; a++) {
        bad = idle ^ (1ULL << a);
        if (ltr_valid(bad)) {
            fprintf(stderr, "single-bit error at bit %d not detected\n", a);
            failed = 1;
        }
    }
    if (ltr_valid(idle ^ (1ULL << 35))) {
        fprintf(stderr, "wrong sync accepted\n");
        failed = 1;
    }

    check_line(idle, "LTR: IDLE AREA=0 CHAN=1 HOME=1 GROUP=255 FREE=1\n");
    check_line(call, "LTR: CALL AREA=0 CHAN=5 HOME=5 GROUP=112 FREE=2\n");
    check_line(turnoff, "LTR: TURNOFF AREA=0 CHAN=31 HOME=5 GROUP=112 FREE=5\n");
    check_line(area1, "LTR: IDLE AREA=1 CHAN=9 HOME=9 GROUP=255 FREE=9\n");

    /* one line per word, with back-to-back words and a receiver offset */
    feed(&idle, 1, 0, 0, "LTR: IDLE AREA=0 CHAN=1 HOME=1 GROUP=255 FREE=1\n");
    feed(words, 3, 0.8f,  0,
         "LTR: CALL AREA=0 CHAN=5 HOME=5 GROUP=112 FREE=2\n"
         "LTR: CALL AREA=0 CHAN=5 HOME=5 GROUP=112 FREE=2\n"
         "LTR: TURNOFF AREA=0 CHAN=31 HOME=5 GROUP=112 FREE=5\n");
    feed(&area1, 1, -0.6f, 0, "LTR: IDLE AREA=1 CHAN=9 HOME=9 GROUP=255 FREE=9\n");
    /* first sync bit lost at key-up */
    feed(&idle, 1, 0, 1, "LTR: IDLE AREA=0 CHAN=1 HOME=1 GROUP=255 FREE=1\n");
    /* any valid word is printed, except the all-zero word */
    bad = 0xAC3FFFFFA9ULL;
    feed(&bad, 1, 0, 0, "LTR: TURNOFF AREA=0 CHAN=31 HOME=31 GROUP=255 FREE=31\n");
    bad = 0xAC00000000ULL;
    feed(&bad, 1, 0, 0, "");
    /* a bit error in the checksum, and one in the sync */
    bad = idle ^ 1;
    feed(&bad, 1, 0, 0, "");
    bad = idle ^ (1ULL << 36);
    feed(&bad, 1, 0, 0, "");

    if (!failed)
        printf("ltr: all tests passed\n");
    return failed;
}
