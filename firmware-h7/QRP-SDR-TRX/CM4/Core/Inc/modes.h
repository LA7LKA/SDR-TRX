#ifndef MODES_H
#define MODES_H

/*
 * Mode display constants only - CM4's own copy, not shared with CM7's
 * modes.c/modes.h (which also owns block-size/FreeDV-chain logic CM4 has
 * no reason to know about). Must stay in sync with CM7's MODE_* values by
 * hand, since a CM4 build can't include CM7's header - independent copies
 * are the deliberate choice for this whole port (see firmware-h7/README.md),
 * and this is about as small and low-churn as a shared enum gets.
 */
#define MODE_NBFM 0
#define MODE_USB  1
#define MODE_LSB  2
#define MODE_FREEDV 3
#define MODE_FREEDV_2400B 4
#define MODE_FREEDV_700D  5
#define MODE_FREEDV_700E  6
#define MODE_AM           7
#define MODE_CW            8

const char *mode_name(int mode);
int         mode_count(void);

#endif
