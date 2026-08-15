#include "modes.h"
#include <stddef.h>

static const char *names[] = {
    [MODE_NBFM]         = "NBFM",
    [MODE_USB]          = "USB",
    [MODE_LSB]          = "LSB",
    [MODE_FREEDV]       = "FreeDV 1600",
    [MODE_FREEDV_2400B] = "FreeDV 2400B",
    [MODE_FREEDV_700D]  = "FreeDV 700D",
    [MODE_FREEDV_700E]  = "FreeDV 700E",
    [MODE_AM]           = "AM",
    [MODE_CW]           = "CW",
};
#define MODE_COUNT ((int)(sizeof(names) / sizeof(names[0])))

const char *mode_name(int mode)
{
    if (mode < 0 || mode >= MODE_COUNT || names[mode] == NULL)
        return names[MODE_USB];
    return names[mode];
}

int mode_count(void) { return MODE_COUNT; }
