#include "modes.h"
#include "freedv_chain.h"
#include <stddef.h>

int MODE = MODE_USB;  // default mode

typedef struct {
    const char *name;
    uint32_t    block_size;
    uint8_t     buffered;
    int8_t      chain_mode;   /* FREEDV_CHAIN_MODE_*, or -1 if not FreeDV */
} mode_cfg_t;

static const mode_cfg_t mode_cfg[] = {
    [MODE_NBFM]         = { "NBFM",  BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_USB]          = { "USB",   BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_LSB]          = { "LSB",   BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_FREEDV]       = { "FreeDV 1600",  BLOCK_SIZE_FREEDV, 1, FREEDV_CHAIN_MODE_1600  },
    [MODE_FREEDV_2400B] = { "FreeDV 2400B", BLOCK_SIZE_2400B,  1, FREEDV_CHAIN_MODE_2400B },
    [MODE_FREEDV_700D]  = { "FreeDV 700D",  BLOCK_SIZE_700D,   1, FREEDV_CHAIN_MODE_700D  },
    [MODE_FREEDV_700E]  = { "FreeDV 700E",  BLOCK_SIZE_700E,   1, FREEDV_CHAIN_MODE_700E  },
    [MODE_AM]           = { "AM",    BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_CW]           = { "CW",    BLOCK_SIZE_ANALOG, 0, -1 },
};

#define MODE_COUNT ((int)(sizeof(mode_cfg) / sizeof(mode_cfg[0])))

static const mode_cfg_t *cfg_for(int mode)
{
    if (mode < 0 || mode >= MODE_COUNT || mode_cfg[mode].name == NULL)
        return &mode_cfg[MODE_USB];
    return &mode_cfg[mode];
}

uint32_t    block_size_for(int mode) { return cfg_for(mode)->block_size; }
int         mode_is_freedv(int mode) { return cfg_for(mode)->chain_mode >= 0; }
int         chain_mode_for(int mode) { return cfg_for(mode)->chain_mode; }
int         mode_is_buffered(int mode) { return cfg_for(mode)->buffered; }
const char *mode_name(int mode)   { return cfg_for(mode)->name; }
int         mode_count(void)      { return MODE_COUNT; }
