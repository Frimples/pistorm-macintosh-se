// SPDX-License-Identifier: MIT

#include <stdlib.h>
#include <string.h>

#include "platforms/platforms.h"

static int setup_platform_macse(struct emulator_config *cfg) {
    (void)cfg;
    return 0;
}

void create_platform_macse(struct platform_config *cfg, char *subsys) {
    cfg->id = PLATFORM_MACSE;
    cfg->custom_read = NULL;
    cfg->custom_write = NULL;
    cfg->register_read = NULL;
    cfg->register_write = NULL;
    cfg->platform_initial_setup = setup_platform_macse;

    if (subsys) {
        cfg->subsys = malloc(strlen(subsys) + 1);
        strcpy(cfg->subsys, subsys);
    }
}
