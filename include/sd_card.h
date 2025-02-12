#ifndef SD_CARD_H
#define SD_CARD_H

#include "esp_err.h"

esp_err_t init_sd_card();
void cleanup_sd_card();

#endif