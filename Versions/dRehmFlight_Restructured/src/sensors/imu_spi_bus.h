#pragma once

#include "../../config.h"
#include <Arduino.h>

// Boards with several IMUs on one SPI bus must park every chip select high
// before the first transfer, or an unconfigured chip can answer on MISO.
inline void imu_spi_deselect_all() {
#if defined(BOARD_FMU_RT1062)
  for (int cs : pins::IMU_SPI_CS_ALL) {
    pinMode(cs, OUTPUT);
    digitalWriteFast(cs, HIGH);
  }
#endif
}
