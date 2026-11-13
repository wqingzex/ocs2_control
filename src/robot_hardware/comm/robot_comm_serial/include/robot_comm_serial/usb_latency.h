#pragma once

#include <string>
#include <vector>

#include "robot_comm/serial.h"

namespace robot_comm
{
    /**
     * USB serial adapters (FTDI) hold bytes back for up to 16 ms by default (the latency timer), which ruins the
     * response time of a request / answer protocol. Set it to 1 ms and check it. Resolves device symlinks.
     * Unsupported adapters are skipped; a permission or verification failure only produces a warning notice, the
     * port stays usable.
     */
    std::vector<Notice> configureUsbLatency(const std::string& device);
} // namespace robot_comm
