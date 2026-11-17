#include "dexcap_teleop/dexcap_device.hpp"

#include <cstddef>
#include <memory>
#include <stdexcept>

#include "configuration.h"

namespace dexcap_teleop
{

std::string find_serial_port(const DEXCAP_DEVICE_TYPE device_type)
{
    using DeviceList = std::unique_ptr<
        dexcap_device_serial_port_info,
        decltype(&free_serial_port_device_list)>;
    DeviceList devices(
        alloc_serial_port_device_list(), free_serial_port_device_list);
    if (!devices)
    {
        throw std::runtime_error(
            "Could not allocate the DexCap serial device list");
    }

    std::size_t count = 0;
    enumerate_serial_port_devices(ProductVersion::V4, devices.get(), &count);
    for (std::size_t index = 0; index < count; ++index)
    {
        if (devices.get()[index].device_type ==
            static_cast<uint8_t>(device_type))
        {
            return devices.get()[index].serial_port_name;
        }
    }
    return {};
}

}  // namespace dexcap_teleop
