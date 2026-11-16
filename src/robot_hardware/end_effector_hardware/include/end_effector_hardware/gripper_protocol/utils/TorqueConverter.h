//
// Torque Converter - Common utility functions for gripper torque conversion
//
#pragma once

#include <algorithm>
#include <cmath>

namespace gripper_protocol
{
    /**
     * @brief Torque conversion utilities for different gripper types
     * 
     * This utility class provides torque conversion functions for:
     * - Jodell gripper
     * 
     * All conversions use normalized torque (0.0-1.0) where:
     * - 0.0 = no torque
     * - 1.0 = maximum torque
     */
    class TorqueConverter
    {
    public:
        /**
         * @brief Jodell gripper torque conversion
         * 
         * Torque range on wire: Modbus high byte 0-255 (see JodellCommandBuilder).
         * Normalized: 0.0 = no torque, 1.0 = maximum torque
         */
        class Jodell
        {
        public:
            static constexpr int MAX_TORQUE = 255;  // Maximum torque value (typical max is 255, 0xFF)

            /**
             * @brief Convert normalized torque (0.0-1.0) to Jodell torque byte (Modbus high byte)
             * @param normalized Normalized torque (0.0=no torque, 1.0=max torque)
             * @return Torque value 0-255 (written as high byte of the speed/torque register)
             */
            static int normalizedToJodell(double normalized)
            {
                // Limit to valid range
                normalized = std::clamp(normalized, 0.0, 1.0);
                
                // If normalized is exactly 0.0, return 0 (completely off)
                if (normalized == 0.0)
                {
                    return 0;
                }
                
                // Linear mapping: 0.0 -> 0, 1.0 -> 255
                int torque = static_cast<int>(normalized * MAX_TORQUE);
                return std::min(MAX_TORQUE, torque);
            }

            /**
             * @brief Convert Jodell torque byte to normalized torque (0.0-1.0)
             * @param jodell_torque Torque byte 0-255 (same range as Modbus high byte)
             * @return Normalized torque (0.0=no torque, 1.0=max torque)
             */
            static double jodellToNormalized(int jodell_torque)
            {
                // Limit to valid range
                jodell_torque = std::max(0, std::min(MAX_TORQUE, jodell_torque));
                return static_cast<double>(jodell_torque) / MAX_TORQUE;
            }
        };
    };
} // namespace gripper_protocol

