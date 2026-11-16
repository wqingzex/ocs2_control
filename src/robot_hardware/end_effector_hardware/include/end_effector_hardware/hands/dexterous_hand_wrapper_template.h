#pragma once

#include "end_effector_hardware/hands/dexterous_hand_base.h"
#include "robot_comm/modbus.h"
#include "end_effector_hardware/gripper_protocol/utils/ModbusConfig.h"
#include <unordered_map>
#include <memory>
#include <array>
#include <string>

namespace end_effector_hardware
{
    // Scoped to this namespace so that the directive does not leak into every file that includes this header.
    using namespace gripper_protocol;
    namespace DexterousHand = ModbusConfig::DexterousHand;

    /**
     * @brief Product type enum for different dexterous hand models
     */
    enum class DexterousHandProduct
    {
        L6,  // LinkerHand L6 (6-DOF), the only LinkerHand model in the robot descriptions
    };

    /**
     * @brief Traits class for dexterous hand products
     * 
     * Specialize this for different hand products to define:
     * - Joint count (DOF)
     * - Register mapping
     * - Product name for logging
     */
    template<DexterousHandProduct Product>
    struct DexterousHandProductTraits;

    /**
     * @brief L6 hand traits (6-DOF)
     * 
     * Modbus: FC04 reads input registers, FC16 writes holding registers; position 0-5, torque 6-11, speed 12-17.
     */
    template<>
    struct DexterousHandProductTraits<DexterousHandProduct::L6>
    {
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr uint16_t JOINT_POSITION_REG_START = 0;
        static constexpr uint16_t TORQUE_REG_START = 6;
        static constexpr uint16_t SPEED_REG_START = 12;
        static constexpr const char* PRODUCT_NAME = "L6";
        
        /**
         * @brief Map joint name to Modbus register index for L6 hand
         * 
         * Register mapping:
         * 0: Thumb_Pitch
         * 1: Thumb_Yaw
         * 2: Index_Pitch
         * 3: Middle_Pitch
         * 4: Ring_Pitch
         * 5: Little_Pitch
         */
        static int getModbusRegisterIndex(const std::string& joint_name);
    };

    /**
     * @brief Template-based dexterous hand wrapper
     * 
     * Implementation for the LinkerHand L6.
     * Configuration is specified as a template parameter from DexterousHandProduct.
     * 
     * @tparam Product Product type (L6)
     */
    template<DexterousHandProduct Product>
    class DexterousHandWrapperTemplate : public DexterousHandBase
    {
    public:
        using Traits = DexterousHandProductTraits<Product>;
        static constexpr size_t JOINT_COUNT = Traits::JOINT_COUNT;
        static constexpr const char* PRODUCT_NAME = Traits::PRODUCT_NAME;

        /**
         * @brief Constructor
         */
        DexterousHandWrapperTemplate(
            rclcpp::Logger logger,
            rclcpp::Clock::SharedPtr clock,
            const std::vector<std::string>& joint_names
        );

        /**
         * @brief Destructor
         */
        ~DexterousHandWrapperTemplate() override;

        /**
         * @brief Initialize the dexterous hand
         */
        bool initialize(
            robot_comm::ModbusMaster* communicator,
            const std::unordered_map<std::string, std::string>& params
        ) override;

        /**
         * @brief Read hand status
         */
        bool readStatus() override;

        /**
         * @brief Write hand command
         */
        bool writeCommand() override;

        void setToolRatios(double torque_ratio, double velocity_ratio) override;

        /**
         * @brief Shutdown hand connection
         */
        void shutdown() override;

    private:
        // Modbus communicator
        robot_comm::ModbusMaster* communicator_;
        
        // Hand configuration
        uint8_t modbus_slave_id_;

        // Joint limits (radians)
        std::array<double, JOINT_COUNT> joint_lower_limits_ = {};
        std::array<double, JOINT_COUNT> joint_upper_limits_ = {};
        
        std::atomic<double> torque_ratio_{1.0};
        std::atomic<double> velocity_ratio_{1.0};
        std::atomic<bool> dynamics_write_pending_{true};

        // Position conversion: ROS2 Control radians <-> Modbus raw value (0x00-0xFF)
        uint8_t radiansToRaw(size_t joint_index, double radians) const;
        double rawToRadians(size_t joint_index, uint8_t raw) const;

        // Logging helpers
        void logInitialized();
        void logStatus(const uint8_t* raw_values);
        void logCommand(const uint16_t* raw_positions);
    };

    // Type aliases for convenience
    using L6DexterousHandWrapper = DexterousHandWrapperTemplate<DexterousHandProduct::L6>;      // L6 hand (6-DOF)
} // namespace end_effector_hardware

// Include template implementation
#include "end_effector_hardware/hands/linkerhand/linkerhand_rs485_wrapper_impl.h"
