#ifndef ORCA_FUNCTION_CODES_H_
#define ORCA_FUNCTION_CODES_H_

#include <stdint.h>
#include "message_priority.h"
#include "transaction.h"

namespace orcaSDK {

namespace OrcaFunctionCodes
{
	/**
	 * @brief Enum of all supported function codes.
	 */
	enum function_codes_e {
		ext_motor_command = 0x66
	};
}

namespace OrcaModbusFunctions {

	/**
	 * @brief Format a ext_motor_command request, function code 102, and add the request to the buffer queue
	 * @param device_address	The address of the server device that will accept the transaction, 0 if broadcast
     * @param mode 				The mode command
     * @param command			The 4-byte command field which is interpreted based on mode
     * @param read_reg			The address to start reading the 4 consecutive registers from
	 * @return 					An integer - 1 if the transaction is formatted and added to the buffer queue successfully, 0 if an exception occurs
	 */
	void ext_motor_command_fn(Transaction* t, uint8_t device_address, uint8_t mode, uint32_t command, uint16_t read_reg, MessagePriority priority);
	static constexpr int kExtMotorCmdNumRegRead = 2; // How many registers are read per request of this type. This should be even.
	static_assert(kExtMotorCmdNumRegRead % 2 == 0, "Must read even number of registers in ext_motor_command.");

	// Enum for the mode argument of the ext_motor_command_fn
	enum ExtMotorCommandMode {
		SleepMode 	= 1,
		ForceMode 	= 2,
		PosMode 	= 3,
		KinMode 	= 5
	};
}}

#endif /* ORCA_FUNCTION_CODES_H_ */
