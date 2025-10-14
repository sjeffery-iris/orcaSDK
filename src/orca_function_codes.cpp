#include <stdint.h>
#include "orca_function_codes.h"
#include "modbus_client.h"
#include "message_priority.h"
#include "transaction.h"

namespace orcaSDK { namespace OrcaModbusFunctions {

	void ext_motor_command_fn(Transaction* t, uint8_t device_address, uint8_t mode, uint32_t command, uint16_t read_reg, MessagePriority priority) {
		t->reset_transaction();
		constexpr int kResponseSize = 42; // Multiple of 14 -> important for FIFO size on Triton
		constexpr int kSizeOfData = 7;
		uint8_t data_bytes[kSizeOfData] = {
									uint8_t(mode),
									uint8_t(command >> 24),
									uint8_t(command >> 16),
									uint8_t(command >> 8),
									uint8_t(command),
									uint8_t(read_reg >> 8),
									uint8_t(read_reg)
									};
		if (priority == MessagePriority::important) t->mark_important();
		t->load_transmission_data(
			device_address,
			OrcaFunctionCodes::ext_motor_command,
			data_bytes,
			kSizeOfData,
			kResponseSize);
	}
}}
