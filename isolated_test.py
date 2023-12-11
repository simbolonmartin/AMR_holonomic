class Solution():
    def set_speed(self, id, speed):
        id_byte = self.id_to_byte(id)
        speed_byte = self.speed_to_byte_command(speed)
        message_before_speed = [0x10, 0x00, 0x5A, 0x00, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00]
        message_after_speed = [0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xC4, 0x00, 0x00, 0x03,
                        0xE8, 0x00, 0x00, 0x00, 0x01]
        pre_CRC = id_byte + message_before_speed + speed_byte + message_after_speed
        CRC_two_bytes = self.calculate_modbus_crc(pre_CRC)
        final_message = pre_CRC + list(CRC_two_bytes)
        return final_message

    def calculate_modbus_crc(self, data):
        crc = 0xFFFF  # Initial value for Modbus CRC

        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001  # Polynomial: 0x8005
                else:
                    crc >>= 1

        # Swap bytes to get little-endian result
        crc = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)
        return crc.to_bytes(2, byteorder='big')
    
    def id_to_byte(self, id):
        return list(int.to_bytes(id, 1, "big", signed=True))

    def speed_to_byte_command(self, speed):
        return list(int.to_bytes(speed, 2, 'big', signed=True))

def printHex(send_data):
    print(f"Combined Data (Hex): {', '.join(hex(byte) for byte in send_data)}")

example = Solution()
printHex(example.set_speed(2,-1000))
