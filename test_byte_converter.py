def speed_to_byte_command(speed):
    return int.to_bytes(speed, 1, 'big', signed=True)

print(speed_to_byte_command(1))