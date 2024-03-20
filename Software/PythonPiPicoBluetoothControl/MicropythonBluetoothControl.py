from machine import Pin,UART,
import time
led = Pin("LED", Pin.OUT)
ATPin = Pin(2, Pin.OUT)
uart = UART(0,9600)
print("hi")

ATPin.value(1)
led.value(1)

time.sleep(1)

def send_at_command(command, delay=1):
    uart.write(command + '\r\n')
    time.sleep(delay)
    response = uart.read()
    return response.decode('utf-8') if response else ''

def decode_inquiry_response():
    send_at_command('AT+INQ')  # Start inquiry

    # Read bytes from the HC-05 and accumulate them into lines
    end_time = time.time() + 20  # Allow 20 seconds for inquiry
    line = ''
    while time.time() < end_time:
        if uart.any():
            byte = uart.read(1).decode('utf-8')
            if byte == '\r':  # Carriage return, ignore
                continue
            if byte == '\n':  # End of line
                if line.startswith('+INQ:'):
                    parts = line.split(',')
                    address = parts[0].split(':')[1].replace(':', '')
                    formatted_address = ':'.join(address[i:i+2] for i in range(0, len(address), 2))
                    device_class = parts[1]
                    rssi = parts[2]
                    device_name = send_at_command('AT+RNAME?' + formatted_address, delay=3).strip()
                    print(f'Address: {formatted_address}, Class: {device_class}, RSSI: {rssi}, Name: {device_name}')
                line = ''  # Reset line
            else:
                line += byte


def find_device_by_name(target_name):
    send_at_command('AT+INQ')  # Start inquiry

    # Read bytes from the HC-05 and accumulate them into lines
    end_time = time.time() + 20  # Allow 20 seconds for inquiry
    line = ''
    while time.time() < end_time:
        if uart.any():
            byte = uart.read(1).decode('utf-8')
            if byte == '\r':  # Carriage return, ignore
                continue
            if byte == '\n':  # End of line
                if line.startswith('+INQ:'):
                    parts = line.split(',')
                    address = parts[0].split(':')[1]
                    device_name = send_at_command('AT+RNAME?' + address, delay=2)
                    if device_name and target_name in device_name:
                        return address, device_name.strip()
                line = ''  # Reset line
            else:
                line += byte

    return None, None  # Device not found



# Change device name
new_name = 'JayConnection'
send_at_command('AT+NAME={}'.format(new_name))
print('Device name changed to:', new_name)

# Change password
new_password = '4321'
send_at_command('AT+PSWD={}'.format(new_password))
print('Password changed to:', new_password)


new_mode = '1'
send_at_command('AT+CMODE={}'.format(new_mode))
print('Mode Changed To:', new_mode)

#commandmode = send_at_command('AT+CMODE?')
#print('Command_Mode:', commandmode)

# Get device name
#name = send_at_command('AT+NAME?')
#print('Device Name:', name)

# Get device password (default is usually 1234 or 0000)
#password = send_at_command('AT+PSWD?')
#print('Password:', password)

# Get MAC address
#mac_address = send_at_command('AT+ADDR?')
#print('MAC Address:', mac_address)



# target_name = 'HC-05'
# address, name = find_device_by_name(target_name)
# if address:
#     print('Found device:', name, 'with address:', address)
# else:
#     print('Device not found')
while True:
    # print('checking BT')
    # Example usage
    decode_inquiry_response()
    #if uart.any():
      #  command = uart.readline()
      #  print(command)