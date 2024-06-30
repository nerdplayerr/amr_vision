import time

# Function to calculate checksum
def checksum_pc_generator(data):
    checksum = 0
    for byte in data:
        checksum += byte
    return checksum & 0xFF

# Function to parse BNO08X data packet
def parse_BNO08X_packet(packet):
    if len(packet) != 16:
        print('Not long enough')
        return None  # Packet length is not correct
    if packet[0] != 0xA5 or packet[1] != 0x5A:
        print('incorrect header')
        return None  # Header bytes are not correct
    if packet[15] != checksum_pc_generator(packet[:15]):
        print('checksum wrong')
        return None  # Checksum doesn't match
    
    BNO08x = {
        'yaw': ((packet[3] << 8) | packet[4]) - 65536 if packet[3] & 0x80 else (packet[3] << 8) | packet[4],
        'pitch': ((packet[5] << 8) | packet[6]) - 65536 if packet[5] & 0x80 else (packet[5] << 8) | packet[6],
        'roll': ((packet[7] << 8) | packet[8]) - 65536 if packet[7] & 0x80 else (packet[7] << 8) | packet[8],
        'x_acceleration': ((packet[9] << 8) | packet[10]) - 65536 if packet[9] & 0x80 else (packet[9] << 8) | packet[10],
        'y_acceleration': ((packet[11] << 8) | packet[12]) - 65536 if packet[11] & 0x80 else (packet[11] << 8) | packet[12],
        'z_acceleration': ((packet[13] << 8) | packet[14]) - 65536 if packet[13] & 0x80 else (packet[13] << 8) | packet[14]
    }

    print(BNO08x)
    
    return BNO08x

# Function to parse Sensor data packet
def parse_Sensor_packet(packet):
    if len(packet) != 16:
        print('Not long enough')
        return None  # Packet length is not correct
    if packet[0] != 0xA5 or packet[1] != 0x5A:
        print('incorrect header')
        return None  # Header bytes are not correct
    if packet[15] != checksum_pc_generator(packet[:15]):
        print('checksum wrong')
        return None  # Checksum doesn't match
    
    Sensor = {
        'temperature': (packet[3] << 8) | packet[4],
        'humidity': (packet[5] << 8) | packet[6],
        'current': (packet[7] << 8) | packet[8],
        'voltage': (packet[9] << 8) | packet[10],
        'loadcell': (packet[11] << 8) | packet[12]
    }

    print(Sensor)
    
    return Sensor

# Function to parse Ping
def parse_pc_ping_response_packet(packet):
    if len(packet) != 16:
        print('Not long enough')
        return None  # Packet length is not correct
    if packet[0] != 0xA5 or packet[1] != 0x5A:
        print('incorrect header')
        return None  # Header bytes are not correct
    if packet[15] != checksum_pc_generator(packet[:15]):
        print('checksum wrong')
        return None  # Checksum doesn't match
    
    print("Ping")

    return True if packet[15] == 0 else False

# Function to parse Encoder
def parse_Encoder_Package_packet(packet):
    if len(packet) != 16:
        print('Not long enough')
        return None  # Packet length is not correct
    if packet[0] != 0xA5 or packet[1] != 0x5A:
        print('incorrect header')
        return None  # Header bytes are not correct
    if packet[15] != checksum_pc_generator(packet[:15]):
        print('checksum wrong')
        return None  # Checksum doesn't match
    
    Encoder_Package = {
        'vertical_distance': (packet[3] << 8) | packet[4],
        'horizontal_distance': (packet[5] << 8) | packet[6],
        'vertical_speed': (packet[7] << 8) | packet[8],
        'horizontal_speed': (packet[9] << 8) | packet[10]
    }

    print(Encoder_Package)
    
    return Encoder_Package

# Function to parse Kinematic
def parse_Kinematic_packet(packet):
    if len(packet) != 16:
        print('Not long enough')
        return None  # Packet length is not correct
    if packet[0] != 0xA5 or packet[1] != 0x5A:
        print('Incorrect header')
        return None  # Header bytes are not correct
    if packet[15] != checksum_pc_generator(packet[:15]):
        print('Checksum wrong')
        return None  # Checksum doesn't match
    
    Sx = (packet[3] << 8) | packet[4]
    Sy = (packet[5] << 8) | packet[6]
    St = (packet[7] << 8) | packet[8]
    T = (packet[9] << 8) | packet[10]
    
    Kinematic_data = {
        'Sx': Sx,
        'Sy': Sy,
        'St': St,
        'T': T
    }

    print(Kinematic_data)
    
    return Kinematic_data

# Function to parse DWM
def parse_DWM_packet(packet):
    if len(packet) != 16:
        print('Packet length is not correct')
        return None
    if packet[0] != 0xA5 or packet[1] != 0x5A:
        print('Header bytes are not correct')
        return None
    if packet[15] != checksum_pc_generator(packet[:15]):
        print('Checksum is wrong')
        return None
    
    Xpos = (packet[3] << 8) | packet[4]
    Ypos = (packet[5] << 8) | packet[6]
    
    DWM_data = {
        'Xpos': Xpos,
        'Ypos': Ypos
    }

    print(DWM_data)
    
    return DWM_data

def checksum_generator(data):
    checksum = 0
    for byte in data:
        checksum += byte
    return checksum & 0xFF

def parse_MQTT_Astar(msg,serial):
    if msg[:2] != 'A5' or msg[2:4] != '5A':
        print('Header bytes are not correct')
        return None
    if msg[len(msg) - 1] != 'F':
        print('Message not complete!')
        return None
    
    length_of_coordinates = int(msg[4:].split('|')[0])
    if((length_of_coordinates>0) and ( msg[-1] == 'F') and (msg[-2] == 'F')):
        coordinates_part = msg[6:-2]
        coordinates_list = coordinates_part.split('|')
        coordinates = coordinates_list[1:]
        x_coordinates = []
        y_coordinates = []
        for coord in coordinates:
            x, y = map(int, coord.split(':'))  # Split the string and convert to integers
            x_coordinates.append(x) 
            y_coordinates.append(y) 
        end_marker = msg[-2:]

        for inc in range(length_of_coordinates//5):

            #Header Bytes
            result = [0xA5, 0x5A, 0x13]

            # Prepare ID Message
            result.append(inc)
            
            # Send Length of the Message
            result.append((length_of_coordinates//5))
            
            # Send first X
            if(x_coordinates):
                result.append(x_coordinates[inc]+1)
            else:
                result.append(0x00)

            # Send first Y
            if(y_coordinates):
                result.append(y_coordinates[inc])
            else:
                result.append(0x00)

            # Send second X
            if(x_coordinates):
                result.append(x_coordinates[inc+1])
            else:
                result.append(0x00)

            # Send second Y
            if(y_coordinates):
                result.append(y_coordinates[inc+1])
            else:
                result.append(0x00)

            # Send third X
            if(x_coordinates):
                result.append(x_coordinates[inc+2])
            else:
                result.append(0x00)

            # Send third Y
            if(y_coordinates):
                result.append(y_coordinates[inc+2])
            else:
                result.append(0x00)

            # Send fourth X
            if(x_coordinates):
                result.append(x_coordinates[inc+3])
            else:
                result.append(0x00)

            # Send fourth Y
            if(y_coordinates):
                result.append(y_coordinates[inc+3])
            else:
                result.append(0x00)

            # Send fifth X
            if(x_coordinates):
                result.append(x_coordinates[inc+4])
            else:
                result.append(0x00)

            # Send fifth Y
            if(y_coordinates):
                result.append(y_coordinates[inc+4])
            else:
                result.append(0x00)

            # Add Null Message
            result.append(0x00)
            result.append(0x00)
            result.append(0x00)

            # Add Checksum 
            chksm = checksum_generator(result)
            result.append(chksm)

            print(f"Send Data : `{bytearray(result)}`")

            serial.write(bytearray(result))

            time.sleep(1)

        print("Length of Coordinates:", length_of_coordinates)
        print("Coordinates:", coordinates)
        print("End Marker:", end_marker)


def parse_MQTT_Coordinate(msg,serial):

        #Header Bytes
        result = [0xA5, 0x5A, 0x12]

        # Check for positive/negative indicator
        pos_neg_indicator = 0x00 if msg[2] == 'P' else 0x10
        result.append(pos_neg_indicator)
        
        # Get X Position Data
        pos_x = int(msg[3:6])
        result.append(pos_x)

        # Check for positive/negative indicator
        pos_neg_indicator = 0x00 if msg[6] == 'P' else 0x10
        result.append(pos_neg_indicator)

        # Get Y Position Data
        pos_y = int(msg[7:10])
        result.append(pos_y)

        # Check for positive/negative indicator
        pos_neg_indicator = 0x00 if msg[10] == 'O' else 0x10
        result.append(pos_neg_indicator)

         # Get Y Position Data
        orient = int(msg[11:14])
        result.append(orient)

        # Check for Step
        pos_neg_indicator = 0x00 if msg[15] == 'S' else 0x73
        result.append(pos_neg_indicator)

        # Get Step Data
        step = int(msg[16:19])
        result.append(step)

        # Add Null Message
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)

        # Add Checksum 
        chksm = checksum_generator(result)
        result.append(chksm)

        print(f"Send Data : `{bytearray(result)}`")

        serial.write(bytearray(result))

def send_command(serial,x_speed,y_speed,t_speed):

        #Header Bytes
        result = [0xA5, 0x5A, 0x12]

        # Get X Speed Data
        result.append((x_speed >> 8) & 0xFF)
        result.append(x_speed & 0xFF)
        
        # Get Y Speed Data
        result.append((y_speed >> 8) & 0xFF)
        result.append(y_speed & 0xFF)

        # Get T Speed Data
        result.append((t_speed >> 8) & 0xFF)
        result.append(t_speed & 0xFF)
        

        # Add Null Message
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)
        result.append(0x00)

        # Add Checksum 
        chksm = checksum_generator(result)
        result.append(chksm)

        print(f"Send Data : `{bytearray(result)}`")

        serial.write(bytearray(result))
