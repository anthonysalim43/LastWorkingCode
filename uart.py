import serial
import time
serial_port = "/dev/ttyUSB1"  # Change this to the appropriate port
#baud_rate = 500000
#baud_rate = 5000000
#baud_rate=10000
baud_rate=150000
memory = []
count=0
memory.clear()
with serial.Serial(serial_port, baud_rate) as device:
    
    memory = []
    memory.clear()
    while True:
        # start = time.time()
       #
       # 
       # input("Press any key :")
        data_bytes = device.read(1)  # Read one byte
        if len(data_bytes) == 1:
            byte1 = data_bytes
            print("byte =", byte1) 
            
           # input("Press any key :")
        if byte1 != b'z':
            count+=1
            memory.append(int.from_bytes(byte1,'big'))
            print("count=",count)
            print("memory length",len(memory))
            if count>=60:
                print("Break")
                break
                
        else:
            print("memory length",len(memory))
            #    if len(memory) != 0:
                    # print("Angle =", self.memory[0])
                    # print("2 =", self.memory[1])
                    # print("3 =", self.memory[2])
                    # print("4 =", self.memory[3])
                    #bytes_dist = bytes([memory[0], memory[1]])
                    #int_bytes_dist = int.from_bytes(bytes_dist, byteorder='big')
            #        print(count)
                    #print("byte value =", memory[count-1])
                    #voltage = (int_bytes_dist / 4096) * 3.3  # distance in cm
                    #print("Voltage =", voltage)
            #        print("byte value =", memory[count-1])

              
                   # memory.clear()
                  

       
    

count=0
first_byte=True
output_file_path = 'received_data.txt'
with open(output_file_path, 'w') as file:
# Iterate over the list in steps of 2
    for i in range(0, len(memory), 1):
        # Check if the next element exists to avoid IndexError

        
        if i+1 < len(memory):
            # Write current and next byte as a two-digit hex number together
           # file.write(f"{memory[i+1]:02x}{memory[i]:02x}\n")
            file.write(f"{memory[i]}\n")
            #file.write(f"{memory[i]:02x}\n")
            # Combine two bytes into one integer assuming they form a 16-bit numbe
            
            #combined_number = memory[i+1] * 256 + memory[i]  # Big-endian
            
            # Write the combined number as a decimal integer
            #file.write(f"distance {combined_number} mm \n")
            

       # else:
            # Write the last byte alone if no adjacent byte exists
       #     file.write(f"{memory[i]:02x}\n")
    memory.clear()
    print("Data successfully written to file.")