import serial
# setup UART Tx pin(Board) 8 Rx pin(Board) 10 
uart = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
uart.close()
uart.open()

#uart.write("Please Type Something.\r\n".encode())

while True:
    string_echo = ""
    while len(string_echo) < 1:
        string_echo = uart.read_until('\r'.encode()).decode('utf-8')
    #uart.write((string_echo + "\r\n").encode())
    print (string_echo)
    """try:
        if 'stop' in string_echo:
            print ("stop close communication")
            uart.close()
            break
        else:
            string_echo = ''

    except KeyboardInterrupt:
         #clean up on exit
         uart.close()"""
    
    

