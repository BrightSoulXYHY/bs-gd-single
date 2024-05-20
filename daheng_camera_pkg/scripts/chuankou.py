import serial

ser = serial.Serial('COM1',9600)    #create chuankou
ser.open()   #connected to chuankou
data = ser.read(10)   # read data of chuankou
ser.write("hello")    #write data to chuankou
ser.close()   #close chuankou
