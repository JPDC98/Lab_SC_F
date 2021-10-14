from typing import Type
import serial
from serial.serialutil import Timeout

try:
    com_serial = serial.Serial(port='COM6',#Poner dirección del dispositivo si es sistema linux; Windows verificar puerto asignado al bluetooth.
                            baudrate=115200, #Velidicad de recepción de datos, HC-06 por defecto esta a 9600
                            bytesize=serial.EIGHTBITS, #Numero de bits a contar y empaquetar
                            parity=serial.PARITY_NONE, #Desabilitamos bits de paridad por el momento
                            stopbits = serial.STOPBITS_ONE, #Se detiene la recepción de datos cuando se detecta un estado alto. 
                            timeout=None)
    print(com_serial.name) #imprimimos el puerto serial a utilizar, conexión correcta
    while(1):
        datos_bytes = com_serial.read(1) #Leemos los dos bits enviados de la FPGA
        print(datos_bytes)

except:
    com_serial.close() # se cierra la comunicación para evitar loops indeseados.
    print("ERROR") # se nos hace saber que hubo un error.