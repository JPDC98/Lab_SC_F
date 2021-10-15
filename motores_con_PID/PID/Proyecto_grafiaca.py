import matplotlib.pyplot as plt
import matplotlib.animation as creador
import serial
import threading
import numpy as np

datos_t = [[],[],[],[]]
muestras = 200
num_sensores = 4

def conexion(out_data):
    try:
        com_serial = serial.Serial(port='COM6',#Poner dirección del dispositivo si es sistema linux; Windows verificar puerto asignado al bluetooth.
                            baudrate=115200, #Velidicad de recepción de datos, HC-06 por defecto esta a 9600, si utilizamos otro baudrate hay que modificarlo.
                            bytesize=serial.EIGHTBITS, #Numero de bits a contar y empaquetar
                            parity=serial.PARITY_NONE, #Desabilitamos bits de paridad por el momento
                            stopbits = serial.STOPBITS_ONE, #Se detiene la recepción de datos cuando se detecta un estado alto. 
                            timeout=None)#Se epera por tiempo indefinido los bytes
        print(com_serial.name) #imprimimos el puerto serial a utilizar, conexión correcta
        while(1):        
            datos = [int(com_serial.readline().strip().decode('ascii')) for a in range(num_sensores)] #Leemos los dos bits enviados de la FPGA
            out_data[0].append(datos[0])
            out_data[1].append(datos[1])
            out_data[2].append(datos[2])
            out_data[3].append(datos[3])

            if len(out_data[0]) > muestras:
                out_data[0].pop(0)
            if len(out_data[1]) > muestras:
                out_data[1].pop(0)
            if len(datos_t[2]) > muestras:
                datos_t[2].pop(0)
            if len(datos_t[3]) > muestras:
                datos_t[3].pop(0)   
    except:
        print("ERROR EN LA CONEXION") # se nos hace saber que hubo un error.

t_1 = threading.Thread(name="hilo1",target=conexion, args=(datos_t,))
t_1.start()

def inicio():
    ax.set_xlim(0,muestras)
    ax.set_ylim(0,50)
    del x1data[:]
    del y1data[:],y2data[:],y3data[:],y4data[:]
    linea1.set_data(x1data,y1data)
    linea2.set_data(x1data,y2data)
    linea3.set_data(x1data,y3data)
    linea4.set_data(x1data,y4data)
    return linea1,linea2,linea3,linea4,

def animacion(i,linea1,linea2,linea3,linea4,d_t):
    x1data = np.array(range(len(d_t[0])))
    y1data = np.array(d_t[0])
    y2data = np.array(d_t[1])
    y3data = np.array(d_t[2])
    y4data = np.array(d_t[3])
    ax.set_title("Datos Silla de rudedas")
    ax.set_xlabel("Tiempo")
    ax.set_ylabel("DATOS: velocidad[cm/s] , Distancia[cm]")
    ax.legend((linea1,linea2,linea3,linea4),("SetPoint","llanta_1","llanta_2","Distancia"),loc='lower left')
    linea1.set_data(x1data,y1data)
    linea2.set_data(x1data,y2data)
    linea3.set_data(x1data,y3data)
    linea4.set_data(x1data,y4data)
    return linea1,linea2,linea3,linea4,

graf, ax = plt.subplots()
linea1, = ax.plot([],[],label="distancia")
linea2, = ax.plot([],[],label="dato1")
linea3, = ax.plot([],[],label="dato2")
linea4, = ax.plot([],[],label="dato3")
 

x1data,y1data = [],[]
y2data,y3data,y4data = [],[],[]

funcion = creador.FuncAnimation(graf,animacion,fargs=(linea1,linea2,linea3,linea4,datos_t),init_func=inicio,interval=10,repeat=False,blit=False)

plt.show()
t_1.join()

