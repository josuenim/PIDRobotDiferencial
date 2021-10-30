import serial
import time
import collections
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from tkinter import *
from threading import Thread
import sys

def getData():
     time.sleep(1)
     serialConnection.reset_input_buffer()
     while (isRun):
          global isReceiving
          global value
          for i in range(sizeData):
               value[i] = float(serialConnection.readline().strip())
          isReceiving = True

def onClosing():
     global isRun
     isRun = False
     thread.join()
     serialConnection.close()
     root.quit()
     root.destroy()

def plotData(self,lines, lineValueText, lineLabel):
     for i in range(sizeData):
          data[i].append(value[i])
          lines[i].set_data(range(samples),data[i])
          lineValueText[i].set_text(lineLabel[i] + ' = ' + str(value[i]))


######### Declaracion de variables ##########

# Comunicacion Serial
serialPort = 'COM12'
baudRate  = 9600
sizeData = 2
isReceiving = False
isRun = True

# Graficas
samples  = 100
sampleTime = 100
value  = []
data = []

for i in range(sizeData):
     data.append(collections.deque([0]*samples, maxlen = samples))
     value.append(0)
     
xmin = 0
xmax = samples
ymin = 0
ymax = 11

lineLabel = ['Setpoint', 'Variable Medida']
style = ['r-', 'b-']
lines = []
lineValueText = []

################# Crear grafica ##################

fig = plt.figure(facecolor = '0.94')
ax = plt.axes(xlim= (xmin,xmax), ylim=(ymin,ymax))
plt.title('Grafica de Señales')
plt.grid()
ax.set_xlabel('Muestras')
ax.set_ylabel('Velocidad Angular')

for i in range(sizeData):
     lines.append(ax.plot([], [], style[i], label=lineLabel[i])[0])
     lineValueText.append(ax.text(0.65, 0.90-i*0.05, '', transform=ax.transAxes))

plt.legend(loc="upper left")

############## Abrir puerto serial ##############
try:
     serialConnection = serial.Serial(serialPort,baudRate)
except:
     sys.exit('No se puede conectar al puerto')

############## Recibir datos en segundo plano ############
thread = Thread(target = getData)
thread.start()

while isReceiving !=True:
     print('Iniciando recepcion de datos')
     time.sleep(0.1)
     
############### Intefaz Grafica ####################

root = Tk()
root.protocol('WM_DELETE_WINDOW', onClosing)

canvas = FigureCanvasTkAgg(fig, master = root)
canvas._tkcanvas.grid(row = 0,column = 0)


anim = animation.FuncAnimation(fig,plotData, fargs=(lines, lineValueText, lineLabel), interval=sampleTime)    # fargs has to be a tuple


root.mainloop()

















     
