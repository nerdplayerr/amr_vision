
#library
import PySimpleGUI as sg
import threading
import time
import serial
import serial.tools.list_ports
from ultralytics import YOLO
from ultralytics.engine.results import Results
import cv2
from collections import defaultdict
import pandas as pd

# import keyboard
import os
import pygame


#----------------Mas Munir--------------#


import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


#---------------------------------------#


# https://docs.ultralytics.com/python/

# PREDEFINE CP

# Predefined control points
control_points = [
            [509, 326],
            [536, 6292],
            [669, 6398],
            [3917, 6425],
            [3433, 6478],
            [4369, 6691],


            #Finish poin
            [4395, 10335],
            [4395, 10335],
            [4395, 10335],
            # Reverse / balik
            # [2981, 6691],
            # [3167, 6478],
            # [3433, 6425],
            # [6681, 6398],
            # [6814, 6292],
            # [6841, 5654]
            ]
#Tema
sg.theme('DarkGrey2')


#YOLO

model = YOLO('misteriusbest.engine',task='detect')
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
scale_show = 100 # Provide the scale percent directly
id_obj = 0
las_track_id = 0
coord = [0,0]
ball_none = 0
none_ball = 0
header = ""
las_xSTM = 0
las_ySTM = 0

left_wall = 120
right_wall = 240
upperWall_cutoff = 10



koordinat_terkecil=[]
x_ball = 0
y_ball = 0
# Store the track history

track_history = defaultdict(lambda: [])
track = []
closest_point = [0,0]
ballReadytoGrip = False

sudut = 0.0

tolerance = 5.0

"""
mode_robot = 1 : robot dalam mode trajektori,STM menerima data dari fungsi generate bspline
mode_robot = 2 : robot dalam mode yolo, STM menerima data koordinat pixel gambar objek dari yolo
"""
######### JANGAN DIUBAH NILAINYA #########
mode_robot = 1
##########################################

# Using resizeWindow()
#############################  SERIAL MODE   ###################################


""""
2 = Not Connected with STM32
1 = Connected with STM32
"""
serial_mode = 0


################################################################################



########################    POSISI PENGAMBILAN BOLA     ########################
grip_point = (320, 340)
########################
grip_size = 100
grip_area = 10 #radius lingkaran
grip_distance = 100 
########################           YNH            #########################
"""
1 = TIM BIRU
2 = TIM MERAH
3 = NONE
"""
team = 3 #jangan diubah
########################


#---------------SAVE SILO---------------
# Define a dictionary to store the rectangles
rectangles = {}

# Define a dictionary to store the text annotations
rect_text_dict= {}

# Define a dictionary to store the colors for each silo
colors = {'Silo1': 'red', 'Silo2': 'green', 'Silo3': 'blue'}

SiloButtonState = ''

saveSilo = []

def BacaDataSilo() :
    # Read from the serial port and print the position
    print(ser.in_waiting)
    #if ser.in_waiting > 0:

    print("Ser Open")
    # ser.open()
    while 1:
        print('TEST')
        line = ser.readline().decode('utf-8').rstrip()
        print('DATA MASUK')
        header, TfLuna, Sick = line.split(' ')

        print(header, xSilo, ySilo)

        TfLuna = float(TfLuna)
        Sick = float(Sick) 

        #Kalkulasi belum jadi.
        xSilo = 7350 - TfLuna * 10
        ySilo = 12000 - Sick * 10


        if header == 'S0' or header == 'S1' or header == 'S2'or header == 'S3':
            break

    print(header, xSilo, ySilo)


    if header == 'S0' or header == 'S1' or header == 'S2'or header == 'S3':
        # If saveSilo is empty, append the new value
        if not saveSilo:
            saveSilo.append((xSilo, ySilo))
        else:
            # If saveSilo already has a value, update the first element
            saveSilo[0] = (xSilo, ySilo)

        #print(ser.in_waiting)
        #print(saveSilo)

        #print('pembatas')

        #print(saveSilo[0])

        # If a rectangle for this silo already exists, remove it
        if SiloButtonState in rectangles:
            rectangles[SiloButtonState].remove()

        # Draw a new rectangle at the clicked location
        rectangle = plt.Rectangle((xSilo, ySilo), 200, 200, fill=False)
        ax.add_patch(rectangle)  # Adds the rectangle to the plot

        # Store the rectangle in the dictionary
        rectangles[SiloButtonState] = rectangle

#-------------------------------------------------------------




def tim(): 
    if(team == 1):
        return "Tim Biru"
    elif(team == 2):
        return "Tim Merah"
    else:
        return "Tim None"

class merah:
 id = []
 distances = []
 closest_distance = []
 sorted_distances = []
 closest_point = []
 sorted_cord_x = []
 sorted_cord_y = []
 x = [] 
 y = []

class biru:
 id = []
 sorted_cord_x = []
 sorted_cord_y = []
 x = []
 y = []
 distances = []
 closest_distance = []
 closest_point = []
 sorted_distances = []


class ungu:
 closest_distance = []
 closest_point = []
 sorted_distances = []
 id = []
 sorted_cord_x = []
 sorted_cord_y = []
 x = []
 y = []
 distances = []


org = (50, 50)
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
thickness = 2


# Create the serial object
if(serial_mode):
    ser = serial.Serial('/dev/ttyACM0', 115200) #STM
    #ser = serial.Serial('/dev/ttyUSB0', 9600) #ARDUINO
    pass


#Variable PID
# Define the default PID settings
default_pid_settings = {
   '-KP1-': '1.0',
   '-KI1-': '0.0',
   '-KD1-': '0.0',
   '-KP2-': '1.0',
   '-KI2-': '0.0',
   '-KD2-': '0.0',
   '-KP3-': '1.0',
   '-KI3-': '0.0',
   '-KD3-': '0.0',
   '-KP4-': '1.0',
   '-KI4-': '0.0',
   '-KD4-': '0.0',
}


# Preset 1
preset1_pid_settings = {
   '-KP1-': '0.0',
   '-KI1-': '0.0',
   '-KD1-': '0.0',
   '-KP2-': '0.0',
   '-KI2-': '0.0',
   '-KD2-': '0.0',
   '-KP3-': '0.0',
   '-KI3-': '0.0',
   '-KD3-': '0.0',
   '-KP4-': '0.0',
   '-KI4-': '0.0',
   '-KD4-': '0.0',
}


# Preset 2
preset2_pid_settings = {
   '-KP1-': '0.0',
   '-KI1-': '0.0',
   '-KD1-': '0.0',
   '-KP2-': '0.0',
   '-KI2-': '0.0',
   '-KD2-': '0.0',
   '-KP3-': '0.0',
   '-KI3-': '0.0',
   '-KD3-': '0.0',
   '-KP4-': '0.0',
   '-KI4-': '0.0',
   '-KD4-': '0.0',
}


# Preset 3
preset3_pid_settings = {
   '-KP1-': '0.0',
   '-KI1-': '0.0',
   '-KD1-': '0.0',
   '-KP2-': '0.0',
   '-KI2-': '0.0',
   '-KD2-': '0.0',
   '-KP3-': '0.0',
   '-KI3-': '0.0',
   '-KD3-': '0.0',
   '-KP4-': '0.0',
   '-KI4-': '0.0',
   '-KD4-': '0.0',
}






# Initialize the current preset
current_preset = 'PID 1'






#Model Name
modelname='model_name'


# Define the keys you want to block
blocked_keys = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
               'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
               'enter', 'space', 'tab', 'backspace', '+', '-', '/', '*',
               'left windows', 'right windows', 'esc', 'f1', 'f2', 'f3', 'f4',
               'f5', 'f6', 'f7', 'f8', 'f9', 'f10', 'f11', 'f12']


blocked_comma = ','


# Create a flag to track if comma is already pressed in each input field
comma_pressed = {}


# Block the keys
# for key in blocked_keys:
#     keyboard.block_key(key)




#List print status



def comma_detector(e):
   global comma_pressed
   if e.event_type == 'down':
       comma_pressed[selected_key] = True


def get_com_ports():
   ports = serial.tools.list_ports.comports()
   return [port.device for port in ports] if ports else ['No COM ports available']


# Mode Save Silo sent to STM
def SendDataStm(data):
 if(serial_mode):
   ser.write(str(data).encode())


# Add a hook for comma key
# keyboard.on_press_key(',', comma_detector)


# Input Suara
def sound(path):
    pygame.mixer.init()
    pygame.mixer.music.load(path)
    pygame.mixer.music.play()

column_layout1_status = [
           [sg.Text('Status: OFF', key='-STATUS-', font=("Bahnschrift SemiBold", 15,"bold"))],
           [sg.Text('Team: None', key='-TEAM-', font=("Bahnschrift SemiBold", 15,"bold"))],
]


column_layout2_PID1 = [
           [sg.Text('PID 1',font=("Arial",10,"bold"))],
           [sg.Button('Select KP', button_color=('black', 'GreenYellow'), key='KP1'), sg.Input(size=(20, 1), key='-KP1-') ],
           [sg.Button('Select KI  ', button_color=('black', 'GreenYellow'), key='KI1'), sg.Input(size=(20, 1), key='-KI1-'), ],
           [sg.Button('Select KD', button_color=('black', 'GreenYellow'), key='KD1'), sg.Input(size=(20, 1), key='-KD1-'), ],
           [sg.Text('PID 2',font=("Arial",10,"bold"))],
           [sg.Button('Select KP', button_color=('black', 'GreenYellow'), key='KP2'), sg.Input(size=(20, 1), key='-KP2-'), ],
           [sg.Button('Select KI  ', button_color=('black', 'GreenYellow'), key='KI2'), sg.Input(size=(20, 1), key='-KI2-'), ],
           [sg.Button('Select KD', button_color=('black', 'GreenYellow'), key='KD2'), sg.Input(size=(20, 1), key='-KD2-'), ],
           [sg.Text('PID 3',font=("Arial",10,"bold"))],
           [sg.Button('Select KP', button_color=('black', 'GreenYellow'), key='KP3'), sg.Input(size=(20, 1), key='-KP3-'), ],
           [sg.Button('Select KI  ', button_color=('black', 'GreenYellow'), key='KI3'), sg.Input(size=(20, 1), key='-KI3-'), ],
           [sg.Button('Select KD', button_color=('black', 'GreenYellow'), key='KD3'), sg.Input(size=(20, 1), key='-KD3-'), ],
           [sg.Text('PID 4',font=("Arial",10,"bold"))],
           [sg.Button('Select KP', button_color=('black', 'GreenYellow'), key='KP4'), sg.Input(size=(20, 1), key='-KP4-'), ],
           [sg.Button('Select KI  ', button_color=('black', 'GreenYellow'), key='KI4'), sg.Input(size=(20, 1), key='-KI4-'), ],
           [sg.Button('Select KD', button_color=('black', 'GreenYellow'), key='KD4'), sg.Input(size=(20, 1), key='-KD4-'), ],
           [sg.Text('',font=("Arial",10,"bold"))],
           [sg.Button('Restore To Default', button_color=('White','Red'), key='RestoreDefault'), sg.Push(), sg.Button('Preset 1', key='Preset1', button_color=('black', 'GreenYellow'))],
           [sg.Button('Save To Preset', button_color=('White','blue'), key='SavePreset'), sg.Push(), sg.Button('Preset 2', key='Preset2', button_color=('black', 'GreenYellow'))],
           [sg.Push(), sg.Button('Preset 3', key='Preset3', button_color=('black', 'GreenYellow'))],
]


column_layout3_MAP = [
   # Zoom slider
   [sg.Slider(range=(1,10), orientation='h', size=(34,20), default_value=1, key='zoom_slider')],


   # X slider
   [sg.Slider(range=(0, 6684), orientation='h', size=(34, 20), default_value=0, key='x_slider', tick_interval=1000)],


   # Y Slider and Canvas
   [sg.Slider(range=(0, 10800), orientation='v', size=(20, 20), default_value=0, key='y_slider', tick_interval=1000), sg.Canvas(key='canvas')],
]




# Define the layout
layout = [
  
   [sg.Text('ROBOT 2', font=("Bahnschrift SemiBold", 20,"bold"),), sg.Text('Pos : ', key='-TP-'), sg.Push(), sg.Button('Exit')],


   [sg.Button('START', size = (15,6),button_color=('white', 'green'), border_width=5, key='-ON-', font=('Helvetica', 10,'bold')),sg.Button('STOP', size = (15,6), button_color=('white', 'red'), border_width=5, key='-OFF-', font=('Helvetica', 10,'bold')), sg.Button('RETRY', size = (15,6), button_color=('White', 'Gray1'), border_width=5, key='-RETRY-', font=('Helvetica', 10,'bold')),sg.Column(column_layout1_status, element_justification='center'), sg.OptionMenu(values=get_com_ports(), key='-COM-', default_value='No COM ports available'), sg.Push(), sg.Button(image_filename=r'/home/bandhayudha/Documents/GUI/Projek_bandha/B.png',image_size=(100, 100), image_subsample=5, border_width=0, button_color=(sg.theme_background_color(), sg.theme_background_color()), key='-MAP-')], #ON OFF
   [sg.HorizontalSeparator(color='Gray1')],


   [sg.vtop(sg.TabGroup([[
       sg.Tab('Team', [
           [sg.Text('Team', font=("Arial", 20))],
           [sg.Button('MERAH', size=(15,14), key='-MERAH-', button_color=('white', 'red'), font=('Helvetica', 10,'bold')),sg.Button('BIRU', size=(15,14), key='-BIRU-', button_color=('white', 'blue'), font=('Helvetica', 10,'bold'))],
           [sg.Button('Reset TEAM', size=(32,6), key='-RESET-', button_color=('black', 'Gray'))],
       ]),
       sg.Tab('Control', [
           [sg.Button(str(i), size=(8,4), key=str(i), button_color=('black', 'GreenYellow')) for i in range(1, 4)],
           [sg.Button(str(i), size=(8,4), key=str(i), button_color=('black', 'GreenYellow')) for i in range(4, 7)],
           [sg.Button(str(i), size=(8,4), key=str(i), button_color=('black', 'GreenYellow')) for i in range(7, 10)],
           [sg.Button('0', size=(8,4), key='0', button_color=('black', 'GreenYellow')), sg.Button('.', size=(18,4), key='-.-', button_color=('black', 'GreenYellow'),)],
           [sg.Button('Clear', size=(10, 2), button_color=('black', 'GreenYellow')), sg.Button('Submit', size=(10, 2), button_color=('black', 'GreenYellow'))],
       ])
   ]], selected_background_color='GreenYellow', selected_title_color='Black')),sg.VerticalSeparator(),
   sg.vtop(sg.TabGroup([[
       sg.Tab('PID 1', [
           [sg.Column(column_layout2_PID1, scrollable=True, vertical_scroll_only=True, sbar_trough_color='gray', sbar_arrow_color='black', sbar_background_color='GreenYellow')],
       ],),
       sg.Tab('TAB 2', [
           [sg.Button('Start From Retry', key='Button1')],
           [sg.Button('Start From Start', key='Button2')],
       ]),
       sg.Tab('TAB 3', [
           [sg.Button('Silo 1', key='Silo1')],
           [sg.Button('Silo 2', key='Silo2')],
           [sg.Button('Silo 3', key='Silo3')],
           [sg.Button('Save Silo', key='SaveRect')],
           [sg.Button('Load Silo', key='LoadRect')],
           [sg.Button('Clear Silo', key='EraseRect')],
           [sg.Text('')],
           [sg.Button('Go TO Silo', key='-GS-')]
       ]),
       sg.Tab('TAB 4', [
           [sg.Text('Pos : ', key='-TP-')],
           [sg.Button('WL', key='WL')],
           [sg.Button('WX', key='WX')],
           [sg.Button('WY', key='WY')],
           [sg.Button('WR', key='WR')],
           [sg.Button('ORIGIN', key='Z0')],
           [sg.Text('')],
           [sg.Button('SP', key='SP')],


       ])
   ]],selected_background_color='GreenYellow', selected_title_color='Black')),sg.VerticalSeparator(),
   sg.vtop(sg.TabGroup([[
       sg.Tab('MAP',
           [
               [sg.Column(column_layout3_MAP)],
           ]),
       sg.Tab('VIDEO',
           [
              [sg.Image(filename='', key='image',expand_x=True, expand_y=True)],   
           ])
   ]])),sg.VerticalSeparator(),
   sg.vtop(sg.TabGroup([[]]))


   ],
   [sg.HorizontalSeparator(color='Gray1')],




   [sg.Button('Exit', size=(8,4), button_color=('black', 'GreenYellow'))]
]


# Create the window
window = sg.Window('Bandha Robot 2', layout, resizable=True, finalize=True)
window.maximize()  # Add this line to maximize the window
run_model = False


selected_key = None


# Create a flag for each input field
clear_input = False


# Initialize a variable to keep track of the previously selected PID button
previous_pid_button = None


# Initialize a variable to keep track of the previously selected Preset button
previous_preset_button = None


com_ports = get_com_ports()


#----------------Mas Munir--------------#


# Add the plot to the window
fig, ax = plt.subplots()


# Set the aspect ratio of the axes
ax.set_aspect('equal')


ax.set_xlim([0, 7350])  # Set the x-axis limits
ax.set_ylim([0, 120000])  # Set the y-axis limits
ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.yaxis.set_ticks_position('left')


# Set the color of x and y axis to greenyellow
ax.spines['left'].set_color('black')
ax.spines['bottom'].set_color('black')
ax.xaxis.label.set_color('black')
ax.yaxis.label.set_color('black')
ax.tick_params(colors='black')


# Load the background image
background_image = plt.imread(r'/home/bandhayudha/vision/TestYolo - Copy/MAP.png')
ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Set alpha for transparency


# Reduce the gap between the border and the graph
plt.subplots_adjust(left=0.02, right=0.98, top=0.97, bottom=0.03)


# Line & Trajectory
scatter = ax.scatter([], [])
line, = ax.plot([], [], '-')
trj, = ax.plot([], [], '-')


# Add the plot to the window
canvas = FigureCanvasTkAgg(fig, master=window['canvas'].TKCanvas)
canvas.draw()
canvas.get_tk_widget().pack(side='top', fill='both', expand=1)


points = []


#--------------------------B-Spline--------------------------
max_control_point = 0
last_max_control_point = 0
degree = 3
knot_vector = []
control_points = []
bspline_points = []


# Variable input
max_acc = 1.0
max_speed = 100.0
des = 0


# Variable code
acc = 0.0
current_speed = 0.0 # Belum ada satuan
position = 0.0
pos_save_max_s = 0.0
n = 0

# For Stoping the Generate bspile
running = False


def generate_knot_vector ():
   ln = max_control_point + degree + 1
   n = 1.0 / (max_control_point - degree)
   x = 0
   knot_vector.clear()
   for i in range (0, ln):
       if i < degree:
           knot_vector.append(0.0)
       elif i >= (ln-degree):
           knot_vector.append(1.0)
       else:
           knot_vector.append(x)
           x+=n


def basis (i, k, t):
   if k == 1:
       if knot_vector[i] <= t and t < knot_vector[i+1]:
           return 1.0
       else:
           return 0.0
   else:
       denom1 = knot_vector[i+k-1] - knot_vector[i]
       denom2 = knot_vector[i+k] - knot_vector[i+1]
       term1 = 0
       term2 = 0
       if denom1 != 0:
           term1 = ((t - knot_vector[i]) / denom1) * basis(i, k-1, t)
       if (denom2 != 0):
           term2 = ((knot_vector[i+k] - t) / denom2) * basis(i+1, k-1, t)
       return term1+term2
  
def evaluate_cubic_bspline (t):
   x = 0
   y = 0
   for i in range (0, max_control_point):
       b = basis(i, degree+1, t)
       x += control_points[i][0]*b
       y += control_points[i][1]*b
   return x, y


def trapezoidal_speed():
   global current_speed
   global max_speed
   global max_acc
   global acc
   global position
   global pos_save_max_s
   global n
  
   if n == 0:
       if current_speed < max_speed:
           acc = max_acc
           if position > des / 2 :
               acc = -max_acc
       else:
           if n == 0:
               n = 1
               pos_save_max_s = position
               acc = 0
   else:
       if (des - position) < pos_save_max_s:
           acc = -max_acc




   current_speed += acc
   if current_speed < 10.0:
       current_speed = 10.0


   delay = 1.0 / current_speed #Cm/sec
   return delay


def generate_bspile():
   
   global running
  
   last_x_bspline = 0.0
   last_y_bspline = 0.0
   x_bspline = 0.0
   y_bspline = 0.0
   dx = 0.0
   dy = 0.0
   d  = 0.0
   t  = 0.0
   generate_knot_vector()
   #kalkulasi


   global des
   global current_speed 
   global position
   global n
   global sudut

   global header
   global xSTM
   global ySTM

   des = 0.0
   position = 0.0
   while running and t < 1.0 :
       x_bspline , y_bspline = evaluate_cubic_bspline(t)
       if t > 0 :
           dx = x_bspline - last_x_bspline
           dy = y_bspline - last_y_bspline
           d = math.sqrt(dx*dx + dy*dy)
           if d > 10 :
               last_x_bspline = x_bspline
               last_y_bspline = y_bspline
               des += d
               # print(f'{des} + {d}, {t}')
       else:
               last_x_bspline = x_bspline
               last_y_bspline = y_bspline
       t += 0.0001


   t = 0.0
   last_x_bspline = 0.0
   last_y_bspline = 0.0
   n = 0
   while running and t < 1 :
        x_bspline , y_bspline = evaluate_cubic_bspline(t)
        if t > 0 :
           dx = x_bspline - last_x_bspline
           dy = y_bspline - last_y_bspline
           d = math.sqrt(dx*dx + dy*dy)
           if d > 10 :
               last_x_bspline = x_bspline
               last_y_bspline = y_bspline
               x = int(x_bspline)
               y = int(y_bspline)
               
    
               if x > 2956-10 and x > 2956+10 and y >= 7994 :
                sudut = -90.0

               # Prepare data
               # data = "A 0 {} {} {}\n".format(x, y, 0) # Data yang dikirim
               # RobotPort.write(data.encode())
               position += d
               # print(f'{position}, {d}')


            #    delay = trapezoidal_speed()
               global mode_robot

               pos = x,y

               base_speed = 10
               # Belokan pertama
               if x > 2956-10 and x > 2956+10 and y >= 7994 :
                   base_speed = -90.0

               print(x,y)
               # Kirim data XY ke Linux
               if(serial_mode == 1 and mode_robot == 1):
                 #SendDataStm(f'T {base_speed} {x} {y} {sudut}\n')
                 SendDataStm(f'T {x} {y} {sudut}\n')

               #print(f'INI LOH JALAN GA {pos}, {control_points[-1]}')

            #    if(pos is control_points[-1] and serial_mode):
            #         SendDataStm(f'B {control_points[-1]} {sudut}')
            #         mode_robot = 2

               while 1 :
                #print("A2")
                line = ser.readline().decode('utf-8').rstrip()
                header, xSTM, ySTM, yawSTM = line.split(' ')
                xSTM = float(xSTM)
                ySTM = float(ySTM)
                yawSTM = float(yawSTM)

                #print("A3")
                               
                if header == "P":
                    #print(header, xSTM, ySTM, yawSTM)
                    break
                    # while( math.fabs(xSTM - x) > tolerance or math.fabs(ySTM - y)> tolerance):
                    # if xSTM is not x and ySTM is not y :
                        # wait()
                        # pass
                        # print("jalan")
                        # time.sleep(0.001)

                    # else :
                        # print(f'{position}, {d}')

                        # delay = trapezoidal_speed()

                        # print(f'{x} , {y}')

                        # time.sleep(delay)
        else:
               last_x_bspline = x_bspline
               last_y_bspline = y_bspline
        t += 0.0001

   print('B')
   SendDataStm(f'B {control_points[-1]} {sudut}')
   mode_robot = 2


#--------------------------B-Spline--------------------------




def onclick(event):
   global max_control_point
   if event.xdata is not None and event.ydata is not None:
       if event.xdata >= 0.0 and event.xdata <= 7350.0 and event.ydata >=0.0 and event.ydata <= 12000.0:
          
           #Save Pos X and Y
           event.xdata


           # print(f'{event.xdata} , {event.ydata}')
           ax.scatter(event.xdata, event.ydata)
           # ax.text(event.xdata, event.ydata-0.5, f'({event.xdata:.2f}, {event.ydata:.2f})', verticalalignment='top')
           points.append((event.xdata, event.ydata))
           line.set_data(zip(*points))


          
           control_points.append([event.xdata, event.ydata])
           max_control_point+=1
           bspline_points.clear()
           if max_control_point > 3:
               generate_knot_vector()
               for t in np.arange(0.0, 1.0, 0.001):
                   x, y = evaluate_cubic_bspline(t)
                   bspline_points.append((x, y))
                   trj.set_data(zip(*bspline_points))
                   # print (x, y)


           canvas.draw()
       else :
           print('Out of Bound')
   else :
       print('Out of Bound')


# ------------------------ BARU MASUKIN KE YANG DI ORIN -------------------



# Set the initial number of control points
max_control_point = len(control_points)

# Draw the control points on the plot
for point in control_points:
    ax.scatter(*point)

# If there are more than 3 control points, generate the B-Spline curve
if max_control_point > 3:
    generate_knot_vector()
    for t in np.arange(0.0, 1.0, 0.001):
        x, y = evaluate_cubic_bspline(t)
        bspline_points.append((x, y))
    trj.set_data(zip(*bspline_points))

canvas.draw()  # Redraw the canvas


# ------------------------ SAMPE SINI ---------------------------------------


fig.canvas.mpl_connect('button_press_event', onclick)


#---------------------------------------#

runningSp = '0'
# ---TESTING 08 -------

def Tp() :
    global runningSp
    # Read from the serial port and print the position
    #print(ser.in_waiting)
    #if ser.in_waiting > 0:

    #print("Ser Open")
    # ser.open()
    while 1:
        #print('TEST')
        line = ser.readline().decode('utf-8').rstrip()
        #print('DATA MASUK')
        header, xTp, yTp, YAW = line.split(' ')

        #print(header, xTp, yTp)

        xTp = float(xTp)
        yTp = float(yTp)
        YAW = float(YAW) 



        if header == 'TP' or header == 'SP':
            break
    
    if header == 'TP':
        print(xTp, yTp, YAW)
        window['-TP-'].update(f"POS : {xTp}, {yTp}, {YAW}")

    elif header == 'SP' :
        runningSp = '0'
        #os.system('cls' if os.name == 'nt' else 'clear')
        pass

def GS():
    line = ''
    while 1:
        print('in Waiting')
        line = ser.readline().decode('utf-8').rstrip()
        print(f'DATA MASUK {line}')
        header, OK = line.split(' ')
        print(header, OK)
        #print(header, xTp, yTp)





        if header == 'OK' :
            break
    
    if header == 'OK':
        SendDataStm(f'STB\n')
        header =''
        OK = ''

# --------------------

inc=0

while True:
   event, values = window.read(timeout=1)  # Non-blocking read


   # Update the x-axis and y-axis limits based on the slider values
   zoom = values['zoom_slider']
   x = values['x_slider']
   y = values['y_slider']
   ax.set_xlim([x, x + 7350/zoom])  # Set x limits based on X slider value
   ax.set_ylim([y, y + 12000/zoom])  # Set y limits based on Y slider value
   canvas.draw()
   #threading.Thread(target=Tp).start() 

   if runningSp == '1':
    Tp()

   if event == '-GS-':
       SendDataStm(f'GS{inc}\n')
       print(f'GS{inc}')
       GS()
       inc += 1
       

   if event == sg.WIN_CLOSED or event == 'Exit':
       break
       
   elif event == 'TIMEOUT':
       # Zoom
       zoom = values['zoom_slider']
       x = values['x_slider']
       y = values['y_slider']
       ax.set_xlim([x, x + 7350/zoom])  # Set x limits based on X slider value
       ax.set_ylim([y, y + 12000/zoom])  # Set y limits based on Y slider value
       canvas.draw()


   elif event in ['KP1', 'KI1', 'KD1', 'KP2', 'KI2', 'KD2', 'KP3', 'KI3', 'KD3', 'KP4', 'KI4', 'KD4']:
       # If a PID button was previously selected, change its color back to normal
       if previous_pid_button is not None:
           window[previous_pid_button].update(button_color=('black', 'GreenYellow'))  # Change 'grey' to your original button color


       selected_key = '-' + event + '-'
       clear_input = True  # Set the flag when a button is pressed


       # Change the color of the selected button to green and update the previously selected PID button
       window[event].update(button_color=('white', 'green'))
       previous_pid_button = event


   elif event in [str(i) for i in range(10)] and selected_key:
       if clear_input:
           # Clear the input field when starting to type a new value
           window[selected_key].update(event)
           clear_input = False  # Reset the flag
       else:
           # Append the number to the current value of the selected input field
           # play_sound(r'C:\Users\eatmi\OneDrive\Documents\Projek_bandha\TestYolo\Keypad.wav')
           window[selected_key].update(values[selected_key] + event)
          
   elif event == '-.-' and selected_key:
       # Only append a dot if there isn't one already
       if '.' not in values[selected_key]:
           window[selected_key].update(values[selected_key] + '.')
           # play_sound(r'C:\Users\eatmi\OneDrive\Documents\Projek_bandha\TestYolo\Keypad.wav')


   # Rest of your code


   elif event == 'SavePreset':
   # Check which preset is currently selected
       if current_preset == 'Preset1':
           # Save the current PID settings to Preset 1
           for key in preset1_pid_settings.keys():
               preset1_pid_settings[key] = values[key]
           with open('preset1.txt', 'w') as f:
               f.write(str(preset1_pid_settings))
       elif current_preset == 'Preset2':
           # Save the current PID settings to Preset 2
           for key in preset2_pid_settings.keys():
               preset2_pid_settings[key] = values[key]
           with open('preset2.txt', 'w') as f:
               f.write(str(preset2_pid_settings))
       elif current_preset == 'Preset3':
           # Save the current PID settings to Preset 3
           for key in preset3_pid_settings.keys():
               preset3_pid_settings[key] = values[key]
           with open('preset3.txt', 'w') as f:
               f.write(str(preset3_pid_settings))
       else:
           print("Please select a preset before saving.")




   elif event == 'RestoreDefault':
       # Update the input fields with the default PID settings
       for key, value in default_pid_settings.items():
           window[key].update(value)


   # Only update the PID values when a preset button is pressed
   elif event in ['Preset1', 'Preset2', 'Preset3']:
       # If a preset button was previously selected, change its color back to normal
       if previous_preset_button is not None:
           window[previous_preset_button].update(button_color=('black', 'GreenYellow'))  # Change 'grey' to your original button color
       # Update the current preset
       current_preset = event
       print(f"Current preset: {current_preset}")  # Debugging print statement
       # Check which preset is currently selected and update the PID values
       filename = current_preset.lower() + '.txt'
       if os.path.exists(filename):
           with open(filename, 'r') as f:
               preset_settings = eval(f.read())
           for key, value in preset_settings.items():
               window[key].update(value)
       else:
           print(f"No saved settings for preset: {current_preset}")
       # Change the color of the selected button to green and update the previously selected preset button
       window[event].update(button_color=('white', 'green'))
       previous_preset_button = event
       print(f"Previous preset button: {previous_preset_button}")  # Debugging print statement


   # Testing
   elif event == 'Button1' : #Start From Retry
        
        if team == 3 :
            sg.popup("Pilih Tim Terlebih dahulu")
        
        elif team == 1: #biru
            ax.cla()  # Clear the axes


            ax.spines['left'].set_position('zero')
            ax.spines['bottom'].set_position('zero')
            ax.spines['right'].set_color('none')
            ax.spines['top'].set_color('none')
            ax.xaxis.set_ticks_position('bottom')
            ax.yaxis.set_ticks_position('left')


            # Clear the data structures
            points.clear()
            control_points.clear()
            bspline_points.clear()
            max_control_point = 0
            line, = ax.plot([], [], '-')
            trj, = ax.plot([], [], '-')
            # Predefined control points
            control_points = [

            [6841, 5654],
            [6814, 6292],
            [6681, 6398],
            [3433, 6425],
            [3167, 6478],
            [2981, 6691],
            [2981, 8000],
            [2981, 9500],
            ]


            # Set the initial number of control points
            max_control_point = len(control_points)

            # Draw the control points on the plot
            for point in control_points:
                ax.scatter(*point)

            # If there are more than 3 control points, generate the B-Spline curve
            if max_control_point > 3:
                generate_knot_vector()
                for t in np.arange(0.0, 1.0, 0.001):
                    x, y = evaluate_cubic_bspline(t)
                    bspline_points.append((x, y))
                trj.set_data(zip(*bspline_points))

            if team == 1 : # biru
                background_image = np.fliplr(plt.imread(r'/home/bandhayudha/vision/TestYolo - Copy/MAP.png'))  # Load and flip the image
                ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image


        elif team == 2 : #tim merah
            ax.cla()  # Clear the axes


            ax.spines['left'].set_position('zero')
            ax.spines['bottom'].set_position('zero')
            ax.spines['right'].set_color('none')
            ax.spines['top'].set_color('none')
            ax.xaxis.set_ticks_position('bottom')
            ax.yaxis.set_ticks_position('left')


            # Clear the data structures
            points.clear()
            control_points.clear()
            bspline_points.clear()
            max_control_point = 0
            line, = ax.plot([], [], '-')
            trj, = ax.plot([], [], '-')
            # Predefined control points
            control_points = [

            [509, 5654],
            [536, 6292],
            [669, 6398],
            [3917, 6425],
            [4183, 6478],
            [4369, 6691],
            [4369, 8000],
            [4369, 9500],
            ]


            # Set the initial number of control points
            max_control_point = len(control_points)

            # Draw the control points on the plot
            for point in control_points:
                ax.scatter(*point)

            # If there are more than 3 control points, generate the B-Spline curve
            if max_control_point > 3:
                generate_knot_vector()
                for t in np.arange(0.0, 1.0, 0.001):
                    x, y = evaluate_cubic_bspline(t)
                    bspline_points.append((x, y))
                trj.set_data(zip(*bspline_points))

            if team == 2 : # merah
                background_image = plt.imread(r'/home/bandhayudha/vision/TestYolo - Copy/MAP.png')  # Load the original image
                ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image
            canvas.draw()  # Redraw the canvas


        else :
            sg.popup("???")
       
   elif event == 'Button2': #Start From Start
        
        if team == 3 :
            sg.popup("Pilih Tim Terlebih dahulu")
        
        elif team == 1: #biru
            ax.cla()  # Clear the axes


            ax.spines['left'].set_position('zero')
            ax.spines['bottom'].set_position('zero')
            ax.spines['right'].set_color('none')
            ax.spines['top'].set_color('none')
            ax.xaxis.set_ticks_position('bottom')
            ax.yaxis.set_ticks_position('left')


            # Clear the data structures
            points.clear()
            control_points.clear()
            bspline_points.clear()
            max_control_point = 0
            line, = ax.plot([], [], '-')
            trj, = ax.plot([], [], '-')
            # Predefined control points
            control_points = [
            [6841, 326],
            [6814, 6292],
            [6681, 6398],
            [3433, 6425],
            [3167, 6478],
            [2981, 6691],


            #Finish poin
            [2981, 10000],
            #[3700, 10000],


            #Tendang Bola
            [6200, 9900],

            # [2955, 10335],
            # [2955, 10335],
            # Reverse / balik
            # [2981, 6691],
            # [3167, 6478],
            # [3433, 6425],
            # [6681, 6398],
            # [6814, 6292],
            # [6841, 5654]
            ]


            # Set the initial number of control points
            max_control_point = len(control_points)

            # Draw the control points on the plot
            for point in control_points:
                ax.scatter(*point)

            # If there are more than 3 control points, generate the B-Spline curve
            if max_control_point > 3:
                generate_knot_vector()
                for t in np.arange(0.0, 1.0, 0.001):
                    x, y = evaluate_cubic_bspline(t)
                    bspline_points.append((x, y))
                trj.set_data(zip(*bspline_points))

            if team == 1 : # biru
                background_image = np.fliplr(plt.imread(r'/home/bandhayudha/vision/TestYolo - Copy/MAP.png'))  # Load and flip the image
                ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image


        elif team == 2 : #tim merah
            ax.cla()  # Clear the axes


            ax.spines['left'].set_position('zero')
            ax.spines['bottom'].set_position('zero')
            ax.spines['right'].set_color('none')
            ax.spines['top'].set_color('none')
            ax.xaxis.set_ticks_position('bottom')
            ax.yaxis.set_ticks_position('left')


            # Clear the data structures
            points.clear()
            control_points.clear()
            bspline_points.clear()
            max_control_point = 0
            line, = ax.plot([], [], '-')
            trj, = ax.plot([], [], '-')
            # Predefined control points
            control_points = [
            [509, 326],
            [536, 6292],
            [669, 6398],
            [3917, 6425],
            [3433, 6478],
            [4369, 6691],


            #Finish poin
            [4395, 10335],
            [4395, 10335],
            [4395, 10335],
            # Reverse / balik
            # [2981, 6691],
            # [3167, 6478],
            # [3433, 6425],
            # [6681, 6398],
            # [6814, 6292],
            # [6841, 5654]
            ]

            # Set the initial number of control points
            max_control_point = len(control_points)

            # Draw the control points on the plot
            for point in control_points:
                ax.scatter(*point)

            # If there are more than 3 control points, generate the B-Spline curve
            if max_control_point > 3:
                generate_knot_vector()
                for t in np.arange(0.0, 1.0, 0.001):
                    x, y = evaluate_cubic_bspline(t)
                    bspline_points.append((x, y))
                trj.set_data(zip(*bspline_points))

            if team == 2 : # merah
                background_image = plt.imread(r'/home/bandhayudha/vision/TestYolo - Copy/MAP.png')  # Load the original image
                ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image
            canvas.draw()  # Redraw the canvas


        else :
            sg.popup("???")

# ------------------- SAVE SILO -----------------



    #Silo1
   elif event == 'Silo1':

        SiloButtonState = 'Silo1'
        
        xSilo = 0.0
        ySilo = 0.0

        SendDataStm(f'S0')

        # Send 'A1' to Arduino
        print('Data Sent : S0')

        print('Baca Data')
        BacaDataSilo()


        time.sleep(1)
        print('DRAW')
        canvas.draw()

    #Silo2
   elif event == 'Silo2':



        SiloButtonState = 'Silo2'
        
        xSilo = 0.0
        ySilo = 0.0

        SendDataStm(f'S1') #Minta Data Sick Silo1
        #ser.write(b'A2')

        # Send 'A1' to Arduino
        print('Data Sent : S1')

        BacaDataSilo()
        canvas.draw()

    #Silo1
   elif event == 'Silo3':

        SiloButtonState = 'Silo3'
        
        xSilo = 0.0
        ySilo = 0.0

        SendDataStm(f'S2')

        # Send 'A1' to Arduino
        print('Data Sent : S2')

        BacaDataSilo()
        canvas.draw()

    
    #
    # Save rectangles
   elif event == 'SaveRect':
        with open('rectangles.txt', 'w') as file:
            for key, rect in rectangles.items():
                x, y = rect.get_xy()
                file.write(f'{key} {x} {y}\n')

    # Load rectangles
   elif event == 'LoadRect':

        ax.cla()

        dot_list = []

        ax.spines['left'].set_position('zero')
        ax.spines['bottom'].set_position('zero')
        ax.spines['right'].set_color('none')
        ax.spines['top'].set_color('none')
        ax.xaxis.set_ticks_position('bottom')
        ax.yaxis.set_ticks_position('left')

        # Clear the data structures
        points.clear()
        control_points.clear()
        bspline_points.clear()
        max_control_point = 0
        line, = ax.plot([], [], '-')
        trj, = ax.plot([], [], '-')

        
        ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image

        with open('rectangles.txt', 'r') as file:
            for line in file:
                key, x, y = line.split()
                x = float(x)
                y = float(y)

                # If a rectangle for this silo already exists, remove it
                if key in rectangles:
                    rectangles[key].remove()

                # Draw a new rectangle at the loaded location with the color specified in the colors dictionary
                rectangle = plt.Rectangle((x, y), 200, 200, fill=False, edgecolor=colors[key])
                ax.add_patch(rectangle)  # Adds the rectangle to the plot

                # Store the rectangle in the dictionary
                rectangles[key] = rectangle

                # Add text on top of the rectangle
                rect_text = ax.text(x, y + 200, key, ha='center')
                
                # Store the rect_text in the dictionary
                rect_text_dict[key] = rect_text

                canvas.draw()

    # Erase rectangles
   elif event == 'EraseRect':
        # If a rectangle for this silo already exists, remove it
        for key in rectangles.keys():
            rectangles[key].remove()

        # If an rect_text for this silo already exists, remove it
        for key in rect_text_dict.keys():
            rect_text_dict[key].remove()

        # Clear the dictionaries
        rectangles.clear()
        rect_text_dict.clear()

        # Redraw the canvas
        canvas.draw()

   if event == 'SP':
       SendDataStm(f'SP')
       runningSp = '1'
       


# ------------------------------------------------




   # ON OFF
   elif event == '-ON-':
        if(serial_mode):
             SendDataStm(f'A {control_points[0]} {sudut}')
        onoff = '1'
        window['-STATUS-'].update('Status: ON')
        # ser.write(onoff.encode())
        #abc=ser.readline().decode().strip()
        print("ON event triggered")  # Debugging print statement.
        # liststatus()
        time.sleep(0.1)

        if max_control_point > 3:
            if max_control_point != last_max_control_point :
                print(f'INI LOKASI = {pos_save_max_s}')
            
                running = True
                # Create a new thread for generate_bspile()
                threading.Thread(target=generate_bspile).start()
                last_max_control_point = max_control_point
        last_max_control_point = 0



   elif event == '-OFF-':
    #    sound(r'/home/bandhayudha/Documents/GUI/Projek_bandha/error-2-36058.mp3')
       team = 3
       mode_robot = 1
       running = False
       onoff = '0'
       window['-STATUS-'].update('Status: OFF')
       #ser.write(onoff.encode())
       #abc=ser.readline().decode().strip()
       print("OFF event triggered")  # Debugging print statement.
       #liststatus()
       time.sleep(0.1)

   elif event == '-RETRY-' : #Retry Button
        pass

   # TEAM
   elif event == '-MERAH-':
       team = 2
       window['-BIRU-'].update(disabled=True)
       window['-TEAM-'].update('Team: Merah')
       #ser.write(Team.encode())
       background_image = plt.imread(r'/home/bandhayudha/vision/TestYolo - Copy/MAP.png')  # Load the original image
       ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image
       canvas.draw()
       print("MERAH event triggered")  # Debugging print statement.
       #liststatus()
       time.sleep(0.1)


   elif event == '-BIRU-':
       team = 1
       window['-MERAH-'].update(disabled=True)
       window['-TEAM-'].update('Team: Biru')
       #ser.write(Team.encode())
       background_image = np.fliplr(plt.imread(r'/home/bandhayudha/vision/TestYolo - Copy/MAP.png'))  # Load and flip the image
       ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image
       canvas.draw()  # Redraw the canvas
       print("BIRU event triggered")  # Debugging print statement.
       #liststatus()
       time.sleep(0.1)


   elif event == '-MAP-':
       ax.cla()  # Clear the axes


       ax.spines['left'].set_position('zero')
       ax.spines['bottom'].set_position('zero')
       ax.spines['right'].set_color('none')
       ax.spines['top'].set_color('none')
       ax.xaxis.set_ticks_position('bottom')
       ax.yaxis.set_ticks_position('left')


       # Clear the data structures
       points.clear()
       control_points.clear()
       bspline_points.clear()
       max_control_point = 0
       line, = ax.plot([], [], '-')
       trj, = ax.plot([], [], '-')


       ax.imshow(background_image, extent=[0, 7350, 0, 12000], alpha=1)  # Redraw the background image
       canvas.draw()  # Redraw the canvas
       os.system('cls' if os.name == 'nt' else 'clear')


   elif event == 'Submit':
       # Print the values of KP, KI, and KD when Submit is pressed
       print('----[PDI 1]----')
       SendDataStm(f'KP1 {values["-KP1-"]}\n')
       time.sleep(0.002)
       SendDataStm(f'KI1 {values["-KI1-"]}\n')
       time.sleep(0.002)
       SendDataStm(f'KD1 {values["-KD1-"]}\n')
       time.sleep(0.002)
       print('PID 1 Terkirim')
       time.sleep(0.002)

       print('----[PDI 2]----')
       SendDataStm(f'KP2 {values["-KP2-"]}\n')
       time.sleep(0.002)
       SendDataStm(f'KI2 {values["-KI2-"]}\n')
       time.sleep(0.002)
       SendDataStm(f'KD2 {values["-KD2-"]}\n')
       time.sleep(0.002)
       print('PID 2 Terkirim')
       time.sleep(0.002)



   elif event == 'Clear' and selected_key:
       window[selected_key].update('')
   #Reset
   elif event == '-RESET-':
        team = 3
        ax.cla()  # Clear the axes


        ax.spines['left'].set_position('zero')
        ax.spines['bottom'].set_position('zero')
        ax.spines['right'].set_color('none')
        ax.spines['top'].set_color('none')
        ax.xaxis.set_ticks_position('bottom')
        ax.yaxis.set_ticks_position('left')


        # Clear the data structures
        points.clear()
        control_points.clear()
        bspline_points.clear()
        max_control_point = 0
        line, = ax.plot([], [], '-')
        trj, = ax.plot([], [], '-')

        window['-MERAH-'].update(disabled=False)
        window['-BIRU-'].update(disabled=False)
        window['-TEAM-'].update('Team: None')
        #ser.write(Team.encode())
        #liststatus()
        time.sleep(0.1)

# --------------- TESTING --------------------------
   elif event == 'WL':

       
       SendDataStm('WX\n')
       pass
   
   elif event == 'WY':
       
       SendDataStm('WY\n')
       pass
   
   elif event == 'WR':
       
       SendDataStm('WR\n')
       pass

   elif event == 'Z0':
       SendDataStm('Z0\n')
       pass

# ---------------------------------------------

   #YOLO
   # Run Signal
   # Read a frame from the video
   success, frame = cap.read()
   if success:
       # Run YOLOv8 tracking on the frame, persisting tracks between frames
       results = model.track(frame, persist=True)

       for r in results :
           # Get the boxes and track IDs
           boxes = r.boxes

           # Visualize the results on the frame
           annotated_frame = results[0].plot()

           #if there's no item, continue
           if results[0].boxes is None or results[0].boxes.id is None:
               continue
           track_ids = results[0].boxes.id.int().cuda().tolist()


           # Plot the tracks
           for box, track_id in zip(boxes, track_ids):
               x, y, w, h = box.xywh[0]
               id = box.id.cuda()
               classs = int(box.cls.cuda())
               print([x, y, w, h])
            #    track.append((float(x), float(y)))  # x, y center point
            #    if len(track) > 30:  # retain 90 tracks for 90 frames
            #        track.pop(0)

               annotated_frame = cv2.line(annotated_frame, grip_point, (int(x),int(y)), (0,0,0), 5)
               jarak =  math.sqrt((int(x) - grip_point[0])**2 + (int(y) - grip_point[1])**2)
               if classs == 0:  # biru
                   biru.distances.append((jarak,track_id, (int(x), int(y))))               
                   print("KELAS = 0")

               elif classs == 2:  # merah
                   merah.distances.append((jarak,track_id, (int(x), int(y))))
                   print("KELAS = 2")
               elif classs == 1:  # ungu
                   ungu.distances.append((jarak,track_id, (int(x), int(y))))
                   print(ungu.distances)  

           if(team == 3):
               closest_point = grip_point

           if biru.distances and team == 1:
               print("------------biru")
               none_ball = 1
               biru.sorted_distances = sorted(biru.distances)
               biru.closest_point = biru.sorted_distances[0][2]
               biru.closest_distance = int(biru.sorted_distances[0][0])
               if(biru.closest_distance<grip_distance):
                   cv2.ellipse(annotated_frame,grip_point, (grip_size,grip_size), 180.0, 0.0, 180.0, (0, 0, 255), 3)
                   ballReadytoGrip = True
               else:
                   ballReadytoGrip = False
               #print(closest_distance, ballReadytoGrip)

           if merah.distances and team == 2:
               print("------------merah")
               none_ball = 1 
               merah.sorted_distances = sorted(merah.distances)
               merah.closest_point = merah.sorted_distances[0][2]
               merah.closest_distance = int(merah.sorted_distances[0][0])
               if(merah.closest_distance<grip_distance):
                   cv2.ellipse(annotated_frame,grip_point, (grip_size,grip_size), 180.0, 0.0, 180.0, (0, 0, 255), 3)
                   ballReadytoGrip = True
               else:
                   ballReadytoGrip = False
               #print(closest_distance, ballReadytoGrip)
               ###### Team 1 = biru,  team 2  =  merah
               

           if ungu.distances and team == 1:
                print("------------Abccc")
                ungu.sorted_distances = sorted(ungu.distances)
                ungu.closest_point = ungu.sorted_distances[0][2]
                ungu.closest_distance = int(ungu.sorted_distances[0][0])
                if(team == 1):
                    print("------------ungu")
                    if(ungu.closest_distance < biru.closest_distance):
                        print("------------masuk")
                        buf_x_pos = (ungu.closest_point[0] - upperWall_cutoff) - biru.closest_point[0]
                    if(buf_x_pos > biru.closest_point[0] + right_wall):
                        print("------------kanan")
                        SendDataStm(f"YR")
                        annotated_frame =  cv2.putText(annotated_frame, "YR", (50, 80), font, 1, (151,21,254), 2)        

                    elif(buf_x_pos < -1*(biru.closest_point[0] - left_wall)):
                        print("------------kiri")
                        SendDataStm(f"YL")
                        annotated_frame =  cv2.putText(annotated_frame, "YL", (50, 80), font, 1, (151,21,254), 2)        
                    else:
                        print("------------tengah")
                        SendDataStm(f"YC")
                        annotated_frame =  cv2.putText(annotated_frame, "YC", (50, 80), font, 1, (151,21,254), 2)        
#########################################    sampai siniiiii!!!!!!!!!!!!!!!!!    ##########

                # if(ungu.closest_distance<grip_distance):
                #     cv2.ellipse(annotated_frame,grip_point, (grip_size,grip_size), 180.0, 0.0, 180.0, (0, 0, 255), 3)
                #     ballReadytoGrip = True
                # else:
                #     ballReadytoGrip = False
                #print(closest_distance, ballReadytoGrip)
           if none_ball == 0: # UNGU  
                SendDataStm("YNO\n")
                print("KELAS = 1")
                
           none_ball = 0
           print(closest_point)
           ball_none = 1
           if closest_point is not None:
                ball_none = 0
                print('YNO')
                if(team != 3):
                    annotated_frame = cv2.circle(annotated_frame, tuple(closest_point),(int(w/2)), (18,241,252), 4)
                annotated_frame =  cv2.putText(annotated_frame,str(closest_point),(400, 50), font, font_scale, (151,21,254), thickness)        
                #print("x: ",int(x)," y: ", int(y), "class: ",classs,"id: ", track_id) 
                #print(mode_robot, serial_mode)
                if(mode_robot == 2 and serial_mode):
                    #print(f'{closest_point[0]} {closest_point[1]}') 
                    if(team == 1):
                        SendDataStm(f'Y {biru.closest_point[0]} {biru.closest_point[1]}\n')
                    else:
                        SendDataStm(f'Y {merah.closest_point[0]} {merah.closest_point[1]}\n')

                    #print('DATA TERKIRIM')
                #print(f'{closest_point[0]} {closest_point[1]}')
                annotated_frame =  cv2.putText(annotated_frame, tim(), (50, 50), font, 1, (151,21,254), 2)        

               # Kirim data TO STM
               #c{closest_point[0]} {closest_point[1]}\n')


               #print(f'{closest_point[0]} {closest_point[1]}')
               # annotated_frame =  cv2.putText(annotated_frame, tim(), org, font, font_scale, (151,21,254), thickness)       
                       #print dengan format "Akelas id x y"
               # print ("A%d %d %d %d" % (classs, id, x, y))
               # print("biru  :x ;", biru.sorted_cord_x,"y ;", biru.sorted_cord_y)
               # print("merah  :x ;", merah.sorted_cord_x,"y ;", merah.sorted_cord_y)
               # print("ungu  :x ;", ungu.sorted_cord_x,"y ;", ungu.sorted_cord_y)
       if ball_none:
            SendDataStm("YNO\n")
       ball_none = 1

       print(ball_none, "BAAAALL")
       biru.id.clear()
       biru.x.clear()
       biru.y.clear()
       biru.distances.clear()
       merah.id.clear()
       merah.x.clear()
       merah.y.clear()
       merah.distances.clear()

       ungu.id.clear()
       ungu.x.clear()
       ungu.y.clear()
       ungu.distances.clear()

       if cv2.waitKey(1) & 0xFF == ord("q"):
           break
      
       annotated_frame = cv2.resize(annotated_frame, (320,320))

       imgbytes = cv2.imencode('.png', annotated_frame)[1].tobytes()
       window['image'].update(data=imgbytes)
   else:
       # Break the loop if not read
       cap.release()
       run_model = False


cap.release()
# Closes all the frames
cv2.destroyAllWindows()

# Close window
window.close()
