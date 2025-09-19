import os
import time 
import random
import winsound

#-----------
#Gears
#-----------

gear_0 = [
    ['#', '#', '#', '#'],
    ['#', ' ', ' ', '#'],
    ['#', ' ', ' ', '#'],
    ['#', ' ', ' ', '#'],
    ['#', '#', '#', '#']
]

gear_1 = [
    [' ', ' ', ' ', '#'],
    [' ', ' ', ' ', '#'],
    [' ', ' ', ' ', '#'],
    [' ', ' ', ' ', '#'],
    [' ', ' ', ' ', '#']
]

gear_2 = [
    ['#', '#', '#', '#'],
    [' ', ' ', ' ', '#'],
    ['#', '#', '#', '#'],
    ['#', ' ', ' ', ' '],
    ['#', '#', '#', '#']
]

gear_3 = [
    ['#', '#', '#', '#'],
    [' ', ' ', ' ', '#'],
    ['#', '#', '#', '#'],
    [' ', ' ', ' ', '#'],
    ['#', '#', '#', '#']
]

gear_4 = [
    ['#', ' ', ' ', '#'],
    ['#', ' ', ' ', '#'],
    ['#', '#', '#', '#'],
    [' ', ' ', ' ', '#'],
    [' ', ' ', ' ', '#']
]

gear_5 = [
    ['#', '#', '#', '#'],
    ['#', ' ', ' ', ' '],
    ['#', '#', '#', '#'],
    [' ', ' ', ' ', '#'],
    ['#', '#', '#', '#']
]

gear_6 = [
    ['#', '#', '#', '#'],
    ['#', ' ', ' ', ' '],
    ['#', '#', '#', '#'],
    ['#', ' ', ' ', '#'],
    ['#', '#', '#', '#']
]

gear_7 = [
    ['#', '#', '#', '#'],
    [' ', ' ', ' ', '#'],
    [' ', ' ', '#', ' '],
    [' ', '#', ' ', ' '],
    ['#', ' ', ' ', ' ']
]

gear_8 = [
    ['#', '#', '#', '#'],
    ['#', ' ', ' ', '#'],
    ['#', '#', '#', '#'],
    ['#', ' ', ' ', '#'],
    ['#', '#', '#', '#']
]


#----------
#Gear printing 
#----------
def print_gear(gear_segment):
    for line in gear_segment :
        for element in line :
            print (element,end='')
        print("\n")

#----------
#display control
#----------------
def display_grid(gear) :
    match gear :
        case 0 : print_gear(gear_0)
        case 1 : print_gear(gear_1) 
        case 2 : print_gear(gear_2)
        case 3 : print_gear(gear_3)
        case 4 : print_gear(gear_4)
        case 5 : print_gear(gear_5)
        case 6 : print_gear(gear_6)
        case 7 : print_gear(gear_7)
        case 8 : print_gear(gear_8)

#----------
#User-Control
#----------
def user_input():
    while (1) :
        gear = input("Enter Gear (0-8): ")
        if gear.isdigit() :                                                          #integer , not char - not float
            gear = int (gear)                                                               
            if gear >= 0 and  gear <=8 :                                
                return gear                                                         #ends loop when user enters 0 < int < 9
        print ("Gear must be a digit between 0 and 8")    

#----------
# Animation
#----------
def animate_switch():
    for freq in [60, 70, 800]:
        random_gear = [[random.choice(['#', ' ']) for _ in range(4)] for _ in range(5)] #random gear for animation 
        os.system("cls")                                                                                                #clears console 
        print_gear(random_gear)
        # Play a beep sound at the current frequency for 80 milliseconds
        winsound.Beep(freq, 80)
        time.sleep(0.03)                                                                                                #short pause
    os.system("cls")
    print("Shifting...")
    time.sleep(0.15)
    
    winsound.Beep(250, 100)                                                                                      #beeb sounds
    time.sleep(0.05)
    winsound.Beep(200, 80)

#----------
#Main loop
#----------
while(1) : 
    gear = user_input() 
    animate_switch()
    os.system("cls")                      
    display_grid(gear) 

