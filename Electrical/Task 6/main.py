import tkinter as tk 
from tkinter import ttk , messagebox
from game_logic import *
#-------------
#Default values 
#-------------
selected = 5 
prev = 5
grid_frame = None
cells = []
selected_cell = [0,0]
prev_cell = [0,0]
input_letters = []
wordle = None
style = ttk.Style()
locked_cells = set()

#Theme & Helpers

#Highlighting
def set_cell_color(cell,color,lock=False) :
    cell.config(bg = color)
    if lock :
        for r,row_cells in enumerate(cells):
            for c , entry in enumerate(row_cells):
                if entry == cell    :#store locked cells so editing can be blocked letter
                    locked_cells.add((r,c)) #add new cell in a tuble 
                    return
                
#selecting cell                
def select_cell(row,col) :
    global selected_cell,prev_cell
    try :
        pR,pC = prev_cell       #removing highlight of prev cell
        if (pR,pC) not in locked_cells :
            cells[pR][pC].config(bg="white")
        
    except :
        pass
    
    selected_cell = [row , col]
    prev_cell = [row,col] # updating for next call

    if(row, col) not in locked_cells :
        try :
             cells[row][col].config(bg="yellow")
        except Exception :
            pass
    
    try :
        cells[row][col].focus_set()     #focusing on new cell
    except :
        pass

##writing by keyboard
def key_press(event) :
    global selected_cell,input_letters,wordle
    
    if wordle is None :
        return "break"
    
    r,c = selected_cell
    if r < 0 or r>= len(cells) or c<0 or c>= wordle.word_length :   #bounds checking
        return "break"
    
    #backspace
    if event.keysym == "BackSpace" :
        if (r,c) in locked_cells :
            return "break"

        current_val = cells[r][c].get()
        if current_val != "" :                       #backspacing within non empty cell
            cells[r][c].config(state = 'normal')     #for_safety -> setting it writable
            cells[r][c].delete(0,tk.END)             #delete the cell content
            input_letters[r][c] = ""
            select_cell(r,c)                         #to stay in same cell
        
        else :                                       #empty_box
            if c > 0 :
                next_c = c -1 
                if (r,next_c) in locked_cells :
                    select_cell(r,c) 
                    return "break"
                
                cells[r][next_c].config(state='normal')    #prev cell
                cells[r][next_c].delete(0,tk.END)
                input_letters[r][next_c] = ""
                select_cell(r,next_c)
        return "break"
    
    elif event.keysym == "Return"   :                   #enter button
        sumbit_guess()
        return
    
    elif len(event.char) == 1 and event.char.isalpha() :  # a single charcater
        if (r,c) in locked_cells :
            found = False
            for new_char in range (c+1,wordle.word_length):
                if (r,new_char) not in locked_cells :
                    select_cell(r,new_char)             #select the next cell when you write in a filled cell    
                    r, c = selected_cell
                    found = True                        #
                    break
            if not found :
                return "break"
            
            letter = event.char.upper() #uppercasing
            cells[r][c].config(state  = "normal")
            cells[r][c].delete(0,tk.END)
            cells[r][c].insert(0,letter)    #inster new letter
            input_letters[r][c] = letter    #store it in grid

            next_c = c + 1 
            while next_c < wordle.word_length and (r,next_c) in locked_cells :  #still in grid and next cell is locked
                next_c += 1 
            if next_c < wordle.word_length :
                select_cell(r,next_c)
            return "break"
        else :
            letter = event.char.upper()
            cells[r][c].config(state = "normal")
            cells[r][c].delete(0,tk.END)
            cells[r][c].insert(0,letter)
            input_letters[r][c] = letter

            next_c = c+1
            while next_c <wordle.word_length and (r,next_c) in locked_cells:
                next_c += 1
            if next_c < wordle.word_length :
                select_cell(r,next_c)
            return "break"
    return 
    

#-------------
#System
#-------------
def centering (window,width,height) :
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    x_cord = int((screen_width / 2) - (width / 2))   #coordinate to center 
    y_cord = int((screen_height / 2) - (height / 2))

    window.geometry(f"{width}x{height}+{x_cord}+{y_cord}") #opens in center

def build_grid(window,rebuild_flag):
    global grid_frame, cells , selected , wordle,locked_cells,input_letters
    locked_cells = set()
    #grid
    wordle = Wordle(word_length=selected)
    n_rows = 6
    n_cols = selected if rebuild_flag else 5 #= 5 when rebuilding other wise 5 by default

    selected = n_cols

    input_letters = [[""] * n_cols for _ in range (n_rows)] #6 2d list initialized to apply values 
    cells = []
    if rebuild_flag and grid_frame :
            grid_frame.destroy()#destory current frame

    #frame to hold the grid
    grid_frame = tk.Frame(window)
    grid_frame.pack(expand=True,fill='both',pady=30,padx= 20) #fills the window in the y direction
    
    #allowing grid cells to expand
    for i in range (n_rows) :
        grid_frame.rowconfigure(i,weight=1)
    for i in range(n_cols) :
        grid_frame.columnconfigure(i,weight=1)


    for row in range(n_rows) :
        row_cells = []
        for col in range (n_cols) :
            entry = tk.Entry(\
                grid_frame, #the window the labels set on
                text = "", #for letters entered
                width= 16,
                font= ("BROPELLA" , 18, "bold", "italic"), #black by default
                relief= "solid",    #a button look
                borderwidth= 0.5, 
                bg= "white",
                fg= "black",
                justify= "center"
            )
            entry.grid(row=row,column = col,sticky='nsew', padx= 7 , pady=10, ipady=10 , ipadx=10)
            entry.bind("<Button-1>", lambda e, r=row, c=col: select_cell(r, c))
            row_cells.append(entry)                         #appending the row to the grid
        cells.append(row_cells)
    select_cell(0,0) #initially select first cell


def setings_button () :
    #window
    settings_window = tk.Toplevel(wordle_gui) #at the top of our worlde window
    settings_window.title ("Settings") 
    settings_window.geometry("400x150")
    settings_window.configure(bg="#121213")
    settings_window.resizable(True, True)


    #word_lenggth_slider
    tk.Label(settings_window, text="Select word length" , font= ("Arial",12)).pack(pady= 10 , padx= 5)
    slider_frame = tk.Frame(settings_window) #on settings window
    slider_frame.pack()
    word_length = tk.IntVar(settings_window ,value=selected) #default is 5 
    slider = tk.Scale(settings_window,from_=3,to=6
                    ,orient="horizontal",variable=word_length,
                    showvalue= False, length= 250) #slides horizontally from 3 to 6
    slider.pack(side=tk.LEFT)

    floating_slider_value = tk.Label(slider_frame, text= 
                                     str(word_length.get()), #converts sliders current value to string
                                     font=("Arial", 12)) 
    floating_slider_value.pack(side=tk.LEFT,padx=20)

    def on_slider_change(val) :
        floating_slider_value.config(text=val) #updates
    
    slider.config(command = on_slider_change , bg="#121213") #whenever the slider is touched the function is activated

    #save button 
    def save_word_length() :
        global selected_cell,prev,selected
        selected = word_length.get() 
        if selected != prev  :      #reset only if the size was changed
            prev = selected
            settings_window.destroy() #shuts down settings window after saving
            build_grid(wordle_gui,True) #rebuilds
        else :
            settings_window.destroy()
    
    tk.Button(settings_window, text="Save" , command= save_word_length).pack(pady=10) 

def sumbit_guess():
    global wordle , locked_cells
    if wordle is None :
        return

    current_row = wordle.max_attempts - wordle.attempts_left #every row consume an attempt

    if current_row >= wordle.max_attempts :
        return

    guess = "".join(input_letters[current_row]).lower().strip() #breaks the row input into string

    if len(guess) < wordle.word_length :
        messagebox.showinfo("Not enought letters" ,"not enough letters for this gues")
        return

    result = wordle.evaluate_guess(guess)
    #print(result)

    if isinstance(result, str): #when the word isn't in the txt
        messagebox.showinfo("Invalid word" , f'{result} not in text')
        return

    for col,ev in enumerate(result) :
        cells[current_row][col].config(state="normal")
        cells[current_row][col].config(readonlybackground = ev.value)
        cells[current_row][col].config(state = "readonly")
        locked_cells.add((current_row,col))
    
    if wordle.Won :
        messagebox.showinfo("You won" , f"word is {wordle.selected_word}")
        return
    
    if wordle.game_over : 
        messagebox.showinfo("You lost" , f"word is {wordle.selected_word}")
        return
    
    next_row = current_row + 1
    select_cell (next_row,0)
#---------------
#main window
#---------------
wordle_gui = tk.Tk()
wordle_gui.title ("Wordle")
worlde_width = 1200
world_height = 1600 

#centering
centering(wordle_gui,worlde_width,world_height)

#settings 
settings_btn = tk.Button(wordle_gui,text="⚙ Settings" ,
                        font=("Arial",12),command=setings_button,
                        bg="#538d43", fg="white" , relief="flat")
settings_btn.pack(pady=10) 

#building
build_grid(wordle_gui,False)

#Theme
wordle_gui.configure(bg="#121213")  # Dark gray background


#input 
input_frame = tk.Frame(wordle_gui,bg="#121213")
input_frame.pack(pady=20)

guess_var = tk.StringVar(wordle_gui)    #initalize guess as tk string variable
guess_entry = tk.Entry(input_frame,textvariable=guess_var)
guess_entry.pack(side=tk.RIGHT , padx= 25)

#sumbit button 
sumbit_btn = tk.Button(input_frame, text= "sumbit" , font= ("Arial" , 18) , bg = "#1B9602" , command= sumbit_guess)
guess_entry.pack(side=tk.RIGHT , padx= 25)

wordle_gui.bind("<Key>",key_press)  #bind keys in this window with functin
wordle_gui.focus_force()            #forces input to this window

wordle_gui.mainloop()










wordle_gui.mainloop()

