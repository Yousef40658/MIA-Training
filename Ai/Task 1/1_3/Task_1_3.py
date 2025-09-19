import random
import pyttsx3
import time
import speech_recognition as sr
import difflib   # for fuzzy matching

Automatic = True    #Mode

#-----------------------
# Voice setup
#-----------------------n
engine = pyttsx3.init()
engine.setProperty('volume', 1.0)      # full volume
engine.setProperty('rate', 150)          #frequency of speaking
recognizer = sr.Recognizer()             # Create a Recognizer object to process and convert speech to text
mic = sr.Microphone()

def speak(text: str):
    engine.say(text)
    engine.runAndWait()

def listen(timeout=3, retries=2) -> str:
    for _ in range(retries):
        print("Listening....")
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=0.2)
            try:
                audio = recognizer.listen(source, timeout=timeout)
            except sr.WaitTimeoutError:
                print("No speech detected.")
                continue
        try:
            result = recognizer.recognize_google(audio).lower()
            print(f"[heard] {result}")
            return result
        except sr.UnknownValueError:
            print("Didn't catch that, please repeat.")
        except sr.RequestError:
            print("Speech service unavailable.")
            break
    return ""

#-----------------------
# Parent
#-----------------------
class driver:
    #------------------
    # Controlling turns
    #------------------
    driver_1_to_move = True     #control which turn
    tire_impact = 0                    #damage by attacker
    tire_defense = 0                  #defense by defender
    under_attack = 3

    #------------------
    # Driver_Attributes
    #------------------
    def __init__(self):
        self.tire_health = 100
        self.fuel = 1000        #usually results in a draw > better increased to 1000 and decrease the after-delay
        self.fuel_cost = 0  

    #------------------
    # Move_Making
    #------------------
    def choose_move(self, name, mode: str, moves_list: list):
        self.fuel_cost = 0
        valid_moves = []
        for move in moves_list:                                     # Filter valid moves based on fuel
            move()                                                           # set self.fuel_cost
            if self.fuel_cost <= self.fuel:
                valid_moves.append(move)

        if Automatic:
            if valid_moves:
                chosen_move = random.choice(valid_moves)                #randomize by default 
                chosen_move()
                print(f"{name} {mode}ed with {chosen_move.__name__} !")
                self.fuel -= self.fuel_cost
                if mode == "attack":
                    driver.tire_defense = 0
                    driver.under_attack = 1
            else:
                if mode == "attack":
                    driver.tire_impact = 0
                    driver.under_attack = 0
                else:
                    driver.tire_defense = 0
                self.fuel_cost = 0
                print(f"{name} is unable to {mode}!")
            return valid_moves, chosen_move if valid_moves else None

        # Manual
        opts = [m.__name__ for m in valid_moves]
        prompt = f"{name}, choose your {mode}: " + (", ".join(o.replace("_", " ") for o in opts) if opts else "none")
        print(prompt)

        if not valid_moves:
            if mode == "attack":
                driver.tire_impact = 0
                driver.under_attack = 0
            else:
                driver.tire_defense = 0
            self.fuel_cost = 0
            return valid_moves, None

        choice = listen()
        picked = None

        for m in valid_moves:
            if m.__name__.lower() in choice:
                picked = m
                break

        if not picked:
            close = difflib.get_close_matches(choice, [o.lower() for o in opts], n=1, cutoff=0.4)       #to get fuzzy matchings
            if close:
                for m in valid_moves:
                    if m.__name__.lower() == close[0]:
                        picked = m
                        break

        if not picked:
            picked = random.choice(valid_moves)                                                                             #fall_back to random
        picked()
        print(f"{name} {mode}ed with {picked.__name__}!")
        self.fuel -= self.fuel_cost
        if mode == "attack":
            driver.tire_defense = 0                                                  #set to zero during attack
            driver.under_attack = 1                                                 #no need to defend when not attacked

        return valid_moves, picked
    
    #Offense
    def offense(self, name, moves_list):
        self.offensive_moves = moves_list
        valid_moves, picked = self.choose_move(name, "attack", self.offensive_moves)
        self.offensive_move = picked if picked else None #unable to attack

    #Defense
    def defense(self, name, moves_list):
        self.defensive_moves = moves_list
        valid_moves, picked = self.choose_move(name, "defend", self.defensive_moves)
        self.defensive_move = picked if picked else None #unable to defend

#-----------------------
# Driver_1
#-----------------------
class Verstappen(driver):
    ERS_limit = 3
    def __init__(self):
        super().__init__()

    #-------------
    # Offense
    #-------------
    def Verstappen_attack(self):
        self.offense("Verstappen",
                     [self.DRS_boost, self.Red_Bull_Surge, self.Precision_Turn])

    def DRS_boost(self):
        driver.tire_impact = 12
        self.fuel_cost = 45

    def Red_Bull_Surge(self):
        driver.tire_impact = 20
        self.fuel_cost = 80

    def Precision_Turn(self):
        driver.tire_impact = 8
        self.fuel_cost = 30

    #-------------
    # Defense
    #-------------
    def Verstappen_defense(self):
        if self.ERS_limit > 0:
            self.defense("Verstappen",
                         [self.Brake_Late, self.ERS_Deployment])
        else:
            self.defense("Verstappen", [self.Brake_Late])

    def Brake_Late(self):
        self.fuel_cost = 25
        driver.tire_defense = 0.3 * driver.tire_impact

    def ERS_Deployment(self):
        self.fuel_cost = 40
        driver.tire_defense = 0.5 * driver.tire_impact

#-----------------------
# Driver_2
#-----------------------
class Mostafa(driver):
    aggressive_block_limit = 2
    def __init__(self):
        super().__init__()

    #-------------
    # Offense
    #-------------
    def Mostafa_attack(self):
        self.offense("Mostafa",
                     [self.Turbo_boost, self.Mercedes_charge, self.Corner_mastery])

    def Turbo_boost(self):
        driver.tire_impact = 10
        self.fuel_cost = 50

    def Mercedes_charge(self):
        driver.tire_impact = 22
        self.fuel_cost = 90

    def Corner_mastery(self):
        driver.tire_impact = 7
        self.fuel_cost = 25

    #-------------
    # Defense
    #-------------
    def Mostafa_defense(self):
        if self.aggressive_block_limit > 0:
            self.defense("Mostafa",
                         [self.Slipstream_Cut, self.Aggressive_Block])
        else:
            self.defense("Mostafa", [self.Slipstream_Cut])

    def Slipstream_Cut(self):
        self.fuel_cost = 20
        driver.tire_defense = 0.4 * driver.tire_impact

    def Aggressive_Block(self):
        self.fuel_cost = 35
        driver.tire_defense = driver.tire_impact

#-----------------------
# Main
#-----------------------
def run_game(gui=None, mode=None):
    global Automatic, verstappen_, mostafa_
    
    verstappen_ = Verstappen()
    mostafa_ = Mostafa()
    turn = 1
    min_cost = 25                                                       #no further moves can be done by both
    
    if mode:
        Automatic = (mode == "Automatic")
    else:
        while True:
            response = input("Do you want the game to run automatically Y/N : ")
            if response.lower() == 'y':
                Automatic = True
                break
            elif response.lower() == 'n':
                Automatic = False
                break
            print("Only Y/N are valid responses")                                           #input_handling
    
    # Start match
    while (mostafa_.tire_health > 0 and verstappen_.tire_health > 0
           and (mostafa_.fuel >= min_cost or verstappen_.fuel >= min_cost)):
        line = f"----------TURN {turn}----------"
        print(line)
        if gui:
            gui["turn_var"].set(f"Turn: {turn}")
            gui["log_text"].insert("end", line + "\n")
            gui["log_text"].see("end")
        #----------------------
        #Vers turn
        #----------------------
        if driver.driver_1_to_move:
            verstappen_.Verstappen_attack()
            if driver.under_attack == 0:
                msg = "Mostafa doesn't have to defend"
                print(msg)
                if gui:
                    gui["log_text"].insert("end", msg + "\n")
                    gui["log_text"].see("end")
                driver.driver_1_to_move = not driver.driver_1_to_move
                turn += 1
                continue
            mostafa_.Mostafa_defense()
            if mostafa_.defensive_move and mostafa_.defensive_move.__name__ == 'Aggressive_Block':  #move_limiter
                mostafa_.aggressive_block_limit -= 1
                
            damage = max(0, driver.tire_impact - driver.tire_defense)     #in cases of raw values damage might be negative
            msg = f'{damage:.2f} = {driver.tire_impact:.2f} - {driver.tire_defense:.2f}'
            mostafa_.tire_health -= damage
        else:
         #----------------------
        #Mostafa _turn
        #----------------------
            mostafa_.Mostafa_attack()
            if driver.under_attack == 0:
                msg = "Verstappen doesn't have to defend"
                print(msg)
                if gui:
                    gui["log_text"].insert("end", msg + "\n")
                    gui["log_text"].see("end")
                driver.driver_1_to_move = not driver.driver_1_to_move
                turn += 1
                continue

            verstappen_.Verstappen_defense()
            if verstappen_.defensive_move and verstappen_.defensive_move.__name__ == 'ERS_Deployment':
                verstappen_.ERS_limit -= 1
            damage = max(0, driver.tire_impact - driver.tire_defense)
            msg = f'{damage:.2f} = {driver.tire_impact:.2f} - {driver.tire_defense:.2f}'
            verstappen_.tire_health -= damage

        print(msg)
        if gui:
            gui["log_text"].insert("end", msg + "\n")
            gui["log_text"].see("end")

        # Status update
        stats = [
            f"Mostafa health : {mostafa_.tire_health:.2f} , Fuel : {mostafa_.fuel}",
            f"Verstappen health : {verstappen_.tire_health:.2f} , Fuel : {verstappen_.fuel}"
        ]
        for line in stats:
            print(line)
            if gui:
                gui["log_text"].insert("end", line + "\n")
                gui["log_text"].see("end")

        # GUI labels
        if gui:
            gui["mostafa_health"].set(f"Health: {mostafa_.tire_health:.2f}")
            gui["mostafa_fuel"].set(f"Fuel: {mostafa_.fuel}")
            gui["verstappen_health"].set(f"Health: {verstappen_.tire_health:.2f}")
            gui["verstappen_fuel"].set(f"Fuel: {verstappen_.fuel}")

        driver.driver_1_to_move = not driver.driver_1_to_move           #swapping turn
        turn += 1
        driver.under_attack = 1                                                         #resetting under attack to default

        if Automatic:
            time.sleep(0.3)         #delay 
    
    #-------------------
    # END_OF_MATCH
    #-------------------
    winner = ""
    if (mostafa_.fuel < min_cost and verstappen_.fuel < min_cost and
            mostafa_.tire_health > 0 and verstappen_.tire_health > 0):
        if mostafa_.tire_health > verstappen_.tire_health:
            winner = "Mostafa"
        else:
            winner = "Verstappen"
        result = f"{winner} was close but the match ended in a Draw !!"                     #rules can be altered
    elif mostafa_.tire_health <= 0:
        result = "Verstappen WON!"
        speak(result)
    else:
        result = "Mostafa WON !!"
        speak(result)

    print(result)
    if gui:
        gui["log_text"].insert("end", result + "\n")
        gui["log_text"].see("end")


if __name__ == "__main__":                                              #to run terminal version when run here
    run_game()
