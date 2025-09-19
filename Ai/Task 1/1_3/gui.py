#Styling was done by AI
import tkinter as tk
from tkinter import ttk
import threading
from playsound import playsound
import os
from Task_1_3 import run_game

# --- Main Window Setup ---
root = tk.Tk()
root.title("🏎️ F1 Battle - Verstappen vs Mostafa")
root.geometry("1000x750")
root.resizable(False, False)

# --- Dark Mode Colors ---
BG_DARK = "#1e1e1e"
TEXT_LIGHT = "#ffffff"
TEXT_GRAY = "#d0d0d0"
RED = "#e06666"
BLUE = "#6fa8dc"
BOX_DARK = "#2a2a2a"

root.configure(bg=BG_DARK)
style = ttk.Style()
style.theme_use("default")

# --- Style Config ---
style.configure("TLabel", background=BG_DARK, foreground=TEXT_LIGHT, font=("Helvetica", 14))
style.configure("TButton", background=BOX_DARK, foreground=TEXT_LIGHT, font=("Helvetica", 16))
style.configure("TFrame", background=BG_DARK)

# --- Title ---
title_label = ttk.Label(root, text="🏁 F1 Battle Arena", font=("Helvetica", 24, "bold"), foreground=TEXT_GRAY)
title_label.pack(pady=10)

# --- Mode Selection ---
mode_frame = ttk.Frame(root)
mode_frame.pack(pady=10)

selected_mode = tk.StringVar(value="")

def start_game_mode(mode):
    selected_mode.set(mode)
    manual_btn["state"] = "disabled"
    auto_btn["state"] = "disabled"
    
    threading.Thread(
        target=run_game,
        kwargs={
            "gui": {
                "turn_var": turn_var,
                "log_text": log_text,
                "mostafa_health": mostafa_health,
                "mostafa_fuel": mostafa_fuel,
                "verstappen_health": verstappen_health,
                "verstappen_fuel": verstappen_fuel,
                "selected_mode": mode
            },
            "mode": mode
        },
        daemon=True
    ).start()

ttk.Label(mode_frame, text="🎮 Choose Game Mode:").grid(row=0, column=0, padx=5)
manual_btn = ttk.Button(mode_frame, text="🎙️ Manual", command=lambda: start_game_mode("Manual"))
manual_btn.grid(row=0, column=1, padx=5)
auto_btn = ttk.Button(mode_frame, text="⚙️ Automatic", command=lambda: start_game_mode("Automatic"))
auto_btn.grid(row=0, column=2, padx=5)

# --- Turn Label ---
turn_var = tk.StringVar(value="Turn: 1")
turn_label = ttk.Label(root, textvariable=turn_var, font=("Helvetica", 16, "bold"), foreground=TEXT_GRAY)
turn_label.pack(pady=15)

# --- Status Frame ---
status_frame = ttk.Frame(root)
status_frame.pack(pady=15)

# Verstappen Stats
verstappen_health = tk.StringVar(value="Health: 100")
verstappen_fuel = tk.StringVar(value="Fuel: 1000")
ttk.Label(status_frame, text="Verstappen", font=("Helvetica", 17, "bold"), foreground=BLUE).grid(row=0, column=0, padx=40)
ttk.Label(status_frame, textvariable=verstappen_health, font=("Helvetica", 17)).grid(row=1, column=0)
ttk.Label(status_frame, textvariable=verstappen_fuel, font=("Helvetica", 17)).grid(row=2, column=0)

# Mostafa Stats
mostafa_health = tk.StringVar(value="Health: 100")
mostafa_fuel = tk.StringVar(value="Fuel: 1000")
ttk.Label(status_frame, text="Mostafa", font=("Helvetica", 17 ,"bold"), foreground=RED).grid(row=0, column=1, padx=40)
ttk.Label(status_frame, textvariable=mostafa_health, font=("Helvetica", 17)).grid(row=1, column=1)
ttk.Label(status_frame, textvariable=mostafa_fuel, font=("Helvetica", 17)).grid(row=2, column=1)

# --- Log Output Box ---
log_frame = ttk.Frame(root)
log_frame.pack(fill='both', expand=True, padx=17, pady=10)

log_text = tk.Text(
    log_frame, height=17, wrap='word',
    font=("Courier", 15), background=BOX_DARK,
    foreground=TEXT_LIGHT, insertbackground=TEXT_LIGHT
)
log_text.pack(side='left', fill='both', expand=True)

scrollbar = ttk.Scrollbar(log_frame, command=log_text.yview)
scrollbar.pack(side='right', fill='y')
log_text.config(yscrollcommand=scrollbar.set)

# --- Run GUI ---
root.mainloop()
