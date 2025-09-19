#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import customtkinter as ctk
from readfiles import menu_data   # your menu data file
from food import Item 

# ================== ROS INIT ==================
rospy.init_node("order_gui", anonymous=True)
pub = rospy.Publisher("/orders", String, queue_size=10)

# ================== CUSTOMTKINTER INIT ==================
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

root = ctk.CTk()
root.title("🍽️ Restaurant Menu")
root.geometry("1000x600")

# ================== POPUP ==================
def show_item_popup(item_name, category_name):
    """Show popup with item details, size, and extras"""

    base_price, cooking_time, description = menu_data[category_name][item_name]

    # Popup window
    popup = ctk.CTkToplevel(root)
    popup.title(f"Order {item_name}")
    popup.geometry("480x480")

    # Title
    ctk.CTkLabel(
        popup, text=item_name, font=ctk.CTkFont(size=20, weight="bold")
    ).pack(pady=10)

    # --- PRICE LABEL ---
    price_var = ctk.StringVar(value=f"Price: ${base_price:.2f}")
    price_label = ctk.CTkLabel(popup, textvariable=price_var)
    price_label.pack(pady=3)

    ctk.CTkLabel(popup, text=description, wraplength=350).pack(pady=10)

    # ----- SIZE SELECTION -----
    size_var = ctk.StringVar(value="M")
    size_frame = ctk.CTkFrame(popup)
    size_frame.pack(pady=5)
    ctk.CTkLabel(size_frame, text="Size:").pack(side="left", padx=5)
    for s in ["S", "M", "L"]:
        ctk.CTkRadioButton(size_frame, text=s, variable=size_var, value=s,
                           command=lambda: update_price()).pack(side="left", padx=5)

    # ----- EXTRAS -----
    extras = ["cheese", "sauce", "white_sauce", "fries", "tomato"]
    extra_vars = {}
    extras_frame = ctk.CTkFrame(popup)
    extras_frame.pack(pady=10)
    ctk.CTkLabel(extras_frame, text="Extras:").pack(anchor="w")
    for e in extras:
        var = ctk.BooleanVar()
        chk = ctk.CTkCheckBox(extras_frame, text=e, variable=var,
                              command=lambda: update_price())
        chk.pack(anchor="w")
        extra_vars[e] = var

    # ----- PRICE UPDATE FUNCTION -----
    def update_price():
        size = size_var.get()
        chosen_extras = [e for e, v in extra_vars.items() if v.get()]

        # create a temp Item and use its get_total_price
        temp_item = Item(item_name, size, price=base_price,
                         cooking_time=cooking_time,
                         description=description,
                         extras=chosen_extras)

        total = temp_item.get_total_price()
        price_var.set(f"Price: ${total:.2f}")

    # ----- CONFIRM -----
    def add_order():
        size = size_var.get()
        chosen_extras = [e for e, v in extra_vars.items() if v.get()]
        order_str = f"{size} {item_name} {' '.join(chosen_extras)}".strip()

        rospy.loginfo(f"Publishing order: {order_str}")
        pub.publish(order_str)
        popup.destroy()

    ctk.CTkButton(popup, text="Add to Order", command=add_order,
                  fg_color="#1DB954").pack(pady=15)

    # run once at start
    update_price()

# ================== MENU BUILD ==================
# Extract item names
simple_menu = {cat: list(items.keys()) for cat, items in menu_data.items()}

# Main Frame
main_frame = ctk.CTkFrame(root, fg_color="transparent")
main_frame.pack(fill="both", expand=True, padx=20, pady=20)

# Title
title_label = ctk.CTkLabel(
    main_frame, text="RESTAURANT MENU", font=ctk.CTkFont(size=28, weight="bold")
)
title_label.pack(pady=20)

# Categories Frame
categories_frame = ctk.CTkFrame(main_frame, fg_color="transparent")
categories_frame.pack(fill="both", expand=True, pady=20)

# Build category sections with buttons
for category_name, items in simple_menu.items():
    category_frame = ctk.CTkFrame(categories_frame, corner_radius=15)
    category_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    category_label = ctk.CTkLabel(
        category_frame,
        text=category_name.upper(),
        font=ctk.CTkFont(size=16, weight="bold"),
    )
    category_label.pack(pady=10)

    for item in items:
        item_btn = ctk.CTkButton(
            category_frame,
            text=item,
            command=lambda i=item, c=category_name: show_item_popup(i, c),
            corner_radius=10,
            height=40,
            font=ctk.CTkFont(size=14),
        )
        item_btn.pack(fill="x", pady=5, padx=10)

# ================== INPUT FRAME (BOTTOM) ==================
input_frame = ctk.CTkFrame(root, height=80)
input_frame.pack(fill="x", padx=20, pady=10)

# --- SIZE ---
quick_size_var = ctk.StringVar(value="M")
size_frame = ctk.CTkFrame(input_frame)
size_frame.pack(side="left", padx=10, pady=10)
ctk.CTkLabel(size_frame, text="Size:").pack(side="left", padx=5)
for s in ["S", "M", "L"]:
    ctk.CTkRadioButton(size_frame, text=s, variable=quick_size_var, value=s).pack(side="left")

# --- ITEM NAME ---
entry1 = ctk.CTkEntry(input_frame, placeholder_text="Item name...", width=200)
entry1.pack(side="left", padx=10, pady=10)

# --- EXTRAS ---
entry2 = ctk.CTkEntry(input_frame, placeholder_text="Extras (space separated)...", width=250)
entry2.pack(side="left", padx=10, pady=10)

# --- SUBMIT ---
def quick_order():
    size = quick_size_var.get()
    item_name = entry1.get().strip()
    extras_str = entry2.get().strip()
    order_str = f"{size} {item_name} {extras_str}".strip()

    rospy.loginfo(f"Publishing quick order: {order_str}")
    pub.publish(order_str)

submit_btn = ctk.CTkButton(
    input_frame,
    text="Order",
    command=quick_order,
    fg_color="#FF6B6B",
    hover_color="#FF5252",
)
submit_btn.pack(side="left", padx=10, pady=10)

# ================== RUN ==================
root.mainloop()
