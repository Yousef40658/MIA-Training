import os
import ast

menu_data = {}
file_names = ["mains", "drinks", "desserts", "snacks"]

# Get the directory of the current script
directory = os.path.dirname(os.path.abspath(__file__))

for file_name in file_names:
    file_path = os.path.join(directory, f"{file_name}.txt")
    with open(file_path, "r") as f:
        content = f.read()
    menu_data[file_name] = ast.literal_eval(content)



# print("Menu data content:", menu_data)
# print("Type:", type(menu_data))
# print("Keys:", list(menu_data.keys()) if menu_data else "Empty!")
# menu_data={
#     "mains": ["Burger", "Pizza", "Pasta"],
#     "drinks": ["Coke", "Water", "Juice"],
#     "desserts": ["Ice Cream", "Cake", "Pie"],
#     "snacks": ["Chips", "Nuts", "Popcorn"]
# }


# menu_data={'drinks': {'Soda': (1.99, 2, 'Refreshing carbonated drink available in multiple flavors'),
#              'Milkshake': (3.99, 5, 'Creamy milkshake with vanilla, chocolate, or strawberry flavors'), 
#              'Energon Drink': (5.5, 3, 'Special energizing drink inspired by Transformers')},

#   'desserts': {'Ice Cream': (2.99, 10, 'Classic vanilla ice cream with various toppings'),
#                'Cake': (4.99, 5, 'Rich chocolate cake with creamy frosting'),
#                'Pie': (3.49, 8, 'Homemade apple pie with a flaky crust')}
# }
# menu_data['drinks']['Soda']


# for category_name, items_dict in menu_data.items():
#     print(f"Category: {category_name}")
#     print("-" * 30)
    
#     # Loop through items in this category
#     for item_name, item_data in items_dict.items():
#         price, quantity, description = item_data
#         print(f"  {item_name}: ${price} (Qty: {quantity})")
#         print(f"    Description: {description}")
#     print()

# simple_menu = {}
# for category_name, items_dict in menu_data.items():
#     simple_menu[category_name] = list(items_dict.keys()) 

# print(simple_menu)