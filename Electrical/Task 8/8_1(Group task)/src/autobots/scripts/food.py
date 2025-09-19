import ast
import os   

# ---------- File Reader ----------
def read_file(file_name):
    # Get directory of the current script
    base_dir = os.path.dirname(__file__)  
    
    # Join directory with the file name
    file_path = os.path.join(base_dir, f"{file_name}.txt")  
    
    with open(file_path, "r") as f:
        content = f.read()
    return ast.literal_eval(content)

# ---------- Item Class ----------
class Item:
    SIZE_PRICE = {"S": 0.8 , "M": 1, "L": 1.3}
    SIZE_TIME = {"S": 1.1, "M": 1, "L": 1.05}

    #Methods
    def __init__(self, name, size, price=10.0, cooking_time=10.0, description="Custom_food", extras=[]):
        self.name = name
        self.size = size
        self.price = price
        self.cooking_time = cooking_time
        self.description = description
        self.extras = extras

    def get_total_price(self):
        total = (self.price * Item.SIZE_PRICE.get(self.size, 0))
        total += len(self.extras) * 0.5  # $0.5 per extra
        return round(total , 1)

    def get_total_cooking_time(self):
        total = (self.cooking_time * Item.SIZE_TIME.get(self.size, 0))
        total += len(self.extras) * 0.5  # 0.2 min per extra
        return round(total , 1)
    
    #Display
    def __repr__(self):
         return self.__str__()
    
    def __str__(self):
        extras_str = ", ".join(self.extras) if self.extras else "No extras"
        return f"{self.name} ({self.size}) - ${self.get_total_price()} | {self.get_total_cooking_time()} mins | {self.description} | Extras: {extras_str}"

# ---------- Category Class  ----------
class Category():
    def __init__(self, file_name, default_size):
        self.items = []
        data = read_file(file_name)
        for name, (price, time, desc) in data.items():
            item = Item(name, default_size, price, time, desc)
            self.items.append(item)

# ---------- Specific Categories ----------
class Main(Category):
    def __init__(self , size = 'M'):
        super().__init__("mains", f"{size}")

class Snacks(Category):
    def __init__(self,size = 'M'):
        super().__init__("snacks", f"{size}")

class Drinks(Category):
    def __init__(self,size ='M'):
        super().__init__("drinks", f"{size}")

class Dessert(Category):
    def __init__(self,size = 'M'):
        super().__init__("desserts", f"{size}")

# ---------- Example Usage ----------
mains = Main()
snacks = Snacks()
drinks = Drinks()
desserts = Dessert()

mains_list = mains.items
snacks_list = snacks.items
drinks_list = drinks.items
desserts_list = desserts.items
    
Menu = [mains_list , snacks_list , drinks_list,desserts_list]
    
if __name__ == "__main__":
     print (Menu)
