#!/usr/bin/env python3
import rospy
import time
import threading
import queue
import actionlib
from colorama import Fore, Style, init   # 🎨 for colorful logs

from food import *
from autobots.srv import Order, OrderResponse
from autobots.msg import OrderAction, OrderGoal, OrderFeedback, OrderResult

# Initialize colorama
init(autoreset=True)


# ====================== Restaurant ======================
class Restaurant:
    def __init__(self, menu=Menu):
        self.menu = menu
        self.order_queue = []
        self.next_order_id = 1  # auto-increment order IDs

    def find_item(self, item_name, size="M", extras=[]):
        """Find item in menu by name"""
        for category in Menu:
            for item in category:
                if item.name.lower() == item_name.lower():
                    return Item(item.name, size, item.price, item.cooking_time,
                                item.description, extras)
        return None

    def add_order(self, order_data):
        """Save new order and assign an ID"""
        order_data["id"] = self.next_order_id
        self.order_queue.append(order_data)
        self.next_order_id += 1
        return order_data["id"]


restaurant = Restaurant()


# ====================== SERVICE (Confirm Order) ======================
def confirm_order(req):
    order_parts = req.order.split()

    if len(order_parts) < 2:
        return OrderResponse(f"{Fore.RED}❌ Invalid order. Use: <Size> <Item> [Extras...]")

    size = order_parts[0].upper()
    item_name = order_parts[1].capitalize()
    extras = [extra.replace("_", " ").capitalize() for extra in order_parts[2:]]

    # Try to find the item
    food_item = restaurant.find_item(item_name, size, extras)

    if not food_item:
        rospy.logwarn(f"{Fore.YELLOW}⚠️  Item {item_name} not found in menu. Assigning default values.")
        food_item = Item(
            name=item_name,
            size=size,
            description="Custom order",
            extras=extras
        )

    price = food_item.get_total_price()
    time_needed = food_item.get_total_cooking_time()

    # Save order and assign ID
    order_id = restaurant.add_order({
        "item": food_item,
        "time": time_needed,
        "price": price,
        "extras": extras
    })

    confirmation_msg = (
        f"\n{Fore.CYAN}🧾 Order Confirmed!\n"
        f"{Fore.WHITE}-----------------------------\n"
        f"{Fore.GREEN}✅ Item: {size} {item_name}\n"
        f"{Fore.MAGENTA}➕ Extras: {extras or 'No extras'}\n"
        f"{Fore.YELLOW}⏱️  Time: {time_needed} mins\n"
        f"{Fore.BLUE}💰 Price: {price}$\n"
        f"{Fore.CYAN}📌 Order ID: {order_id}\n"
        f"{Fore.WHITE}-----------------------------\n"
    )

    return OrderResponse(confirmation_msg)


# ====================== ACTION SERVER with Chef Limit + Queue ======================
class OrderActionServer:
    def __init__(self, restaurant, max_chefs=3):
        self.restaurant = restaurant
        self.max_chefs = max_chefs
        self.active_chefs = 0
        self.lock = threading.Lock()
        self.waiting_queue = queue.Queue()

        self.server = actionlib.ActionServer(
            "execute_order",
            OrderAction,
            goal_cb=self.goal_callback,
            cancel_cb=self.cancel_callback,
            auto_start=False
        )
        self.server.start()

    def goal_callback(self, goal_handle):
        """New order received"""
        with self.lock:
            if self.active_chefs < self.max_chefs:
                # Start cooking immediately
                self.active_chefs += 1
                rospy.loginfo(f"{Fore.GREEN}👨‍🍳 Accepting order immediately. Active chefs = {self.active_chefs}")
                goal_handle.set_accepted()
                t = threading.Thread(target=self.cook_order, args=(goal_handle,))
                t.start()
            else:
                # Put in waiting queue
                rospy.loginfo(f"{Fore.YELLOW}⌛ All chefs busy. Queuing order.")
                self.waiting_queue.put(goal_handle)

    def cancel_callback(self, goal_handle):
        """Handle cancellations"""
        rospy.loginfo(f"{Fore.RED}❌ Order cancelled by client")
        goal_handle.set_canceled()

    def cook_order(self, goal_handle):
        """Cook the order (runs in a thread)"""
        goal = goal_handle.get_goal()
        order_id = goal.order_id

        # Find order by ID
        order = next((o for o in self.restaurant.order_queue if o["id"] == order_id), None)
        if not order:
            result = OrderResult(status="No such order")
            goal_handle.set_aborted(result)
            self.free_chef()
            return

        self.restaurant.order_queue.remove(order)
        item = order["item"]
        cook_time = order["time"]

        feedback = OrderFeedback()
        result = OrderResult()

        feedback.status = f"{Fore.CYAN}Order {order_id}: Received. Starting cooking..."
        goal_handle.publish_feedback(feedback)
        rospy.sleep(1)

        # Simulate progress
        for progress in range(0, 101, 20):
            if goal_handle.get_goal_status().status == 2:  # Cancelled
                rospy.loginfo(f"{Fore.RED}❌ Order {order_id} cancelled during cooking")
                goal_handle.set_canceled()
                self.free_chef()
                return

            feedback.status = f"{Fore.MAGENTA}Order {order_id}: Cooking {item.name}... {progress}%"
            goal_handle.publish_feedback(feedback)
            rospy.sleep(cook_time / 5.0)

        feedback.status = f"{Fore.YELLOW}🚚 Order {order_id}: Food is on the way!"
        goal_handle.publish_feedback(feedback)
        rospy.sleep(1)

        result.status = f"{Fore.GREEN}🎉 Order {order_id}: Completed! Enjoy your {item.name}."
        goal_handle.set_succeeded(result)

        self.free_chef()

    def free_chef(self):
        """Free a chef and check waiting queue"""
        with self.lock:
            self.active_chefs -= 1
            rospy.loginfo(f"{Fore.CYAN}👨‍🍳 Chef freed. Active chefs = {self.active_chefs}")

            # Start next order if queue not empty
            if not self.waiting_queue.empty():
                next_goal = self.waiting_queue.get()
                self.active_chefs += 1
                rospy.loginfo(f"{Fore.BLUE}➡️  Dequeued new order. Active chefs = {self.active_chefs}")
                next_goal.set_accepted()
                t = threading.Thread(target=self.cook_order, args=(next_goal,))
                t.start()


# ====================== Main ======================
def main():
    rospy.init_node("restaurant")

    # Start service
    rospy.Service("confirm_order", Order, confirm_order)
    rospy.loginfo(f"{Fore.CYAN}🛎️  Order confirmation service ready")

    # Start action server with 3 chefs
    OrderActionServer(restaurant, max_chefs=3)
    rospy.loginfo(f"{Fore.GREEN}🍳 Order execution action server ready with 3 chefs + queue")

    rospy.spin()


if __name__ == "__main__":
    main()
