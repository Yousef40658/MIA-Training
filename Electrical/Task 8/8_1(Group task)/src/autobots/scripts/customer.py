#!/usr/bin/env python3
import rospy
import actionlib
from autobots.srv import Order
from autobots.msg import OrderAction, OrderGoal
from std_msgs.msg import String


# ====================== Service Client ======================
def order_client(order_text):
    """Send order to confirmation service"""
    rospy.wait_for_service("confirm_order")
    try:
        take_order = rospy.ServiceProxy("confirm_order", Order)
        response = take_order(order_text)
        return response.confirmation
    except rospy.ServiceException as e:
        rospy.logerr(f"Service failed: {e}")
        return None


# ====================== Feedback Callback ======================
def feedback_callback(feedback):
    rospy.loginfo(f"Feedback: {feedback.status}")

# ====================== Orders Storage ======================
orders = []

def fill_orders(msg):
    global orders
    order_text = msg.data
    orders.append(order_text)
    rospy.loginfo(f"Received order: {order_text}")


# ====================== Main ======================
if __name__ == "__main__":
    rospy.init_node("customer")

    #Subscriber to /orders (from gui)
    sub = rospy.Subscriber("/orders" , String, fill_orders)

    clients = []
    rate = rospy.Rate(1)  # check orders every second
    while not rospy.is_shutdown():
        while orders:  # process all received orders
            order_text = orders.pop(0)
            rospy.loginfo(f"Sending order: {order_text}")

            # Step 1: Confirm order via service
            confirmation = order_client(order_text)
            if confirmation:
                rospy.loginfo(f"Service response:\n{confirmation}")

                # Step 2: Extract order ID
                try:
                    order_id = int([line for line in confirmation.split("\n") if "Order ID" in line][0].split(":")[1].strip())
                    rospy.loginfo(f"Parsed Order ID: {order_id}")

                    # Step 3: Create action client + send goal
                    client = actionlib.SimpleActionClient("execute_order", OrderAction)
                    client.wait_for_server()

                    goal = OrderGoal(order_id=order_id)
                    rospy.loginfo(f"Sending order {order_id} to kitchen...")
                    client.send_goal(goal, feedback_cb=feedback_callback)

                    # Save client so we can collect results later
                    clients.append(client)

                except Exception as e:
                    rospy.logerr(f"Could not parse order ID: {e}")

        # Wait a bit before checking for new orders
        rate.sleep()

    # Wait for all action clients to finish before exiting
    for client in clients:
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(f"Result: {result.status}")