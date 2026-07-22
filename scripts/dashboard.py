import rospy
from std_srvs.srv import Trigger
import rosservice


class Dashboard:
    def __init__(self):
        rospy.init_node("dashboard")
        services = rosservice.get_service_list()
        print("Available services:")
        for service in services:
            print(service)
        self.init_proxies()

    def init_proxies(self):
        # rospy.wait_for_service("/sensor/start", timeout=5)
        # rospy.wait_for_service("/top/controller/start", timeout=5)
        # rospy.wait_for_service("/bottom/controller/start", timeout=5)
        # rospy.wait_for_service("/top/state/set", timeout=5)
        # rospy.wait_for_service("/bottom/state/set", timeout=5)


        namespaces = ["/top", "/bottom"]
        # namespaces = ["/bottom"]
        self.state_proxies = [rospy.ServiceProxy(f"{ns}/state/set", Trigger) for ns in namespaces]
        self.controller_proxies = [rospy.ServiceProxy(f"{ns}/controller/start", Trigger) for ns in namespaces]
        self.sensor_proxies = [rospy.ServiceProxy(f"{ns}/sensor/start", Trigger) for ns in namespaces]
    
    def print_menu(self):
        print("|Dashboard Menu:")
        print("|----------------")
        print("|p: Print available services.")
        print("|s: Start Sensor. This first calibrates the frame, the pendulum must stand on the sensor in 2 seconds.")
        print("|c: Start Controller. This starts sending motor commands based on the published angles")
        print("|i: Set Initial State.")
        print("|q: Quit")
    
    def run(self):
        while not rospy.is_shutdown():
            self.print_menu()
            
            cmd = input("> ")

            if cmd == "s":
                for proxy in self.sensor_proxies:
                    try:
                        proxy()
                        print("Sensor started successfully.")
                    except rospy.ServiceException as e:
                        print(f"Error occurred while calling sensor proxy: {e}")

            elif cmd == "c":
                for proxy in self.controller_proxies:
                    try:
                        proxy()
                        print("Controller started successfully.")
                    except rospy.ServiceException as e:
                        print(f"Error occurred while calling controller proxy: {e}")
            
            elif cmd == "i":
                for proxy in self.state_proxies:
                    try:
                        proxy()
                        print("Initial state set successfully.")
                    except rospy.ServiceException as e:
                        print(f"Error occurred while calling state proxy: {e}")
            
            elif cmd == 'p':
                services = rosservice.get_service_list()
                print("Available services:")
                for service in services:
                    print(service)

            elif cmd == "q":
                break

    def test(self):
        while not rospy.is_shutdown():
            self.print_menu()
            cmd = input("> ")

            if cmd == "s":
                print("Starting sensor...")

            elif cmd == "c":
                print("Starting controller...")

            elif cmd == "i":
                print("Set initial state...")

            elif cmd == "q":
                break

if __name__ == "__main__":
    dashboard = Dashboard()
    dashboard.run()