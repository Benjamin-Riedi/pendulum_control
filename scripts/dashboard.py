import rospy
from std_srvs.srv import Trigger

class Dashboard:
    def __init__(self):
        rospy.init_node("dashboard")
        self.init_proxies()

    def init_proxies(self):
        # rospy.wait_for_service("/sensor/start", timeout=5)
        rospy.wait_for_service("/top/controller/start", timeout=5)
        rospy.wait_for_service("/bottom/controller/start", timeout=5)
        # rospy.wait_for_service("/top/state/set", timeout=5)
        # rospy.wait_for_service("/bottom/state/set", timeout=5)


        namespaces = ["/top", "/bottom"]
        self.state_proxies = [rospy.ServiceProxy(f"{ns}/state/set", Trigger) for ns in namespaces]
        self.controller_proxies = [rospy.ServiceProxy(f"{ns}/controller/start", Trigger) for ns in namespaces]
        self.sensor_proxies = [rospy.ServiceProxy(f"{ns}/sensor/start", Trigger) for ns in namespaces]
    
    def print_menu(self):
        print("|Dashboard Menu:")
        print("|----------------")
        print("|s: Start Sensor. This first calibrates the frame, the pendulum must stand on the sensor in 2 seconds.")
        print("|c: Start Controller. This starts sending motor commands based on the published angles")
        print("|h: Start Homing.")
        print("|q: Quit")
    
    def run(self):
        while not rospy.is_shutdown():
            self.print_menu()
            cmd = input("> ")

            if cmd == "s":
                for proxy in self.sensor_proxies:
                    proxy()

            elif cmd == "c":
                for proxy in self.controller_proxies:
                    proxy()
            
            elif cmd == "h":
                for proxy in self.state_proxies:
                    proxy()

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

            elif cmd == "h":
                print("Starting homing...")

            elif cmd == "q":
                break

if __name__ == "__main__":
    dashboard = Dashboard()
    dashboard.run()