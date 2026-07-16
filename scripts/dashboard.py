import rospy
from std_srvs.srv import Trigger

class Dashboard:
    def __init__(self):
        rospy.init_node("dashboard")
        self.init_proxies()

    def init_proxies(self):
        rospy.wait_for_service("/sensor/start", timeout=5)
        rospy.wait_for_service("/controller/start", timeout=5)
        # rospy.wait_for_service("/homing/start", timeout=5)

        self.start_sensor = rospy.ServiceProxy(
            "/sensor/start",
            Trigger
        )
        # self.start_homing = rospy.ServiceProxy(
        #     "/homing/start",
        #     Trigger
        # )
        self.start_controller = rospy.ServiceProxy(
            "/controller/start",
            Trigger
        )
    
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
                self.start_sensor()

            elif cmd == "c":
                self.start_controller()

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