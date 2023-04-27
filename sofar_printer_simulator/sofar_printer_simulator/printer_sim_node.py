import arcade
import rclpy 
from threading import Thread
from rclpy.node import Node
from .lib.printer_sim import PrinterSimulation

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point

from sofar_printer_simulator_interface.srv import EndEffectorPosition

# Thread to run simulation in background
class PrinterSimThread(Thread):
   def __init__(self):
      Thread.__init__(self)
   
   def run(self):
      arcade.run()

# Simulation node
class PrinterSimNode(Node):

    def __init__(self):
        super().__init__("printer_simulation_node")
        
        # Set up simulation environment
        self.sim = PrinterSimulation()
        self.sim.setup()
        
        # Run simulation in separate thread
        self.thread = PrinterSimThread()
        self.thread.start()

        # Subscribers for motors' positions
        self.sub_motor_x = self.create_subscription(Float64, "/motor_x", self.callback_motor_x, 10)
        self.sub_motor_y = self.create_subscription(Float64, "/motor_y", self.callback_motor_y, 10)
        # Subscriber for next waypoint to be drawn on canvas
        self.waypoint_sub = self.create_subscription(Point, "/next_waypoint", self.waiypoint_callback, 10)
        # Subscriber for commanding whether or not drawing on canvas
        self.drawing_sub = self.create_subscription(Bool, "/draw", self.drawing_command_callback, 10)

        # Publisher for next controller setpoint
        self.setpoint_pub = self.create_publisher(Point, "/controller_setpoint", 10)
        # Service for requesting motors' starting positions
        self.starting_pos_srv = self.create_service(EndEffectorPosition, "/end_effector_position", self.ee_position_callback)

    # Callback for setting position of motor x 
    def callback_motor_x(self, msg: Float64):
        self.sim.set_end_effector_position(x=msg.data)

    # Callback for setting position of motor y
    def callback_motor_y(self, msg: Float64):
        self.sim.set_end_effector_position(y=msg.data)

    # Callback for enabling/disabling drawing mode..
    def drawing_command_callback(self, msg: Bool):
        self.sim.is_drawing = msg.data
        mode = "ON" if msg.data else "OFF"
        self.get_logger().info("Drawing mode: ({0})".format(mode))
        
    # Callback for drawing reached waypoint and publish controller's setpoint for next one
    def waiypoint_callback(self, msg: Point):
        # Fill new msg for next controller's setpoint
        self.get_logger().info("Received next waypoint coordinates: ({0}, {1})".format(msg.x, msg.y))
        next_setpoint = Point()
        next_setpoint.x, next_setpoint.y = self.sim.get_coords_in_ee_frame(msg.x, msg.y)
        self.setpoint_pub.publish(next_setpoint)
 
    # Callback for retrieving initial end-effector's position (to properly initialize controllers)
    def ee_position_callback(self, request: EndEffectorPosition.Request, response: EndEffectorPosition.Response):
        drawing_area = self.sim.get_drawing_area()
        response.end_effector_position.x = 0.5 * drawing_area[0]
        response.end_effector_position.y = 0.5 * drawing_area[1]
        return response
        

# Main function
def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation
    printer_sim_node = PrinterSimNode()

    # Set printer's end-effector initial position in the center
    drawing_area = printer_sim_node.sim.get_drawing_area()
    printer_sim_node.sim.set_end_effector_position(0.5 * drawing_area[0], 0.5 * drawing_area[1])
    
    # Spin indefinitely..
    rclpy.spin(printer_sim_node)

    # On shutdown...
    printer_sim_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()