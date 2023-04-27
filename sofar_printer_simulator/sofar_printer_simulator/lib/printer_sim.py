import arcade
from ament_index_python.packages import get_package_share_directory

# Constants
SCREEN_WIDTH = 1025
SCREEN_HEIGHT = 600
SCREEN_TITLE = "Printer Sim"

# Constants defining offset for printer's origin
X_OFFSET = 220
Y_OFFSET = 477

# Constants defining max dimensions for crane workspace
PRINTER_MAX_X = 590
PRINTER_MAX_Y = 375

# Constants used to scale our sprites from their original size
BASE_SCALING = 0.5
PRINTER_SCALING = 0.25
END_EFFECTOR_SCALING = 0.08

class PrinterSimulation(arcade.Window):

    def __init__(self):
        super().__init__(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
        # Base ROS2 package path
        self.base_share_directory = get_package_share_directory("sofar_printer_simulator")
        arcade.set_background_color(arcade.csscolor.WHITE)

        # Variables for keeping track of sprites lists
        self.printer_structure_list = None
        self.end_effector = None
        self.chain_list = None
        self.waypoints_list = None

        # Utilities
        self.is_drawing = False
        
    # Method to create static crane structure 
    def make_printer_structure(self):
        self.printer_structure_list = arcade.SpriteList(use_spatial_hash=True)
        # Left walls - vertical
        for y in range(5):
            v_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_vertical.png", BASE_SCALING)
            v_wall.position = [200, 150 + 70*y]
            self.printer_structure_list.append(v_wall)
        # Top-left wall
        tl_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_top_left.png", PRINTER_SCALING)
        tl_wall.position = [215, 138.5 + 70*(y+1)]
        self.printer_structure_list.append(tl_wall)
        # Top walls - horizontal
        for x in range(8):
            h_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_horizontal.png", BASE_SCALING)
            h_wall.position = [270 + 70*x, 154 + 70*(y+1)]
            self.printer_structure_list.append(h_wall)
        # Top-right wall
        tr_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_top_right.png", PRINTER_SCALING)
        tr_wall.position = [270 + 70*(x) + 57.5, 138.5 + 70*(y+1)]
        self.printer_structure_list.append(tr_wall)
        # Right walls - vertical
        for y in range(5):
            v_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_vertical.png", BASE_SCALING)
            v_wall.position = [270 + 70*(x+1) + 2.5, 150 + 70*y]
            self.printer_structure_list.append(v_wall)
        # Bottom left wall
        bl_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_bottom_left.png", PRINTER_SCALING)
        bl_wall.position = [215, 92]
        self.printer_structure_list.append(bl_wall)
        # Bottom walls - horizontal
        for x in range(8):
            h_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_horizontal.png", BASE_SCALING)
            h_wall.position = [270 + 70*x, 78]
            self.printer_structure_list.append(h_wall)
        # Bottom right wall
        br_wall = arcade.Sprite(self.base_share_directory + "/resource/beam_bottom_right.png", PRINTER_SCALING)
        br_wall.position = [258 + 70*(x+1), 92]
        self.printer_structure_list.append(br_wall)

    # Method to instantiate end-effector sprite and spawn it in printer's origin position
    def make_end_effector(self):
        self.end_effector = arcade.Sprite(self.base_share_directory + "/resource/laser.png", END_EFFECTOR_SCALING)
        self.end_effector.position = [X_OFFSET, Y_OFFSET]
        self.chain_list = arcade.SpriteList(use_spatial_hash=True)
        

    # Arcade standard method to initialize all graphical components - called once during initialization
    def setup(self):
        self.make_printer_structure()
        self.make_end_effector()
        # Set waypoints list up
        self.waypoints_list = arcade.SpriteList(use_spatial_hash=True)
        
    # Method to update end-effector position with values from motors
    def set_end_effector_position(self, x=None, y=None):
        if x is not None:
            self.end_effector.center_x = X_OFFSET + x 
        if y is not None:
            self.end_effector.center_y = Y_OFFSET - y
        # Then check if ee's position is still within graphical limits
        self.check_end_effector_in_limits()

    # Additional checks to make sure end-effector is drawn inside crane
    def check_end_effector_in_limits(self):
        # Checks for the X axis
        if self.end_effector.center_x > X_OFFSET + PRINTER_MAX_X:
            self.end_effector.center_x = X_OFFSET + PRINTER_MAX_X
        elif self.end_effector.center_x < X_OFFSET:
            self.end_effector.center_x = X_OFFSET
        # Checks for the Y axis
        if self.end_effector.center_y > Y_OFFSET:
            self.end_effector.center_y = Y_OFFSET 
        elif abs(self.end_effector.center_y - Y_OFFSET) > PRINTER_MAX_Y:
            self.end_effector.center_y = Y_OFFSET - PRINTER_MAX_Y

    # Method to retrieve drawing area (printer's inner dimensions)
    def get_drawing_area(self):
        return (PRINTER_MAX_X, PRINTER_MAX_Y)
    
    # Methods to retrieve coordinates in end-effector's frame of reference
    def get_coords_in_ee_frame(self, x, y):
        new_x = float(PRINTER_MAX_X / 2 + x)
        new_y = float(PRINTER_MAX_Y / 2 + y)
        # Assign coordinates as last waypoint
        self.last_waypoint = [new_x + X_OFFSET, Y_OFFSET - new_y]
        return new_x, new_y

    # Arcade standard method that redraws the canvas - called at every frame
    def on_draw(self):
        # Clear the screen to the background color
        arcade.start_render()
        # Draw printer structure
        self.printer_structure_list.draw()
        # Draw end-effector and chain
        self.end_effector.draw()
        self.chain_list.draw()
        # Draw waypoints, if drawing mode enabled
        self.waypoints_list.draw()


    # Method to draw chain once end-effector is moving
    def update_end_effector_chain(self):
        self.chain_list.clear()
        for v in range(Y_OFFSET + 10, int(self.end_effector.center_y) + 20, -10):
            chain = arcade.Sprite(self.base_share_directory + "/resource/chain.png", BASE_SCALING)
            chain.center_x = self.end_effector.center_x
            chain.center_y = v
            self.chain_list.append(chain)

    # Arcade standard method for updating simulation - called at every frame
    def on_update(self, delta_time: float):
        self.update_end_effector_chain()
        # Update line to draw following end-effector's movement
        if self.is_drawing:
            new_waypoint = arcade.Sprite(self.base_share_directory + "/resource/waypoint.png", BASE_SCALING)
            new_waypoint.center_x = self.end_effector.center_x
            new_waypoint.center_y = self.end_effector.center_y
            self.waypoints_list.append(new_waypoint)
