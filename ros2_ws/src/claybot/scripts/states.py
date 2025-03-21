from enum import Enum, auto

class RobotStates(Enum):
    """
    Enumeration of all possible states for the clay cutting robot.
    
    The states follow a pattern for each shape:
    - INIT: Initial state before starting any operation
    - CUT_SIDE_X: Robot is cutting the X-th side of the shape
    - TURN_TO_SIDE_X: Robot is rotating to position for cutting the X-th side
    - COMPLETE: Operation is complete, robot returned to initial position
    - ERROR: An error occurred during operation
    - RECOVERY: Robot is attempting to recover from an error
    
    Shape-specific states are grouped together and follow the same pattern.
    """
    
    # General states
    INIT = auto()                # Initial state before any operation
    IDLE = auto()                # Robot is idle, waiting for commands
    COMPLETE = auto()            # Operation complete
    
    # TO SQUARE states (4 sides, 90 degree turns)
    SQUARE_CUT_SIDE_1 = auto()   # Cutting first side of square
    SQUARE_TURN_TO_SIDE_2 = auto()  # Turning 90 degrees for second side
    SQUARE_CUT_SIDE_2 = auto()   # Cutting second side of square
    SQUARE_TURN_TO_SIDE_3 = auto()  # Turning 90 degrees for third side
    SQUARE_CUT_SIDE_3 = auto()   # Cutting third side of square
    SQUARE_TURN_TO_SIDE_4 = auto()  # Turning 90 degrees for fourth side
    SQUARE_CUT_SIDE_4 = auto()   # Cutting fourth side of square
    SQUARE_TURN_TO_COMPLETE = auto()  # Turning 90 degrees to return to initial position
    
    # TO TRIANGLE states (3 sides, 120 degree turns)
    TRIANGLE_CUT_SIDE_1 = auto()  # Cutting first side of triangle
    TRIANGLE_TURN_TO_SIDE_2 = auto()  # Turning 120 degrees for second side
    TRIANGLE_CUT_SIDE_2 = auto()  # Cutting second side of triangle
    TRIANGLE_TURN_TO_SIDE_3 = auto()  # Turning 120 degrees for third side
    TRIANGLE_CUT_SIDE_3 = auto()  # Cutting third side of triangle
    TRIANGLE_TURN_TO_COMPLETE = auto()  # Turning 120 degrees to return to initial position
    
    # TO HEXAGON states (6 sides, 60 degree turns)
    HEXAGON_CUT_SIDE_1 = auto()  # Cutting first side of hexagon
    HEXAGON_TURN_TO_SIDE_2 = auto()  # Turning 60 degrees for second side
    HEXAGON_CUT_SIDE_2 = auto()  # Cutting second side of hexagon
    HEXAGON_TURN_TO_SIDE_3 = auto()  # Turning 60 degrees for third side
    HEXAGON_CUT_SIDE_3 = auto()  # Cutting third side of hexagon
    HEXAGON_TURN_TO_SIDE_4 = auto()  # Turning 60 degrees for fourth side
    HEXAGON_CUT_SIDE_4 = auto()  # Cutting fourth side of hexagon
    HEXAGON_TURN_TO_SIDE_5 = auto()  # Turning 60 degrees for fifth side
    HEXAGON_CUT_SIDE_5 = auto()  # Cutting fifth side of hexagon
    HEXAGON_TURN_TO_SIDE_6 = auto()  # Turning 60 degrees for sixth side
    HEXAGON_CUT_SIDE_6 = auto()  # Cutting sixth side of hexagon
    HEXAGON_TURN_TO_COMPLETE = auto()  # Turning 60 degrees to return to initial position

    # Helper method to get all states for a specific shape
    @classmethod
    def get_shape_states(cls, shape):
        """
        Returns a list of states for the specified shape.
        
        Args:
            shape (str): One of 'SQUARE', 'TRIANGLE', or 'HEXAGON'
            
        Returns:
            list: Ordered list of states for the specified shape
        """
        if shape.upper() == 'SQUARE':
            return [
                cls.INIT,
                cls.SQUARE_CUT_SIDE_1,
                cls.SQUARE_TURN_TO_SIDE_2,
                cls.SQUARE_CUT_SIDE_2,
                cls.SQUARE_TURN_TO_SIDE_3,
                cls.SQUARE_CUT_SIDE_3,
                cls.SQUARE_TURN_TO_SIDE_4,
                cls.SQUARE_CUT_SIDE_4,
                cls.SQUARE_TURN_TO_COMPLETE,
                cls.COMPLETE
            ]
        elif shape.upper() == 'TRIANGLE':
            return [
                cls.INIT,
                cls.TRIANGLE_CUT_SIDE_1,
                cls.TRIANGLE_TURN_TO_SIDE_2,
                cls.TRIANGLE_CUT_SIDE_2,
                cls.TRIANGLE_TURN_TO_SIDE_3,
                cls.TRIANGLE_CUT_SIDE_3,
                cls.TRIANGLE_TURN_TO_COMPLETE,
                cls.COMPLETE
            ]
        elif shape.upper() == 'HEXAGON':
            return [
                cls.INIT,
                cls.HEXAGON_CUT_SIDE_1,
                cls.HEXAGON_TURN_TO_SIDE_2,
                cls.HEXAGON_CUT_SIDE_2,
                cls.HEXAGON_TURN_TO_SIDE_3,
                cls.HEXAGON_CUT_SIDE_3,
                cls.HEXAGON_TURN_TO_SIDE_4,
                cls.HEXAGON_CUT_SIDE_4,
                cls.HEXAGON_TURN_TO_SIDE_5,
                cls.HEXAGON_CUT_SIDE_5,
                cls.HEXAGON_TURN_TO_SIDE_6,
                cls.HEXAGON_CUT_SIDE_6,
                cls.HEXAGON_TURN_TO_COMPLETE,
                cls.COMPLETE
            ]
        else:
            raise ValueError(f"Unknown shape: {shape}. Supported shapes are SQUARE, TRIANGLE, and HEXAGON.")

    @classmethod
    def get_turn_angle(cls, state):
        """
        Returns the turn angle in degrees for a given turning state.
        
        Args:
            state (RobotStates): A turning state
            
        Returns:
            float: Turn angle in degrees
        """
        if state.name.startswith('SQUARE'):
            return 90.0
        elif state.name.startswith('TRIANGLE'):
            return 120.0
        elif state.name.startswith('HEXAGON'):
            return 60.0
        else:
            raise ValueError(f"Not a turning state: {state}")