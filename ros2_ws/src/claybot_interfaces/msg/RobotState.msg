# General states
uint8 INIT=1
uint8 IDLE=2
uint8 ERROR=3
uint8 COMPLETE=4

# SQUARE states
uint8 SQUARE_CUT_SIDE_1=5
uint8 SQUARE_TURN_TO_SIDE_2=6
uint8 SQUARE_CUT_SIDE_2=7
uint8 SQUARE_TURN_TO_SIDE_3=8
uint8 SQUARE_CUT_SIDE_3=9
uint8 SQUARE_TURN_TO_SIDE_4=10
uint8 SQUARE_CUT_SIDE_4=11
uint8 SQUARE_TURN_TO_COMPLETE=12

# TRIANGLE states
uint8 TRIANGLE_CUT_SIDE_1=13
uint8 TRIANGLE_TURN_TO_SIDE_2=14
uint8 TRIANGLE_CUT_SIDE_2=15
uint8 TRIANGLE_TURN_TO_SIDE_3=16
uint8 TRIANGLE_CUT_SIDE_3=17
uint8 TRIANGLE_TURN_TO_COMPLETE=18

# HEXAGON states
uint8 HEXAGON_CUT_SIDE_1=19
uint8 HEXAGON_TURN_TO_SIDE_2=20
uint8 HEXAGON_CUT_SIDE_2=21
uint8 HEXAGON_TURN_TO_SIDE_3=22
uint8 HEXAGON_CUT_SIDE_3=23
uint8 HEXAGON_TURN_TO_SIDE_4=24
uint8 HEXAGON_CUT_SIDE_4=25
uint8 HEXAGON_TURN_TO_SIDE_5=26
uint8 HEXAGON_CUT_SIDE_5=27
uint8 HEXAGON_TURN_TO_SIDE_6=28
uint8 HEXAGON_CUT_SIDE_6=29
uint8 HEXAGON_TURN_TO_COMPLETE=30

# The current state of the robot
uint8 state
string shape_type  # "SQUARE", "TRIANGLE", or "HEXAGON"