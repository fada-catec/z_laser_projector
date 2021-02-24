uint8 POLYLINE=0
uint8 CIRCLE=1
uint8 ARC=2
uint8 OVAL=3
uint8 TEXT=4
uint8 POINTER=5

int32 figure_type # Type of figure (0 polyline, 1 circle, 2 arc, 3 oval, 4 text)

string projection_group
string figure_name

# Position of projection element:
#    Start point for POLYLINE, TEXT
#    Center point for CIRCLE, ARC, OVAL
geometry_msgs/Point position

# size is a 2-D array. NOTE: second index only used for OVAL and TEXT figure types
#
#  size[0] = length (POLYLINE)
#            radius (CIRCLE, ARC)
#            width (OVAL)
#            char height (TEXT)
#
#  size[1] = height (OVAL) 
#            char spacing (TEXT) 
float64[] size

# angle is a 2-D array expressed on Degrees. NOTE: second index only used for ARC figure type
#
#  angle[0] = slope (POLYLINE)
#             start angle (ARC)
#             rotation (OVAL, TEXT)
#
#  angle[1] = opening angle (ARC) 
float64[] angle

# NOTE: only used for TEXT figure type
# It refers to string of characters to be projected
string text
