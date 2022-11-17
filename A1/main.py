# Convex hull
#
# Usage: python main.py [-d] file_of_points
#
# You can press ESC in the window to exit.
#
# You'll need Python 3 and must install these packages:
#
#   PyOpenGL, GLFW


import sys, os, math
from tempfile import tempdir

try: # PyOpenGL
  from OpenGL.GL import *
except:
  print( 'Error: PyOpenGL has not been installed.' )
  sys.exit(1)

try: # GLFW
  import glfw
except:
  print( 'Error: GLFW has not been installed.' )
  sys.exit(1)



# Globals

window = None

windowWidth  = 800 # window dimensions
windowHeight = 800

minX = None # range of points
maxX = None
minY = None
maxY = None

r  = 0.01 # point radius as fraction of window size

numAngles = 32
thetas = [ i/float(numAngles)*2*3.14159 for i in range(numAngles) ] # used for circle drawing

allPoints = [] # list of points

lastKey = None  # last key pressed

discardPoints = False

# Point
#
# A Point stores its coordinates and pointers to the two points beside
# it (CW and CCW) on its hull.  The CW and CCW pointers are None if
# the point is not on any hull.
#
# For debugging, you can set the 'highlight' flag of a point.  This
# will cause the point to be highlighted when it's drawn.

class Point(object):

    def __init__( self, coords ):

      self.x = float( coords[0] ) # coordinates
      self.y = float( coords[1] )

      self.ccwPoint = None # point CCW of this on hull
      self.cwPoint  = None # point CW of this on hull

      self.highlight = False # to cause drawing to highlight this point


    def __repr__(self):
      return 'pt(%g,%g)' % (self.x, self.y)


    def drawPoint(self):

      # Highlight with yellow fill
      
      if self.highlight:
          glColor3f( 0.9, 0.9, 0.4 )
          glBegin( GL_POLYGON )
          for theta in thetas:
              glVertex2f( self.x+r*math.cos(theta), self.y+r*math.sin(theta) )
          glEnd()

      # Outline the point
      
      glColor3f( 0, 0, 0 )
      glBegin( GL_LINE_LOOP )
      for theta in thetas:
          glVertex2f( self.x+r*math.cos(theta), self.y+r*math.sin(theta) )
      glEnd()

      # Draw edges to next CCW and CW points.

      if self.ccwPoint:
        glColor3f( 0, 0, 1 )
        drawArrow( self.x, self.y, self.ccwPoint.x, self.ccwPoint.y )

      if self.cwPoint:
        glColor3f( 1, 0, 0 )
        drawArrow( self.x, self.y, self.cwPoint.x, self.cwPoint.y )



# Draw an arrow between two points, offset a bit to the right

def drawArrow( x0,y0, x1,y1 ):

    d = math.sqrt( (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) )

    vx = (x1-x0) / d      # unit direction (x0,y0) -> (x1,y1)
    vy = (y1-y0) / d

    vpx = -vy             # unit direction perpendicular to (vx,vy)
    vpy = vx

    xa = x0 + 1.5*r*vx - 0.4*r*vpx # arrow tail
    ya = y0 + 1.5*r*vy - 0.4*r*vpy

    xb = x1 - 1.5*r*vx - 0.4*r*vpx # arrow head
    yb = y1 - 1.5*r*vy - 0.4*r*vpy

    xc = xb - 2*r*vx + 0.5*r*vpx # arrow outside left
    yc = yb - 2*r*vy + 0.5*r*vpy

    xd = xb - 2*r*vx - 0.5*r*vpx # arrow outside right
    yd = yb - 2*r*vy - 0.5*r*vpy

    glBegin( GL_LINES )
    glVertex2f( xa, ya )
    glVertex2f( xb, yb )
    glEnd()

    glBegin( GL_POLYGON )
    glVertex2f( xb, yb )
    glVertex2f( xc, yc )
    glVertex2f( xd, yd )
    glEnd()
      
      

# Determine whether three points make a left or right turn

LEFT_TURN  = 1
RIGHT_TURN = 2
COLLINEAR  = 3

def turn( a, b, c ):

    det = (a.x-c.x) * (b.y-c.y) - (b.x-c.x) * (a.y-c.y)

    if det > 0:
        return LEFT_TURN
    elif det < 0:
        return RIGHT_TURN
    else:
        return COLLINEAR


# Build a convex hull from a set of point
#
# Use the method described in class
#pointsToRemove=[]

# FUNCTION: merge
# DESC: merges two hulls together using WalkUpward and WalkDownward concepts
# PRE: two list of points that form a convex hull we want to join
# POST: returns list of points that form a convex hull
def merge(hull1, hull2):

    # base case, check if we have a hull that is null 
    #if(hull1 == None or hull2 == None):
    #    return

    #print("\nmerging\n" + str(hull1) + "\nand\n" + str(hull2))

    # get the rightmost point of left convex hull -- lima, i used NATO phonetic cause idk the greek counter part at the time :/
    lima = max(hull1, key = lambda point: point.x)

    # get the leftmost point of right convex hull -- romeo
    romeo = min(hull2, key = lambda point: point.x)

    # initalize array holding our points
    hull = []
    pointsToRemove=[]

    # SUBFUNCTION: Walk upward joins the topmost points of both halls by
    #              starting at lima (rightmost point of left convex hull)
    #              and romeo (leftmost point of right convex hull) and moves
    #              upward by comparing the interior angles of each connected point.
    #              Our output would update the upper tangent points with new pointers.
    def walkUpward( lima, romeo ):
        # check interior angles
        while turn( lima.ccwPoint, lima, romeo) == LEFT_TURN or turn(lima, romeo, romeo.cwPoint) == LEFT_TURN:
            if turn ( lima.ccwPoint, lima, romeo ) == LEFT_TURN: # move lima upwards
                if lima not in pointsToRemove:
                    pointsToRemove.append(lima) # save point we want to delete later
                lima = lima.ccwPoint
            else: # move romeo upwards
                if romeo not in pointsToRemove:
                    pointsToRemove.append(romeo)
                romeo = romeo.cwPoint
        #print("linking " + str(lima) + " and " + str(romeo))
        display(True)
        lima.cwPoint = romeo # update pointer values
        romeo.ccwPoint = lima
        # add points we want to remove and append to our completed hull
        if lima in pointsToRemove:
            pointsToRemove.remove(lima)
        if romeo in pointsToRemove:
            pointsToRemove.remove(romeo)
        if lima not in hull:
            hull.append(lima)
        if romeo not in hull:
            hull.append(romeo)
        if lima in hull1:
            hull1.remove(lima)
        if romeo in hull2:
            hull2.remove(romeo)

    # SUBFUNCTION: Walk downwards joins the bottommost points of both halls by
    #              starting at lima (rightmost point of left convex hull)
    #              and romeo (leftmost point of right convex hull) and moves
    #              downward by comparing the interior angles of each connected point.
    #              Our output would update the lower tangent points with new pointers.
    def walkDownward( lima, romeo ):
        while turn( lima.cwPoint, lima, romeo) == RIGHT_TURN or turn(lima, romeo, romeo.ccwPoint) == RIGHT_TURN:
            if turn ( lima.cwPoint, lima, romeo ) == RIGHT_TURN:
                if lima not in pointsToRemove: # move lima downward
                    pointsToRemove.append(lima)
                lima = lima.cwPoint
            else: # move romeo downwards
                if romeo not in pointsToRemove: # move romeo downward
                    pointsToRemove.append(romeo)
                romeo = romeo.ccwPoint
            
        #print("linking " + str(lima) + " and " + str(romeo))
        display(True)
        romeo.cwPoint = lima
        lima.ccwPoint = romeo
        # add points we want to remove and append to our completed hull
        if lima in pointsToRemove:
            pointsToRemove.remove(lima)
        if romeo in pointsToRemove:
            pointsToRemove.remove(romeo)
        if lima not in hull:
            hull.append(lima)
        if romeo not in hull:
            hull.append(romeo)
        if lima in hull1:
            hull1.remove(lima)
        if romeo in hull2:
            hull2.remove(romeo)

    # join upper part of our two hulls
    walkUpward(lima, romeo)

    # get the rightmost point of left convex hull after update
    lima = max(hull1, key = lambda point: point.x)

    # get the leftmost poitn of right convex hull after update
    romeo = min(hull2, key = lambda point: point.x)

    # join lower part of our two hulls
    walkDownward(lima, romeo)
    
    # append points we didn't have to traverse to into our finished hull
    for point1 in hull1:
        if point1 not in pointsToRemove and point1 not in hull:
            #print("adding + " + str(point1))
            hull.append(point1)
    for point2 in hull2:
        if point2 not in pointsToRemove and point2 not in hull:
            #print("adding + " + str(point2))
            hull.append(point2)
    for n in pointsToRemove:
        n.cwPoint = None
        n.ccwPoint = None

    startingPoint = lima
    curr = startingPoint
    while(True):
        print(curr)
        curr = curr.ccwPoint
        if(curr == startingPoint):
            break
    #print("Resultant hull:")
    # sort points for convenience
    hull.sort( key=lambda p: (p.x,p.y))
    #print(hull)
    #print("exiting merge")

    return hull

# FUNCTION: convexBruteForce
# DESC: Brute Force Convex Hull Algorithm for a given hull, used for the base case of buildHull
# PRE: list of points we want to make a hull with
# POST: returns list of points that form a convex hull
def convexBruteForce( points ):
    
    ### THE GREATEST HULL BUILDER ALGORITHM OF YOUR LIFE YO ###

    ## Stage 1 : Find the lowest point possible

    minIndex = 0
    for i in range(1,len(points)):
        if points[i].x < points[minIndex].x:
            minIndex = i
        elif points[i].x == points[minIndex].x:
            if points[i].y > points[minIndex].y:
                minIndex = i

    ## Stage 2: Building the Hull, start at Leftmostpoint
    ##          move left (or counterclockwise) until we
    ##          reach the beginning
    
    currentHull = []

    currentPointIndex = minIndex
    tempPointIndex = 0

    while(True):
        currentHull.append(points[currentPointIndex])

        # search for a counterclockwise path starting from q
        tempPointIndex = (currentPointIndex + 1) % len(points)

        for i in range(len(points)):

            if(turn(points[currentPointIndex], points[i], points[tempPointIndex]) == 2):
                tempPointIndex = i
                
        # tempPointIndex is the most counter clockswise so we overwrite currentPointIndex
        
        points[tempPointIndex].ccwPoint = points[currentPointIndex]
        points[currentPointIndex].cwPoint = points[tempPointIndex]
        #display(True)
        currentPointIndex = tempPointIndex

        if(currentPointIndex == minIndex):
            break
    return currentHull
    
# FUNCTION: buildHull
# DESC: This recursive function utilizes divide and conquer routine to build a convex hull
# PRE: list of points we want to make a hull with
# POST: returns list of points that form a convex hull
def buildHull( points ):

    # define array holding left and right partitions
    left = []
    right = []

    # define index midpoint of all points
    midIndex = int(len(points)/2)

    # Handle base cases of two or three points--runs Graham Scan on a 2/3 point hull
    if len(points) <= 3:
        return convexBruteForce( points )

    # if points == None:
    #     return
    
    # pushes points into left and right partitions
    for i in range(0, midIndex):
        left.append(points[i])
    for j in range(midIndex, len(points)):
        right.append(points[j])

    # print(left)
    # print("Left List contains " + str(len(left)) + " points")
    # print(right)
    # print("Right List contains " + str(len(left)) + " points")
    
    # recursive passes Left and Right partition to buildHull until base case is reached
    left_hull = buildHull(left)
    right_hull = buildHull(right)

    # print("merging " + str(left_hull) + " and " + str(right_hull))

    # merges both partitions together
    merged_hull = merge(left_hull,right_hull)
    #print("Resultant hull: \n" + str(merged_hull))
    
    

    # After you get the hull-merge working, do the following: For each
    # point that was removed from the convex hull in a merge, set that
    # point's CCW and CW pointers to None.  You'll see that the arrows
    # from interior points disappear after you do this.

    # loop helps properly visualize hull on GUI
    # for n in pointsToRemove:
    #     n.cwPoint = None
    #     n.ccwPoint = None

    # You can do the following to help in debugging.  This highlights
    # all the points, then shows them, then pauses until you press
    # 'p'.  While paused, you can click on a point and its coordinates
    # will be printed in the console window.  If you are using an IDE
    # in which you can inspect your variables, this will help you to
    # identify which point on the screen is which point in your data
    # structure.
    #
    # This is good to do, for example, after you have recursively
    # built two hulls, to see that the two hulls look right.
    #
    # This can also be done immediately after you have merged to hulls
    # ... again, to see that the merged hull looks right.
    #
    # Always after you have inspected things, you should remove the
    # highlighting from the points that you previously highlighted.

    # for p in points:
    #     p.highlight = True
    # display(wait=True)


    
    # At the very end of buildHull(), you should display the result
    # after every merge, as shown below.  This call to display() does
    # not pause.
    return merged_hull
    display()

  

windowLeft   = None
windowRight  = None
windowTop    = None
windowBottom = None


# Set up the display and draw the current image

def display( wait=False ):

    global lastKey, windowLeft, windowRight, windowBottom, windowTop
    
    # Handle any events that have occurred

    glfw.poll_events()

    # Set up window

    glClearColor( 1,1,1,0 )
    glClear( GL_COLOR_BUFFER_BIT )
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL )

    glMatrixMode( GL_PROJECTION )
    glLoadIdentity()

    glMatrixMode( GL_MODELVIEW )
    glLoadIdentity()

    windowLeft   = -0.1*(maxX-minX)+minX
    windowRight  =  1.1*(maxX-minX)+minX
    windowTop    =  1.1*(maxY-minY)+minY
    windowBottom = -0.1*(maxY-minY)+minY

    glOrtho( windowLeft, windowRight, windowBottom, windowTop, 0, 1 )

    # Draw points and hull

    for p in allPoints:
        p.drawPoint()

    # Show window

    glfw.swap_buffers( window )

    # Maybe wait until the user presses 'p' to proceed
    
    if wait:

        sys.stderr.write( 'Press "p" to proceed ' )
        sys.stderr.flush()

        lastKey = None
        while lastKey != 80: # wait for 'p'
            glfw.wait_events()
            if glfw.window_should_close( window ):
              sys.exit(0)
            display()

        sys.stderr.write( '\r                     \r' )
        sys.stderr.flush()


    

# Handle keyboard input

def keyCallback( window, key, scancode, action, mods ):

    global lastKey
    
    if action == glfw.PRESS:
    
        if key == glfw.KEY_ESCAPE:      # quit upon ESC
            glfw.set_window_should_close( window, GL_TRUE )
        else:
            lastKey = key



# Handle window reshape


def windowReshapeCallback( window, newWidth, newHeight ):

    global windowWidth, windowHeight

    windowWidth  = newWidth
    windowHeight = newHeight



# Handle mouse click/release

def mouseButtonCallback( window, btn, action, keyModifiers ):

    if action == glfw.PRESS:

        # Find point under mouse

        x,y = glfw.get_cursor_pos( window ) # mouse position

        wx = (x-0)/float(windowWidth)  * (windowRight-windowLeft) + windowLeft
        wy = (windowHeight-y)/float(windowHeight) * (windowTop-windowBottom) + windowBottom

        minDist = windowRight-windowLeft
        minPoint = None
        for p in allPoints:
            dist = math.sqrt( (p.x-wx)*(p.x-wx) + (p.y-wy)*(p.y-wy) )
            if dist < r and dist < minDist:
                minDist = dist
                minPoint = p

        # print point and toggle its highlight

        if minPoint:
            minPoint.highlight = not minPoint.highlight
            print( minPoint )

        
    
# Initialize GLFW and run the main event loop

def main():

    global window, allPoints, minX, maxX, minY, maxY, r, discardPoints
    
    # Check command-line args

    if len(sys.argv) < 2:
        print( 'Usage: %s filename' % sys.argv[0] )
        sys.exit(1)

    args = sys.argv[1:]
    while len(args) > 1:
        print( args )
        if args[0] == '-d':
            discardPoints = not discardPoints
        args = args[1:]

    # Set up window
  
    if not glfw.init():
        print( 'Error: GLFW failed to initialize' )
        sys.exit(1)

    window = glfw.create_window( windowWidth, windowHeight, "Assignment 1", None, None )

    if not window:
        glfw.terminate()
        print( 'Error: GLFW failed to create a window' )
        sys.exit(1)

    glfw.make_context_current( window )
    glfw.swap_interval( 1 )
    glfw.set_key_callback( window, keyCallback )
    glfw.set_window_size_callback( window, windowReshapeCallback )
    glfw.set_mouse_button_callback( window, mouseButtonCallback )

    # Read the points

    with open( args[0], 'rb' ) as f:
      allPoints = [ Point( line.split(b' ') ) for line in f.readlines() ]

    # Get bounding box of points

    minX = min( p.x for p in allPoints )
    maxX = max( p.x for p in allPoints )
    minY = min( p.y for p in allPoints )
    maxY = max( p.y for p in allPoints )

    # Adjust point radius in proportion to bounding box
    
    if maxX-minX > maxY-minY:
        r *= maxX-minX
    else:
        r *= maxY-minY

    # Sort by increasing x.  For equal x, sort by increasing y.
    
    allPoints.sort( key=lambda p: (p.x,p.y))
    print("Points List in file:")
    print(allPoints)
    print("List contains " + str(len(allPoints)) + " points")
    print("\n")

    # Run the code
    
    hull = buildHull( allPoints )
    #hull = convexBruteForce(allPoints)
    print("Your convex hull, my good sir, is :")
    print(hull)
    print(len(hull))
    for p in hull:
        p.highlight = True
    display(wait=True)
    

    # Wait to exit

    while not glfw.window_should_close( window ):
        glfw.wait_events()

    glfw.destroy_window( window )
    glfw.terminate()
    


if __name__ == '__main__':
    main()
