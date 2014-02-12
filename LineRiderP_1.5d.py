#LineRiderP_1.5b.py
#David Lu + delu + section A
from Tkinter import *
import Tkinter, tkMessageBox
import time
import math
import random
import os
import copy
import pickle
#import cucumber
#import cherry
#import pomegranate
#collision detection adapted from
#www.t3hprogrammer.com/research/line-circle-collision/tutorial
#rigid body dynamics adapted from
#www.gpgstudy.com/gpgiki/GDC 2001%3A Advanced Character Physics
"""
x, y are coordinates
r is a position vector made of <x,y>
p is a point that is interacting with physics
p, q are arbitrary coordinates (x, y)
u, v are arbitrary vectors <x, y>
line is a line segment made of two position vectors
cnstr is a constraint made of two points
"""
###############################################################################
#                           ~ VECTORS 'N STUFF ~ 
###############################################################################
class vector(object):
    #slots for optimization...?
#   __slots__ = ('x', 'y')
    def __init__(self, x, y = 0):
        if type(x) == tuple:
            x, y = x[0], x[1]
        self.x = float(x)
        self.y = float(y)
    def __repr__(self):
        return "vector"+str((self.x, self.y))
    #vector addition/subtraction
    def __add__(self, other):
        return vector(self.x+other.x, self.y+other.y)
    def __sub__(self, other):
        return vector(self.x-other.x, self.y-other.y)
    def __iadd__(self,other):
        self.x += other.x; self.y += other.y
        return self
    def __isub__(self,other):
        self.x -= other.x; self.y -= other.y
        return self
    #scalar multiplication/division
    def __mul__(self, other):
        """exception: if other is also a vector, DOT PRODUCT
            I do it all the time in my 3d calc hw anyways :|"""
        if isinstance(other, vector):
            return dot(self, other)
        else: return vector(self.x*other, self.y*other)
    def __div__(self, other):
        return vector(self.x/other, self.y/other)
    def __imul__(self,other):
        self.x *= other; self.y *= other
        return self
    def __idiv__(self, other):
        self.x /= other; self.y /= other
        return self
    def magnitude2(self):
        """the square of the magnitude of this vector"""
        return distance2((self.x, self.y), (0,0))
    def magnitude(self):
        """magnitude of this vector)"""
        return distance((self.x, self.y), (0,0))
    def normalize(self):
        """unit vector. same direction but with a magnitude of 1"""
        if (self.x, self.y) == (0,0): return vector(0,0)
        else: return self/self.magnitude()
    def rotate(self, other):
        """rotates vector in radians"""
        x = self.x*math.cos(other) - self.y*math.sin(other)
        y = self.x*math.sin(other) + self.y*math.cos(other)
        return vector(x, y)
    def getAngle(self):
        """gets angle of vector relative to x axis"""
        return math.atan2(self.y, self.x)
        
# x**0.5 is annoying to compute, in python and in real life, it's a bit faster
def distance2(p, q):
    """distance squared between these points"""
    if isinstance(p, vector) and isinstance(q, vector):
        return (p.x-q.x)**2 + (p.y-q.y)**2
    else:
        return (p[0]-q[0])**2 + (p[1]-q[1])**2

def distance(p, q):
    """distance between these points"""
    return distance2(p, q)**0.5
        
def dot(u, v):
    """dot product of these vectors, a scalar"""
    return u.x*v.x + u.y*v.y

def cross(u, v):
    """magnitude of the cross product of these vectors"""
    return det(u.x, u.y, v.x, v.y)
###############################################################################
#                           ~ MOAR LINEAR ALGEBRA ~
###############################################################################
class Line(object):
    """lines are defined by two points, p1 and p2"""
#   __slots__ = ('r1', 'r2') #I ANTICIPATE THOUSANDS OF LINES
    def __init__(self, r1, r2):
    #points are position vectors
        if isinstance(r1, vector) and isinstance(r2, vector):
            self.r1 = r1
            self.r2 = r2
        else: #inputs are tuples
            self.r1 = vector(r1[0], r1[1])
            self.r2 = vector(r2[0], r2[1])
    def __repr__(self):
        return "Line"+str((self.r1, self.r2, self.lineType))
    def linearEquation(self):
        """in the form of ax+by=c"""
        a = self.r2.y - self.r1.y
        b = self.r1.x - self.r2.x
        c = a*self.r1.x + b*self.r1.y
        return a, b, c
    
class Point(object):
    """points have position and velocity implied by previous position"""
    def __init__(self, *args):
        x, y = args[0], args[1]
        #current position
        self.r = vector(x, y)
        #position one frame before. no value, set to same as x,y
        if len(args) == 2:
            x0, y0 = x, y
        self.r0 = vector(x0, y0)
    def __repr__(self):
        return "Point"+str( (self.r, self.r0) )

def intersectPoint(line1, line2):
    """Returns the coordinates of intersection between line segments.
    If the lines are not intersecting, returns None.""" 
    a1,b1,c1 = line1.linearEquation()
    a2,b2,c2 = line2.linearEquation()
    d = det(a1, b1, a2, b2)
    if d == 0: return None
    x = 1.0*det(c1, b1, c2, b2)/d
    y = 1.0*det(a1, c1, a2, c2)/d
    intersection = vector(x, y)
    if (isInLineRegion(intersection, line1)
        and isInLineRegion(intersection, line2)):
        return intersection #position vector of point of intersection
        
def det(a, b, c, d):
    """determinant"""
    # | a b |
    # | c d |
    return a*d - b*c

def isInLineRegion(r, line):
    """rectangular region defined by endpoints of a line segment or points"""
    return isInRegion(r, line.r1, line.r2)


def isInRegion(r, pnt1, pnt2):
    if isinstance(pnt1, vector):
        x1, x2, y1, y2 = pnt1.x, pnt2.x, pnt1.y, pnt2.y
    else:
        x1, x2, y1, y2 = pnt1[0], pnt2[0], pnt1[1], pnt2[1]
    if isinstance(r, vector):
        rx, ry = r.x, r.y
    else:
        rx, ry = r[0], r[1]
    x1, x2 = min(x1, x2), max(x1, x2)
    y1, y2 = min(y1, y2), max(y1, y2)
    isInRect = x1<= rx <=x2 and y1<= ry <=y2
    isOnHLine = x1==x2 and almostEqual(x1,rx) and y1 <= ry <= y2
    isOnVLine = y1==y2 and almostEqual(y1,ry) and x1 <= rx <= x2
    return isInRect or isOnHLine or isOnVLine

def almostEqual(a, b):
    return abs(a-b) < canvas.data.epsilon/100

def closestPointOnLine(r, line):
    """closest point on a line to a point(position vector)"""
    a, b, c1 = line.linearEquation()
    c2 = -b*r.x + a*r.y
    d = det(a, b, -b, a)
    if d == 0:
        point = r #POINT IS ON LINE
    else:
        x = det(a, b, c2, c1)/d
        y = det(a, -b, c1, c2)/d
        point = vector(x, y) #it's a position vector!
    return point
def distanceFromLine(r, line):
    """distance between a point and a line segment"""
    linePoint = closestPointOnLine(r, line)
    if isInLineRegion(linePoint, line):
        dist = distance(r, linePoint)
    else:
        dist = min(distance(r, line.r1), distance(r, line.r2))
    return dist



###############################################################################
#                       ~ POINT-LINE COLLISIONS OH SNAP ~
###############################################################################
class solidLine(Line):
    """collidable lines"""
    def __init__(self, r1, r2, lineType):
        super(solidLine, self).__init__(r1, r2)
#       self.dir = (self.r2-self.r1).normalize() #direction the line points in
    #normal to line, 90 degrees ccw (to the left). DARN YOU FLIPPED COORDINATES
#       self.norm = vector(self.dir.y, -self.dir.x)
        self.type = lineType
    def __repr__(self):
        return "solidLine"+str((self.r1, self.r2, self.type))

#new resolveCollision, evaluates the closest line first, then reevaluates
#that means the original order of lines DOESN'T MATTER
def resolveCollision(pnt):
    """takes a solid point, finds and resolves collisions,
    and returns the acceleration lines it collided with"""
    hasCollided = True
    a = canvas.data.acc
    maxiter = canvas.data.maxiter
    accLines = set()
    while hasCollided == True and maxiter > 0:
        hasCollided = False
#get the lines the point may collide with
        lines = getCollisionLines(pnt)
#get the collision points of the lines the point actually collides with
        collidingLines, collisionPoints, intersections = getCollidingLines(pnt, lines)
#no collisions? break
        if collisionPoints == []:
            break
        elif len(collisionPoints) >1:
#more than one collision points: get the intersection point closest to the point
            futurePoint = closestCollisionPoint(pnt, intersections, collisionPoints)
            index=collisionPoints.index(futurePoint)
            collidingLine = collidingLines[index]
        else:
            futurePoint = collisionPoints[0]
            collidingLine = collidingLines[0]
#set future point to above point, evaluate acc line if necessary
        pnt.r = futurePoint
        if collidingLine.type == "acc":
            accLines.add(collidingLine)
        hasCollided = True
        maxiter -= 1
        if canvas.data.viewCollisions:
           canvas.data.collisionPoints += [copy.copy(futurePoint)]
        sensitiveButt(pnt)
    return accLines
#repeat if there was a collision

def getCollidingLines(pnt, lines):
    """"returns a list of the lines "pnt" actually collides with
        and the respective intersection points"""
    collisionPoints = []
    collidingLines = []
    intersections = []
    for line in lines:
        point = getCollision(pnt, line)
        if point != None:
            collidingLines += [line]
            collisionPoints += [point[0]]
            intersections += [point[1]]
    return collidingLines, collisionPoints, intersections


def closestPoint(pnt, points):
    closestPoint = points[0] #first point
    minDist = distance(closestPoint,pnt)
    for point in points[1:]:
        dist = distance(point,pnt)
        if dist < minDist:
            minDist = dist
            closestPoint = point
    return closestPoint

def closestCollisionPoint(pnt, intersections, collisionPoints):
    closestIntersection = closestPoint(pnt.r0, intersections)
    i = intersections.index(closestIntersection)
    return collisionPoints[i]

def getCollisionLines(pnt):
    """returns a set of lines that exist in the same cells as the point"""
    vLine = Line(pnt.r0, pnt.r)
    cells = getGridCells(vLine) #list of cell positions
    lines = set()
    solids = canvas.grid.solids
    for gPos in cells:
        cell = solids.get(gPos, set())
        lines |= cell #add to set of lines to check collisions
    return lines

def getCollision(pnt, line):
    """returns the position after collision(if it exists)"""
    trajectory = Line(pnt.r, pnt.r0)
    intersection = intersectPoint(trajectory, line)
    thickness = canvas.data.lineThickness + canvas.data.epsilon
    if intersection != None:
        #if intersecting, project position onto line
        futurePos = closestPointOnLine(pnt.r, line)
        #line thickness
        futurePos += (futurePos-pnt.r).normalize()*thickness
#        print "intersection!"
        return futurePos, intersection
    elif distanceFromLine(pnt.r, line) < canvas.data.lineThickness:
        #if inside line, do same as above except reverse direction of epsilon
        futurePos = closestPointOnLine(pnt.r, line)
        futurePos += (pnt.r-futurePos).normalize()*thickness
#        print "inside line! Moving", pnt.r, "to", futurePos
        return futurePos, pnt.r #pnt.r is in hte line: it's the intersection
    else: return None

def sideOfLine(pnt, line):
    """the side of the line that the point collides with"""
#dot product of the normal and velocity
    velocity = pnt.r - pnt.r0
    side = velocity*line.norm
#the sign determines the side the point is colliding with the line
#-1 is right, +1 is left, to coorespond with the side of the normal
    if side>0: return -1
    elif side<0: return 1
    else: return 0 # THIS IS GONNA BUG THE HELL OUT OF ME AHHHHHHHHH
###############################################################################
#                       ~ RIGID BODY DYNAMICS WHAT EVEN ~
###############################################################################
class Constraint():
    def __init__(self, pnt1, pnt2, restLength):
        #p1 and p2 are points
        self.pnt1, self.pnt2 = pnt1, pnt2
        self.restLength = restLength
    def __repr__(self):
        return str((self.pnt1, self.pnt2, self.restLength))
        
def resolveConstraint(cnstr):
    """resolves a given constraint"""
    delta = cnstr.pnt1.r - cnstr.pnt2.r
    deltaLength = delta.magnitude()
    diff = (deltaLength-cnstr.restLength)/deltaLength
    cnstr.pnt1.r -= delta*diff/2
    cnstr.pnt2.r += delta*diff/2

def resolveScarf(cnstr):
    """one sided constraints"""
    delta = cnstr.pnt1.r - cnstr.pnt2.r
    deltaLength = delta.magnitude()
    diff = (deltaLength-cnstr.restLength)/deltaLength
    cnstr.pnt2.r += delta*diff

def resolveLegs(cnstr):
    """the constraint can only take on a minimum length"""
    delta = cnstr.pnt1.r - cnstr.pnt2.r
    deltaLength = delta.magnitude()
    diff = (deltaLength-cnstr.restLength)/deltaLength
    if diff < 0: #length is too small
        cnstr.pnt1.r -= delta*diff/2
        cnstr.pnt2.r += delta*diff/2

def checkEndurance(cnstr):
    """if the ratio of the difference of length is beyond a certain
        limit, destroy line rider's attachment to the sled"""
    endurance = canvas.data.endurance
    delta = cnstr.pnt1.r - cnstr.pnt2.r
    deltaLength = delta.magnitude()
    diff = (deltaLength-cnstr.restLength)
    ratio = abs(diff/cnstr.restLength)
    if ratio > endurance:
        #remove constraints
        killBosh()

def killBosh():
    if canvas.rider.onSled:
        canvas.rider.constraints = canvas.rider.constraints[:-8]
        canvas.rider.onSled = False

def sensitiveButt(pnt):
    if pnt == canvas.rider.points[0]:
        #LINE RIDER'S BUTT IS SENSITIVE. TOUCH IT AND HE FALLS OFF THE SLED.
        killBosh()
###############################################################################
#                           ~ OBEY MY PHYSICS, DARN IT ~
###############################################################################
def freeFall(pnt, mass=1):
    """All points are independent, acting only on inertia, drag, and gravity
    The velocity is implied with the previous position"""
    velocity = pnt.r - pnt.r0
    pnt.r =  pnt.r + velocity*canvas.data.drag*mass + canvas.data.grav
    
def updatePositions():
    if canvas.data.viewCollisions:
        canvas.data.collisionPoints = []
    for pnt in canvas.rider.points:
        #first, update points based on inertia, gravity, and drag
        pastPos = pnt.r
        freeFall(pnt)
        pnt.r0 = pastPos
    #scarves are special :|
    for pnt in canvas.rider.scarf:
        pastPos = pnt.r
        freeFall(pnt, 0.5)
        pnt.r0 = pastPos
    #add in the acceleration lines from previous step
    accQueue = canvas.rider.accQueueNow
    a = canvas.data.acc
    for pnt, lines in accQueue.iteritems():
        for line in lines:
            acc = (line.r2 - line.r1).normalize()
            acc *= a
            pnt.r += acc
    canvas.rider.accQueuePast = copy.copy(canvas.rider.accQueueNow)
    canvas.rider.accQueueNow = dict() #empty queue after
    for i in xrange(canvas.data.iterations):
        #collisions get priority to prevent phasing through lines
        for cnstr in canvas.rider.legsC:
            resolveLegs(cnstr)
        if canvas.rider.onSled:
            for cnstr in canvas.rider.slshC:
                checkEndurance(cnstr)
        for cnstr in canvas.rider.constraints:
            resolveConstraint(cnstr)
        for pnt in canvas.rider.points:
            accLines = resolveCollision(pnt)
            if len(accLines)>0: #contains lines
                canvas.rider.accQueueNow[pnt] = accLines
    scarfStrength = 1
    #again, scarves are special
    for i in xrange(scarfStrength):
        for cnstr in canvas.rider.scarfCnstr:
            resolveScarf(cnstr)
#    canvas.data.tracer += [canvas.rider.pos.r]
###############################################################################
#                               ~OPTIMIZATION PRIME~
###############################################################################
#perhaps the least intuitive part of this program.
#the grid is canvas.grid, a dictionary of tuples that coorespond to
#what line exists in the block the coordinates point to
#credits to my brother for explaining how this works
def getGridCells(line):
    """returns a list of the cells the line exists in"""
    firstCell = gridPos(line.r1)
    lastCell = gridPos(line.r2)
    if firstCell[0] > lastCell[0]: #going in negative x direction MAKES BUGS
        #JUST FLIP'EM
        firstCell, lastCell = lastCell, firstCell
    cells = [firstCell]
    gridInts = getGridInts(line, firstCell, lastCell)
    cell = firstCell
    for x in sorted(gridInts): #ordered by which cell the line enters
        dcor = gridInts[x]
        cell = cell[0]+dcor[0], cell[1]+dcor[1]
        cells += [cell]
#   if lastCell not in cells:
#       cells += [lastCell]
    return cells

def gridFloor(x):
    return int(x - x%canvas.data.gridSize)

def gridPos(pnt):
    return gridFloor(pnt.x), gridFloor(pnt.y)

def getGridInts(line, firstCell, lastCell):
    a, b, c = line.linearEquation()
    dx = dy = canvas.data.gridSize #defined to be always positive
    if lastCell[1] < firstCell[1]: #y is decreasing
        dy *= -1
    gridInts = {}
    xInc, yInc = (dx, 0), (0, dy)
    #normally, I would just use x = (c-b*y)/a
    if b==0: #a might be 0 so SWAP VALUES
        b, a = a, b
        yInc, xInc = xInc, yInc
        dy, dx = dx, dy
        firstCell = firstCell[1], firstCell[0]
        lastCell = lastCell[1], lastCell[0]
    #vertical line intersections, exclude 0th line
    for x in xrange(firstCell[0], lastCell[0], dx):
        x += dx
        gridInts[x] = xInc
    #horizontal line intersections, exclude 0th line
    for y in xrange(firstCell[1], lastCell[1], dy):
        if dy>0:
            y += dy
        x = (c-b*y)/a
        gridInts[x] = yInc
    return gridInts

def resetGrid():
    for line in canvas.track.lines:
        addToGrid(line)

def addToGrid(line):
    cells = getGridCells(line)
    if line.type == "scene":
        grid = canvas.grid.scenery
    else:
        grid = canvas.grid.solids
    for cell in cells:
        lines = grid.get(cell, set([]) )
        lines |= set([line])
        grid[cell] = lines
            
def removeFromGrid(line):
    """removes the line in the cells the line exists in"""
    removedCells = getGridCells(line) #list of cell positions
    if line.type == "scene":
        grid = canvas.grid.scenery
    else:
        grid = canvas.grid.solids
    for gPos in removedCells:
        cell = grid[gPos]
        #SET OF LINES, REMEMBER?
        cell.remove(line)
        if len(cell)==0: #get rid of the cell entirely if no lines
            grid.pop(gPos)
           
def gridNeighbors(pos):
    """returns a list of the positions of the cells at and around the pos"""
    cells = {}
    x, y = gridPos(pos)
    g = canvas.data.gridSize
    return [(x,y),(x+g,y),(x+g,y+g),(x,y+g),(x-g,y+g),
            (x-g,y),(x-g,y-g),(x,y-g),(x+g,y-g) ]

def gridInScreen():
    """returns a list of visible cells"""
    #absolute positions
    topLeft = gridPos(canvas.data.topLeft)
    bottomRight = gridPos(canvas.data.bottomRight)
    x1, x2 = topLeft[0], bottomRight[0]
    y1, y2 = topLeft[1], bottomRight[1]
    g = canvas.data.gridSize
    cols = xrange(x1, x2+g, g)
    rows = xrange(y1, y2+g, g)
    cells = [ (x,y) for x in cols for y in rows]
    return cells

###############################################################################
#                           ~ EDITING COMMANDS TO EDIT SH*T ~
###############################################################################
def undoCmd():
    if len(canvas.data.undoStack)==0:
        pass
    else:
        obj, command = canvas.data.undoStack.pop(-1) #last action
        command(obj, undo=True)

def redoCmd():
    if len(canvas.data.redoStack)==0:
        pass
    else:
        obj, command = canvas.data.redoStack.pop(-1)
        command(obj, redo=True)

def addLine(line, undo=False, redo=False):
    """adds a single line to the track"""
    if len(canvas.track.lines) == 0:
        canvas.track.startPoint = line.r1 - vector(0,30)
        makeRider()
    canvas.track.lines += [line]
    addToGrid(line)
    inverse = (line, removeLine)
    addToHistory(inverse, undo, redo)    
    

def removeLine(line, undo=False, redo=False):
    """removes a single line from the track"""
    canvas.track.lines.remove(line)
    removeFromGrid(line)
    inverse = (line, addLine)
    addToHistory(inverse, undo, redo)

def addToHistory(action, undo, redo):
    undoStack = canvas.data.undoStack
    redoStack = canvas.data.redoStack
    if undo: #call from undo, put in redo stack
        redoStack += [action]
    else: #else, put in undo stack
        undoStack += [action]
        if len(undoStack)>500: #it's up to the user to back up :|
            undoStack.pop(0)
    if not redo and not undo: #eg call from line tool
        redoStack = [] #clear redo stack
    trackModified()

def inversePZ(pnt):
    """turns relative position to absolute"""
    return (pnt-canvas.data.center)/canvas.track.zoom + canvas.data.cam

def inWindow(pos):
    return isInRegion(pos,vector(0,0), canvas.data.windowSize)

def pencil(event):
    pos = vector(event.x, event.y)
    if canvas.data.pause == True and inWindow(pos):
        pos = inversePZ(pos)
        if event.type == "4": #pressed
            canvas.data.tempPoint = pos
        elif event.type == "6" and canvas.data.tempPoint != None: #moved
            #to avoid making lines of 0 length
            minLen = tools.snapRadius/canvas.track.zoom
            if distance(canvas.data.tempPoint, pos) > minLen:
                lineType = tools.lineType
                line = solidLine(canvas.data.tempPoint, pos, lineType)
                addLine(line)
                canvas.data.tempPoint = pos
        elif event.type == "5":
            canvas.data.tempPoint = None


def makeLine(event):
    pos = vector(event.x, event.y)
    if canvas.data.pause == True and inWindow(pos):
        pos = inversePZ(pos)
        if tools.snap == True:
            pos = closestPointToLinePoint(pos)
        if event.type == "4": #pressed
            canvas.data.tempPoint = pos
        elif event.type == "5" and canvas.data.tempPoint != None: #released
            canvas.data.tempLine = None
            lineType = tools.lineType
            minLen = tools.snapRadius/canvas.track.zoom
            #to avoid making lines of 0 length
            if distance(canvas.data.tempPoint, pos) > minLen:
                line = solidLine(canvas.data.tempPoint, pos, lineType)
                addLine(line)
            canvas.data.tempPoint = None
        elif event.type == "6" and canvas.data.tempPoint != None: #moved
            canvas.data.tempLine = Line(canvas.data.tempPoint, pos)
   
def eraser(event):
    pos = vector(event.x, event.y)
    if canvas.data.pause == True and event.type != "5" and inWindow(pos):
        pos = inversePZ(pos)
        removedLines = removeLinesList(pos)
        if len(removedLines) > 0:
            for line in removedLines:
                if line in canvas.track.lines:
                    removeLine(line)

def closestPointToLinePoint(pos):
    """finds the closest endpoint of a line segment to a given point"""
    closestPoint = pos
    minDist = tools.snapRadius/canvas.track.zoom
    for line in linesInScreen():
        dist = distance(line.r1, pos)
        if dist < minDist:
            minDist = dist
            closestPoint = line.r1
        dist = distance(line.r2, pos)
        if dist < minDist:
            minDist = dist
            closestPoint = line.r2
    return closestPoint
            
def removeLinesList(pos):
    """returns a set of lines to be removed, part of the eraser"""
    z = canvas.track.zoom
    removedLines = set()
    radius = tools.eraserRadius
    cells = gridNeighbors(pos) #list of 9 closest cell positions
    for gPos in cells: #each cell has a position/key on the grid/dict
        cell = canvas.grid.solids.get(gPos, set() ) #each cell is a set of lines
        for line in cell:
            if distanceFromLine(pos, line)*z <= radius:
                removedLines |= set([line])
        cell = canvas.grid.scenery.get(gPos, set() )
        for line in cell:
            if distanceFromLine(pos, line)*z <= radius:
                removedLines |= set([line])
    return removedLines

def pan(event):
    if canvas.data.pause or not canvas.data.follow:
        z = canvas.track.zoom
        pos = (event.x, event.y)
        if event.type == "4": #pressed
            canvas.data.tempCam = pos
        elif event.type == "5": #released
            pass
        elif event.type == "6": #moved
            pan = vector(canvas.data.tempCam) - vector(pos)
            canvas.track.panPos += pan/z
            canvas.data.tempCam = pos

def zoom(event):
    if event.type=="4": #pressed
        tools.tempZoom = event.y
#        cor = inversePZ(vector(event.x, event.y))
#        print cor.x, cor.y
    elif event.type=="6": #moved
        delta = event.y - tools.tempZoom
        zoom = 0.99**(delta)
        zoom *= canvas.track.zoom
        if ( (0.1 < zoom and delta > 0) or
             (10 > zoom  and delta < 0) ):
                        canvas.track.zoom = zoom          
        tools.tempZoom = event.y

def zoomM(event):
    if (event.delta%120)==0:
        event.delta /= 120
    zoom = 1.1**(event.delta)
    zoom *= canvas.track.zoom
    if ( (0.1 < zoom and event.delta < 0) or
             (10 > zoom  and event.delta > 0) ):
        canvas.track.zoom = zoom   

def stop():
    canvas.data.pause = True
    canvas.data.slowmo = False
    resetRider()

def playPause(): #technically toggles between play and pause
    canvas.data.pause = not canvas.data.pause
    if canvas.data.pause and canvas.data.follow: #pausing
        canvas.track.panPos = canvas.rider.pos.r-canvas.data.center
    elif not canvas.data.pause:
        canvas.data.tempLine = None

def flag():
    canvas.data.flag = True
    canvas.data.flagBosh = copy.deepcopy(canvas.rider)
    #very tricky part here
#   canvas.data.flagBosh.accQueueNow = copy.copy(
#                canvas.data.flagBosh.accQueuePast)

def resetFlag():
    canvas.data.flag = False

def playFromBeginning():
    canvas.data.pause = False
    resetRider(True)

def resetRider(fromBeginning=False):
    canvas.data.tracer = []
    if fromBeginning or not canvas.data.flag:
        makeRider()
    else:
        canvas.rider = copy.deepcopy(canvas.data.flagBosh)
        
###############################################################################
#               ~ I'M TRYING MY BEST TO PREVENT LINE RIDER GRIEF ~
###############################################################################
def trackModified(isMdfy=True):
    canvas.data.modified = isMdfy
    showMdfy()

def showMdfy():
    if canvas.data.modified:
        canvas.data.message="*"
    else: canvas.data.message=""

def newTrack():
    if canvas.data.modified:
        if tkMessageBox.askokcancel(
            "Unsaved changes!", "Unsaved changes!\nContinue?"):
            init()
            resetRider()
    else:
        init()
        resetRider()
    
def loadTrack():
    window = Toplevel()
    window.title("Load Track")
    loadWindow = Listbox(window)
    loadWindow.pack()
    def exit():
        window.destroy()
    if os.path.isdir("savedLines"):
        def doLoad():
            loadWindow.delete(0, END)
            for track in os.listdir("savedLines"):
                loadWindow.insert(0, track)
            def load():
                name = loadWindow.get(ACTIVE)
                path = "savedLines/"+ name
                backupLines = canvas.track.lines
                backupStart = canvas.track.startPoint
                init()
                resetRider()
                with open(path, "rb") as track:
                    try:
                        canvas.track = pickle.load(track) #ACTUAL LOADING
                    except Exception as error: #in case it's not a valid file
                        canvas.track.lines = backupLines
                        canvas.track.startPoint = backupStart
                        loadWindow.delete(0, END)
                        loadWindow.insert(0, "Y U NO LOAD")
                        loadWindow.insert(END, "VALID TRACK")
                        print error
                        window.after(1000, doLoad)
                makeRider()
                resetGrid()
            cancelButton.config(text="Close")
            loadButton.config(text="Load", command=load)
        cancelButton = Button(window, text="Cancel", command=exit)
        loadButton = Button(window, text="Ok", command=doLoad)
        loadButton.pack()
        cancelButton.pack()
        if canvas.data.modified:
            loadWindow.insert(0, "Unsaved changes!")
            loadWindow.insert(END, "Continue?")
        else:
            doLoad()
    else:
        loadWindow.insert(0, "savedLines folder")
        loadWindow.insert(END, "does not exist!")
        cancelButton = Button(window, text="Okay ):", command=exit)
        loadWindow.pack()
    
def saveTrack():
    window = Toplevel()
    window.title("Save Track")
    saveWindow = Entry(window)
    saveWindow.insert(0, canvas.track.name)
    def exit():
        window.destroy()
    def saveAttempt():
        name = saveWindow.get()
        canvas.track.name = name
        path = "savedLines/"+ name
        def save():
            saveWindow.delete(0, END)
            try:
                with open(path, "wb") as track:
                    pickle.dump(canvas.track, track) #ACTUAL SAVING
                saveWindow.insert(0, "Saved!")
                trackModified(False)
                window.after(1000, exit)
            except Exception as error:
                saveWindow.insert(0, "Failed to save! D:")
                print error
                window.after(1000, lambda: undo(False))
        def undo(YN = True):
            saveWindow.delete(0, END)
            saveWindow.insert(0, name)
            loadButton.config(text="Save", command=saveAttempt)
            if YN:
                cancelButton.destroy()
        if not os.path.isdir("savedLines"):
            os.mkdir("savedLines")
        if os.path.isfile(path):
            saveWindow.delete(0, END)
            saveWindow.insert(0, "Overwrite track?")
            loadButton.config(text="Yes", command=save)
            cancelButton = Button(window, text="No", command=undo)
            cancelButton.pack()
        else:
            save()
    loadButton = Button(window, text="Save", command=saveAttempt)
    saveWindow.pack()
    loadButton.pack()

def fastSave():
    name = canvas.track.name
    if (name == "ONEXITSAVE_" or name == "Untitled"
        or not os.path.isdir("savedLines")):
        saveTrack() #save as
    else:
        path = "savedLines/"+name
        try:
            with open(path, "wb") as track:
                pickle.dump(canvas.track, track)
                trackModified()
            canvas.data.message = "Saved!"
        except Exception as error:
            canvas.data.message = "Failed to save! D:"
            print error
        canvas.after(1000, showMdfy)

def onExitSave(root):
    if not os.path.isdir("savedLines"):
        os.mkdir("savedLines")
    path = "savedLines/ONEXITSAVE_"
    canvas.track.name = "ONEXITSAVE_"
    try:
        with open(path, "wb") as track:
            pickle.dump(canvas.track, track)
    except Exception as error:
        print error
        if not canvas.data.modified or tkMessageBox.askokcancel(
            "Unsaved changes!","Failed to save! D:\nExit anyways?"):
            root.destroy()
    root.destroy()

def reloadOnExitSave():
    path = "savedLines/ONEXITSAVE_"
    if os.path.isfile(path):    
        with open(path, "r") as track:
            canvas.track = pickle.load(track)
        resetGrid()
###############################################################################
#                           ~ HOLY RUN ON, BATMAN ~
###############################################################################
def LmousePressed(event):
    tool = tools.leftTool
    tool(event)

def RmousePressed(event):
    tool = tools.rightTool
    tool(event)

def MmousePressed(event):
    zoom(event)
    redrawAll()

def setupWindow(root):
    
    root.title("Line Rider Python")
    def displayT(boolean, m1, m2):
        if boolean: message = m1
        else: message = m2
        canvas.data.message = "  "+message
        canvas.after(1000, showMdfy)
    menubar = Menu(root)
    root.config(menu=menubar)
#FILE
    fileMenu = Menu(menubar, tearoff=False)
    fileMenu.add_command(label="new (ctrl+n)", command = newTrack )
    fileMenu.add_command(label="load (ctrl+o)", command = loadTrack)
    fileMenu.add_command(label="save (ctrl+s)", command = saveTrack)
    menubar.add_cascade(label="File", menu=fileMenu)
#EDIT
    def undo():
        if canvas.data.pause: undoCmd()
    def redo():
        if canvas.data.pause: redoCmd()
    def snap():
        tools.snap = not tools.snap
        displayT(tools.snap,"Line snapping on","Line snapping off")
    editMenu = Menu(menubar, tearoff=False)
    editMenu.add_command(label="undo (ctrl+z)", command = undo)
    editMenu.add_command(label="redo (ctrl+shift+z)", command = redo)
    editMenu.add_command(label="toggle line snapping (s)", command = snap)
    menubar.add_cascade(label="Edit", menu=editMenu)
#TOOLS
    def setLTool(tool):
        tools.leftTool = tool
    def setRTool(tool):
        tools.rightTool = tool
    def setLine(lineType):
        tools.lineType = lineType
    toolMenu = Menu(menubar)
    toolMenu.add_command(label="pencil (q)", command=lambda:setLTool(pencil))
    toolMenu.add_command(label="line (w)", command= lambda: setLTool(makeLine))
    types = Menu(toolMenu)
    types.add_command(label="solid (1)", command= lambda:setLine("solid"))
    types.add_command(label="acceleration (2)", command=lambda:setLine("acc"))
    types.add_command(label="scenery (3)", command=lambda:setLine("scene"))
    toolMenu.add_cascade(label="line type", menu=types)
    toolMenu.add_command(label="eraser (e)", command= lambda: setLTool(eraser))
#    toolMenu.add_separator()
#    toolMenu.add_command(label="pan", command=setRTool(pan))
    menubar.add_cascade(label="Tools", menu=toolMenu)
#view
    def viewVector():
        canvas.data.viewVector = not canvas.data.viewVector
        displayT(canvas.data.viewVector,"Showing velocity","Hiding velocity")
    def viewPoints():
        canvas.data.viewPoints = not canvas.data.viewPoints
        displayT(canvas.data.viewPoints,"Showing snapping points",
                 "Hiding snapping points")
    def viewLines():
        canvas.data.viewLines = not canvas.data.viewLines
        displayT(canvas.data.viewLines, "Showing lines", "Hiding lines")
    def viewGrid():
        canvas.data.viewGrid = not canvas.data.viewGrid
        displayT(canvas.data.viewGrid, "Showing grid", "hiding grid")
    def viewStatus():
        canvas.data.viewStatus = not canvas.data.viewStatus
        displayT(canvas.data.viewStatus, "Showing status messages",
                 "Status messages are hidden, how can you see this?")
    def viewCollisions():
        canvas.data.viewCollisions = not canvas.data.viewCollisions
        displayT(canvas.data.viewCollisions, "Showing collision points",
                 "Hiding collision points")
    def goToStart():
        if canvas.data.pause:
            canvas.track.panPos = canvas.track.startPoint-canvas.data.center
    def lastLine():
        if canvas.data.pause and len(canvas.track.lines)>0:
            lastLine = canvas.track.lines[-1].r2
            canvas.track.panPos = lastLine - canvas.data.center
    def followRider():
        canvas.data.follow = not canvas.data.follow
        displayT(canvas.data.follow, "Following rider",  "Not following rider")
    def viewThinLines():
        canvas.data.viewThinLines = not canvas.data.viewThinLines
        displayT(canvas.data.viewThinLines, "Viewing normal lines",
                 "Viewing normal lines")
    viewMenu = Menu(menubar)
    viewMenu.add_command(label="velocity vectors (v)", command=viewVector)
    viewMenu.add_command(label="points (b)", command=viewPoints)
    viewMenu.add_command(label="collisions (c)", command=viewCollisions)
    viewMenu.add_command(label="thin lines", command = viewThinLines)
    viewMenu.add_command(label="grid", command=viewGrid)
    viewMenu.add_command(label="status", command=viewStatus)
    viewMenu.add_separator()
    viewMenu.add_command(label="starting point (home)", command=goToStart)
    viewMenu.add_command(label="last line (end)", command=lastLine)
    viewMenu.add_command(label="follow rider", command=followRider)
    menubar.add_cascade(label="View", menu=viewMenu)
#playback
    def slowmo():
        if not canvas.data.pause:
            canvas.data.slowmo = not canvas.data.slowmo
    playMenu = Menu(menubar)
    playMenu.add_command(label="play/pause (space/p)", command=playPause)
    playMenu.add_command(label="stop (space)", command=stop)
    playMenu.add_command(label="step (t)", command=updatePositions)
    playMenu.add_command(label="reset position (r)", command=resetRider)
    playMenu.add_command(label="flag (f)", command = flag)
    playMenu.add_command(label="reset flag (ctrl+f)", command = resetFlag)
    playMenu.add_command(label="play from beginning (ctrl+p)",
                         command=playFromBeginning)
    playMenu.add_command(label="slow-mo (m)", command=slowmo)
    menubar.add_cascade(label="Playback", menu=playMenu)
#help
    def viewHelp():
        canvas.data.help = not canvas.data.help
    helpMenu = Menu(menubar)
    helpMenu.add_command(label="help", command=viewHelp)
    helpMenu.add_command(label="about", command=doAbout)
    menubar.add_cascade(label="Help", menu=helpMenu)
#bindings
    root.bind("<Button-1>", LmousePressed)
    root.bind("<B1-Motion>", LmousePressed)
    root.bind("<ButtonRelease-1>", LmousePressed)
    root.bind("<Button-3>", RmousePressed)
    root.bind("<B3-Motion>", RmousePressed)
    root.bind("<ButtonRelease-3>", RmousePressed)
    root.bind("<Button-2>", MmousePressed)
    root.bind("<B2-Motion>", MmousePressed)
    root.bind("<MouseWheel>", zoomM)
    tools.ctrlPressed = False
    def keyPressed(event):
        k = event.keysym
        c = event.char
        if k=="Control_L" or k=="Control_R" or k=="Command_L" or k=="Command_R":
            tools.ctrlPressed = True
        if tools.ctrlPressed:
            return None
        elif c == "t":
            if canvas.data.pause: updatePositions()
        elif c == "p": playPause()
        elif c == " ":
            if canvas.data.pause: playPause()
            else: stop()
        elif c == "q": tools.leftTool = pencil
        elif c == "w": tools.leftTool = makeLine
        elif c == "e": tools.leftTool = eraser
        elif c == "m": slowmo()
        elif c == "r": resetRider()
        elif c == "f": flag()
        elif c == "v": viewVector()
        elif c == "b": viewPoints()
        elif c == "1": tools.lineType = "solid"
        elif c == "2": tools.lineType = "acc"
        elif c == "3": tools.lineType = "scene"
        elif c == "d": dump()
        elif c == "h": viewHelp()
        elif c == "s": snap()
        elif c == "c": viewCollisions()
        if canvas.data.help:
            i = canvas.data.helpIndex
            if k == "Left" and i > 0:
                canvas.data.helpIndex -= 1
            if k == "Right" and i < 7:
                canvas.data.helpIndex += 1
        redrawAll()
    def keyReleased(event):
        k = event.keysym
        if k=="Control_L" or k=="Control_R" or k=="Command_L" or k=="Command_R":
            tools.ctrlPressed = False
    root.bind("<KeyPress>", keyPressed)
    root.bind("<KeyRelease>", keyReleased)
    root.bind("<Home>", lambda e: goToStart())
    root.bind("<End>", lambda e: lastLine())
    if os.name == "mac":
        root.bind("<Command-z>", lambda e: undo())
        root.bind("<Command-Shift-Z>", lambda e: redo())
        root.bind("<Command-s>", lambda e: fastSave())
        root.bind("<Command-p>", lambda e: playFromBeginning())
        root.bind("<Command-o>", lambda e: loadTrack())
        root.bind("<Command-n>", lambda e: newTrack())
        root.bind("<Command-f>", lambda e: resetFlag())
    else:
        root.bind("<Control-z>", lambda e: undo())
        root.bind("<Control-Shift-Z>", lambda e: redo())
        root.bind("<Control-s>", lambda e: fastSave())
        root.bind("<Control-p>", lambda e: playFromBeginning())
        root.bind("<Control-o>", lambda e: loadTrack())
        root.bind("<Control-n>", lambda e: newTrack())
        root.bind("<Control-f>", lambda e: resetFlag())
    root.protocol("WM_DELETE_WINDOW", lambda: onExitSave(root))

###############################################################################
#                           ~ YE OLDE RUNTIME STUFF ~
###############################################################################
#data container
class Struct: pass

def run():
    root = Tk()
    global canvas
    canvas = Canvas(root, width=800, height=600, bg="white")
    canvas.data = Struct()
    canvas.track = Struct()
    canvas.rider = Struct()
    canvas.grid = Struct()
    setValues()
    setupWindow(root)
    canvas.pack(fill="both", expand=True)
    def resize(event):
        canvas.data.windowSize = vector(event.width, event.height)
        oldCenter = copy.copy(canvas.data.center)
        canvas.data.center = vector(event.width/2, event.height/2)
        delta = canvas.data.center - oldCenter
        canvas.track.panPos -= delta
    canvas.bind("<Configure>", resize)
    init()
    initRider()
    reloadOnExitSave()
    makeRider()
    timerFired()
    root.mainloop()

def setValues():
    canvas.data.timeDelta = delta = 10 #100 hz/fps
    canvas.data.grav = vector(0, 30.0/1000*delta) #pixels per frame**2
    canvas.data.drag = 0.9999999**delta
    canvas.data.acc = 0.1*delta #acceleration line constant
    canvas.data.epsilon = 0.00000000001 #larger than floating point errors
    canvas.data.lineThickness = 0.001
    canvas.data.endurance = 0.4
    canvas.data.gridSize = 50
    canvas.data.iterations = 10
    canvas.data.maxiter = 100
    canvas.data.viewLines = True
    canvas.data.viewVector = False
    canvas.data.viewPoints = False
    canvas.data.viewGrid = False
    canvas.data.viewStatus = True
    canvas.data.viewCollisions = False
    canvas.data.viewThinLines = False
    canvas.data.slowmo = False
    canvas.data.follow = True
    canvas.data.mouseEnable = True
    canvas.data.center = vector(400,300)
    canvas.data.topLeft = vector(0,0)
    canvas.data.bottomRight = vector(800,600)
    canvas.data.windowSize = vector(800,600)
    canvas.data.message = ""
    global tools
    tools = Struct()
    tools.lineType = "solid"
    tools.eraserRadius = 10
    tools.snapRadius = 10
    tools.leftTool = pencil
    tools.rightTool = pan
    tools.snap = True
    
        
def init():
    canvas.data.help = False
    canvas.data.helpIndex = 0
    tools.ctrlPressed = False
    canvas.track.lines = []
    canvas.track.zoom = 1
    canvas.track.panPos = vector(0,0)-canvas.data.center
    canvas.track.name = "Untitled"
    canvas.grid.solids = dict()
    canvas.grid.scenery = dict()
    canvas.data.undoStack = []
    canvas.data.redoStack = []
    canvas.track.startPoint = vector(0,0)
    canvas.data.tempPoint = vector(0,0)
    canvas.data.tempLine = None
    canvas.data.pause = True
    canvas.data.cam = canvas.track.panPos
    canvas.data.flag = False
    canvas.data.timeCurrent = time.time()
    canvas.data.modified = False
    canvas.data.collisionPoints = []
    canvas.data.tracer = []
    
def updateCamera():
    if canvas.data.pause or not canvas.data.follow:
        canvas.data.cam = canvas.track.panPos + canvas.data.center
    else:
        canvas.data.cam = canvas.rider.pos.r
    c = canvas.data.center
    z = canvas.track.zoom
    cam = canvas.data.cam
    canvas.data.topLeft = cam-c/z
    canvas.data.bottomRight = cam+c/z

def updateCursor():
    if not canvas.data.pause:
        cur = "arrow"
    elif tools.leftTool == pencil:
        cur = "pencil"
    elif tools.leftTool == makeLine:
        cur = "crosshair"
    elif tools.leftTool == eraser:
        cur = "circle"
    canvas.config(cursor = cur)
    
def timerFired():
    start = time.clock()
    if not canvas.data.pause:
        updatePositions()
    updateCamera()
    updateCursor()
    redrawAll()
    #an attempt to keep a constant 40 fps
    delay = int(1000*(time.clock() - start))
    delta = canvas.data.timeDelta - delay
    delta = max(1, delta)
    slow = max(4, 200-delay)
    if canvas.data.slowmo and not canvas.data.pause:
        delta = slow
    canvas.after(delta, timerFired) # WHAT THE HELL IS GOING ON HERE

def evalSpeed():
    totalVel = vector(0,0)
    point = canvas.rider.points[0]
    velocity = point.r - point.r0
    return velocity.magnitude()

def makeRider():
    sled = [Point(0,0),Point(0,10),Point(30,10),Point(35,0)]
    bosh = [Point(10,0),Point(10, -11),Point(23,-10),Point(23,-10),
            Point(20,10),Point(20,10)]
    scrf= [Point(7,-10),Point(3,-10),Point(0,-10),Point(-4,-10),
            Point(-7,-10),Point(-11,-10)]
    sledC =[cnstr(sled[0],sled[1]), cnstr(sled[1],sled[2]),
            cnstr(sled[2],sled[3]), cnstr(sled[3],sled[0]),
            cnstr(sled[0],sled[2]), cnstr(sled[1],sled[3])]
    boshC =[cnstr(bosh[0],bosh[1]), cnstr(bosh[1],bosh[2]),
            cnstr(bosh[1],bosh[3]), cnstr(bosh[0],bosh[4]),
            cnstr(bosh[0],bosh[5]) ]
    #sled+bosh = slsh :|
    slshC =[cnstr(sled[0],bosh[0]), cnstr(sled[1],bosh[0]),
            cnstr(sled[2],bosh[0]), cnstr(sled[0],bosh[1]),
            cnstr(sled[3],bosh[2]), cnstr(sled[3],bosh[3]),
            cnstr(sled[2],bosh[4]), cnstr(sled[2],bosh[5]) ]
    legsC =[cnstr(bosh[1],bosh[4],0.5), cnstr(bosh[1],bosh[5],0.5) ]
    scrfC =[cnstr(bosh[1],scrf[0]), cnstr(scrf[0],scrf[1]),
            cnstr(scrf[1],scrf[2]), cnstr(scrf[2],scrf[3]),
            cnstr(scrf[3],scrf[4]), cnstr(scrf[4],scrf[5]) ]
    startPoint = canvas.track.startPoint
    for point in sled+bosh+scrf:
        point.r += startPoint
        point.r0 += startPoint - vector(1,0)
    canvas.rider.points = bosh+sled
    canvas.rider.constraints = sledC+boshC+slshC
    canvas.rider.scarf = scrf
    canvas.rider.slshC = slshC
    canvas.rider.scarfCnstr = scrfC
    canvas.rider.legsC = legsC
    canvas.rider.pos = bosh[0]
    canvas.rider.accQueuePast = dict()
    canvas.rider.accQueueNow = dict()
    #parts = [arm1, leg1, sled, leg2, body, arm2]
    canvas.rider.boshParts = ((bosh[1],bosh[2]),(bosh[0],bosh[4]),
                              (sled[0],sled[3]),(bosh[0],bosh[5]),
                              (bosh[0],bosh[1]),(bosh[1],bosh[3]) )
    canvas.rider.sledString = ((bosh[2],sled[3]),(bosh[3],sled[3]))
    canvas.rider.onSled = True

def cnstr(pnt1, pnt2,scale=1):
    length = (pnt1.r-pnt2.r).magnitude()
    return Constraint(pnt1, pnt2, length*scale)
################################################################################
#                       ~ MINIMALISTIC HIPSTER GRAPHICS ~   
###############################################################################
def redrawAll():
    canvas.delete(ALL)
    if canvas.data.viewGrid:
        drawGrid()
    if canvas.data.viewLines:
        drawLines()
    if canvas.data.viewPoints:
        drawPoints()
    if canvas.data.flag:
        drawFlag()
#    drawTracer()
    drawRider()
    if canvas.data.viewVector:
        drawVectors()
    if canvas.data.viewCollisions:
        drawCollisions()
    if canvas.data.viewStatus:
        statusDisplay()
    if canvas.data.help:
        doHelp()

def drawTracer():
    tracer = canvas.data.tracer
    if len(tracer) > 1:
        lines = []
        for point in tracer:
            point = adjustPZ(point)
            lines += [(point.x, point.y)]
        canvas.create_line(lines, width=4, fill="cyan")

def statusDisplay():
    """displays fps"""
    timeBefore = canvas.data.timeCurrent
    canvas.data.timeCurrent = timeCurrent = time.time()
    duration = timeCurrent - timeBefore
    if duration != 0: fps = round(1/float(duration))
    else: fps = 0
    lineCount = len(canvas.track.lines)
    speed = ""
    if not canvas.data.pause: 
        speed = str(round(evalSpeed(), 1))+" pixels per frame"
    message = canvas.track.name
    if len(canvas.track.lines) == 0:
        message = "Press H for help"
    if canvas.data.help:
        message = "Press H to close"
    message += " "+canvas.data.message
    info = "%s\n%d frames per second\n%d lines\n%s" % (message,fps,lineCount,speed)
    canvas.create_text(5,0,anchor="nw",text=info)

def drawPoints():
    r = tools.snapRadius
#    for point in canvas.data.points:
#        pnt = adjustPZ(point.r)
#        x, y = pnt.x, pnt.y
#        if isInRegion((x,y), vector(-r,-r), canvas.data.center*2+vector(r,r)):
#            canvas.create_oval((x-r, y-r), (x+r, y+r))
    for line in canvas.track.lines:
        pnt = adjustPZ(line.r1)
        x, y = pnt.x, pnt.y
        canvas.create_oval((x-r, y-r), (x+r, y+r), outline="blue", width=3)
        pnt = adjustPZ(line.r2)
        x, y = pnt.x, pnt.y
        canvas.create_oval((x-r, y-r), (x+r, y+r), outline="blue", width=3)
        
def adjustPZ(pnt):
    return (pnt - canvas.data.cam) * canvas.track.zoom + canvas.data.center

def linesInScreen():
    lines = set()
    for gPos in gridInScreen():
        try: #a bit more efficient than using a conditional
            lines |= canvas.grid.solids[gPos]
        except KeyError:
            pass
        try:
            lines |= canvas.grid.scenery[gPos]
        except KeyError:
            pass
    return lines

def drawLines():
    z = canvas.track.zoom
    paused = canvas.data.pause
    w = 3*z
    if canvas.data.viewThinLines:
        w = 1
    for line in linesInScreen(): #RENDERS ONLY VISIBLE LINES
        a, b = adjustPZ(line.r1), adjustPZ(line.r2)
        color = "black"
        arrow=None
        if line.type == "scene" and paused:
            color = "green"
        if line.type == "acc" and paused:
            color = "red"
            arrow = LAST
        canvas.create_line(a.x, a.y, b.x, b.y, width=w,
                           caps=ROUND, fill=color, arrow=arrow)
    if canvas.rider.onSled:
        for line in canvas.rider.sledString:
            a, b = adjustPZ(line[0].r), adjustPZ(line[1].r)
            canvas.create_line(a.x, a.y, b.x, b.y)
    if canvas.data.tempLine != None and paused:
        line = canvas.data.tempLine
        a, b = adjustPZ(line.r1), adjustPZ(line.r2)
        minLen = tools.snapRadius
        if distance(a, b) < minLen:
            color = "red" #can't make this line
        else:
            color = "grey"
        canvas.create_line(a.x, a.y, b.x, b.y, fill=color)

def drawVectors():
#    for pnt in canvas.rider.points:
    pnt = canvas.rider.pos
    velocity = (pnt.r - pnt.r0)
    vel = velocity.normalize()*100
    a, b = adjustPZ(pnt.r), adjustPZ(pnt.r+vel)
    speed = velocity.magnitude()
    red = int(254*(1.02**(-speed)))+1
    blue = int(255*(1-(1.05**(-speed))))
    color = blue+(red<<16) # 0xrr00bb
    color = hex(color)[2:]
    if len(color)<6:
        color = "#0"+color
    else:
        color = "#"+color
    canvas.create_line(a.x, a.y, b.x, b.y, width = 3, arrow=LAST,
                       fill = color)

def drawVectorsPrecise():
    for pnt in canvas.rider.points:
        velocity = (pnt.r - pnt.r0)
        a, b = adjustPZ(pnt.r), adjustPZ(pnt.r+velocity)
        canvas.create_line(a.x, a.y, b.x, b.y, width = 1,
                           fill = "blue")

def drawCollisions():
    for point in canvas.rider.points:
        pnt = adjustPZ(point.r)
        canvas.create_oval(pnt.x+2, pnt.y+2, pnt.x-2, pnt.y-2, fill="red")
    for pnt in canvas.data.collisionPoints:
        pnt = adjustPZ(pnt)
        canvas.create_oval(pnt.x+2, pnt.y+2, pnt.x-2, pnt.y-2, fill="yellow", width=0)

def showGrid():
    window = canvas.data.windowSize
    topLeft, bottomRight = canvas.data.topLeft, canvas.data.bottomRight
    g = canvas.data.gridSize
    topLeft = gridPos(topLeft)
    bottomRight = gridPos(bottomRight+vector(g,g))
    for x in xrange(topLeft[0], bottomRight[0], g):
        a, b = vector(x, topLeft[1]), vector(x, bottomRight[1])
        a, b = adjustPZ(a), adjustPZ(b)
        canvas.create_line(a.x, a.y, b.x, b.y)
    for y in xrange(topLeft[1], bottomRight[1], g):
        a, b = vector(topLeft[0], y), vector(bottomRight[0], y)
        a, b = adjustPZ(a), adjustPZ(b)
        canvas.create_line(a.x, a.y, b.x, b.y)

def drawGrid():
    """for debugging"""
    g = canvas.data.gridSize
    showGrid()
    for cell in canvas.grid.solids: #cells are positions/keys in the grid dict
        cell = vector(cell)
        cell2 = cell + vector(g, g)
        cell, cell2 = adjustPZ(cell), adjustPZ(cell2)
        canvas.create_rectangle(cell.x, cell.y, cell2.x, cell2.y, fill="yellow")
    for point in canvas.rider.points:
        velocity = point.r - point.r0
        velLine = Line(point.r, point.r+velocity)
        cells = getGridCells(velLine)
        for cell in cells:
            if cell in canvas.grid.solids:
                color = "green"
            else: color = "cyan"
            cell = vector(cell)
            cell2 = cell + vector(g, g)
            cell, cell2 = adjustPZ(cell), adjustPZ(cell2)
            canvas.create_rectangle(cell.x, cell.y, cell2.x, cell2.y, fill=color)

def drawScarf(c):
    color = c
    w = 4*canvas.track.zoom
    for line in canvas.rider.scarfCnstr:
        if color == c:
            color = "white"
        else: color = c
        pnt1, pnt2 = adjustPZ(line.pnt1.r), adjustPZ(line.pnt2.r)
        canvas.create_line( [ (pnt1.x, pnt1.y),(pnt2.x,pnt2.y) ],
                          width=w, fill=color, capstyle=BUTT)

def drawRider():
    drawScarf("red")
    parts = canvas.rider.parts
    bosh = canvas.rider.boshParts
    for i in xrange(len(parts)):
        part = parts[i] #part contains tuples of line segments and stuff
        point0, point1 = bosh[i] #each value has two Point objects
        angle = (point1.r-point0.r).getAngle()
        for shape in part: #segment: tuple of line properties for rendering
            shape.render(point0.r, angle)

def drawFlag():
    parts = canvas.data.flagParts
    bosh = canvas.data.flagBosh.boshParts
    for i in xrange(len(parts)):
        part = parts[i] #part contains tuples of line segments and stuff
        point0, point1 = bosh[i] #each value has two Point objects
        angle = (point1.r-point0.r).getAngle()
        for shape in part: #segment: tuple of line properties for rendering
            shape.render(point0.r, angle)

class Shape(object):
    # [type, [coords], fill=fColor, outline=oColor, width=w, special
    #special: (r, start, extent) or (smooth, cap)]    cors = copy.copy(sgmnt[0])
    def __init__(self, cors, fillColor, lineColor, width):
#        cors = copy.copy(coords)
        for i in xrange(len(cors)):
            cors[i] = vector(cors[i])*0.25 #scale
        self.cors = cors
        self.fillColor = fillColor
        self.lineColor = lineColor
        self.width = width

class LineShape(Shape):
    def __init__(self ,cors, fillColor=None, lineColor="black", width=1,
                 smooth=False, cap=ROUND):
        super(LineShape, self).__init__(cors, fillColor, lineColor, width)
        self.isSmooth = smooth
        self.capstyle = cap
    def render(self, pnt0, angle):
        w = self.width
        if w != 1:
            w *= canvas.track.zoom*0.25
        cors = []
        for i in xrange(len(self.cors)):
            cor = self.cors[i].rotate(angle)    
            cor = adjustPZ(cor+pnt0)
            cors += (cor.x, cor.y) #convert to tuples, put into list
        canvas.create_line(cors, fill=self.lineColor, width=w,
                           joinstyle=MITER, capstyle=self.capstyle)

class Polygon(Shape):
    def __init__(self, cors, fillColor="white", lineColor="black", width=1,
                 smooth=False):
        super(Polygon, self).__init__(cors, fillColor, lineColor, width)
        self.isSmooth = smooth
    def render(self, pnt0, angle):
        w = self.width
        if w != 1:
            w *= canvas.track.zoom*0.25
        cors = []
        for i in xrange(len(self.cors)):
            cor = self.cors[i].rotate(angle)    
            cor = adjustPZ(cor+pnt0)
            cors += (cor.x, cor.y) #convert to tuples, put into list
        canvas.create_polygon(cors, fill=self.fillColor, outline=self.lineColor,
                              smooth=self.isSmooth, width=w)

class Arc(Shape): #also pieslice
    def __init__(self,cors, fillColor="white", lineColor="black",width=1,
                 theta=(1, 0, 90)):
        super(Arc, self).__init__(cors, fillColor, lineColor, width)
        self.center = self.cors[0]
        self.radius = theta[0]*0.25
        self.start = theta[1]
        self.extent = theta[2]
    def render(self, pnt0, angle):
        center = self.center.rotate(angle)
        center = adjustPZ(center+pnt0)
        x,y = center.x, center.y
        w = self.width
        if w != 1:
            w *= canvas.track.zoom*0.25
        angle = math.degrees(angle)
        strt = self.start - angle
        r = self.radius*canvas.track.zoom
        if self.fillColor == None:
            canvas.create_arc(x+r,y+r,x-r,y-r, style=ARC,
                              start=strt, extent=self.extent,
                              width=w, outline=self.lineColor)
        else:
            canvas.create_arc(x+r,y+r,x-r,y-r, style=PIESLICE, width=w,
                              start=strt, extent=self.extent,
                              fill=self.fillColor, outline=self.lineColor)

class Circle(Shape):
    def __init__(self, cors, fillColor="white", lineColor="black", width=1,
                 radius=1):
        super(Circle, self).__init__(cors, fillColor, lineColor, width)
        self.center = self.cors[0]
        self.radius = radius*0.25
    def render(self, pnt0, angle):
        center = self.center.rotate(angle)
        center = adjustPZ(center+pnt0)
        x,y = center.x, center.y
        w = self.width
        if w != 1:
            w *= canvas.track.zoom*0.25
        r = self.radius*canvas.track.zoom
        canvas.create_oval(x+r,y+r,x-r,y-r, width=w,
                           fill=self.fillColor, outline=self.lineColor)

def initRider():
    """loads the vector graphics of the rider into memory"""
    s = 0.25 #scale down
    sled =[
        #base structure
        LineShape( [ (0,0),(94.4,0) ] ,width=6, lineColor="#aaa") ,
        LineShape( [ (-2,38.2),(108.4,38.2) ] ,width=6, lineColor="#aaa") ,
        LineShape( [ (16.6,3),(16.6,35.2) ] ,width=6, lineColor="#aaa") ,
        LineShape( [ (75,3),(75,35.2) ] ,width=6, lineColor="#aaa") ,
        Arc( [ (108.4,11.6) ], theta=(26.7,-90,260),
             width=6, lineColor="#aaa", fillColor=None)

        #outline
        ]
    body = [
        #face
        Polygon( [(54,-17.4),(54,19),(60.6,28.4),(86,18.6),(80.8,-17.4)],
                 smooth=True) ,
        Circle( [ (68,12) ], fillColor="black", radius=3.2) ,
        #torso
        Polygon( [ (0,-17.4),(56,-17.4),(56,17.8),(0,17.8) ] ) ,
        #hat
        Arc( [ (80.8,0) ], theta=(20.2, -90, 180) ) ,
        Circle( [ (106.8,0) ], fillColor="black",radius =5.8) ,
        LineShape( [ (80.8,21.2),(80.8,-21.2)], width=6) ,
        Polygon( [ (56,-19.4),(56,-1.6),(80.8,-1.6),(80.8,-19.4) ],
                 fillColor="black") ,
        #scarf
        LineShape( [(49.2,-20),(49.2,-12)], lineColor="red",width=16, cap=BUTT),
        LineShape( [(49.2,-12),(49.2,-4)], lineColor="white",width=16,cap=BUTT),
        LineShape( [(49.2,-4),(49.2,4)], lineColor="red",width=16, cap=BUTT),
        LineShape( [(49.2,4),(49.2,12)], lineColor="white",width=16, cap=BUTT),
        LineShape( [(49.2,12),(49.2,20)], lineColor="red",width=16, cap=BUTT)
        ]
    arm1 = [
        LineShape( [ (0,0),(40,0) ] ,width=10.8) ,
        Polygon( [ (40,-5.4),(44,-5.4),(46.4,-9.2),(48.4,-9.6),(49,-8.8),
                   (48.4,-5.2),(52.4,-5.4),(55.2,-4.4),(56.6,-2.4),(56.6,2.4),
                   (55.2,4.4),(52.4,5.4),(40,5.4) ] )
        ]
    leg1 = [
        LineShape( [ (0,0),(38.2,0) ], width=10.8) ,
        Polygon( [ (38.2,-5.4),(47.2,-5.4),(53,-15.8),(57.4,-15.4),
                   (57.4,5.6),(38.2,5.4) ] )
        ]
    arm2 = copy.deepcopy(arm1)
    leg2 = copy.deepcopy(leg1)
    parts = [arm1, leg1, sled, leg2, body, arm2]
    canvas.rider.parts = parts #vector graphics
    initFlag()

def initFlag():
    parts = copy.deepcopy(canvas.rider.parts)
    for part in parts:
        for shape in part:
            if shape.lineColor == "red":
                shape.lineColor = "#eee"
            elif shape.lineColor == "#aaa":
                shape.lineColor = "#eee"
            elif shape.lineColor == "black":
                shape.lineColor = "#ddd"
            if shape.fillColor == "black":
                shape.fillColor = "#ddd"
    canvas.data.flagParts = parts
################################################################################
#               ~ okay I'm out of weird and witty and capslock ~  
################################################################################
def dump():
    print canvas.track.lines     

def doHelp():
    center = canvas.data.center
    TL = center - vector(300,200)
    BR = center + vector(300,200)
    canvas.create_rectangle(TL.x, TL.y, BR.x, BR.y, width=5, fill="#eee")
    def title(text):
        canvas.create_text(TL.x+10, TL.y+5, anchor=NW, width = 580,
                           font=("Arial","25", "bold"), text=text)
    def contents(x, y, w, text):
        canvas.create_text(TL.x+x, TL.y+y, anchor=NW, width=w,
                           font=("Arial","15"), text=text)
    def toPlay():
        title("How to play Line Rider Python")
        contents(10, 60, 580, """Draw a line from the top left to the bottom \
right with the left mouse button. Press the space bar to play and watch the \
rider go. Press space again to stop and go back to editing. (paraphrased from \
the original version of Line Rider)

That was interesting. Was it? If you want to do more, use the left and \
right arrow keys to navigate the help contents to see how to do more.

Note: If you open a menu and click on the dashed lines, you "tear off" \
the menu to make a separate window for the menu.

Note2: \
If you're on a mac, the menus are at the top of your screen, not this window.
""")
    def tools(): #and edit
        title("Editing tracks")
        contents(10, 60, 580, """Left mouse button: Draw/erase lines.
Middle mouse button: Zoom in and out of the track.
Right mouse button: Pan around the track.

In the Edit menu:
Undo and redo: Undoes and redoes your last action.
Toggle line snapping: Toggles the lines you draw to snap to the endpoints of \
other lines.

In the Tools menu:
Pencil: For free-hand drawing
Line: Draws straight lines
Eraser: Removes lines from the track
""")
    def lineTypes():
        title("Types of lines")
        contents(10, 60, 580, """Solid line: An ordinary line. However, unlike \
the lines in the original Line Rider, the rider collides with BOTH SIDES of \
the line.

Acceleration line: A solid line that accelerates the rider in the direction of \
the arrow.

Scenery line: The rider does not collide with this line.

Spring line: Not implemented yet.

Trigger line: Not implemented yet.
""")
    def playback():
        title("Playing tracks")
        contents(10, 60, 580, """Play: Starts the track.
Stop: Stops the track. You can alternate between playing and stopping by \
pressing just the space bar.
Pause: Pauses the track. You can alternate between playing and pausing by \
pressing just the "p" button.
Step: Plays one frame of the track, for precise track making.
Reset position: resets the position of the rider to the start point.
Flag: Saves the current position and velocity of the rider. This causes "Play" \
to start the track from this point, and "Reset position" to reset to this point.
Reset flag: Removes the flag.
Play from beginning: Plays the track from the beginning, ignoring the flag.
Slow-mo: Toggles playing the track in slow motion.
""")
    def view():
        title("View options")
        contents(10, 60, 580, """Velocity vector: Shows the vector describing \
the rider's speed and direction of motion. Red is slow, blue is fast.
Points: Shows the endpoints of the lines in the track. The radius of the \
circle cooresponds with the radius of line snapping.
Thin lines: Turns the width of lines to be as thin as possible, in case you \
need precision.
Status: Shows/hides the information on the top-left corner of the window.
Starting point: Moves the camera to the starting point
Last line: Moves the camera to the end of the last line drawn.
Follow rider: Toggles whether the camera is following the rider in playback.
Grid and collisions: Debugging stuff. Don't worry about it :|
""")
    def saving():
        title("Saving and loading")
        contents(10, 60, 580, """New track: Erases the current track.
Save track: Saves the current track with a given name to /savedLines.
Load track: Lets you load a track from /savedLines.

/SavedLines is a folder in the same place as where this .py file is.
When you exit, the current track is automatically saved to a file called \
"ONEXITSAVE_".
When you open this program again, it automatically loads "ONEXITSAVE_".
""")
    def tips1():
        title("Tips on making tracks")
        contents(10, 60, 580, """If you didn't already find out, the rider \
will fall off his sled if you hit him hard enough. However, he'll also fall \
off if a line forces him off the seat of his sled. Yes, he will fall off if a \
line touches his butt. HE'S VERY SENSITIVE THERE :|

If, for whatever reason, you want to drive \
the rider into a sharp angle, that will cause this program to lag (not crash). \
If you really want to do it, I advise you to add in a small perpendicular line \
at the point of intersection.
""")
        lines =[
            ((174.0, 275.0),(491.0, 317.0)), ((491.0, 317.0),(188.0, 369.0)),
            ((480.0, 300.0),(480.0, 340.0)), ( (43.0, 324.0),(169.0, 328.0)),
            ((169.0, 328.0),(133.0, 311.0)), ((169.0, 328.0),(134.0, 351.0))]
        for line in lines:
            a, b = vector(line[0])+TL, vector(line[1])+TL
            canvas.create_line(a.x, a.y, b.x, b.y, width=3, cap=ROUND)
    def tips2():
        title("Tips on making tracks")
        contents(10, 60, 580, """If you notice you're spending a lot of time \
on a single track, it would be a good idea to make back up files just in case \
something bad happens (eg this program somehow corrupts the save or you mess \
up the track and can't fix it)

On drawing smooth curves:
Lazy way                  Lazy but effective          Pro (takes time)
\n\n\n\n\n
Don't forget to have fun! ~Conundrumer
""")
        lines = [
            ((220.0, 238.0),(230.0, 292.0)), ((219.0, 260.0),(247.0, 312.0)),
            ((230.0, 292.0),(270.0, 331.0)), ((247.0, 312.0),(297.0, 343.0)),
            ((270.0, 331.0),(335.0, 352.0)), ((297.0, 343.0),(370.0, 354.0)),
            ((432.0, 239.0),(435.0, 260.0)), ((435.0, 260.0),(443.0, 283.0)),
            ((443.0, 283.0),(456.0, 304.0)), ((456.0, 304.0),(475.0, 324.0)),
            ((475.0, 324.0),(497.0, 337.0)), ((497.0, 337.0),(526.0, 346.0)),
            ((526.0, 346.0),(554.0, 350.0)), ((554.0, 350.0),(585.0, 351.0)),
            ( (16.0, 230.0), (19.0, 346.0)), ( (19.0, 346.0),(163.0, 347.0)),
            ( (16.0, 230.0), (34.0, 346.0)), ( (18.0, 260.0), (56.0, 347.0)),
            ( (18.0, 278.0), (76.0, 346.0)), ( (20.0, 297.0), (98.0, 346.0)),
            ( (20.0, 314.0),(123.0, 348.0)), ( (21.0, 329.0),(147.0, 346.0)) ]
        for line in lines:
            a, b = vector(line[0])+TL, vector(line[1])+TL
            canvas.create_line(a.x, a.y, b.x, b.y, width=1, cap=ROUND)
    helpContents = [toPlay,tools,lineTypes,playback,view,saving,tips1,tips2]
    i = canvas.data.helpIndex
    helpContents[i]()
    canvas.create_text(BR.x-10, TL.y+5, anchor=NE, 
                        font=("Arial","15"), text=str(i+1)+"/8")
def doAbout():
    tkMessageBox.showinfo("About", """Line Rider Python v1.4
Created by David Lu (Conundrumer)
Something about fair use for education purposes here...

Line Rider originally created by Bostjan Cadez (fsk)

Collision detection adapted from:
www.t3hprogrammer.com/research/line-circle-collision/tutorial

Constraint algorithm adapted from:
www.gpgstudy.com/gpgiki/GDC 2001%3A Advanced Character Physics

Thanks to:
Matthew (mhenr18)
My brother
Everyone else at www.weridethelines.com

Line Rider Python will be discontinued after
spring lines and trigger lines are implemented""")
###############################################################################
# DUN DUN DUN
run()
###############################################################################
"""
logs:
before all of this: played too much Line Rider
10/19: worked out the math behind a physics simulator with no collision
10/22: finalized plan to make physics simulator with Kiraz
10/26: found a tutorial on simple circle-line collisions
11/6: completed physics with gravity and drag + simple display
11/7: wrote logs/todo
11/8: partially completed collision
11/15: rewrote classes
11/16: debugged classes, rewrote PLcollision, and got it working well enough :|
11/18: attempted rigid body collisions
11/20: continued attempt but gave up
11/21: used different algorithm, RIGID BODY COLLISIONS WORK HOLY CRAP
11/23: panning, camera following moving point, drawing lines, playback
11/25: saving/loading+zoom
11/27: snapping, some GUI improvements
11/28: more GUI stuff
11/29: a bit more organizing
11/30: GRID WORKING, SHOULD BE FASTER, more GUI stuff, segment drawing function
12/1: I do GUI when I'm bored :|
12/2: drew bosh. DARN IT LIMITED VECTOR GRAPHICS
12/3: implemented flags and acc/scenery lines
12/4: bug fixes via BHCS and mhenr18, also BULLETPROOFED THESE LINES F YEA
12/5: acc lines now consistent, tweaked line tool hotkey, view grid less laggy
      vectors and collision viewing now prettier, tweaked flag
12/6: added pencil tool, sensitive butt, help menu, other gui stuff I forgot,
      oh yea, redid start point adjusting
12/6 wait, what?: fixed an awful bug with the flag tool

ToDo:
make moving background lines collidable
augment physics (spring,trigger)
advance GUI/editing/sim:
    - xy snap
    - continuous tracer
    - make starting point adjustable (position and velocity+lock)
    - shortcuts (zoomoutall)
    - title screen
    - scenery fill (with stipling) (make polygon detecting algorithm)
    - invisible/white lines (outline)
    - filter (view only x), select (select only x), move lines and adjust points
    - save points
    - AUTOSAVE
#not gonna bother with the below
PORTALS (intra-track, inter-track)
track scrubbing (rewind/fastforward)
export screenshot of entire track
loading previews
add music sync
export videos
make track
implement multiplayer? I should just move on to java for this...
"""
