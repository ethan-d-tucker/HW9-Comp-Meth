import math
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg


class Position():
    """
    I made this position for holding a position in 3D space (i.e., a point).  I've given it some ability to do
    vector arithmitic and vector algebra (i.e., a dot product).  I could have used a numpy array, but I wanted
    to create my own.  This class uses operator overloading as explained in the class.
    """

    def __init__(self, pos=None, x=None, y=None, z=None):
        """
        x, y, and z have the expected meanings
        :param pos: a tuple (x,y,z)
        :param x: float
        :param y: float
        :param z: float
        """
        # set default values
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # unpack position from a tuple if given
        if pos is not None:
            self.x, self.y, self.z = pos
        # override the x,y,z defaults if they are given as arguments
        self.x = x if x is not None else self.x
        self.y = y if y is not None else self.y
        self.z = z if z is not None else self.z

    # region operator overloads $NEW$ 4/7/21
    def __eq__(self, other):
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        return True

    # this is overloading the addition operator.  Allows me to add Position objects with simple math: c=a+b, where
    # a, b, and c are all position objects.
    def __add__(self, other):
        return Position((self.x + other.x, self.y + other.y, self.z + other.z))

    # this overloads the iterative add operator
    def __iadd__(self, other):
        if other in (float, int):
            self.x += other
            self.y += other
            self.z += other
            return self
        if type(other) == Position:
            self.x += other.x
            self.y += other.y
            self.z += other.z
            return self

    # this is overloading the subtraction operator.  Allows me to subtract Positions. (i.e., c=b-a)
    def __sub__(self, other):
        return Position((self.x - other.x, self.y - other.y, self.z - other.z))

    # this overloads the iterative subtraction operator
    def __isub__(self, other):
        if other in (float, int):
            self.x -= other
            self.y -= other
            self.z -= other
            return self
        if type(other) == Position:
            self.x -= other.x
            self.y -= other.y
            self.z -= other.z
            return self

    # this is overloading the multiply operator.  Allows me to multiply a scalar or do a dot product (i.e., b=s*a or c=b*a)
    def __mul__(self, other):
        if type(other) in (float, int):
            return Position((self.x * other, self.y * other, self.z * other))
        if type(other) is Position:
            return Position((self.x * other.x, self.y * other.y, self.z * other.z))

    # this is overloading the __rmul__ operator so that s*Pt works.
    def __rmul__(self, other):
        return self * other

    # this is overloading the *= operator.  Same as a = Position((a.x*other, a.y*other, a.z*other))
    def __imul__(self, other):
        if type(other) in (float, int):
            self.x *= other
            self.y *= other
            self.z *= other
            return self

    # this is overloading the division operator.  Allows me to divide by a scalar (i.e., b=a/s)
    def __truediv__(self, other):
        if type(other) in (float, int):
            return Position((self.x / other, self.y / other, self.z / other))

    # this is overloading the /= operator.  Same as a = Position((a.x/other, a.y/other, a.z/other))
    def __idiv__(self, other):
        if type(other) in (float, int):
            self.x /= other
            self.y /= other
            self.z /= other
            return self

    # endregion

    def set(self, strXYZ=None, tupXYZ=None):
        # set position by string or tuple
        if strXYZ is not None:
            cells = strXYZ.replace('(', '').replace(')', '').strip().split(',')
            x, y, z = float(cells[0]), float(cells[1]), float(cells[2])
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
        elif tupXYZ is not None:
            x, y, z = tupXYZ  # [0], strXYZ[1],strXYZ[2]
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    def getTup(self):  # return (x,y,z) as a tuple
        return (self.x, self.y, self.z)

    def getStr(self, nPlaces=3):
        return '{}, {}, {}'.format(round(self.x, nPlaces), round(self.y, nPlaces), round(self.z, nPlaces))

    def mag(self):  # normal way to calculate magnitude of a vector
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5

    def normalize(self):  # typical way to normalize to a unit vector
        l = self.mag()
        if l <= 0.0:
            return
        self.__idiv__(l)

    def getAngleRad(self):
        """
        Gets angle of position relative to an origin (0,0) in the x-y plane
        :return: angle in x-y plane in radians
        """
        l = self.mag()
        if l <= 0.0:
            return 0
        if self.y >= 0.0:
            return math.acos(self.x / l)
        return 2.0 * math.pi - math.acos(self.x / l)

    def getAngleDeg(self):
        """
        Gets angle of position relative to an origin (0,0) in the x-y plane
        :return: angle in x-y plane in degrees
        """
        return 180.0 / math.pi * self.getAngleRad()


class Material():
    def __init__(self, uts=None, ys=None, modulus=None, staticFactor=None):
        self.uts = uts
        self.ys = ys
        self.E = modulus
        self.staticFactor = staticFactor


class Node():
    def __init__(self, name=None, position=None, is_pivot=False):
        self.name = name
        self.position = position if position is not None else Position()
        self.is_pivot = is_pivot  # flag to indicate pivot nodes

    def __eq__(self, other):
        """
        This overloads the == operator such that I can compare two nodes to see if they are the same node.  This is
        useful when reading in nodes to make sure I don't get duplicate nodes
        """
        if self.name != other.name:
            return False
        if self.position != other.position:
            return False
        return True

    def set_pivot_points(truss):
        min_y = min(node.position.y for node in truss.nodes)
        for node in truss.nodes:
            if node.position.y == min_y:
                node.is_pivot = True


class Link():
    def __init__(self, name="", node1="1", node2="2", length=None, angleRad=None):
        """
        Basic definition of a link contains a name and names of node1 and node2
        """
        self.name = ""
        self.node1_Name = node1
        self.node2_Name = node2
        self.length = None
        self.angleRad = None

    def __eq__(self, other):
        """
        This overloads the == operator for comparing equivalence of two links.
        """
        if self.node1_Name != other.node1_Name: return False
        if self.node2_Name != other.node2_Name: return False
        if self.length != other.length: return False
        if self.angleRad != other.angleRad: return False
        return True

    def set(self, node1=None, node2=None, length=None, angleRad=None):
        self.node1_Name = node1
        self.node2_Name = node2
        self.length = length
        self.angleRad = angleRad


class TrussModel():
    def __init__(self):
        self.title = None
        self.links = []
        self.nodes = []
        self.material = Material()

    def getNode(self, name):
        for n in self.nodes:
            if n.name == name:
                return n


class TrussController():
    def __init__(self):
        self.truss = TrussModel()
        self.view = TrussView()

    def ImportFromFile(self, data):

        self.clearCurrentTruss()

        reading_material = False
        reading_static_factor = False
        reading_nodes = False
        reading_links = False
        for line in data:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            if line.lower().startswith('title'):
                self.truss.title = line.split(',')[1].strip().strip("'")
            elif line.lower().startswith('material'):
                values = line.split(',')[1:]
                self.truss.material = Material(uts=float(values[0].strip()), ys=float(values[1].strip()),
                                               modulus=float(values[2].strip()) * 1e6)
            elif line.lower().startswith('static_factor'):
                self.truss.material.staticFactor = float(line.split(',')[1].strip())
            elif line.lower().startswith('node'):
                parts = line.split(',')
                node = Node(name=parts[1].strip(),
                            position=Position(x=float(parts[2].strip()), y=float(parts[3].strip())))
                if not self.hasNode(node.name):
                    self.addNode(node)
            elif line.lower().startswith('link'):
                parts = line.split(',')
                link = Link(name=parts[1].strip(), node1=parts[2].strip(), node2=parts[3].strip())
                self.addLink(link)

        self.identifyPivotNodes()
        self.calcLinkVals()
        self.displayReport()
        self.drawTruss()

    def clearCurrentTruss(self):
        # Clear the model data
        self.truss.links.clear()
        self.truss.nodes.clear()

        # Clear the view/scene
        self.view.scene.clear()
        self.view.scene.update()

    def identifyPivotNodes(self):
        # Find the lowest y-coordinate among all nodes
        lowest_y = min(node.position.y for node in self.truss.nodes)
        # Identify the pivot nodes
        for node in self.truss.nodes:
            if node.position.y == lowest_y:
                node.is_pivot = True

    def hasNode(self, name):
        for n in self.truss.nodes:
            if n.name == name:
                return True
        return False

    def addNode(self, node):
        self.truss.nodes.append(node)

    def getNode(self, name):
        for n in self.truss.nodes:
            if n.name == name:
                return n

    def addLink(self, link):
        self.truss.links.append(link)

    def calcLinkVals(self):
        for l in self.truss.links:
            n1 = None
            n2 = None
            if self.hasNode(l.node1_Name):
                n1 = self.getNode(l.node1_Name)
            if self.hasNode(l.node2_Name):
                n2 = self.getNode(l.node2_Name)
            if n1 is not None and n2 is not None:
                r = n2.position - n1.position
                l.length = r.mag()
                l.angleRad = r.getAngleRad()

    def setDisplayWidgets(self, args):
        self.view.setDisplayWidgets(args)

    def displayReport(self):
        self.view.displayReport(truss=self.truss)

    def drawTruss(self):
        self.view.buildScene(truss=self.truss)

class RigidLink(qtw.QGraphicsItem):
    def __init__(self, stX, stY, enX, enY, radius=10, parent = None, pen=None, brush=None):
        """
        This is a custom class for drawing a rigid link.  The paint function executes everytime the scene
        which holds the link is updated.  The steps to making the link are:
        1. Specify the start and end x,y coordinates of the link
        2. Specify the radius (i.e., the width of the link)
        3. Compute the length of the link
        3. Compute the angle of the link relative to the x-axis
        4. Compute the angle normal the angle of the length by adding pi/2
        5. Compute the rectangle that will contain the link (i.e., its bounding box)
        These steps are executed each time the paint function is invoked
        :param stX:
        :param stY:
        :param enX:
        :param enY:
        :param radius:
        :param parent:
        :param pen:
        :param brush:
        """
        super().__init__(parent)
        #step 1
        self.startX = stX
        self.startY = stY
        self.endX = enX
        self.endY = enY
        #step 2
        self.radius = radius
        #step 3
        self.length=self.linkLength()
        #step 4
        self.angle = self.linkAngle()
        #step 5
        self.normAngle = self.angle+math.pi/2
        #step 6
        self.width=self.endX-self.startX+2*self.radius
        self.height=self.endY-self.startY+2*self.radius
        self.rect=qtc.QRectF(self.startX, self.startY, self.width, self.height)

        self.pen=pen
        self.brush=brush

    def boundingRect(self):
        return self.rect

    def linkLength(self):
        self.length = math.sqrt(math.pow(self.startX - self.endX, 2) + math.pow(self.startY - self.endY, 2))
        return self.length

    def linkAngle(self):
        self.angle= math.acos((self.endX-self.startX)/self.linkLength())
        self.angle *= -1 if (self.endY>self.startY) else 1
        return self.angle

    def paint(self, painter, option, widget=None):
        """
        This function creates a path painter the paints a semicircle around the start point (ccw), a straight line
        offset from the main axis of the link, a semicircle around the end point (ccw), and a straight line offset from
        the main axis.  It then assigns a pen and brush.  Finally, it draws a circle at the start and end points to
        indicate the pivot points.
        :param painter:
        :param option:
        :param widget:
        :return:
        """
        path = qtg.QPainterPath()
        len = self.linkLength()
        angLink = self.linkAngle()*180/math.pi
        perpAng = angLink+90
        xOffset = self.radius*math.cos(perpAng*math.pi/180)
        yOffset = -self.radius*math.sin(perpAng*math.pi/180)
        rectStart = qtc.QRectF(self.startX-self.radius, self.startY-self.radius, 2*self.radius, 2*self.radius)
        rectEnd = qtc.QRectF(self.endX-self.radius, self.endY-self.radius, 2*self.radius, 2*self.radius)
        centerLinePen= qtg.QPen()
        centerLinePen.setStyle(qtc.Qt.DashDotLine)
        r,g,b,a=self.pen.color().getRgb()
        centerLinePen.setColor(qtg.QColor(r,g,b,128))
        centerLinePen.setWidth(1)
        p1=qtc.QPointF(self.startX, self.startY)
        p2=qtc.QPointF(self.endX, self.endY)
        painter.setPen(centerLinePen)
        painter.drawLine(p1,p2)
        path.arcMoveTo(rectStart, perpAng)
        path.arcTo(rectStart, perpAng, 180)
        path.lineTo(self.endX-xOffset, self.endY-yOffset)
        path.arcMoveTo(rectEnd, perpAng+180)
        path.arcTo(rectEnd, perpAng+180, 180)
        path.lineTo(self.startX+xOffset, self.startY+yOffset)
        if self.pen is not None:
            painter.setPen(self.pen)  # Red color pen
        if self.brush is not None:
            painter.setBrush(self.brush)
        painter.drawPath(path)
        pivotStart=qtc.QRectF(self.startX-self.radius/6, self.startY-self.radius/6, self.radius/3, self.radius/3)
        pivotEnd=qtc.QRectF(self.endX-self.radius/6, self.endY-self.radius/6, self.radius/3, self.radius/3)
        painter.drawEllipse(pivotStart)
        painter.drawEllipse(pivotEnd)

class RigidPivotPoint(qtw.QGraphicsItem):
    def __init__(self, ptX, ptY, pivotHeight, pivotWidth, parent=None, pen=None, brush=None, rotation=0):
        super().__init__(parent)
        self.x = ptX
        self.y = ptY
        self.pen = pen
        self.brush = brush
        self.height = pivotHeight
        self.width = pivotWidth
        self.radius = min(self.height, self.width) / 4
        self.rect = qtc.QRectF(self.x - self.width / 2, self.y - self.radius, self.width, self.height + self.radius)
        self.rotationAngle = rotation

    def boundingRect(self):
        return self.rect
    def rotate(self, angle):
        self.rotationAngle=angle


    def paint(self, painter, option, widget=None):
        path = qtg.QPainterPath()
        radius = min(self.height,self.width)/2
        rect = qtc.QRectF(self.x-self.width/2, self.y-radius, self.width,self.height+radius)
        H=math.sqrt(math.pow(self.width/2,2)+math.pow(self.height,2))
        phi=math.asin(radius/H)
        theta=math.asin(self.height/H)
        ang=math.pi-phi-theta
        l=H*math.cos(phi)
        x1=self.x+self.width/2
        y1=self.y+self.height
        path.moveTo(x1,y1)
        x2=l*math.cos(ang)
        y2=l*math.sin(ang)
        path.lineTo(x1+x2, y1-y2)
        pivotRect=qtc.QRectF(self.x-radius, self.y-radius, 2*radius, 2*radius)
        stAng=math.pi/2-phi-theta
        spanAng=math.pi-2*stAng
        path.arcTo(pivotRect,stAng*180/math.pi, spanAng*180/math.pi)
        x4=self.x-self.width/2
        y4=self.y+self.height
        path.lineTo(x4,y4)
        #path.arcTo(pivotRect,ang*180/math.pi, 90)
        if self.pen is not None:
            painter.setPen(self.pen)  # Red color pen
        if self.brush is not None:
            painter.setBrush(self.brush)
        painter.drawPath(path)

        pivotPtRect=qtc.QRectF(self.x-radius/4, self.y-radius/4, radius/2,radius/2)
        painter.drawEllipse(pivotPtRect)
        x5=self.x-self.width
        x6=self.x+self.width
        painter.drawLine(x5,y4,x6,y4)
        penOutline = qtg.QPen(qtc.Qt.NoPen)
        hatchbrush = qtg.QBrush(qtc.Qt.BDiagPattern)
        painter.setPen(penOutline)
        painter.setBrush(hatchbrush)
        support = qtc.QRectF(x5,y4,self.width*2, self.height)
        painter.drawRect(support)
        self.setRotation(self.rotationAngle)
class TrussView():
    def __init__(self):
        # setup widgets for display.  redefine these when you have a gui to work with using setDisplayWidgets
        self.scene = qtw.QGraphicsScene()
        self.le_LongLinkName = qtw.QLineEdit()
        self.le_LongLinkNode1 = qtw.QLineEdit()
        self.le_LongLinkNode2 = qtw.QLineEdit()
        self.le_LongLinkLength = qtw.QLineEdit()
        self.te_Report = qtw.QTextEdit()
        self.gv = qtw.QGraphicsView()

        # region setup pens and brushes and scene
        # make the pens first
        # a thick darkGray pen
        self.penLink = qtg.QPen(qtc.Qt.darkGray)
        self.penLink.setWidth(4)
        # a medium darkBlue pen
        self.penNode = qtg.QPen(qtc.Qt.darkBlue)
        self.penNode.setStyle(qtc.Qt.SolidLine)
        self.penNode.setWidth(1)
        # a pen for the grid lines
        self.penGridLines = qtg.QPen()
        self.penGridLines.setWidth(1)
        # I wanted to make the grid lines more subtle, so set alpha=25
        self.penGridLines.setColor(qtg.QColor.fromHsv(197, 144, 228, alpha=50))
        # now make some brushes
        # build a brush for filling with solid red
        self.brushFill = qtg.QBrush(qtc.Qt.darkRed)
        # a brush that makes a hatch pattern
        self.brushNode = qtg.QBrush(qtg.QColor.fromCmyk(0, 0, 255, 0, alpha=100))
        # a brush for the background of my grid
        self.brushGrid = qtg.QBrush(qtg.QColor.fromHsv(87, 98, 245, alpha=128))
        # endregion

    def setDisplayWidgets(self, args):
        self.te_Report = args[0]
        self.le_LongLinkName = args[1]
        self.le_LongLinkNode1 = args[2]
        self.le_LongLinkNode2 = args[3]
        self.le_LongLinkLength = args[4]
        self.gv = args[5]
        self.gv.setScene(self.scene)

    def displayReport(self, truss=None):
        st = '\tTruss Design Report\n'
        st += 'Title:  {}\n'.format(truss.title)
        st += 'Static Factor of Safety:  {:0.2f}\n'.format(truss.material.staticFactor)
        st += 'Ultimate Strength:  {:0.2f}\n'.format(truss.material.uts)
        st += 'Yield Strength:  {:0.2f}\n'.format(truss.material.ys)
        st += 'Modulus of Elasticity:  {:0.2f}\n'.format(truss.material.E)
        st += '_____________Link Summary________________\n'
        st += 'Link\t(1)\t(2)\tLength\tAngle\n'
        longest = None
        for l in truss.links:
            if longest is None or l.length > longest.length:
                longest = l
            st += '{}\t{}\t{}\t{:0.2f}\t{:0.2f}\n'.format(l.name, l.node1_Name, l.node2_Name, l.length, l.angleRad)
        self.te_Report.setText(st)
        self.le_LongLinkName.setText(longest.name)
        self.le_LongLinkLength.setText("{:0.2f}".format(longest.length))
        self.le_LongLinkNode1.setText(longest.node1_Name)
        self.le_LongLinkNode2.setText(longest.node2_Name)

    def buildScene(self, truss=None):
        # Create a QRect() object to help with drawing the background grid.
        rect = qtc.QRect()
        rect.setTop(truss.nodes[0].position.y)
        rect.setLeft(truss.nodes[0].position.x)
        rect.setHeight(0)
        rect.setWidth(0)
        for n in truss.nodes:
            if n.position.y > rect.top(): rect.setTop(n.position.y)
            if n.position.y < rect.bottom(): rect.setBottom(n.position.y)
            if n.position.x > rect.right(): rect.setRight(n.position.x)
            if n.position.x < rect.left(): rect.setLeft(n.position.x)
        rect.adjust(-50, 50, 50, -50)

        # clear out the old scene first
        self.scene.clear()

        # draw a grid
        self.drawAGrid(DeltaX=10, DeltaY=10, Height=abs(rect.height()), Width=abs(rect.width()),
                       CenterX=rect.center().x(), CenterY=rect.center().y())
        # draw the truss
        self.drawLinks(truss=truss)
        self.drawNodes(truss=truss)

    def drawAGrid(self, DeltaX=10, DeltaY=10, Height=320, Width=180, CenterX=120, CenterY=60):
        """
        This makes a grid for reference.  No snapping to grid enabled.
        :param DeltaX: grid spacing in x direction
        :param DeltaY: grid spacing in y direction
        :param Height: height of grid (y)
        :param Width: width of grid (x)
        :param CenterX: center of grid (x, in scene coords)
        :param CenterY: center of grid (y, in scene coords)
        :param Pen: pen for grid lines
        :param Brush: brush for background
        :return: nothing
        """

        startX = (CenterX - Width / 2) - 200
        endX = (CenterX + Width / 2) + 200
        startY = (CenterY - Height / 2) - 200
        endY = (CenterY + Height / 2) + 200

        for x in range(int(startX), int(endX), DeltaX):
            self.scene.addLine(x, startY, x, endY, self.penGridLines)
        for y in range(int(startY), int(endY), DeltaY):
            self.scene.addLine(startX, y, endX, y, self.penGridLines)

    def drawLinks(self, truss):
        highest_y = max(node.position.y for node in truss.nodes)
        for link in truss.links:
            node1 = truss.getNode(link.node1_Name)
            node2 = truss.getNode(link.node2_Name)
            if node1 and node2:
                # Flip the y coordinate around the highest Y to draw with origin at the bottom
                node1_screen_y = highest_y - node1.position.y
                node2_screen_y = highest_y - node2.position.y

                rigid_link = RigidLink(node1.position.x, node1_screen_y,
                                       node2.position.x, node2_screen_y,
                                       radius=5, pen=self.penLink, brush=self.brushFill)
                self.scene.addItem(rigid_link)

    def drawNodes(self, truss):
        # Find the highest Y-coordinate among all nodes to flip the truss correctly
        highest_y = max(node.position.y for node in truss.nodes)
        for node in truss.nodes:
            # Flip the y coordinate around the highest Y to draw with origin at the bottom
            node_screen_y = highest_y - node.position.y
            if node.is_pivot:  # Check if the node is flagged as a pivot
                pivot_point = RigidPivotPoint(node.position.x, node_screen_y,
                                              pivotHeight=20, pivotWidth=20,
                                              pen=self.penNode, brush=self.brushNode)
                self.scene.addItem(pivot_point)
            else:
                self.scene.addEllipse(node.position.x - 5, node_screen_y - 5, 10, 10, self.penNode, self.brushNode)

    def drawALabel(self, x, y, str='', pen=None, brush=None, tip=None):
        text_item = self.scene.addText(str, qtg.QFont("Arial", 10))
        text_item.setPos(x, y)
        if pen:
            text_item.setDefaultTextColor(pen.color())
        if tip:
            text_item.setToolTip(tip)

    def drawACircle(self, centerX, centerY, Radius, angle=0, brush=None, pen=None, name=None, tooltip=None):
        ellipse = self.scene.addEllipse(centerX - Radius, centerY - Radius, 2 * Radius, 2 * Radius, pen, brush)
        if name:
            ellipse.setData(0, name)
        if tooltip:
            ellipse.setToolTip(tooltip)
