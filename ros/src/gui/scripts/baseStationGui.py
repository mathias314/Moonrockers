import PySimpleGUI as sg
import math

# manages the base station's GUI, interact with the GUI only through this class
class BaseStationGui():
    def __init__(self):
        # PySimpleGUI config
        sg.theme('Dark Gray 13')

        # stick graph config
        self.stickBorderRadius = 90
        self.stickRadius = 2

        # stick graph storage
        self.leftStickId = None
        self.rightStickId = None
        
        # robot drawing config, dimensions in cm
        self.robotRadius = 50

        layout = [
        # dimensions for graphMap are in cm
        [sg.Graph((680, 500), (0, -250), (680, 250), enable_events = True, background_color = 'white', key = 'graphMap')],
        [sg.Graph((200, 200), (-100, -100), (100, 100), key = 'graphLeftStick'), sg.Graph((200, 200), (-100, -100), (100, 100), key = 'graphRightStick')],
        [sg.Text('MODE', key = 'textDriveMode')]
        ]

        self.window = sg.Window('Moonrockers Base Station', layout, resizable = True, finalize = True)

        # get graph objects
        self.graphLeftStick = self.window['graphLeftStick']
        self.graphRightStick = self.window['graphRightStick']
        self.graphMap = self.window['graphMap']
       
        # draw persistent graph objects
        self.drawStickBorder(self.graphLeftStick)
        self.drawStickBorder(self.graphRightStick)

        # set initial graph state (will likely be overwritten very quickly)
        self.leftStickId = self.drawStick(self.graphLeftStick, 0.5, 0.1)
        self.rightStickId = self.drawStick(self.graphRightStick, 0, 0.4)
        self.robotIds = self.drawRobot(self.graphMap, 100, 100, 1)

    # returns event, values from the window
    def read(self, timeout_ms: int):
        return self.window.read(timeout_ms) 

    # ----- GRAPH UPDATE FUNCTIONS -----
    def updateSticks(self, lx: float, ly: float, rx: float, ry: float):
        leftStickIdLast = self.leftStickId
        rightStickIdLast = self.rightStickId

        self.leftStickId = self.drawStick(self.graphLeftStick, lx, ly)
        self.rightStickId = self.drawStick(self.graphRightStick, rx, ry)

        self.graphLeftStick.delete_figure(leftStickIdLast)
        self.graphRightStick.delete_figure(rightStickIdLast)

    def updateRobotPosition(self, x: float, y: float, theta: float):
        # remove previously drawn robot, if present
        for id in self.robotIds:
            self.graphMap.delete_figure(id)
            
        # draw robot in new position
        self.robotIds = self.drawRobot(self.graphMap, x, y, theta)

    # ----- STICK GRPAH FUNCTIONS -----
    # draws a circle to represent the maximum extent of the stick's motion
    def drawStickBorder(self, stickGraph: sg.Graph):
        stickGraph.draw_circle((0,0), self.stickBorderRadius, line_color='white', line_width=2)

    # draw a representation of the current position of the stick within the border
    #  returns the ID of the figure drawn, can be used to erase the element later
    def drawStick(self, stickGraph: sg.Graph, x: float, y: float):
        xAdj =  x * self.stickBorderRadius
        yAdj = -y * self.stickBorderRadius

        id = stickGraph.draw_circle((xAdj,yAdj), self.stickRadius, fill_color='red', line_color='red')
        return id
        
    # draws a representation of the current position and angle of the robot on the map graph
    #  returns a list of IDs of the figures drawn, can be used to erase the element later
    def drawRobot(self, mapGraph: sg.Graph, x: float, y: float, theta: float):
        ids = []

        lineEndPoint = (x + self.robotRadius * math.cos(theta), y + self.robotRadius * math.sin(theta))

        ids.append(mapGraph.draw_circle((x,y), self.robotRadius))
        ids.append(mapGraph.draw_line((x,y), lineEndPoint))

        return ids
