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
        [sg.Slider(range=(0,680), default_value = 200, size=(20, 15), orientation='horizontal', key='sliderDistance'), sg.Checkbox('Enable Automatic Driving', key = 'checkEnableAutodrive')],
        [sg.Text('MODE', key = 'textDriveMode')]
        ]

        self.window = sg.Window('Moonrockers Base Station', layout, resizable = True, finalize = True)

        # get objects
        self.graphMap = self.window['graphMap']

        # set initial graph state (will likely be overwritten very quickly)
        self.robotIds = self.drawRobot(self.graphMap, 100, 100, 1)

    # returns event, values from the window
    def read(self, timeout_ms: int):
        return self.window.read(timeout_ms) 

    # ----- GRAPH UPDATE FUNCTIONS -----
    def updateRobotPosition(self, x: float, y: float, theta: float):
        # remove previously drawn robot, if present
        for id in self.robotIds:
            self.graphMap.delete_figure(id)
            
        # draw robot in new position
        self.robotIds = self.drawRobot(self.graphMap, x, y, theta)
        
    # draws a representation of the current position and angle of the robot on the map graph
    #  returns a list of IDs of the figures drawn, can be used to erase the element later
    def drawRobot(self, mapGraph: sg.Graph, x: float, y: float, theta: float):
        ids = []

        lineEndPoint = (x + self.robotRadius * math.cos(theta), y + self.robotRadius * math.sin(theta))

        ids.append(mapGraph.draw_circle((x,y), self.robotRadius))
        ids.append(mapGraph.draw_line((x,y), lineEndPoint))

        return ids
