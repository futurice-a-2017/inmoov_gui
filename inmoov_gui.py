#!/usr/bin/env python

## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')

## Then load sys to get sys.argv.
import sys

## Next import all the Qt bindings into the current namespace, for
## convenience.  This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed.  The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *

## Finally import the RViz bindings themselves.
import rviz

## Rest of the imports
import threading
import rospy
import config
from listenerThread import listenerThread

## The MyViz class is the main container widget.
class MyViz( QWidget ):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:

    def __init__(self):
        super(MyViz, self).__init__()
        self.initUI()

        # Initialize ROS-node
        rospy.init_node('inmoov_gui')

        # Create and start a thread for listener
        self.listener = listenerThread()
        self.listener.start()

        # Crate timer for updating values (e.g. joint angle values)
        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(lambda: self.updateValues())
        self.updateTimer.start()


    def initUI(self):

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath( "" )

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        rviz_config = rviz.Config()
        reader.readFile( rviz_config, "rviz_config.rviz" )
        self.frame.load( rviz_config )

        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( rviz_config.mapGetChild( "Title" ).getValue() )

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager = self.frame.getManager()

        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        ## Here we create the layout and other widgets in the usual Qt way.

        # Main layout - horizontal box
        self.layout = QHBoxLayout()

        # RViz frame to layout
        self.layout.addWidget( self.frame )

        # 1st vertical layout
        #self.v1_layout = QVBoxLayout()
        #self.v1_layout.setAlignment(Qt.AlignTop)

        # Preset views (defined in rviz_config.rviz) buttons
        #self.top_button = QPushButton( "Top View" )
        #self.top_button.clicked.connect( self.onTopButtonClick )
        #self.v1_layout.addWidget( self.top_button )
        
        #self.side_button = QPushButton( "Side View" )
        #self.side_button.clicked.connect( self.onSideButtonClick )
        #self.v1_layout.addWidget( self.side_button )
        
        # Thickness slider
        #self.thickness_slider = QSlider( Qt.Horizontal )
        #self.thickness_slider.setTracking( True )
        #self.thickness_slider.setMinimum( 1 )
        #self.thickness_slider.setMaximum( 1000 )
        #self.thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        #self.v1_layout.addWidget( self.thickness_slider )

        # 2nd vertical layout
        self.v2_layout = QVBoxLayout()
        self.v2_layout.setAlignment(Qt.AlignTop)

#TODO: labels for joints

        # Labels for joints
        self.labelWaistRotate = QLabel(self)
        labelText = 'Waist rotation: {}'.format(config.angles[0])
        self.labelWaistRotate.setText(labelText)
        self.v2_layout.addWidget(self.labelWaistRotate)

        self.labelRightIndex = QLabel(self)
        labelText = 'Right index: {}'.format(config.angles[1])
        self.labelRightIndex.setText(labelText)
        self.v2_layout.addWidget(self.labelRightIndex)
        
        # Add sublayouts to main layout
        #self.layout.addLayout(self.v1_layout)        
        self.layout.addLayout(self.v2_layout)
        
        self.setLayout( self.layout )

        self.resize( 500, 500 )
        self.show()

    ## Handle GUI events
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## After the constructor, for this example the class just needs to
    ## respond to GUI events.  Here is the slider callback.
    ## rviz.Display is a subclass of rviz.Property.  Each Property can
    ## have sub-properties, forming a tree.  To change a Property of a
    ## Display, use the subProp() function to walk down the tree to
    ## find the child you need.
    def onThicknessSliderChanged( self, new_value ):
        if self.grid_display != None:
            self.grid_display.subProp( "Line Style" ).subProp( "Line Width" ).setValue( new_value / 1000.0 )

    ## The view buttons just call switchToView() with the name of a saved view.
    def onTopButtonClick( self ):
        self.switchToView( "Top View" )

        
    def onSideButtonClick( self ):
        self.switchToView( "Side View" )
        
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

#TODO: labels for joints

    def updateValues(self):
        labelText = 'Waist rotation: {}'.format(config.angles[0])
        self.labelWaistRotate.setText(labelText)

        labelText = 'Right index: {}'.format(config.angles[1])
        self.labelRightIndex.setText(labelText)

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
if __name__ == '__main__':
    app = QApplication( sys.argv )

    myviz = MyViz()

    sys.exit(app.exec_())