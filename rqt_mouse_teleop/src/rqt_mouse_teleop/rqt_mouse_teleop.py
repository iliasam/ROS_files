# when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
import roslib
roslib.load_manifest('rqt_mouse_teleop')

import os
import rospy
import rospkg
import math

from geometry_msgs.msg import Twist
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Qt, QTimer, QPoint, QRect
from python_qt_binding.QtGui import QPainter, QCursor, QPen
from std_msgs.msg import String



class Mouse_teleop(Plugin):
    max_x_vel  = 0.4
    max_ang_vel  = 1.0
    tar_x_vel  = 0.0
    tar_ang_vel  = 0.0
    cur_x_vel = 0.0
    cur_ang_vel = 0.0
    acc_x_vel  = 0.1 #linear accelaration - meters/(time step)
    acc_ang_vel = 0.1 #angular accelaration - rad/(time step)

    def __init__(self, context):
        super(Mouse_teleop, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Mouse_teleop')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        #self._widget = QWidget()
        self._widget = my_widget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mouse_teleop'), 'resource', 'qt_gui3.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('qt_gui3')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 0:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._handle_update)
        self._update_timer.start(100)#time step
	self._publisher = rospy.Publisher('/cmd_vel', Twist)


    def shutdown_plugin(self):
        self._publisher.unregister()
	self._publisher = None
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
       
    def _handle_update(self):
        #print 'timer'
	self._widget.update()
	self.calculate_tspeeds(self._widget.cur_sector, self._widget.cur_nradius)
	self._widget.label.setText('vel X: ' + ('%.2f' % self.tar_x_vel))
	self._widget.label2.setText('vel Ang: ' + ('%.2f' % self.tar_ang_vel))
	
	if (self._widget.checkBox1.isChecked() == True):
		self._widget.checkBox2.setEnabled(True)
	else:
		self._widget.checkBox2.setEnabled(False)
	
	self.calculate_cspeeds(self.tar_x_vel, self.tar_ang_vel)
	self.send_twist(self.cur_x_vel, self.cur_ang_vel)
        pass

    #calculate current speed. linear, angular - target speeds
    def calculate_cspeeds(self, linear, angular):
	if (self._widget.checkBox1.isChecked() == True): #acceleration enabled
		self.cur_x_vel = self.calculate_speed_acc(self.cur_x_vel, self.tar_x_vel, self.acc_x_vel)
		self.cur_ang_vel = self.calculate_speed_acc(self.cur_ang_vel, self.tar_ang_vel, self.acc_ang_vel)
		if (self._widget.checkBox2.isChecked() == True):#quick stop enabled
			if (linear == 0.0):
				self.cur_x_vel = 0.0
			if (angular == 0.0):
				self.cur_ang_vel = 0.0
    	else:
    		self.cur_x_vel = linear
    		self.cur_ang_vel = angular
	pass

    def calculate_speed_acc(self, cur_speed, new_speed, acc_step):
	set_speed = 0;
	delta = abs(cur_speed - new_speed)
	if (delta < acc_step):
		set_speed = new_speed #no acceleration now
	else:
		if (new_speed > cur_speed):
			set_speed = cur_speed + acc_step
		else:
			set_speed = cur_speed - acc_step
	return set_speed


    def calculate_tspeeds(self, pos, radius):
	self.tar_x_vel = 0.0
	self.tar_ang_vel = 0.0
	if (pos == 0): #forwards
		self.tar_x_vel = self.max_x_vel * radius
	elif (pos == 2): #backwards
		self.tar_x_vel = self.max_x_vel * radius * (-1.0)
	if (pos == 1): #left
		self.tar_ang_vel = self.max_ang_vel * radius
	if (pos == 3): #right
		self.tar_ang_vel = self.max_ang_vel * radius * (-1.0)
	pass

    def send_twist(self, linear, angular):
	twist = Twist()
	twist.linear.x = linear
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = angular
	self._publisher.publish(twist)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        
        
class my_widget(QWidget):

    mouse_is_pressed = False
    cursor_pos = QPoint(0,0)
    cur_nradius = 0 #normalized radius
    cur_sector = 0

    def __init__  (self,*args):
        QWidget. __init__ (self,*args)

    def paintEvent(self,event=None):
	#global mouse_is_pressed

	w_height = self.frameGeometry().height()
	w_width = self.frameGeometry().width()
	if w_height > w_width:
	    diam = w_width
	else:
	    diam = w_height
	radius = int(diam / 2) - 10

	start_x = w_width/2 - radius
	start_y = w_height/2 - radius

	line1_x1 = w_width/2 + radius*0.707
	line1_x2 = w_width/2 - radius*0.707
	line1_y1 = w_height/2 - radius*0.707
	line1_y2 = w_height/2 + radius*0.707
	line2_x1 = w_width/2 - radius*0.707
	line2_x2 = w_width/2 + radius*0.707
	line2_y1 = line1_y1
	line2_y2 = line1_y2

        painter=QPainter()
        painter.begin(self)
        painter.setPen(QPen(Qt.black))
        painter.drawLine(line1_x1, line1_y1, line1_x2, line1_y2) #cross
	painter.drawLine(line2_x1, line2_y1, line2_x2, line2_y2)
	painter.drawEllipse(start_x,start_y , (radius*2), (radius*2))

	if self.mouse_is_pressed == True:
	    painter.setBrush(Qt.blue)
	    rad = self.get_radius(w_width, w_height, self.cursor_pos)
            pos = self.get_pos(w_width, w_height, self.cursor_pos)
	    if (radius < rad):
	        rad = radius
	    self.draw_arc(painter, w_width, w_height, pos, rad)
	    self.cur_nradius = rad*1.0/radius
	    self.cur_sector = pos
	else:
	    self.cur_nradius = 0.0
	    self.cur_sector = 0.0
        painter.end()
	#print self.mouse_is_pressed
        pass

    #get direction of mouse point
    def get_pos(self, w_width, w_height, cursor_pos):
	center_x = w_width/2
	center_y = w_height/2
	angle = math.atan2((center_x - cursor_pos.x()),(center_y - cursor_pos.y()))*180.0/math.pi
	angle = angle - 45
	if (angle<0):
		angle = 360+angle
	pos = int(angle/90) + 1
	if (pos>3):
		pos = 0
	return pos

    #get radius of mouse point
    def get_radius(self, w_width, w_height, cursor_pos):
	center_x = w_width/2
	center_y = w_height/2
	tmp_val = math.pow((center_x - cursor_pos.x()),2) + math.pow((center_y - cursor_pos.y()),2)
	rad = int(math.sqrt(tmp_val))
	return rad

    def draw_arc(self,qp, w_width, w_height, pos, rad):
	start_x = w_width/2 - rad
	start_y = w_height/2 - rad
	rectangle = QRect(start_x,start_y,(rad*2),(rad*2))
	qp.drawPie(rectangle, (45+90*pos)*16, 90*16)
	

    def mousePressEvent(self, QMouseEvent):
	self.cursor_pos = QMouseEvent.pos()
	self.mouse_is_pressed = True
	my_widget.update(self)

    def mouseMoveEvent(self, QMouseEvent):
	self.cursor_pos = QMouseEvent.pos()
	self.mouse_is_pressed = True

    def mouseReleaseEvent(self, QMouseEvent):
	self.cursor_pos = QMouseEvent.pos()
	self.mouse_is_pressed = False
	my_widget.update(self)
	


