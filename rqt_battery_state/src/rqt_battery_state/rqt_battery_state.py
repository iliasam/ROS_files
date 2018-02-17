# when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
import roslib
roslib.load_manifest('rqt_battery_state')

import os
import rospy
import rospkg
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Qt, QTimer, QPoint, QRect
from python_qt_binding.QtGui import QPainter, QCursor, QPen
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState


class Battery_state(Plugin):
    max_x_vel  = 0.4
    max_ang_vel  = 1.0
    tar_x_vel  = 0.0
    tar_ang_vel  = 0.0
    cur_x_vel = 0.0
    cur_ang_vel = 0.0
    acc_x_vel  = 0.1 #linear accelaration - meters/(time step)
    acc_ang_vel = 0.1 #angular accelaration - rad/(time step)

    def __init__(self, context):
        super(Battery_state, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Battery_state')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_battery_state'), 'resource', 'qt_gui3.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('qt_gui3')
        if context.serial_number() > 0:
            self._widget.setWindowTitle('Battery State')
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._handle_update)
        self._update_timer.start(1000)#time step
        
        self.subscriber = rospy.Subscriber('/my_roomba/battery_state', BatteryState, self.subscriber_callback)
	#self._publisher = rospy.Publisher('/cmd_vel', Twist)


    def shutdown_plugin(self):
        #self._publisher.unregister()
	    #self._publisher = None
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
    
    def subscriber_callback(self, msg):
        voltage  = msg.voltage
        current  = msg.current
        f_capacity  = msg.capacity
        capacity  = msg.charge
        percentage  = msg.percentage
        charging = msg.power_supply_status
        
        if math.isnan(percentage):
            percentage = 0
            
        if math.isnan(current):
            current = 0
            
        if math.isnan(f_capacity):
            f_capacity = 0

        if math.isnan(capacity):
            capacity = 0

        self._widget.label_voltage.setText('Voltage: ' + ('%.2f' % voltage) + ' V')
        self._widget.label_current.setText('Current: ' + ('%.2f' % current) + ' A')
        self._widget.label_f_capacity.setText('Full capacity: ' + ('%.2f' % f_capacity) + ' A*h')
        self._widget.label_capacity.setText('Capacity: ' + ('%.2f' % capacity) + ' A*h')
        self._widget.progressBar.setValue(int(percentage))
        
        if (charging == 0):
            self._widget.label_charge.setText('Charging: N/A')
        elif (charging == 1):
            self._widget.label_charge.setText('Charging: Charge')
        elif (charging == 2):
            self._widget.label_charge.setText('Charging: Discharge')
        elif (charging == 3):
            self._widget.label_charge.setText('Charging: Not charging')
        elif (charging == 4):
            self._widget.label_charge.setText('Charging: Full')
        else:
            self._widget.label_charge.setText('Charging: N/A')
       

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        
 