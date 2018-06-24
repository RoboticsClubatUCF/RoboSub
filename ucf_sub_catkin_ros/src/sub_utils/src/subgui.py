from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.properties import StringProperty
import rospy
from sensor_msgs.msg import BatteryState


class Box_a(BoxLayout):
	voltage = StringProperty()
	current = StringProperty()
	charge = StringProperty()

	def chgText(self, msg):
		self.voltage = str(msg.voltage)
		self.current = str(msg.current)
		self.charge = str(msg.charge)voltagvoltage = StringProperty()e = StringProperty()

class SubGUI(App):
    def build(self):
    	layout = Box_a()
    	self.button_subscribe = rospy.Subscriber('/BatteryStatus', BatteryState, layout.chgText)
        return  layout

if __name__ == '__main__':
	rospy.init_node('sub_gui', anonymous=False)
	gui = SubGUI()
	gui.run()