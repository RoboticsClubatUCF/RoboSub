from kivy.app import App
from kivy.clock import Clock
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.properties import StringProperty
from kivy.graphics.texture import Texture
import rospy
from sensor_msgs.msg import BatteryState
import cv2


class KivyCamera(Image):
    def __init__(self, capture, fps, **kwargs):
        super(KivyCamera, self).__init__(**kwargs)
        self.capture = capture
        Clock.schedule_interval(self.update, 1.0 / fps)

    def update(self, dt):
        ret, frame = self.capture.read()
        if ret:
            # convert it to texture
            #frame = cv2.resize(frame, (5120, 3840))
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tostring()
            image_texture = Texture.create(
                size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            # display image from the texture
            self.texture = image_texture

class Battery_a(FloatLayout):
	voltage = StringProperty()
	current = StringProperty()
	charge = StringProperty()

	def chgText(self, msg):
		self.voltage = str(msg.voltage)
		self.current = str(msg.current)
		self.charge = str(msg.charge)

class SubGUI(App):
    def build(self):
    	self.battery = Battery_a()
    	self.button_subscribe = rospy.Subscriber('/BatteryStatus', BatteryState, self.battery.chgText)
    	self.capture = cv2.VideoCapture(0)
    	self.my_camera = KivyCamera(capture=self.capture, fps=30)
    	layout = FloatLayout(size=(640,480))
    	layout.add_widget(self.my_camera)
    	layout.add_widget(self.battery)
    	print(self.my_camera.parent.size)

        return  layout 

        def on_stop(self):
        	#without this, app will not exit even if the window is closed
			self.capture.release()

if __name__ == '__main__':
	rospy.init_node('sub_gui', anonymous=False)
	SubGUI().run()