from gi.repository import Gdk, GdkPixbuf
from gui.guiMethod import guiMethod
import cv2

class uiFrame:
	
	def __init__(self, gtk_frame, event_frame):
		self._gtk_frame = gtk_frame
		event_frame.add_events(Gdk.EventMask.POINTER_MOTION_MASK)
	
	@guiMethod
	def do_update_frame(self, image_data, shape=(471, 350)):
		image_sized = cv2.resize(image_data, shape)
		height, width, depth = image_sized.shape
		pixbuf = GdkPixbuf.Pixbuf.new_from_data(image_sized.tostring(), GdkPixbuf.Colorspace.RGB, False, 8, width, height, depth*width)
		self._gtk_frame.set_from_pixbuf(pixbuf.copy())
		self._gtk_frame.show()
	
	@guiMethod
	def clear_image(self):
		self._gtk_frame.set_from_icon_name("face-crying-symbolic", 200)
