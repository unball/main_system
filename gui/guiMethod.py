from gi.repository import GLib

def guiMethod(func):
	def inner(*args):
		GLib.idle_add(func, *args)
	return inner