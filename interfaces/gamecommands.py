#import rospy
#from std_msgs.msg import String
import gui.mainWindow
from gi.repository import Gtk, Gdk
import gui.singleton
from statics.static_classes import world
from statics import field

class gameCommands(metaclass=gui.singleton.Singleton):
	def __init__(self):
		self.goalAlly = 0
		self.goalEnemy = 0
		self.oponente = "Oponente"
		#rospy.init_node('game_commands')
		#pub = rospy.Publisher('game_commands', String, queue_size=1)
		#rate = rospy.Rate(30)
		
	def left(self):
		print ("L")
		world.change_field_side(field.LEFT)
		#pub.publish("L")
		#rate.sleep()
	
	def right(self):
		world.change_field_side(field.RIGHT)
		print ("R")
		#pub.publish("R")
		#rate.sleep()
	
	def ally(self):
		estadoLabel = gui.mainWindow.MainWindow().getObject("gameCommands_estado")
		placarLabel = gui.mainWindow.MainWindow().getObject("gameCommands_placar")
		playPouseButton = gui.mainWindow.MainWindow().getObject("gameCommands_playpause")
		
		if estadoLabel.get_text() == "O jogo está rodando":
			print("Goal")
			self.goalAlly += 1
			world.increaseGameScore()
			placarLabel.set_text("Placar\nUnBall " + str(self.goalAlly) + " x " + str(self.goalEnemy) + " " + self.oponente)
			estadoLabel.set_text("O jogo está pausado")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			playPouseButton.set_label("Play")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("green"))
			#pub.publish("P")
			#rate.sleep()
	
	def enemy(self):
		estadoLabel = gui.mainWindow.MainWindow().getObject("gameCommands_estado")
		placarLabel = gui.mainWindow.MainWindow().getObject("gameCommands_placar")
		playPouseButton = gui.mainWindow.MainWindow().getObject("gameCommands_playpause")
		
		if estadoLabel.get_text() == "O jogo está rodando":
			print("E Goal")
			self.goalEnemy += 1
			world.decreaseGameScore()
			placarLabel.set_text("Placar\nUnBall " + str(self.goalAlly) + " x " + str(self.goalEnemy) + " " + self.oponente)
			estadoLabel.set_text("O jogo está pausado")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			playPouseButton.set_label("Play")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("green"))
			#pub.publish("P")
			#rate.sleep()

	def fimdejogo(self, event):
		print ("Q")
	
	def IniciodeJogo(self):
		estadoLabel = gui.mainWindow.MainWindow().getObject("gameCommands_estado")
		playPouseButton = gui.mainWindow.MainWindow().getObject("gameCommands_playpause")
		if estadoLabel.get_text() == "Equipe UnBall":
			estadoLabel.set_text("O jogo está rodando")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("blue"))
			playPouseButton.set_label("Pause")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			#pub.publish("G")
			#rate.sleep()
			
	def mudarTexto(self):
		estadoLabel = gui.mainWindow.MainWindow().getObject("gameCommands_estado")
		playPouseButton = gui.mainWindow.MainWindow().getObject("gameCommands_playpause")
		if estadoLabel.get_text() == "O jogo está rodando":
			estadoLabel.set_text("O jogo está pausado")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			playPouseButton.set_label("Play")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("green"))
			#pub.publish("P")
			#rate.sleep()
		else:
			estadoLabel.set_text("O jogo está rodando")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("blue"))
			playPouseButton.set_label("Pause")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			#pub.publish("G")
			#rate.sleep()

