def a():
	print 12

def b():
	print 90

class Classfier(object):
	def __init__(self):
		pass
	def identify(world):
		pertinence = normal(x,mu,sig)
		out = pertinence * w
		indx = max(out)
		states[indx]()

class Action(Object):
	def __init__(self):
		pass

class MainTask(Classfier):
	def __init__(self):
		super(MainTask, self).__init__()	
		states = [Ofensive.identify(), Defensive.identify()]

class Ofensive(Classfier):
	def __init__(self):
		super(Ofensive, self).__init__()
		states = [Goalkeeper.identify(), Support.identify(), Stricker.identify()]
 	
class Defensive(Classfier):
	 def __init__(self):
		super(Defensive, self).__init__()
		states = [Goalkeeper.identify(), Defender.identify(), Defender.identify()]

class Stricker(Action):
	def __init__(selS, arg):
		super(stricker, self).__init__()
		states = [straighten_and_go_to_goal, flip_to_rigth, flip_to_left]
		self.arg = arg

class Defender(Action):
	def __init__(self, arg):
		super(Defender, self).__init__()
		states = [formation_defender, go_to_ball]
		self.arg = arg

class Support(Action):
	def __init__(self, arg):
		super(Suport, self).__init__()
		states = [go_to_formation, straighten_and_go_to_goal]
		self.arg = arg

class Goalkeeper(Action):
	def __init__(self, arg):
		super(Goalkeeper, self).__init__()
		states = [formation_goalkeeper, ]
		self.arg = arg
		
				

MainTask.identify()

c = [a,b]
