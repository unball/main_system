import json

def getConfig():
	try:
		with open("config.json", "r") as f:
			return json.load(f)
	except:
		saveConfig({})
		return {}

def saveConfig(config):
	with open("config.json", "w") as f:
		json.dump(config, f)
		
def getValue(key, default_key=None):
	config = getConfig()
	
	if(config.get(key)):
		ret = config[key]
	else:
		config[key] = default_key
		ret = default_key
	
	saveConfig(config)
	return ret

def setValue(key, value):
	config = getConfig()
	config[key] = value
	saveConfig(config)
