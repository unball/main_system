from statics.static_classes import world

def pixel2meters(position, shape):
	camera_x_length = shape[1]
	camera_y_length = shape[0]
	field_x_length = world.field_x_length
	field_y_length = world.field_y_length
	x_conversion = field_x_length / camera_x_length
	y_conversion = (field_y_length / camera_y_length) * -1

	x = position[0] - camera_x_length / 2
	y = position[1] - camera_y_length / 2
	x *= x_conversion
	y *= y_conversion
	
	return (x,y)
