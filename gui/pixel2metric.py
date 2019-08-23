field_x_length = 1.60
field_y_length = 1.30

camera_x_length = 640
camera_y_length = 480

def pixels2meters(message):
    x_conversion = field_x_length / camera_x_length;
    y_conversion = (field_y_length / camera_y_length) * -1

    for i in range(6):
        message.x[i] -= camera_x_length / 2
        message.y[i] -= camera_y_length / 2
        message.x[i] *= x_conversion
        message.y[i] *= y_conversion
    
    message.ball_x -= camera_x_length / 2
    message.ball_y -= camera_y_length / 2
    message.ball_x *= x_conversion
    message.ball_y *= y_conversion

    return message
    
def pixel2meters(position, shape):
	camera_x_length = shape[1]
	camera_y_length = shape[0]
	x_conversion = field_x_length / camera_x_length
	y_conversion = (field_y_length / camera_y_length) * -1

	x = position[0] - camera_x_length / 2
	y = position[1] - camera_y_length / 2
	x *= x_conversion
	y *= y_conversion
	
	return (x,y)
