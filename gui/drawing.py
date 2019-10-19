import cv2
from statics.static_classes import world
from statics import field

class Drawing():
    def draw_left_rectangle(image, color, thickness=5):
        height, width, _ = image.shape
        cv2.line(image, (int(width/2)-thickness,0), (0,0), color, thickness)
        cv2.line(image, (0,0), (0, height), color, thickness)
        cv2.line(image, (int(width/2)-thickness,height), (0, height), color, thickness)

    def draw_right_rectangle(image, color, thickness=5):
        height, width, _ = image.shape
        cv2.line(image, (int(width/2)+thickness,0), (width,0), color, thickness)
        cv2.line(image, (width,0), (width, height), color, thickness)
        cv2.line(image, (width, height), (int(width/2)+thickness, height), color, thickness)

    def draw_middle_line(image):
        height, width, _ = image.shape
        cv2.line(image, (int(width/2),0), (int(width/2),height), (100,100,100), 1)

    def draw_field(processed_image):
        Drawing.draw_left_rectangle(processed_image, (0,255,0) if world.fieldSide == field.LEFT else (0,0,255))
        Drawing.draw_right_rectangle(processed_image, (0,255,0) if world.fieldSide == field.RIGHT else (0,0,255))
        Drawing.draw_middle_line(processed_image)