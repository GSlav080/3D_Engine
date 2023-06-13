import time
from main import *


sphere = Sphere(Point(0, 0, -15), 3)
sphere2 = Sphere(Point(-6, 4, -5), 4)
height = 40
wight = 40

# lock_at =  Vector(Point(0, 0, -10))
lock_at = Vector(sphere.position)
# Создаем камеру с позицией (0, 0
# , 10) и направлением взгляда на точку (0, 0, 0)
camera = Camera(Point(0, 0, 10), lock_at, 90, 100, height, wight)





map_1 =Map()


sq1 = Cube(Point(0, 0, -3),2)
sq2 = Cube(Point(-10, 0, 2),2)
sq3 = Cube(Point(2, 0, -6),2)


map_1.append(sphere)
map_1.append(sq1)
map_1.append(sq2)
map_1.append(sq3)




coordsystem = CoordinateSystem(10)
canvas = Console(map_1, camera, coordsystem)
canvas.draw()



import keyboard
import time

Flag = True
while True:
    try:
        key = keyboard.read_key()  # получаем значение нажатой клавиши
        if key == "e":
            if Flag:
                sphers = map_1.objects[0]

                r = sphers.radius
                pos = sphers.position
                for i in range(r, 1, -1):
                    map_1.objects[0].radius = i
                    canvas.map = map_1
                    canvas.draw()

                for i in range(0, r+1, 1):
                    map_1.objects[0] = Cube(pos,i)
                    canvas.draw()
                Flag = False
            else:
                cube = map_1.objects[0]
                pos = cube.position
                size = cube.size
                for i in range(size, 1, -1):
                    map_1.objects[0].radius = i
                    canvas.map = map_1
                    canvas.draw()

                for i in range(0, size + 1, 1):
                    map_1.objects[0] = Sphere(pos, i)
                    canvas.draw()
                Flag=True
                


        
        canvas.camera.ivent(key=key)
        canvas.draw()

    except:
        break



