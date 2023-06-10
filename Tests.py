import time
from main import *
#
#
# sphere = Sphere(Point(0, 0, 2), 3)
#
# height = 600
# wight = 800
#
# # lock_at =  Vector(Point(0, 0, -10))
# lock_at = Vector(sphere.position)
# # Создаем камеру с позицией (0, 0, 10) и направлением взгляда на точку (0, 0, 0)
# camera = Camera(Point(0, 0, 10), lock_at, 90, 100, height, wight)
#
#
#
#
# # sq = Cube(Point(-4, 0, 1),4)
# # map.append(sq)
# map =Map()
# map.append(sphere)
#
# # sphere2 =Sphere(Point(-3, 0, 5), 1)
# # sphere3 = Sphere(Point(-3, 0, 2), 3)
# # map.append(sphere2)
# # map.append(sphere3)
#
# coordsystem = CoordinateSystem(10)
# canvas = Console(map, camera, coordsystem)
# canvas.draw()

import pygame
from pygame.locals import *

# Инициализация Pygame
pygame.init()

map = Map()

map.append(Sphere(Point(0, 0, -5), 1))

map.append(Cube(Point(0, 0, -1), 1))

# Определение размеров экрана
screen_width = 200
screen_height = 100

# Создание окна


screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Camera Example")

# Создание объекта камеры
camera = Camera(position=Point(0, 0, 2), lookAt=Vector(Point(0, 0, -1)), FOV=90, drawDistance=100,
                screenWidth=screen_width, screenHeight=screen_height)

# Создание карты с объектами


# Игровой цикл
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == KEYDOWN:
            # Обработка нажатия клавиш
            camera.handle_key_event(event.key)

    # Обновление камеры

    # Очистка экрана
    screen.fill((0, 0, 0))

    # Генерация лучей для рендеринга
    rays = camera.generate_rays()
    # Рендеринг сцены с использованием лучей
    for y in range(camera.screenHeight):
        for x in range(camera.screenWidth):
            ray = rays[y][x]

            # Выполняйте проверку пересечений лучей с объектами на карте
            # и отрисовывайте объекты на экране с учетом их позиции и цвета

            for obj in map.objects:
                intersection_point = obj.intersects(ray)
                if intersection_point:
                    # Вычисляем расстояние от камеры до пересечения луча и объекта
                    distance = (intersection_point - camera.position).length()

                    # Вычисляем интенсивность цвета в зависимости от расстояния
                    intensity = max(0, (distance / obj.position.length())) % 256

                    # Выполняйте отрисовку объекта на экране с учетом интенсивности цвета
                    color = (int(256 * intensity) % 256, 0, 0)
                    pygame.draw.circle(screen, color, (x, y), 1)

    # Отображение на экране
    pygame.display.flip()

# Завершение Pygame

pygame.quit()
#
