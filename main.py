from abc import abstractmethod, ABC
from math import *
import math



def Angel(v, angle_x, angle_y, angle_z):
    # Углы в радианах
    angle_x = math.radians(angle_x)
    angle_y = math.radians(angle_y)
    angle_z = math.radians(angle_z)

    # Матрицы поворота
    Rx = [[1, 0, 0],
          [0, math.cos(angle_x), -math.sin(angle_x)],
          [0, math.sin(angle_x), math.cos(angle_x)]]

    Ry = [[math.cos(angle_y), 0, math.sin(angle_y)],
          [0, 1, 0],
          [-math.sin(angle_y), 0, math.cos(angle_y)]]

    Rz = [[math.cos(angle_z), -math.sin(angle_z), 0],
          [math.sin(angle_z), math.cos(angle_z), 0],
          [0, 0, 1]]

    # Результирующая матрица поворота
    R = [[0, 0, 0],
         [0, 0, 0],
         [0, 0, 0]]

    # Умножение матриц
    for i in range(3):
        for j in range(3):
            for k in range(3):
                R[i][j] += Rx[i][k] * Ry[k][j]
            R[i][j] = round(R[i][j], 10)

    for i in range(3):
        for j in range(3):
            for k in range(3):
                R[i][j] *= Rz[k][j]
            R[i][j] = round(R[i][j], 10)

    # Умножение вектора на матрицу поворота
    x = R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z
    y = R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z
    z = R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z

    return Vector(Point(x, y, z))


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, t):
        return Point(self.x * t, self.y * t, self.z * t)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def PRINT(self):
        return f"({self.x}, {self.y}, {self.z})"

    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    def length(self):
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5

    def to_vector(self):
        return Vector(Point(self.x, self.y, self.z))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def cross(self, other):
        x = self.y * other.z - self.z * other.y
        y = self.z * other.x - self.x * other.z
        z = self.x * other.y - self.y * other.x
        return Vector(Point(x, y, z))

    def to_tuple(self):
        return [self.x, self.y, self.z]


class Vector:
    def __init__(self, end):
        self.start = Point(0, 0, 0)
        self.end = end

    def PRINT(self):
        return self.start.PRINT(), self.end.PRINT()

    def __add__(self, other):
        return Vector(self.end + other)

    def __sub__(self, other):
        return Vector(self.end - other)

    def __mul__(self, other):
        if isinstance(other, Point):
            return Vector(Point(self.end.x * other.x, self.end.y * other.y, self.end.z * other.z))
        else:
            return Vector(Point(self.end.x * other, self.end.y * other, self.end.z * other))

    def __rmul__(self, other):
        return Vector(self.end * other)

    def __truediv__(self, other):
        return Vector(self.end * (1 / other))

    def __neg__(self):
        return Vector(-self.end)

    def dot(self, other):
        return (self.end - self.start).x * (other.end - other.start).x + \
            (self.end - self.start).y * (other.end - other.start).y + \
            (self.end - self.start).z * (other.end - other.start).z

    def cross(self, other):
        x = self.end.y * other.end.z - self.end.z * other.end.y - self.start.y * other.start.z + self.start.z * other.start.y
        y = self.end.z * other.end.x - self.end.x * other.end.z - self.start.z * other.start.x + self.start.x * other.start.z
        z = self.end.x * other.end.y - self.end.y * other.end.x - self.start.x * other.start.y + self.start.y * other.start.x
        return Vector(Point(x, y, z))

    def length(self):
        return math.sqrt((self.end.x - self.start.x) ** 2 +
                         (self.end.y - self.start.y) ** 2 +
                         (self.end.z - self.start.z) ** 2)

    def normalize(self):
        l = self.length()
        return Vector(Point(self.end.x / l, self.end.y / l, self.end.z / l))

    def to_point(self):
        return Point(self.end.x, self.end.y, self.end.z)

    def rotate_vector(self, angle_x, angle_y, angle_z):
        # конвертирование углов в радианы
        rad_x = math.radians(angle_x)
        rad_y = math.radians(angle_y)
        rad_z = math.radians(angle_z)

        # поворот конечной точки вокруг оси X
        y = self.end.y * math.cos(rad_x) - self.end.z * math.sin(rad_x)
        z = self.end.y * math.sin(rad_x) + self.end.z * math.cos(rad_x)
        self.end.y, self.end.z = y, z

        # поворот конечной точки вокруг оси Y
        x = self.end.x * math.cos(rad_y) + self.end.z * math.sin(rad_y)
        z = -self.end.x * math.sin(rad_y) + self.end.z * math.cos(rad_y)
        self.end.x, self.end.z = x, z

        # поворот конечной точки вокруг оси Z
        x = self.end.x * math.cos(rad_z) - self.end.y * math.sin(rad_z)
        y = self.end.x * math.sin(rad_z) + self.end.y * math.cos(rad_z)
        self.end.x, self.end.y = x, y

        return self


class Object:
    def __init__(self, position, rotation=None):
        self.position = position
        self.rotation = rotation

    def intersect(self, ray):
        # Это метод заглушка, который должен быть переопределен в наследниках
        raise NotImplementedError("Method intersect is not implemented in subclass")


class Sphere(Object):
    def __init__(self, position, radius):
        super().__init__(position)
        self.radius = radius

    def intersects(self, ray):
        oc = ray.origin - self.position
        a = ray.direction.dot(ray.direction)
        b = 2.0 * Vector(oc).dot(ray.direction)
        c = Vector(oc).dot(Vector(oc)) - self.radius ** 2
        discriminant = b ** 2 - 4 * a * c
        if discriminant < 0:
            return None
        else:
            t = (-b + math.sqrt(discriminant)) / (2 * a)
            if t > 0:
                return ray.origin + ray.direction.to_point() * t
            else:
                return None

    def is_point(sphere, point):
        distance = (point - sphere.position).length() - sphere.radius - 0.01
        return distance <= 0




class CoordinateSystem:
    def __init__(self, unit_length):
        self.unit_length = unit_length


class Ray:
    def __init__(self, origin, direction):
        self.origin = origin
        self.direction = direction.normalize()

    def point_at_parameter(self, t):
        return self.origin + self.direction * t

    def intersect(self, scene):
        closest_distance = float('inf')
        closest_object = None

        for obj in scene.objects:
            intersection = obj.intersects(self)
            if intersection is not None:
                distance = (intersection - self.origin).length()
                if distance < closest_distance:
                    closest_distance = distance
                    closest_object = obj
        return closest_object


import math

class Event:
    event = []
    def handle_key_event(self):
        pass
    def add(self, ev):
        self.event.append(ev)

class Camera:
    def __init__(self, position, lookAt, FOV, drawDistance, screenWidth, screenHeight):
        self.position = position
        self.lookAt = lookAt.normalize()
        self.FOV = FOV
        self.drawDistance = drawDistance
        self.screenDistance = (1 / math.tan(math.radians(self.FOV / 2)))
        self.screenWidth = screenWidth
        self.screenHeight = screenHeight
        self.movement_speed = 0.5
        self.rotation_speed = 10

    def calculate_yaw(self):
        right = self.lookAt.cross(Vector(Point(0, 1, 0))).to_point()
        yaw = math.atan2(right.x, right.z)
        return math.degrees(yaw)

    def calculate_pitch(self):
        pitch = math.asin(self.lookAt.to_point().y)
        return math.degrees(pitch)

    def ivent(self, key):
        if key == 'p':
            running = False
        elif key == 'w':
            self.position += self.lookAt.to_point() * self.movement_speed
        elif key == 's':
            self.position -= self.lookAt.to_point() * self.movement_speed
        elif key == 'a':
            right = self.lookAt.cross(Vector(Point(0, 1, 0))).to_point()
            self.position -= right * self.movement_speed
        elif key == 'd':
            right = self.lookAt.cross(Vector(Point(0, 1, 0))).to_point()
            self.position += right * self.movement_speed
        elif key == 'up':
            pitch = math.asin(self.lookAt.to_point().y)
            pitch += math.radians(self.rotation_speed)
            length = self.lookAt.to_point().length()
            self.lookAt = Vector(
                Point(self.lookAt.to_point().x, math.sin(pitch), -math.cos(pitch))).normalize() * length
        elif key == 'down':
            pitch = math.asin(self.lookAt.to_point().y)
            pitch -= math.radians(self.rotation_speed)
            length = self.lookAt.to_point().length()
            self.lookAt = Vector(
                Point(self.lookAt.to_point().x, math.sin(pitch), -math.cos(pitch))).normalize() * length
        elif key == 'left':
            yaw = math.atan2(self.lookAt.to_point().y, self.lookAt.to_point().x)
            yaw += math.radians(self.rotation_speed)
            length = self.lookAt.to_point().length()
            self.lookAt = Vector(Point(math.cos(yaw), math.sin(yaw), -self.lookAt.to_point().z)).normalize() * length
        elif key == 'right':
            yaw = math.atan2(self.lookAt.to_point().y, self.lookAt.to_point().x)
            yaw -= math.radians(self.rotation_speed)
            length = self.lookAt.to_point().length()
            self.lookAt = Vector(Point(math.cos(yaw), math.sin(yaw), -self.lookAt.to_point().z)).normalize() * length

    def generate_rays(self):
        rays = []
        for y in range(self.screenHeight):
            ray = []
            for x in range(self.screenWidth):
                # Нормализуем координаты пикселя от -1 до 1
                pixel_x = (2 * ((x + 0.5) / self.screenWidth) - 1) * self.screenWidth / self.screenHeight * self.drawDistance / self.screenDistance
                pixel_y = (1 - 2 * (y + 0.5) / self.screenHeight) * self.drawDistance / self.screenDistance

                # Вычисляем направление луча для данного пикселя
                direction = Vector(Point(pixel_x, pixel_y, -self.drawDistance))
                direction = direction.normalize()
                # Создаем луч, исходящий из позиции камеры в данном направлении
                ray.append(Ray(self.position, direction))
            rays.append(ray)
        return rays

    # def generate_rays2(self):
    #     rays = []
    #     for y in range(self.screenHeight):
    #         ray = []
    #         for x in range(self.screenWidth):
    #             # Нормализуем координаты пикселя от -1 до 1
    #             pixel_x = (2 * ((
    #                                     x + 0.5) / self.screenWidth) - 1) * self.screenWidth / self.screenHeight * self.drawDistance / self.screenDistance
    #             pixel_y = (1 - 2 * (y + 0.5) / self.screenHeight) * self.drawDistance / self.screenDistance
    #
    #             # Вычисляем направление луча для данного пикселя
    #             direction = Vector(Point(pixel_x, pixel_y, -self.drawDistance))
    #             direction = direction.rotate_vector(self.calculate_yaw(), self.calculate_pitch(), 0)
    #             direction = direction.normalize()
    #
    #             # Поворачиваем направление луча с учетом взгляда камеры
    #             direction = direction.rotate_vector(self.lookAt.to_point().x, self.lookAt.to_point().y,
    #                                                 self.lookAt.to_point().z)
    #
    #             # Создаем луч, исходящий из позиции камеры в данном направлении
    #             ray.append(Ray(self.position, direction))
    #         rays.append(ray)
    #     return rays

    def rotate(self, delta_yaw, delta_pitch):
        self.yaw += delta_yaw
        self.pitch += delta_pitch

        # Ограничение угла pitch
        self.pitch = max(-90, min(90, self.pitch))

        # Преобразование углов в вектор направления
        yaw_radians = math.radians(self.yaw)
        pitch_radians = math.radians(self.pitch)
        x = math.cos(yaw_radians) * math.cos(pitch_radians)
        y = math.sin(pitch_radians)
        z = math.sin(yaw_radians) * math.cos(pitch_radians)
        self.lookAt = Vector(Point(x, y, z)).normalize()


class Parameters:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def translate(self, vector):
        # перемещение коэффициентов на заданный вектор
        translated_point = Point(-vector.x, -vector.y, -vector.z)
        translated_d = self.d - translated_point.dot(Vector(self.a, self.b, self.c))
        return Parameters(self.a, self.b, self.c, translated_d)

    def scale(self, factor):
        # масштабирование коэффициентов на заданный коэффициент
        return Parameters(self.a / factor, self.b / factor, self.c / factor, self.d / (factor ** 2))



class Plane(Object):
    def __init__(self, point, normal, color, reflectivity=0):
        super().__init__(color, reflectivity)
        self.point = point
        self.normal = normal

    def contains(self, point):
        return Vector.dot(point - self.point, self.normal) == 0

    def intersect(self, ray):
        denominator = Vector.dot(ray.direction, self.normal)
        if denominator == 0:
            # Ray is parallel to plane
            return None
        else:
            t = Vector.dot(self.point - ray.origin, self.normal) / denominator
            if t > 0:
                return ray.origin + t * ray.direction
            else:
                return None


class Cube(Object):
    def __init__(self, position, size):
        super().__init__(position)
        self.size = size

    def intersects(self, ray):
        tmin = (self.position.x - self.size / 2 - ray.origin.x) / ray.direction.to_point().x
        tmax = (self.position.x + self.size / 2 - ray.origin.x) / ray.direction.to_point().x

        if tmin > tmax:
            tmin, tmax = tmax, tmin

        tymin = (self.position.y - self.size / 2 - ray.origin.y) / ray.direction.to_point().y
        tymax = (self.position.y + self.size / 2 - ray.origin.y) / ray.direction.to_point().y

        if tymin > tymax:
            tymin, tymax = tymax, tymin

        if tmin > tymax or tymin > tmax:
            return None

        if tymin > tmin:
            tmin = tymin

        if tymax < tmax:
            tmax = tymax

        tzmin = (self.position.z - self.size / 2 - ray.origin.z) / ray.direction.to_point().z
        tzmax = (self.position.z + self.size / 2 - ray.origin.z) / ray.direction.to_point().z

        if tzmin > tzmax:
            tzmin, tzmax = tzmax, tzmin

        if tmin > tzmax or tzmin > tmax:
            return None

        if tzmin > tmin:
            tmin = tzmin

        if tzmax < tmax:
            tmax = tzmax

        if tmin < 0:
            if tmax < 0:
                return None
            else:
                return ray.origin + ray.direction.to_point() * tmax
        else:
            return ray.origin + ray.direction.to_point() * tmin

    def is_point(self, point):
        half_size = self.size / 2
        return (self.position.x - half_size - 1 <= point.x <= self.position.x + half_size or
                self.position.y - half_size - 1 <= point.y <= self.position.y + half_size or
                self.position.z - half_size - 1 <= point.z <= self.position.z + half_size)


class ParametersPlane(Parameters):
    def __init__(self, A, B, C, D):
        super().__init__()
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def transform(self, matrix):
        """
        Преобразует коэффициенты уравнения плоскости в зависимости от матрицы преобразования
        :param matrix: матрица преобразования
        """
        a, b, c, d = self.A, self.B, self.C, self.D
        A = matrix[0][0] * a + matrix[0][1] * b + matrix[0][2] * c
        B = matrix[1][0] * a + matrix[1][1] * b + matrix[1][2] * c
        C = matrix[2][0] * a + matrix[2][1] * b + matrix[2][2] * c
        D = matrix[0][3] * A + matrix[1][3] * B + matrix[2][3] * C + d
        self.A, self.B, self.C, self.D = A, B, C, D

    def intersect(self, ray):
        """
        Находит точку пересечения луча и плоскости
        :param ray: луч
        :return: точка пересечения или None
        """
        direction = ray.direction
        origin = ray.origin
        t = -(self.A * origin.x + self.B * origin.y + self.C * origin.z + self.D) / (
                self.A * direction.x + self.B * direction.y + self.C * direction.z)
        if t > 0:
            point = origin + t * direction
            return point
        return None

    def contains(self, point, tolerance=1e-6):
        """
        Проверяет, содержит ли плоскость заданную точку
        :param point: точка
        :param tolerance: допустимая погрешность
        :return: True, если содержит, иначе False
        """
        return abs(self.A * point.x + self.B * point.y + self.C * point.z + self.D) < tolerance


class Map:
    def __init__(self):
        self.objects = []

    def append(self, obj):
        self.objects.append(obj)


class Canvas(ABC):
    def __init__(self, map, camera, coordsystem):
        self.map = map
        self.camera = camera
        self.coordsystem = coordsystem

    @abstractmethod
    def draw(self):
        pass


from colorama import init, Fore, Back, Style


class Console(Canvas):
    def __init__(self, map: Map, camera, coordsystem):
        super().__init__(map, camera, coordsystem)
        init()  # Инициализация colorama

    def draw(self):
        # Создаем пустой двумерный массив для отображения карты
        screen = []

        # Создаем массив лучей для каждого пикселя экрана
        rays_many = self.camera.generate_rays()

        objects = [i for i in self.map.objects]

        contrast = 0
        for rays in rays_many:
            consol = []
            for ray in rays:
                data = []
                for obj in objects:

                    intersection = obj.intersects(ray)
                    if intersection is not None and obj.is_point(intersection):
                        # Если есть пересечение, определяем глубину точки пересечения
                        depth = (intersection - self.camera.position).length()
                        data.append(depth)
                        # Определяем символ для отрисовки на основе глубины

                if data != []:

                    consol.append(min(data))
                    mx = max(data)
                    if (contrast < mx):
                        contrast = mx
                else:
                    consol.append(0)
            screen.append(consol)
        max_val = contrast
        min_val = 0
        symbols =" ~@#$%^&*.O"

        scale_factor = (len(symbols) - 1) / (max_val - min_val) if max_val - min_val != 0 else 0
        depth_map = [[round((val - min_val) * scale_factor) if max_val - min_val != 0 else 0 for val in i] for i in
                     screen]

        text_list = []
        for i in depth_map:
            for j in i:
                text_list.append(symbols[j] * 2)
            text_list.append('\n')
        text = ''.join(text_list)
        print(text)

class Player:
    def __int__(self, speed, position, camera):
        super().__init__(speed, position, camera)
        init()  # Инициализация player

class Events:
    def __init__(self):
        self.handlers = {}

    def append(self, event_name):
        self.handlers[event_name] = []

    def handle(self, event_name, handler):
        if event_name in self.handlers:
            self.handlers[event_name].append(handler)
        else:
            print(f"Event '{event_name}' does not exist.")

    def trigger(self, event_name):
        if event_name in self.handlers:
            handlers = self.handlers[event_name]
            for handler in handlers:
                handler()
        else:
            print(f"Event '{event_name}' does not exist.")
