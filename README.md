# Documentation for the library

The library provides several classes and functions for performing calculations related to 3D graphics and camera perspective in Pygame.

## Classes

### Point
Represents a point in 3D space with coordinates (x, y, z).

**Methods:**

- `__init__(self, x, y, z)`: Initializes a new Point object with the given x, y, and z coordinates.
- `__mul__(self, t)`: Multiplies the point coordinates by a scalar value t and returns a new Point object.
- `__add__(self, other)`: Adds the coordinates of two points and returns a new Point object.
- `__sub__(self, other)`: Subtracts the coordinates of two points and returns a new Point object.
- `PRINT(self)`: Returns a string representation of the point in the format "(x, y, z)".
- `distance(self, other)`: Calculates the Euclidean distance between two points.
- `length(self)`: Calculates the length of the vector represented by the point.
- `to_vector(self)`: Converts the point to a Vector object.
- `__eq__(self, other)`: Checks if two points are equal by comparing their coordinates.
- `cross(self, other)`: Calculates the cross product of two points and returns a new Vector object.
- `to_tuple(self)`: Returns the coordinates of the point as a list [x, y, z].

### Vector
Represents a vector in 3D space with a starting point and an ending point.

**Methods:**

- `__init__(self, end)`: Initializes a new Vector object with the given ending point.
- `PRINT(self)`: Returns a tuple of string representations of the start and end points in the format "(start, end)".
- `__add__(self, other)`: Adds two vectors and returns a new Vector object.
- `__sub__(self, other)`: Subtracts two vectors and returns a new Vector object.
- `__mul__(self, other)`: Multiplies the vector coordinates by a scalar value or another Point object and returns a new Vector object.
- `__rmul__(self, other)`: Multiplies the vector coordinates by a scalar value and returns a new Vector object.
- `__truediv__(self, other)`: Divides the vector coordinates by a scalar value and returns a new Vector object.
- `__neg__(self)`: Returns a new Vector object with negated coordinates.
- `dot(self, other)`: Calculates the dot product of two vectors.
- `cross(self, other)`: Calculates the cross product of two vectors and returns a new Vector object.
- `length(self)`: Calculates the length of the vector.
- `normalize(self)`: Returns a new Vector object with normalized coordinates.
- `to_point(self)`: Converts the vector to a Point object.
- `rotate_vector(self, angle_x, angle_y, angle_z)`: Rotates the vector around the X, Y, and Z axes by the given angles.

### Object
An abstract class representing an object in 3D space.

**Methods:**

- `__init__(self, position, rotation=None)`: Initializes a new Object object with the given position and rotation.
- `intersect(self, ray)`: Abstract method that needs to be implemented by subclasses. It calculates the intersection of the object with a given ray.

### Sphere (subclass of Object)
Represents a sphere object in 3D space.

Methods:

- `__init__(self, position, radius)`: Initializes a new Sphere object with the given position and radius.
- `intersects(self, ray)`: Calculates the intersection of the sphere with a given ray.

    Parameters:
        - ray: A Ray object representing the ray to intersect with the sphere.
    
    Returns:
        - If the ray intersects with the sphere, returns a tuple (point, normal) representing the intersection point and surface normal at that point.
        - If the ray does not intersect with the sphere, returns None.

