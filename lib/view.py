from pyglet.gl import (GL_MODELVIEW, GLfloat, glTranslatef, glMatrixMode, glLoadMatrixf)
from math import pi, acos
from euclid import (Vector3, Matrix4)
from obj import SCALE_FACTOR

class Camera(object):
    MIN_DISTANCE = .25 * SCALE_FACTOR
    '''
    A simple camera class encapsulating 'glLookAt' behavior.
    The camera is used for View class in order to support rotating,
    panning and zooming of view, as well as predefined views.
    '''
    def __init__(self, eye, center=Vector3(), up=Vector3(0., 0., 1.)):
        self.orient = Matrix4()
        self.eye = Vector3(*eye)
        self.center = Vector3(*center)
        self.up = Vector3(*up).normalize()
        # Update distance and direction
        direction = self.center - self.eye
        self.distance = direction.magnitude()
        self.dir = direction.normalize()
        self.do_orient_matrix()

    def _build_matrix(self, direction):
        '''
        OpenGL Distilled, page 91.
        The view transformation transforms vertices into eye coordinates and is the inverse of a
        transformation to place and orient the viewer in world coordinates. The view transformation has
        two components: a 4 x 4 orientation matrix, O, and a 4 x 4 translation matrix, T.
        The view transformation matrix, V, is the concatenation of the two 4 x 4 matrices:
        V = O*T
        The orientation matrix O is an orthonormal basis derived from the view direction and up
        vectors. Given an [x y z] view direction d and an [x y z] up vector u, compute their cross
        product c as:
        c = d x u
        Discard u and compute the actual up vector U as:
        U = c x d
        Normalize d, c, and u' to unit length, and create the orientation matrix O as follows:
        | Cx  Cy  Cz  0 |
        | Ux  Uy  Uz  0 |
        |-Dx -Dy -Dz  0 |
        | 0   0   0   1 |
        '''
        orient = Matrix4()
        c = direction.cross(self.up).normalize()
        u = c.cross(direction).normalize()
        orient[0:3] = (c.x, u.x, -direction.x)
        orient[4:7] = (c.y, u.y, -direction.y)
        orient[8:11] = (c.z, u.z, -direction.z)
        return orient

    def do_orient_matrix(self):
        self.orient = self._build_matrix(self.dir)

    def load_matrix(self):
        # Create ctype array of the matrix
        orient = (GLfloat * 16)(*self.orient)
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(orient)
        # Create ctype array of the eye vector
        eye = (GLfloat * 3)(*self.eye)
        # The translation matrix T is simply the inverse translation to the eye location.
        glTranslatef(-eye[0], -eye[1], -eye[2])

    def _update_eye_position(self):
        self.eye = self.dir * -self.distance
        self.eye += self.center
        self.do_orient_matrix()

    def set_center(self, center):
        self.center = center
        self._update_eye_position()

    def set_direction(self, direction):
        self.dir = direction.normalize()
        self._update_eye_position()

    def set_distance(self, distance):
        self.distance = distance if distance > Camera.MIN_DISTANCE else Camera.MIN_DISTANCE
        self._update_eye_position()

    def set_eye_position(self, eye):
        self.eye = eye
        # Update distance and direction
        direction = self.center - self.eye
        self.distance = direction.magnitude()
        self.dir = direction.normalize()
        self.do_orient_matrix()

    def get_parent2local(self, direction=None):
        '''
        The model transformation transforms vertices into world coordinates and is the inverse of a
        transformation to place and orient the viewer in eye coordinates.
        '''
        if direction:
            return self._build_matrix(direction).inverse()
        else:
            return self.orient.inverse()


PIOVER2 = pi / 2.
PIOVER16 = pi / 16.
_2PI = 2. * pi

class View(Camera):
    '''
    The altitude - azimuth view class with fixed up vector.
    '''
    def __init__(self, *args, **kwargs):
        super(View, self).__init__(*args, **kwargs)
        # Save normalized direction vector
        self._dir = Vector3(*self.dir)
        '''
        Our initial altitude angle is computed as:
        1. Take the dot product of the up and view dir vectors.
        This yields the cosine of the angle between them.
        2. The arc-cosine of the dot product produces the angle
        in radian's of the view dir away from the "pole".
        3. To get angle away from the "equator", subtract the angle
        computed in step 2 from pi/2.
        '''
        self.altitude = PIOVER2 - acos(self.up.dot(self._dir))
        # Next, compute a default direction vector that is at
        # the 'equator' (pi / 2 radian's away from the 'pole')
        c = self._dir.cross(self.up)
        self._dir = self.up.cross(c)
        # Our default azimuth angle is initially zero
        self.azimuth = 0.
        # Zoom distance
        self.zoom_distance = 0.

    def rotate(self, dx, dy):
        '''
        Turn the deltas into radians. Multiplying by 0.01 means,
        for example, that moving the mouse 314 pixels will
        rotate by (approximately) PI radians.
        Fine tune the delta motion to minimize 'jumps' on
        big delta motions dx vs. dy.
        Make verticals moves a bit slower than horizontals.
        '''
        if dx != 0.:
            rdydx = abs(dy / dx)
            dx *= -.0010 if rdydx < 3. else -.0001
        if dy != 0.:
            rdxdy = abs(dx / dy)
            dy *= .0010 if rdxdy < 3. else .0001
        # Compute the new altitude angle. Clamp to avoid wrapping
        # around the up vector (or its negation).
        self.altitude += dy
        if self.altitude > -PIOVER16:
            self.altitude = -PIOVER16
        elif self.altitude < -PIOVER2:
            self.altitude = -PIOVER2
        # Compute the new azimuth angle in the range 0 < azimuth < 2*PI
        self.azimuth += dx
        while self.azimuth > _2PI:
            self.azimuth -= _2PI
        while self.azimuth < 0.:
            self.azimuth += _2PI
        '''
        Compute the view direction vector by starting with the
        original direction vector, and rotating it by our
        updated altitude and azimuth angles.
        '''
        c = self._dir.cross(self.up).normalize()
        direction = Vector3(*self._dir)
        direction = direction.rotate_around(c, self.altitude)
        direction = direction.rotate_around(self.up, self.azimuth)
        self.set_direction(direction)

    def zoom(self, y):
        '''
        Change the view distance as a function of the current
        distance. This creates larger movement further away,
        and less movement as the eye get closer to the center.
        '''
        dy = self.zoom_distance - y
        self.zoom_distance = y
        if dy < 300.:
            distance = self.distance * (1. - dy / 300.)
            self.set_distance(distance)

    def pan(self, dx, dy):
        '''
        Panning of view is achieved in a way that views center is
        translated as a point lying in a plane defined by point p
        and vectors v, w.

        r = p + sv + tw	(any point in a plane)

        where: p - views origin (0,0,0)
               v - vector orthogonal to the direction and up vector
               w - vector orthogonal to the direction and v vector (real up vector)
               s - delta x motion
               w - delta y motion
        '''
        # Approximate fine-tuning (on smaller distances performs smaller moves)
        tune = self.distance / 3001.
        dx *= tune
        dy *= tune
        v = self.dir.cross(self.up).normalize()
        w = self.dir.cross(v).normalize()
        v *= -dx
        w *= dy
        center = self.center + v + w
        self.set_center(center)
