from obj import (OBJ, OBJ_vbo, SCALE_FACTOR)
from euclid import (Vector3 as eVector3, Matrix4)
from math import pi
from pyglet.gl import (GLfloat, glPushMatrix, glPopMatrix, glMultMatrixf)
from numpy import array
from bullet.bullet import (
    Vector3 as bVector3, Transform,
    IndexedMesh, TriangleIndexVertexArray, BvhTriangleMeshShape,
    DefaultMotionState, UserMotionState,
    RigidBody,
    SphereShape, CapsuleShapeZ, DISABLE_DEACTIVATION)


class Model(object):
    '''
    A simple model class.
    The model is loaded from Wavefront OBJ file.
    '''

    def __init__(self, filename, view):
        self.model = OBJ_vbo(filename)
        self.view = view
        self.morient = Matrix4()
        self.pivot = eVector3()
        self.position = eVector3()
        self.do_transformation_matrix()

    def do_transformation_matrix(self):
        if self.view.up.y == 1.:
            # Obj's are exported with z axis up (0,0,1), so we need to
            # rotate x for -90 degrees to align with view's y up axis (0,1,0)
            self.orient.rotatex(pi / -2.)
        '''
        Apply transformations in following order (translation -> -pivot -> orientation -> pivot)
        in order to rotate around arbitrary pivot point and properly position the model.
        TODO: Optimize matrix concatenation (rewrite in compact form)
        '''
        self.mtransform = Matrix4.new_translate(*self.position) * Matrix4.new_translate(*(self.pivot * -1.)) * \
                          self.morient * Matrix4.new_translate(*self.pivot)

        # Create ctype array of the matrix
        self.m = (GLfloat * 16)(*self.mtransform)

    def orient_from_direction(self, direction=None):
        '''
        If direction is None the orientation will be determined
        from view's direction vector which means that model will
        align with camera's direction.
        '''
        self.morient = self.view.get_parent2local(direction)
        self.do_transformation_matrix()

    def orient_from_axis_angle(self, angle, axis):
        self.morient.identity()
        self.morient.rotate_axis(angle, axis)
        self.do_transformation_matrix()

    def set_position(self, position):
        self.position = eVector3(*position)

    def set_pivot(self, pivot):
        self.pivot = eVector3(*pivot)

    def render(self):
        glPushMatrix()
        glMultMatrixf(self.m)
        self.model.render()
        glPopMatrix()


class TriangleMesh(object):
    '''
    A class used to load OBJ Wavefront mesh
    into IndexedMesh to be used for Bullet
    Shape (i.e. Rigid Body)
    '''

    def __init__(self, filename):
        obj = OBJ(filename)

        # The number of materials should be one
        assert len(obj.mfaces) is 1
        material, tfaces = obj.mfaces[0]
        # The number of meshes should be one
        assert len(tfaces) is 1
        desc, faces = tfaces[0]
        # The number of faces should be three
        assert desc[0] is 3

        # Decouple tuples
        vertices = []
        for v in obj.vertices:
            vertices.extend(v)

        # Note that face/quads/polys indices in Wavefront OBJ
        # starts from 1 and therefore must be decremented in order
        # to properly work with Bullet
        indices = [i-1 for i in faces[0]]

        # Build numpy arrays
        self.vertices = array(vertices, 'f')
        self.indices = array(indices, 'i')

        # Create mesh
        self.mesh = IndexedMesh()
        self.mesh.setVertices(self.vertices.size / 3, 3 * self.vertices.itemsize, self.vertices)
        self.mesh.setIndices(self.indices.size / 3, 3 * self.indices.itemsize, self.indices)


class Table(Model, TriangleMesh):

    def __init__(self, filename, view):
        Model.__init__(self, filename, view)
        TriangleMesh.__init__(self, filename)

        # Create rigid body
        tvia = TriangleIndexVertexArray()
        tvia.addIndexedMesh(self.mesh)
        shape = BvhTriangleMeshShape(tvia)
        shape.buildOptimizedBvh()
        transform = Transform()
        transform.setIdentity()
        transform.setOrigin(bVector3(0, 0, 0))
        motion = DefaultMotionState()
        motion.setWorldTransform(transform)

        self.body = RigidBody.fromConstructionInfo(motion,              #  MotionState motion
                                                   shape,               #  CollisionShape shape
                                                   0.0,                 #  btScalar mass
                                                   bVector3(0, 0, 0),   #  Vector3 inertia
                                                   transform,           #  Transform worldTransform
                                                   0.0,                 #  btScalar linearDamping
                                                   0.0,                 #  btScalar angularDamping
                                                   0.75,                #  btScalar friction
                                                   0.5,                 #  btScalar restitution
                                                   0.0,                 #  btScalar linearSleepingThreshold
                                                   0.0)                 #  btScalar angularSleepingThreshold
        self.motion = motion
        self.body.setContactProcessingThreshold(0)


class Rails(Model, TriangleMesh):

    def __init__(self, filename, view):
        Model.__init__(self, filename, view)
        TriangleMesh.__init__(self, filename)

        # Create rigid body
        tvia = TriangleIndexVertexArray()
        tvia.addIndexedMesh(self.mesh)
        shape = BvhTriangleMeshShape(tvia)
        shape.buildOptimizedBvh()
        transform = Transform()
        transform.setIdentity()
        transform.setOrigin(bVector3(0, 0, 0))
        motion = DefaultMotionState()
        motion.setWorldTransform(transform)

        self.body = RigidBody.fromConstructionInfo(motion,              #  MotionState motion
                                                   shape,               #  CollisionShape shape
                                                   0.0,                 #  btScalar mass
                                                   bVector3(0, 0, 0),   #  Vector3 inertia
                                                   transform,           #  Transform worldTransform
                                                   0.0,               	#  btScalar linearDamping
                                                   0.0,               	#  btScalar angularDamping
                                                   0.75,                #  btScalar friction
                                                   0.95,                #  btScalar restitution
                                                   0.0,                 #  btScalar linearSleepingThreshold
                                                   0.0)                 #  btScalar angularSleepingThreshold
        self.motion = motion
        self.body.setContactProcessingThreshold(0)


class CSMotionState(UserMotionState):

    def getWorldTransform(self):
        p = self.cue.pivot
        t = self.cue.position
        self.cue.set_pivot(eVector3(p.x, p.y, p.z + self.cue.delta * 0.2))
        self.cue.set_position(eVector3(t.x, t.y, t.z + self.cue.delta * 0.2))
        self.cue.do_transformation_matrix()
        mnumpy = array(self.cue.m, 'f')
        self.transform.setFromOpenGLMatrix(mnumpy)
        return self.transform

    def setWorldTransform(self, transform):
        self.transform = transform


class CueStick(Model):
    # TODO: move constants to the bindings
    CF_KINEMATIC_OBJECT = 2
    CF_NO_CONTACT_RESPONSE = 4

    def __init__(self, filename, view, cball_radius):
        super(CueStick, self).__init__(filename, view)
        self.half = self.model.dimension / 2.0
        self.tip_radius = self.half.x
        self.pivot = eVector3(0, 0, self.half.z + cball_radius)
        self.org_pivot = eVector3(*self.pivot)
        shape = CapsuleShapeZ(self.tip_radius, self.model.dimension.z - self.tip_radius * 2)
        #shape = CylinderShapeZ(bVector3(*self.half))
        transform = Transform()
        transform.setIdentity()
        self.motion = CSMotionState()
        self.motion.cue = self
        self.delta = 0.
        self.dx = 0.
        self.dy = 0.
        self.motion.setWorldTransform(transform)
        self.body = RigidBody.fromConstructionInfo(self.motion,         #  MotionState motion
                                                   shape,               #  CollisionShape shape
                                                   0.0,                 #  btScalar mass
                                                   bVector3(0, 0, 0),   #  Vector3 inertia
                                                   transform,           #  Transform worldTransform
                                                   0.0,                 #  btScalar linearDamping
                                                   0.0,                 #  btScalar angularDamping
                                                   0.0,                 #  btScalar friction
                                                   0.71,                #  btScalar restitution
                                                   0.0,                 #  btScalar linearSleepingThreshold
                                                   0.0)                 #  btScalar angularSleepingThreshold
        self.body.setContactProcessingThreshold(0)
        self.body.setCcdMotionThreshold(0)
        self.body.setHitFraction(0.1 * SCALE_FACTOR)
        self.body.setGravity(bVector3(0, 0, 0))

        # Make it kinematic body with collision contacts generation but no response impulses
        flags = self.body.getCollisionFlags() | CueStick.CF_KINEMATIC_OBJECT | CueStick.CF_NO_CONTACT_RESPONSE
        self.body.setCollisionFlags(flags)
        self.body.setActivationState(DISABLE_DEACTIVATION)

    def orient_from_direction(self, direction=None):
        '''
        Slightly lower direction vector for pi/32 radian's,
        so cue will render beneath the view's eye position.
        '''
        c = self.view.dir.cross(self.view.up).normalize()
        self.direction = eVector3(*self.view.dir)
        self.direction = self.direction.rotate_around(c, pi / 32.)
        super(CueStick, self).orient_from_direction(self.direction)
        self.update_body_position()

        self.dx = 0.
        self.dy = 0.

    def update_body_position(self):
        '''
        Update body position
        '''
        transform = Transform()
        self.mnumpy = array(self.m, 'f')
        transform.setFromOpenGLMatrix(self.mnumpy)
        self.motion.setWorldTransform(transform)

    def move(self, dx, dy):
        '''
        Move in a view's plane
        '''
        dx *= -0.0025
        dy *= -0.0025

        if abs(self.dx - dx) < 0.25 and abs(self.dy - dy) < 0.25:
            v = self.view.dir.cross(self.view.up).normalize()
            w = self.view.dir.cross(v).normalize()
            v *= -dx
            w *= dy
            self.position = self.position + v + w
            self.dx += -dx
            self.dy += -dy


class CueBall(Model):

    def __init__(self, filename, view, mass, position=eVector3()):
        super(CueBall, self).__init__(filename, view)
        self.radius = self.model.dimension.z / 2.0
        self.mass = mass
        # Update z pos from radius
        position.z = self.radius

        # Init Transform
        transform = Transform()
        self.position = position
        self.do_transformation_matrix()
        self.mnumpy = array(self.m, 'f')
        transform.setFromOpenGLMatrix(self.mnumpy)

        # The Rigid Body
        mass = self.mass * SCALE_FACTOR
        shape = SphereShape(self.radius)
        i = shape.getLocalInertia(mass) * 0.65
        self.motion = DefaultMotionState()
        self.motion.setWorldTransform(transform)
        self.body = RigidBody.fromConstructionInfo(self.motion,         #  MotionState motion
                                                   shape,               #  CollisionShape shape
                                                   mass,                	#  btScalar mass
                                                   bVector3(i.x, i.y, i.z), #  Vector3 inertia
                                                   transform,           #  Transform worldTransform
                                                   0.2,              	#  btScalar linearDamping
                                                   0.65,              	#  btScalar angularDamping
                                                   0.25,                #  btScalar friction
                                                   0.7,          		#  btScalar restitution
                                                   1.5,                 #  btScalar linearSleepingThreshold
                                                   1.5)                 #  btScalar angularSleepingThreshold

        self.body.setContactProcessingThreshold(0)
        self.body.setCcdMotionThreshold(0)
        self.body.setHitFraction(0.1 * SCALE_FACTOR)
