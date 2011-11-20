from pyglet.gl import *
from pyglet.window import mouse
from pyglet.window import key
from view import View
from model import (Table, Rails, CueBall, CueStick)
import time
from euclid import Vector3 as eVector3, Matrix4
from bullet.bullet import (Vector3 as bVector3, DiscreteDynamicsWorld,
                           DRAW_WIREFRAME, DRAW_CONTACT_POINTS)
from obj import SCALE_FACTOR


# Define a simple function to create ctypes arrays of floats
def vec(*args):
    return (GLfloat * len(args))(*args)


class DebugDraw:
    mode = DRAW_WIREFRAME | DRAW_CONTACT_POINTS

    def reset(self):
        self.lines = []
        self.contacts = []

    def drawLine(self, *args):
        self.lines.append(args)

    def drawContactPoint(self, *args):
        self.contacts.append(args)

    def setDebugMode(self, mode):
        self.mode = mode

    def getDebugMode(self):
        return self.mode


class PoolWindow(pyglet.window.Window):
    ZOOM_SCROLL_STEP = 5
    BALL_MAX_SPEED = 7500

    def __init__(self, *args, **kwargs):
        super(PoolWindow, self).__init__(*args, **kwargs)
        self.set_size(1680, 1000)
        #self.set_fullscreen()
        self.keys = key.KeyStateHandler()
        self.push_handlers(self.keys)

        # Setup GL
        self.set_vsync(True)
        glClearColor(.15, .15, .15, 1)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        glShadeModel(GL_SMOOTH)

        self.lpos1 = (0, 0.5 * SCALE_FACTOR, 1 * SCALE_FACTOR, 0)
        self.lpos2 = (0, -0.5 * SCALE_FACTOR, 1 * SCALE_FACTOR, 0)
        glLightfv(GL_LIGHT0, GL_POSITION, vec(*self.lpos1))
        glLightfv(GL_LIGHT1, GL_POSITION, vec(*self.lpos2))

        glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHT1)

        # Scroll zoom
        self.wheel_step = PoolWindow.ZOOM_SCROLL_STEP

        # Setup view
        self.view = View(eye=(0, -3 * SCALE_FACTOR, .75 * SCALE_FACTOR),
                         center=(0, 0, -SCALE_FACTOR))
        self.view.set_distance(SCALE_FACTOR)

        # The scene
        self.table = Table('obj/table.obj', self.view)
        self.rails = Rails('obj/rails.obj', self.view)
        self.ball = {}
        self.ball['cue'] = CueBall('obj/ball.obj', self.view, .260)
        self.ball['red'] = CueBall('obj/red.obj', self.view, .260,
                                   eVector3(0, 0.5 * SCALE_FACTOR, 0))
        self.ball['yellow'] = CueBall('obj/yellow.obj', self.view, .260,
                                      eVector3(0, -0.5 * SCALE_FACTOR, 0))
        self.cue = CueStick('obj/cue.obj', self.view, self.ball['cue'].radius)

        self.table.body.setUserPointer("Table")
        self.rails.body.setUserPointer("Rails")
        self.cue.body.setUserPointer("Cue")
        self.ball['cue'].body.setUserPointer("Cue Ball")
        self.ball['red'].body.setUserPointer("Red")
        self.ball['yellow'].body.setUserPointer("Yellow")

        # Set view center
        pos = self.ball['cue'].motion.getWorldTransform().getOrigin()
        self.view.set_center(eVector3(pos.x, pos.y, pos.z))

        # Setup cue (position and orient)
        self._reset_cue()

        # Initialize physics
        self.world = DiscreteDynamicsWorld()
        self.debug = DebugDraw()
        self.world.setDebugDrawer(self.debug)
        self.world.addRigidBody(self.table.body)
        self.world.addRigidBody(self.cue.body)
        self.world.addRigidBody(self.rails.body)
        for ball in self.ball.values():
            self.world.addRigidBody(ball.body)
        self.world.setGravity(bVector3(0, 0, -9.81 * SCALE_FACTOR))

        # Register call-backs
        # A 1/60 call-back to run physics (the same as render)
        pyglet.clock.schedule(self._step)
        # Physic clock callback (usually few times faster than render clock)
        self.world.setInternalTickCallback(self.internal_tick_callback, True)

        self._setup_pan()
        self.topview = False
        self.helper = True

        self.push_handlers(on_key_press=self._on_key_press)

    def _setup_pan(self):
        self.view.pan(0, - self.width / 5.0)

    def internal_tick_callback(self, timeStep):
        if self._cue_ball_rest():
            disp = self.world.getDispatcher()
            n = disp.getNumManifolds()
            for i in xrange(n):
                cm = disp.getManifoldByIndexInternal(i)
                contacts = cm.getNumContacts()
                for c in xrange(contacts):
                    obA = cm.getBody0().getUserPointer()
                    obB = cm.getBody1().getUserPointer()
                    if obA == 'Cue' and obB == 'Cue Ball' \
                    or obA == 'Cue Ball' and obB == 'Cue':
                        p = cm.getContactPoint(c)
                        if p.getDistance() < 0.0: # test for penetration
                            # The cue's strike force is calculated from actual
                            # linear velocity applied to the cues orientation vector
                            b = self.cue.body.getLinearVelocity()
                            v = eVector3(b.x, b.y, b.z)
                            d = self.cue.direction if hasattr(self.cue, 'direction') \
                                                   else self.view.dir
                            # get force
                            impuls = v.magnitude() * SCALE_FACTOR * 25.
                            force = bVector3(d.x * impuls,
                                             d.y * impuls,
                                             0.0)
                            # span offset to [0..1]
                            offset = eVector3(self.cue.dx * 4., 0., self.cue.dy * 4.)

                            # calculate offset from cue's position [convert from eye into object space]
                            # TODO: (rewrite this in compact form of vector vs. matrix multiplication)
                            moffset = self.view.orient.inverse() * Matrix4.new_translate(*offset)
                            offset = eVector3(moffset.d, moffset.h, moffset.l)
                            cue = self.ball['cue']
                            cue.body.clearForces()
                            cue.body.applyGravity()
                            cue.body.applyForce(force, bVector3(offset.x, offset.y, offset.z))

                            # Restore cue
                            self._reset_cue()

        def ball_cant_fly(ball):
            # check for flying
            pos = ball.motion.getWorldTransform().getOrigin()
            if pos.z > ball.radius:
                ball.body.applyCentralForce(bVector3(0, 0, -15 * SCALE_FACTOR))
            # check for ball speed
            v = ball.body.getLinearVelocity()
            vel = eVector3(v.x, v.y, v.z)
            if vel.magnitude_squared() > PoolWindow.BALL_MAX_SPEED:
                ball.body.setLinearVelocity(v * 0.9)

        for ball in self.ball.values():
            ball_cant_fly(ball)

    def on_mouse_press(self, x, y, button, modifiers):
        if button & mouse.RIGHT:
            self.view.zoom_distance = y

    def on_mouse_motion(self, x, y, dx, dy):
        if self.keys[key.LCTRL] and self._cue_ball_rest():
            self.cue.delta = dy * -0.01 * SCALE_FACTOR

    def on_mouse_drag(self, x, y, dx, dy, button, modifiers):
        if button & mouse.LEFT:
            if modifiers & key.MOD_WINDOWS:
                self.view.rotate(dx, dy)
            else:
                self.view.rotate(dx, dy)
                pos = self.ball['cue'].motion.getWorldTransform().getOrigin()
                self.view.set_center(eVector3(pos.x, pos.y, pos.z))
                self.cue.set_position(eVector3(pos.x, pos.y, pos.z + self.cue.pivot.z))
                self.cue.orient_from_direction()
                self._setup_pan()
        elif button & mouse.RIGHT:
            pos = self.ball['cue'].motion.getWorldTransform().getOrigin()
            self.view.set_center(eVector3(pos.x, pos.y, pos.z))
            self.view.zoom(y)
            self._setup_pan()
        elif button & mouse.MIDDLE:
            if modifiers & key.MOD_SHIFT:
                self.cue.move(dx, dy)
            else:
                self.view.pan(dx, dy)

    def _update_scroll(self, dt):
        distance = self.view.distance * self.zmul
        self.view.set_distance(distance)
        pos = self.ball['cue'].motion.getWorldTransform().getOrigin()
        self.view.set_center(eVector3(pos.x, pos.y, pos.z))
        self._setup_pan()
        self.wheel_step -= 1
        if self.wheel_step == 0:
            pyglet.clock.unschedule(self._update_scroll)

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        # Regulate zooming direction depending on scroll wheel direction
        # Schedule scroll update
        self.zmul = .98 if scroll_y > 0 else 1.02
        self.wheel_step += 2 if self.wheel_step > 0 else PoolWindow.ZOOM_SCROLL_STEP
        pyglet.clock.schedule(self._update_scroll)

    def _on_key_press(self, symbol, modifiers):
        if symbol == key.TAB:
            self.topview = not self.topview
        if symbol == key.H:
            self.helper = not self.helper

    def on_resize(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(40., width / float(height), .01, 1000.)
        self.view.load_matrix()
        glMatrixMode(GL_MODELVIEW)
        return pyglet.event.EVENT_HANDLED

    def on_draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glViewport(0, 0, self.width, self.height)

        def redraw_scene(self, debug=False):
            # table
            self.table.render()

            # rails
            self.rails.render()

            # balls
            for ball in self.ball.values():
                p = ball.motion.getWorldTransform().getOrigin()
                q = ball.motion.getWorldTransform().getRotation()
                a = q.getAxis()
                ball.set_position((p.x, p.y, p.z))
                ball.orient_from_axis_angle(q.getAngle(), eVector3(a.x, a.y, a.z))
                ball.render()

            # cue stick (only if cue ball is resting)
            if self._cue_ball_rest():
                self.cue.render()

            # debug draw
            if debug:
                self.debug.reset()
                self.world.debugDrawWorld()
                glDisable(GL_LIGHTING)
                glBegin(GL_LINES)
                for line in self.debug.lines:
                    glColor3f(*line[6:])
                    glVertex3f(*line[:3])
                    glVertex3f(*line[3:6])
                glEnd()
                glEnable(GL_LIGHTING)

        def render_top_cue(self):
            # Redraw helper top-right (top view)
            glViewport(self.width - self.width / 3, self.height - self.height / 3, self.width / 3, self.height / 3)
            glLoadIdentity()
            # top-view
            pos = self.ball['cue'].position
            gluLookAt(pos.x, pos.y, pos.z + SCALE_FACTOR / 2,
                      pos.x, pos.y, pos.z,
                      1., 0., 0.)
            redraw_scene(self)

        def render_front_cue(self):
            # Redraw helper top-right (front view)
            glViewport(self.width - self.width / 3, self.height - self.height / 3, self.width / 3, self.height / 3)
            glLoadIdentity()
            eye = eVector3(self.cue.mtransform.d, self.cue.mtransform.h, self.cue.mtransform.l)
            eye += self.cue.direction * 6.
            # render front
            pos = self.ball['cue'].position
            gluLookAt(eye.x, eye.y, eye.z,
                      pos.x, pos.y, pos.z,
                      0., 0., 1.)
            redraw_scene(self, True)

        def render_top_table(self):
            # render top
            glLoadIdentity()
            gluLookAt(0, 0, 3 * SCALE_FACTOR,
                      0, 0, 0,
                      1, 0., 0.)
            redraw_scene(self)

        # render table
        if not self.topview:
            self.view.load_matrix()
            redraw_scene(self)
        else:
            render_top_table(self)

        # render helper windows (for aim and shoot)
        if self._cue_ball_rest() and self.helper:
            if self.keys[key.LSHIFT]:
                render_front_cue(self)
            elif not self.topview:
                render_top_cue(self)

    def _step(self, dt):
        timeStep = fixedTimeStep = 1.0 / 300.0
        self.world.stepSimulation(dt, 5, fixedTimeStep)
        now = time.time()
        delay = now % timeStep
        time.sleep(delay)

        # Cancel cue stick motion
        self.cue.delta = 0.0

    def _cue_ball_rest(self):
        vel = self.ball['cue'].body.getLinearVelocity()
        mag = eVector3(vel.x, vel.y, vel.z).magnitude()
        return mag < 0.001

    def _reset_cue(self):
        # Position and orient the cue
        pos = self.ball['cue'].motion.getWorldTransform().getOrigin()
        p = self.cue.org_pivot
        self.cue.set_pivot(eVector3(p.x, p.y, p.z + SCALE_FACTOR * 0.1))
        self.cue.set_position(eVector3(pos.x, pos.y, pos.z + self.cue.pivot.z))
        self.cue.orient_from_direction()

try:
    # Try and create a window with multisampling (antialiasing)
    config = Config(sample_buffers=1, samples=4, depth_size=16, double_buffer=True)
    window = PoolWindow(resizable=True, config=config)
except pyglet.window.NoSuchConfigException:
    # Fall back to no multisampling for old hardware
    window = PoolWindow(resizable=True)

pyglet.app.run()
