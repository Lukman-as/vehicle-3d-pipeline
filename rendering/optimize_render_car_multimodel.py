"""
Multi-model car renderer.

For each detected car it fits ALL FOUR per-type models (sedan / suv / truck /
van) to the point cloud and renders the one with the smallest body-shape
residual -- i.e. the vehicle type is inferred by which type-specific shape best
explains the observed points.  The heavy lifting (conditional params, the four
geometry templates, the fit, and the selection) lives in car_models_core; this
file is only the OpenGL/Qt rendering of the winning model.

Run:  python optimize_render_car_multimodel.py
(reads sample.txt + points0/1/2.csv exactly like optimize_render_car.py)
"""

import sys
import numpy as np
import pandas as pd   # type: ignore
from OpenGL.GL import *   # type: ignore
from OpenGL.GLU import *   # type: ignore
from OpenGL.GLUT import *   # type: ignore
from PyQt5.QtWidgets import QApplication, QOpenGLWidget   # type: ignore
from PyQt5.QtCore import Qt   # type: ignore

import car_models_core as core
import vehicle_meshes

# Body colour per winning type, so the chosen model is obvious on screen.
TYPE_COLORS = {
    'sedan': (0.20, 0.40, 0.80),   # blue
    'suv':   (0.20, 0.65, 0.30),   # green
    'truck': (0.85, 0.45, 0.10),   # orange
    'van':   (0.55, 0.30, 0.70),   # purple
}
BLACK = (0.1, 0.1, 0.1)


def clear_gl_errors():
    while glGetError() != GL_NO_ERROR:
        pass


class GLWidget(QOpenGLWidget):
    def __init__(self, points_file, pred_ow, pred_wb, pred_ol, pred_oh,
                 pred_heading_angle, dist_to_move, parent=None):
        super().__init__(parent)
        glutInit(sys.argv)
        self.setMinimumSize(1000, 1000)
        self.setFocusPolicy(Qt.StrongFocus)

        # ---- load + transform the point cloud (identical to the original) ----
        if isinstance(points_file, np.ndarray):
            self.points = np.array(points_file, dtype=float)
        else:
            points_df = pd.read_csv(points_file, header=None)
            self.points = points_df.values
        transformed = np.zeros_like(self.points, dtype=float)
        transformed[:, 0] = -self.points[:, 2]   # x = width
        transformed[:, 1] = -self.points[:, 1]   # y = height up
        transformed[:, 2] = -self.points[:, 0]   # z = -length (away)
        self.points = transformed * 100.0        # m -> cm
        self.points[:, 0] -= np.mean(self.points[:, 0])
        self.points[:, 1] -= np.min(self.points[:, 1])
        self.points[:, 2] -= np.min(self.points[:, 2])

        # ---- view ----
        self.x, self.y, self.z = 0, 0, -1000
        self.rotX, self.rotY, self.rotZ = 10.0, 0.0, 0.0
        self.lastPos = None

        # ---- fit all four type models, keep the best ----
        init_angle = core.as_scalar(pred_heading_angle) + 180
        best, results = core.select_best_fit(
            self.points,
            ow=pred_ow * 100, wb=pred_wb * 100, ol=pred_ol * 100, oh=pred_oh * 100,
            init_angle=init_angle)

        self.vehicle_type = best
        self.params = results[best]['params']
        self.geometry = results[best]['geom']
        self.dist_to_move = tuple(results[best]['translation'])
        self.pred_heading_angle = results[best]['angle']
        self.body_color = TYPE_COLORS[best]

        # Real display mesh for the winning type (the box stays the FIT geometry;
        # this only replaces what's drawn).  Parametric Kenney reconstruction:
        # body scaled to OW/OH/OL, wheels relocated to the derived WB(F,G)/TW/tire_r.
        self.car_list = None
        try:
            self.mesh = vehicle_meshes.reconstruct(best, self.params)
            self.use_mesh = True
        except Exception as exc:
            print(f"[mesh] parametric model unavailable, drawing box instead: {exc}")
            self.use_mesh = False

        print(f"\n[{points_file}] selected: {best.upper()}")
        for vt, r in sorted(results.items(), key=lambda kv: kv[1]['score']):
            mark = '  <-- BEST' if vt == best else ''
            print(f"   {vt:6} score={r['score']:9.2f}  = resid {r['residual']:8.2f} "
                  f"+ {core.PRIOR_WEIGHT:g}*nll {r['nll']:6.2f}   (angle={r['angle'] % 360:6.1f}){mark}")

    # ---------------------------------------------------------------- GL setup
    def initializeGL(self):
        self.makeCurrent()
        ctx = self.context()
        if ctx is None or not ctx.isValid():
            raise RuntimeError("OpenGL context was not created successfully.")
        clear_gl_errors()
        glClearColor(0, 0, 0, 1)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        self.resizeGL(self.width(), self.height())
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [100.0, 1000.0, -1000.0, 0.5])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.25, 0.25, 0.25, 1.0])
        # Soft directional fill from the front so dark (PBR-baked) bodies aren't
        # lost in shadow on the camera-facing side.
        glEnable(GL_LIGHT1)
        glLightfv(GL_LIGHT1, GL_POSITION, [-600.0, 500.0, 1200.0, 0.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.55, 0.55, 0.55, 1.0])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.40, 0.40, 0.40, 1.0])  # global ambient lift
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0)
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)  # light back faces too (winding-proof)
        if self.use_mesh:
            self._build_car_list()

    def _build_car_list(self):
        """Compile the real vehicle triangle soup into a display list once
        (needs a live GL context, so it runs from initializeGL)."""
        pos, nrm, col = self.mesh
        self.car_list = glGenLists(1)
        glNewList(self.car_list, GL_COMPILE)
        glBegin(GL_TRIANGLES)
        for p, n, c in zip(pos, nrm, col):
            glColor3f(c[0], c[1], c[2])
            glNormal3f(n[0], n[1], n[2])
            glVertex3f(p[0], p[1], p[2])
        glEnd()
        glEndList()

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = width / height if height != 0 else 1
        gluPerspective(45.0, aspect, 0.1, 7000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(self.x, self.y, self.z)
        glRotatef(self.rotX, 1, 0, 0)
        glRotatef(self.rotY, 0, 1, 0)
        glRotatef(self.rotZ, 0, 0, 1)

        glPushMatrix()
        glTranslatef(self.dist_to_move[0], self.dist_to_move[1], self.dist_to_move[2])
        glRotatef(self.pred_heading_angle, 0, 1, 0)
        self.renderCar()
        glPopMatrix()

        self.renderPoints()

    # ------------------------------------------------------------- rendering
    def renderCar(self):
        if self.use_mesh and self.car_list is not None:
            glCallList(self.car_list)            # real Kenney model
        else:                                    # fallback: procedural box + tires
            for q in self.geometry['quads']:
                v = q['vertices']
                self.renderQuad(v[0], v[1], v[2], v[3], q['normal'], self.body_color)
            for cyl in self.geometry['cylinders']:
                self.renderCylinder(BLACK, cyl['center'], cyl['radius'], cyl['height'])
        self.renderGroundPlane(self.params[8], self.params[7])   # OW, OL

    def renderPoints(self):
        glPointSize(15.0)
        glBegin(GL_POINTS)
        glColor3f(1.0, 0.0, 0.0)
        for p in self.points:
            glVertex3f(p[0], p[1], p[2])
        glEnd()

    def renderQuad(self, v1, v2, v3, v4, normal, color):
        glNormal3f(*normal)
        glBegin(GL_QUADS)
        glColor3f(*color)
        glVertex3f(*v1); glVertex3f(*v2); glVertex3f(*v3); glVertex3f(*v4)
        glEnd()

    def renderCylinder(self, color, center, r, t):
        step = 15
        glColor3f(*color)
        glBegin(GL_TRIANGLES)
        for theta in range(0, 360, step):
            a0 = np.deg2rad(theta); a1 = np.deg2rad(theta + step)
            glVertex3f(center[0] - t / 2, center[1], center[2])
            glVertex3f(center[0] - t / 2, center[1] + r * np.cos(a0), center[2] + r * np.sin(a0))
            glVertex3f(center[0] - t / 2, center[1] + r * np.cos(a1), center[2] + r * np.sin(a1))
        glEnd()
        glColor3f(0, 0, 0)
        glBegin(GL_QUAD_STRIP)
        for theta in range(0, 360, step):
            a0 = np.deg2rad(theta); a1 = np.deg2rad(theta + step)
            glVertex3f(center[0] - t / 2, center[1] + r * np.cos(a0), center[2] + r * np.sin(a0))
            glVertex3f(center[0] - t / 2, center[1] + r * np.cos(a1), center[2] + r * np.sin(a1))
            glVertex3f(center[0] + t / 2, center[1] + r * np.cos(a0), center[2] + r * np.sin(a0))
            glVertex3f(center[0] + t / 2, center[1] + r * np.cos(a1), center[2] + r * np.sin(a1))
        glEnd()
        glColor3f(*color)
        glBegin(GL_TRIANGLES)
        for theta in range(0, 360, step):
            a0 = np.deg2rad(theta); a1 = np.deg2rad(theta + step)
            glVertex3f(center[0] + t / 2, center[1], center[2])
            glVertex3f(center[0] + t / 2, center[1] + r * np.cos(a0), center[2] + r * np.sin(a0))
            glVertex3f(center[0] + t / 2, center[1] + r * np.cos(a1), center[2] + r * np.sin(a1))
        glEnd()

    def renderGroundPlane(self, ow, ol):
        asphalt = (0.3, 0.3, 0.3)
        xLength, zLength = 4000, 6000
        xStart, zStart = ow / 2 - xLength / 2, -ol / 2 - zLength / 2
        xSharp, zSharp = 50, 75
        xSteps, zSteps = xLength / xSharp, zLength / zSharp
        for x in range(xSharp):
            for z in range(zSharp):
                v1 = (xStart + x * xSteps, 0, zStart + z * zSteps)
                v2 = (xStart + (x + 1) * xSteps, 0, zStart + z * zSteps)
                v3 = (xStart + (x + 1) * xSteps, 0, zStart + (z + 1) * zSteps)
                v4 = (xStart + x * xSteps, 0, zStart + (z + 1) * zSteps)
                color = asphalt if (x + z) % 2 == 0 else (0.4, 0.4, 0.4)
                self.renderQuad(v1, v2, v3, v4, (0.0, 1.0, 0.0), color)

    # ------------------------------------------------------------- interaction
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        if self.lastPos is not None:
            dx = event.x() - self.lastPos.x()
            dy = event.y() - self.lastPos.y()
            self.rotY += dx * 0.5
            self.rotX += dy * 0.5
            self.lastPos = event.pos()
            self.update()

    def keyPressEvent(self, event):
        spd = 50
        k = event.key()
        if k == Qt.Key_Left:   self.x += spd
        elif k == Qt.Key_Right: self.x -= spd
        elif k == Qt.Key_Up:    self.y -= spd
        elif k == Qt.Key_Down:  self.y += spd
        elif k == Qt.Key_W:     self.z += spd
        elif k == Qt.Key_S:     self.z -= spd
        self.update()


if __name__ == '__main__':
    import render_io
    row, points = render_io.select_car()

    app = QApplication(sys.argv)
    widget = GLWidget(points,
                      row["PRED_WWOM"], row["PRED_WB"],
                      row["PRED_OL"], row["PRED_OH"],
                      row["pred_heading_angle"], row["dist_to_move"])
    widget.show()
    sys.exit(app.exec_())
