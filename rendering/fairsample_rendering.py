"""
Fair-sample a vehicle category's joint Gaussian and render the draws.

This is the GENERATIVE demo (not reconstruction): it does NOT take a real car
as input.  You name a category, it draws fair (unbiased) samples from that
category's 11-dimensional joint Gaussian over the vehicle dimensions, and
renders each sampled vehicle as the parametric POLYGON model from
originalrender.py (car_models_core_new.build_geometry -> watertight body quads +
tire cylinders), driven by the sampled dimensions.  Several samples are shown
side by side so the diversity of the category is visible at a glance.

The joint Gaussian (mean mu, full covariance Sigma over all 11 variables) is
estimated directly from priors/cars_database/cleans_data/<type>.csv -- the same
data the predict_a_from_b models were fit on.  A fair sample is then:
    x = mu + L z,   z ~ N(0, I),   Sigma = L L^T   (Cholesky)
so each draw appears with frequency proportional to its true density.

Run:
    python fairsample_rendering.py sedan          # 4 fair sedans
    python fairsample_rendering.py truck 6        # 6 fair trucks
    python fairsample_rendering.py suv --seed 0   # reproducible draw
"""

import argparse
import sys

import numpy as np
import pandas as pd   # type: ignore
from OpenGL.GL import *   # type: ignore
from OpenGL.GLU import *   # type: ignore
from OpenGL.GLUT import *   # type: ignore
from PyQt5.QtWidgets import QApplication, QOpenGLWidget   # type: ignore
from PyQt5.QtCore import Qt   # type: ignore

import car_models_core_new as core   # parametric polygon model (same as originalrender.py)
import render_io   # for canonical_type (Car -> sedan, etc.)
from pathlib import Path

# The 11 joint variables (a_part + b_part of the predict_a_from_b model).
# FH/RH are the CSV's F/G columns; TW is the mean of TWF/TWR (see
# eval/get_multivariate_model_and_make_model.py).
JOINT_VARS = ['A', 'C', 'D', 'E', 'FH', 'RH', 'TW', 'OW', 'WB', 'OH', 'OL']

# Map a sampled JOINT_VARS vector to the geometry param vector that
# build_geometry expects: PARAM_NAMES = [A, C, D, E, F, G, OH, OL, OW, TW].
# F=FH, G=RH; WB is not placed geometrically (it only conditions the prior).
def to_geometry_params(x, idx):
    return np.array([x[idx['A']], x[idx['C']], x[idx['D']], x[idx['E']],
                     x[idx['FH']], x[idx['RH']], x[idx['OH']], x[idx['OL']],
                     x[idx['OW']], x[idx['TW']]], dtype=float)


BLACK = (0.1, 0.1, 0.1)

ROOT = Path(__file__).resolve().parents[1]
CARS_DATA = ROOT / 'priors' / 'cars_database' / 'cleans_data'

TYPE_COLORS = {
    'sedan': (0.20, 0.40, 0.80),   # blue
    'suv':   (0.20, 0.65, 0.30),   # green
    'truck': (0.85, 0.45, 0.10),   # orange
    'van':   (0.55, 0.30, 0.70),   # purple
}


def clear_gl_errors():
    while glGetError() != GL_NO_ERROR:
        pass


# ---------------------------------------------------------------- sampling
def build_joint_gaussian(vehicle_type, cars_data=CARS_DATA):
    """Estimate (mu, Sigma) of the 11 joint variables from <type>.csv."""
    csv = Path(cars_data) / f'{vehicle_type}.csv'
    if not csv.is_file():
        sys.exit(f"no data file {csv}")
    df = pd.read_csv(csv)
    df.columns = df.columns.str.strip()              # 'OH ', 'TWR ' -> 'OH','TWR'
    df = df.rename(columns={'F': 'FH', 'G': 'RH'})
    df['TW'] = (pd.to_numeric(df['TWF'], errors='coerce')
                + pd.to_numeric(df['TWR'], errors='coerce')) / 2.0
    sdf = df[JOINT_VARS].apply(pd.to_numeric, errors='coerce').dropna()
    if len(sdf) < len(JOINT_VARS) + 1:
        sys.exit(f"not enough usable rows in {csv} ({len(sdf)})")
    mu = sdf.mean().values
    sigma = np.cov(sdf.values, rowvar=False)
    print(f"[{vehicle_type}] joint Gaussian from {len(sdf)} cars "
          f"({len(JOINT_VARS)} vars)")
    return mu, sigma


def fair_samples(mu, sigma, n, rng):
    """Draw n fair samples x = mu + L z (Cholesky).  Re-draws a sample only if
    a physical dimension comes out non-positive (impossible, not unlikely)."""
    L = np.linalg.cholesky(sigma + 1e-6 * np.eye(len(mu)))
    idx = {name: i for i, name in enumerate(JOINT_VARS)}
    out = []
    while len(out) < n:
        x = mu + L @ rng.standard_normal(len(mu))
        if x[idx['OW']] > 0 and x[idx['OH']] > 0 and x[idx['OL']] > 0:
            out.append(x)
    return np.array(out), idx


# ---------------------------------------------------------------- rendering
class GLWidget(QOpenGLWidget):
    def __init__(self, vehicle_type, samples, idx, parent=None):
        super().__init__(parent)
        glutInit(sys.argv)
        self.setMinimumSize(1400, 900)
        self.setFocusPolicy(Qt.StrongFocus)

        self.vehicle_type = vehicle_type
        self.body_color = TYPE_COLORS.get(vehicle_type, (0.6, 0.6, 0.6))
        self.samples = samples
        self.idx = idx
        self.n = len(samples)

        # One parametric polygon geometry (quads + tire cylinders) per sample,
        # built from the sampled dimensions -- same model as originalrender.py.
        self.geoms = []
        for x in samples:
            params = to_geometry_params(x, idx)
            self.geoms.append(core.build_geometry(params, vehicle_type))

        # View: slight 3/4 angle.
        self.x, self.y = 0.0, -40.0
        self.rotX, self.rotY, self.rotZ = 14.0, -28.0, 0.0
        self.lastPos = None

        # Lay the cars out in a row along X.  Each car's LENGTH (OL) runs along
        # Z, so at the 3/4 view angle it projects onto screen-X by OL*sin(angle);
        # size the spacing from that rotated footprint (not just OW) or the cars
        # telescope into each other and rear wheels show under front bodies.
        ang = np.radians(abs(self.rotY))
        max_ow = float(np.max(samples[:, idx['OW']]))
        max_ol = float(np.max(samples[:, idx['OL']]))
        footprint = max_ow * np.cos(ang) + max_ol * np.sin(ang)
        self.spacing = max(footprint * 1.15 + 60.0, 260.0)

        # Pull back far enough to frame the whole row.
        self.z = -(self.spacing * self.n + 900.0)

    # ------------------------------------------------------------ GL setup
    def initializeGL(self):
        self.makeCurrent()
        ctx = self.context()
        if ctx is None or not ctx.isValid():
            raise RuntimeError("OpenGL context was not created successfully.")
        clear_gl_errors()
        glClearColor(0.05, 0.05, 0.08, 1)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        self.resizeGL(self.width(), self.height())
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [100.0, 1000.0, -1000.0, 0.5])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.25, 0.25, 0.25, 1.0])
        glEnable(GL_LIGHT1)
        glLightfv(GL_LIGHT1, GL_POSITION, [-600.0, 500.0, 1200.0, 0.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.55, 0.55, 0.55, 1.0])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.40, 0.40, 0.40, 1.0])
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0)
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = width / height if height != 0 else 1
        gluPerspective(45.0, aspect, 0.1, 20000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(self.x, self.y, self.z)
        glRotatef(self.rotX, 1, 0, 0)
        glRotatef(self.rotY, 0, 1, 0)
        glRotatef(self.rotZ, 0, 0, 1)

        first = -(self.n - 1) / 2.0 * self.spacing
        for i in range(self.n):
            x_off = first + i * self.spacing
            glPushMatrix()
            glTranslatef(x_off, 0.0, 0.0)
            self.renderCar(self.geoms[i])
            ow = self.samples[i, self.idx['OW']]
            ol = self.samples[i, self.idx['OL']]
            self.renderGroundPatch(ow, ol)
            glPopMatrix()

    # ---- parametric polygon model (quads + tire cylinders), as originalrender.py ----
    def renderCar(self, geom):
        for q in geom['quads']:
            v = q['vertices']
            self.renderQuad(v[0], v[1], v[2], v[3], q['normal'], self.body_color)
        for cyl in geom['cylinders']:
            self.renderCylinder(BLACK, cyl['center'], cyl['radius'], cyl['height'])

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

    def renderGroundPatch(self, ow, ol):
        """Small checkerboard pad under each car so it doesn't float."""
        half_x, half_z = ow * 1.4, ol * 0.9
        nx, nz = 8, 12
        sx, sz = (2 * half_x) / nx, (2 * half_z) / nz
        glBegin(GL_QUADS)
        glNormal3f(0, 1, 0)
        for ix in range(nx):
            for iz in range(nz):
                x0 = -half_x + ix * sx
                z0 = -half_z + iz * sz
                shade = 0.30 if (ix + iz) % 2 == 0 else 0.40
                glColor3f(shade, shade, shade)
                glVertex3f(x0, 0, z0)
                glVertex3f(x0 + sx, 0, z0)
                glVertex3f(x0 + sx, 0, z0 + sz)
                glVertex3f(x0, 0, z0 + sz)
        glEnd()

    # ------------------------------------------------------------ interaction
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

    def wheelEvent(self, event):
        self.z += event.angleDelta().y()
        self.update()

    def keyPressEvent(self, event):
        spd = 60
        k = event.key()
        if k == Qt.Key_Left:    self.x += spd
        elif k == Qt.Key_Right: self.x -= spd
        elif k == Qt.Key_Up:    self.y -= spd
        elif k == Qt.Key_Down:  self.y += spd
        elif k == Qt.Key_W:     self.z += spd
        elif k == Qt.Key_S:     self.z -= spd
        self.update()

def main():
    ap = argparse.ArgumentParser(description="Fair-sample and render a vehicle category.")
    ap.add_argument("vehicle_type", help="sedan | suv | truck | van  (also accepts 'car')")
    ap.add_argument("n", nargs="?", type=int, default=4, help="number of samples (default 4)")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed for reproducible draws")
    ap.add_argument("--cars-data", default=str(CARS_DATA), help="cleans_data folder")
    args, _ = ap.parse_known_args()   # let Qt flags pass through

    vt = render_io.canonical_type(args.vehicle_type) or args.vehicle_type.lower()
    if vt not in TYPE_COLORS:
        ap.error(f"unknown vehicle type {args.vehicle_type!r}; "
                 f"choose from {', '.join(TYPE_COLORS)}")

    rng = np.random.default_rng(args.seed)
    mu, sigma = build_joint_gaussian(vt, args.cars_data)
    samples, idx = fair_samples(mu, sigma, args.n, rng)

    # Report each fair draw (left -> right in the window).
    print(f"\n{args.n} fair sample(s) of {vt.upper()}  (cm):")
    head = "  #  " + "  ".join(f"{v:>6}" for v in JOINT_VARS)
    print(head)
    for i, x in enumerate(samples):
        print(f"  {i:<2} " + "  ".join(f"{x[idx[v]]:6.1f}" for v in JOINT_VARS))

    app = QApplication(sys.argv)
    widget = GLWidget(vt, samples, idx)
    widget.setWindowTitle(f"Fair samples - {vt} (n={args.n})")
    widget.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()