import sys
import numpy as np
import pandas as pd   # type: ignore
from OpenGL.GL import *   # type: ignore
from OpenGL.GLU import *   # type: ignore
from PyQt5.QtWidgets import QApplication, QOpenGLWidget   # type: ignore
from PyQt5.QtCore import Qt   # type: ignore

def load_gaussian_model(filename):
    data = np.load(filename, allow_pickle=True)
    return (data['mu_a'], data['cov_ab'], data['cov_bb_inv'], data['mu_b'],
            data['a_part'].tolist(), data['b_part'].tolist())

def as_scalar(value):
    return float(np.asarray(value).reshape(-1)[0])

class GLWidget(QOpenGLWidget):
    def __init__(self, pred_ow, pred_wb, pred_ol, pred_oh, pred_heading_angle, dist_to_move, parent=None):
        super().__init__(parent)
        self.setMinimumSize(1000, 1000)
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Load mean car parameters from CSV
        import os
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'priors', 'mean_car_shape_2011_2023.csv')
        df = pd.read_csv(csv_path)
        self.mean_params = df.loc[0, ['A', 'C', 'D', 'E', 'F', 'G', 'OH', 'OL', 'OW', 'TWF', 'TWR']].values
        
        # Load 3D points from points.csv (assuming no header, comma-separated)
        points_df = pd.read_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'samples', 'points0.csv'), header=None)
        self.points = points_df.values  # Shape: (num_points, 3)
        print(f"Loaded {len(self.points)} points")  # Debug: Check number of points
        
        # Swap Y and Z axes (as per your manual change)
        # self.points = self.points[:, [0, 2, 1]]  # New order: X, Z (old Y), Y (old Z)
        self.points[:, 1] = -self.points[:, 1]
        self.points[:, 2] = -self.points[:, 2]

        # Scale points to cm (assuming original in meters)
        self.points *= 100
        
        # center the point cloud
        mean_x = np.mean(self.points[:, 0])  
        self.points[:, 0] -= mean_x
        min_y = np.min(self.points[:, 1])  
        self.points[:, 1] -= min_y
        min_z = np.min(self.points[:, 2])  
        self.points[:, 2] -= min_z
        
        # Initial view
        self.x, self.y, self.z = 0, 0, -1000  # Centered and closer
        self.rotX, self.rotY, self.rotZ = 10.0, 0.0, 0.0
        self.lastPos = None

        # Set pose and parameters
        self.pred_heading_angle = as_scalar(pred_heading_angle)
        self.dist_to_move = (0, 0, as_scalar(dist_to_move) * 100)  # Assume scalar dist_to_move is along z-axis (length), scaled to cm
        self.param_names = ['A', 'C', 'D', 'E', 'F', 'G', 'OH', 'OL', 'OW', 'TW']
        self.params = self.get_conditional_params(pred_ow * 100, pred_wb * 100, pred_ol * 100, pred_oh * 100)  # Scale inputs to cm for Gaussian model

    def initializeGL(self):
        glClearColor(0, 0, 0, 1)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        self.resizeGL(self.width(), self.height())

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect_ratio = width / height if height != 0 else 1
        gluPerspective(45.0, aspect_ratio, 0.1, 7000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(self.x, self.y, self.z)
        glRotatef(self.rotX, 1, 0, 0)
        glRotatef(self.rotY, 0, 1, 0)
        glRotatef(self.rotZ, 0, 0, 1)

        self.renderCoordinateArrows()
        # self.renderCar((0, 0, 0), self.mean_params)
        self.renderGroundPlane((0, 0, 0), 200, 500)
        self.renderPoints()

    def get_conditional_params(self, pred_ow, pred_wb, pred_ol, pred_oh):
        import os
        filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'priors', 'models', 'gaussian_model_OW_WB_OH_OL.npz')
        mu_a, cov_ab, cov_bb_inv, mu_b, _, b_part = load_gaussian_model(filename)
        conditioning_values = {
            'OW': as_scalar(pred_ow),
            'WB': as_scalar(pred_wb),
            'OH': as_scalar(pred_oh),
            'OL': as_scalar(pred_ol),
        }
        b_vector = np.array([conditioning_values[name] for name in b_part], dtype=float)
        params = mu_a + cov_ab @ cov_bb_inv @ (b_vector - mu_b)
        
        # Map to param_names (FH=F, RH=G, TW for TWF/TWR)
        result = np.zeros(len(self.param_names))
        result[0:4] = params[0:4]  # A, C, D, E
        result[4] = params[4]      # F (FH)
        result[5] = params[5]      # G (RH)
        result[6] = conditioning_values['OH']  # OH (direct)
        result[7] = conditioning_values['OL']  # OL (direct)
        result[8] = conditioning_values['OW']  # OW (direct)
        result[9] = params[6]      # TW
        return result
    
    def renderPoints(self):
        glPointSize(10.0)
        glBegin(GL_POINTS)
        glColor3f(1.0, 0.0, 0.0)  # Red color
        for point in self.points:
            glVertex3f(point[0], point[1], point[2])
        glEnd()

    def renderCar(self, anchor, params):
        a, c, d, e, f, g, oh, ol, ow, twf, twr = params
        tire_radius = 43.18
        tire_width = ow - twf + 5
        
        v1 = (anchor[0] + (ow - e)/2, anchor[1] + (oh - c - d), anchor[2])
        v2 = (anchor[0] + (ow - e)/2, anchor[1] + (oh - c - d*(1/5)), anchor[2])
        v3 = (anchor[0] + ow - (ow - e)/2, anchor[1] + (oh - c - d*(1/5)), anchor[2])
        v4 = (anchor[0] + ow - (ow - e)/2, anchor[1] + (oh - c - d), anchor[2])
        v5 = (anchor[0], anchor[1] + (oh - c - d), anchor[2] - a*(1/5))
        v6 = (anchor[0], anchor[1] + (oh - c), anchor[2] - a*(1/5))
        v7 = (anchor[0] + ow, anchor[1] + (oh - c), anchor[2] - a*(1/5))
        v8 = (anchor[0] + ow, anchor[1] + (oh - c - d), anchor[2] - a*(1/5))
        v9 = (anchor[0], anchor[1] + (oh - c), anchor[2] - a)
        v10 = (anchor[0] + ow, anchor[1] + (oh - c), anchor[2] - a)
        v11 = (anchor[0] + (ow - e)/2, anchor[1] + oh, anchor[2] - a - (ol - a)*(2/10))
        v12 = (anchor[0] + ow - (ow - e)/2, anchor[1] + oh, anchor[2] - a - (ol - a)*(2/10))
        v13 = (anchor[0] + (ow - e)/2, anchor[1] + oh, anchor[2] - (ol - (ol - a)*(1/10)))
        v14 = (anchor[0] + ow - (ow - e)/2, anchor[1] + oh, anchor[2] - (ol - (ol - a)*(1/10)))
        v15 = (anchor[0], anchor[1] + (oh - c), anchor[2] - ol)
        v16 = (anchor[0] + ow, anchor[1] + (oh - c), anchor[2] - ol)
        v17 = (anchor[0], anchor[1] + (oh - c - d), anchor[2] - ol)
        v18 = (anchor[0] + ow, anchor[1] + (oh - c - d), anchor[2] - ol)
        
        v19 = (anchor[0] + ow - (ow - twf)/2, anchor[1] + tire_radius, anchor[2] - f)
        v20 = (anchor[0] + (ow - twf)/2, anchor[1] + tire_radius, anchor[2] - f)
        v21 = (anchor[0] + ow - (ow - twf)/2, anchor[1] + tire_radius, anchor[2] - (ol - g))
        v22 = (anchor[0] + (ow - twf)/2, anchor[1] + tire_radius, anchor[2] - (ol - g))
        
        brown_light = (150/256, 75/256, 0/256)
        near_black = (50/256, 50/256, 50/256)
        gray_dark = (100/256, 100/256, 100/256)
        
        self.renderQuad(v1, v2, v3, v4, brown_light)
        self.renderQuad(v1, v2, v6, v5, brown_light)
        self.renderQuad(v2, v3, v7, v6, brown_light)
        self.renderQuad(v3, v4, v8, v7, brown_light)
        self.renderQuad(v6, v7, v10, v9, brown_light)
        self.renderQuad(v9, v10, v12, v11, near_black)
        self.renderQuad(v11, v12, v14, v13, brown_light)
        self.renderQuad(v13, v14, v16, v15, near_black)
        self.renderQuad(v16, v15, v17, v18, brown_light)
        self.renderQuad(v10, v12, v14, v16, near_black)
        self.renderQuad(v8, v7, v16, v18, brown_light)
        self.renderQuad(v5, v6, v15, v17, brown_light)
        self.renderQuad(v9, v11, v13, v15, near_black)
        self.renderQuad(v5, v8, v18, v17, brown_light)
        
        self.renderCylinder(gray_dark, v19, tire_radius, tire_width)
        self.renderCylinder(gray_dark, v20, tire_radius, tire_width)
        self.renderCylinder(gray_dark, v21, tire_radius, tire_width)
        self.renderCylinder(gray_dark, v22, tire_radius, tire_width)
        
        self.renderGroundPlane(anchor, ow, ol)

    def renderQuad(self, v1, v2, v3, v4, color):
        glBegin(GL_QUADS)
        glColor3f(*color)
        glVertex3f(*v1)
        glVertex3f(*v2)
        glVertex3f(*v3)
        glVertex3f(*v4)
        glEnd()

    def renderCylinder(self, color, center, r, t):
        step = 15
        glColor3f(*color)
        glBegin(GL_TRIANGLES)
        for theta in range(0, 360, step):
            arc_start = np.deg2rad(theta)
            arc_end = np.deg2rad(theta + step)
            glVertex3f(center[0] - t/2, center[1], center[2])
            glVertex3f(center[0] - t/2, center[1] + r * np.cos(arc_start), center[2] + r * np.sin(arc_start))
            glVertex3f(center[0] - t/2, center[1] + r * np.cos(arc_end), center[2] + r * np.sin(arc_end))
        glEnd()
        glColor3f(0, 0, 0)
        glBegin(GL_QUAD_STRIP)
        for theta in range(0, 360, step):
            arc_start = np.deg2rad(theta)
            arc_end = np.deg2rad(theta + step)
            glVertex3f(center[0] - t/2, center[1] + r * np.cos(arc_start), center[2] + r * np.sin(arc_start))
            glVertex3f(center[0] - t/2, center[1] + r * np.cos(arc_end), center[2] + r * np.sin(arc_end))
            glVertex3f(center[0] + t/2, center[1] + r * np.cos(arc_start), center[2] + r * np.sin(arc_start))
            glVertex3f(center[0] + t/2, center[1] + r * np.cos(arc_end), center[2] + r * np.sin(arc_end))
        glEnd()
        glColor3f(*color)
        glBegin(GL_TRIANGLES)
        for theta in range(0, 360, step):
            arc_start = np.deg2rad(theta)
            arc_end = np.deg2rad(theta + step)
            glVertex3f(center[0] + t/2, center[1], center[2])
            glVertex3f(center[0] + t/2, center[1] + r * np.cos(arc_start), center[2] + r * np.sin(arc_start))
            glVertex3f(center[0] + t/2, center[1] + r * np.cos(arc_end), center[2] + r * np.sin(arc_end))
        glEnd()

    def renderGroundPlane(self, anchor, ow, ol):
        gray_dark = (100/256, 100/256, 100/256)
        gray_light = (200/256, 200/256, 200/256)
        xLength, zLength = 4000, 6000
        xStart, zStart = ow/2 - xLength/2, -ol/2 - zLength/2
        xSharpness, zSharpness = 50, 75
        xSteps, zSteps = xLength / xSharpness, zLength / zSharpness
        for x in range(xSharpness):
            for z in range(zSharpness):
                v1 = (xStart + x * xSteps, 0, zStart + z * zSteps)
                v2 = (xStart + (x + 1) * xSteps, 0, zStart + z * zSteps)
                v3 = (xStart + (x + 1) * xSteps, 0, zStart + (z + 1) * zSteps)
                v4 = (xStart + x * xSteps, 0, zStart + (z + 1) * zSteps)
                color = gray_light if (x + z) % 2 == 1 else gray_dark
                self.renderQuad(v1, v2, v3, v4, color)

    def renderCoordinateArrows(self):
        glBegin(GL_LINES)
        glColor3f(1, 0, 0)  # X red
        glVertex3f(-40, 0, 0); glVertex3f(40, 0, 0)
        glVertex3f(40, 0, 0); glVertex3f(30, 10, 0)
        glVertex3f(40, 0, 0); glVertex3f(30, -10, 0)
        glColor3f(0, 10, 0)  # Y green
        glVertex3f(0, -40, 0); glVertex3f(0, 40, 0)
        glVertex3f(0, 40, 0); glVertex3f(10, 30, 0)
        glVertex3f(0, 40, 0); glVertex3f(-10, 30, 0)
        glColor3f(0, 0, 1)  # Z blue
        glVertex3f(0, 0, -40); glVertex3f(0, 0, 40)
        glVertex3f(0, 0, 40); glVertex3f(0, 10, 30)
        glVertex3f(0, 0, 40); glVertex3f(0, -10, 30)
        glEnd()

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
        movement_speed = 50
        key = event.key()
        if key == Qt.Key_Left:
            self.x += movement_speed  # Move left (+X)
        elif key == Qt.Key_Right:
            self.x -= movement_speed  # Move right (-X)
        elif key == Qt.Key_Up:
            self.y -= movement_speed  # Move up (-Y)
        elif key == Qt.Key_Down:
            self.y += movement_speed  # Move down (+Y)
        elif key == Qt.Key_W:
            self.z += movement_speed  # Move forward / closer
        elif key == Qt.Key_S:
            self.z -= movement_speed  # Move backward / farther
        self.update()

if __name__ == '__main__':
    filename = "sample.txt"
    df = pd.read_csv(filename, sep=' ', header=None)
    col_names = ["camera","target", "annotated_car_id",
       "num_sym_pairs","bbox_2D_height",
             "reproj_error","gt_heading_angle",
             "pred_heading_angle","angle_difference",
             "dist_base_gt_bbox","dist_base_pred_bbox",
             "dist_base_bbox_diff","dist_nearest_corner_gt_bbox",
             "dist_nearest_corner_pred_bbox","dist_nearest_corner_diff",
             "iou","iou_bev","mounting_height","ds",
            "PRED_OL","PRED_OW","PRED_OH","PRED_WB","LD_OL","LD_OW","LD_OH","PRED_WWOM","tire_both_sides","has_mirrors","dist_to_move",
            "LD_OW_NON","LD_OH_NON","LD_OL_NON","LENGTH_BY_GAUSSIAN","NUM_TIRES"]
    df.columns = col_names
    # sample is the first car
    df1 = df.iloc[[0]]

    app = QApplication(sys.argv)
    widget = GLWidget(df1["PRED_OW"], df1["PRED_WB"], df1["PRED_OL"], df1["PRED_OH"],
                  df1["pred_heading_angle"], df1["dist_to_move"])
    widget.show()
    sys.exit(app.exec_())
