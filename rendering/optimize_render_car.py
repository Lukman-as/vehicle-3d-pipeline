import sys
import numpy as np
import pandas as pd   # type: ignore
from OpenGL.GL import *   # type: ignore
from OpenGL.GLU import *   # type: ignore
from OpenGL.GLUT import *   # type: ignore
from PyQt5.QtWidgets import QApplication, QOpenGLWidget   # type: ignore
from PyQt5.QtCore import Qt   # type: ignore
from PyQt5.QtGui import QSurfaceFormat   # type: ignore
import torch    # type: ignore
import matplotlib.pyplot as plt

# torch.autograd.set_detect_anomaly(True)

def load_gaussian_model(filename):
    data = np.load(filename, allow_pickle=True)
    return (data['mu_a'], data['cov_ab'], data['cov_bb_inv'], data['mu_b'],
            data['a_part'].tolist(), data['b_part'].tolist())

def as_scalar(value):
    return float(np.asarray(value).reshape(-1)[0])

def clear_gl_errors():
    while glGetError() != GL_NO_ERROR:
        pass

def ud_quad(p, a, b, c, d, xp=torch, eps=1e-5):
    def dot(u, v): return torch.sum(u * v, dim=-1)
    def dot2(v): return torch.sum(v * v, dim=-1)
    def cross(u, v): return torch.cross(u, v, dim=-1)
    ba = b - a; pa = p - a
    cb = c - b; pb = p - b
    dc = d - c; pc = p - c
    ad = a - d; pd = p - d
    nor = cross(ba, ad)
    s = (torch.sign(dot(cross(ba, nor), pa)) +
         torch.sign(dot(cross(cb, nor), pb)) +
         torch.sign(dot(cross(dc, nor), pc)) +
         torch.sign(dot(cross(ad, nor), pd)))
    def seg_dist2(pa_, ba_):
        denom = dot2(ba_) + eps
        t = torch.clamp(dot(ba_, pa_) / denom, 0.0, 1.0)
        v = ba_ * t[..., None] - pa_
        return dot2(v)
    e0 = seg_dist2(pa, ba)
    e1 = seg_dist2(pb, cb)
    e2 = seg_dist2(pc, dc)
    e3 = seg_dist2(pd, ad)
    min_edge = torch.min(torch.stack([e0, e1, e2, e3], axis=-1), axis=-1)[0]
    numer = dot(nor, pa)
    plane_d2 = (numer * numer) / (dot2(nor) + eps)
    use_edges = s < 3.0
    d2 = torch.where(use_edges, min_edge, plane_d2)
    return torch.sqrt(d2.abs() + eps)
       

def distance_to_cylinder(point, center, radius, height, axis):
    # Distance to tire-contact point only
    bottom_center = center.clone()
    bottom_center[1] = 0  # Project center to ground plane (y=0)
    bottom_center += (height/2)*axis
    radial_vec = point - bottom_center
    radial_dist = torch.norm(radial_vec)
    return torch.abs(radial_dist)



class GLWidget(QOpenGLWidget):
    def __init__(self, points_file, pred_ow, pred_wb, pred_ol, pred_oh, pred_heading_angle, dist_to_move, parent=None):
        super().__init__(parent)
        glutInit(sys.argv)
        self.setMinimumSize(1000, 1000)
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Load 3D points from points.csv (assuming no header, comma-separated)
        if isinstance(points_file, np.ndarray):
            self.points = np.array(points_file, dtype=float)
        else:
            points_df = pd.read_csv(points_file, header=None)
            self.points = points_df.values  # Shape: (num_points, 3)
        
        # Original: x=length, y=height (down), z=width
        # Rendering: x=width, y=height (up), z=-length (away)
        transformed_points = np.zeros_like(self.points)
        transformed_points[:, 0] = -self.points[:, 2]  # x = z (width)
        transformed_points[:, 1] = -self.points[:, 1]  # y = -y (flip height up)
        transformed_points[:, 2] = -self.points[:, 0]  # z = -x (length away)
        self.points = transformed_points

        # Scale points to cm (assuming original in meters)
        self.points *= 100
        
        # center the point cloud
        mean_x = np.mean(self.points[:, 0])  
        self.points[:, 0] -= mean_x
        min_y = np.min(self.points[:, 1])  
        self.points[:, 1] -= min_y
        # mean_z = np.mean(self.points[:, 2])  
        # self.points[:, 2] -= mean_z
        min_z = np.min(self.points[:, 2])  
        self.points[:, 2] -= min_z
        
        # Initial view
        self.x, self.y, self.z = 0, 0, -1000 
        self.rotX, self.rotY, self.rotZ = 10.0, 0.0, 0.0
        self.lastPos = None
        
        # Set pose and parameters
        self.pred_heading_angle = as_scalar(pred_heading_angle) + 180
        self.dist_to_move = (0, 0, 0)  # Assume scalar along z, scaled to cm
        self.param_names = ['A', 'C', 'D', 'E', 'F', 'G', 'OH', 'OL', 'OW', 'TW']
        self.params = self.get_conditional_params(pred_ow * 100, pred_wb * 100, pred_ol * 100, pred_oh * 100)  # Scale to cm
        # self.params = np.array([119.0, 43.0, 82.0, 102.0, 94.0, 112.0, 169, 491.0, 192.0,  164.0])    # example dimension of a real car

        self.anchor = (0, 0, 0)
        self.geometry = self.get_geometry(self.anchor, self.params)  # Compute geometry for distances
        self.mean = np.mean(self.points, axis=0)
        # print(self.geometry["center_car"])
        self.optimize_fit()

    def initializeGL(self):
        self.makeCurrent()
        context = self.context()
        if context is None or not context.isValid():
            raise RuntimeError("OpenGL context was not created successfully.")
        clear_gl_errors()
        glClearColor(0, 0, 0, 1)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        self.resizeGL(self.width(), self.height())

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [100.0, 1000.0, -1000.0, 0.5])  # Overhead light
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])     # White light
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
        glEnable(GL_COLOR_MATERIAL)  # Colors affect material
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])  # Moderate shine
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0)  # Adjust for gloss (0-128)

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

        glPushMatrix()
        glTranslatef(self.dist_to_move[0], self.dist_to_move[1], self.dist_to_move[2])
        glRotatef(self.pred_heading_angle, 0, 1, 0)
        self.renderCar(self.anchor, self.params)
        glPopMatrix()

        self.renderPoints()
        # self.renderCoordinateArrows()

        # # reder center of the car and point cloud
        # glBegin(GL_LINES)
        # glColor3f(0, 10, 0)  # Y green -> points
        # glVertex3f(self.mean[0], self.mean[1], self.mean[2]); glVertex3f(self.mean[0], 500, self.mean[2])
        # glColor3f(0, 0, 1)  # Z blue -> car
        # glVertex3f(self.geometry["center_car"][0], self.geometry["center_car"][1], self.geometry["center_car"][2]); glVertex3f(self.geometry["center_car"][0], 500, self.geometry["center_car"][2])
        # glEnd()

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
        result[9] = params[6]   # TW
        return result
    
    def get_geometry(self, anchor, params):
        a, c, d, e, f, g, oh, ol, ow, tw = params
        tire_radius = 43.18
        tire_width = ow - tw
        
        # Compute vertices 
        v1 = np.array([anchor[0] + (ow - e)/2, anchor[1] + (oh - c - d), anchor[2]])
        v2 = np.array([anchor[0] + (ow - e)/2, anchor[1] + (oh - c - d*(1/5)), anchor[2]])
        v3 = np.array([anchor[0] + ow - (ow - e)/2, anchor[1] + (oh - c - d*(1/5)), anchor[2]])
        v4 = np.array([anchor[0] + ow - (ow - e)/2, anchor[1] + (oh - c - d), anchor[2]])
        v5 = np.array([anchor[0], anchor[1] + (oh - c - d), anchor[2] - a*(1/5)])
        v6 = np.array([anchor[0], anchor[1] + (oh - c), anchor[2] - a*(1/5)])
        v7 = np.array([anchor[0] + ow, anchor[1] + (oh - c), anchor[2] - a*(1/5)])
        v8 = np.array([anchor[0] + ow, anchor[1] + (oh - c - d), anchor[2] - a*(1/5)])
        v9 = np.array([anchor[0], anchor[1] + (oh - c), anchor[2] - a])
        v10 = np.array([anchor[0] + ow, anchor[1] + (oh - c), anchor[2] - a])
        v11 = np.array([anchor[0] + (ow - e)/2, anchor[1] + oh, anchor[2] - a - (ol - a)*(2/10)])
        v12 = np.array([anchor[0]+ ow - (ow - e)/2, anchor[1] + oh, anchor[2] - a - (ol - a)*(2/10)])
        v13 = np.array([anchor[0]+ (ow - e)/2, anchor[1] + oh, anchor[2] - (ol - (ol - a)*(1/10))])
        v14 = np.array([anchor[0] + ow - (ow - e)/2, anchor[1] + oh, anchor[2] - (ol - (ol - a)*(1/10))])
        v15 = np.array([anchor[0], anchor[1] + (oh - c), anchor[2] - ol])
        v16 = np.array([anchor[0] + ow, anchor[1] + (oh - c), anchor[2] - ol])
        v17 = np.array([anchor[0], anchor[1] + (oh - c - d), anchor[2] - ol])
        v18 = np.array([anchor[0] + ow, anchor[1] + (oh - c - d), anchor[2] - ol])

        # Center all body vertices 
        body_points = [v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18]
        x_shift = ow / 2.0  # Center in x (symmetry)
        z_shift = ol / 2.0  # Center in z (front-to-rear; since z ranges ~0 to -ol, +ol/2 brings mean ~0)

        for v in body_points:
            v[0] -= x_shift
            v[2] += z_shift

        # Quads: list of dicts {'vertices': [v1,v2,v3,v4], 'normal': computed normal}
        quads = []
        def compute_normal(verts):
            edge1 = verts[1] - verts[0]
            edge2 = verts[3] - verts[0]
            normal = np.cross(edge1, edge2)
            normal /= np.linalg.norm(normal)
            return normal
        
        quads.append({'vertices': [v1, v4, v3, v2], 'normal': compute_normal([v1, v4, v3, v2])})
        quads.append({'vertices': [v1, v2, v6, v5], 'normal': compute_normal([v1, v2, v6, v5])})
        quads.append({'vertices': [v2, v3, v7, v6], 'normal': compute_normal([v2, v3, v7, v6])})
        quads.append({'vertices': [v3, v4, v8, v7], 'normal': compute_normal([v3, v4, v8, v7])})
        quads.append({'vertices': [v6, v7, v10, v9], 'normal': compute_normal([v6, v7, v10, v9])})
        quads.append({'vertices': [v9, v10, v12, v11], 'normal': compute_normal([v9, v10, v12, v11])})
        quads.append({'vertices': [v11, v12, v14, v13], 'normal': compute_normal([v11, v12, v14, v13])})
        quads.append({'vertices': [v13, v14, v16, v15], 'normal': compute_normal([v13, v14, v16, v15])})
        quads.append({'vertices': [v15, v16, v18, v17], 'normal': compute_normal([v15, v16, v18, v17])})
        quads.append({'vertices': [v10, v16, v14, v12], 'normal': compute_normal([v10, v16, v14, v12])})
        quads.append({'vertices': [v8, v18, v16, v7], 'normal': compute_normal([v8, v18, v16, v7])})
        quads.append({'vertices': [v5, v6, v15, v17], 'normal': compute_normal([v5, v6, v15, v17])})
        quads.append({'vertices': [v9, v11, v13, v15], 'normal': compute_normal([v9, v11, v13, v15])})
        quads.append({'vertices': [v5, v17, v18, v8], 'normal': compute_normal([v5, v17, v18, v8])})
        
        # Cylinders: list of dicts {'center': center, 'radius': r, 'height': tire_width, 'axis': [0,0,1]} assume z-axis for tires
        cylinders = []
        offset = 5  # tires shift outside the car
        v19 = np.array([anchor[0] + ow - (ow - tw)/2 + offset, anchor[1] + tire_radius, anchor[2] - f])
        v20 = np.array([anchor[0] + (ow - tw)/2 - offset, anchor[1] + tire_radius, anchor[2] - f])
        v21 = np.array([anchor[0] + ow - (ow - tw)/2 + offset, anchor[1] + tire_radius, anchor[2] - (ol - g)])
        v22 = np.array([anchor[0] + (ow - tw)/2 - offset, anchor[1] + tire_radius, anchor[2] - (ol - g)])

        axis_right = np.array([1, 0, 0])  # Positive x outward for right
        axis_left = np.array([-1, 0, 0])  # Negative x outward for left
        cylinders.append({'center': v19, 'radius': tire_radius, 'height': tire_width, 'axis': axis_right})  # Right front
        cylinders.append({'center': v20, 'radius': tire_radius, 'height': tire_width, 'axis': axis_left})   # Left front
        cylinders.append({'center': v21, 'radius': tire_radius, 'height': tire_width, 'axis': axis_right})  # Right rear
        cylinders.append({'center': v22, 'radius': tire_radius, 'height': tire_width, 'axis': axis_left})   # Left rear
        
        # Center all body vertices 
        tire_points = [v19, v20, v21, v22]
        for v in tire_points:
            v[0] -= x_shift
            v[2] += z_shift

        # Center of car
        all_points = np.vstack(body_points + tire_points)
        center_car = np.mean(all_points, axis=0)
        return {'quads': quads, 'cylinders': cylinders, 'center_car': center_car}

    def renderPoints(self):
        glPointSize(15.0)
        glBegin(GL_POINTS)
        glColor3f(1.0, 0.0, 0.0)  # Red color
        for point in self.points:
            glVertex3f(point[0], point[1], point[2])
        glEnd()

    def renderCar(self, anchor, params):
        # Use get_geometry to get quads and cylinders, but render them
        geom = self.get_geometry(anchor, params)
        red = (1, 0, 0)
        brown_light = (150/256, 75/256, 0/256)
        brown_regular = (140 / 256, 67 / 256, 0 / 256)
        near_black = (50/256, 50/256, 50/256)
        gray_dark = (100/256, 100/256, 100/256)
        blue_metallic = (0.2, 0.4, 0.8)  # Vibrant blue with a hint of shine under lighting
        black = (0.1, 0.1, 0.1)         # Near-black for tires
        
        for q in geom['quads']:
            verts = q['vertices']
            self.renderQuad(verts[0], verts[1], verts[2], verts[3], q["normal"], blue_metallic)
        
        for cyl in geom['cylinders']:
            self.renderCylinder(black, cyl['center'], cyl['radius'], cyl['height'])
            # print("center:", cyl["center"], "r:", cyl["radius"], "h:", cyl["height"], "axis:", cyl["axis"])
        self.renderGroundPlane(anchor, params[8], params[7])  # ow, ol

        # # Render quads with labels
        # for idx, q in enumerate(geom['quads']):
        #     verts = q['vertices']
        #     color = blue_metallic
        #     self.renderQuad(verts[0], verts[1], verts[2], verts[3], q["normal"], color)
            
        #     # Compute quad center and normal for offset
        #     center = np.mean(verts, axis=0)
        #     normal = q['normal']  # Use precomputed normal from get_geometry
            
        #     # Offset label along normal to avoid hiding inside
        #     offset_distance = 10.0  # cm; adjust for visibility
        #     label_pos = center + normal * offset_distance
            
        #     # Draw label (number) at center
        #     glPushMatrix()
        #     glTranslatef(label_pos[0], label_pos[1], label_pos[2])
        #     glColor3f(1.0, 0.0, 0.0)  # White text
        #     glRasterPos3f(0, 0, 0)
        #     label = str(idx + 1)  # Quad 1 to 14
        #     for char in label:
        #         glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, ord(char))
        #     glPopMatrix()
        
        # # Render cylinders with labels
        # for idx, cyl in enumerate(geom['cylinders']):
        #     self.renderCylinder(black, cyl['center'], cyl['radius'], cyl['height'])
        
        #     # Draw label above cylinder center to avoid hiding
        #     center = cyl['center']
        #     offset_distance = cyl['height']/2 + 5  # Above top of tire
        #     label_pos = np.array([center[0] + offset_distance, center[1], center[2]])

        #     glPushMatrix()
        #     glTranslatef(label_pos[0], label_pos[1], label_pos[2])
        #     glColor3f(1.0, 0.0, 0.0)  # White text
        #     glRasterPos3f(0, 0, 0)
        #     label = f"C{idx + 1}"  # Tire 1 to 4
        #     for char in label:
        #         glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(char))
        #     glPopMatrix()

    def optimize_fit(self):
        mean_points = np.mean(self.points, axis=0)
        geom = self.get_geometry((0, 0, 0), self.params)
        center_car = geom["center_car"]
        dis = mean_points - center_car
        translation = torch.tensor(dis, requires_grad=True, dtype=torch.float)
        angle = torch.tensor(self.pred_heading_angle, requires_grad=True, dtype=torch.float)
        optimizer = torch.optim.RMSprop([translation, angle], lr=0.8, momentum=0.5)  # Tune lr: 0.1–10.0
        points_t = torch.from_numpy(self.points).float()
        losses = []

        for i in range(50):
            optimizer.zero_grad()
            min_dists = []
            c = torch.cos(torch.deg2rad(angle))
            s = torch.sin(torch.deg2rad(angle))
            z = torch.zeros_like(c)  # Creates a zero tensor matching c's shape, dtype, and device
            o = torch.ones_like(c)   # Creates a one tensor similarly

            rot = torch.stack([
                torch.stack([c, z, s]),
                torch.stack([z, o, z]),
                torch.stack([-s, z, c])
            ])

            for p in points_t:
                if p[1] > 0.5:
                    p_t = p - translation
                    p_t = rot.T @ p_t
                    md = torch.tensor(float('inf'))
                    for q in geom['quads']:
                        verts = [torch.tensor(v, dtype=torch.float) for v in q['vertices']]
                        md = torch.min(md, ud_quad(p_t, *verts))
                else:
                    p_t = p - translation
                    p_t = rot.T @ p_t
                    md = torch.tensor(float('inf'))
                    for cyl in geom['cylinders']:
                        center = torch.tensor(cyl['center'], dtype=torch.float)
                        axis = torch.tensor(cyl['axis'], dtype=torch.float)
                        md = torch.min(md, distance_to_cylinder(p_t, center, cyl['radius'], cyl['height'], axis) * 5)
                min_dists.append(md)

            loss = torch.mean(torch.abs(torch.stack(min_dists))**2)
            losses.append(loss.item())
            loss.backward()
            # print(f"Iter {i}: Loss: {loss.detach().item():.2f}, Anchor: {translation.detach().numpy()}, Theta: {angle.detach().item():.2f} Theta grad: {angle.grad.detach().item()}")
            optimizer.step()
            

        # Plot loss vs. iterations
        # plt.figure(figsize=(8, 6))
        # plt.plot(losses, label='Loss (Mean Distance)')
        # plt.xlabel('Iteration')
        # plt.ylabel('Loss (cm)')
        # plt.title('Loss vs. Iteration')
        # plt.legend()
        # plt.grid(True)
        # plt.savefig('loss_vs_iteration.png')
        # plt.show()

        # xz = params[:3].detach().numpy()
        # self.dist_to_move = (xz[0], 0.0, xz[2])
        self.dist_to_move = tuple(translation.detach().numpy())
        self.pred_heading_angle = angle.detach().item()
        self.angle = self.pred_heading_angle
        # self.geometry = self.get_geometry((0, 0, 0), self.params)
        self.update()
       

    def renderGroundPlane(self, anchor, ow, ol):
        color = (1.0, 1.0, 1.0)     # white
        asphalt_gray = (0.3, 0.3, 0.3)  # Darker road-like ground
        gray_light = (200 / 256, 200 / 256, 200 / 256)
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
                color = asphalt_gray if (x + z) % 2 == 0 else (0.4, 0.4, 0.4)  # Alternate shades
                self.renderQuad(v1, v2, v3, v4, (0.0, 1.0, 0.0), color)

    def renderQuad(self, v1, v2, v3, v4, normal, color):
        glNormal3f(*normal)  # Apply normal for lighting
        glBegin(GL_QUADS)
        glColor3f(*color)
        glVertex3f(*v1)
        glVertex3f(*v2)
        glVertex3f(*v3)
        glVertex3f(*v4)
        glEnd()

    def renderCylinder(self, color, center, r, t):
        step = 15
        # FACE 1 OF CYLINDER
        glColor3f(*color)
        glBegin(GL_TRIANGLES)
        for theta in range(0, 360, step):
            arc_start = np.deg2rad(theta)
            arc_end = np.deg2rad(theta + step)
            glVertex3f(center[0] - t/2, center[1], center[2])
            glVertex3f(center[0] - t/2, center[1] + r * np.cos(arc_start), center[2] + r * np.sin(arc_start))
            glVertex3f(center[0] - t/2, center[1] + r * np.cos(arc_end), center[2] + r * np.sin(arc_end))
        glEnd()
        # SIDE OF CYLINDER
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
        # FACE 2 OF CYLINDER
        glColor3f(*color)
        glBegin(GL_TRIANGLES)
        for theta in range(0, 360, step):
            arc_start = np.deg2rad(theta)
            arc_end = np.deg2rad(theta + step)
            glVertex3f(center[0] + t/2, center[1], center[2])
            glVertex3f(center[0] + t/2, center[1] + r * np.cos(arc_start), center[2] + r * np.sin(arc_start))
            glVertex3f(center[0] + t/2, center[1] + r * np.cos(arc_end), center[2] + r * np.sin(arc_end))
        glEnd()

    def renderCoordinateArrows(self):
        glBegin(GL_LINES)
        glColor3f(1, 0, 0)  # X red
        glVertex3f(-40, 5, 0); glVertex3f(40, 5, 0)
        glVertex3f(40, 5, 0); glVertex3f(30, 15, 0)
        glVertex3f(40, 5, 0); glVertex3f(30, -15, 0)
        glColor3f(0, 10, 0)  # Y green
        glVertex3f(0, -45, 0); glVertex3f(0, 45, 0)
        glVertex3f(0, 45, 0); glVertex3f(10, 35, 0)
        glVertex3f(0, 45, 0); glVertex3f(-10, 35, 0)
        glColor3f(0, 0, 1)  # Z blue
        glVertex3f(0, 5, -40); glVertex3f(0, 5, 40)
        glVertex3f(0, 5, 40); glVertex3f(0, 15, 30)
        glVertex3f(0, 5, 40); glVertex3f(0, -15, 30)
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
    import render_io
    row, points = render_io.select_car()

    app = QApplication(sys.argv)
    widget = GLWidget(points,
                      row["PRED_WWOM"], row["PRED_WB"],
                      row["PRED_OL"], row["PRED_OH"],
                      row["pred_heading_angle"], row["dist_to_move"])
    widget.show()
    sys.exit(app.exec_())
