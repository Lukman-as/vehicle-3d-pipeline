import pygame   # type: ignore
from pygame.locals import *   # type: ignore
from OpenGL.GL import *   # type: ignore
from OpenGL.GLU import *   # type: ignore
import pandas as pd   # type: ignore

# Load CSV points
import os, sys
_default = os.path.join(os.path.dirname(os.path.abspath(__file__)), "samples", "points0.csv")
points_df = pd.read_csv(sys.argv[1] if len(sys.argv) > 1 else _default, header=None)
points_df.columns = ['x', 'y', 'z']
points = points_df.values.tolist()

# Normalize to center the model
import numpy as np
points = np.array(points)
center = points.mean(axis=0)
points -= center
points = points.tolist()

def draw_points():
    glBegin(GL_POINTS)
    for point in points:
        glVertex3f(point[0]*10, -point[1]*10, point[2]*10)
    glEnd()

def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    
    gluPerspective(45, (display[0] / display[1]), 0.1, 1000.0)
    glTranslatef(0.0, 0.0, -100.0)  # Move camera back to see the object

    glEnable(GL_DEPTH_TEST)
    glPointSize(3)  # Size of each point

    clock = pygame.time.Clock()
    
    rotate_x = 0
    rotate_y = 0
    rotate_z = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Rotate the model
        # rotate_x += 0.3
        # rotate_y += 0.5
        rotate_z += 0.3

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()
        # glRotatef(rotate_x, 1, 0, 0)
        # glRotatef(rotate_y, 0, 1, 0)
        glRotatef(rotate_z, 0, 1, 0)

        draw_points()

        glPopMatrix()
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
