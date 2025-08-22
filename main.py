import tkinter as tk
import numpy as np
import math

class Path:
    def __init__(self):
        self.waypoints = []

    def add_line(self, start, end, num_points=20):
        x_vals = np.linspace(start[0], end[0], num_points)
        y_vals = np.linspace(start[1], end[1], num_points)
        self.waypoints += list(zip(x_vals, y_vals))

    def add_arc(self, center, radius, start_angle, end_angle, num_points=20):
        angles = np.linspace(start_angle, end_angle, num_points)
        x_vals = center[0] + radius * np.cos(angles)
        y_vals = center[1] + radius * np.sin(angles)
        self.waypoints += list(zip(x_vals, y_vals))

    def get_path(self):
        return self.waypoints
    
class Environment:
    def __init__(self, path, wind=(0,0), gravity=0, friction=0.1):
        self.path = path                # list of (x,y)
        self.wind = wind                # (wx, wy)
        self.gravity = gravity          # gaya ke arah y misalnya
        self.friction = friction        # koefisien gesekan

    def get_disturbance(self, v):
        """Hitung gangguan yang dialami robot"""
        wind_x, wind_y = self.wind
        drag = -self.friction * v
        return (wind_x, wind_y + self.gravity + drag)

    def get_path(self):
        return self.path
    
class Robot:
    def __init__(self, x=100, y=300, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0
        self.omega = 0
        self.shape = None  # referensi ke gambar di canvas

    def draw(self, canvas):
        r = 8  # radius robot
        if self.shape is None:
            self.shape = canvas.create_oval(self.x-r, self.y-r, self.x+r, self.y+r, fill="red")
        else:
            canvas.coords(self.shape, self.x-r, self.y-r, self.x+r, self.y+r)

    def step(self, v, omega, dt, env):
        self.v = v
        self.omega = omega

        dx_env, dy_env = env.get_disturbance(v)

        self.x += (v * math.cos(self.theta) + dx_env) * dt
        self.y += (v * math.sin(self.theta) + dy_env) * dt
        self.theta += omega * dt

# ====================
# Buat path U-turn + 90°
# ====================
path = Path()

path.add_line((100, 300), (300, 300))
path.add_arc(center=(300, 250), radius=50, start_angle=np.pi/2, end_angle=3*np.pi/2)
path.add_line((300, 200), (100, 200))
path.add_arc(center=(100, 150), radius=50, start_angle=np.pi/2, end_angle=0)

path.add_line((150, 100), (300, 100))

waypoints = path.get_path()

env = Environment(waypoints, wind=(0.5, 0), gravity=0.1, friction=0.05)

robot = Robot(x=100, y=300, theta=0)

root = tk.Tk()
root.title("Path Visualization (U-turn + 90°)")

canvas = tk.Canvas(root, width=600, height=400, bg="white")
canvas.pack()

# Gambar path biru
wp = env.get_path()
for i in range(len(wp) - 1):
    canvas.create_line(wp[i][0], wp[i][1], wp[i+1][0], wp[i+1][1], fill="blue", width=2)

# Animasi robot (jalan lurus dulu)
def animate():
    robot.step(v=2.0, omega=0.02, dt=1, env=env)  # nanti MPC yang tentukan v, omega
    robot.draw(canvas)
    root.after(50, animate)

animate()
root.mainloop()
