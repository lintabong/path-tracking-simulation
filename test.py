import tkinter as tk
import math
import time
import random
import openpyxl
import ctypes

ctypes.windll.shcore.SetProcessDpiAwareness(1)

class RobotSim:
    def __init__(self, canvas, path, env, control_mode=None):
        self.canvas = canvas
        self.path = path
        self.env = env
        self.index = 0
        self.robot = None
        self.running = False
        self.control_mode = control_mode

        # robot state
        self.x, self.y = path[0]
        self.theta = 0.0
        self.v = 2.0   # m/s
        self.omega = 0.0

        # data log
        self.log_data = []

    def draw_robot(self):
        r = 6
        if self.robot:
            self.canvas.delete(self.robot)
        self.robot = self.canvas.create_oval(
            self.x-r, self.y-r, self.x+r, self.y+r,
            fill="blue"
        )

    def step(self):
        if not self.running:
            return

        if self.index < len(self.path)-1:
            # referensi titik berikutnya
            x_ref, y_ref = self.path[self.index+1]
            dx = x_ref - self.x
            dy = y_ref - self.y
            dist = math.hypot(dx, dy)
            theta_ref = math.atan2(dy, dx)

            ey = -math.sin(self.theta) * dx + math.cos(self.theta) * dy
            ey += random.gauss(0, 0.1)   # noise sensor posisi
            
            etheta = theta_ref - self.theta
            etheta += random.gauss(0, 0.01)
            etheta = math.atan2(math.sin(etheta), math.cos(etheta))

            error_value = self.compute_accuracy()
            error_value = 0 if not error_value else error_value

            accuracy = round((100 - error_value), 4)
            accuracy = round((1 - error_value / 100) * 100, 4)
            accuracy_var.set(f"{str(accuracy)} %")
            print(f"ey={ey:.2f}, etheta={etheta:.2f}, error={error_value}")

            # simpan ke log
            self.log_data.append({
                "x": self.x,
                "y": self.y,
                "x_ref": x_ref,
                "y_ref": y_ref,
                "ey": ey,
                "etheta": etheta,
                "dist": dist
            })

            if self.control_mode.get() == "mpc":
                k_omega = 2.0
                self.omega = k_omega * etheta
            elif self.control_mode.get() == "fuzzy":
                k_omega = 2.0  # sementara sama dulu
                self.omega = k_omega * etheta
            else:
                k_omega = 2.0  # sementara sama dulu
                self.omega = k_omega * etheta

            # get wind parameter
            try:
                self.env["wind_x"] = float(wind_x_var.get())
                self.env["wind_y"] = float(wind_y_var.get())
            except ValueError:
                self.env["wind_x"] = 0.0
                self.env["wind_y"] = 0.0

            # update state (dengan efek angin)
            self.theta += self.omega * 0.1
            self.x += self.v * math.cos(self.theta) * 0.1 + self.env["wind_x"]*0.1
            self.y += self.v * math.sin(self.theta) * 0.1 + self.env["wind_y"]*0.1

            if dist < 5:
                self.index += 1

            self.draw_robot()
            self.canvas.after(5, self.step)

    def start(self):
        if not self.running:
            self.running = True
            self.step()

    def stop(self):
        self.running = False

    def reset(self):
        self.stop()
        self.x, self.y = self.path[0]
        self.theta = 0.0
        self.index = 0
        self.log_data = []
        self.draw_robot()

    def export_to_excel(self, filename="robot_log.xlsx"):
        wb = openpyxl.Workbook()
        ws = wb.active
        ws.title = "Log Data"
        ws.append(["x", "y", "x_ref", "y_ref", "ey", "etheta", "dist"])
        for row in self.log_data:
            ws.append([row["x"], row["y"], row["x_ref"], row["y_ref"], row["ey"], row["etheta"], row["dist"]])


        wb.save(filename)
        print(f"Data exported to {filename}")

    def compute_accuracy(self):
        if not self.log_data:
            return None
        ey_list = [row["ey"] for row in self.log_data]
        mse = sum(e**2 for e in ey_list) / len(ey_list)
        rmse = math.sqrt(mse)
        return rmse

# --- UI ---
root = tk.Tk()
root.title("Robot Path Tracking with Env")

frame_left = tk.Frame(root, bd=2, relief="solid", width=240)
frame_left.pack(side="left", fill="y", padx=10, pady=10)
frame_left.pack_propagate(False)  # biar width tetap, tidak mengecil otomatis

# kanan
frame_right = tk.Frame(root, bd=2, relief="solid")
frame_right.pack(side="right", expand=True, fill="both", padx=5, pady=5)

canvas_frame = tk.Frame(frame_right, bd=2, relief="solid", padx=10, pady=10)
canvas_frame.pack(expand=True, fill="both", padx=10, pady=10)

canvas = tk.Canvas(canvas_frame, width=500, height=400, bg="white")
canvas.pack()

lbl_acc = tk.Label(frame_left, text="Accuracy:")
lbl_acc.pack(anchor="w", padx=10)

accuracy_var = tk.StringVar()
entry_acc = tk.Entry(frame_left, textvariable=accuracy_var, width=20)
entry_acc.pack(padx=10, pady=5)

path = [(50,200),(150,200),(250,100),(350,200),(450,200),(450,300),(350,300),(250,200)]
for i in range(len(path)-1):
    canvas.create_line(path[i][0], path[i][1], path[i+1][0], path[i+1][1], fill="black")

# environment (contoh angin ke kanan)
env = {"wind_x":0.5, "wind_y":0.0}
canvas.create_line(30,30,60,30, arrow=tk.LAST, fill="red", width=2)
canvas.create_text(80,30, text="Wind")

control_var = tk.StringVar(value="simple")

robot = RobotSim(canvas, path, env, control_var)
robot.draw_robot()

lbl_mode = tk.Label(frame_left, text="Control Mode", font=("Arial", 10, "bold"))
lbl_mode.pack(pady=(5,0))

radio_simple = tk.Radiobutton(frame_left, text="Simple", variable=control_var, value="simple")
radio_simple.pack(anchor="w", padx=10)
radio_fuzzy = tk.Radiobutton(frame_left, text="Fuzzy", variable=control_var, value="fuzzy")
radio_fuzzy.pack(anchor="w", padx=10)
radio_mpc = tk.Radiobutton(frame_left, text="MPC", variable=control_var, value="mpc")
radio_mpc.pack(anchor="w", padx=10)

# --- tambahan variabel wind ---
wind_x_var = tk.StringVar(value="0.0")
wind_y_var = tk.StringVar(value="0.0")

lbl_wind = tk.Label(frame_left, text="Wind Settings", font=("Arial", 10, "bold"))
lbl_wind.pack(anchor="w", padx=10, pady=(10,0))

frame_wind = tk.Frame(frame_left)
frame_wind.pack(padx=10, pady=5, anchor="w")

lbl_wx = tk.Label(frame_wind, text="Wind X:")
lbl_wx.grid(row=0, column=0, sticky="w")
entry_wx = tk.Entry(frame_wind, textvariable=wind_x_var, width=8)
entry_wx.grid(row=0, column=1, padx=5)

lbl_wy = tk.Label(frame_wind, text="Wind Y:")
lbl_wy.grid(row=1, column=0, sticky="w")
entry_wy = tk.Entry(frame_wind, textvariable=wind_y_var, width=8)
entry_wy.grid(row=1, column=1, padx=5)

btn_frame = tk.Frame(frame_left)
btn_frame.pack(pady=10)

btn_start = tk.Button(btn_frame, text="Start", width=8, command=robot.start)
btn_start.pack(side="left", padx=2)

btn_stop = tk.Button(btn_frame, text="Stop", width=8, command=robot.stop)
btn_stop.pack(side="left", padx=2)

btn_reset = tk.Button(btn_frame, text="Reset", width=8, command=robot.reset)
btn_reset.pack(side="left", padx=2)

btn_export = tk.Button(frame_left, text="Export to Excel", command=robot.export_to_excel)
btn_export.pack(pady=10)

root.mainloop()
