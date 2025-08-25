import tkinter as tk
import math
import time
import random
import openpyxl
import ctypes

ctypes.windll.shcore.SetProcessDpiAwareness(1)

# === MPC: shooting dengan kontrol konstan di horizon ===
def mpc_control(ey, etheta, v, dt=0.1,
                omega_max=2.5,  # batas kecepatan sudut (rad/s)
                H=10,           # panjang horizon (langkah)
                qy=1.0, qt=0.6, r=0.05, grid_step=0.25):
    """
    MPC: pilih ω konstan selama horizon yang meminimalkan
    cost Σ(qy*ey^2 + qt*etheta^2 + r*ω^2).
    Model error diskret:
      ey_{k+1}     = ey_k + v*sin(etheta_k)*dt
      etheta_{k+1} = etheta_k + ω*dt
    """
    best_cost = float("inf")
    best_omega = 0.0

    # kandidat ω pada grid
    w = -omega_max
    while w <= omega_max + 1e-9:
        ey_k = ey
        th_k = etheta
        J = 0.0
        for _ in range(H):
            # akumulasi cost
            J += qy*(ey_k**2) + qt*(th_k**2) + r*(w**2)
            # propagate model
            ey_k = ey_k + v*math.sin(th_k)*dt
            th_k = th_k + w*dt
            # wrap heading error ke [-pi, pi]
            th_k = math.atan2(math.sin(th_k), math.cos(th_k))
        if J < best_cost:
            best_cost = J
            best_omega = w
        w += grid_step
    return best_omega


# === Fuzzy Sugeno untuk ω ===
def _tri(x, a, b, c):
    """Triangular membership."""
    if x <= a or x >= c: return 0.0
    if x == b: return 1.0
    if x < b:  return (x - a) / (b - a)
    return (c - x) / (c - b)

def _mf_ey(ey, ey_scale=0.5):
    """
    Membership ey (meter):
      NG ~ <-ey_scale, ZE ~ 0, PG ~ >+ey_scale
    """
    NG = _tri(ey, -10*ey_scale, -ey_scale, 0.0)
    ZE = _tri(ey, -ey_scale, 0.0, ey_scale)
    PG = _tri(ey, 0.0, ey_scale, 10*ey_scale)
    return {"NG": NG, "ZE": ZE, "PG": PG}

def _mf_th(etheta, th_scale=0.15):
    """
    Membership etheta (rad):
      NG ~ -th_scale, ZE ~ 0, PG ~ +th_scale
    """
    NG = _tri(etheta, -10*th_scale, -th_scale, 0.0)
    ZE = _tri(etheta, -th_scale, 0.0, th_scale)
    PG = _tri(etheta, 0.0, th_scale, 10*th_scale)
    return {"NG": NG, "ZE": ZE, "PG": PG}

def fuzzy_control(ey, etheta, omega_max=2.5):
    """
    Fuzzy Sugeno (singleton consequents). Output langsung ω.
    Rules ringkas:
      1) etheta NG -> NB
      2) etheta PG -> PB
      3) etheta ZE & ey NG -> PM
      4) etheta ZE & ey PG -> NM
      5) etheta ZE & ey ZE -> ZE
      6) etheta NG & ey PG -> NM
      7) etheta PG & ey NG -> PM
    Konsekuen (singleton):
      NB=-1.0, NM=-0.6, ZE=0, PM=+0.6, PB=+1.0 (× omega_max)
    """
    Ey = _mf_ey(ey)
    Th = _mf_th(etheta)

    # singleton (skala relatif, nanti dikali omega_max)
    NB, NM, ZE, PM, PB = -1.0, -0.6, 0.0, 0.6, 1.0

    # rule firing strengths
    w1 = Th["NG"]                           # -> NB
    w2 = Th["PG"]                           # -> PB
    w3 = min(Th["ZE"], Ey["NG"])            # -> PM
    w4 = min(Th["ZE"], Ey["PG"])            # -> NM
    w5 = min(Th["ZE"], Ey["ZE"])            # -> ZE
    w6 = min(Th["NG"], Ey["PG"])            # -> NM
    w7 = min(Th["PG"], Ey["NG"])            # -> PM

    # agregasi sug.e no (weighted average)
    num = (w1*NB + w2*PB + w3*PM + w4*NM + w5*ZE + w6*NM + w7*PM)
    den = (w1 + w2 + w3 + w4 + w5 + w6 + w7)

    out_rel = 0.0 if den == 0 else (num / den)
    omega = max(-omega_max, min(omega_max, out_rel * omega_max))
    return omega


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

            dt=0.1
            if self.control_mode.get() == "mpc":
                self.omega = mpc_control(
                    ey=ey, 
                    etheta=etheta, 
                    v=self.v,
                    dt=float(mpc_dt_var.get()),
                    omega_max=float(mpc_omega_max_var.get()),
                    H=int(mpc_H_var.get()),
                    qy=float(mpc_qy_var.get()),
                    qt=float(mpc_qt_var.get()),
                    r=float(mpc_r_var.get()),
                    grid_step=float(mpc_grid_var.get())
                )
            elif self.control_mode.get() == "fuzzy":
                self.omega = fuzzy_control(
                    ey=ey, 
                    etheta=etheta, 
                    omega_max=2.5
                )
            else:
                k_omega = 2.0
                self.omega = k_omega * etheta

            # get wind parameter
            try:
                self.env["wind_x"] = float(wind_x_var.get())
                self.env["wind_y"] = float(wind_y_var.get())
            except ValueError:
                self.env["wind_x"] = 0.0
                self.env["wind_y"] = 0.0

            # update state
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
        print(f'Data exported to {filename}')

    def compute_accuracy(self):
        if not self.log_data:
            return None
        ey_list = [row["ey"] for row in self.log_data]
        mse = sum(e**2 for e in ey_list) / len(ey_list)
        rmse = math.sqrt(mse)
        return rmse

# --- UI ---
root = tk.Tk()
root.title('MPC Path Tracking Simulation')

w, h = 800, 650

# ukuran layar
w_screen = root.winfo_screenwidth()
h_screen = root.winfo_screenheight()

x = (w_screen // 2) - (w // 2)
y = (h_screen // 2) - (h // 2)

root.geometry(f"{w}x{h}+{x}+{y}")
root.resizable(False, False)

frame_left = tk.Frame(root, bd=2, relief='solid', width=240)
frame_left.pack(side='left', fill='y', padx=10, pady=10)
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

env = {"wind_x":0.5, "wind_y":0.0}
canvas.create_line(30,30,60,30, arrow=tk.LAST, fill="red", width=2)
canvas.create_text(80,30, text="Wind")

control_var = tk.StringVar(value="fuzzy")

mpc_dt_var = tk.StringVar(value="0.1")
mpc_omega_max_var = tk.StringVar(value="2.5")
mpc_H_var = tk.StringVar(value="10")
mpc_qy_var = tk.StringVar(value="1.0")
mpc_qt_var = tk.StringVar(value="0.6")
mpc_r_var = tk.StringVar(value="0.05")
mpc_grid_var = tk.StringVar(value="0.25")

robot = RobotSim(canvas, path, env, control_var)
robot.draw_robot()

lbl_mode = tk.Label(frame_left, text="Control Mode", font=("Arial", 10, "bold"))
lbl_mode.pack(pady=(5,0))

# --- frame setting MPC (awal disembunyikan) ---
frame_mpc = tk.LabelFrame(frame_left, text="MPC Settings", padx=10, pady=5)
frame_mpc.pack(fill="x", padx=10, pady=5)
frame_mpc.pack_forget()  # disembunyikan dulu

def toggle_mpc_settings():
    if control_var.get() == "mpc":
        frame_mpc.pack(fill="x", padx=10, pady=5)
    else:
        frame_mpc.pack_forget()

# isi frame_mpc
labels = ["dt","omega_max","H","qy","qt","r","grid_step"]
vars_ = [mpc_dt_var, mpc_omega_max_var, mpc_H_var, mpc_qy_var, mpc_qt_var, mpc_r_var, mpc_grid_var]

for i,(lbl,var) in enumerate(zip(labels,vars_)):
    tk.Label(frame_mpc, text=lbl).grid(row=i, column=0, sticky="w")
    tk.Entry(frame_mpc, textvariable=var, width=8).grid(row=i, column=1, padx=5, pady=2)

radio_fuzzy = tk.Radiobutton(frame_left, text="Fuzzy", variable=control_var, value="fuzzy", command=toggle_mpc_settings)
radio_fuzzy.pack(anchor="w", padx=10)
radio_mpc = tk.Radiobutton(frame_left, text="MPC", variable=control_var, value="mpc", command=toggle_mpc_settings)
radio_mpc.pack(anchor="w", padx=10)

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
