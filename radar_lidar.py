"""
Radar LIDAR - Interface Python pour ESP32 + Sharp GP2Y0A21YK0F
Connexion : USB/Serial à 115200 baud
Format des trames : "angle_deg;dist_mm\n"
"""

import sys
import queue
import threading
import collections

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import time
import serial
import serial.tools.list_ports

# ── Paramètres ────────────────────────────────────────────────────────────────

BAUDRATE      = 115200
MAX_POINTS    = 400       # Nombre de points conservés en mémoire
ANNOTATE_LAST = 15        # Nombre des derniers points annotés avec la distance
ANIM_INTERVAL = 50        # ms entre deux frames (~20 fps)
R_MAX         = 85        # cm – limite du radar
GRID_RINGS    = [20, 40, 60, 80]  # Cercles de grille (cm)
FADE_SECONDS  = 6.0               # Durée avant disparition complète d'un point

# Couleurs style radar sombre
BG_COLOR      = "#0a0a1a"
GRID_COLOR    = "#1a3a1a"
TICK_COLOR    = "#2a6a2a"
TEXT_COLOR    = "#88ff88"
POINT_CMAP    = "RdYlGn"   # rouge=proche, vert=loin

# ── Détection automatique du port ─────────────────────────────────────────────

ESP_KEYWORDS = ["esp32", "cp210", "ch340", "ch341", "ftdi", "usb serial", "uart"]

def find_serial_port() -> str:
    """Tente de détecter automatiquement le port de l'ESP32."""
    ports = list(serial.tools.list_ports.comports())

    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(kw in desc or kw in hwid for kw in ESP_KEYWORDS):
            candidates.append(p.device)

    if len(candidates) == 1:
        print(f"[INFO] Port détecté automatiquement : {candidates[0]}")
        return candidates[0]

    if len(candidates) > 1:
        print("[INFO] Plusieurs ports ESP32 détectés :")
        for i, c in enumerate(candidates):
            print(f"  [{i}] {c}")
        choice = input("Sélectionnez le numéro du port : ").strip()
        return candidates[int(choice)]

    # Aucun port trouvé automatiquement → liste complète
    if not ports:
        print("[ERREUR] Aucun port série disponible.")
        sys.exit(1)

    print("[INFO] Ports série disponibles :")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  —  {p.description}")
    choice = input("Sélectionnez le numéro du port ESP32 : ").strip()
    return ports[int(choice)].device


# ── Thread de lecture Serial ──────────────────────────────────────────────────

def serial_reader(port: str, baudrate: int, data_queue: queue.Queue, stop_event: threading.Event):
    """
    Python prend le contrôle total du port série :
      - Envoie START pour déclencher le moteur et l'acquisition sur l'ESP32.
      - Lit les trames angle;dist_mm et les pousse dans la queue.
      - Envoie STOP avant de fermer (fermeture fenêtre ou Ctrl-C).
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[INFO] Connecté sur {port} à {baudrate} baud")
    except serial.SerialException as e:
        print(f"[ERREUR] Impossible d'ouvrir {port} : {e}")
        stop_event.set()
        return

    # L'ouverture du port reset l'ESP32 (signal DTR).
    # On attend qu'il termine son boot avant d'envoyer START.
    print("[INFO] Attente du démarrage de l'ESP32…")
    time.sleep(2)
    ser.reset_input_buffer()
    ser.write(b"START\n")
    print("[INFO] Commande START envoyée — acquisition en cours")

    try:
        while not stop_event.is_set():
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                parts = line.split(";")
                if len(parts) != 2:
                    continue
                angle_deg = float(parts[0])
                dist_mm   = float(parts[1])
                dist_cm   = dist_mm / 10.0
                # Filtrer les valeurs hors plage
                if not (0.0 <= angle_deg <= 360.0) or not (10.0 <= dist_cm <= 80.0):
                    continue
                data_queue.put((angle_deg, dist_cm, time.time()))
            except (ValueError, UnicodeDecodeError):
                continue
            except serial.SerialException:
                print("[ERREUR] Connexion série perdue.")
                stop_event.set()
    finally:
        # Arrêt propre : stoppe le moteur côté ESP32
        try:
            ser.write(b"STOP\n")
            print("[INFO] Commande STOP envoyée — moteur arrêté")
        except Exception:
            pass
        ser.close()


# ── Application Radar ─────────────────────────────────────────────────────────

class RadarApp:
    def __init__(self, port: str):
        self.data_queue  = queue.Queue()
        self.stop_event  = threading.Event()
        self.points      = collections.deque(maxlen=MAX_POINTS)  # (angle_rad, dist_cm)
        self.total_count = 0

        # Démarrage du thread Serial
        self.thread = threading.Thread(
            target=serial_reader,
            args=(port, BAUDRATE, self.data_queue, self.stop_event),
            daemon=True,
        )
        self.thread.start()

        # ── Figure ──────────────────────────────────────────────────────────
        self.fig = plt.figure(figsize=(8, 8), facecolor=BG_COLOR)
        self.fig.canvas.manager.set_window_title("Radar LIDAR — ESP32 Sharp")

        self.ax = self.fig.add_subplot(111, projection="polar")
        self._setup_axes()

        # Scatter plot (initialisé vide)
        self.scatter = self.ax.scatter([], [], s=25, zorder=5)

        # Ligne de balayage (suit l'angle courant du moteur)
        self.sweep_line, = self.ax.plot(
            [0, 0], [0, R_MAX],
            color="#44ff44", linewidth=1.5, alpha=0.85, zorder=4
        )
        self.last_angle = 0.0

        # Annotations de distance (pool fixe, masquées par défaut)
        self.annotations = [
            self.ax.text(0, 0, "", color="white", fontsize=7,
                         ha="center", va="center", zorder=6,
                         visible=False)
            for _ in range(ANNOTATE_LAST)
        ]

        # Texte d'info (coin supérieur gauche en coordonnées figure)
        self.info_text = self.fig.text(
            0.02, 0.97, "", va="top", ha="left",
            color=TEXT_COLOR, fontsize=9,
            fontfamily="monospace",
            transform=self.fig.transFigure,
        )

        # Colorbar
        sm = plt.cm.ScalarMappable(cmap=POINT_CMAP,
                                   norm=mcolors.Normalize(vmin=10, vmax=80))
        sm.set_array([])
        cbar = self.fig.colorbar(sm, ax=self.ax, pad=0.08, fraction=0.03)
        cbar.set_label("Distance (cm)", color=TEXT_COLOR, fontsize=9)
        cbar.ax.yaxis.set_tick_params(color=TEXT_COLOR)
        plt.setp(cbar.ax.yaxis.get_ticklabels(), color=TEXT_COLOR)
        cbar.outline.set_edgecolor(GRID_COLOR)

        self.fig.canvas.mpl_connect("close_event", self._on_close)

    # ── Configuration des axes polaires ─────────────────────────────────────

    def _setup_axes(self):
        ax = self.ax
        ax.set_facecolor(BG_COLOR)
        ax.set_theta_zero_location("N")   # 0° en haut
        ax.set_theta_direction(-1)         # Sens horaire
        ax.set_rlim(0, R_MAX)
        ax.set_rticks(GRID_RINGS)
        ax.set_yticklabels([f"{r} cm" for r in GRID_RINGS],
                           color=TEXT_COLOR, fontsize=8)
        ax.tick_params(axis="x", colors=TEXT_COLOR, labelsize=8)
        ax.grid(color=GRID_COLOR, linewidth=0.8)
        ax.spines["polar"].set_color(GRID_COLOR)
        ax.set_title("Radar LIDAR — Sharp GP2Y0A21YK0F",
                     color=TEXT_COLOR, pad=15, fontsize=11)

    # ── Mise à jour de l'animation ───────────────────────────────────────────

    def update(self, frame):
        # Vider la queue
        new_data = False
        while not self.data_queue.empty():
            try:
                angle_deg, dist_cm, ts = self.data_queue.get_nowait()
                self.points.append((np.radians(angle_deg), dist_cm, ts))
                self.total_count += 1
                new_data = True
            except queue.Empty:
                break

        if not self.points:
            return self.scatter, self.sweep_line, *self.annotations, self.info_text

        angles = np.array([p[0] for p in self.points])
        dists  = np.array([p[1] for p in self.points])

        # Mise à jour du scatter avec fondu temporel
        now    = time.time()
        ages   = np.array([now - p[2] for p in self.points])
        alphas = np.clip(1.0 - ages / FADE_SECONDS, 0.05, 1.0)
        cmap   = matplotlib.colormaps[POINT_CMAP]
        norm   = mcolors.Normalize(vmin=10, vmax=80)
        colors = cmap(norm(dists))   # (N, 4) RGBA
        colors[:, 3] = alphas

        self.scatter.set_offsets(np.column_stack([angles, dists]))
        self.scatter.set_facecolor(colors)

        # Ligne de balayage à l'angle courant du moteur
        if self.points:
            self.last_angle = self.points[-1][0]
        self.sweep_line.set_data([self.last_angle, self.last_angle], [0, R_MAX])

        # Annotations sur les ANNOTATE_LAST derniers points
        recent = list(self.points)[-ANNOTATE_LAST:]
        for i, ann in enumerate(self.annotations):
            if i < len(recent):
                a, d, _ = recent[-(i + 1)]
                ann.set_position((a, d))
                ann.set_text(f"{d:.0f} cm")
                ann.set_visible(True)
            else:
                ann.set_visible(False)

        # Texte d'info
        last_a, last_d, _ = self.points[-1]
        self.info_text.set_text(
            f"Points reçus : {self.total_count}\n"
            f"Dernier angle : {np.degrees(last_a):.1f}°\n"
            f"Dernière dist : {last_d:.1f} cm"
        )

        return self.scatter, self.sweep_line, *self.annotations, self.info_text

    # ── Lancement ────────────────────────────────────────────────────────────

    def run(self):
        self.anim = animation.FuncAnimation(
            self.fig, self.update,
            interval=ANIM_INTERVAL,
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show()
        # Garantir l'envoi de STOP avant que le processus se termine
        self.stop_event.set()
        self.thread.join(timeout=3)

    def _on_close(self, event):
        self.stop_event.set()


# ── Point d'entrée ────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"[INFO] Utilisation du port spécifié : {port}")
    else:
        port = find_serial_port()

    app = RadarApp(port)
    app.run()

