import sys
import os
import matplotlib

matplotlib.use("Agg")  # Non-interactive backend (no display needed)
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.typesys import Stores, get_typestore
from rosbags.rosbag2 import Reader

# ── Joint classification ──────────────────────────────────────────────
# Pick-and-place joints (arm + gripper) used to compute G_parcial
PICK_PLACE_JOINTS = {
    "lower_link_joint",
    "middle_link_joint",
    "upper_link_joint",
    "end_link_joint",
    "wrist_link_joint",
    "left_finger_link_joint",
    "right_finger_link_joint",
}

# Wheel joints for position graph
WHEEL_KEYWORDS = ["wheel"]


def is_wheel_joint(name):
    return any(kw in name.lower() for kw in WHEEL_KEYWORDS)


def analyze_bag(bag_path, out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # IMU data
    time_imu = []
    accel_x, accel_y, accel_z = [], [], []

    # Wheel position data
    wheel_positions = {}  # name -> [(t, pos), ...]

    # Pick-and-place effort → G_parcial
    gasto_times = []
    gasto_values = []

    print(f"Leyendo rosbag en: {bag_path}")

    try:
        with Reader(Path(bag_path)) as reader:
            # Show available topics
            for conn in reader.connections:
                print(f"  Topic: {conn.topic} ({conn.msgtype})")

            for conn, timestamp, rawdata in reader.messages():
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

                # ── IMU ──
                if conn.topic in ["/imu", "/imu/data"]:
                    time_imu.append(timestamp / 1e9)
                    accel_x.append(msg.linear_acceleration.x)
                    accel_y.append(msg.linear_acceleration.y)
                    accel_z.append(msg.linear_acceleration.z)

                # ── Joint States ──
                elif conn.topic == "/joint_states":
                    t = timestamp / 1e9
                    g_partial = 0.0
                    has_pp_data = False

                    for i, name in enumerate(msg.name):
                        pos = msg.position[i] if i < len(msg.position) else 0.0
                        eff = msg.effort[i] if i < len(msg.effort) else 0.0

                        # Wheel positions
                        if is_wheel_joint(name):
                            if name not in wheel_positions:
                                wheel_positions[name] = []
                            wheel_positions[name].append((t, pos))

                        # Pick-and-place effort → G_parcial
                        if name in PICK_PLACE_JOINTS:
                            g_partial += abs(eff)
                            has_pp_data = True

                    if has_pp_data:
                        gasto_times.append(t)
                        gasto_values.append(g_partial)

    except Exception as e:
        print(f"Error al leer el rosbag: {e}")
        import traceback

        traceback.print_exc()
        return

    # ── Compute time origin ──
    all_starts = []
    if time_imu:
        all_starts.append(time_imu[0])
    if gasto_times:
        all_starts.append(gasto_times[0])
    for data in wheel_positions.values():
        if data:
            all_starts.append(data[0][0])

    if not all_starts:
        print("No se encontraron datos de /imu o /joint_states en el rosbag.")
        return

    start_time = min(all_starts)
    time_imu = [t - start_time for t in time_imu]
    gasto_times = [t - start_time for t in gasto_times]

    # ═══════════════════════════════════════════════════════════════════
    # Gráfica 1: Posición de Ruedas vs Tiempo
    # ═══════════════════════════════════════════════════════════════════
    plt.figure(figsize=(12, 6))
    for name, data in sorted(wheel_positions.items()):
        if data:
            t_vals, pos_vals = zip(*data)
            t_vals = [t - start_time for t in t_vals]
            plt.plot(t_vals, pos_vals, label=name, linewidth=1.2)
    plt.title("Posición de las Ruedas vs Tiempo", fontsize=14)
    plt.xlabel("Tiempo (s)", fontsize=12)
    plt.ylabel("Posición (rad)", fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path1 = os.path.join(out_dir, "grafica_posicion_ruedas.png")
    plt.savefig(path1, dpi=150)
    print(f"Guardado {path1}")
    plt.close()

    # ═══════════════════════════════════════════════════════════════════
    # Gráfica 2: Aceleración (IMU) vs Tiempo
    # ═══════════════════════════════════════════════════════════════════
    plt.figure(figsize=(12, 6))
    if time_imu:
        plt.plot(time_imu, accel_x, label="Accel X", linewidth=1.2)
        plt.plot(time_imu, accel_y, label="Accel Y", linewidth=1.2)
        plt.plot(time_imu, accel_z, label="Accel Z", linewidth=1.2)
    else:
        plt.text(
            0.5,
            0.5,
            "No se encontraron datos de IMU en el rosbag\n"
            "(el topic /imu no fue grabado)",
            ha="center",
            va="center",
            fontsize=14,
            transform=plt.gca().transAxes,
            color="red",
        )
    plt.title("Aceleración (IMU) vs Tiempo", fontsize=14)
    plt.xlabel("Tiempo (s)", fontsize=12)
    plt.ylabel("Aceleración (m/s²)", fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path2 = os.path.join(out_dir, "grafica_aceleracion_imu.png")
    plt.savefig(path2, dpi=150)
    print(f"Guardado {path2}")
    plt.close()

    # ═══════════════════════════════════════════════════════════════════
    # Gráfica 3: Gasto (G_parcial) vs Tiempo
    #   G_parcial = Σ |F_i|  for all pick-and-place joints
    # ═══════════════════════════════════════════════════════════════════
    plt.figure(figsize=(12, 6))
    if gasto_times:
        plt.plot(
            gasto_times, gasto_values, label="G_parcial", linewidth=1.2, color="tab:red"
        )
    plt.title("Gasto del Mecanismo Pick and Place vs Tiempo", fontsize=14)
    plt.xlabel("Tiempo (s)", fontsize=12)
    plt.ylabel("G_parcial = Σ|F_i| (N·m)", fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    path3 = os.path.join(out_dir, "grafica_gasto.png")
    plt.savefig(path3, dpi=150)
    print(f"Guardado {path3}")
    plt.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python3 graph_rosbag.py <ruta_al_rosbag> [directorio_salida]")
        sys.exit(1)

    bag_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "."
    analyze_bag(bag_path, out_dir)
