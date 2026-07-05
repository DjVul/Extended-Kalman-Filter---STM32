import numpy as np
import serial
import struct
import time
import matplotlib.pyplot as plt

# =============================================================================
# 1. KONFIGURACIJA
# =============================================================================

# IMU parametri
DURATION = 10.0          # sekundi
DT = 0.01               # 100 Hz
GYRO_NOISE_STD = 0.05
GYRO_BIAS = [0.015, -0.008, 0.025]
ACCEL_NOISE_STD = 0.3
G = 9.81

# UART parametri
PORT = 'COM5'           # Promeni!
BAUDRATE = 115200
TIMEOUT = 0.1

SCALE = 1000.0


def generate_data():
    np.random.seed(42)
    t = np.arange(0, DURATION, DT)
    N = len(t)

    # Uglovi (istina)
    roll_true = 0.3 * np.sin(0.8 * t) + 0.1 * np.sin(1.2 * t + 0.5)
    pitch_true = 0.2 * np.cos(0.6 * t) + 0.15 * np.sin(0.9 * t + 0.3)
    yaw_true = 0.1 * t + 0.2 * np.sin(0.3 * t)
    true_angles = np.column_stack([roll_true, pitch_true, yaw_true])

    # Žiroskop
    true_gyro = np.gradient(true_angles, DT, axis=0)
    gyro_meas = true_gyro + GYRO_BIAS + np.random.normal(0, GYRO_NOISE_STD, (N, 3))

    # Akcelerometar
    ax_true = -G * np.sin(pitch_true)
    ay_true = G * np.cos(pitch_true) * np.sin(roll_true)
    az_true = G * np.cos(pitch_true) * np.cos(roll_true)
    accel_meas = np.column_stack([ax_true, ay_true, az_true]) + np.random.normal(0, ACCEL_NOISE_STD, (N, 3))

    return t, gyro_meas, accel_meas, true_angles


def pack(gyro, accel):
    """6 x int16 → 12 bajtova"""
    gx = int(np.round(gyro[0] * SCALE))
    gy = int(np.round(gyro[1] * SCALE))
    gz = int(np.round(gyro[2] * SCALE))
    ax = int(np.round(accel[0] * SCALE))
    ay = int(np.round(accel[1] * SCALE))
    az = int(np.round(accel[2] * SCALE))
    return struct.pack('<6h', gx, gy, gz, ax, ay, az)

def unpack(data):
    """12 bajtova → 3 x float (roll, pitch, yaw)"""
    if len(data) >= 12:
        return struct.unpack('<3f', data[:12])
    return None


def main():
    print(" Generisanje podataka...")
    t, gyro, accel, true_angles = generate_data()
    N = len(t)
    print(f" {N} uzoraka")

    # Otvori UART
    print(f" Otvaram {PORT} @ {BAUDRATE} baud...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)  # Čekaj da se STM32 resetuje
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("UART otvoren")

    # Inicijalizuj nizove za rezultate
    roll_filt = np.zeros(N)
    pitch_filt = np.zeros(N)
    yaw_filt = np.zeros(N)
    roll_raw = np.zeros(N)
    pitch_raw = np.zeros(N)

    print(" Slanje podataka...")
    start_time = time.time()

    for i in range(N):
        # 1. Pošalji paket
        ser.write(pack(gyro[i], accel[i]))

        # 2. Čekaj odgovor (12 bajtova)
        data = ser.read(12)
        if len(data) == 12:
            roll, pitch, yaw = unpack(data)
            roll_filt[i] = roll
            pitch_filt[i] = pitch
            yaw_filt[i] = yaw
        else:
            print(f" Greška na uzorku {i}: primljeno {len(data)} bajtova")
            roll_filt[i] = 0
            pitch_filt[i] = 0
            yaw_filt[i] = 0

        # 3. Izračunaj sirove uglove iz akcelerometra (za poređenje)
        ax, ay, az = accel[i]
        roll_raw[i] = np.arctan2(ay, az)
        pitch_raw[i] = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # Ispis statusa
        if (i + 1) % 100 == 0:
            print(f" {i+1}/{N}")

    elapsed = time.time() - start_time
    ser.close()
    print(f" Gotovo za {elapsed:.1f}s")

    print(" Crtanje...")
    plt.style.use('dark_background')
    
    COLOR_TRUE = '#00ff88'
    COLOR_RAW = '#ff6644'
    COLOR_FILT = '#44aaff'
    COLOR_BG = '#1a1a2e'

    # Zoom parametri (poslednje 2 sekunde)
    ZOOM_SECONDS = 2.0
    ZOOM_SAMPLES = int(ZOOM_SECONDS / DT)
    start_idx = max(0, N - ZOOM_SAMPLES)


    fig1, axes1 = plt.subplots(2, 1, figsize=(14, 8))
    fig1.suptitle("IMU - Roll & Pitch (Full Signal)", fontsize=16, color='white')

    titles = ['Roll', 'Pitch']
    for i, ax in enumerate(axes1):
        ax.set_facecolor(COLOR_BG)
        ax.set_ylabel('Ugao (°)', color='white')
        ax.grid(True, alpha=0.2, color='gray')
        ax.tick_params(colors='white')

        # True
        ax.plot(t, true_angles[:, i] * 180/np.pi,
                color=COLOR_TRUE, linewidth=2, label='True')

        # Raw (iz akcelerometra)
        raw_plot = [roll_raw, pitch_raw][i] * 180/np.pi
        ax.plot(t, raw_plot, color=COLOR_RAW, linewidth=1.5,
                linestyle='--', label='Raw')

        # Filtered (EKF)
        filt_plot = [roll_filt, pitch_filt][i] * 180/np.pi
        ax.plot(t, filt_plot, color=COLOR_FILT, linewidth=2.5,
                label='Filtered (EKF)')

        ax.set_title(titles[i], color='white')
        ax.legend(loc='upper right', facecolor=COLOR_BG,
                  edgecolor='white', labelcolor='white')

    axes1[1].set_xlabel('Vreme (s)', color='white')
    plt.tight_layout()


    fig2, axes2 = plt.subplots(2, 1, figsize=(14, 8))
    fig2.suptitle(f"IMU - Roll & Pitch (Zoom - last {ZOOM_SECONDS}s)",
                  fontsize=16, color='white')

    t_zoom = t[start_idx:]

    for i, ax in enumerate(axes2):
        ax.set_facecolor(COLOR_BG)
        ax.set_ylabel('Ugao (°)', color='white')
        ax.grid(True, alpha=0.2, color='gray')
        ax.tick_params(colors='white')

        # True (zoom)
        ax.plot(t_zoom, true_angles[start_idx:, i] * 180/np.pi,
                color=COLOR_TRUE, linewidth=2, label='True')

        # Raw (zoom)
        raw_plot = [roll_raw, pitch_raw][i][start_idx:] * 180/np.pi
        ax.plot(t_zoom, raw_plot, color=COLOR_RAW, linewidth=1.5,
                linestyle='--', label='Raw')

        # Filtered (zoom)
        filt_plot = [roll_filt, pitch_filt][i][start_idx:] * 180/np.pi
        ax.plot(t_zoom, filt_plot, color=COLOR_FILT, linewidth=2.5,
                label='Filtered (EKF)')

        ax.set_title(titles[i], color='white')
        ax.legend(loc='upper right', facecolor=COLOR_BG,
                  edgecolor='white', labelcolor='white')

    axes2[1].set_xlabel('Vreme (s)', color='white')
    plt.tight_layout()

    # Prikaži oba plota
    plt.show()

    print("\n" + "="*50)
    print(" STATISTIKA")
    print("="*50)
    
    # Greška EKF vs True (samo za Roll i Pitch, Yaw drifta)
    for i, name in enumerate(['Roll', 'Pitch']):
        err = (true_angles[:, i] - [roll_filt, pitch_filt][i]) * 180/np.pi
        print(f"{name}:")
        print(f"  Mean error: {np.mean(err):.2f}°")
        print(f"  Std dev:    {np.std(err):.2f}°")
        print(f"  Max error:  {np.max(np.abs(err)):.2f}°")

if __name__ == "__main__":
    main()