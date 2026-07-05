import numpy as np
import matplotlib.pyplot as plt

def generate_imu_data(duration=10.0, dt=0.01, random_seed=42):
    """Generise sinteticke IMU podatke."""
    np.random.seed(random_seed)
    t = np.arange(0, duration, dt)
    N = len(t)

    # 1. Generisi "istinu" (uglove)
    true_roll = 0.3 * np.sin(0.8 * t) + 0.1 * np.sin(1.2 * t + 0.5)
    true_pitch = 0.2 * np.cos(0.6 * t) + 0.15 * np.sin(0.9 * t + 0.3)
    true_yaw = 0.1 * t + 0.2 * np.sin(0.3 * t)
    true_angles = np.column_stack([true_roll, true_pitch, true_yaw])

    # 2. Ugaone brzine (ziroskop)
    true_gyro = np.gradient(true_angles, dt, axis=0)

    # 3. Akcelerometar (gravitacija)
    g = 9.81
    ax_true = -g * np.sin(true_pitch)
    ay_true = g * np.cos(true_pitch) * np.sin(true_roll)
    az_true = g * np.cos(true_pitch) * np.cos(true_roll)
    true_accel = np.column_stack([ax_true, ay_true, az_true])

    # 4. Dodaj sum i bias
    gyro_bias = np.array([0.015, -0.008, 0.025])
    gyro_noise_std = 0.005
    accel_noise_std = 0.03

    gyro_meas = true_gyro + gyro_bias + np.random.normal(0, gyro_noise_std, (N, 3))
    accel_meas = true_accel + np.random.normal(0, accel_noise_std, (N, 3))

    return t, gyro_meas, accel_meas, true_angles, true_gyro, true_accel


def plot_and_save_imu_data(t, gyro_meas, accel_meas, true_angles, filename='imu_plot.png'):
    """
    Prikazuje i cuva grafike IMU podataka.
    """
    # Kreiraj sliku sa 3 reda, 1 kolona
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # -------------------------------------------------------------------------
    # 1. Graf: Pravi uglovi (roll, pitch, yaw)
    # -------------------------------------------------------------------------
    axes[0].plot(t, true_angles[:, 0] * 180/np.pi, label='Roll', linewidth=2, color='red')
    axes[0].plot(t, true_angles[:, 1] * 180/np.pi, label='Pitch', linewidth=2, color='green')
    axes[0].plot(t, true_angles[:, 2] * 180/np.pi, label='Yaw', linewidth=2, color='blue')
    axes[0].set_ylabel('Ugao (stepeni)')
    axes[0].set_title('Prava orijentacija (ground truth)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    # -------------------------------------------------------------------------
    # 2. Graf: Merenja ziroskopa (sa sumom i biasom)
    # -------------------------------------------------------------------------
    axes[1].plot(t, gyro_meas[:, 0] * 180/np.pi, label='Gyro X', alpha=0.8, color='red')
    axes[1].plot(t, gyro_meas[:, 1] * 180/np.pi, label='Gyro Y', alpha=0.8, color='green')
    axes[1].plot(t, gyro_meas[:, 2] * 180/np.pi, label='Gyro Z', alpha=0.8, color='blue')
    axes[1].set_ylabel('Ugaona brzina (stepeni/s)')
    axes[1].set_title('Merenja ziroskopa (sa sumom i biasom)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    # -------------------------------------------------------------------------
    # 3. Graf: Merenja akcelerometra (sa sumom)
    # -------------------------------------------------------------------------
    axes[2].plot(t, accel_meas[:, 0], label='Accel X', alpha=0.8, color='red')
    axes[2].plot(t, accel_meas[:, 1], label='Accel Y', alpha=0.8, color='green')
    axes[2].plot(t, accel_meas[:, 2], label='Accel Z', alpha=0.8, color='blue')
    axes[2].set_xlabel('Vreme (s)')
    axes[2].set_ylabel('Ubrzanje (m/s^2)')
    axes[2].set_title('Merenja akcelerometra (sa sumom)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    # Podesi razmak izmedju grafika
    plt.tight_layout()

    # -------------------------------------------------------------------------
    # 4. SACUVAJ SLIKU
    # -------------------------------------------------------------------------
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Slika sacuvana: {filename}")

    # Prikazi sliku na ekranu
    plt.show()
    
    return fig


def print_statistics(gyro_meas, accel_meas, true_angles):
    """Ispisuje osnovne statistike generisanih podataka."""
    print("\n" + "="*50)
    print("STATISTIKA PODATAKA")
    print("="*50)
    
    print(f"\nZiroskop - srednja vrednost (rad/s):")
    print(f"  X: {np.mean(gyro_meas[:, 0]):.4f}")
    print(f"  Y: {np.mean(gyro_meas[:, 1]):.4f}")
    print(f"  Z: {np.mean(gyro_meas[:, 2]):.4f}")
    
    print(f"\nZiroskop - standardna devijacija (rad/s):")
    print(f"  X: {np.std(gyro_meas[:, 0]):.4f}")
    print(f"  Y: {np.std(gyro_meas[:, 1]):.4f}")
    print(f"  Z: {np.std(gyro_meas[:, 2]):.4f}")
    
    print(f"\nAkcelerometar - srednja vrednost (m/s^2):")
    print(f"  X: {np.mean(accel_meas[:, 0]):.4f}")
    print(f"  Y: {np.mean(accel_meas[:, 1]):.4f}")
    print(f"  Z: {np.mean(accel_meas[:, 2]):.4f}")
    
    print(f"\nAkcelerometar - standardna devijacija (m/s^2):")
    print(f"  X: {np.std(accel_meas[:, 0]):.4f}")
    print(f"  Y: {np.std(accel_meas[:, 1]):.4f}")
    print(f"  Z: {np.std(accel_meas[:, 2]):.4f}")
    
    print(f"\nUglovi - opseg (stepeni):")
    print(f"  Roll:  {np.min(true_angles[:, 0]*180/np.pi):.1f}° do {np.max(true_angles[:, 0]*180/np.pi):.1f}°")
    print(f"  Pitch: {np.min(true_angles[:, 1]*180/np.pi):.1f}° do {np.max(true_angles[:, 1]*180/np.pi):.1f}°")
    print(f"  Yaw:   {np.min(true_angles[:, 2]*180/np.pi):.1f}° do {np.max(true_angles[:, 2]*180/np.pi):.1f}°")


if __name__ == "__main__":
    # 1. Generisi podatke (10 sekundi, 100 Hz)
    print("Generisanje IMU podataka...")
    t, gyro, accel, true_ang, _, _ = generate_imu_data(
        duration=10.0,
        dt=0.01,
        random_seed=42
    )

    # 2. Prikazi i sacuvaj sliku
    print("\nCrtanje grafika...")
    plot_and_save_imu_data(t, gyro, accel, true_ang, filename='imu_plot.png')

    # 3. Ispisi statistiku
    print_statistics(gyro, accel, true_ang)

    print("\n" + "="*50)
    print("GOTOVO!")
    print("="*50)