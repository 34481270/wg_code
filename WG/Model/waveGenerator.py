import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


def excited_force(U195):
    # 1. 定義 RAO 原始資料
    RAO_data = [
        {"omega": 0.2, "RAO_amp": 0.9836, "RAO_phase": 0.0},
        {"omega": 0.26, "RAO_amp": 0.9833, "RAO_phase": 0.0},
        {"omega": 0.32, "RAO_amp": 0.982, "RAO_phase": 0.002},
        {"omega": 0.38, "RAO_amp": 0.9787, "RAO_phase": 0.009},
        {"omega": 0.44, "RAO_amp": 0.9719, "RAO_phase": 0.026},
        {"omega": 0.5, "RAO_amp": 0.9591, "RAO_phase": 0.059},
        {"omega": 0.56, "RAO_amp": 0.9373, "RAO_phase": 0.115},
        {"omega": 0.62, "RAO_amp": 0.9032, "RAO_phase": 0.201},
        {"omega": 0.68, "RAO_amp": 0.8463, "RAO_phase": 0.371},
        {"omega": 0.74, "RAO_amp": 0.7848, "RAO_phase": 0.651},
        {"omega": 0.8, "RAO_amp": 0.6927, "RAO_phase": 1.419},
        {"omega": 0.86, "RAO_amp": 0.5808, "RAO_phase": 3.278},
        {"omega": 0.92, "RAO_amp": 0.4515, "RAO_phase": 7.667},
        {"omega": 0.98, "RAO_amp": 0.3316, "RAO_phase": 16.187},
        {"omega": 1.04, "RAO_amp": 0.2323, "RAO_phase": 32.016},
        {"omega": 1.1, "RAO_amp": 0.1698, "RAO_phase": 57.619},
        {"omega": 1.16, "RAO_amp": 0.1466, "RAO_phase": 88.452},
        {"omega": 1.22, "RAO_amp": 0.1407, "RAO_phase": 113.236},
        {"omega": 1.28, "RAO_amp": 0.1267, "RAO_phase": 134.125},
        {"omega": 1.34, "RAO_amp": 0.104, "RAO_phase": 153.336},
        {"omega": 1.4, "RAO_amp": 0.08037, "RAO_phase": 177.125},
        {"omega": 1.46, "RAO_amp": 0.06156, "RAO_phase": -147.447},
        {"omega": 1.52, "RAO_amp": 0.05231, "RAO_phase": -113.337},
        {"omega": 1.58, "RAO_amp": 0.04233, "RAO_phase": -76.601},
        {"omega": 1.64, "RAO_amp": 0.032, "RAO_phase": -42.856},
        {"omega": 1.7, "RAO_amp": 0.02031, "RAO_phase": 5.518},
        {"omega": 1.76, "RAO_amp": 0.01539, "RAO_phase": 75.741},
        {"omega": 1.82, "RAO_amp": 0.04563, "RAO_phase": 107.679},
        {"omega": 1.88, "RAO_amp": 0.056, "RAO_phase": 67.279},
        {"omega": 1.94, "RAO_amp": 0.1054, "RAO_phase": -3.767},
        {"omega": 2.0, "RAO_amp": 0.2106, "RAO_phase": -14.683}
    ]

    # 2. 提取 RAO 資料
    omega_raw = np.array([d["omega"] for d in RAO_data])
    RAO_amp_raw = np.array([d["RAO_amp"] for d in RAO_data])
    RAO_phase_raw = np.deg2rad([d["RAO_phase"] for d in RAO_data])  # 🔁 轉成 radians

    # 3. 插值 RAO
    interp_amp = interp1d(omega_raw, RAO_amp_raw, bounds_error=False, fill_value=0)
    interp_phase = interp1d(omega_raw, RAO_phase_raw, bounds_error=False, fill_value=0)

    # 4. 合成波浪頻率
    N = 500
    omega = np.linspace(0.2, 2.0, N)
    domega = omega[1] - omega[0]

    # PM spectrum
    g = 9.81
    omega_0 = g / U195
    alpha = 8.1e-3
    beta = 0.74
    S = alpha * g**2 * omega**(-5) * np.exp(-beta * (omega_0 / omega)**4)
    S = np.nan_to_num(S)

    # 波浪振幅 Ak
    Ak = np.sqrt(2 * S * domega)

    # 5. 合成 heave 運動
    RAO_amp = interp_amp(omega)
    RAO_phase = interp_phase(omega)
    zk = Ak * RAO_amp
    phik = np.random.uniform(0, 2*np.pi, N)
    t = np.linspace(0, 100, 1000)

    # 合成 heave z(t)
    z_t = np.sum([
        zk[i] * np.cos(omega[i] * t + phik[i] + RAO_phase[i])
        for i in range(N)
    ], axis=0)

    # # 6. 畫圖
    # plt.plot(t, z_t)
    # plt.xlabel("Time (s)")
    # plt.ylabel("Heave displacement z(t) [m]")
    # plt.title("Simulated Heave Motion using RAO and PM Spectrum")
    # plt.grid(True)
    # plt.show()


    # 假設參數（你可以根據模型替換）
    m = 50                   # kg
    A_heave = 6.58             # kg，附加質量
    B_heave = 0.4             # Ns/m，線性阻尼
    A_wp = 0.6              # m²，水線面積
    rho = 1025               # kg/m³，海水密度

    # 系統參數
    M = m + A_heave
    B = B_heave
    C = rho * g * A_wp

    # 一階與二階微分
    dt = t[1] - t[0]
    zdot = np.gradient(z_t, dt)
    zddot = np.gradient(zdot, dt)

    # 無 tether，計算 heave 波浪激發力
    tau_wave = M * zddot + B * zdot + C * z_t
    return tau_wave
    # # 畫出波浪激發力
    # plt.plot(t, tau_wave)
    # plt.xlabel("Time (s)")
    # plt.ylabel("Heave Wave Excitation Force [N]")
    # plt.title("Wave Excitation Force in Heave")
    # plt.grid(True)
    # plt.show()

excited_force(U195=10)