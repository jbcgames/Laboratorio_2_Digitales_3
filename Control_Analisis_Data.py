# -*- coding: utf-8 -*-
"""
/**
 * @file    model_fopdt_fit.py
 * @brief   Ajuste de un modelo FOPDT por escalones y generación de un gráfico
 *          global comparativo (datos vs modelo) a partir de un registro CSV.
 *
 * El programa:
 *   - Lee un archivo CSV con tiempo, PWM y RPM de un ensayo de barrido.
 *   - Detecta los escalones de PWM (subidas y bajadas) en la señal medida.
 *   - Elimina valores atípicos mediante la prueba de Hampel.
 *   - Ajusta, por mínimos cuadrados no‑lineales, un modelo FOPDT a cada tramo.
 *   - Genera gráficos PNG individuales por tramo con la curva medida y el modelo.
 *   - Construye y guarda un gráfico global (todas las curvas) para validar el
 *     modelo completo.
 *
 * @version 1.2
 * @date    2025‑05‑06
 * @author  Miguel Ángel Álvarez Guzmán
 */
"""

import sys, os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# -----------------------------------------------------------------------------
# Funciones utilitarias                                                         
# -----------------------------------------------------------------------------

def hampel(series: pd.Series, window_size: int = 7, n_sigmas: float = 3.0) -> pd.Series:
    """!
    Identifica valores atípicos usando el filtro de Hampel.

    @param series        Señal de entrada (Series de Pandas).
    @param window_size   Tamaño de la ventana deslizante (número de muestras).
    @param n_sigmas      Multiplicador del MAD (desviación absoluta mediana)
                         para definir el umbral.
    @return Serie booleana donde *True* indica un outlier.
    """
    k = 1.4826  # factor para aproximar la desviación estándar
    med_rolling = series.rolling(window_size, center=True).median()
    diff = np.abs(series - med_rolling)
    mad = k * diff.rolling(window_size, center=True).median()
    return (diff > n_sigmas * mad).fillna(False)

# -----------------------------------------------------------------------------
# Modelo FOPDT                                                                  
# -----------------------------------------------------------------------------

def step_response(t, K: float, tau: float, theta: float, y0: float, du: float):
    """!
    Función de transferencia FOPDT en el dominio del tiempo.

    @param t      Vector de tiempos (s).
    @param K      Ganancia estacionaria.
    @param tau    Constante de tiempo (s).
    @param theta  Retardo puro (s).
    @param y0     Nivel inicial (RPM) al iniciar el escalón.
    @param du     Cambio en la entrada PWM (Δu).
    @return       Respuesta y(t) del sistema.
    """
    t = np.asarray(t)
    return np.where(t < theta,
                    y0,
                    y0 + K * du * (1 - np.exp(-(t - theta) / tau)))


def ajustar_fopdt(t: pd.Series, y: pd.Series, y0: float, du: float):
    """!
    Ajusta un modelo FOPDT a los datos de un tramo mediante *curve_fit*.

    @param t   Tiempo relativo (s) del tramo.
    @param y   Señal de salida (RPM) medida.
    @param y0  Nivel inicial de y.
    @param du  Cambio de entrada PWM.
    @return    Tupla (K, tau, theta) del modelo ajustado.
    """
    K0 = (y.iloc[-1] - y0) / du if du else 1.0
    tau0 = max((t.iloc[-1] - t.iloc[0]) / 3.0, 0.05)
    popt, _ = curve_fit(lambda t_, K, tau, theta: step_response(t_, K, tau, theta, y0, du),
                        t, y, p0=[K0, tau0, 0],
                        bounds=([-np.inf, 1e-3, 0], [np.inf, np.inf, t.iloc[-1]]),
                        method='trf', loss='soft_l1')
    return popt  # (K, tau, theta)

# -----------------------------------------------------------------------------
# Detección de escalones                                                        
# -----------------------------------------------------------------------------

def detectar_escalones(df: pd.DataFrame, pwm_col: str):
    """!
    Encuentra los índices (inicio, fin) de cada tramo de PWM constante.

    @param df        DataFrame con los datos originales.
    @param pwm_col   Nombre de la columna PWM.
    @return          Lista de tuplas (idx_ini, idx_fin) por tramo.
    """
    cambio = df[pwm_col].diff().fillna(0) != 0
    pasos = np.where(cambio)[0]
    pasos = np.append(pasos, len(df))
    return [(pasos[i], pasos[i+1]) for i in range(len(pasos)-1)]

# -----------------------------------------------------------------------------
# Programa principal                                                            
# -----------------------------------------------------------------------------

def main() -> None:
    """!
    Punto de entrada. Procesa el CSV, ajusta tramo a tramo y genera gráficos.
    """
    csv_file = sys.argv[1] if len(sys.argv) > 1 else input("Nombre del CSV: ")
    df = pd.read_csv(csv_file)

    # --------------------------- Selección de columnas -----------------------
    time_col = ('Tiempo_ms' if 'Tiempo_ms' in df.columns else
                'timestamp_us' if 'timestamp_us' in df.columns else df.columns[0])
    if time_col == 'timestamp_us':
        df['Tiempo_ms'] = df['timestamp_us'] / 1000.0
        time_col = 'Tiempo_ms'

    pwm_col = 'PWM_porcentaje' if 'PWM_porcentaje' in df.columns else df.columns[1]
    rpm_col = 'RPM' if 'RPM' in df.columns else df.columns[2]

    # Normaliza tiempo absoluto a segundos desde inicio
    df['t_s'] = (df[time_col] - df[time_col].iloc[0]) / 1000.0

    tramos = detectar_escalones(df, pwm_col)
    resultados = []  # Acumula parámetros para la curva global

    # --------------------------- Procesamiento por tramo --------------------
    for idx_start, idx_end in tramos:
        seg = df.iloc[idx_start:idx_end].copy()
        if len(seg) < 8:
            continue  # tramo demasiado corto

        pwm_new  = seg[pwm_col].iloc[0]
        pwm_prev = df[pwm_col].iloc[idx_start-1] if idx_start > 0 else pwm_new
        du = pwm_new - pwm_prev
        if du == 0:
            continue  # sin cambio

        direccion = 'subida' if du > 0 else 'bajada'
        t_rel = seg['t_s'] - seg['t_s'].iloc[0]
        y     = seg[rpm_col]
        y0    = y.iloc[0]

        # ---- filtro Hampel ---------------------------------------------------
        mask = ~hampel(y, 7, 3.0)
        t_clean, y_clean = t_rel[mask], y[mask]
        if len(y_clean) < 5:
            print(f"[WARN] Tramo PWM {pwm_prev}->{pwm_new}% sin datos suficientes.")
            continue

        try:
            K, tau, theta = ajustar_fopdt(t_clean, y_clean, y0, du)
        except RuntimeError:
            print(f"[ERR] No converge PWM {pwm_new}% ({direccion}). Omitido.")
            continue

        t_seg = t_rel
        y_mod = step_response(t_seg, K, tau, theta, y0, du)

        # ---- gráfico individual ---------------------------------------------
        rmse = np.sqrt(np.mean((y_clean - step_response(t_clean, K, tau, theta, y0, du))**2))
        plt.figure(figsize=(8,4))
        plt.plot(t_clean, y_clean, '.', label='Medido')
        plt.plot(t_seg, y_mod, '-', label=f'Modelo; K={K:.2f}, τ={tau:.2f}s, θ={theta:.2f}s')
        plt.text(0.02, 0.95, f'RMSE={rmse:.1f} RPM', transform=plt.gca().transAxes,
                 ha='left', va='top', bbox=dict(fc='white', alpha=0.75, ec='none'))
        plt.title(f'PWM {pwm_prev:.0f}→{pwm_new:.0f}% ({direccion})')
        plt.xlabel('Tiempo [s]'); plt.ylabel('RPM')
        plt.grid(); plt.legend(); plt.tight_layout()
        fname = os.path.join('Salida Mpy', f"pwm_{int(pwm_new)}_{direccion}_mpy.png")
        os.makedirs('Salida Mpy', exist_ok=True)
        plt.savefig(fname, dpi=150); plt.close()
        print(f"[OK] Guardado: {fname}")

        # ---- guardar parámetros para curva global ---------------------------
        resultados.append({
            'slice': slice(idx_start, idx_end),
            'K': K, 'tau': tau, 'theta': theta,
            'y0': y0, 'du': du,
            't0': df['t_s'].iloc[idx_start]
        })

    # --------------------------- Gráfico global ------------------------------
    if resultados:
        y_model_global = np.full(len(df), np.nan)
        for r in resultados:
            idx = r['slice']
            t_seg_abs = df['t_s'].iloc[idx] - r['t0']     # tiempo relativo tramo
            y_model_global[idx] = step_response(t_seg_abs,
                                                r['K'], r['tau'], r['theta'],
                                                r['y0'], r['du'])

        # puede haber pequeñas zonas NaN (tramos demasiado cortos / descartados)
        # --> sustituirlas por interpolación lineal para la gráfica
        y_model_interp = pd.Series(y_model_global).interpolate(method='linear')

        # ---- gráfico global ------------------------------------------------------
        plt.figure(figsize=(10,5))
        plt.plot(df['t_s'], df[rpm_col], '.', markersize=2, label='Medido')
        plt.plot(df['t_s'], y_model_interp, '-', linewidth=1.2, label='Modelo global')
        plt.title('Respuesta completa del sistema (medición vs modelo FOPDT)')
        plt.xlabel('Tiempo [s]'); plt.ylabel('RPM'); plt.grid(); plt.legend()
        plt.ylim(0, 8000)  # Limitar el eje y a un máximo de 8000
        plt.tight_layout()
        global_name = "Salida Mpy/pwm_global_mpy.png"
        plt.savefig(global_name, dpi=150); plt.close()
        print(f"[OK] Gráfico global guardado: {global_name}")
    else:
        print("[INFO] No se generó gráfico global (sin tramos válidos).")

# ----------------------------------------------------------------------
if __name__ == "__main__":
    main()
