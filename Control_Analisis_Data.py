# -*- coding: utf-8 -*-
"""
/**
* @file    model_fopdt_fit.py
* @brief   Ajuste de un modelo FOPDT (K, τ, θ) a cada escalón de PWM → RPM
* @details
*   * Lee un archivo CSV con columnas de tiempo, PWM y RPM.  
*   * Detecta los escalones de PWM y divide la señal en tramos “subida/bajada”.  
*   * Filtra valores atípicos mediante la prueba de Hampel.  
*   * Ajusta un modelo FOPDT por mínimos cuadrados no‑lineales a cada tramo.  
*   * Genera y guarda una figura PNG por tramo con la curva medida y la curva
*     modelo (`pwm_<valorPWM>_<subida|bajada>.png`).  
* Ejecución:
* \code{.bash}
*   python model_fopdt_fit.py datos.csv
* \endcode
* Si no se pasa el CSV como argumento, el nombre se pedirá por *stdin*.
*
* @warning  Requiere los paquetes *NumPy*, *Pandas*, *SciPy* y *Matplotlib*.
* @version  1.0
* @date     2025‑05‑04
* @author   Miguel Angel Alvarez Guzman
**/
"""
import sys, os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# ----------------------------------------------------------------------
# ----------------------  UTILIDADES DE FILTRO  ------------------------
def hampel(series: pd.Series,
           window_size: int = 7,
           n_sigmas: float = 3.0) -> pd.Series:
    """
    @brief   Detecta *outliers* con la prueba de Hampel.
    @details Calcula la Mediana de la Desviación Absoluta (MAD) en una ventana
             móvil y marca como atípicos los puntos cuyo desvío supera
             `n_sigmas·MAD`.
    @param   series       Serie de datos a inspeccionar.
    @param   window_size  Tamaño de la ventana móvil (en muestras).
    @param   n_sigmas     Umbral en desviaciones MAD para etiquetar *outliers*.
    @return  Serie booleana (`True` donde el punto es *outlier*).
    """
    k = 1.4826                         # ≈ convertir MAD a σ (distrib. normal)
    med_rolling = series.rolling(window_size, center=True).median()
    diff = np.abs(series - med_rolling)
    mad = k * diff.rolling(window_size, center=True).median()
    outliers = diff > n_sigmas * mad
    return outliers.fillna(False)


# ----------------------------------------------------------------------
# ---------------------  MODELO Y AJUSTE  ------------------------------
def step_response(t: np.ndarray,
                  K: float,
                  tau: float,
                  theta: float,
                  y0: float,
                  du: float) -> np.ndarray:
    """
    @brief   Respuesta de 1er orden con retardo a un escalón.
    @param   t      Vector de tiempo (s).
    @param   K      Ganancia estacionaria del proceso.
    @param   tau    Constante de tiempo (s).
    @param   theta  Retardo puro (s).
    @param   y0     Valor inicial de la salida.
    @param   du     Amplitud del escalón de entrada.
    @return  Vector con la respuesta simulada.
    """
    t = np.asarray(t)
    y = np.where(
        t < theta,
        y0,
        y0 + K * du * (1 - np.exp(-(t - theta) / tau))
    )
    return y


def ajustar_fopdt(t: pd.Series,
                  y: pd.Series,
                  y0: float,
                  du: float) -> tuple[float, float, float]:
    """
    @brief   Ajusta los parámetros FOPDT (K, τ, θ) por mínimos cuadrados.
    @param   t   Tiempo (s) relativo al inicio del tramo.
    @param   y   Datos de salida (RPM) sin *outliers*.
    @param   y0  Valor inicial de la salida.
    @param   du  Amplitud del escalón PWM.
    @return  Tupla (K, τ, θ) optimizada.
    """
    # --- Estimaciones iniciales ----------------------------
    K0     = (y.iloc[-1] - y0) / du if du != 0 else 1.0
    tau0   = max((t.iloc[-1] - t.iloc[0]) / 3.0, 0.05)
    theta0 = 0.0

    def modelo(t_, K, tau, theta):        # Función interna para curve_fit
        return step_response(t_, K, tau, theta, y0, du)

    popt, _ = curve_fit(
        modelo, t, y,
        p0=[K0, tau0, theta0],
        bounds=([-np.inf, 1e-3, 0], [np.inf, np.inf, t.iloc[-1]]),
        method='trf',          # algoritmo robusto
        loss='soft_l1',        # penaliza menos los outliers residuales
        f_scale=1.0
    )
    return popt


# ----------------------------------------------------------------------
def detectar_escalones(df: pd.DataFrame, pwm_col: str) -> list[tuple[int, int]]:
    """
    @brief   Localiza los índices de inicio/fin de cada tramo posterior
             a un cambio de PWM.
    @param   df       *DataFrame* con los datos.
    @param   pwm_col  Nombre de la columna PWM.
    @return  Lista de tuplas (idx_start, idx_end) por tramo.
    """
    cambio = df[pwm_col].diff().fillna(0) != 0
    pasos  = np.where(cambio)[0]
    pasos  = np.append(pasos, len(df))     # Añade fin del archivo
    tramos = [(pasos[i], pasos[i+1]) for i in range(len(pasos)-1)]
    return tramos


# ----------------------------------------------------------------------
def main() -> None:
    """
    @brief   Punto de entrada del script.
    @details
      1. Carga el CSV.  
      2. Detecta las columnas relevantes (Tiempo, PWM, RPM).  
      3. Segmenta en escalones.  
      4. Aplica filtro, ajusta modelo y genera una figura por tramo.
    """
    # -------- 1. Cargar CSV -------------------------------------------
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = input("Nombre del CSV: ")
    df = pd.read_csv(csv_file)

    # -------- 2. Detectar columnas ------------------------------------
    time_col = (
        'Tiempo_ms'     if 'Tiempo_ms'     in df.columns else
        'timestamp_us'  if 'timestamp_us'  in df.columns else
        df.columns[0]
    )
    if time_col == 'timestamp_us':         # Normaliza a milisegundos
        df['Tiempo_ms'] = df['timestamp_us'] / 1000.0
        time_col = 'Tiempo_ms'

    pwm_col = (
        'PWM_porcentaje' if 'PWM_porcentaje' in df.columns else df.columns[1]
    )
    rpm_col = 'RPM' if 'RPM' in df.columns else df.columns[2]

    # -------- 3. Detectar escalones -----------------------------------
    tramos = detectar_escalones(df, pwm_col)

    # -------- 4. Procesar cada tramo ----------------------------------
    for idx_start, idx_end in tramos:
        seg = df.iloc[idx_start:idx_end].copy()
        if len(seg) < 8:                   # Evita tramos muy cortos
            continue

        pwm_new  = seg[pwm_col].iloc[0]
        pwm_prev = df[pwm_col].iloc[idx_start-1] if idx_start > 0 else pwm_new
        du = pwm_new - pwm_prev
        if du == 0:
            continue                       # No hay escalón real

        direction = 'subida' if du > 0 else 'bajada'

        # --- 4.1 Eje de tiempo relativo (s) ---------------------------
        t_ms = seg[time_col] - seg[time_col].iloc[0]
        t_s  = t_ms / 1000.0
        y    = seg[rpm_col]
        y0   = y.iloc[0]

        # --- 4.2 Filtro de *outliers* ---------------------------------
        outliers  = hampel(y, window_size=7, n_sigmas=3.0)
        mask      = ~outliers
        t_clean   = t_s[mask]
        y_clean   = y[mask]

        if len(y_clean) < 5:
            print(f"[WARN] Tramo PWM {pwm_prev}→{pwm_new}% "
                  f"({direction}) sin suficientes datos tras filtrar.")
            continue

        # --- 4.3 Ajuste FOPDT -----------------------------------------
        try:
            K, tau, theta = ajustar_fopdt(t_clean, y_clean, y0, du)
        except RuntimeError:
            print(f"[ERR] No converge en PWM {pwm_new}% ({direction}). "
                  f"Se omite.")
            continue

        y_model = step_response(t_s, K, tau, theta, y0, du)

        # --- 4.4 Gráfica ----------------------------------------------
        plt.figure(figsize=(8, 4))
        plt.plot(t_clean, y_clean, '.', label='Medido ')
        plt.plot(t_s, y_model, '-', label=f'Modelo 1º orden\n'
                 f'K={K:.2f}, τ={tau:.2f}s, θ={theta:.2f}s')

        plt.title(f'Respuesta motor: PWM {pwm_prev:.0f}→{pwm_new:.0f}% '
                  f'({direction})')
        plt.xlabel('Tiempo [s]')
        plt.ylabel('RPM')
        plt.grid(True)
        plt.legend()

        # --- 4.5 Guardar PNG ------------------------------------------
        fname = f"Salida Mpy/pwm_{int(pwm_new)}_{direction}_mpy.png"
        plt.tight_layout()
        plt.savefig(fname, dpi=150)
        plt.close()
        print(f"[OK] Guardado: {fname}")


# ----------------------------------------------------------------------
if __name__ == "__main__":    # ¡Ejecútame!
    main()
