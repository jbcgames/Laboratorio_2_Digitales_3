# -*- coding: utf-8 -*-
"""
/**
 * @file    encoder_rpm_logger.py
 * @brief   Medición de RPM con RP2040 usando un disco encoder y generación de
 *          señal PWM para caracterización de motor.
 *
 * El script:
 *   - Configura un pin PWM para excitar el motor a 100 kHz.
 *   - Usa interrupciones para medir el tiempo en alto de cada sector del
 *     disco encoder y convertirlo en RPM.
 *   - Permite dos modos:
 *       1.  **Captura automática**: barrido de PWM con registro CSV.
 *       2.  **Manual**: aplica un ciclo PWM fijo y muestra la RPM en consola.
 *
 * Comandos de consola:
 *   - `START <paso>`  Inicia la captura automática con incrementos de <paso> %.
 *   - `PWM <valor>`   Activa el modo manual con un PWM constante (0–100 %).
 *   - `EXIT`          Detiene el PWM y sale.
 *
 * @author  Miguel Angel Alvarez Guzman
 * @date    2025‑05‑04
 */
"""

# =================== IMPORTS ===================
from machine import Pin, PWM, Timer  # Hardware‑specific clases de MicroPython
import time                          # Temporización (µs / ms)
import sys, uos, math                # utils varios

# =================== PARAMETROS GENERALES ===================
# --- Disco encoder ---
ENCODER_HUECOS      = 20                           
SECTORS_REV         = ENCODER_HUECOS * 2          
RPM_CONST_US        = 60_000_000 // SECTORS_REV    
MIN_DT_US           = 100                          

# --- PWM / escalones ---
PWM_FREQ_HZ         = 100_000                     
TIEMPO_ENTRE_ESCALONES = 2_000                     
REPORT_INTERVALO_MS = 500                       

# =================== HARDWARE ===================
pwm_pin = PWM(Pin(3))              # Pin GP3 → PWM (canal por defecto)
pwm_pin.freq(PWM_FREQ_HZ)

sensor_pin = Pin(5, Pin.IN, Pin.PULL_UP)  # GP5 ← sensor óptico

# =================== VARIABLES DE ESTADO ===================
high_start_us: int | None = None   # Momento inicio pulso alto (µs)
rpm_global: float      = 0.0       # Último valor de RPM calculado
modo_captura: bool     = False     # True → capturando CSV
pwm_manual:  int       = 0         # Ciclo PWM actual en modo manual (0‑100)

# =================== ISR: mide SOLO el tiempo en alto ===================
def medir_pulso(pin: Pin) -> None:
    """
    /**
     * @brief  Interrupción en flancos del sensor.
     *
     *  * **Flanco de subida (LOW→HIGH)**: almacena el instante en que empieza
     *    el sector opaco (pulso alto).
     *  * **Flanco de bajada (HIGH→LOW)**: computa la duración del pulso alto
     *    (∆t) y, si supera el antirrebote, actualiza la variable global
     *    `rpm_global` con el valor de RPM correspondiente.
     *
     * @param pin  Objeto `machine.Pin` que generó la interrupción.
     * @note       Se asume que el disco tiene @c SECTORS_REV sectores por
     *             revolución, de modo que un pulso corresponde a 1 /
     *             `SECTORS_REV` de vuelta.
     */
    """
    global high_start_us, rpm_global

    ahora = time.ticks_us()

    if pin.value():                      # LOW → HIGH
        high_start_us = ahora           # Marca inicio de pulso alto
        return

    # Aquí sólo entra en HIGH → LOW
    if high_start_us is None:
        return                          # Subida perdida ⇒ descarta medición

    dt = time.ticks_diff(ahora, high_start_us)
    high_start_us = None                # Prepara siguiente ciclo

    if dt < MIN_DT_US:                  # Filtra rebotes / ruido
        return

    rpm_global = RPM_CONST_US / dt      # Convierte µs ⇒ RPM


# Asocia ambos flancos a la ISR
sensor_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
               handler=medir_pulso)

# =================== MODOS DE FUNCIONAMIENTO ===================
def ejecutar_captura(paso_pwm: int) -> None:
    """
    /**
     * @brief  Barrido automático de PWM y registro de datos en CSV.
     *
     * Genera una rampa ascendente y otra descendente de @c 0 % → @c 100 %
     * (paso @p paso_pwm) aplicando cada valor durante
     * `TIEMPO_ENTRE_ESCALONES` ms.  Se crea un archivo CSV con columnas:
     * `Tiempo_ms, PWM_porcentaje, RPM`.
     *
     * @param paso_pwm  Incremento/decremento porcentual (1–99).
     */
    """
    global modo_captura
    modo_captura = True

    pwm_max = 100
    secuencia = (list(range(0, pwm_max + 1, paso_pwm)) +
                 list(range(pwm_max - paso_pwm, -1, -paso_pwm)))

    # ---------- Creación de archivo ----------
    nombre = f"captura_{time.ticks_ms()}.csv"
    try:
        f = open(nombre, "w")
        f.write("Tiempo_ms,PWM_porcentaje,RPM\n")
    except Exception as e:
        print("Error al crear archivo:", e)
        modo_captura = False
        return

    print("Iniciando captura…  (Ctrl‑C para abortar)")
    inicio = time.ticks_ms()

    try:
        # ---------- Bucle principal ----------
        for pwm_val in secuencia:
            pwm_pin.duty_u16(int(65535 * pwm_val / 100))
            print(f"PWM aplicado: {pwm_val}%")

            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < TIEMPO_ENTRE_ESCALONES:
                time.sleep_ms(10)  # latencia mínima sin bloquear IRQ
                marca = time.ticks_diff(time.ticks_ms(), inicio)
                try:
                    f.write(f"{marca},{pwm_val},{rpm_global:.4f}\n")
                except Exception as e:
                    print("Error al escribir CSV:", e)
                    raise
    except KeyboardInterrupt:
        print("Captura interrumpida por usuario.")
    finally:
        pwm_pin.duty_u16(0)
        modo_captura = False
        f.close()
        print(f"Datos guardados en «{nombre}»")

def modo_manual(pwm_val: int) -> None:
    """
    /**
     * @brief  Modo manual: aplica un PWM fijo y muestra la RPM periódicamente.
     *
     * @param pwm_val  Ciclo de trabajo en porcentaje (0–100 %).
     * @note  Presione **Ctrl‑C** para salir y desactivar el PWM.
     */
    """
    global pwm_manual
    pwm_manual = pwm_val
    pwm_pin.duty_u16(int(65535 * pwm_val / 100))

    print(f"PWM manual {pwm_val}% activo — Ctrl‑C para salir")
    ultimo = time.ticks_ms()

    try:
        while True:
            actual = time.ticks_ms()
            if time.ticks_diff(actual, ultimo) >= REPORT_INTERVALO_MS:
                print(f"PWM={pwm_manual}%  |  RPM={rpm_global:.2f}")
                ultimo = actual
    except KeyboardInterrupt:
        print("Modo manual interrumpido.")
        pwm_pin.duty_u16(0)
        pwm_manual = 0

# =================== BUCLE DE CONSOLA ===================
print("Sistema listo. Comandos:  START <paso>  |  PWM <valor>  |  EXIT")

while True:
    try:
        cmd = input(">> ").strip()
        if cmd.upper() == "EXIT":
            print("Saliendo…")
            pwm_pin.duty_u16(0)
            break

        elif cmd.upper().startswith("START"):
            partes = cmd.split()
            if len(partes) == 2 and partes[1].isdigit():
                paso = int(partes[1])
                if 1 <= paso < 100:
                    ejecutar_captura(paso)
                else:
                    print("ERROR: paso 1‑99")
            else:
                print("Uso: START <paso>")

        elif cmd.upper().startswith("PWM"):
            partes = cmd.split()
            if len(partes) == 2 and partes[1].isdigit():
                val = int(partes[1])
                if 0 <= val <= 100:
                    modo_manual(val)
                else:
                    print("ERROR: PWM 0‑100")
            else:
                print("Uso: PWM <valor>")

        else:
            print("Comando no reconocido.")

    except KeyboardInterrupt:
        print("\nInterrumpido; puedes volver a escribir un comando.")
