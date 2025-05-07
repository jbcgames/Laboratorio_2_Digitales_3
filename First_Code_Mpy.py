# -*- coding: utf-8 -*-
"""
/**
 * @file    encoder_rpm_logger.py
 * @brief   Medición de RPM con RP2040 usando un disco encoder y generación de
 *          señal PWM para caracterización de motor.
 *
 * El script:
 *   - Configura un pin PWM para excitar el motor a 100 kHz.
 *   - Usa interrupciones para medir el número de ranuras del encoder que
 *     pasan por el sensor y convertir dicho conteo en RPM cada 4 ms.
 *   - Recorre automáticamente un barrido de ciclos de trabajo de 0 → 100 % y
 *     vuelve a 0 % para capturar la curva de respuesta del motor.
 *   - Almacena las tripletas {tiempo [ms], PWM [%], RPM} en un archivo CSV.
 *
 * @author  Miguel Ángel Álvarez Guzmán
 * @date    2025‑05‑06
 */
"""

from machine import Pin, PWM
import time

# -----------------------------------------------------------------------------
# Parámetros de configuración                                                  
# -----------------------------------------------------------------------------

ENCODER_HUECOS = 20          #: Número de ranuras/reflectores del disco encoder
TIEMPO_ENTRE_PASOS = 2000    #: Duración de cada escalón PWM [ms]
MEDICION_INTERVALO_MS = 4    #: Período de cálculo de RPM [ms]
PASOS_PWM = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]  #: Secuencia de ciclos de trabajo [%]
ARCHIVO = "datos.csv"        #: Nombre del archivo de salida en la flash

# -----------------------------------------------------------------------------
# Variables de estado                                                          
# -----------------------------------------------------------------------------

contador = 0                 #: Contador global de flancos detectados por el encoder

# -----------------------------------------------------------------------------
# Inicialización de hardware                                                   
# -----------------------------------------------------------------------------

pwm_pin = PWM(Pin(3))        # Pin PWM conectado al puente H / transistor de potencia
pwm_pin.freq(100_000)        # Frecuencia PWM de 100 kHz

sensor_pin = Pin(5, Pin.IN, Pin.PULL_UP)  # Pin de entrada conectado al fotointerruptor


def incrementar_contador(pin: Pin) -> None:
    """!
    Incrementa el contador global de flancos.

    @param pin  Objeto :class:`machine.Pin` que disparó la interrupción.
    @note  Esta función se registra como manejador de interrupción (IRQ) en el
           flanco de bajada generado cada vez que pasa una ranura del encoder.
    """
    global contador
    contador += 1


# Registro de la rutina de interrupción ─ se hace *después* de definirla.
sensor_pin.irq(trigger=Pin.IRQ_FALLING, handler=incrementar_contador)


def calcular_rpm() -> float:
    """!
    Calcula las revoluciones por minuto (RPM) en función de los pulsos
    acumulados desde la última llamada.

    @return Velocidad instantánea del eje en RPM.
    """
    global contador
    vueltas = contador / ENCODER_HUECOS
    rpm = vueltas * (60000 / MEDICION_INTERVALO_MS)
    contador = 0  # Reinicia el contador para la siguiente ventana
    return rpm


# -----------------------------------------------------------------------------
# Preparación de archivo                                                       
# -----------------------------------------------------------------------------

with open(ARCHIVO, "w") as f:
    f.write("Tiempo_ms,PWM_porcentaje,RPM\n")

# -----------------------------------------------------------------------------
# Secuencia principal                                                          
# -----------------------------------------------------------------------------

# Construye la secuencia completa 0→100 % y regreso 100→0 % (sin repetir el 0 %)
secuencia_pwm = PASOS_PWM + PASOS_PWM[::-1][1:]

tiempo_inicio = time.ticks_ms()

for porcentaje in secuencia_pwm:
    # ---------------------------------------------------------------------
    # Aplicar nuevo valor de PWM                                           
    # ---------------------------------------------------------------------
    duty = int(65535 * porcentaje / 100)
    pwm_pin.duty_u16(duty)
    print(f"PWM = {porcentaje}%")

    # Variables para almacenar los datos de este escalón
    datos_temporales = []
    tiempo_inicio_paso = time.ticks_ms()
    ultimo_tiempo_medicion = time.ticks_ms()

    # ---------------------------------------------------------------------
    # Medición durante la duración del escalón                             
    # ---------------------------------------------------------------------
    while time.ticks_diff(time.ticks_ms(), tiempo_inicio_paso) < TIEMPO_ENTRE_PASOS:
        tiempo_actual = time.ticks_ms()
        if time.ticks_diff(tiempo_actual, ultimo_tiempo_medicion) >= MEDICION_INTERVALO_MS:
            rpm = calcular_rpm()
            marca_tiempo = time.ticks_diff(tiempo_actual, tiempo_inicio)
            datos_temporales.append((marca_tiempo, porcentaje, rpm))
            ultimo_tiempo_medicion = tiempo_actual

    # ---------------------------------------------------------------------
    # Almacenar los datos de este escalón en el archivo CSV                
    # ---------------------------------------------------------------------
    with open(ARCHIVO, "a") as f:
        for fila in datos_temporales:
            f.write("{},{},{}\n".format(*fila))
        f.flush()  #: Garantiza la escritura inmediata en flash

# -----------------------------------------------------------------------------
# Finalización                                                                 
# -----------------------------------------------------------------------------

pwm_pin.duty_u16(0)  # Apaga la señal PWM
print("Caracterización finalizada. Datos guardados en flash.")
