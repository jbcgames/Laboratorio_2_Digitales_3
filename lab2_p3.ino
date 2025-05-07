/**
 * @file sketch_motor_dc.ino
 * @brief Sistema de caracterización de motor DC en Arduino (compatible Pico).
 * @details Mide velocidad con encoder óptico, genera PWM, captura curva de reacción a 250Hz,
 *          detecta PWM mínimo de arranque y documentado con Doxygen.
 */

// =================== PARÁMETROS GENERALES ===================
#define ENCODER_HUECOS      20                          // huecos por revolución generados por la rueda del encoder
#define SECTORS_REV         (ENCODER_HUECOS * 2)        // huecos + partes opacas
#define RPM_CONST_US        (60000000UL / SECTORS_REV)  // factor de conversión de microsegundos de pulso alto a RPM
#define MIN_DT_US           200                         // ignora pulsos muy cortos (rebotes)

// --- PWM / escalones ---
#define PWM_PIN             3
#define PWM_MAX             255     // resolución de 8-bits
#define DETEC_ARRANQUE_MS   500     // para detectar pwm mínimo

#define ENCODER_PIN         5       // pin que se usa como interrupción externa

// =================== VARIABLES DE ESTADO ===================
volatile uint32_t highStartUs = 0;
volatile float rpmGlobal = 0.0;

// ========================= PROTOTIPOS =========================
int detectarPWMarranque();
void medirPulso();

// =================== ISR: pulso encoder ===================
/**
 * @brief ISR del encoder óptico, mide tiempo alto y actualiza rpmGlobal.
 */
void medirPulso() {
  uint32_t ahora = micros();
  if (digitalRead(ENCODER_PIN) == HIGH) {
    highStartUs = ahora;                  // flanco RISING: marca inicio del pulso alto
  } else {
    uint32_t dt = ahora - highStartUs;    // flanco FALLING: mide duración del pulso
    if (dt >= MIN_DT_US) 
      rpmGlobal = (float)RPM_CONST_US / dt; // actualiza rpmGlobal (convierto duración en RPM)
  }
}

// =================== DETECCIÓN PWM MÍNIMO ===================
/**
 * @brief Encuentra el PWM mínimo que arranca el motor.
 * @return Porcentaje de ciclo mínimo o -1 si no arranca.
 */
int detectarPWMarranque() {
  /*
  Barre PWM de 0 % a 100 % en pasos de 1 %, espera DETEC_ARRANQUE_MS ms en cada uno,
  y si detecta rpmGlobal>1, devuelve ese porcentaje como el mínimo que
  pone en marcha el motor.
  */
  rpmGlobal = 0.0f;  // Reinicia el valor antes de comenzar

  for (uint8_t pct = 0; pct <= 100; pct++) {
    analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));
    delay(DETEC_ARRANQUE_MS);
    
    if (rpmGlobal > 1.0f) return pct;

    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim(); line.toUpperCase();
      if (line == "EXIT") return -1;
    }
  }
  return -1;
}

// =================== SETUP ===================
void setup() {
  // Configuración de pines
  pinMode(PWM_PIN, OUTPUT);
  // Asocia la ISR del encoder
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), medirPulso, CHANGE);
  // Inicializa la consola serial (115200 bps) y espera a que se abra
  Serial.begin(115200);
  while (!Serial);
  // Comando disponibles
  Serial.println("Listo. Comandos: DETECTAR | EXIT");
}

// =================== BUCLE PRINCIPAL ===================
void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim(); line.toUpperCase();

  if (line == "EXIT") {                         // Apaga PWM y loop infinito para detener el sketch
    analogWrite(PWM_PIN, 0);
  }
  else if (line == "DETECTAR") {                // Ejecuta detección de PWM mínimo
    Serial.print("PWM mínimo de arranque: ");
    Serial.println(String(detectarPWMarranque()) + "%");
  }
  else {                                        // Error de comando
    Serial.println("Comando no reconocido");
  }
}
