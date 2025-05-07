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
#define MIN_DT_US           100                         // ignora pulsos muy cortos (rebotes)

// --- PWM / escalones ---
#define PWM_PIN             3
#define PWM_MAX             255     // resolución de 8-bits
#define TIEMPO_ESCALON_MS   2000    // escalón de 2 segundos
#define MUESTREO_MS         4       // muestreo cada 4 ms (250 Hz)
#define DETEC_ARRANQUE_MS   500     // para detectar pwm mínimo

// Tamaño máximo de buffer (ajustar según RAM disponible)
#define MAX_SAMPLES         25000   // tamaño máximo de muestras en el buffer para almacenar información
                                    // 25000 × (4+1+4)bytes
// --- Hardware ---
#define ENCODER_PIN         5       // pin que se usa como interrupción externa

// =================== VARIABLES DE ESTADO ===================
volatile uint32_t highStartUs = 0;
volatile float rpmGlobal = 0.0;

uint32_t bufTimestamp[MAX_SAMPLES];   // (4 bytes)
uint8_t bufPWM[MAX_SAMPLES];          // (1 byte)
float bufRPM[MAX_SAMPLES];            // (4 bytes)
int16_t bufIndex = 0;

bool modoCaptura = false;             // sistema no empieza a registrar datos hasta que se activa con START <paso>
uint8_t currentPct = 0;               // porcentaje del ciclo de trabajo PWM (de 0 a 100)
uint32_t capturaStartUs = 0;          // 0 a 4294967295 microsegundos (aproximadamente 71.6 minutos)                ---> instante de inicio de la captura, en µs

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
  for (uint8_t pct = 0; pct <= 100; pct++) {
    analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));
    delay(DETEC_ARRANQUE_MS);
    if (rpmGlobal > 1.0f) return pct;
  }
  return -1;
}

// =================== MODO CAPTURA AUTOMÁTICA ===================
/**
 * @brief Captura curva de reacción: guarda en buffer a 250Hz y envía CSV al finalizar.
 * @param pasoPercent Paso de PWM para escalones.
 */
void ejecutarCaptura(int pasoPercent) {
  modoCaptura = true;
  bufIndex = 0;
  capturaStartUs = micros();
  uint32_t lastSampleUs = capturaStartUs;

  const int maxPercent = 100;
  for (uint8_t dir = 0; dir < 2; dir++) {               // controla subida (0) ó bajada (1)
    for (int pct = (dir == 0 ? 0 : maxPercent - pasoPercent); //inicio
         (dir == 0 ? pct <= maxPercent : pct >= 0);           //condición
         pct += (dir == 0 ? pasoPercent : -pasoPercent)) {    //incrementa ó decrementa
      currentPct = pct;
      analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));

      uint32_t stepStartUs = micros();
      /*
        La duración del escalón es de 2s (2000ms) (TIEMPO_ESCALON_MS) y dentro de este
        escalón se va muestreando la curva de reacción cada 4ms (MUESTREO_MS), para así
        tener 500 muestras de cada curva de reacción.
      */
      while (micros() - stepStartUs < TIEMPO_ESCALON_MS * 1000UL) { // TIEMPO_ESCALON_MS * 1000UL es para pasarlo a microsegundos
        uint32_t nowUs = micros();
        if (nowUs - lastSampleUs >= MUESTREO_MS * 1000UL) {         // MUESTREO_MS * 1000UL es para pasarlo a microsegundos
          uint32_t elapsedMs = (nowUs - capturaStartUs) / 1000UL;
          if (bufIndex < MAX_SAMPLES) {
            // Guardar una muestra: tiempo, PWM actual y rpmGlobal
            bufTimestamp[bufIndex] = elapsedMs;
            bufPWM[bufIndex]       = currentPct;
            bufRPM[bufIndex]       = rpmGlobal;
            bufIndex++;
          }
          lastSampleUs = nowUs;
        }
      }
    }
  }

  modoCaptura = false;
  analogWrite(PWM_PIN, 0);

  // Enviar buffer completo en CSV
  Serial.println("Tiempo_ms,PWM_porcentaje,RPM");
  for (uint16_t i = 0; i < bufIndex; i++) {
    Serial.print(bufTimestamp[i]); Serial.print(',');
    Serial.print(bufPWM[i]);       Serial.print(',');
    Serial.println(bufRPM[i], 2);
  }
  Serial.println("CAPTURA_FINALIZADA");
}

// =================== MODO MANUAL ===================
/**
 * @brief Modo manual: aplica PWM fijo y muestra datos a 2 Hz.
 * @param pct Porcentaje de ciclo.
 */
void modoManual(int pct) {
  /*
  Aplica un PWM fijo y, cada 500 ms, imprime el porcentaje y la RPM actual. Solo
  sale del modo cuando recibe “EXIT” por serial.
  */
  analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));
  Serial.print("Modo manual: PWM="); Serial.print(pct); Serial.println("% (EXIT para salir)");
  uint32_t last = millis();
  while (true) {
    if (Serial.available() && Serial.readStringUntil('\n') == "EXIT") break;
    if (millis() - last >= 500) {
      Serial.print("PWM="); Serial.print(pct);
      Serial.print("% | RPM="); Serial.println(rpmGlobal, 2);
      last = millis();
    }
  }
  analogWrite(PWM_PIN, 0);
  Serial.println("Modo manual detenido");
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
  // Comandos disponibles
  Serial.println("Listo. Comandos: DETECTAR | START <paso> | PWM <valor> | EXIT");
}

// =================== BUCLE PRINCIPAL ===================
void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim(); line.toUpperCase();

  if (line == "EXIT") {                         // Apaga PWM y loop infinito para detener el sketch
    analogWrite(PWM_PIN, 0);
    Serial.println("Saliendo...");
    while (true);
  }
  else if (line == "DETECTAR") {                // Ejecuta detección de PWM mínimo
    Serial.print("PWM minimo de arranque: "); Serial.print(detectarPWMarranque()); Serial.println("%");
  }
  else if (line.startsWith("START ")) {         // Pasa a captura automática con el paso indicado
    ejecutarCaptura(line.substring(6).toInt());
  }
  else if (line.startsWith("PWM ")) {           // Entra en modo manual
    modoManual(line.substring(4).toInt());
  }
  else {                                        // Error de comando
    Serial.println("Comando no reconocido");
  }
}
