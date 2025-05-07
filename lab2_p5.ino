/**
 * @file sketch_motor_dc.ino
 * @brief Sistema de caracterización de motor DC en Arduino (compatible Pico).
 */

// =================== PARÁMETROS GENERALES ===================
#define ENCODER_HUECOS      20                          // huecos por revolución generados por la rueda del encoder
#define SECTORS_REV         (ENCODER_HUECOS * 2)        // huecos + partes opacas
#define RPM_CONST_US        (60000000UL / SECTORS_REV)  // factor de conversión de microsegundos de pulso alto a RPM
#define MIN_DT_US           200                         // ignora pulsos muy cortos (rebotes)

// --- PWM / escalones ---
#define PWM_PIN             3
#define PWM_MAX             255     // resolución de 8-bits

#define TIEMPO_ESCALON_MS   2000    // escalón de 2 segundos
#define MUESTREO_MS         4       // muestreo cada 4 ms (250 Hz)

// Tamaño máximo de buffer (ajustar según RAM disponible)
#define MAX_SAMPLES         25000   // tamaño máximo de muestras en el buffer para almacenar información
                                    // 25000 × (4+1+4)bytes

#define ENCODER_PIN         5       // pin que se usa como interrupción externa

// =================== VARIABLES DE ESTADO ===================
volatile uint32_t highStartUs = 0;
volatile float rpmGlobal = 0.0;

// --- Filtro de media móvil ---
#define FILTER_SIZE 10
float rpmBuffer[FILTER_SIZE] = {0};
uint8_t rpmIndex = 0;

// --- Caracterización automática (con pasoPercent predefinido) ---
uint32_t bufTimestamp[MAX_SAMPLES];   // (4 bytes)
uint8_t bufPWM[MAX_SAMPLES];          // (1 byte)
float bufRPM[MAX_SAMPLES];            // (4 bytes)
int16_t bufIndex = 0;

uint8_t currentPct = 0;               // porcentaje del ciclo de trabajo PWM (de 0 a 100)
uint32_t capturaStartUs = 0;          // 0 a 4294967295 microsegundos (aproximadamente 71.6 minutos)                ---> instante de inicio de la captura, en µs

// ========================= PROTOTIPOS =========================
void modoManual(int pct);
bool procesarComando(int &pct);
float mediaMovil(float value);

void ejecutarCaptura(int pasoPercent);

void medirPulso();

// =================== FUNCIÓN DE MEDIA MÓVIL ===================
/**
 * @brief Inserta una nueva lectura de RPM en el buffer circular y retorna la media.
 * @param[in] value  Lectura actual de rpmGlobal.
 * @return float     Media de las últimas FILTER_SIZE lecturas.
 */
float mediaMovil(float value) {
  rpmBuffer[rpmIndex] = value;
  rpmIndex = (rpmIndex + 1) % FILTER_SIZE;
  float sum = 0;
  for (uint8_t i = 0; i < FILTER_SIZE; i++)
    sum += rpmBuffer[i];
  return sum / FILTER_SIZE;
}

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

// =================== MODO CAPTURA AUTOMÁTICA ===================
/**
 * @brief Captura curva de reacción: guarda en buffer a 250Hz y envía CSV al finalizar.
 * @param pasoPercent Paso de PWM para escalones.
 */
void ejecutarCaptura(int pasoPercent) {
  bufIndex = 0;
  capturaStartUs = micros();
  uint32_t lastSampleUs = capturaStartUs;

  const int maxPercent = 100;                           // PWM va de 0% a 100%
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
        // TIEMPO_ESCALON_MS * 1000UL es para pasarlo a microsegundos
        // MUESTREO_MS * 1000UL es para pasarlo a microsegundos
      */
      while (micros() - stepStartUs < TIEMPO_ESCALON_MS * 1000UL) { // mientras no se cumplan los 2 segundos (on + off)
        uint32_t nowUs = micros();
        if (nowUs - lastSampleUs >= MUESTREO_MS * 1000UL) {         // si la dif. entre ahora y la última vez que se muestreó >= 4ms ---> se guarda muestra
          uint32_t elapsedMs = (nowUs - capturaStartUs) / 1000UL;
          if (bufIndex < MAX_SAMPLES) {                             // (500 x --50--) < 25000, aprox. 50 escalones
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
 * @brief Lee un comando de Serial de manera eficiente.
 * @param[in,out] pct  Referencia al porcentaje de ciclo PWM. Se actualiza
 *                     si se recibe un comando "PWM <valor>" válido.
 * @return true  Si debe continuar en modo manual.
 * @return false Si se ha recibido otro comando, debe abandonarse el modo manual.
 */
bool procesarComando(int &pct) {
  if (!Serial.available()) return true;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toUpperCase();

  if (line.startsWith("PWM ")) {
    int nuevo = line.substring(4).toInt();
    pct = constrain(nuevo, 0, 100);
    analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));
    //Serial.print("PWM actualizado: "); Serial.print(pct); Serial.println("%");
  }
  else if (line.startsWith("START ")) {
    Serial.println("Comando START, vuelva a ejecutarlo");
    return false;
  }
  else {
    Serial.println("Comando no reconocido");
    return false;
  }

  return true;
}

/**
 * @brief Modo manual: aplica PWM fijo y acumula datos hasta recibir otro tipo de comando.
 * @param pct Porcentaje de ciclo PWM inicial.
 */
void modoManual(int pct) {
  pct = constrain(pct, 0, 100);
  analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));
  //Serial.print("Modo manual: PWM="); Serial.print(pct); Serial.println("% (EXIT para salir)");

  uint32_t last = millis();
  bool seguir = true;

  while (seguir) {
    seguir = procesarComando(pct);
    if (millis() - last >= 500) {
      float rpmSuavizada = mediaMovil(rpmGlobal);
      Serial.print("PWM="); Serial.print(pct);
      Serial.print("% | RPM="); Serial.println(rpmSuavizada, 2);
      last = millis();
    }
  }

  analogWrite(PWM_PIN, 0);  // se detiene el modo manual

  Serial.println("CAPTURA_FINALIZADA");
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
  Serial.println("Listo. Comandos: START <paso> | PWM <valor>");
}

// =================== BUCLE PRINCIPAL ===================
void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim(); line.toUpperCase();

  if (line.startsWith("START ")) {         // Pasa a captura automática con el paso indicado
    ejecutarCaptura(line.substring(6).toInt());
  }
  else if (line.startsWith("PWM ")) {           // Modo manual
    int valor = line.substring(4).toInt();
    modoManual(valor);
  }
  else {                                        // Error de comando
    Serial.println("Comando no reconocido");
  }
}
