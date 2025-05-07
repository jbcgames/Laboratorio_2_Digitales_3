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

#define PWM_PIN             3
#define PWM_MAX             255     // resolución de 8-bits

#define ENCODER_PIN         5       // pin que se usa como interrupción externa

// =================== VARIABLES DE ESTADO ===================
volatile uint32_t highStartUs = 0;
volatile float rpmGlobal = 0.0;

// --- Filtro de media móvil ---
#define FILTER_SIZE 10
float rpmBuffer[FILTER_SIZE] = {0};
uint8_t rpmIndex = 0;

// --- Caracterización ---
#define MAX_DATOS 1000  // Ajusta según la memoria disponible

// Estructura para almacenar cada muestra (caracterización)
struct Dato {
  uint32_t tiempo_ms;
  int pwm_pct;
  float rpm;
};

Dato datos[MAX_DATOS];
uint16_t indice_dato = 0;

// ========================= PROTOTIPOS =========================
void modoManual(int pct);
bool procesarComando(int &pct);
float mediaMovil(float value);
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

// =================== MODO MANUAL ===================
/**
 * @brief Lee un comando de Serial de manera eficiente; retorna false si es EXIT.
 * @param[in,out] pct  Referencia al porcentaje de ciclo PWM. Se actualiza
 *                     si se recibe un comando "PWM <valor>" válido.
 * @return true  Si debe continuar en modo manual.
 * @return false Si se ha recibido "EXIT" y debe abandonarse el modo manual.
 */
bool procesarComando(int &pct) {
  if (!Serial.available()) return true;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toUpperCase();

  if (line == "EXIT"){
    return false;
  }
  else if (line.startsWith("PWM ")) {
    int nuevo = line.substring(4).toInt();
    pct = constrain(nuevo, 0, 100);
    analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));
    //Serial.print("PWM actualizado: "); Serial.print(pct); Serial.println("%");
  }
  else Serial.println("Comando no reconocido");

  return true;
}

/**
 * @brief Modo manual: aplica PWM fijo y acumula datos hasta recibir 'EXIT'.
 * @param pct Porcentaje de ciclo PWM inicial.
 */
void modoManual(int pct) {
  pct = constrain(pct, 0, 100);
  analogWrite(PWM_PIN, map(pct, 0, 100, 0, PWM_MAX));
  //Serial.print("Modo manual: PWM="); Serial.print(pct); Serial.println("% (EXIT para salir)");

  uint32_t last = millis();
  bool seguir = true;
  indice_dato = 0;

  while (seguir) {
    seguir = procesarComando(pct);
    if (millis() - last >= 500 && indice_dato < MAX_DATOS) {
      float rpmSuavizada = mediaMovil(rpmGlobal);
      datos[indice_dato].tiempo_ms = millis();
      datos[indice_dato].pwm_pct = pct;
      datos[indice_dato].rpm = rpmSuavizada;
      indice_dato++;
      last = millis();
    }
  }

  analogWrite(PWM_PIN, 0);  // se detiene el modo manual

  // Imprimir encabezado CSV
  Serial.println("tiempo_ms,pwm_porcentaje,rpm");

  // Imprimir datos acumulados
  for (uint16_t i = 0; i < indice_dato; i++) {
    Serial.print(datos[i].tiempo_ms); Serial.print(",");
    Serial.print(datos[i].pwm_pct); Serial.print(",");
    Serial.println(datos[i].rpm, 2);
  }

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
  Serial.println("Listo. Comandos: PWM <valor> | EXIT");
}

// =================== BUCLE PRINCIPAL ===================
void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim(); line.toUpperCase();

  if (line == "EXIT") {                         // Apaga PWM y loop infinito para detener el sketch
    analogWrite(PWM_PIN, 0);
  }
  else if (line.startsWith("PWM ")) {           // Modo manual
    int valor = line.substring(4).toInt();
    modoManual(valor);
  }
  else {                                        // Error de comando
    Serial.println("Comando no reconocido");
  }
}
