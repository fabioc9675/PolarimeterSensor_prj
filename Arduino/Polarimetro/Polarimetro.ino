// Polarimeter_ESP32_packeted.ino
// Captura 64 muestras/vuelta, acumula 8 vueltas (512 muestras) y
// responde al comando "GET\n" enviando un paquete único con SAMPLE_PERIOD_US y los 512 valores CSV.

// Ajustes
#define INTERRUPT_PIN 12   // pin del foto-interruptor (ajusta)
#define ADC_PIN 35         // pin ADC (ajusta)
#define NPTS 64            // muestras por vuelta (potencia de 2)
#define NVU 8              // vueltas por paquete
#define TOTAL_SAMPLES (NPTS * NVU)

#define SERIAL_BAUD 460800 // ajustar si quieres (460800 o 921600)

// Buffers y flags compartidos entre ISR y tareas
volatile uint16_t dataBuffer[TOTAL_SAMPLES]; // buffer principal (escrito por ISRs)
volatile int cur_vuelta = 0;   // vuelta actual en la que estamos escribiendo (0..NVU-1)
volatile int cur_index = 0;    // índice dentro de la vuelta (0..NPTS-1)
volatile bool startMeasuring = false;
volatile bool readyToSend = false; // true cuando dataBuffer está completo
volatile bool sending = false;     // true cuando la tarea de comm está enviando

// Sincronización de periodo
volatile unsigned long prev_trigger = 0;
volatile unsigned long rotation_period = 20000; // μs (inicial)

// Timer y mux
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Helper: integer exponential smoothing con N=10 (0.9 y 0.1) sin float
static inline void update_rotation_period(unsigned long delta_us) {
  // rotation_period = 0.9*old + 0.1*delta  => integer: (old*9 + delta)/10
  rotation_period = (rotation_period * 9 + delta_us) / 10;
}


// ISR del fotointerruptor: actualiza rotation_period y arranca muestreo para la nueva vuelta
void IRAM_ATTR trigger_ISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  unsigned long now = micros();
  if (prev_trigger != 0) {
    unsigned long delta = now - prev_trigger;
    update_rotation_period(delta);
  }
  prev_trigger = now;

  // Solo arrancar si no estamos midiendo y no estamos en modo envio
  if (!startMeasuring && !readyToSend && !sending) {
    startMeasuring = true;
    cur_index = 0;
    // Toma la primera muestra inmediatamente (reduce jitter)
    uint16_t v = analogRead(ADC_PIN);
    int global_idx = cur_vuelta * NPTS + cur_index;
    dataBuffer[global_idx] = v;
    cur_index++;

    // Rearma timer con periodo = rotation_period / NPTS (μs por muestra)
    uint32_t period_us = rotation_period / NPTS;
    if (period_us < 1) period_us = 1;
    timerAlarm(timer, period_us, true, 0);
    timerStart(timer);
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}


// ISR del timer: muestrea la señal en cada tick
void IRAM_ATTR sampling_ISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (startMeasuring) {
    uint16_t v = analogRead(ADC_PIN);
    int global_idx = cur_vuelta * NPTS + cur_index;
    dataBuffer[global_idx] = v;
    cur_index++;

    if (cur_index >= NPTS) {
      // Fin de esta vuelta
      cur_index = 0;
      cur_vuelta++;
      startMeasuring = false;
      timerStop(timer);

      if (cur_vuelta >= NVU) {
        // Buffer completo: listo para enviar
        cur_vuelta = 0;        // se reinicia el contador de vueltas (no empezará a grabar hasta que sending sea false)
        readyToSend = true;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}


// Tarea de comunicación: espera "GET\n" y responde con paquete cuando esté listo
void commTask(void *pvParameters) {
  (void)pvParameters;
  const int OUTBUF_SZ = 8192; // suficiente para ~512 valores CSV (ajustar si cambias NVU/NPTS)
  static char outBuf[OUTBUF_SZ];

  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "GET") {
        // Intentar tomar el paquete si está listo
        bool doSend = false;
        unsigned long rotation_period_snapshot = 0;

        // Marca que estamos enviando para que ISRs no empiecen nueva captura
        portENTER_CRITICAL(&timerMux);
        if (readyToSend && !sending) {
          sending = true;
          doSend = true;
          // congelar rotation_period para enviar
          rotation_period_snapshot = rotation_period;
        }
        portEXIT_CRITICAL(&timerMux);

        if (!doSend) {
          Serial.println("NOT_READY");
        } else {
          // Copiar buffer a local (ahora ISRs no comenzarán nuevas adquisiciones por 'sending' = true)
          // NOTA: dataBuffer es volatile, copiar fuera de critical está OK porque sending=true impide escritura
          static uint16_t localCopy[TOTAL_SAMPLES];
          for (int i = 0; i < TOTAL_SAMPLES; ++i) localCopy[i] = dataBuffer[i];

          // Ahora podemos liberar readyToSend y sending (preparar para próximas adquisiciones)
          portENTER_CRITICAL(&timerMux);
          readyToSend = false;
          sending = false; // permitimos nuevas adquisiciones en el siguiente trigger
          portEXIT_CRITICAL(&timerMux);

          // Construir header + CSV en outBuf
          // Header simple: START\nSAMPLE_PERIOD_US=xxx\nTOTAL=nnn\nDATA:
          int pos = 0;
          pos += snprintf(outBuf + pos, OUTBUF_SZ - pos, "START\n");
          pos += snprintf(outBuf + pos, OUTBUF_SZ - pos, "SAMPLE_PERIOD_US=%lu\n", (unsigned long)(rotation_period_snapshot / NPTS));
          pos += snprintf(outBuf + pos, OUTBUF_SZ - pos, "TOTAL=%d\n", TOTAL_SAMPLES);
          pos += snprintf(outBuf + pos, OUTBUF_SZ - pos, "DATA=");

          // Rellenar CSV
          for (int i = 0; i < TOTAL_SAMPLES; ++i) {
            // seguridad: evitar overflow de outBuf
            if (pos + 12 >= OUTBUF_SZ) { // 12 chars por entrada es seguro para 0..65535 +
              break;
            }
            pos += snprintf(outBuf + pos, OUTBUF_SZ - pos, "%u", (unsigned int)localCopy[i]);
            if (i < TOTAL_SAMPLES - 1) {
              outBuf[pos++] = ','; // separador
            }
          }
          // Final de datos
          if (pos + 2 < OUTBUF_SZ) {
            outBuf[pos++] = '\n';
            outBuf[pos] = '\0';
          } else {
            outBuf[OUTBUF_SZ - 1] = '\0';
          }

          // Enviar todo en una sola llamada
          Serial.write((uint8_t*)outBuf, pos);

          // Optional: marcar fin
          Serial.println("END");

          // listo; espera próximo GET
        }
      } // GET
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // pequeña espera
  }
}


void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(ADC_PIN, INPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP); // ajusta si tu sensor necesita pullup/pulldown

  // Attach interrupt del fotointerruptor
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), trigger_ISR, RISING);

  // Timer configurado con base de 1 MHz (1 us ticks)
  timer = timerBegin(1000000); // si tu core no acepta este overload, usa la variante que compila en tu entorno
  timerAttachInterrupt(timer, &sampling_ISR);
  timerAlarm(timer, rotation_period / NPTS, true, 0);
  timerStart(timer);

  // Crear tarea de comunicación en el core 1
  xTaskCreatePinnedToCore(commTask, "commTask", 8192, NULL, 1, NULL, 1);
}

void loop() {
  // Nada en loop(), todo en ISRs y tarea commTask
  delay(1000);
}
