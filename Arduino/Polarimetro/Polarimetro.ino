// Polarimeter_ESP32.ino
// Migración del código Polarimeter_final.ino para ESP32
// - Usa ADC interno del ESP32 (12 bits)
// - FFT con la librería arduinoFFT
// - Comunicación Serial en lugar de LCD
// - Timers con esp32-hal-timer




// Parámetros generales
#define interruptPin 12   // Pin de entrada del foto-interruptor
#define NPTS 64           // Número de puntos por vuelta
#define NVU 8             // Número de vueltas a acumular
#define adcPin 35         // Pin analógico


// Buffers
volatile uint16_t buffer[NVU][NPTS];  
volatile int j = 0;       // índice dentro de una vuelta
volatile int vuelta = 0;  // índice de vuelta actual
volatile bool startMeasuring = false;
volatile bool readyToSend = false;

// Sincronización
volatile unsigned long prev_trigger = 0;
volatile unsigned long curr_trigger = 0;
volatile unsigned long rotation_period = 20000; // inicial

// Timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;




// ISR del fotointerruptor (reinicia captura por vuelta)
void IRAM_ATTR trigger_ISR() {
    portENTER_CRITICAL_ISR(&timerMux);
    curr_trigger = micros();
    rotation_period = 0.9 * rotation_period + 0.1 * (curr_trigger - prev_trigger);
    prev_trigger = curr_trigger;

    if (!startMeasuring && !readyToSend) {
        startMeasuring = true;
        j = 0;
        buffer[vuelta][j++] = analogRead(adcPin);

        timerAlarm(timer, rotation_period / NPTS, true, 0);
        timerStart(timer);
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}


// ISR del timer (muestreo de puntos)
void IRAM_ATTR sampling_ISR() {
    portENTER_CRITICAL_ISR(&timerMux);
    if (startMeasuring) {
        buffer[vuelta][j++] = analogRead(adcPin);

        if (j >= NPTS) {
            timerStop(timer);
            startMeasuring = false;
            vuelta++;

            if (vuelta >= NVU) {
                vuelta = 0;
                readyToSend = true;  // Se completaron 8 vueltas
            }
        }
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}



// Tarea de comunicación (espera comando del PC y envía datos)
void commTask(void *pvParameters) {
    String command;

    while (true) {
        if (Serial.available()) {
            command = Serial.readStringUntil('\n');

            if (command == "GET") {
                if (readyToSend) {
                    // Construir paquete
                    Serial.print("PERIOD=");
                    Serial.println(rotation_period);

                    for (int v = 0; v < NVU; v++) {
                        for (int k = 0; k < NPTS; k++) {
                            Serial.print(buffer[v][k]);
                            if (k < NPTS - 1) Serial.print(",");
                        }
                        Serial.println(); // Fin de vuelta
                    }

                    readyToSend = false; // Listo para siguiente adquisición
                } else {
                    Serial.println("NOT_READY");
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // evitar bloqueo
    }
}

void setup() {
    Serial.begin(460800);
    pinMode(adcPin, INPUT);
    pinMode(interruptPin, INPUT);

    attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_ISR, RISING);

    timer = timerBegin(1000000); // base de 1 µs
    timerAttachInterrupt(timer, &sampling_ISR);
    timerAlarm(timer, rotation_period / NPTS, true, 0);
    timerStart(timer);

    // Crear tarea de comunicación
    xTaskCreatePinnedToCore(commTask, "commTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // Nada, FreeRTOS maneja todo
}





