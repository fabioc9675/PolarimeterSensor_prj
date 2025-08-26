// Polarimeter_ESP32.ino
// Migración del código Polarimeter_final.ino para ESP32
// - Usa ADC interno del ESP32 (12 bits)
// - FFT con la librería arduinoFFT
// - Comunicación Serial en lugar de LCD
// - Timers con esp32-hal-timer




// Parámetros generales
#define interruptPin 12 // Pin de entrada del foto-interruptor (ajusta según tu hardware)
#define NPTS 128         // Número de puntos por ciclo (potencia de 2)
#define adcPin 35       // Pin analógico para el fotodiodo


// FFT
volatile double vReal[NPTS];
volatile double vImag[NPTS];
volatile int j = 0;


// Variables de sincronización
volatile unsigned long prev_trigger = 0;
volatile unsigned long curr_trigger = 0;
volatile unsigned long rotation_period = 20000; // valor inicial en µs
volatile bool startMeasuring = false;
volatile bool processing = false;


// Timer del ESP32
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;






void IRAM_ATTR trigger_ISR()
{
    portENTER_CRITICAL_ISR(&timerMux);
    curr_trigger = micros();
    rotation_period = 0.9 * rotation_period + 0.1 * (curr_trigger - prev_trigger);
    prev_trigger = curr_trigger;

    if (!processing && !startMeasuring)
    {

        //Serial.println("Reinicia timer");
        startMeasuring = true;
        vReal[0] = analogRead(adcPin);
        j = 1;
        // reiniciar timer con nuevo periodo
        //Serial.print("Rotacion = ");
        //Serial.println(rotation_period / NPTS);

        timerAlarm(timer, rotation_period / NPTS, true, 0);
        timerStart(timer);

        
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}



void IRAM_ATTR sampling_ISR()
{
    portENTER_CRITICAL_ISR(&timerMux);
    if (startMeasuring)
    {
        if (j == NPTS - 1)
        {
            timerStop(timer);
            startMeasuring = false;
            processing = true;
        }
        vReal[j] = analogRead(adcPin);
        j = (j + 1) % NPTS;
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}


void setup()
{
    Serial.begin(460800);
    pinMode(adcPin, INPUT);
    pinMode(interruptPin, INPUT);

    attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_ISR, RISING);

    // Timer del ESP32 → ticks de 1 µs (80 MHz / 80 = 1 MHz)
    timer = timerBegin(1000000); // 1 MHz base (1 µs ticks)
    timerAttachInterrupt(timer, &sampling_ISR);
    timerAlarm(timer, rotation_period / NPTS, true, 0);
    timerStart(timer);

    // Serial.println("Polarimeter ESP32 iniciado...");
}

void loop()
{
    if (processing)
    {
        // FFT
        for (int k = 0; k < NPTS; k++) {
          Serial.print(vReal[k]);
          if (k < NPTS - 1) Serial.print(","); // separador
        }
        Serial.println(); // marca de fin de bloque
        
        processing = false; // listo para el próximo ciclo
    }
}






