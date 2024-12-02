/**
 * Run a TensorFlow model to predict the IRIS dataset
 * For a complete guide, visit
 * https://eloquentarduino.com/tensorflow-lite-esp32
 */
// replace with your own model
// include BEFORE <eloquent_tinyml.h>!
#include "modelo_esp32.h" // Tu modelo en formato .h
// include the runtime specific for your board
// either tflm_esp32 or tflm_cortexm
#include <tflm_esp32.h>
// now you can include the eloquent tinyml wrapper
#include <eloquent_tinyml.h>


// Configuración del micrófono MAX4466
const int sampleWindow = 5000; // Ventana de muestreo en ms
const int AMP_PIN = 34;        // Pin del micrófono (ADC1_CHANNEL6)

// Configuración de EloquentTinyML
#define ARENA_SIZE 2 * 1024  // Tamaño del área para la memoria
#define TF_NUM_OPS 10       // Número de operaciones en el modelo (ajusta según el modelo)

Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;

void setup() {
    Serial.begin(115200);

    // Inicializar el modelo
    Serial.println("Cargando el modelo...");
    while (!tf.begin(modelo_esp32).isOk()) {
        Serial.println(tf.exception.toString());
        delay(1000);
    }

    Serial.println("Modelo cargado correctamente. Listo para inferencias.");
}

void loop() {
    unsigned long startMillis = millis();
    unsigned int signalMax = 0;
    unsigned int signalMin = 4095;
    unsigned int sample;

    // Leer datos del micrófono durante la ventana de muestreo
    while (millis() - startMillis < sampleWindow) {
        sample = analogRead(AMP_PIN);
        if (sample < 4095) { // Filtrar lecturas espurias
            if (sample > signalMax) {
                signalMax = sample;
            }
            if (sample < signalMin) {
                signalMin = sample;
            }
        }
    }

    // Calcular la amplitud pico a pico
    unsigned int peakToPeak = signalMax - signalMin;

    // Normalizar la entrada para el modelo (entre 0 y 1)
    float normalizedValue = (float)peakToPeak / 4095.0;

    // Ejecutar la inferencia
    float input[1] = {normalizedValue};
    if (!tf.predict(input).isOk()) {
        Serial.println(tf.exception.toString());
        return;
    }

    // Leer el resultado de la inferencia
    float output0 = tf.output(0); // Primera salida del modelo
    float output1 = tf.output(1); // Segunda salida del modelo

    // Mostrar resultados
    Serial.print("Salida 0: ");
    Serial.println(output0);
    Serial.print("Salida 1: ");
    Serial.println(output1);

    // Evaluar el resultado y tomar decisiones
    if (output0 > output1) {
        Serial.println("Predicción: No es una motosierra");
    } else {
        Serial.println("Predicción: Es una motosierra");
    }

    delay(1000); // Esperar antes del próximo ciclo
}