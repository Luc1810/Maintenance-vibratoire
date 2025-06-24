#include <WiFiNINA.h>

#include <Arduino_LSM6DS3.h>

#define SECRET_SSID "Wifi_OPTA"
#define SECRET_PASS "wifiopta10"
#define SAMPLES 128
#define SAMPLING_FREQUENCY 4000  // en Hz
#define PI 3.14159265358979323846

// Wi-Fi Configuration
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
WiFiClient client;
IPAddress server(192, 168, 40, 1);  // Adresse IP du serveur qui fera la prédiction
int port = 80;

// Buffers
float x, y, z;
float bufferZ[SAMPLES];
float real[SAMPLES];
float imag[SAMPLES];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connexion au Wi-Fi
  Serial.println("Connexion au Wi-Fi...");
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connexion...");
  }
  Serial.println("Connecté au réseau Wi-Fi");

  // Initialisation de l'IMU
  if (!IMU.begin()) {
    Serial.println("Erreur IMU !");
    while (1);
  }

  for (int i = 0; i < SAMPLES; i++) bufferZ[i] = 0.0;
}

void loop() {
  static unsigned long lastSampleTime = 0;
  unsigned long currentMicros = micros();

  if (currentMicros - lastSampleTime >= (1000000 / SAMPLING_FREQUENCY)) {
    lastSampleTime = currentMicros;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      float filteredZ = lowPassFilter(z, 0.1);

      bufferZ[bufferIndex++] = filteredZ;

      if (bufferIndex >= SAMPLES) {
        bufferIndex = 0;

        for (int i = 0; i < SAMPLES; i++) {
          real[i] = bufferZ[i];
          imag[i] = 0.0;
        }

        applyHammingWindow(real, SAMPLES);
        fft(real, imag, SAMPLES);

        float powerSpectrum[SAMPLES / 2];
        for (int i = 0; i < SAMPLES / 2; i++) {
          powerSpectrum[i] = sqrt(real[i] * real[i] + imag[i] * imag[i]);
        }

        sendSpectrumViaWiFi(powerSpectrum, SAMPLES / 2);
      }
    }
  }
}

void sendSpectrumViaWiFi(float* spectrum, int length) {
  if (!client.connect(server, port)) {
    Serial.println("Échec de la connexion au serveur");
    return;
  }

  client.print("SPECTRUM:");
  for (int i = 0; i < length; i++) {
    client.print(spectrum[i], 4);
    if (i < length - 1) client.print(",");
  }
  client.println();
  client.stop();
  Serial.println("Spectre envoyé au serveur pour prédiction.");
}

float lowPassFilter(float input, float alpha) {
  static float filteredValue = 0;
  filteredValue = alpha * input + (1 - alpha) * filteredValue;
  return filteredValue;
}

void applyHammingWindow(float* buffer, int size) {
  for (int i = 0; i < size; i++) {
    float window = 0.54 - 0.46 * cos(2 * PI * i / (size - 1));
    buffer[i] *= window;
  }
}

unsigned int reverseBits(unsigned int x, unsigned int bits) {
  unsigned int y = 0;
  for (unsigned int i = 0; i < bits; ++i) {
    y <<= 1;
    y |= (x & 1);
    x >>= 1;
  }
  return y;
}

void fft(float* real, float* imag, int n) {
  int log2n = log(n) / log(2);

  for (unsigned int i = 0; i < n; ++i) {
    unsigned int j = reverseBits(i, log2n);
    if (j > i) {
      float temp_re = real[i];
      float temp_im = imag[i];
      real[i] = real[j];
      imag[i] = imag[j];
      real[j] = temp_re;
      imag[j] = temp_im;
    }
  }

  for (int s = 1; s <= log2n; ++s) {
    int m = 1 << s;
    float wm_re = cos(2 * PI / m);
    float wm_im = -sin(2 * PI / m);

    for (int k = 0; k < n; k += m) {
      float w_re = 1;
      float w_im = 0;
      for (int j = 0; j < m / 2; ++j) {
        int t = k + j;
        int u = t + m / 2;

        float re_t = w_re * real[u] - w_im * imag[u];
        float im_t = w_re * imag[u] + w_im * real[u];

        real[u] = real[t] - re_t;
        imag[u] = imag[t] - im_t;
        real[t] += re_t;
        imag[t] += im_t;

        float w_next_re = w_re * wm_re - w_im * wm_im;
        float w_next_im = w_re * wm_im + w_im * wm_re;
        w_re = w_next_re;
        w_im = w_next_im;
      }
    }
  }
}

