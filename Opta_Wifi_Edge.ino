#include <Opta_trainingData_inferencing.h>
#include <WiFi.h>


#define SECRET_SSID "Wifi_OPTA"
#define SECRET_PASS "wifiopta10"
#define TAILLE_BUFFER 64  // Taille du buffer pour le spectre de puissance

//*** WiFi Configuration ***//
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;

WiFiServer wifi_server(80);
WiFiClient currentClient;
bool clientConnected = false;
IPAddress WiFi_opta(192, 168, 40, 1);
IPAddress WiFi_subnet(255, 255, 255, 0);
IPAddress WiFi_gateway(192, 168, 40, 1);

// Buffers des valeurs et spectre de puissance
float powerSpectrumZ[TAILLE_BUFFER];
bool spectrumReady = false;

// Synchronisation des actions
unsigned long lastMillisWiFi = 0;
const unsigned long wifiInterval = 100;    // Intervalle pour gérer les clients Wi-Fi

void setup() {
  Serial.begin(115200);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, HIGH);

  // Initialisation du Wi-Fi
  WiFi.config(WiFi_opta, WiFi_gateway, WiFi_subnet);
  status = WiFi.beginAP(ssid, pass);
  delay(3000);
  wifi_server.begin();

  for (int i = 0; i < TAILLE_BUFFER; i++) powerSpectrumZ[i] = 0.0;
}

void loop() {
  unsigned long currentMillis = millis();

  // Gestion des clients Wi-Fi à intervalles réguliers
  if (currentMillis - lastMillisWiFi >= wifiInterval) {
    lastMillisWiFi = currentMillis;
    handleWiFiClient();
  }

  // Envoi vers Edge Impulse uniquement si spectre valide
  if (spectrumReady) {
    runEdgeImpulsePrediction();
    spectrumReady = false;
  }
}

// Gestion des clients Wi-Fi et traitement des données reçues
void handleWiFiClient() {
  WiFiClient newClient = wifi_server.available();
  if (newClient) {
    String currentLine = "";

    while (newClient.connected()) {
      while (newClient.available() > 0) {
        char c = newClient.read();

        if (c == '\n') {
          if (currentLine.startsWith("SPECTRUM:")) {
            processSpectrumData(currentLine);
            sendSpectrum(powerSpectrumZ, TAILLE_BUFFER, newClient); // Envoi du spectre au client Wi-Fi
          }
          currentLine = "";
          break;
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    newClient.stop();
  }
}

// Traitement des données de spectre de puissance
void processSpectrumData(String data) {
  data.remove(0, 9);
  int index = 0;
  char* cstr = new char[data.length() + 1];
  strcpy(cstr, data.c_str());
  char* token = strtok(cstr, ",");

  while (token != nullptr && index < TAILLE_BUFFER) {
    powerSpectrumZ[index] = atof(token);
    token = strtok(nullptr, ",");
    index++;
  }
  delete[] cstr;
  spectrumReady = true;
}

// Envoi des données du spectre au client Wi-Fi
void sendSpectrum(float* spectrum, int length, WiFiClient &client) {
  client.print("SPECTRUM:");
  for (int i = 0; i < length; i++) {
    client.print(spectrum[i]);
    if (i < length - 1) client.print(",");
  }
  client.println();
}

// Exécution de la prédiction Edge Impulse
void runEdgeImpulsePrediction() {
  signal_t signal;
  numpy::signal_from_buffer(powerSpectrumZ, TAILLE_BUFFER, &signal);

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

  if (res != EI_IMPULSE_OK) {
    Serial.println("Erreur de classification");
    return;
  }

  // Affichage du résultat
  Serial.println("=== Résultat Edge Impulse ===");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    Serial.print(result.classification[ix].label);
    Serial.print(": ");
    Serial.print(result.classification[ix].value * 100, 2);
    Serial.println(" %");
  }
  Serial.println("===========================");
}

// Envoi vers Edge Impulse via Serial (ligne brute)
void sendSpectrumToSerial(float* spectrum, int length) {
  for (int i = 0; i < length; i++) {
    if (!isnan(spectrum[i])) {
      Serial.print(spectrum[i], 4);
    } else {
      Serial.print(0.0000);
    }
    if (i < length - 1) Serial.print(",");
  }
  Serial.println();
}
