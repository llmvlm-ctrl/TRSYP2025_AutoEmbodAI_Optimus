#include <WiFi.h>           // ✅ ESP32 uses this instead of <ESP8266WiFi.h>
#include <PubSubClient.h>

// --- WiFi Parameters ---
const char* ssid = "INTERNET NAME";
const char* password = "PASSWORD";

// --- MQTT Parameters ---
const char* mqtt_server = "54.36.178.49";   // Broker IP
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Client_1";  // ✅ Rename client ID
const char* mqtt_topic_pub = "test/esp32/pub";  // ✅ Rename topic for clarity
const char* mqtt_topic_sub = "test/esp32/sub";

// --- MQTT and WiFi Objects ---
WiFiClient espClient;
PubSubClient client(espClient);

// --- Callback Function (called when a message is received) ---
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message reçu [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // Example: toggle LED when receiving "on" or "off"
  if (message.equalsIgnoreCase("on")) {
    digitalWrite(13, LOW);  // Turn ON LED (active LOW)
  } else if (message.equalsIgnoreCase("off")) {
    digitalWrite(13, HIGH); // Turn OFF LED
  }
}

// --- Connect to WiFi ---
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connexion à ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("✅ WiFi connecté");
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());
}

// --- MQTT Auto-Reconnect ---
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connexion au broker MQTT...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("Connecté !");
      client.subscribe(mqtt_topic_sub);
    } else {
      Serial.print("Échec, code erreur = ");
      Serial.print(client.state());
      Serial.println(" ; nouvelle tentative dans 5 secondes");
      delay(5000);
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // LED off by default

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("ESP32 prêt. Tapez un message dans le Serial pour l’envoyer via MQTT.");
}

// --- Loop ---
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publish message from Serial to MQTT
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    message.trim();
    if (message.length() > 0) {
      client.publish(mqtt_topic_pub, message.c_str());
      Serial.print("Message envoyé : ");
      Serial.println(message);
    }
  }
}
