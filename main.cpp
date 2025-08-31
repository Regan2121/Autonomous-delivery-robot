#include <WiFi.h>
  #include <WebServer.h>
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_HMC5883_U.h>
  #include <TinyGPS++.h>
  #include <HardwareSerial.h>
  
  // ---------- WiFi Configuration ----------
  const char* apSSID = "ESP32-AP";  // Access Point SSID
  const char* apPassword = "12345678";  // Access Point Password
  
  const char* staSSID = "YourNetworkSSID";  // Change this to your router's SSID
  const char* staPassword = "YourNetworkPassword";  // Change this to your router's password
  
  WebServer server(80);
  
  // ---------- HTML Page ----------
  const char* index_html = R"rawliteral(
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="UTF-8">
      <title>ESP32 GPS Client - Autonomous Delivery Robot</title>
      <style>
        @import url('https://fonts.googleapis.com/css2?family=Poppins:wght@400;600;700&display=swap');
    
        body {
          font-family: 'Poppins', sans-serif;
          background: linear-gradient(135deg, #4158D0, #C850C0, #FFCC70);
          background-size: 400% 400%;
          animation: gradientBG 10s ease infinite;
          height: 100vh;
          margin: 0;
          display: flex;
          justify-content: center;
          align-items: center;
          overflow: hidden;
          flex-direction: column;
          text-align: center;
        }
    
        @keyframes gradientBG {
          0% {background-position: 0% 50%;}
          50% {background-position: 100% 50%;}
          100% {background-position: 0% 50%;}
        }
    
        .card {
          background: rgba(255, 255, 255, 0.15);
          box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
          backdrop-filter: blur(8px);
          -webkit-backdrop-filter: blur(8px);
          border-radius: 20px;
          padding: 40px 30px;
          color: #fff;
          max-width: 550px;
          width: 90%;
          margin-bottom: 20px;
        }
    
        h1 {
          font-size: 2.5rem;
          margin-bottom: 10px;
          text-shadow: 0px 0px 10px rgba(255,255,255,0.8);
        }
    
        .team {
          font-size: 1.1rem;
          margin: 20px 0;
          color: #ffe;
        }
    
        .team-member {
          margin: 5px 0;
        }
    
        button {
          background: #00F260;
          background: linear-gradient(45deg, #0575E6, #00F260);
          color: white;
          border: none;
          border-radius: 50px;
          padding: 15px 40px;
          font-size: 1.5rem;
          cursor: pointer;
          transition: all 0.3s ease;
          box-shadow: 0 0 20px #00f2ff, 0 0 30px #00f2ff;
          margin-top: 20px;
        }
    
        button:hover {
          transform: scale(1.1);
          box-shadow: 0 0 30px #00f2ff, 0 0 60px #00f2ff;
        }
    
        p {
          margin-top: 20px;
          font-size: 1.1rem;
          color: #e0e0e0;
        }
    
        #output {
          margin-top: 30px;
          font-size: 1.2rem;
          background: rgba(255, 255, 255, 0.1);
          padding: 15px;
          border-radius: 15px;
          word-wrap: break-word;
        }
    
        .footer {
          margin-top: 20px;
          font-size: 1rem;
          color: #fff;
        }
      </style>
    </head>
    <body>
    
      <div class="card">
        <h1>🚀 Autonomous Delivery Robot</h1>
        
        <div class="team">
          <strong>Team Members:</strong>
          <div class="team-member">Regan M</div>
          <div class="team-member">Pavan Sai E</div>
          <div class="team-member">Pradeep Vasan R</div>
          <div class="team-member">Praven O R</div>
        </div>
    
        <button onclick="sendClientGPS()">📍 Send My Location</button>
        <p>Share your live GPS location with ESP32 Rover!</p>
        <div id="output"></div>
      </div>
    
      <div class="footer">
        Developed as part of the Autonomous Delivery Robot Project 🚚
      </div>
    
      <script>
        function sendClientGPS() {
          if (navigator.geolocation) {
            navigator.geolocation.getCurrentPosition(function(position) {
              const gpsData = {
                lat: position.coords.latitude,
                lon: position.coords.longitude
              };
              fetch("/clientgps", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(gpsData)
              })
              .then(response => response.text())
              .then(data => {
                alert("📡 Response from ESP32: " + data);
                document.getElementById('output').innerHTML = `
                  <strong>✅ Sent Coordinates:</strong><br>
                  Latitude: ${gpsData.lat.toFixed(6)}<br>
                  Longitude: ${gpsData.lon.toFixed(6)}
                `;
              })
              .catch(error => alert("❗Error: " + error));
            }, function(error) {
              alert("❗Error obtaining location: " + error.message);
            });
          } else {
            alert("❗Geolocation not supported by this browser.");
          }
        }
      </script>
    
    </body>
    </html>
  )rawliteral";
  
  // ---------- GPS and Compass Setup ----------
  TinyGPSPlus gps;
  Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
  HardwareSerial GPS_Serial(1);  // RX = GPIO16, TX = GPIO17
  
  // ---------- Motor Pins ----------
  #define IN1 32
  #define IN2 33
  #define IN3 25
  #define IN4 26
  #define ENA 27
  #define ENB 14
  
  // ---------- Navigation Variables ----------
  double destLat = 0.0;
  double destLon = 0.0;
  const double arrivalThreshold = 3.0;  // Target arrival threshold in meters
  bool reached = false;
  bool isMoving = false;
  unsigned long lastCorrectTime = 0;
  unsigned long currentTime = 0;
  double initialHeading = 0.0;
  
  void setup() {
    Serial.begin(115200);
    GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);  // RX = 16, TX = 17
  
    Wire.begin();
  
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
    if (!compass.begin()) {
      Serial.println("⚠ Compass not detected!");
      while (1);  // Halt the program if the compass is not found
    }
  
    // Start the ESP32 in AP mode
    WiFi.softAP(apSSID, apPassword);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  
    // Start the web server
    server.on("/", HTTP_GET, []() {
      server.send(200, "text/html", index_html);
    });
  
    // Handle receiving GPS coordinates from the client via POST request
    server.on("/clientgps", HTTP_POST, []() {
      String body = server.arg("plain");
      int latIndex = body.indexOf("lat");
      int lonIndex = body.indexOf("lon");
      if (latIndex > -1 && lonIndex > -1) {
        destLat = body.substring(body.indexOf(":", latIndex)+1, body.indexOf(",")).toDouble();
        destLon = body.substring(body.lastIndexOf(":")+1, body.indexOf("}")).toDouble();
        Serial.println("📍 Target received via Web:");
        Serial.println("Latitude: " + String(destLat));
        Serial.println("Longitude: " + String(destLon));
        server.send(200, "text/plain", "GPS data received!");
      } else {
        server.send(400, "text/plain", "Invalid data");
      }
    });
  
    server.begin();
    Serial.println("HTTP server started");
  }
  
  void loop() {
    server.handleClient();  // Handle client requests
  
    while (GPS_Serial.available()) {
      gps.encode(GPS_Serial.read());  // Feed data to the GPS object
    }
  
    if (gps.location.isUpdated() && destLat != 0.0 && destLon != 0.0) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      double distance = haversine(lat, lon, destLat, destLon);
      double azimuth = calculateAzimuth(lat, lon, destLat, destLon);
      double heading = getHeading();
      double diff = angleDifference(heading, azimuth);
  
      Serial.println("\n========== GPS & Heading ==========");
      Serial.printf("Location: %.6f, %.6f\n", lat, lon);
      Serial.printf("Distance: %.2f m\nAzimuth: %.2f\nHeading: %.2f\nAngle Error: %.2f\n", distance, azimuth, heading, diff);
      Serial.println("===================================");
  
      if (distance <= arrivalThreshold) {
        if (!reached) {
          stopMotors();
          Serial.println("✅ Destination reached.");
          reached = true;
          isMoving = false;
        }
      } else {
        reached = false;
        int speed = (distance <= 10) ? 100 : (distance <= 20) ? 180 : 255;
        if (!isMoving) {
          initialHeading = heading;
          if (abs(diff) > 10) rotateToAlign(diff);
          else {
            moveForward(speed);
            isMoving = true;
          }
        } else {
          currentTime = millis();
          if (currentTime - lastCorrectTime >= 10000) {
            lastCorrectTime = currentTime;
            double newHeading = getHeading();
            double headingError = angleDifference(initialHeading, newHeading);
            if (abs(headingError) > 10) rotateToAlign(headingError);
          }
          moveForward(speed);
        }
      }
    }
    delay(1000);
  }
  
  // ---------- Navigation Functions ----------
  double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000;  // Earth radius in meters
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double a = sin(dLat/2)*sin(dLat/2) + cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
  }
  
  double calculateAzimuth(double lat1, double lon1, double lat2, double lon2) {
    double dLon = radians(lon2 - lon1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
    double bearing = degrees(atan2(y, x));
    return fmod((bearing + 360), 360);
  }
  
  double getHeading() {
    sensors_event_t event;
    compass.getEvent(&event);
    double heading = atan2(event.magnetic.y, event.magnetic.x);
    heading = degrees(heading);
    if (heading < 0) heading += 360;
    return heading;
  }
  
  double angleDifference(double from, double to) {
    double diff = fmod((to - from + 540.0), 360.0) - 180.0;
    return diff;
  }
  
  void moveForward(int speed) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  
  void rotateToAlign(double error) {
    stopMotors();
    delay(500);
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    if (error > 0) {
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    }
    delay(800);
    stopMotors();
    delay(500);
  }
  
  void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }