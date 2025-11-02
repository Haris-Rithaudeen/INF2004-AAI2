# MQTT Test Procedure (Pico W and Mosquitto Broker)

## 1. Configure `wifi_credentials.h`

1. In your project directory, create a file named **`wifi_credentials.h`**.
2. Add the following lines (replace the placeholders with your actual Wi-Fi SSID, password, and your computer’s Wi-Fi IPv4 address):

   ```c
   #define WIFI_SSID_ACTUAL        "YourWiFiSSID"
   #define WIFI_PASSWORD_ACTUAL    "YourWiFiPassword"
   #define MQTT_BROKER_IP_ACTUAL   "192.168.1.xx"   // Your computer’s Wi-Fi IP
   ```

3. In `wifi_mqtt.h`, ensure this line is present near the top:

   ```c
   #define USE_WIFI_CREDENTIALS_FILE
   ```

   This allows your main code to use the credentials from `wifi_credentials.h`.

---

## 2. Set Up Mosquitto Broker on Windows

1. Locate or create a configuration file at:

   ```
   C:\Program Files\mosquitto\mosquitto.conf
   ```

2. Edit the file (run Notepad as Administrator) and **add these lines at the end**:

   ```
   per_listener_settings true
   listener 1883 0.0.0.0
   allow_anonymous true
   ```

3. Save and close the file.

---

## 3. Allow Mosquitto Through Windows Firewall

1. Open **Command Prompt as Administrator**.
2. Run this command:

   ```bat
   netsh advfirewall firewall add rule name="Mosquitto 1883" dir=in action=allow protocol=TCP localport=1883
   ```

3. You should see `Ok.` confirming the rule was added.

---

## 4. Run Mosquitto Broker

1. Open a new **Command Prompt (Administrator)**.
2. Start the broker manually:

   ```bat
   cd "C:\Program Files\mosquitto"
   mosquitto -v -c mosquitto.conf
   ```

3. Confirm it prints:

   ```
   Config loaded from ...\mosquitto.conf
   Opening ipv4 listen socket on port 1883.
   ```

   Leave this window open while testing.

---

## 5. Sanity Check (Local Test)

1. Open another Command Prompt.
2. In one window, subscribe to a test topic:

   ```bat
   mosquitto_sub -h 127.0.0.1 -t test -v
   ```

3. In another window, publish to the same topic:

   ```bat
   mosquitto_pub -h 127.0.0.1 -t test -m hello
   ```

4. You should see the subscriber print:

   ```
   test hello
   ```

   This confirms the broker is working locally.

---

## 6. Connect the Pico W to Wi-Fi and Broker

1. Ensure your **Pico and computer are on the same Wi-Fi network** (2.4 GHz only).
2. In your code, set `MQTT_BROKER_IP` to your computer’s Wi-Fi IPv4.
3. Flash and run your Pico.
4. Open the **serial monitor** in VS Code or Thonny.
5. You should see:

   ```
   [WiFi] ✓ Connected successfully
   [MQTT] ✓ Connected to broker
   [MQTT] ✓ Subscribed to cmd/robot1/#
   [MQTT->] tele/robot1/heartbeat {"uptime_ms":12345,"status":"ok"}
   ```

---

## 7. Publish and Subscribe Test

1. Keep the broker window and serial monitor open.
2. Open two additional Command Prompts.

### A. Subscriber Window
Watch what the Pico publishes:
```bat
mosquitto_sub -h 192.168.1.xx -t tele/robot1/# -v
```

Expected output:
```
tele/robot1/heartbeat {"uptime_ms":12345,"status":"ok"}
```

### B. Publisher Window
Send a command to the Pico:
```bat
mosquitto_pub -h 192.168.1.xx -t cmd/robot1/ping -m ping
```

---

## 8. Expected Pico Serial Output

```
[MQTT<-] topic=cmd/robot1/ping payload=ping
[MQTT->] tele/robot1/reply pong
```

and in the subscriber window:
```
tele/robot1/reply pong
```

This confirms successful MQTT publish and subscribe operation.

---

## 9. Optional Reconnection Test

1. Temporarily close Mosquitto or disconnect Wi-Fi.
2. Observe the Pico output:

   ```
   [MQTT] ! Attempting reconnect...
   [MQTT] ✓ Connected to broker
   ```

   The Pico should reconnect automatically.
