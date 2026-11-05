# Handoff: UmutBot auf dem Pi in Betrieb nehmen

Stand 2026-11-05. Weitergabe von der Session auf dem Windows-PC an eine Claude-Session auf dem Raspberry Pi 5.

## Was bisher passiert ist

- **Neuer Motorcontroller:** Arduino Nano (CH340-Klon) statt ESP32. Er steuert einen L298N an und liest zwei Quadratur-Encoder.
  - Firmware: [`firmware/umutbot_nano/umutbot_nano.ino`](firmware/umutbot_nano/umutbot_nano.ino)
  - Pinbelegung: in der README
  - Der Name „pico“ in `pico_ros.py` und `pico.launch` ist ein Überbleibsel und meint den Nano.
- **Firmware ist geflasht und auf dem Prüfstand getestet:**
  - Beide Räder fahren vorwärts, beide Encoder zählen positiv. Dafür sind `MOTOR_A_REVERSED` und `ENC2_REVERSED` gesetzt.
  - Jedes Rad hat einen eigenen Regler (Vorsteuerung + PI) und hält den Sollwert auf etwa ±5–8 %.
  - Stopp nach 500 ms ohne Befehl.
- **Serielles Protokoll** (115200 Baud, eine Nachricht pro Zeile):
  - Pi → Nano: `cmd_vel:<linear>:<angular>`
  - Nano → Pi: `encoder1:<ticks>,encoder2:<ticks>`, 20 Hz
  - Testbefehle: `pwm:<links>:<rechts>` (festes PWM, Regler aus) und `debug:1` (Soll, Ist und PWM pro Rad)
- **Gemessene Grenze:** Mit beiden Motoren auf PWM 255 schaffen die Räder nur etwa 0,17 m/s, weil die Versorgung einbricht.
  - In der Firmware gilt deshalb `MAX_WHEEL_SPEED = 0.15`.
  - Der Planer ist darauf eingestellt (`max_vel_x: 0.12`, `acc_lim_x: 0.4`).
- **Zuletzt geändert, aber noch NICHT auf echter Hardware oder in ROS getestet:**
  - udev-Regel [`src/umut_bot/udev/99-umutbot.rules`](src/umut_bot/udev/99-umutbot.rules): feste Namen `/dev/umutbot_nano` und `/dev/rplidar`
  - Launch-Dateien: Standardwerte sind jetzt diese Namen (Argumente `nano_port`, `lidar_port`). Die Bridge startet neu, wenn die USB-Verbindung abbricht (`respawn`).
  - `python3-serial` steht jetzt im Dockerfile.
  - `encoder_odom.py` neu geschrieben: rechnet einmal pro Nano-Meldung statt starr mit 100 Hz, und `/odom` sowie TF kommen jetzt mit 20 Hz. Die Logik ist nur mit nachgebauten ROS-Modulen getestet (geradeaus, Drehen, Kreisbogen, Neustart des Nano), nicht mit echtem ROS.

## Was auf dem Pi zu tun ist

1. **Repo aktualisieren:** `git pull` und prüfen, dass diese Datei da ist.
2. **udev-Regel installieren:** Befehle stehen in der README unter „USB device names“.
   - Mit `lsusb` prüfen: Nano `1a86:7523`. Für das Lidar ist `10c4:ea60` nur angenommen, nicht überprüft.
   - Danach `ls -l /dev/umutbot_nano /dev/rplidar`.
3. **Docker-Image neu bauen**, damit `python3-serial` drin ist.
   - Achtung: `docker_container.sh` baut das Image nur, gestartet wird es woanders.
   - Den bestehenden `docker run`-Befehl um `--device=/dev/umutbot_nano --device=/dev/rplidar` ergänzen, oder um `-v /dev:/dev --privileged`.
4. **Nur die Bridge testen**, Robot dabei aufbocken:
   - `catkin_make && source devel/setup.bash`
   - `roslaunch umut_bot pico.launch`
   - `rostopic hz /encoder_feedback_1` sollte etwa 20 Hz zeigen.
   - Mit `teleop_twist_keyboard` fahren. `repeat_rate` setzen, sonst stoppt der Nano nach 0,5 s.
5. **Odometrie prüfen** mit `encoder_odom.py` aus `navigation.launch` oder einzeln gestartet:
   - `rostopic hz /odom` sollte etwa 20 Hz zeigen.
   - 1 m geradeaus fahren (Maßband): Steht in `/odom` `x ≈ 1.0`?
   - Einmal um 360° drehen: Kommt `theta` wieder bei etwa 0 an?
   - Bei Abweichungen `ticks_per_revolution` (403) bzw. `wheel_radius` (0.035) anpassen, sowohl in `navigation.launch` als auch in der Firmware (`TICKS_PER_REV`, `WHEEL_RADIUS`). Die 403 wurden nie von Hand nachgezählt.
6. **Gesamtsystem starten:** `roslaunch umut_bot navigation.launch`, dann in RViz prüfen, ob die Karte entsteht und move_base Ziele anfährt.

## Offene Punkte / Hinweise

- **Versorgung zu schwach:** Laufen beide Motoren, bricht die Spannung ein.
  - Bei einem Testlauf ist die USB-Verbindung beim Anlaufen eines Motors einmal abgebrochen.
  - Empfehlung: 100-nF-Kondensatoren direkt an die Motorklemmen, saubere gemeinsame Masse, stärkerer Akku.
  - Danach die Höchstgeschwindigkeit neu messen (`pwm:255:255` + `debug:1`) und `MAX_WHEEL_SPEED` sowie die Planer-Limits anheben.
- **Encoder-VCC (blau):** Sollte an 5V hängen. Eine Zeit lang hing es an 3V3. Prüfen, ob das umgesteckt wurde.
- **Firmware flashen vom Pi aus** (optional), es ist ein Klon mit altem Bootloader:
  `arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old -u -p /dev/umutbot_nano firmware/umutbot_nano`
  Vorher die Bridge stoppen, sonst ist der Port belegt.
- **Nicht an die GPIO-UART des Pi anschließen:** Der Nano hat 5-V-Pegel. USB ist die vorgesehene Verbindung.
- **Commits:** Der Nutzer will direkt auf `main` committen, ohne Feature-Branch. Gepusht wird nur auf ausdrückliche Anfrage.
