# Handoff: UmutBot auf dem Pi in Betrieb nehmen

Stand 2026-11-05. Weitergabe von der Session auf dem Windows-PC an eine Claude-Session auf dem Raspberry Pi 5.

## Was bisher passiert ist

- **Neuer Motorcontroller:** Arduino Nano (CH340-Klon) statt ESP32. Er steuert einen L298N an und liest zwei Quadratur-Encoder.
  - Firmware: [`firmware/umutbot_nano/umutbot_nano.ino`](firmware/umutbot_nano/umutbot_nano.ino)
  - Pinbelegung: in der README
  - Der Name „pico“ in `pico_ros.py` und `pico.launch` ist ein Überbleibsel und meint den Nano.
- **Firmware ist geflasht und auf dem Prüfstand getestet:**
  - Beide Räder fahren vorwärts (Richtung Stützrad, +x in der URDF), beide Encoder zählen dabei positiv. Dafür sind `MOTOR_B_REVERSED` und `ENC1_REVERSED` gesetzt.
  - Die erste Einstellung vom Prüfstand war genau verkehrt herum: Am Boden fuhr und drehte der Robot falsch, und die Karte verschmierte. Am 2026-11-06 wurden deshalb alle vier Werte umgedreht.
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
3. **Docker-Image neu bauen:** `cd src/dockerfiles && ./docker_container.sh` (Image-Name `noetic`). Danach ist `python3-serial` fest im Image.
   - Das Dockerfile stellt apt auf das letzte Noetic-Snapshot-Repo um. Grund: Noetic ist seit Mai 2025 end-of-life, und der alte Schlüssel von packages.ros.org ist abgelaufen (`apt update` meldet `EXPKEYSIG F42ED6FBAB17C654`).
   - Keine Pi-GPIO-Bibliotheken mehr im Image (lgpio/rpi-lgpio): Der Nano steuert die Motoren über USB, und lgpio lässt sich unter Ubuntu 20.04 nicht bauen. Ein lokaler `lg/`-Ordner ist nicht nötig.
   - `docker_container.sh` baut das Image nur. Starten z. B. mit `docker run -it --net=host --privileged -v /dev:/dev -v ~/UmutBot:/UmutBot noetic bash`. `--privileged -v /dev:/dev` sorgt dafür, dass der Nano auch nach einem USB-Abbruch erreichbar bleibt.
4. **Nur die Bridge testen**, Robot dabei aufbocken:
   - `catkin_make && source devel/setup.bash`
   - `roslaunch umut_bot pico.launch`
   - `rostopic hz /encoder_feedback_1` sollte etwa 20 Hz zeigen.
   - Mit `teleop_twist_keyboard` fahren. `repeat_rate` setzen, sonst stoppt der Nano nach 0,5 s.
5. **Odometrie kalibrieren** mit `rosrun umut_bot calibrate_odom.py` (Bridge muss laufen):
   - `rostopic hz /odom` sollte etwa 20 Hz zeigen.
   - Test 1: Robot etwa 2 m von Hand am Maßband entlang schieben. Von Hand, weil er unter eigenem Antrieb leicht nach rechts zieht. Ergibt den Radius jedes Rads.
   - Test 2: Mit teleop (`j`/`l`) einige volle Umdrehungen auf der Stelle drehen, bis er wieder an einer Markierung ausgerichtet ist. Ergibt den effektiven Radabstand.
   - Die ausgegebenen Werte in `navigation.launch` (`left_wheel_radius`, `right_wheel_radius`, `wheel_base`) und in die Firmware (`WHEEL_RADIUS_LEFT`, `WHEEL_RADIUS_RIGHT`, `WHEEL_BASE`) eintragen, dann den Nano neu flashen.
   - Anlass: Der Scan wanderte in der Karte mit dem Robot mit. Ein erster Drehtest zeigte etwa 279° Odometrie für eine echte halbe Drehung. Die 403 Ticks pro Umdrehung wurden nie von Hand nachgezählt.
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
