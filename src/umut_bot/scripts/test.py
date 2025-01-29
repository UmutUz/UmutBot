import cv2

# Lade das vortrainierte Haarcascades-Modell von OpenCV
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# Starte die Kamera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Konvertiere das Bild in Graustufen (besser für die Erkennung)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Erkenne Gesichter
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    # Zeichne Rechtecke um die erkannten Gesichter
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)

    # Zeige das Bild
    cv2.imshow("Gesichtserkennung", frame)

    # Beende mit 'q'
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
