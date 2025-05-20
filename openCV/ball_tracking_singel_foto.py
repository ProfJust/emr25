# ball_tracking_image.py
# ------------------------------------------
# Anpassung für Einzelbildverarbeitung
# ------------------------------------------
import argparse
import cv2

# Parser für Bildpfad
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Pfad zum Eingangsbild")
args = vars(ap.parse_args())

# Farbbereich für den Ball (anpassen!)
greenLower = (0, 0, 51)
greenUpper = (255, 255, 255)

# Bild laden und verarbeiten
image = cv2.imread(args["image"])
if image is None:
    print("Fehler: Bild konnte nicht geladen werden")
    exit()

# Bildverarbeitungsschritte
image = cv2.resize(image, (600, int(image.shape[0] * 600 / image.shape[1])))
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(hsv, greenLower, greenUpper)
mask = cv2.erode(mask, None, iterations=2)
mask = cv2.dilate(mask, None, iterations=2)

cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

if len(cnts) > 0:
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    
    if radius > 10:
        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        print(f"Objekt erkannt bei Position (X: {x:.1f}, Y: {y:.1f}) mit Radius {radius:.1f}")

# Ergebnisse anzeigen
cv2.imshow("Originalbild", image)
cv2.imshow("Maske", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
