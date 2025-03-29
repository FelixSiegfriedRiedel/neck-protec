# NeckProtec

## Arduino

- aktuelles Modell (Edge Impulse Modell) in Arduino-IDE als .zip-Bibliothek installieren
- Bibliotheken installieren: Arduino_LSM9DS1, ArduinoBLE

## WebApp

- https://neckprotec-209b7.web.app/
- Desktop: Bluetooth und Benachichtigungen zulassen
- Mobil: Android verwenden (nicht getestet), alternativ Bluefy-Browser auf IOS Geräten

### Hosting mit Firebase:

- Navigation in Projekt-Ordner
`cd webapp/neckprotec`
- Packages installieren
`npm install`
- Firebase konfigurieren
`firebase innit`
- Firebase Config in src/main.js ergänzen
```
const firebaseConfig = {
  };
```
- Deployment
```
npm run build
firebase deploy
```


