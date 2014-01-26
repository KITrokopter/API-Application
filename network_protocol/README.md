# API

## Globaler Ping

*   Name: Ping
*   Typ: Topic
*   Sender: Alle außer API
*   Empfänger: API
*   Beschreibung: Kanal, in dem alle Anwendungen mitteilen, dass sie da sind.

### Daten

```
Header header
uint32 ID
```

## Systemstart und Systemshutdown

*   Name: System
*   Sender: API
*   Empfänger: Alle außer API
*   Beschreibung: Kanal, in dem die API das System den Startbefehl geben und beenden kann.

### Daten

```
Header header
uint8 command # 1 = Start, 2 = Ende
```

## Nachrichten

*   Name: Message
*   Typ: Topic
*   Sender: Alle außer API
*   Empfänger: API
*   Beschreibung: Kanal, über den Nachrichten in die API gesendet werden können.

### Daten

```
Header header
uint32 senderID
uint8 type # 1 = Message, 2 = Warning, 3 = Error
uint8[] message
```

## Anmeldeservice

*   Name: Announce
*   Sender: Alle außer API // Die API hat per default den Namen API
*   Empfänger: API
*   Beschreibung: Service an dem sich jede Anwendung anmeldet.

### Daten

```
Header header
uint8 type # 0 = Camera, 1 = Quadcopter, 2 = Controller, 3 = Position
uint64 cameraId # Wenn type != 0 dann cameraId undefiniert
uint8[] initializeServiceName
---
uint32 ID # Wenn ID = -1 Dann Fehler
```

# Steueranwendung

## Setze Formation

*   Name: SetFormation
*   Typ: Topic
*   Sender: API
*   Empfänger: Steuerungsanwendung
*   Beschreibung: Service mit dem die API die vom User gesetzte Formation weitergeben kann.

### Daten

```
Header header
float64[] xPositions
float64[] yPositions
float64[] zPositions
```

## Bewege Formation

*   Name: MoveFormation
*   Typ: Topic
*   Sender: API
*   Empfänger: Steuerungsanwendung
*   Beschreibung: Service mit dem die API die vom User gesetzte Formation weitergeben kann.

### Daten

```
Header header
float64 xMovement
float64 yMovement
float64 zMovement
```

## Starte Kalibrierung

*   Name: StartCalibration
*   Typ: Service
*   Sender: API
*   Empfänger: Steuerungsanwendung
*   Beschreibung: Service mit dem die API den Kalibrierungsprozess starten kann.

### Daten

```
Header header
---
```

## Mache Kalibrierungsbild

*   Name: TakeCalibrationPicture
*   Typ: Service
*   Sender: API
*   Empfänger: Steuerungsanwendung
*   Beschreibung: Service, der bewirkt, dass alle Kameras ein Bild machen, und wenn genug gute Bilder dabei sind, werden diese an die API gesendet.

### Daten

```
Header header
---
uint8[][] images
```

## Berechne Kalibrierung

*   Name: CalculateCalibration
*   Typ: Service
*   Sender: API
*   Empfänger: Steuerungsanwendung
*   Beschreibung: Service, der die Kalibrierung berechnet.

### Daten

```
Header header
---
float64[] cameraXPositions
float64[] cameraYPositions
float64[] cameraZPositions
float64[] cameraXOrientation
float64[] cameraYOrientation
float64[] cameraZOrientation
float64[] cameraIDs
```

## Aktuelle Positionen

*   Name: CurrentPositions
*   Typ: Topic
*   Sender: Steueranwendung
*   Empfänger: API
*   Beschreibung: Kanal um neu berechnete aktuelle Positionen(Position mit Orientierung) der Quadrokopter zu senden

Daten
```
Header header
uint32 ID
float32 xPosition
float32 yPosition
float32 zPosition
float32 xOrientation
float32 yOrientation
float32 zOrientation
```

## Gesamtkoordinatensystem

*   Name: GlobalCoordinateSystem
*   Typ: Topic
*   Sender: Steueranwendung
*   Empfänger: API
*   Beschreibung: Kanal um das Gesamtkoordinatensystem zu senden

## Bewegungsdaten

*   Name: Movement
*   Typ: Topic
*   Sender: Steueranwendung
*   Empfänger: Quadcopters
*   Beschreibung: Kanal um die Bewegungsdaten der Quadrokopter zu senden

Daten
```
Header header
uint32 id
uint16 thrust
float32 yaw
float32 pitch
float32 roll
```

# Kameraanwendung

## Bilder

*   Name: Picture
*   Typ: Topic
*   Sender: Kameraanwendung
*   Empfänger: API-Anwendung
*   Beschreibung: Kanal um Bilder zu senden

### Daten

```
Header header
uint32 ID
uint32 imageNumber # Bei Kalibrierung Nummer des Kalibrierungs-
# bilds, sonst inkrementiert pro Bild
uint64 timestamp # Wann das Bild gemacht wurde
uint8[640*480] image
```
## Positionsinformationen

*   Name: RawPosition
*   Typ: Topic
*   Sender: Kameraanwendung
*   Empfänger: Positionsmodul
*   Beschreibung: Kanal um die Positionsinformationen die aus dem Bild extrathiert wurden zu senden.

### Daten

```
Header header
uint32 ID
uint32 imageNumber
uint64 timestamp # Wann das Quellbild gemacht wurde
float64[] xPositions
float64[] yPositions
uint8[] quadcopterIds
```
## Bildsendungsaktivierung

*   Name: PictureSendingActivation
*   Typ: Topic
*   Sender: API
*   Empfänger: Kameraanwendung
*   Beschreibung: Kanal um das senden der Bilder zu aktivieren und deaktivieren.

### Daten

```
Header header
uint32 ID
boolean active
```

## Kalibrierungsauftrag

*   Name: CalibrateCamera
*   Typ: Topic
*   Sender: API
*   Empfänger: Kameraanwendung
*   Beschreibung: Kanal um das Kalibrieren eine Kamera zu starten. Kalibrierungsbilder werden über das Bildertopic gesendet. Bei Erfolg werden die Daten über das Einzelkamera-Kalibrierungsdatentopic gesendet.

### Daten

```
Header header
uint32 ID
uint32 imageAmount
uint32 imageDelay
```
## Einzelkamera-Kalibrierungsdaten

*   Name: CameraCalibrationData
*   Typ: Topic
*   Sender: API, Kameraanwendung
*   Empfänger: API, Kameraanwendung
*   Beschreibung: Kanal zum Senden von Kalibrierungsdaten

### Daten

```
Header header
uint64 cameraHardwareId
boolean createdByCamera
float64[3*3] intrinsics
float64[4] distortion
```

## Initialize Camera

*   Name: InitializeCameraService[ID]
*   Typ: Service
*   Sender: API
*   Empfänger: Kameraanwendung
*   Beschreibung: Kanal zum Initialisieren der Kamera

### Daten

```
Header header
uint32[] hsvColorRanges # erster Range: [0-1], zweiter Range: [2-3] …
uint32[] quadCopterIds # erster Copter: [0], zweiter Copter: [1] …
---
uint8 error # 0 bei ok, != 0 sonst
```

# Quadrokopteranwendung

## Quadrokopter Statusinformationen

*   Name: QuadcopterStatus
*   Typ: Topic
*   Sender: Quadrokopteranwendung
*   Empfänger: API-Anwendung
*   Beschreibung: Topic, an welches die Quadrokopteranwendung periodisch neue Statusinformationen der Quadrokopter schickt.

### Daten

```
Header header
uint32 ID
float32 battery_status
float32 link_quality
float32 altimeter
float32 mag_x
float32 mag_y
float32 mag_z
float32 gyro_x
float32 gyro_y
float32 gyro_z
float32 acc_x
float32 acc_y
float32 acc_z
uint16 motor_m1
uint16 motor_m2
uint16 motor_m3
uint16 motor_m4
float32 roll
float32 pitch
float32 yaw
```
