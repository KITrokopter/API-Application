# API

## <a name="h.hyhi525h2d3v"></a> Globaler Ping

*   Name: Ping
*   Typ: Topic
*   Sender: Alle au&szlig;er API
*   Empf&auml;nger: API
*   Beschreibung: Kanal, in dem alle Anwendungen mitteilen, dass sie da sind.

&nbsp;

### <a name="h.13lbj1l89yia"></a> Daten

```
Header header
uint32 ID
```

## <a name="h.2b4ax7tvfjj5"></a> Systemstart und Systemshutdown

*   Name: System
*   Sender: API
*   Empf&auml;nger: Alle au&szlig;er API
*   Beschreibung: Kanal, in dem die API das System den Startbefehl geben und beenden kann.

### <a name="h.1sg5zzdaw0os"></a> Daten

```
Header header
uint8 command # 1 = Start, 2 = Ende
```

## <a name="h.9fd375e0v4ln"></a> Nachrichten

*   Name: Message
*   Typ: Topic
*   Sender: Alle au&szlig;er API
*   Empf&auml;nger: API
*   Beschreibung: Kanal, &uuml;ber den Nachrichten in die API gesendet werden k&ouml;nnen.

### <a name="h.3qphc3vim31z"></a> Daten

```
Header header
uint32 senderID
uint8 type # 1 = Message, 2 = Warning, 3 = Error
uint8[] message
```

## <a name="h.8h8sncihn8lz"></a>Anmeldeservice

*   Name: Announce
*   Sender: Alle au&szlig;er API // Die API hat per default den Namen API
*   Empf&auml;nger: API
*   Beschreibung: Service an dem sich jede Anwendung anmeldet.

### <a name="h.qzez4ztcwyaw"></a> Daten

```
Header header
uint8 type # 0 = Camera, 1 = Quadcopter, 2 = Controller, 3 = Position
uint64 cameraId # Wenn type != 0 dann cameraId undefiniert
uint8[] initializeServiceName
---
uint32 ID # Wenn ID = -1 Dann Fehler
```

# <a name="h.6l52x2dch50o"></a> Steueranwendung

## <a name="h.rwwbp9d5s3m7"></a> Setze Formation

*   Name: SetFormation
*   Typ: Topic
*   Sender: API
*   Empf&auml;nger: Steuerungsanwendung
*   Beschreibung: Service mit dem die API die vom User gesetzte Formation weitergeben kann.

### <a name="h.g43yxuia2f4j"></a> Daten

```
Header header
float64[] xPositions
float64[] yPositions
float64[] zPositions
```

## <a name="h.me7p3tpzl1k1"></a> Bewege Formation

*   Name: MoveFormation
*   Typ: Topic
*   Sender: API
*   Empf&auml;nger: Steuerungsanwendung
*   Beschreibung: Service mit dem die API die vom User gesetzte Formation weitergeben kann.

### <a name="h.dnzxvfeaark8"></a> Daten

```
Header header
float64 xMovement
float64 yMovement
float64 zMovement
```

## <a name="h.wjbekfg8a8qq"></a>Starte Kalibrierung

*   Name: StartCalibration
*   Typ: Service
*   Sender: API
*   Empf&auml;nger: Steuerungsanwendung
*   Beschreibung: Service mit dem die API den Kalibrierungsprozess starten kann.

### <a name="h.mt281j8brno3"></a> Daten

```
Header header
---
```

## <a name="h.e6vbf1m81k0"></a> Mache Kalibrierungsbild

*   Name: TakeCalibrationPicture
*   Typ: Service
*   Sender: API
*   Empf&auml;nger: Steuerungsanwendung
*   Beschreibung: Service, der bewirkt, dass alle Kameras ein Bild machen, und wenn genug gute Bilder dabei sind, werden diese an die API gesendet.

### <a name="h.fp7lvc54mldb"></a> Daten

```
Header header
---
uint8[][] images
```

## <a name="h.w8uo4dblor2w"></a> Berechne Kalibrierung

*   Name: CalculateCalibration
*   Typ: Service
*   Sender: API
*   Empf&auml;nger: Steuerungsanwendung
*   Beschreibung: Service, der die Kalibrierung berechnet.

### <a name="h.mjmfqkkatygi"></a> Daten

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

## <a name="h.d4aayalu18xg"></a>Aktuelle Positionen

*   Name: CurrentPositions
*   Typ: Topic
*   Sender: Steueranwendung
*   Empf&auml;nger: API
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
&nbsp;

## <a name="h.iutufwt9dh5"></a> Gesamtkoordinatensystem

*   Name: GlobalCoordinateSystem
*   Typ: Topic
*   Sender: Steueranwendung
*   Empf&auml;nger: API
*   Beschreibung: Kanal um das Gesamtkoordinatensystem zu senden

## <a name="h.8btndew697oo"></a> Bewegungsdaten

*   Name: Movement
*   Typ: Topic
*   Sender: Steueranwendung
*   Empf&auml;nger: Quadcopters
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

# <a name="h.3g5qczgbcr5u"></a>Kameraanwendung

## <a name="h.bog1cvyjhmge"></a> Bilder

*   Name: Picture
*   Typ: Topic
*   Sender: Kameraanwendung
*   Empf&auml;nger: API-Anwendung
*   Beschreibung: Kanal um Bilder zu senden

### <a name="h.ugy20nwhs0ni"></a> Daten

```
Header header
uint32 ID
uint32 imageNumber # Bei Kalibrierung Nummer des Kalibrierungs-
# bilds, sonst inkrementiert pro Bild
uint64 timestamp # Wann das Bild gemacht wurde
uint8[640*480] image
```
## <a name="h.66trsnoez8a"></a> Positionsinformationen

*   Name: RawPosition
*   Typ: Topic
*   Sender: Kameraanwendung
*   Empf&auml;nger: Positionsmodul
*   Beschreibung: Kanal um die Positionsinformationen die aus dem Bild extrathiert wurden zu senden.

### <a name="h.b3cb9d1qpu4s"></a> Daten

```
Header header
uint32 ID
uint32 imageNumber
uint64 timestamp # Wann das Quellbild gemacht wurde
float64[] xPositions
float64[] yPositions
uint8[] quadcopterIds
```
## <a name="h.cpg6abgbppfz"></a> Bildsendungsaktivierung

*   Name: PictureSendingActivation
*   Typ: Topic
*   Sender: API
*   Empf&auml;nger: Kameraanwendung
*   Beschreibung: Kanal um das senden der Bilder zu aktivieren und deaktivieren.

### <a name="h.7ek6luufm7f3"></a> Daten

```
Header header
uint32 ID
boolean active
```

## <a name="h.fjt9uhpmyxgk"></a> Kalibrierungsauftrag

*   Name: CalibrateCamera
*   Typ: Topic
*   Sender: API
*   Empf&auml;nger: Kameraanwendung
*   Beschreibung: Kanal um das Kalibrieren eine Kamera zu starten. Kalibrierungsbilder werden &uuml;ber das Bildertopic gesendet. Bei Erfolg werden die Daten &uuml;ber das Einzelkamera-Kalibrierungsdatentopic gesendet.

### <a name="h.n6v87tvkymq3"></a> Daten

```
Header header
uint32 ID
uint32 imageAmount
uint32 imageDelay
```
## <a name="h.uf44kj7aofhv"></a> Einzelkamera-Kalibrierungsdaten

*   Name: CameraCalibrationData
*   Typ: Topic
*   Sender: API, Kameraanwendung
*   Empf&auml;nger: API, Kameraanwendung
*   Beschreibung: Kanal zum Senden von Kalibrierungsdaten

### <a name="h.dih1l81jk2iy"></a> Daten

```
Header header
uint64 cameraHardwareId
boolean createdByCamera
float64[3*3] intrinsics
float64[4] distortion
```

## <a name="h.xq1mzhvgcen6"></a> Initialize Camera

*   Name: InitializeCameraService[ID]
*   Typ: Service
*   Sender: API
*   Empf&auml;nger: Kameraanwendung
*   Beschreibung: Kanal zum Initialisieren der Kamera

### <a name="h.5bf5dj3bzjkn"></a> Daten

```
Header header
uint32[] hsvColorRanges # erster Range: [0-1], zweiter Range: [2-3] &hellip;
uint32[] quadCopterIds # erster Copter: [0], zweiter Copter: [1] &hellip;
---
uint8 error # 0 bei ok, != 0 sonst
```

# <a name="h.ehjh1xexyrzh"></a>Quadrokopteranwendung

## <a name="h.bsrsypit6amc"></a> Quadrokopter Statusinformationen

*   Name: QuadcopterStatus
*   Typ: Topic
*   Sender: Quadrokopteranwendung
*   Empf&auml;nger: API-Anwendung
*   Beschreibung: Topic, an welches die Quadrokopteranwendung periodisch neue Statusinformationen der Quadrokopter schickt.

### <a name="h.oyvcadfiw4df"></a> Daten

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
