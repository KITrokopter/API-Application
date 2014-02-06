Header header
uint8 type # 0 = Camera, 1 = Quadcopter, 2 = Controller, 3 = Position
uint64 cameraId # Wenn type != 0 dann cameraId undefiniert
uint8[] initializeServiceName
---
uint32 ID # Wenn ID = -1 Dann Fehler
