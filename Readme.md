# ESP32 HySeq OM60-HWT906

Sistema di misura professionale basato su ESP32-S3 con display touch.

## 🔧 Sensori
- **OM60** (Baumer): Sensore laser 200-1000mm, uscita analogica 0-10V
- **HWT906** (WitMotion): IMU 9-assi con output Pitch/Yaw

## 📋 Caratteristiche
- Display touch 2.8" con menu ad albero
- Precisione distanza: ±0.05mm
- Update rate: 10Hz
- Calibrazione multi-punto
- Service menu protetto (PIN: 235711)

## 🔌 Hardware
- ESP32-S3 DevKit
- ADC 12-bit con oversampling per OM60
- I2C + UART per HWT906
- Condizionamento segnale 10V→3.3V

## 📁 Struttura
```
├── ESP32_HySeq_OM60_HWT906.ino   # Main program
├── src/
│   ├── om60_handler.*             # Driver sensore distanza
│   ├── hwt906_handler.*           # Driver IMU
│   ├── ui_config.h                # Configurazione UI
│   ├── display_utils.*            # Utility display
│   ├── touch_handler.*            # Gestione touch
│   ├── menu_state.*               # Macchina stati menu
│   ├── leaf_actions.*             # Azioni finali menu
│   └── service_menu.*             # Menu service
├── docs/
│   ├── HARDWARE_SETUP.md          # Schema connessioni
│   ├── CALIBRATION.md             # Procedura calibrazione
│   └── PROTOCOL.md                # Protocolli comunicazione
└── hardware/
    ├── schematic.pdf              # Schema elettrico
    └── pcb/                       # File PCB (se disponibili)
```

## 🚀 Quick Start

### Prerequisiti
- Arduino IDE 2.x
- ESP32 board package (3.x)
- Librerie richieste:
  - Arduino_GFX_Library
  - CSE_CST328 (touch controller)
  - Wire (inclusa)
  - EEPROM (inclusa)

### Installazione
1. Clona questo repository:
   ```bash
   git clone https://github.com/[tuo-username]/ESP32-HySeq-OM60-HWT906.git
   ```

2. Apri `ESP32_HySeq_OM60_HWT906.ino` in Arduino IDE

3. Seleziona la board:
   - Board: "ESP32S3 Dev Module"
   - Flash Size: "4MB"
   - Partition Scheme: "Default 4MB with spiffs"

4. Carica lo sketch

## 📐 Connessioni Hardware

### Display Touch 2.8"
| ESP32-S3 | Display |
|----------|---------|
| GPIO 42  | CS      |
| GPIO 39  | RST     |
| GPIO 41  | DC      |
| GPIO 40  | SCK     |
| GPIO 45  | MOSI    |
| GPIO 5   | BL      |
| GPIO 1   | TP_SDA  |
| GPIO 3   | TP_SCL  |

### HWT906 IMU
| ESP32-S3 | HWT906  |
|----------|---------|
| GPIO 11  | SDA     |
| GPIO 10  | SCL     |
| GPIO 44  | RX→TX   |
| GPIO 43  | TX→RX   |
| 3.3V     | VCC     |
| GND      | GND     |

### OM60 Laser
| OM60     | Circuito    | ESP32-S3 |
|----------|-------------|----------|
| OUT 0-10V| R1 27kΩ     | GPIO 15  |
|          | R2 10kΩ→GND |          |
| GND      | GND         | GND      |
| +24V     | Alimentatore|          |

## 🎮 Utilizzo

### Menu Principale
- **PITCH,YAW,DIST**: Visualizzazione live dei valori
- **CALIB. IMU**: Calibrazione zero IMU
- **SERVICE MENU**: Menu tecnico (PIN: 235711)
  - Settings: Parametri sistema
  - Calibrazione OM60
  - Factory Reset

### Calibrazione OM60
1. Entra nel Service Menu
2. Seleziona "Calibrate OM60"
3. Posiziona target a distanze note:
   - 200mm
   - 400mm  
   - 600mm
   - 800mm
   - 1000mm
4. Tocca per confermare ogni punto
5. La calibrazione viene salvata in EEPROM

### Calibrazione HWT906
1. Posiziona il sensore su superficie piana
2. Menu → "CALIB. IMU"
3. Non muovere per 5 secondi
4. Gli angoli saranno azzerati

## 📊 Specifiche Tecniche

### Precisione
- **Distanza**: ±0.05mm (dopo calibrazione)
- **Pitch**: ±0.1°
- **Yaw**: ±0.5° (drift < 0.1°/5min)

### Performance
- Update rate: 10Hz
- Latenza display: <100ms
- Boot time: <3s

### Condizioni Operative
- Temperatura: 0-50°C
- Alimentazione: 5V USB o batteria LiPo
- Consumo: ~200mA @ 5V

## 🔧 Troubleshooting

### OM60 legge valori errati
- Verificare alimentazione 24V
- Controllare partitore resistivo
- Eseguire calibrazione multi-punto

### HWT906 non risponde
- Verificare connessioni I2C e UART
- Controllare baudrate (9600)
- Reset hardware: scollegare alimentazione

### Touch non funziona
- Verificare connessioni I2C su Wire1
- Pulire schermo
- Ricalibrare touch se necessario

## 📝 Note Sviluppo

### Compilazione
- IDE: Arduino IDE 2.x o PlatformIO
- Core: ESP32 Arduino Core 3.x
- Warnings: Default

### Modifiche Codice
- UI: Modificare solo `ui_config.h`
- Sensori: Modificare `om60_handler.*` o `hwt906_handler.*`
- Menu: Aggiornare `menu_state.*` e `leaf_actions.*`

## 🤝 Contributi
Contributi benvenuti! Per favore:
1. Fork il repository
2. Crea un feature branch
3. Commit le modifiche
4. Push al branch
5. Apri una Pull Request

## 📜 Licenza
Questo progetto è rilasciato sotto licenza MIT. Vedi file [LICENSE](LICENSE) per dettagli.

## 🙏 Credits
- Progetto derivato da [ESP32-TouchMenu-HySeq](https://github.com/[original-author]/ESP32-TouchMenu-HySeq)
- Display library: [Arduino_GFX](https://github.com/moononournation/Arduino_GFX)
- Touch driver: CSE_CST328

## 📞 Supporto
- Issues: [GitHub Issues](https://github.com/[tuo-username]/ESP32-HySeq-OM60-HWT906/issues)
- Email: [tua-email]
- Documentation: [Wiki](https://github.com/[tuo-username]/ESP32-HySeq-OM60-HWT906/wiki)

---
*Ultimo aggiornamento: Agosto 2025*
