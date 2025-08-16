✅ **TABELLA COMPLETA GENERATA!**

Ho creato una tabella comprensiva di **tutti i comandi I2C disponibili** per il HWT906, organizzata per categorie:

## 📋 **STRUTTURA DELLA TABELLA**

### **🔧 Sezioni Principali:**
1. **Comandi di Sistema** - UNLOCK/SAVE & LOCK
2. **Configurazione Principale** - RSW, RRATE, BAUD  
3. **Range Sensori** - Accelerometro, Giroscopio, Magnetometro
4. **Filtri e Modalità** - Bandwidth, Power Mode, Orientation
5. **Offset Calibrazione** - Tutti i registri 0x05-0x0D
6. **Lettura Dati** - Raw data registers 0x34-0x3F
7. **Registri Avanzati** - Funzioni speciali
8. **Dati Estesi** - Temperatura, velocità, posizione, quaternioni
9. **Identificazione** - Numero seriale

### **🎯 Indicatori di Utilizzo:**
- ✅ **UTILIZZATO** - Configurato e attivo nel progetto
- ✅ **CONFIGURATO** - Impostato durante init
- ⚠️ **IMPLEMENTAZIONE MANCANTE** - Dovrebbe essere implementato
- ❌ **Non utilizzato** - Disponibile ma non usato

### **📊 Informazioni Complete:**
- **Codice registro** (hex)
- **Nome** descrittivo
- **Valori possibili** e range
- **Formule di conversione** per i dati
- **Note specifiche** per il nostro progetto

La tabella mostra chiaramente che utilizziamo **solo una frazione** dei comandi disponibili, con particolare focus sui **registri di configurazione base** e **ignorando completamente** funzioni avanzate come quaternioni, velocità integrata, temperatura, ecc.

Questo ti permette di vedere **esattamente quali funzionalità potresti aggiungere** in futuro per espandere le capacità del sistema!

<div style="page-break-before: always;"></div>

# 📋 HWT906 - TABELLA COMPLETA COMANDI I2C

## 🔧 COMANDI DI SISTEMA E CONTROLLO

| **CODICE** | **NOME** | **DESCRIZIONE** | **VALORI** | **R/W** | **NOTE** |
|------------|----------|-----------------|------------|---------|----------|
| `0xFF 0xAA 0x69 0x88 0xB5` | **UNLOCK** | Sblocca registri per scrittura | Sequenza fissa | W | ⚠️ **OBBLIGATORIO** prima di qualsiasi config |
| `0xFF 0xAA 0x00 0x00 0x00` | **SAVE & LOCK** | Salva configurazione e blocca registri | Sequenza fissa | W | ⚠️ **OBBLIGATORIO** dopo config, delay 150ms |

## ⚙️ REGISTRI DI CONFIGURAZIONE PRINCIPALE

| **CODICE** | **NOME** | **DESCRIZIONE** | **VALORI** | **R/W** | **NOTE** |
|------------|----------|-----------------|------------|---------|----------|
| `0x02` | **RSW** | Output Content Control | `0x001E` (default) | R/W | ✅ **UTILIZZATO** - Bit mask stream UART |
|  |  | - Bit 0: TIME (0x50) | `0/1` | | ON=1, OFF=0 |
|  |  | - Bit 1: ACC (0x51) | `0/1` | | ✅ **Accelerazione** |
|  |  | - Bit 2: GYRO (0x52) | `0/1` | | ✅ **Giroscopio** |
|  |  | - Bit 3: ANGLE (0x53) | `0/1` | | ✅ **Angoli Eulero** |
|  |  | - Bit 4: MAG (0x54) | `0/1` | | ✅ **Magnetometro** |
|  |  | - Bit 5: PORT (0x55) | `0/1` | | ❌ Non utilizzato |
|  |  | - Bit 6: PRESS (0x56) | `0/1` | | ❌ Non utilizzato |
|  |  | - Bit 7: GPS (0x57) | `0/1` | | ❌ Non utilizzato |
|  |  | - Bit 8: VELOCITY (0x58) | `0/1` | | ❌ Non utilizzato |
|  |  | - Bit 9: QUATER (0x59) | `0/1` | | ❌ Non utilizzato (Quaternioni) |
|  |  | - Bit 10: GSA (0x5A) | `0/1` | | ❌ Non utilizzato |
| `0x03` | **RRATE** | Sample Rate (Hz) | `0x0001-0x00FF` | R/W | ✅ **UTILIZZATO** - Nostro: 10Hz (0x000A) |
| `0x04` | **BAUD** | UART Baud Rate | Vedi tabella sotto | R/W | ✅ **UTILIZZATO** - Nostro: 9600 (0x0005) |

### VALORI BAUD RATE (Registro 0x04)
| **VALORE** | **BAUD RATE** | **NOTE** |
|------------|---------------|----------|
| `0x0000` | 4800 | ❌ Non utilizzato |
| `0x0001` | 9600 | ✅ **UTILIZZATO** nel progetto |
| `0x0002` | 19200 | ❌ Non utilizzato |
| `0x0003` | 38400 | ❌ Non utilizzato |
| `0x0004` | 57600 | ❌ Non utilizzato |
| `0x0005` | 115200 | ❌ Non utilizzato |
| `0x0006` | 230400 | ❌ Non utilizzato |
| `0x0007` | 460800 | ❌ Non utilizzato |
| `0x0008` | 921600 | ❌ Era default, cambiato a 9600 |

## 🎛️ REGISTRI RANGE E SENSIBILITÀ SENSORI

| **CODICE** | **NOME** | **DESCRIZIONE** | **VALORI** | **R/W** | **NOTE** |
|------------|----------|-----------------|------------|---------|----------|
| `0x21` | **ACC_RANGE** | Range Accelerometro | | R/W | ✅ **UTILIZZATO** - Nostro: ±2g |
|  |  | - ±2g (max risoluzione) | `0x0000` | | ✅ **CONFIGURATO** |
|  |  | - ±4g | `0x0001` | | ❌ Non utilizzato |
|  |  | - ±8g | `0x0002` | | ❌ Non utilizzato |
|  |  | - ±16g | `0x0003` | | ❌ Non utilizzato |
| `0x22` | **GYRO_RANGE** | Range Giroscopio | | R/W | ✅ **UTILIZZATO** - Nostro: ±250°/s |
|  |  | - ±250°/s (max precisione) | `0x0000` | | ✅ **CONFIGURATO** |
|  |  | - ±500°/s | `0x0001` | | ❌ Non utilizzato |
|  |  | - ±1000°/s | `0x0002` | | ❌ Non utilizzato |
|  |  | - ±2000°/s | `0x0003` | | ❌ Non utilizzato |
| `0x23` | **MAG_RANGE** | Range Magnetometro | | R/W | ✅ **UTILIZZATO** - Nostro: ±4 gauss |
|  |  | - ±4 gauss (max sensibilità) | `0x0000` | | ✅ **CONFIGURATO** |
|  |  | - ±8 gauss | `0x0001` | | ❌ Non utilizzato |
|  |  | - ±12 gauss | `0x0002` | | ❌ Non utilizzato |
|  |  | - ±16 gauss | `0x0003` | | ❌ Non utilizzato |

## 🔧 REGISTRI FILTRI E MODALITÀ

| **CODICE** | **NOME** | **DESCRIZIONE** | **VALORI** | **R/W** | **NOTE** |
|------------|----------|-----------------|------------|---------|----------|
| `0x24` | **BANDWIDTH** | Filtro Passa-Basso | `0x0000-0x0007` | R/W | ✅ **UTILIZZATO** - Nostro: 0x0001 |
|  |  | - Filtro più aggressivo | `0x0000` | | ❌ Troppo lento per noi |
|  |  | - Filtro medio | `0x0001` | | ✅ **CONFIGURATO** |
|  |  | - Filtro leggero | `0x0002-0x0007` | | ❌ Non utilizzato |
| `0x25` | **PWR_MODE** | Modalità Alimentazione | | R/W | ✅ **UTILIZZATO** - Nostro: Normal |
|  |  | - Normal (performance max) | `0x0000` | | ✅ **CONFIGURATO** |
|  |  | - Low Power | `0x0001` | | ❌ Non utilizzato |
|  |  | - Suspend | `0x0002` | | ❌ Non utilizzato |
| `0x26` | **ORIENT_MODE** | Modalità Orientamento | | R/W | ✅ **UTILIZZATO** - Nostro: 9-axis |
|  |  | - 6-axis (solo acc+gyro) | `0x0000` | | ❌ Non utilizzato |
|  |  | - 9-axis (con magnetometro) | `0x0001` | | ✅ **CONFIGURATO** |

## 📐 REGISTRI OFFSET CALIBRAZIONE

| **CODICE** | **NOME** | **DESCRIZIONE** | **RANGE** | **R/W** | **NOTE** |
|------------|----------|-----------------|-----------|---------|----------|
| `0x05` | **AXOFFSET** | Offset Accelerometro X | `±32768` | R/W | ⚠️ **IMPLEMENTAZIONE MANCANTE** |
| `0x06` | **AYOFFSET** | Offset Accelerometro Y | `±32768` | R/W | ⚠️ **IMPLEMENTAZIONE MANCANTE** |
| `0x07` | **AZOFFSET** | Offset Accelerometro Z | `±32768` | R/W | ⚠️ **IMPLEMENTAZIONE MANCANTE** |
| `0x08` | **GXOFFSET** | Offset Giroscopio X | `±32768` | R/W | ✅ **UTILIZZATO** software |
| `0x09` | **GYOFFSET** | Offset Giroscopio Y | `±32768` | R/W | ✅ **UTILIZZATO** software |
| `0x0A` | **GZOFFSET** | Offset Giroscopio Z | `±32768` | R/W | ✅ **UTILIZZATO** software |
| `0x0B` | **HXOFFSET** | Offset Magnetometro X | `±32768` | R/W | ⚠️ **IMPLEMENTAZIONE MANCANTE** |
| `0x0C` | **HYOFFSET** | Offset Magnetometro Y | `±32768` | R/W | ⚠️ **IMPLEMENTAZIONE MANCANTE** |
| `0x0D` | **HZOFFSET** | Offset Magnetometro Z | `±32768` | R/W | ⚠️ **IMPLEMENTAZIONE MANCANTE** |

**Unità Offset**: Acc = /10000 g, Gyro = /10000 °/s, Mag = raw units

## 📊 REGISTRI DI LETTURA DATI (Read-Only)

| **CODICE** | **NOME** | **DESCRIZIONE** | **FORMULA** | **R/W** | **NOTE** |
|------------|----------|-----------------|-------------|---------|----------|
| `0x34` | **AX** | Accelerazione X | `AX/32768*16g` | R | ✅ **UTILIZZATO** via UART |
| `0x35` | **AY** | Accelerazione Y | `AY/32768*16g` | R | ✅ **UTILIZZATO** via UART |
| `0x36` | **AZ** | Accelerazione Z | `AZ/32768*16g` | R | ✅ **UTILIZZATO** via UART |
| `0x37` | **GX** | Velocità angolare X | `GX/32768*250°/s` | R | ✅ **UTILIZZATO** via UART |
| `0x38` | **GY** | Velocità angolare Y | `GY/32768*250°/s` | R | ✅ **UTILIZZATO** via UART |
| `0x39` | **GZ** | Velocità angolare Z | `GZ/32768*250°/s` | R | ✅ **UTILIZZATO** via UART |
| `0x3A` | **HX** | Campo magnetico X | Raw value | R | ✅ **UTILIZZATO** via UART |
| `0x3B` | **HY** | Campo magnetico Y | Raw value | R | ✅ **UTILIZZATO** via UART |
| `0x3C` | **HZ** | Campo magnetico Z | Raw value | R | ✅ **UTILIZZATO** via UART |
| `0x3D` | **Roll** | Angolo Roll | `Roll/32768*180°` | R | ✅ **UTILIZZATO** via UART |
| `0x3E` | **Pitch** | Angolo Pitch | `Pitch/32768*180°` | R | ✅ **UTILIZZATO** via UART |
| `0x3F` | **Yaw** | Angolo Yaw | `Yaw/32768*180°` | R | ✅ **UTILIZZATO** via UART |

## 🔄 REGISTRI AVANZATI

| **CODICE** | **NOME** | **DESCRIZIONE** | **VALORI** | **R/W** | **NOTE** |
|------------|----------|-----------------|------------|---------|----------|
| `0x27` | **READADDR** | Lettura registro specifico | `0x34-0x3F` | R/W | ❌ Non utilizzato - usiamo UART stream |
| `0x50` | **I2C_ADDR** | Cambio indirizzo I2C | `0x50-0x57` | R/W | ❌ Non utilizzato - fisso a 0x50 |

## 📡 REGISTRI DATI ESTESI (Non utilizzati nel progetto)

| **CODICE** | **NOME** | **DESCRIZIONE** | **FORMULA** | **R/W** | **NOTE** |
|------------|----------|-----------------|-------------|---------|----------|
| `0x40` | **TEMP** | Temperatura | `TEMP/100 °C` | R | ❌ Non utilizzato |
| `0x45` | **LAX** | Accelerazione lineare X | `LAX/32768*16g` | R | ❌ Non utilizzato |
| `0x46` | **LAY** | Accelerazione lineare Y | `LAY/32768*16g` | R | ❌ Non utilizzato |
| `0x47` | **LAZ** | Accelerazione lineare Z | `LAZ/32768*16g` | R | ❌ Non utilizzato |
| `0x48` | **VX** | Velocità X | `VX/32768*16 m/s` | R | ❌ Non utilizzato |
| `0x49` | **VY** | Velocità Y | `VY/32768*16 m/s` | R | ❌ Non utilizzato |
| `0x4A` | **VZ** | Velocità Z | `VZ/32768*16 m/s` | R | ❌ Non utilizzato |
| `0x4B` | **PX** | Posizione X | `PX/32768*16 m` | R | ❌ Non utilizzato |
| `0x4C` | **PY** | Posizione Y | `PY/32768*16 m` | R | ❌ Non utilizzato |
| `0x4D` | **PZ** | Posizione Z | `PZ/32768*16 m` | R | ❌ Non utilizzato |
| `0x51` | **Q0** | Quaternione W | `Q0/32768` | R | ❌ Non utilizzato - usiamo Eulero |
| `0x52` | **Q1** | Quaternione X | `Q1/32768` | R | ❌ Non utilizzato - usiamo Eulero |
| `0x53` | **Q2** | Quaternione Y | `Q2/32768` | R | ❌ Non utilizzato - usiamo Eulero |
| `0x54` | **Q3** | Quaternione Z | `Q3/32768` | R | ❌ Non utilizzato - usiamo Eulero |

## 🆔 REGISTRI IDENTIFICAZIONE

| **CODICE** | **NOME** | **DESCRIZIONE** | **VALORI** | **R/W** | **NOTE** |
|------------|----------|-----------------|------------|---------|----------|
| `0x7F` | **NUMBERID1** | Numero seriale parte 1 | Device specific | R | ❌ Non utilizzato |
| `0x80` | **NUMBERID2** | Numero seriale parte 2 | Device specific | R | ❌ Non utilizzato |
| `0x81` | **NUMBERID3** | Numero seriale parte 3 | Device specific | R | ❌ Non utilizzato |
| `0x82` | **NUMBERID4** | Numero seriale parte 4 | Device specific | R | ❌ Non utilizzato |
| `0x83` | **NUMBERID5** | Numero seriale parte 5 | Device specific | R | ❌ Non utilizzato |
| `0x84` | **NUMBERID6** | Numero seriale parte 6 | Device specific | R | ❌ Non utilizzato |

## ⚙️ SEQUENZA INIZIALIZZAZIONE UTILIZZATA

```cpp
// 1. UNLOCK per scrittura registri
I2C_Write(0x50, 0xFF, [0xAA, 0x69, 0x88, 0xB5])
delay(150);

// 2. Configurazione Static Optimized
I2C_Write(0x50, 0x02, [0x1E, 0x00]);  // RSW: TIME+ACC+GYRO+ANGLE+MAG
delay(150);
I2C_Write(0x50, 0x03, [0x0A, 0x00]);  // RRATE: 10Hz
delay(150);
I2C_Write(0x50, 0x04, [0x01, 0x00]);  // BAUD: 9600
delay(150);
I2C_Write(0x50, 0x21, [0x00, 0x00]);  // ACC_RANGE: ±2g
delay(150);
I2C_Write(0x50, 0x22, [0x00, 0x00]);  // GYRO_RANGE: ±250°/s
delay(150);
I2C_Write(0x50, 0x23, [0x00, 0x00]);  // MAG_RANGE: ±4 gauss
delay(150);
I2C_Write(0x50, 0x24, [0x01, 0x00]);  // BANDWIDTH: Medium filter
delay(150);
I2C_Write(0x50, 0x25, [0x00, 0x00]);  // PWR_MODE: Normal
delay(150);
I2C_Write(0x50, 0x26, [0x01, 0x00]);  // ORIENT_MODE: 9-axis
delay(150);

// 3. SAVE & LOCK
I2C_Write(0x50, 0xFF, [0xAA, 0x00, 0x00, 0x00]);
delay(150);
```

## ⚠️ NOTE IMPORTANTI

- **Formato dati**: Tutti i valori sono 16-bit **little-endian** (Low byte first)
- **Delay obbligatorio**: 150ms tra ogni comando I2C
- **UNLOCK/LOCK**: Sempre necessari per modifiche permanenti
- **Lettura dati**: Avviene solo tramite **UART stream**, non via I2C
- **Indirizzo I2C**: Fisso a **0x50** nel nostro progetto
- **Range configurati**: Massima risoluzione (±2g, ±250°/s, ±4gauss)

## 🎯 REGISTRI DA IMPLEMENTARE

### Priority 1: Calibrazione Hardware
- **0x05-0x0D**: Offset registers per calibrazione persistente hardware
- Attualmente: Calibrazione solo software (persa al reboot)

### Priority 2: Estensioni Future
- **0x40**: Temperatura per compensazione termica
- **0x51-0x54**: Quaternioni per applicazioni 3D avanzate
- **0x27**: Lettura diretta via I2C per debugging