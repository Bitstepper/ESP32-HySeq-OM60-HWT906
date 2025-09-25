

# 📋 HWT906 - misure effettuate sull'IMU
### nelle seguenti impostazioni software: 
Dopo aver riportato l'angolo di calcolo pitch a 180° come di seguito:
currentData.pitch_raw = ((float)pitch_raw / 32768.0f) * 180.0f;

e commentato le funzione:
// applyComplementaryFilter(); // ← COMMENTA QUESTA RIGA


## 🔧 Y=Yaw P=Pitch R=Roll

| **POSIZIONE IMU nello spazio** | **Intorno all'asse :** | **Gradi rotazione** | **Y** | **P** | **R** |
|------------|----------|-----------------|------------|---------|----------|
| nel piano orizzontale x-y | **Z** | 0| -69.5 | +0.53 | -68.3 |
| nel piano orizzontale x-y | **Z** |90| -138.0| +0.41| -68.4 |
| nel piano orizzontale x-y | **Z** | 180  | +104.6 | +0.27 | -68.4 |
| nel piano orizzontale x-y |**Z** | 270| -0.2| +0.35| -68.2 |
| nel piano orizzontale x-z | **Y** | 90 | +63.5 | +37.2 | -51.2 |
| nel piano orizzontale x-z | **Y** | 180|+165.5 | +1.5 | -2.1 |
| nel piano orizzontale z-y | **X** | 90 | -130.2 | +0.4 | -29.1 |
| nel piano orizzontale z-y` | **X** | 180 | -151.2 |-1.5 | -0.1 |

