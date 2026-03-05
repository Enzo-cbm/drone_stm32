# Drone quadricoptère (STM32) — Architecture embarquée (RTOS) | IMU DMA + Radio Capture Timer

Firmware “from scratch” pour un contrôleur de vol sur STM32 (FreeRTOS) orienté performance temps réel et architecture modulaire.
État actuel : acquisition IMU (MPU6050) en I²C + DMA, capture des commandes radio par timer input-capture (interruptions), base temps en µs.
Objectif : vol stabilisé robuste puis extension vers navigation (capteurs externes).

-------------------------------------------------------------------------------

## Statut du projet
- ✅ Module IMU : MPU6050 (I²C + DMA + task notification)
- ✅ Module Radio : capture PWM RC via TIM1 Input Capture + callback HAL
- ✅ Base temps : timebase en microsecondes (pour dt, timeouts, etc.)
- ⏳ Contrôle / moteurs / attitude / PID : (à ajouter)
- ⏳ Intégration capteurs : (à ajouter)

-------------------------------------------------------------------------------

## Roadmap capteurs (prévu)
Ajouts prévus pour améliorer la stabilisation et préparer des modes “assistés” :
- Baromètre (altitude / maintien d’altitude)
- LiDAR (distance sol / anti-collision / altitude proche sol)
- Magnétomètre (cap / réduction dérive yaw)
- GPS (position / vitesse / modes position-hold et retour)
- Caméra optique (stabilisation visuelle / optical flow / navigation assistée)

-------------------------------------------------------------------------------

## Vue d’ensemble (architecture)
- Les modules sont conçus comme des “drivers” isolés :
  - acquisition non-bloquante (DMA / IRQ)
  - publication d’un snapshot cohérent
  - RTOS : notification de tâche uniquement quand des données utiles sont prêtes

Dataflow (actuel)
Radio (PWM) -> TIM1 Input Capture IRQ -> pulses -> radio_update() -> status / snapshot
IMU -> I2C DMA -> HAL_I2C_MemRxCpltCallback -> notify task -> MPU_ProcessLatest() -> imu_sample

-------------------------------------------------------------------------------

## Organisation du dépôt (proposée)



drone-stm32/
│
├── Core/
│   ├── Inc/
│   │   ├── imu.h
│   │   ├── radio.h
│   │   └── timebase.h
│   │
│   └── Src/
│       ├── imu.c
│       └── radio.c
│
├── docs/
│   └── wiring.md
│
├── assets/
│   └── (à ajouter)
│
└── README.md


-------------------------------------------------------------------------------

## Modules (détails)

### 1) IMU — MPU6050 (imu.h / imu.c)
Objectif :
- lecture accel/gyro/temperature via I²C
- mode non-bloquant : DMA -> callback -> notification RTOS
- stockage d’un échantillon courant (imu_sample_t)
- calibration offsets (gyro + accel)

Points clés :
- Adresse : MPU_ADDR = (0x68 << 1)
- Lecture burst : 14 octets à partir de 0x3B
- Démarrage lecture DMA : MPU_StartReadDMA()
- Fin DMA : HAL_I2C_MemRxCpltCallback() -> rx_done=1 + notify task
- Traitement : MPU_ProcessLatest() (parse buffer, applique convention d’axes, lance calibration)
- Getter : MPU_GetSample()
- Calibration :
  - MPU_CalibStart()
  - MPU_IsCalibrated()
  - accumulation sur calib_target = 2000 échantillons
  - correction accel Z : imu.accel_offset[z] -= 1/accel_scale

Remarques (qualité/robustesse) :
- Ajouter un test WHO_AM_I au boot (prévu dans le header).
- Prévoir une gestion d’erreur init I²C (LED / statut global).
- Idée future : double-buffer (snapshot stable) si plusieurs tâches consomment imu.

### 2) Radio — Capture PWM RC par timer (radio.h / radio.c)
Objectif :
- mesurer la largeur d’impulsion RC (µs) sur 4 canaux
- traitement minimal dans l’IRQ
- publication d’une “frame” cohérente (tous les canaux reçus) + notify task

Configuration attendue :
- TIM1 en Input Capture sur 4 canaux
- Timer configuré pour que 1 tick = 1 µs (ex : 16 MHz, PSC=15)
- Gestion wrap 16-bit via diff_ticks()

Fonctionnement :
- IRQ : HAL_TIM_IC_CaptureCallback()
  - identifie canal (HAL_TIM_ACTIVE_CHANNEL_x)
  - rising : mémorise t_rise, bascule polarité en FALLING
  - falling : calcule width, bascule polarité en RISING
  - filtre largeurs aberrantes
  - agrégation : agg_mask |= (1<<ch)
  - si tous canaux attendus : pulses.frame_counter++, pulses.last_update_us=..., pulses.valid_mask=..., notify RTOS

API :
- radioDriver_init(cfg)
- radioDriver_reset()
- radio_update() : met à jour status (âge + timeout + link_ok)
- radio_get_raw_snapshot(out) : copie atomique (IRQ off)
- radio_get_status(out)
- radio_is_sample_valid()
- radio_link_ok()
- RTOS : radioRtos_bind_task(TaskHandle_t)

À corriger / à ajouter :
- pulses.last_update_us = timebase_now_us_isr(); -> à implémenter dans timebase (fonction ISR-safe).
- Dans radio_update(), besoin de timebase_now_us() fonctionnelle.
- timeout_us dans cfg_radio : doit être cohérent avec la fréquence RC.
- Vérifier min/max pulse : ton code compare width < min-200 et > max+500, OK mais à calibrer.

### 3) Timebase — base temps en microsecondes (timebase.h)
Objectif :
- fournir un “now_us” stable pour timeouts et dt
- fonction dt_s() pratique pour les boucles de contrôle

API prévue :
- timebase_init()
- timebase_now_us()
- timebase_dt_s(uint32_t *last_us)

À ajouter (recommandé) :
- timebase_now_us_isr() : version utilisable en interruption (sans appel bloquant).
- préciser l’implémentation (timer dédié ou DWT cycle counter selon STM32F4).

-------------------------------------------------------------------------------

## Câblage minimal (actuel)
(à ajouter)

- IMU MPU6050 :
  - I²C (hi2c1) : SDA/SCL selon ton board + pullups si besoin
  - VCC/GND

- Radio :
  - 4 entrées PWM vers TIM1 CH1..CH4
  - masse commune récepteur <-> STM32

-------------------------------------------------------------------------------

## Compilation / environnement
- STM32CubeIDE + HAL + FreeRTOS (USE_FREERTOS)
- DMA activé pour I²C (lecture MPU6050)
- TIM1 configuré en input capture (4 canaux)

(à ajouter) : version MCU exacte, board, fréquence, config CubeMX.

-------------------------------------------------------------------------------

## Sécurité / bonnes pratiques
- Garder les callbacks IRQ courts (OK : traitement minimal + notify).
- Toujours gérer les timeouts radio (link_ok) avant d’armer les moteurs.
- Tester au banc sans hélices lors de l’intégration moteurs/ESC.
- Ajouter un état global “fault” si IMU ne répond pas.

-------------------------------------------------------------------------------

## TODO (prochaines étapes)
- Implémenter timebase_now_us_isr()
- Ajouter test WHO_AM_I MPU6050 et gestion d’erreur d’init
- Ajouter module attitude (Mahony / Madgwick) sur STM32 (float + quaternions)
- Ajouter PID + mixage moteurs (avec saturation + anti-windup)
- Ajouter driver moteurs (PWM timers) + arming/failsafe
- Intégration capteurs : baro, lidar, mag, GPS, caméra optique (vol stabilisé + modes assistés)

-------------------------------------------------------------------------------

## Licence
(à ajouter)

## Contact
Enzo Coranson-Beaudu — (à ajouter)
