# Sistema IoT de Detección de Deslizamientos

Sistema de monitoreo en tiempo real para la detección temprana de deslizamientos de tierra implementado en ESP32 con funcionalidades avanzadas de ISR + Timer automático.

### Wiki:(https://github.com/DanielPPPf/IoTChallenge/wiki)
## Características Principales

- **ISR para eventos sísmicos** con respuesta <1ms
- **Timer automático de reset** tras 30s de inactividad
- **Lógica de fusión contextual** con umbrales diferenciados
- **Dashboard web embebido** con API REST extendida
- **Sistema de calibración automática** ADC
- **Monitoreo de 4 variables críticas**: inclinación, lluvia, humedad del suelo, vibraciones

## Requisitos del Sistema

### Hardware Requerido
- **ESP32 Development Board** (30 pines)
- **Sensores**:
  - MPU6050 (inclinación) - I2C
  - SW-420 (vibración) - GPIO32/ADC1_CH4  
  - HW-103 (humedad suelo) - GPIO33/ADC1_CH5
  - YL-83 (lluvia) - GPIO35/ADC1_CH7
- **LEDs indicadores**: Verde (GPIO2), Amarillo (GPIO4), Rojo (GPIO5)
- **Resistencias**: 330Ω para LEDs, 4.7kΩ pull-up para I2C
- **Fuente**: 5V/3.3V para alimentación de componentes

### Software Requerido
- **ESP-IDF v5.0+** (framework oficial)
- **Python 3.8+** (para herramientas ESP-IDF)
- **Git** para clonar repositorio
- **VSCode** (recomendado) con extensión ESP-IDF

## Instalación Paso a Paso

### 1. Configuración del Entorno de Desarrollo

#### Instalar ESP-IDF v5.0+
```bash
# Linux/macOS
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.0.2
./install.sh
source export.sh

# Windows
# Usar ESP-IDF Installer oficial desde: https://dl.espressif.com/dl/esp-idf/
```

#### Verificar instalación
```bash
idf.py --version
# Debe mostrar ESP-IDF v5.0.2 o superior
```

### 2. Clonar y Configurar el Proyecto

```bash
# Clonar repositorio
git clone [URL_DEL_REPOSITORIO]
cd sistema-iot-deslizamientos

# Configurar target ESP32
idf.py set-target esp32

# Configuración del proyecto (opcional)
idf.py menuconfig
```

### 3. Conexión Física de Hardware

#### Configuración de Pines (IMPORTANTE: ADC1 Exclusivo)

| Componente | Pin Componente | Pin ESP32 | Canal | Función |
|------------|----------------|-----------|-------|---------|
| **MPU6050** | VCC | 3.3V | - | Alimentación |
| | GND | GND | - | Tierra |
| | SDA | GPIO21 | - | I2C Data |
| | SCL | GPIO22 | - | I2C Clock |
| **SW-420** | VCC | 3.3V | - | Alimentación |
| | GND | GND | - | Tierra |
| | DO | GPIO32 | ADC1_CH4 | ISR Digital |
| **HW-103** | VCC | 3.3V | - | Alimentación |
| | GND | GND | - | Tierra |
| | AO | GPIO33 | ADC1_CH5 | Humedad Suelo |
| **YL-83** | VCC | 3.3V | - | Alimentación |
| | GND | GND | - | Tierra |
| | AO | GPIO35 | ADC1_CH7 | Lluvia |
| **LEDs** | LED Verde + | GPIO2 | - | Normal + R330Ω |
| | LED Amarillo + | GPIO4 | - | Alerta + R330Ω |
| | LED Rojo + | GPIO5 | - | Crítico + R330Ω |
| | Cátodos (-) | GND | - | Tierra común |

#### Esquema de Conexión
```
ESP32 Development Board
┌─────────────────┐
│ 3.3V ●●● VIN    │  ← Alimentación 5V
│ GND  ●●● GND    │  ← Tierra común
│ GPIO21 (SDA)    │  ← MPU6050 SDA
│ GPIO22 (SCL)    │  ← MPU6050 SCL + Pull-up 4.7kΩ
│ GPIO32 (ADC1_4) │  ← SW-420 DO (ISR)
│ GPIO33 (ADC1_5) │  ← HW-103 AO 
│ GPIO35 (ADC1_7) │  ← YL-83 AO
│ GPIO2           │  ← LED Verde + R330Ω
│ GPIO4           │  ← LED Amarillo + R330Ω
│ GPIO5           │  ← LED Rojo + R330Ω
└─────────────────┘
```

**⚠️ IMPORTANTE**: Todos los sensores analógicos deben conectarse a pines ADC1. ADC2 no funciona con Wi-Fi activo.

### 4. Compilación y Flasheo

```bash
# Limpiar proyecto (recomendado)
idf.py clean

# Compilar proyecto
idf.py build

# Conectar ESP32 vía USB y flashear
idf.py flash

# Monitorear salida serial (opcional)
idf.py monitor

# Comando combinado (compilar + flashear + monitorear)
idf.py build flash monitor
```

### 5. Verificación de Funcionamiento

#### Secuencia de Inicio Esperada
1. **Prueba de LEDs**: Secuencia Verde → Amarillo → Rojo (2 ciclos)
2. **Inicialización I2C**: Verificación MPU6050 WHO_AM_I = 0x68
3. **Configuración ADC**: Detección automática de calibración
4. **Configuración ISR**: Setup de interrupción para SW-420
5. **Timer de reset**: Inicialización del timer automático
6. **Wi-Fi AP**: Creación de "WLAN_Alcaldia_Monitoreo"
7. **Servidor HTTP**: Inicio en puerto 80

#### Logs de Verificación Exitosa
```
I (2000) LANDSLIDE_MONITOR: MPU6050 inicializado correctamente
I (2100) LANDSLIDE_MONITOR: ISR de vibración configurado correctamente  
I (2200) LANDSLIDE_MONITOR: Timer de reset automático configurado correctamente
I (3000) LANDSLIDE_MONITOR: Wi-Fi AP iniciado: WLAN_Alcaldia_Monitoreo
I (3500) LANDSLIDE_MONITOR: Servidor HTTP iniciado en http://192.168.1.1/
I (4000) LANDSLIDE_MONITOR: === Sistema Completo Iniciado ===
```

### 6. Acceso al Dashboard Web

#### Conexión a la Red
1. **SSID**: `WLAN_Alcaldia_Monitoreo`
2. **Contraseña**: `MonitoreoTaludes2025`
3. **IP del dashboard**: `http://192.168.1.1/`
4. **IP del API**: `http://192.168.1.1/api/sensores/estado`

#### Verificación del Dashboard
- **Conectividad**: Múltiples dispositivos simultáneos (PC/móvil)
- **Actualización automática**: Cada 5 segundos
- **Indicadores técnicos**: Estado ISR, timer, umbrales
- **API funcional**: Respuesta JSON con metadatos

### 7. Calibración y Ajustes Iniciales

#### Verificación de Sensores
```bash
# Monitorear lecturas en tiempo real
idf.py monitor

# Verificar que los valores cambien dinámicamente:
# - SW-420: Valores variables al mover el sensor
# - HW-103: Diferentes valores en seco/húmedo  
# - YL-83: Cambios al simular lluvia
# - MPU6050: Cambios de pitch/roll al inclinar
```

#### Calibración de Umbrales (si necesario)
Los umbrales están preconfigurados pero pueden ajustarse en `main.c`:

```c
// Umbrales de inclinación
#define INCLINATION_WARNING  15.0f
#define INCLINATION_CRITICAL 25.0f

// Umbrales de vibración contextual
#define VIBRATION_COUNT_ALERT 5
#define VIBRATION_COUNT_HIGH 50  
#define VIBRATION_COUNT_RAIN_CRITICAL 100
#define VIBRATION_COUNT_CRITICAL 200
```

### 8. Pruebas del Sistema ISR + Timer

#### Verificar ISR de Vibración
1. **Tocar/mover el SW-420** físicamente
2. **Observar logs**: "¡EVENTO SÍSMICO! Vibración detectada por ISR"
3. **Verificar contador**: Se incrementa con cada evento
4. **Confirmar timer**: Se reinicia automáticamente

#### Verificar Timer de Reset
1. **Generar vibraciones** para incrementar contador
2. **Esperar 30 segundos** sin nuevas vibraciones
3. **Observar reset**: "RESET AUTOMÁTICO: Contador reseteado tras 30 segundos"

#### Verificar Lógica Contextual
1. **Simular lluvia**: Humedecer YL-83
2. **Simular suelo húmedo**: Humedecer HW-103  
3. **Generar 100+ vibraciones**: Tocar SW-420 repetidamente
4. **Verificar alerta crítica**: LED Rojo + Log "CRÍTICO: Lluvia + suelo saturado + vibraciones críticas"

## Solución de Problemas Comunes

### Problema: Sensores ADC con valores fijos
**Síntoma**: SW-420 o HW-103 muestran siempre el mismo valor (~128mV)
**Causa**: Sensores conectados a ADC2 en lugar de ADC1
**Solución**: Verificar conexiones en GPIO32, GPIO33, GPIO35 (ADC1 exclusivo)

### Problema: ISR no funciona
**Síntoma**: No se detectan vibraciones a pesar de mover SW-420
**Causa**: Pin incorrecto o configuración de ISR
**Solución**: 
1. Verificar conexión en GPIO32
2. Revisar que el LED del SW-420 parpadee al moverlo
3. Confirmar configuración `#define SW420_OUT_PIN GPIO_NUM_32`

### Problema: No se conecta al Wi-Fi
**Síntoma**: Red "WLAN_Alcaldia_Monitoreo" no aparece
**Causa**: Error en inicialización de Wi-Fi AP
**Solución**: Revisar logs de inicialización, verificar que no haya conflictos de red

### Problema: Dashboard no carga
**Síntoma**: Página web no responde en 192.168.1.1
**Causa**: Servidor HTTP no iniciado o memoria insuficiente
**Solución**: 
1. Verificar logs: "Servidor HTTP iniciado"
2. Verificar conexión a la red correcta
3. Revisar configuración de memoria en `menuconfig`

### Problema: Lecturas erráticas de sensores
**Síntoma**: Valores inconsistentes o que no reflejan cambios físicos
**Causa**: Interferencias o conexiones deficientes
**Solución**:
1. Verificar alimentación estable 3.3V/5V
2. Confirmar conexiones firmes
3. Separar cables de sensores analógicos de fuentes de ruido

## Estructura del Proyecto

```
sistema-iot-deslizamientos/
├── main/
│   ├── main.c              # Código principal monolítico
│   └── CMakeLists.txt      # Configuración de compilación
├── CMakeLists.txt          # Configuración raíz del proyecto
├── sdkconfig               # Configuración del ESP32
├── README.md               # Este archivo

```

## Configuración Avanzada

### Modificar Credenciales de Red
```c
// En main.c, líneas ~40-50
#define WIFI_SSID "TU_SSID_PERSONALIZADO"
#define WIFI_PASS "TU_PASSWORD_PERSONALIZADO"
```

### Ajustar Intervalo de Monitoreo
```c
// En main.c, línea ~35
#define REGULAR_MEASUREMENT_INTERVAL_MS 5000  // 5 segundos por defecto
```

### Habilitar Logs Detallados
```bash
# En menuconfig
idf.py menuconfig
# Component config → Log output → Default log verbosity → Verbose
```

## Características del Sistema

### Funcionalidades Implementadas
- **ISR inmediato** para eventos sísmicos (<1ms respuesta)
- **Timer automático** de reset tras 30s inactividad
- **Umbrales contextuales**: 100 vibraciones críticas CON lluvia+húmedo, 200 críticas siempre
- **Calibración automática** de ADC según hardware disponible
- **API REST extendida** con metadatos técnicos del sistema
- **Dashboard responsivo** con indicadores técnicos avanzados
- **Logging inteligente**: Detallado para eventos sísmicos, resumido para operación normal

### Niveles de Alerta
- **Normal (LED Verde)**: Condiciones estables
- **Alerta (LED Amarillo)**: Condiciones de riesgo moderado
- **Crítico (LED Rojo)**: Riesgo inminente, acción requerida

### Especificaciones Técnicas
- **Microcontrolador**: ESP32 240MHz dual-core
- **Memoria**: ~50KB RAM para servidor HTTP, ~100KB Flash para assets web
- **Conectividad**: Wi-Fi 802.11n/g 2.4GHz, hasta 7 clientes simultáneos
- **Sensores**: 1x I2C (100kHz), 3x ADC1 (12-bit), 1x ISR digital
- **Actualización**: Dashboard cada 5s, ISR <1ms, Timer precisión ±100ms

## Licencia

Este proyecto está desarrollado para el curso de Internet de las Cosas, Facultad de Ingeniería, Universidad de La Sabana, como respuesta al Challenge#2 de detección de deslizamientos de tierra.

---

**Desarrollado por**: Daniel Pareja Franco, Juan Jose Forero, Josue David Sarmiento  
**Curso**: Internet de las Cosas 2025-2  
**Universidad**: Universidad de La Sabana  
**Fecha**: Septiembre 2025
