# Jump Measurement Platform

This project is designed to measure an athlete's jump parameters, including:

- **Flight Time**: The duration the athlete spends in the air.
- **Jump Height**: The calculated height based on flight time.
- **Impact Force**: The force exerted upon landing.
- **Body Mass**: The athlete's mass is calculated from load cell readings.

## Hardware Components

The system is built using the following components:

- **ESP32-WROOM-32**: A powerful microcontroller with Wi-Fi and Bluetooth capabilities.
- **ADS1256**: A high-resolution ADC (Analog-to-Digital Converter) for precise load cell readings.
- **YZC-516C Load Cells (x4)**: Force sensors to measure the applied load.

### Component Links

- [ESP32-WROOM-32](https://www.espressif.com/en/products/socs/esp32)
- [ADS1256](https://www.ti.com/product/ADS1256)
- [YZC-516C Load Cell](images/datasheet_YZC-516C.jpeg)

## Wiring Connections

### ESP32 to ADS1256 (SPI)

| ADS1256 Pin | ESP32 Pin |
| ----------- | --------- |
| VCC (5V)    | 5V        |
| GND         | GND       |
| DIN (MOSI)  | GPIO23    |
| DOUT (MISO) | GPIO19    |
| SCLK        | GPIO18    |
| CS          | GPIO5     |
| DRDY        | GPIO16    |
| POWN        | 5V        |

### Load Cells to ADS1256

The system now uses **four** YZC-516C load cells, each connected in **differential mode**:

- **Cell 1**: White to AIN0, Green to AIN1
- **Cell 2**: White to AIN2, Green to AIN3
- **Cell 3**: White to AIN4, Green to AIN5
- **Cell 4**: White to AIN6, Green to AIN7

**Wiring per cell:**
- **Red** → +5V Power
- **Black** → GND
- **White** → Signal +
- **Green** → Signal -

## Driver Installation

To use the ESP32 with your PC, install the CH340 driver from the following link: [CH340 Driver Installation](https://sparks.gogo.co.nz/ch340.html)

## Software Installation

To install the required libraries, run the following command:

```sh
pip install -r requirements.txt
```

## Usage

Once the hardware is set up and software dependencies are installed, you can run the program using:

```sh
python main.py
```

### Requirements

- **Python 3.12 or higher**

---

This project enables precise measurement of an athlete’s jump performance, providing valuable data for training, performance evaluation, and biomechanical analysis.
