# Jump Measurement Platform

This project is designed to measure an athlete's jump parameters, including:

- **Flight Time**: The duration the athlete spends in the air.
- **Jump Height**: The calculated height based on flight time.
- **Impact Force**: The force exerted upon landing.

## Hardware Components

The system is built using the following components:

- **ESP32-WROOM-32**: A powerful microcontroller with Wi-Fi and Bluetooth capabilities.
- **ADS1256**: A high-resolution ADC (Analog-to-Digital Converter) for precise load cell readings.
- **YZC-516C Load Cell**: A force sensor to measure the applied load.

### Component Links:

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
| DRDY        | GPIO4     |
| RESET       | GPIO2     |

### Load Cell to ADS1256

The YZC-516C load cell has four wires:

- **Red** → +5V Power
- **Black** → GND
- **White** → Signal +
- **Green** → Signal -

Connections:

- **Cell 1**: White to AIN0, Green to AIN1 (Differential Mode)
- **Cell 2**: White to AIN2, Green to AIN3 (Differential Mode)

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

This program requires **Python 3.12 or higher** to run correctly.

---

This project enables precise measurement of an athlete’s jump performance, providing valuable data for training and analysis.

