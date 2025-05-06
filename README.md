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
## Description of the mathematical calculations in jump tests

Description of the steps and mathematical formulas used in the internal functions of the tests:

- **Test Depth Jump**  
- **Test Normal Jump**  
- **Test Calculate Force**  
- **Test Calculate Mass**  

---

### 1. testDepthJump

#### Threshold calculations:

1. **Weight**  
   The athlete’s weight is calculated as the product of mass and gravity:  
   $$  
   \mathrm{weight} = m \cdot g  
   $$

2. **Net force**  
   The net force applied is the difference between the measured force and the weight:  
   $$  
   F_{\rm net}(t) = F(t) - \mathrm{weight}  
   $$

3. **Acceleration**  
   The instantaneous acceleration is given by:  
   $$  
   a(t) = \frac{F_{\rm net}(t)}{m}  
   $$

4. **Amortization start threshold $(t_0)$**  
   The first index $t_0$ where the force exceeds a certain percentage of the weight:  
   $$  
   t_0 = \min\{\,t \ge 0 : F(t) \ge \alpha_{\rm start}\,\mathrm{weight}\},  
   \quad \alpha_{\rm start} = \frac{\text{start\_threshold\%}}{100}  
   $$

5. **Impact velocity**  
   Before contact, the subject falls from a known height $h$; the impact velocity is  
   $$  
   v_{\rm impact} = -\sqrt{2\,g\,h}  
   $$

6. **Velocity during amortization**  
   Integrating the acceleration (cumulative trapezoidal method) starting from $v_{\rm impact}$:  
   $$  
   v(t) = v_{\rm impact} + \int_{t_0}^{t} a(\tau)\,\mathrm{d}\tau  
   $$

7. **Push-off start $(t_1)$**  
   The first instant after $t_0$ when velocity reaches zero (within a threshold):  
   $$  
   t_1 = \min\{\,t \ge t_0 : |v(t)| \le \varepsilon_v\}  
   $$

8. **Peak amortization force $(t_2)$**  
   $$
   t_2 = \arg\min_{\,t_0 \le t < t_1} F(t)
   $$

9. **Take‑off $(t_3)$**  
   $$
   t_3 = \min\bigl\{\,t \ge t_1 : F(t) \le \alpha_{\rm takeoff}\,\mathrm{weight}\bigr\},  
   \quad \alpha_{\rm takeoff} = \frac{\text{takeoff\_threshold\%}}{100}
   $$

10. **Landing $(t_4)$**  
    $$
    t_4 = \min\bigl\{\,t \ge t_3 : F(t) > \alpha_{\rm takeoff}\,\mathrm{weight}\bigr\}
    $$

11. **Peak landing force $(t_5)$**  
     $$
     t_5 = \arg\max_{\,t \ge t_4} F(t)
     $$

#### Value calculations:

1. **Push-off impulse**  
   Calculated by integrating the net force between push‑off start and take‑off:  
   $$  
   I = \int_{t_1}^{t_3} \bigl[F(\tau) - m\,g\bigr]\,\mathrm{d}\tau  
   $$

2. **Take‑off velocity**  
   From the impulse $I$:  
   $$  
   v_0 = \frac{I}{m}  
   $$

3. **Jump height (from force)**  
   From energy conservation:  
   $$  
   h_{\rm force} = \frac{v_0^2}{2\,g}  
   $$

4. **Contact time and flight time**  
   - **Contact time**:  
     $$ \Delta t_c = t_3 - t_0 $$  
   - **Flight time**:  
     $$ \Delta t_f = t_4 - t_3 $$

5. **Jump height (from flight time)**  
   Assuming uniformly accelerated motion up and down:  
   $$  
   h_{\rm flight} = \frac12\,g\Bigl(\frac{\Delta t_f}{2}\Bigr)^2  
   $$

6. **Estimated average height**  
   $$  
   h_{\rm avg} = \frac{h_{\rm force} + h_{\rm flight}}{2}  
   $$

---

### 2. testNormalJump

#### Threshold calculations:

1. **Weight**  
   $$ \mathrm{weight} = m \cdot g $$

2. **Start Movement $(t_0)$**  
   $$  
   t_0 = \min\bigl\{\,t \ge 0 : F(t) \le \alpha_{\rm start}\,\mathrm{weight}\bigr\},  
   \quad \alpha_{\rm start} = \frac{\text{start\_threshold\%}}{100}
   $$

3. **Start Deceleration $(t_2)$**  
   $$  
   t_2 = \min\bigl\{\,t \ge t_0 : F(t) \ge \mathrm{weight}\bigr\}
   $$

4. **Start Braking $(t_1)$**  
   $$  
   t_1 = \arg\min_{\,t_0 \le t < t_2} F(t)
   $$

5. **Take‑off $(t_4)$**  
   $$  
   t_4 = \min\bigl\{\,t \ge t_2 : F(t) \le \alpha_{\rm takeoff}\,\mathrm{weight}\bigr\},  
   \quad \alpha_{\rm takeoff} = \frac{\text{takeoff\_threshold\%}}{100}
   $$

6. **Peak Take‑off Force $(t_3)$**  
   $$  
   t_3 = \arg\max_{\,t_2 \le t < t_4} F(t)
   $$

7. **Landing $(t_5)$**  
   $$  
   t_5 = \min\bigl\{\,t \ge t_4 : F(t) > \alpha_{\rm takeoff}\,\mathrm{weight}\bigr\}
   $$

8. **Peak Landing Force $(t_6)$**  
   $$  
   t_6 = \arg\max_{\,t \ge t_5} F(t)
   $$

9. **Negative impulse and balancing**  
   - **Negative (eccentric) impulse**:  
     $$  
     I_- = \int_{t_0}^{t_2} \bigl[F(\tau) - m\,g\bigr]\,\mathrm{d}\tau  
     $$  
   - **Cumulative deceleration**:  
     $$  
     A(t) = \int_{t_2}^{t} \bigl[F(\tau) - m\,g\bigr]\,\mathrm{d}\tau  
     $$  
   - **Balancing $(t_b)$**:  
     $$  
     t_b = \min\bigl\{\,t \ge t_2 : A(t)\ge |I_-|\bigr\}
     $$

10. **End of push‑off $(t_7)$**  
    $$  
    t_7 = \min\bigl\{\,t \ge t_b : F(t) \le \mathrm{weight}\bigr\}
    $$

#### Value calculations:

Same as **Test Depth Jump**, but the impulse integral is taken from **balancing** to **take‑off**:
$$
I = \int_{t_b}^{t_4} \bigl[F(\tau) - m\,g\bigr]\,\mathrm{d}\tau
$$  
followed by:
- $v_0 = I/m$  
- $h_{\rm force} = v_0^2/(2\,g)$  
- $h_{\rm flight} = \tfrac12\,g(\Delta t_f/2)^2$  
- $h_{\rm avg} = (h_{\rm force} + h_{\rm flight})/2$  

---

## 3. Test Calculate Force

1. **Mean force**  
   $$ \bar F = \frac{1}{N}\sum_{i=1}^N F_i $$

2. **Index of maximum force**  
   $$ k = \arg\max_i F_i $$

3. **Resulting mass** (from mean force):  
   $$ m_{\rm avg} = \frac{\bar F}{g} $$

4. **Maximum mass** (from peak force):  
   $$ m_{\rm max} = \frac{F_k}{g} $$

---

## 4. Test Calculate Mass

1. **Mean force**  
   $$ \bar F = \frac{1}{N}\sum_{i=1}^N F_i $$

2. **Resulting mass**:  
   $$ m = \frac{\bar F}{g} $$

---

This project enables precise measurement of an athlete’s jump performance, providing valuable data for training, performance evaluation, and biomechanical analysis.
