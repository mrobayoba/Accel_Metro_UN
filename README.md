# 🚄 Accel_Metro_UN - High-Frequency Vibration Data Logger

### *Stable version of the vibrometer developed by UN for the Metro of Medellin as part of a research project.*

This project is a portable, high-speed data logger designed to measure and record intense vibrations. It was built using STMicroelectronics (STM32) hardware and is designed to process massive amounts of precise vibration data without losing a single drop. 

Even if you don't know much about embedded programming or microcontrollers, this guide will help you understand what this device is, how it works, and how to use it.

---

## 🛠️ What is the Hardware?

This project relies on three main components to work together:

1. **The Brain: [NUCLEO-F411RE Development Board](https://www.st.com/en/evaluation-tools/nucleo-f411re.html)**  
   This is the main computer of the system. It uses an ARM Cortex-M4 processor, which is powerful enough to handle mathematical calculations and move data around very quickly. 
2. **The Sensor: [STM32 IIS3DWB Accelerometer](https://www.st.com/en/mems-and-sensors/iis3dwb.html)**  
   This is an industrial-grade, 3-axis vibration sensor. Unlike the accelerometer in your smartphone, this sensor is designed to measure very fast and violent movements (taking up to **26,667 measurements every single second**!).
3. **The Storage: MicroSD Card Module**  
   All the vibration data is saved onto a standard MicroSD card in a simple text file, making it easy to open in Excel, Python, or MATLAB later.

---

## 🚦 How to Use the Logger

Using the vibrometer is designed to be as simple as pressing a button.

### 1. Preparation
1. Make sure the device is powered off.
2. Insert a MicroSD card (it should be formatted as FAT32) into the SD card slot.
3. Power the device on (e.g., via USB or a battery bank). 
   - A **Blinking LED (pin PC1)** will indicate that the system is turned on, alive, and ready to go.

### 2. Start Recording
- Press the **Blue User Button** down once.
- The system will safely mount the SD card and prepare a new text file. 
- The **Recording LED (pin PA4)** will light up solid, telling you that data is actively being saved to the SD card. 

### 3. Stop Recording
- **Always stop the recording before removing power!** If you pull the power cord while it is recording, your file might become corrupted.
- Press the **Blue User Button** down once again.
- The **Recording LED** will turn off. 
- The SD card is now safe to remove. You can plug it into your computer to read your vibration data.

### ⚠️ What if the Error LED turns on?
If the **Error LED (pin PC0)** turns on, something went wrong. This usually means:
- The SD card is missing, locked, or corrupted.
- The SD card is too slow to keep up with the vibration data.
- **Fix:** Power the board off, check your SD card on a PC, plug it back in tightly, and try again.

---

## 💾 Understanding the Data

Once you plug the SD card into your PC, you will find a file named `record_1.txt`. 

When you open it, the top of the file contains helpful information about the settings used during that specific recording. The actual data is organized into columns like this:

```text
Timestamp(ms) accelX(mg) accelY(mg) accelZ(mg)
0.000 15 22 -1004
0.500 17 21 -1008
1.000 14 24 -1002
...
```
* **Timestamp(ms):** The exact time the measurement was taken, in milliseconds.
* **accelX, accelY, accelZ:** The vibration forces measured in all three spatial dimensions (X, Y, and Z). It is recorded in `mg` (milli-g's, meaning thousandths of Earth's gravity). So, `-1000` means exactly -1G (the normal pull of Earth's gravity).

---

## ⚙️ Technical Details (For the Curious)

Recording data at 26kHz directly to an SD card is tricky because SD cards sometimes "freeze" for a fraction of a second while saving. To prevent data loss, this firmware uses several advanced features:

* **Hardware FIFO (First-In, First-Out):** The accelerometer has its own tiny memory buffer. It saves measurements by itself and only taps the main processor on the shoulder when it has a "batch" of data ready to be collected.
* **DMA (Direct Memory Access):** When reading the sensor, we use DMA. This is a special hardware shortcut that physically moves the data from the sensor into the chip's memory automatically, letting the main processor do other work (like formatting text) at the exact same time.
* **Double Buffering:** We keep two separate 4-Kilobyte memory "buckets" for text data. While the SD card is busy saving Bucket A, the sensor is actively filling up Bucket B. They swap seamlessly.
* **FatFs File System:** We use the [FatFs module by Elm-Chan](http://elm-chan.org/fsw/ff/00index.html), an industry standard for safely writing files to SD cards on small microcontrollers.

---

## 📚 Official Documentation & References

If you wish to learn more about the components or modify the firmware yourself, here are the official manuals:

* **NUCLEO-F411RE Board:** [Schematics, pinouts, and User Manual (UM1724)](https://www.st.com/en/evaluation-tools/nucleo-f411re.html#documentation)
* **STM32F411 MCU:** [Reference Manual (RM0383)](https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) - A deep dive into how the chip's inner hardware (DMA, SPI, Interrupts) works.
* **IIS3DWB Accelerometer:** [Datasheet](https://www.st.com/resource/en/datasheet/iis3dwb.pdf) - Explains all the internal registers, the hardware FIFO, and its massive 26.7 kHz output data rate.
* **FatFs file system:** [Documentation](http://elm-chan.org/fsw/ff/00index.html) - For understanding SD card file writing on embedded systems.
