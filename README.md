# Naturally Augmented Guitar

## Overview
The Naturally Augmented Guitar is an electric guitar enhanced with sensor technology and software processing to modulate sound frequencies. The instrument remains fully functional as a standard electric guitar while also serving as a versatile controller, including MIDI control capabilities.

A common drawback of many augmented instruments, particularly augmented guitars, is that they often require a fundamental change in the way the instrument is played. This project prioritizes two key principles:
1. The guitar must remain fully playable as a conventional instrument.
2. The augmentations should occur naturally as an organic consequence of traditional guitar performance, rather than through external, unrelated interactions.

For example, if an augmented drum kit were designed, software modulations should ideally stem from natural drumming actions, such as the force of each hit, rather than from external factors like ambient light levels or environmental temperature.

A notable comparison can be made to Enda Bates' augmented guitar, which does not adhere to this naturalistic approach. Bates' guitar is not tuned in a standard fashion, making it impractical for conventional use. Additionally, its performance modulations rely on spatial hand movements and guitar orientation changes, occurring after strumming rather than as an intrinsic part of the act. In contrast, the Naturally Augmented Guitar integrates software modulations directly tied to natural aspects of guitar playing.

## Sensors and Implementation
### Sensors Used
- **Force Sensitive Resistors (FSRs):**
  - A 1.5-inch square FSR placed on the floor, controlled via foot interaction.
  - A 0.5-inch circular FSR attached to the guitar pick to measure grip pressure.
  - An additional FSR mounted along the back of the neck beneath a 400mm potentiometer strip.

- **Soft Potentiometer Strips:**
  - Three strips attached to the back of the neck:
    - Two 200mm strips.
    - One 400mm strip (which may be substituted with a "HotPot" depending on sensor availability).
  - Due to the electrical properties of membrane potentiometers, data downscaling is required for positional accuracy. Pressing two points on a strip may produce identical values to pressing another single point elsewhere on the strip.

- **MPU6050 Gyroscope/Accelerometer:**
  - A six-axis sensor mounted on the headstock, providing raw acceleration and rotation data, which is scaled to MIDI range before being sent to Max/MSP.

## Performance Gestures
The Naturally Augmented Guitar is designed to enhance sound through the natural performance gestures of the guitarist, without requiring any additional performance techniques beyond interaction with the embedded hardware.

- The FSR on the neck measures fret pressure.
- The FSR on the guitar pick measures grip intensity.
- The floor-mounted FSR functions as an arbitrary foot controller.
- The soft potentiometer strips measure the position of the fretting hand along the neck.
- The MPU6050 measures acceleration and rotation, capturing subtle movements such as vibrato, which can be mapped to effects like panning, wah, or pitch bending.

## Artistic Vision
The guitar has been a deeply personal instrument for over a decade. This project extends its expressive capabilities by naturally integrating augmentation, preserving the instrument's organic feel while enabling new sonic possibilities. The inclusion of foot-controlled elements aligns with traditional guitar performance techniques, particularly in live settings where foot pedals are commonly used.

## Sound Outcome
The primary objective is to generate data in response to natural guitar playing gestures, rather than to enforce predefined sonic outputs. However, several mapping strategies have been implemented:

- **MPU6050 Sensor:**
  - Pitch mapped to the high-cut frequency of the introductory instrument in Logic.
  - Roll mapped to panning of the introductory instrument in Logic.

- **Soft Potentiometer Strips:**
  - One strip mapped to the Sync 2 parameter of the lead instrument.
  - Another strip mapped to the digital level of the Tidal Space instrument.
  - The HotPot strip mapped to the drive level of a bass instrument.

- **FSRs:**
  - The FSR on the guitar pick mapped to the distortion level of the live guitar signal.
  - The floor-mounted FSR mapped to the phaser rate of the live guitar signal.

## Issues and Revisions
During testing, all sensors provided clean, linear data when connected to the ESP32 via a breadboard. However, after permanently mounting the sensors to the guitar, several issues arose:

1. **Soft Potentiometer Strip Malfunction:**
   - One of the 200mm strips suffered a momentary short, causing intermittent low readings (approximately 600/4095) when no pressure is applied.

2. **Non-Linear Response from 400mm Strip:**
   - The data output is now exhibiting an exponential response rather than the expected linearity. For example, pressing at 200mm (midpoint) yields a 12-bit value around 1024/4095 instead of the anticipated 2048/4095.

3. **FSR and HotPot Interference:**
   - When initially tested separately, both the long FSR and HotPot strip functioned correctly. However, after adhesion, the FSR began detecting near-maximal pressure readings at all times. A 4.7kΩ resistor was added in-line to reduce current load.

4. **Structural Considerations:**
   - While the initial intent was to integrate all components seamlessly into the guitar’s original form factor, unforeseen complications necessitated external attachments. Many components are currently secured with electrical tape for accessibility in case of modifications or repairs.

5. **Copper Tape Wiring Issue:**
   - An attempt was made to replace bundled wiring with copper tape along the neck for a cleaner design. However, the neck itself exhibited slight conductivity, interfering with signal integrity. Future revisions may involve insulating the neck with electrical tape or another non-conductive layer before applying the copper tape.

## Conclusion
The Naturally Augmented Guitar successfully integrates sensor-based augmentation while preserving the traditional playability of an electric guitar. Despite minor hardware challenges, the project achieves its core goal of using natural performance gestures to drive sound modulation. Future iterations will focus on refining sensor placement, improving structural integration, and addressing identified data inconsistencies.

