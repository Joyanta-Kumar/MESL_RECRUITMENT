# The Sweet Tasks
A set of basic to intermediate Arduino projects designed to strengthen both circuit-building and programming skills.  

---

## Common components
These items are assumed available for every project and are not repeated in individual lists:  
- Arduino Uno (or compatible board)  
- Breadboard  
- Jumper Wires (male-to-male, male-to-female as needed)  
- USB Cable for programming/power  
- Power Source (if external supply is required)  

---

## Table of Contents
- [AurkaXi](#aurkaxi)
- [Murad](#murad)
- [Fida](#fida)
- [Pranto](#pranto)
- [Tasnim](#tasnim)
- [Earthij](#earthij)
- [Sami](#sami)
- [Tanmay](#tanmay)
- [Jubaer](#jubaer)
- [Rabiul](#rabiul)
- [Junayed](#junayed)
- [Jahid](#jahid)
- [Abir](#abir)
- [Judging Criteria](#judging-criteria)

---

## AurkaXi
### Smart Pedestrian Traffic Light 
Build a traffic light system with a pedestrian button.  

#### Required Components:
- Red, Yellow, Green LEDs  
- Push Button  
- Resistors (for LEDs and button)  
- (Optional) LCD / 7-Segment Display  

#### What it does: 
1. Normal traffic light cycle runs automatically.  
2. When button pressed → traffic goes red, pedestrian light green for a set time, then safely back to normal.  
3. Show countdown on Serial Monitor or LCD/7-seg (optional).  

#### Skill Tested:
- Timing and sequencing with delays or millis()  
- Input handling (button press detection with debouncing)  
- Multi-output control (LEDs, optional displays)  

---

### Automatic Plant Watering System 
Keep soil moisture within a healthy range using automatic watering.  

#### Required Components:
- Soil Moisture Sensor (analog)  
- Small DC Water Pump  
- Transistor/MOSFET + Diode (pump driver)  
- (Optional) LCD / Indicator LEDs  

#### What it does: 
1. Reads soil moisture continuously.  
2. If soil is dry → activate pump for max 10s.  
3. Pump won’t restart until at least 30s gap.  
4. Show system status on Serial Monitor, LCD, or LEDs.  

#### Skill Tested:
- Analog sensor reading and thresholding  
- Driving motors/pumps with transistors  
- Implementing cooldown timers  
- Data output to monitor or display  

---

### Security Alarm with Motion Sensor 
Detect intruders using a PIR sensor and trigger an alarm system.  

#### Required Components:
- PIR Motion Sensor  
- Buzzer  
- LED  
- (Optional) Push Button or Keypad  

#### What it does: 
1. Motion detected → buzzer/LED alarm stays on for X seconds.  
2. Optional: reset/disable button or simple passcode.  
3. Bonus: add entry delay (e.g., 5s) before alarm activates.  

#### Skill Tested:
- Digital sensor integration (PIR)  
- Event-driven programming  
- Alarm logic and state management  
- Optional security input handling  

---

## Murad
### RGB LED Controller 
Control a three-channel RGB LED using push buttons.  

#### Required Components:
- RGB LED (common cathode or anode)  
- 3 Push Buttons  
- Resistors  

#### What it does: 
1. Each button controls the R, G, or B portion of the LED.  
2. Button press toggles ON/OFF state of that color channel.  
3. Bonus: adjust brightness using PWM without extra buttons.  

#### Skill Tested:
- Button input logic (toggle states)  
- PWM for brightness control  
- Understanding of RGB color mixing  

---

## Fida
### Distance Visualizer 
Display distance values across multiple LEDs, similar to a car battery indicator.  

#### Required Components:
- Ultrasonic Distance Sensor (HC-SR04)  
- 5–10 LEDs  
- Resistors  

#### What it does: 
1. Reads distance using an ultrasonic sensor.  
2. LEDs light up progressively from one side depending on distance.  
3. Bonus: output actual distance on Serial Monitor or LCD.  

#### Skill Tested:
- Using ultrasonic distance sensors  
- Mapping sensor values to LED patterns  
- Visualization of data in physical form  

---

## Pranto
### Reaction Time Game  
Measure player’s reflexes with LED and button.  

#### Required Components:
- LED  
- Push Button  

#### What it does: 
1. LED lights up after a random delay.  
2. Player presses button as fast as possible.  
3. Arduino calculates and shows reaction time on Serial Monitor.  
4. Bonus: track best or average reaction time across rounds.  

#### Skill Tested:
- Random number generation and delays  
- Measuring elapsed time with millis()  
- User interaction with input and output  

---

### Digital Safe (Mini Lock System)  
A simple password-protected system using keypad or buttons.  

#### Required Components:
- Keypad (4x4) or 3 Push Buttons  
- LED or Servo Motor  
- Buzzer  

#### What it does: 
1. User enters a password (via keypad or push buttons).  
2. Correct password → LED ON (or Servo unlocks).  
3. Wrong password → buzzer alarm.  
4. Bonus: limited attempts or timed lockout feature.  

#### Skill Tested:
- Keypad/button input handling  
- Implementing passcode verification logic  
- Using actuators (LED/servo) as output  
- Security features (lockout, buzzer)  

---

### Smart Street Light with Dimmer  
An automatic street light system with brightness control.  

#### Required Components:
- LDR (Light Dependent Resistor)  
- LED  
- (Optional) Potentiometer  

#### What it does: 
1. LDR detects darkness → LED turns ON automatically.  
2. PWM control: LED brightness increases as it gets darker.  
3. Optional: potentiometer sets threshold or dimming curve.  

#### Skill Tested:
- Analog sensor reading (LDR)  
- Using PWM for dimming  
- Mapping sensor values to light output  

---

## Tasnim

### Smart Dustbin
Automatically opens the lid when a hand or object comes close.

#### Required Components:
- Arduino Uno  
- HC-SR04 Ultrasonic Sensor → detects hand/object  
- SG90 Servo Motor → opens/closes lid  
- Jumper wires  
- (Optional) Cardboard or plastic bin with lid attached to servo  

#### What it does:
1. Detects hand or object approaching the dustbin using ultrasonic sensor.  
2. Opens the lid automatically using the servo motor.  
3. Closes the lid after object moves away.  

#### Skill Tested:
- Ultrasonic sensor integration  
- Servo motor control  
- Event-driven automation  

---

### Fire Alarm System
Detect fire using a flame sensor and trigger an alarm.

#### Required Components:
- Arduino Uno  
- Flame sensor module  
- Buzzer (active buzzer recommended)  
- 1 × LED + 220Ω resistor  
- Jumper wires + breadboard  

#### What it does:
1. Monitors flame sensor for fire detection.  
2. Triggers buzzer and LED alarm when flame detected.  
3. Continues monitoring for safety.  

#### Skill Tested:
- Digital sensor integration  
- Alarm logic  
- Event-driven programming  

---

### Smart Street Light
Automatically turns LEDs on at night and off during the day.

#### Required Components:
- Arduino Uno  
- LDR (Light Dependent Resistor)  
- 10kΩ resistor (for LDR voltage divider)  
- 3 × LEDs (representing street lights)  
- 3 × 220Ω resistors  
- Breadboard + jumper wires  

#### What it does:
1. Measures ambient light using LDR.  
2. Turns LEDs ON when dark and OFF when bright.  
3. Adjusts output automatically based on day/night cycle.  

#### Skill Tested:
- Analog sensor reading (LDR)  
- LED control with digital outputs  
- Threshold-based automation

---

## Earthij
### Visualize y = (sin(x))²  
Use a servo motor to visualize the function `y = (sin(x))²`.  

#### Required Components:
- Servo Motor  

#### What it does: 
1. Generate values of y = sin²(x) as x increases linearly.  
2. Map y from range (0.0 – 1.0) to servo angle (0 – 180).  
3. Continuously move servo to demonstrate the function.  

#### Skill Tested:
- Mathematical function implementation in code  
- Mapping values to servo angles  
- Continuous motion control of a servo  

---

## Sami
### Temperature-Based Fan Controller  
Automatically control a fan based on ambient temperature.  

#### Required Components:
- Arduino UNO  
- Temperature sensor (LM35 or DHT11)  
- Small DC fan or LED (for simulation)  
- Resistor  
- Jumper wires  
- Breadboard  

#### What it does: 
1. Reads temperature from the sensor every 5 seconds.  
2. If temperature exceeds 25°C → turn on fan/LED; below 22°C → turn off to avoid rapid toggling.  
3. Ensures a minimum fan run time of 10 seconds to prevent short cycling.  
4. Displays temperature and fan status on Serial Monitor.  

#### Skill Tested:
- Analog/digital sensor reading  
- Threshold-based control with hysteresis  
- Non-blocking timing  

---

### Light-Activated Door Bell  
Trigger a buzzer when light levels drop, simulating a door opening.  

#### Required Components:
- Arduino UNO  
- Photoresistor (LDR)  
- Buzzer  
- Push button (for manual trigger)  
- Resistors  
- Jumper wires  
- Breadboard  

#### What it does: 
1. Monitors light levels using the photoresistor.  
2. If light level drops below threshold → activate buzzer for 3 seconds.  
3. Prevents retriggering for 10 seconds after activation.  
4. Button press triggers buzzer manually.  
5. Displays light level and status on Serial Monitor.  

#### Skill Tested:
- Analog input processing  
- Event-driven logic  
- Debouncing and cooldown timing  

---

### Color-Detecting Night Light  
Adjust brightness based on ambient light and detect a specific color.  

#### Required Components:
- Arduino UNO  
- Photoresistor (LDR)  
- RGB LED  
- Color sensor (TCS34725 or LDR-based setup)  
- Resistors  
- Jumper wires  
- Breadboard  

#### What it does: 
1. Measures ambient light using photoresistor and adjusts RGB LED brightness (dim in dark, brighter in light).  
2. Detects a specific color (e.g., red) → flashes RGB LED in that color for 5 seconds.  
3. Updates every 3 seconds to avoid rapid brightness changes.  
4. Displays ambient light level and detected color on Serial Monitor.  

#### Skill Tested:
- Sensor integration  
- PWM control  
- Conditional logic  
- Data smoothing  

---

## Tanmay

### Arduino Lie Detector
Measure pulse to indicate stress or high heart rate.

#### Required Components:
- Arduino Uno  
- Pulse Sensor  
- Buzzer or LED  
- Resistor  

#### What it does:
1. Reads heartbeat using pulse sensor attached to finger.  
2. Detects stress or elevated pulse.  
3. Triggers buzzer or LED as indication.  

#### Skill Tested:
- Analog sensor reading  
- Event-driven output  
- Interactive project demo  

---

### Reaction Time Game
Measure player’s reflex speed with LED and button.

#### Required Components:
- Arduino Uno  
- Push Button  
- LED  
- Resistor  

#### What it does:
1. LED lights up after a random delay.  
2. Player presses button as fast as possible.  
3. Arduino calculates and shows reaction time on Serial Monitor.  

#### Skill Tested:
- Random number generation and delays  
- Measuring elapsed time with millis()  
- User interaction with input and output  

---

### Digital Dice with LEDs
Simulate dice rolls using LEDs.

#### Required Components:
- Arduino Uno  
- Push Button  
- 7 LEDs  
- Resistors  

#### What it does:
1. Player presses button to roll the dice.  
2. Arduino generates a random number from 1–6.  
3. Displays the number using LED pattern like a digital dice.  

#### Skill Tested:
- Random number generation  
- LED pattern mapping  
- Button input handling

---

## Jubaer

### Musical Water Glasses (Touch Piano)
Use water-filled glasses as touch sensors to play musical notes.

#### Required Components:
- Arduino Uno  
- 4–6 jumper wires with foil/tape on water glasses  
- Piezo buzzer  
- Resistor  

#### What it does:
1. Each water glass acts as a capacitive touch sensor.  
2. Touching a glass triggers Arduino to play a corresponding buzzer tone.  
3. Multiple glasses produce different musical notes, forming a mini piano.  

#### Skill Tested:
- Capacitive touch sensing  
- Piezo buzzer control  
- Interactive sound project  

---

### Reaction Time Game with LEDs
Measure player’s reflex using LEDs and a button.

#### Required Components:
- Arduino Uno  
- 3 LEDs  
- 1 Push Button  
- Resistors  

#### What it does:
1. LED lights up after a random delay.  
2. Player presses button as fast as possible.  
3. Arduino measures reaction time and displays result via LED blink speed.  

#### Skill Tested:
- Random number generation  
- Timing measurement  
- Interactive input/output  

---

### Magic Dice (LED Dice Roller)
Simulate dice rolls using LEDs.

#### Required Components:
- Arduino Uno  
- 7 LEDs  
- Push Button (or tilt sensor)  
- Resistors  

#### What it does:
1. Player presses button (or shakes tilt sensor) to roll dice.  
2. Arduino generates a random number 1–6.  
3. Displays number on LEDs arranged like a real dice.  

#### Skill Tested:
- Random number generation  
- LED pattern mapping  
- Button/tilt input handling

---

## Rabiul

### Sun Tracker
Automatically rotate a panel or LED to face the sun.

#### Required Components:
- Arduino  
- 2 LDRs  
- Servo motor  
- LED or solar panel  
- Jumper wires  

#### What it does:
1. Reads light intensity from two LDRs.  
2. Compares values and rotates servo toward brighter side.  
3. Keeps panel or LED aligned with sunlight.  
4. Optional: 2D tracking using 4 LDRs.  

#### Skill Tested:
- Analog sensor reading  
- Servo motor control  
- Basic tracking algorithm  

---

### Fire Detector
Detect fire or high temperature and trigger an alert.

#### Required Components:
- Arduino  
- Flame or Temperature sensor  
- Buzzer  
- LED (optional)  
- Jumper wires  

#### What it does:
1. Sensor detects fire or high temperature.  
2. Triggers buzzer and optional LED alarm.  
3. Displays “Fire Detected” on Serial Monitor.  
4. Optional: Add delay for buzzer or reset button.  

#### Skill Tested:
- Digital sensor integration  
- Event-driven alerts  
- Optional Serial output  

---

### Rain Detector
Detect rain or water drops and trigger an alert.

#### Required Components:
- Arduino  
- Rain sensor  
- Buzzer  
- LED (optional)  
- Jumper wires  

#### What it does:
1. Sensor detects water/rain drops.  
2. Triggers buzzer and optional LED.  
3. Shows rain detection on Serial Monitor.  
4. Optional: Adjustable sensitivity or count display.  

#### Skill Tested:
- Digital sensor input  
- Event-driven alerts  
- Optional data monitoring

---

## Junayed

### LED Traffic Light Simulator
Simulate a traffic signal using LEDs.

#### Required Components:
- Arduino Uno  
- Red, Yellow, Green LEDs  
- Resistors  
- Jumper wires  
- Breadboard  

#### What it does:
1. Lights up LEDs in sequence to simulate a traffic light cycle.  
2. Red, yellow, and green LEDs represent stop, caution, and go.  
3. Can be programmed with adjustable timing for each light.  

#### Skill Tested:
- Digital output control  
- Timing and sequencing  
- LED circuit design  

#### Example Tinkercad Circuit:
- [Traffic Lights Simulation](https://www.tinkercad.com/things/hiB8R24vpwj-traffic-lights-simulation)  

---

### Ultrasonic Distance Measurement
Measure distance using an ultrasonic sensor and indicate with an LED.

#### Required Components:
- Arduino Uno  
- HC-SR04 Ultrasonic Sensor  
- LED  
- Resistor  
- Jumper wires  
- Breadboard  

#### What it does:
1. Measures distance using ultrasonic sensor (Trig and Echo pins).  
2. Lights up LED if an object is within a specified range.  
3. Displays distance on Serial Monitor optionally.  

#### Skill Tested:
- Ultrasonic sensor integration  
- Conditional LED control  
- Distance measurement  

#### Example Tinkercad Circuit:
- [Ultrasonic Distance Sensor Tutorial](https://www.learnelectronicsindia.com/post/building-an-ultrasonic-distance-sensor-using-tinkercad)  

---

### Simple Temperature Monitor with LED Indicator
Monitor temperature and indicate ranges using LEDs.

#### Required Components:
- Arduino Uno  
- LM35 Temperature Sensor  
- LEDs for range indication  
- Resistors  
- Jumper wires  
- Breadboard  

#### What it does:
1. Reads temperature from LM35 sensor.  
2. Turns on specific LEDs based on temperature thresholds.  
3. Optionally display temperature on Serial Monitor.  

#### Skill Tested:
- Analog sensor reading  
- Threshold-based LED control  
- Serial data output  

#### Example Tinkercad Circuit:
- [Temperature Sensor LED Indicator](https://www.tinkercad.com/things/8m9oHsrbr3U-temperature-sensor-led-indicator)

---

## Jahid

### Arduino Temperature and Humidity Monitor with Real-Time Display
Monitor temperature and humidity and display readings on an LCD.

#### Required Components:
- Arduino board (Uno, Nano, etc.)  
- DHT11 or DHT22 sensor  
- 16x2 LCD with I2C module  
- Jumper wires  
- Breadboard (optional)  

#### What it does:
1. Reads temperature and humidity using DHT sensor.  
2. Arduino processes sensor data.  
3. Displays readings in real-time on the LCD.  

#### Skill Tested:
- Digital sensor integration  
- LCD display interfacing  
- Real-time data monitoring  

#### Tutorial:
- [Temperature and Humidity Monitor](https://shorturl.at/DalNc)  

---

### Arduino Stopwatch with Start, Stop, and Reset Functions
Measure elapsed time with button controls and display on LCD or Serial Monitor.

#### Required Components:
- Arduino board (Uno, Nano, etc.)  
- 16x2 LCD with I2C module (or Serial Monitor)  
- 3 Push Buttons (Start, Stop, Reset)  
- 10kΩ resistors for pull-downs  
- Jumper wires  
- Breadboard  

#### What it does:
1. Buttons control stopwatch operations (start, stop, reset).  
2. Arduino tracks time using `millis()`.  
3. Displays elapsed time on LCD or Serial Monitor.  

#### Skill Tested:
- Timing using Arduino  
- Button input handling  
- Displaying data on LCD or Serial Monitor  

#### Tutorial:
- [Arduino Stopwatch](https://shorturl.at/Qyr19)  

---

### Arduino-Based Digital Voltmeter
Measure voltage and display it digitally on an LCD.

#### Required Components:
- Arduino board (Uno, Nano, etc.)  
- 16x2 LCD with I2C module  
- Voltage divider resistors (e.g., 100kΩ and 10kΩ)  
- Jumper wires  
- Breadboard  
- Wires for voltage input  

#### What it does:
1. Reads analog voltage via A0 pin, scaled using voltage divider.  
2. Calculates actual voltage value.  
3. Displays voltage in volts on the LCD.  

#### Skill Tested:
- Analog voltage measurement  
- Voltage divider calculation  
- LCD display integration  

#### Tutorial:
- [Digital Voltmeter](https://rb.gy/yut20g)

---

## Abir

### Reaction-Based “Whack-a-LED” Game
A mini reaction game using LEDs and buttons.

#### Required Components:
- Arduino Uno  
- 4 LEDs  
- 4 Push Buttons  
- Resistors  
- Jumper wires  
- Breadboard  

#### What it does:
1. Arduino randomly turns on one LED.  
2. Player must press the matching button as quickly as possible.  
3. Correct press → score +1; Wrong press → score -1.  
4. Game runs for 30 seconds and prints final score on Serial Monitor.  

#### Skill Tested:
- Random selection and timing with `millis()`  
- Button input handling  
- Game-like logic and scoring  

---

### Electronic Dice with 7-Segment Display
Simulate dice rolls using a 7-segment display.

#### Required Components:
- Arduino Uno  
- Push Button  
- Common Cathode 7-Segment Display  
- Resistors  
- Jumper wires  
- Breadboard  

#### What it does:
1. Press button to start dice roll simulation.  
2. Arduino cycles random numbers 1–6 for ~1 second.  
3. Stops at a random number and displays it on 7-segment display.  

#### Skill Tested:
- 7-segment display decoding  
- Random number generation  
- Timing control and circuit wiring  

---

### Morse Code Messenger
Blink a predefined message in Morse code.

#### Required Components:
- Arduino Uno  
- Push Button  
- LED  
- Resistor  
- Jumper wires  
- Breadboard  

#### What it does:
1. Stores a predefined message (e.g., “HELLO”) in Arduino.  
2. When button pressed → LED blinks message in Morse code.  
3. Stops after message, ready to replay on next button press.  

#### Skill Tested:
- Mapping letters to Morse patterns  
- Timing short/long blinks  
- Button-triggered event handling  
- Structured programming

---

## Maisha

### Smart Home Automation System
Control home appliances via Bluetooth.

#### Required Components:
- Arduino Uno  
- HC-05 Bluetooth Module  
- Mobile app for control  
- Relay module (for light/fan)  
- Jumper wires  
- Breadboard  

#### What it does:
1. Connects Arduino to smartphone via Bluetooth.  
2. Control light, fan, or other appliances using mobile app.  
3. Relay switches devices ON/OFF as per commands.  

#### Skill Tested:
- Bluetooth communication with Arduino  
- Relay control for appliances  
- Mobile app integration  

---

### Earthquake Detector
Detect vibrations and trigger an alarm.

#### Required Components:
- Arduino Uno  
- Accelerometer (ADXL335 / MPU6050)  
- Buzzer  
- LED (optional)  
- Jumper wires  
- Breadboard  

#### What it does:
1. Accelerometer detects sudden vibrations or shakes.  
2. Arduino processes data to identify abnormal movement.  
3. Triggers buzzer/LED alarm when threshold exceeded.  

#### Skill Tested:
- Sensor integration (accelerometer)  
- Vibration detection logic  
- Alarm triggering  

---

### Smart Dustbin
An automatic dustbin that opens and alerts when full.

#### Required Components:
- Arduino Uno  
- Ultrasonic Sensor (HC-SR04)  
- Servo Motor (for lid control)  
- Buzzer

---






## Judging Criteria
- **Functionality (40%)** → Does it perform as described?  
- **Circuit Correctness (20%)** → Proper wiring, safe driver use (transistor/relay).  
- **Code Quality (20%)** → Clear logic, good comments, avoid unnecessary blocking delays.  
- **Presentation (10%)** → Tinkercad schematic or demo video.  
- **Creativity (10%)** → Bonus features (LCD output, buzzer tones, calibration, etc.).