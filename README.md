# Naturally Augmented Guitar
## Naookie Sato

### Overview

The augmented guitar is an electric guitar which uses sensors in combination with software processing to modulate sound frequencies. The guitar itself may or may not be used as the initial sonic source. In the latter case it can also function as various controllers, such as a midi controller. I will be creating the Naturally Augmented Guitar.

The issue I find with many augmented instruments, especially the augmented guitar, is that implementations generally require a completely different approach to physically playing the instrument. I would like my augmented instrument to fulfill two criteria. The first is that the instrument should remain fully functional as a non-augmented instrument. Second, The augmentations should happen naturally as a consequence of playing the instrument, not by methods which are unrelated to the canonical ways of playing the instrument.

For example, if I create an augmented drum kit then perhaps any modulations in software should arise from natural artifacts of drumming, such as the force of each hit. This is preferred in my model to some external methods such as the level of light in the room or the temperature of the environment in which the drums exist.

Taking Enda Bates' guitar as a concrete example, his augmented guitar is not at all natural. First, it is not in any standard tuning, rendering it useless for 'normal' use. His performance modulations come from the spatial location of the right hand and orientations of the guitar body. Both of these things he modulates after actually strumming, which is why I define them to be external to any natural guitar playing gesture.

My augmented guitar is going to perform software modulations based primarily on the natural aspects of guitar playing. 

### Sensors Used

A 1.5 in x 1.5 in square force sensitive resistor (FSR) will be placed on the floor and controlled with my foot. Another 0.5 in diameter circular FSR will be attached to the guitar pick. A final fsr is mounted along the back of the of the neck underneath the 400 mm potentiometer strip. Its data may or may not be used in practice due to an issue later described.

Three soft potentiometer strips will be attached to the back of the neck of the guitar. Two of them will be 200 mm and one will be 400 mm. When constructing, the 400 mm strip may be a "HotPot" instead of a soft potentiometer, depending on sensor availability. They both should be functionally equivalent for this instrument. Down scaling the data on the strips will be necesessary for positional accuracy due to the electrical properties of membrane potentiometers. Pressing two points on one strip will give the same value as pressing on some other single point on that same strip.

A six-axis MPU6050 gyroscope/accelerometer will be attached to the head of the guitar. Its six raw values will be scaled to the midi range on the board then sent to max.

### Performance Gestures Required

I want my instrument to focus on augmenting sounds based on natural performance gestures and style of the guitarist playing it. Therefor, there are no special performance gestures required, aside from use of the hardware.

The FSR on the back of the neck will measure the pressure with which the frets are pressed. The FSR on the guitar pick will measure the amount of pressure applied to the guitar pick by the right hand. The fsr on the floor will be used as an arbitrary foot controller.

Soft potentiometer strips on the back of the neck will be used to measure position of the fretting hand on the guitar.

The MPU6050 on the head of the guitar will technically measure the guitar's acceleration and rotation, not an actual performance gesture. However, some gestures, such as vibrato, can be derived from this data. I plan on mapping this to effects such as pan, wah, or pitch bend.


### Artistic References

Guitar is an instrument I played heavily for about ten years, from ages eight to eighteen. It is the closest musical extension of myself. By creating an instrument which responds naturally to the performance gestures of the performer, I feel that I can use the guitar as a platform to gain a higher level of expressiveness than if I were to augment some other instrument. 

I chose to implement a controller in addition because I felt like controlling foot switches is a quite natural aspect of guitar playing, especially in a live environment.

### Sound Outcome

The goal of the Naturally Augmented Guitar is simply to generate data in accordance with the natural performance gestures of guitar playing. Thus, there are no strict sound outcomes associated with the instrument.

One example of a sound outcome could be to use the yaw, pitch, and roll values to control the envelope of the guitar signal itself. I could map yaw to attack, pitch to decay, and roll to release. 

The FSR on the floor might map to some time/phase effect commonly found with guitar such as chorus, delay, or reverb. The other FSR on the pick will possibly be mapped to the distortion level of the signal. Alternatively, the floor fsr could modulate the instruments controlled by the soft potentiometers. If I do choose to use the data from the long fsr, it will probably be mapped to some parameter which favors random data.

Specifically in my final performance, the mappings will be as follows:

    Pitch of the MPU6050 mapped to High Cut Frequency of Intro intrument in Logic.
    Roll of the MPU6050 mapped to Pan of Intro intrument in Logic.

    One softpot strip mapped to Sync 2 parameter of lead instrument
    Other softpot strip mapped to the digital level parameter of the Tidal Space instrument.
    Hotpot strip mapped to drive level of one of the bass instruments.
    
    FSR on guitar pick mapped to distortion level of live guitar signal
    FSR on floor mapped to phaser rate of live guitar signal
    
    

### Issues and Revisions

When testing, everything worked properly. I was recieving clean data from all sensors connected to the ESP32 through a breadboard. All sensors were producing mostly-linear data along a full 12-bit range. After wiring everything to the guitar and attaching the sensors with permanent adhesive, one of the 200 mm soft pot strips had a momentary short resulting in some permanent damage. It now intermittently produces random low data (around 600/4095) when no pressure is applied.

There is another issue in the 400 mm strip. It is no longer producing linear data. It seems to be more exponential instead. For example, pressing at 200 mm out of 400 mm produces a 12-bit value of around 1024 out of 4095 when it should be closer to 2048.

Finally, the long fsr essentially does not work for any useful sensing. When initially testing the hot potentiometer strip on top of the fsr, both produced clean data. After adhering the potentiometer strip to the fsr, the fsr constantly senses slightly-random, almost-maximal pressure. I wired a 4.7k ohm resistor in line to reduce current load.

The guitar in its current state is not extremely resistant. I originally wanted everything to fit in the original form factor of the guitar. Do to some unforseen complications in building the guitar, that did not happen. Many things are hanging off the guitar, and most of the circuitry is covered with electrical tape. This is in case modification or repairs are needed.

I had taken the time to lay stips of copper tape as wiring down the neck of the guitar to replace the bundle of wires currently present. What I did not expect was for the neck of the guitar itself to be slightly conductive. In a future revision I might first cover the neck in electrical tape, or some other non-conductive layer. Pictures of my previous work included.

### Links
 
Square FSR

https://www.amazon.com/gp/product/B00B887DBC/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1 

Circular FSR

https://www.amazon.com/SENSING-RESISTOR-CIRCLE-1oz-22LB-FLEXIBLE/dp/B00B887CLS/ref=sr_1_3?s=industrial&ie=UTF8&qid=1524955596&sr=1-3&keywords=0.5+in+fsr

Extra long FSR

https://www.adafruit.com/product/1071

400 mm HotPot

https://www.amazon.com/SPECTRA-SYMBOLLINEAR-HOTPOT-400-MM/dp/B00FPXIBCQ/ref=sr_1_1?ie=UTF8&qid=1527091329&sr=8-1&keywords=400+mm+hotpot

200 mm SoftPot

https://www.amazon.com/SPECTRA-SYMBOL-SoftPot-Membrane-Potentiometer/dp/B004G4XUT4/ref=pd_sbs_328_1?_encoding=UTF8&pd_rd_i=B004G4XUT4&pd_rd_r=B74A8996831R26MG8PDR&pd_rd_w=vE1AN&pd_rd_wg=yHmvE&psc=1&refRID=B74A8996831R26MG8PDR

MPU 6050

https://www.amazon.com/gp/product/B008BOPN40/ref=oh_aui_detailpage_o04_s00?ie=UTF8&psc=1 

Enda Bates

http://endabates.net/EndaBates-AugmentedGuitar.html

