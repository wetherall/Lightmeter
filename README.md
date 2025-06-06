# Lightmeter

Interface behaviour

The light meter will have two columns of twelve LEDs. one column is used for setting and displaying aperture and iso, the other column is used for displaying the calculated shutter speed based on measured lux and the iso/aperture settings. The PCB will have 4 buttons. One button for waking the device and taking a light reading, a button to toggle between iso and aperture setting, and up/down buttons for changing the aperture or iso setting. The dual function settings column will be labelled (ascending) 25,50,100,125,160,200,250,400,500,800,1600,3200 for ISO and 1,1.4,2,2.8,4,5.6,8,11,16,22,32,45 for fstop. The shutter speed display column will be labelled with 1,2,4,8,15,30,60,125,250,500,1000,2000.


Displaying shutter speed

The shutter speed readout LEDs are labelled 1,2,4,8,15,30,60,125,250,500,1000,2000. These are by default fractions of a second. But if the appropriate shutter speed is calculated as being over a second, the same LEDs can be used, but the numbers represent full seconds instead of fraction of one second. To indicated to the user this is the case, the the LEDs should blink when they represent full seconds. Also, the calculated appropriate shutter speed will rarely be exactly one of the options we can display, in these cases the closest shutter speed LED should be the one that illuminates. We can use individual LED brightness control to convey more detailed information to the user. If the true calculated shutter speed is more than 25% of the way from the nearest shutter speed to the next shutter speed, the next shutter speed LED should also illuminate, but at half the brightness of the primary LED. For example. If the calculated shutter speed is 1/1400, we can not show this directly, we would illuminate the 1/1000 LED as it is the closest. However it is more than 25% of the way from 1/1000 and 1/2000, so the 1/2000 LED should illuminate at half the brightness. In the case of shutter speeds of more than 1 second, the numbers are treated as whole seconds and not fractions. To indicate this change the LED for the nearest value will blink at 2hz. If the true calculated shutter speed was more than 25% of the way to the next one in these cases, the next LED will illuminate at half brightness, also blinking at 2hz. Individual brightness control is achieved using 4-bit PWM in software by the MCU.


Aperture selection behaviour

The aperture selection values adjacent to the LEDs are 1,1.4,2,2.8,4,5.6,8,11,16,22,32,45. However, lenses commonly allow the incrementing of apertures in halves. Therefore we must allow the selection of these intermediate values. These in between values should be selectable and indicated by the illumination of the two LEDs flanking the intermediate f-stop. For example, if the user navigated to f5.6, only the f5.6 LED would illuminate. If they pressed the up button once to change up an aperture value, we would increment by 0.5 fstops, to f6.7 instead of all the way to f8. f6.7 would be indicated by illumination of both flanking LEDs, 5.6 and 8. If the user pressed the up button again, f8 would be selected, and only the f8 LED would illuminate. This should function for all half stops between f64 and f1. We can think of this as the real list of available f stops as being 1,1.2,1.4,1.7,2,2.4,2.8,3.3,4,4.8,5.6,6.7,8,9.5,11,13,16,19,22,27,32,38,45. But we only have display LEDs for every other fstop, hence the use of LED pairs to indicate when selecting the value in between.


Software behaviour

Pressing the read button will wake the device. The battery level will be checked. If critically low the device will go back to sleep. If low but not critical, the bottom two LEDs will flash but normal function resumed after (This battery check is repeaetd every 30 seconds teh device is on). The previously set ISO and aperture setting will be used. If no previous setting exists, it will default to 400iso and f11. A light reading will be taken from the light meter IC, in software automatically adjusting gain and read time as appropriate. Once an appropriate shutter speed is calculated for the light level of the scene, using the iso and aperture settings, the LED next to the nearest correct shutter speed should illuminate. The settings column will display the selected aperture all the time while the device is awake, apart from when a reading is being taken. The up and down buttons can be used to change the aperture setting (with the LEDs reflecting this change) and the shutter speed column LEDs should light up to reflect this adjustment too. Pressing the mode button should switch the settings LED column from displaying aperture to displaying iso. The iso display LED should flash slowly to indicate that it is showing iso not fstop. The up and down buttons now change the iso. Pressing the mode button again should switch it back to fstop. 


Components

Component choice considerations were low price, minimal power draw for a battery powered device, availablility from places like LCSC, and abundance of example implementations. The PCB will be controlled by an ATtiny3216. The device will use a VEML7700 for lux measurement, which has a a large range of sensitivity. The four buttons will be small low profile SMD tactile buttons. The PCB would be driven by an LIR2450, which will be managed by an STNS01, which also provides 3.1V. The LEDs will be driven by the MCU via a 4 column 6 row matrix.  In addition to the above i will need all the appropriate passive components to support the function of the various ICs described.

Pins

// System Pins
PA0: UPDI Programming (reserved)
PB0: I2C SDA for VEML7700 (TWI)
PB1: I2C SCL for VEML7700 (TWI)
PC2: Battery Level ADC Input

// Button Pins
PA1: Mode Button
PA2: Up Button (swapped with Read)
PA3: Read/Wake Button (swapped with Up)
PA4: Down Button

// LED Matrix Rows (anodes) - 6 pins
PA5: Row 1
PA6: Row 2
PA7: Row 3
PC0: Row 4
PC1: Row 5
PC3: Row 6

// LED Matrix Columns (cathodes) - 4 pins
PB2: Column 1 (Top Left Half - ISO/Aperture 0-5)
PB3: Column 2 (Bottom Left Half - ISO/Aperture 6-11)
PB4: Column 3 (Top Right Half - Shutter Speed 0-5)
PB5: Column 4 (Bottom Right Half - Shutter Speed 6-11)
