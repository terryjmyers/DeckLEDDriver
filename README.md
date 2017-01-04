# DeckLEDDriver

High Amp RGB LED Driver with pot with webpage control.  This project was developed for my house, specfically for my backyard deck, secifically to my requirements.

Functional Requirements:

  1. Control a string of common anode RGB LEDs mounted underneath the railings of a deck via a dimmer switch mounted nicely inside the house, or via a  webpage
  2. Control a string of white LEDs mounted underneath a deck via a dimmer switch or motion detection.

Detailed Design

1. A server grade 12VDC nom. power supply supplies a rail of 12V to the arduino which has an ethernet shield and some protoshields for wiring
2. This power supply has a 2A 12V rail that is always on and an input that when sinked to ground turns on a 60A rail which will power the LEDs
3. The arduino can control this input via a transistor.  A SPDT with off toggle switch on the case allows the user to enable the arduino to control the power, or turn power on, or off.
4. The arduino hosts a basic webpage that allows the user to control the lighting on the Deck, and allows access to some basic "cool" programs
5. If the dimmer switch is moved, the RGB Deck LEDs are dimmed white according to the dimmer position.  The dimmer always makes the RGB white
  1. The RGB LEDs are color corrected to 3000K by capping the max value of the green and blue LED
  2. Dimmer switches are a 2-wire POT(without the ground connection), becuase I bought the wrong ones, therefore they are linearlized in software "good enough"
  3. LED perceived brightness is nto linear with power, there he overall LED brightness is linearized to human perception with a gamma correction algorythm.
  4. Both dimmer switch analog inputs are heavily smoothed with a low pass filter to prevent "flickering"
6. The Under Deck LEDs are just white LEDs and can only be controlled via an identical linearized dimmer switch
  1. Again the dimmer switch is linearized and the brightness is linearized to human perception.
  2. Generic 12V powered motion sensors (3 wire: Power, ground, relay switched 12V) are used to sense motion.
  3. The relay switched signal is passed through an LM7805 as a kind of "relay" of sorts as an input to the arduino.  An actual mechical relay should be used, but I didn't have one at the time of build.
  4. If motion is sensed the under deck LEDs will fade on for a preprogrammed time, then fade off.
  time.gov is polled once a day and an acurate time is maintained in a register.
  5. he motion sensor inputs are disabled during daylight according to monthly averages of civil twilight hours around Philadelphia PA
7. All LEDs have a timeout timer if left on for more than 3 hours they will turn off
8. All LEDs will turn off at midnigth on a one shot.


TODO:

This was my first BIG project and therefore there are a lot of innefficiencies that I'd like to clean up.  

1. I've combined the pulse timers into a proper struct in my standard template.  I would use this library instead
2. Store the webpage on the SD card instead of storing it in Flash
3. Change debug logic to #IFDEF to save memory.
4.  Change UpdateScanCounter to #IFDEF logic to save memory
