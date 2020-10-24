# Tiny Reflow Controller

An all-in-one Arduino compatible reflow controller powered by ATmega328P (V2) or ATtiny1634R (V1). A reincarnation of the Reflow Oven Controller Shield that requires an external Arduino board like Arduino Uno based on user feedbacks over the years. Powered by the ATmega328P/ATtiny1634R coupled with the latest thermocouple sensor interface IC MAX31856 from Maxim, we managed to remove the need of an Arduino board and reduce the overall cost. We also use as much SMD parts in this revision to keep the cost low (manual soldering and left over residue cleaning is time consuming) and leaving only the terminal block and the LCD connector on through hole version. We also managed to streamline all components to run on 3.3V to further simplify the design. 

All you need is an external Solid State Relay (SSR) (rated accordingly to your oven), K type thermocouple (we recommend those with fiber glass or steel jacket), and an oven of course! 

You can now select to run a lead-free profile or leaded profile from the selection switch. 

V2 comes with 0.96" 128*64 OLED LCD to plot the real-time reflow curve and has a built-in serial-USB interface. V2 also has an optional transistor output drive fan if needed. 

## Building

Use VS Code with platform IO. Set the version you want to build in platform.ini

## Profile Configration

 - Each Profile has 4 stages: pre-heat, soak, reflow and cooldown
 - Each Stage has:
    - Max Temp: The Max Temperature for this stage in degree C. Goal Temp.
    - Micro Time: Milliseconds until the next temperature setpoint increase. (set to 0 for max heating speed)
    - Heat Step: Degree C step size for new setpoint (set to "Max Temp" - "previous stage max" (TEMPERATURE_ROOM for first stage) for max heating speed)
    - Hold Micro Time: Milliseconds to hold at max temp once it is reached (set to 0 to skip)
    - PID KP: 
    - PID KI: 
    - PID KD: 


## Profiles

Profiles can be configured in src/main.cpp starting on line 108.

### Leaded Reflow Curve (Kester EP256)
```
  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)
```

### Lead-Free Reflow Curve
```
  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)
```

### PLA Annealing
```
  Temperature (Degree Celcius)
      |                                     
  63 -|               x   x   x   x   x   x    
      |             x |                   |   x
      |           x   |                   |       x  
      |         x     |                   |           x
      |       x       |                   |               x
      |     x         |                   |                   x
      |   x           |                   |                        x
  30 -| x             |                   |                          |
      |<   4 hours   >|<      30 min     >|<      3 hours           >|
      | Preheat Stage |   Soaking Stage   |          Cool            |
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)
```

### PETG Annealing
```
  Temperature (Degree Celcius)
      |                                     
  177-|               x   x   x   x   x   x    
      |             x |                   |   x
      |           x   |                   |       x  
      |         x     |                   |           x
      |       x       |                   |               x
      |     x         |                   |                   x
      |   x           |                   |                        x
  30 -| x             |                   |                          |
      |<   4 hours   >|<      30 min     >|<      3 hours           >|
      | Preheat Stage |   Soaking Stage   |          Cool            |
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)
```
