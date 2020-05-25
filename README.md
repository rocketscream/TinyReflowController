# Tiny Reflow Controller
An all-in-one Arduino compatible reflow controller powered by ATmega328P (V2) or ATtiny1634R (V1). A reincarnation of the Reflow Oven Controller Shield that requires an external Arduino board like Arduino Uno based on user feedbacks over the years. Powered by the ATmega328P/ATtiny1634R coupled with the latest thermocouple sensor interface IC MAX31856 from Maxim, we managed to remove the need of an Arduino board and reduce the overall cost. We also use as much SMD parts in this revision to keep the cost low (manual soldering and left over residue cleaning is time consuming) and leaving only the terminal block and the LCD connector on through hole version. We also managed to streamline all components to run on 3.3V to further simplify the design. All you need is an external Solid State Relay (SSR) (rated accordingly to your oven), K type thermocouple (we recommend those with fiber glass or steel jacket), and an oven of course! You can now select to run a lead-free profile or leaded profile from the selection switch. V2 comes with 0.96" 128*64 OLED LCD to plot the real-time reflow curve and has a built-in serial-USB interface. V2 also has an optional transistor output drive fan if needed. 

Unofficial Changes
------------------
Below describes most of the substantial changes in this unofficial firmware
version.  All were only tested on a V2 controller, and many only apply to a V2
controller.  Code changes were kept to a minimum, and new code structure,
style, naming, etc. attempted to mimic the original code.

The fan can be toggled by pressing switch #2 when the oven is in any reflow
state other than idle or error.  The animated fan icon in the upper right
corner notifies you that the fan is enabled.  The fan automatically turns off
when it returns to the idle state.

Manual temperature mode can be toggled by pressing both switch #1 and #2
simultaneously ("Manual" displayed on screen, with the selected function and
setpoint shown to the right).  This can be entered from any state (other than
error), and the temperature set point will set to the current temperature
rounded down to the nearest 5 degrees C (or clipped to the min or max of 50C or
250C).  Press switch #1 to change function and switch #2 to change value.  The
functions are setpoint+25 (++), setpoint+5 (+), setpoint-5 (-), setpoint-25
(--), and fan (F). The controller will return to its previous state when manual
mode is exited.  This allows it to be used standalone for baking parts, or
manually adjusting the reflow profile on the fly.

The temperature plot now shows hash marks on the Y axis at 50C, 150C, and 250C,
as well as every minute on the X axis.  The plot also continuously scrolls as
time goes beyond the screen width.

A soft power off and on function is available by pressing and holding switch #1.
The unit can be turned off by holding the switch for 3 seconds from any state.
The unit can be turned back on by holding the switch for 2 seconds.  Turning
the unit off shuts everything down, resets the state, and sits in a loop waiting
for the power on button press, though it is still safest and recommended to
unplug the oven when done.  When the unit is powered back on, the software is
restarted to ensure a fresh state.
