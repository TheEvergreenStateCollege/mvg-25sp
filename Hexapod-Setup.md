# Hexapod Robot Hardware Setup

The Hexapod Robot we use in Multiple View Geometry has a standard setup used to prepare it each time
for programming in this class. It can take up to 30 minutes at the beginning of each class, and up to
15 minutes to do the cleanup at the end, so we document it here for easy reference.

Common abbreviations and terms we include in the glossary.

## Setup Procedure

The robot can be set up either in *autonomous* mode, meaning wireless and battery power setup in the field, or
in *bench* mode.

### Field (Autonomous) Mode

### Bench Mode

In bench mode, the robot's servos and Raspi are powered from the power supply and the bench main supply
rather than using the batteries. This lets us undertake longer, intensive, and more experimental work
on the robot while saving the batteries for later field work, or while the batteries are charging.

#### Battery and Power Setup

(Expected time: 30 minutes)

Remove any batteries from the holder and start charging them so they are fully ready for the
next time we need them in autonomous mode (which could be right after this bench mode).

The bench power supply lets us simulate one isolated bank of batteries of the robot in
autonomous mode.

To simulate two banks of servo's, two bench supplies are needed.

[Note: A hardware improvement to the spideshield board is needed to separate the power pins from the appropriate battery banks from
the two servo sides, and the Raspi]

To simulate the Raspi, which is realistically always running, a third bench supply is needed.

Plug in the bench power supply into the wall supply.

The bench power supply has both Voltage and Current knobs.

Set the bench power supply to "Constant Voltage Mode".

Dial with the coarse grain volage knobs
to about 7.4 V (the nominal voltage of two batteries in series).

Dial the course grain current knobs to about 1.0 Amps (abbreviate A),
which is what the Raspi would need 

## Cleanup Procedure

(Expected time: 15 minutes)

This procedure is mostly the reverse of the bench mode and can be done slightly quicker.

However, it's still important to leave enough time for this at the end of each work session to
allow others to work on the robot effectively the next time, and to respect the lab classroom for
other students, teachers, and staff who use it.

## Glossary

* Bench mode - operating the Raspberry Pi
* Field mode - or autonomous mode, operating the robot outside the classroom on battery power, with wireless internet.
* Raspi - short for Raspberry Pi, the single board computer that controls the robot.
