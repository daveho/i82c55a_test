# Interfacing Arduino to 82C55A

This project is a test of interfacing an Arduino to an 82C55A parallel I/O chip.  I used an Arduino Nano clone.

There's not really any practical reason to do this: I just wanted to make sure how to use the 82C55A before using it in a different project.

The connections between the Arduino pins and the 82C55A pins are described in code comments.  The demo code puts all ports in mode 0 (basic I/O), configures ports A and C for output, and port B for input.  My test circuit has LEDs connected to port A and DIP switches as inputs to port B.
