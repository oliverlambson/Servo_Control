# Bare-metal code for an ARM microcontroller to control servo motors using PWM

Used to automate a process of reliably "picking" an electric guitar string to allow for repeatable experiments.

This formed part of my BEng dissertation where I investigated [the effects of a magnetic pickup on the vibration response of an electric guitar string](https://www.academia.edu/42961860/The_Effects_of_a_Magnetic_Pickup_on_the_Vibration_Response_of_an_Electric_Guitar_String) (2018).

I also separately developed an optical sensor to determine the position of a point along the guitar string in 2 dimensions (see the dissertation for details). I have noticed that a similar (albeit, 1-dimensional) sensor is being brought to market by [oPik](https://www.kickstarter.com/projects/light4sound/opikthe-optical-guitar-pickup).

The findings showed that the magnetic pickups used in electric guitars have the potential to significantly effect the otherwise "free" vibration of the string. [Other research](https://asa.scitation.org/doi/full/10.1121/1.5080465)<sup>1</sup> has since been published that sheds more light on what may be going on.

---

Software tools:
- Atollic TrueSTUDIO (for embedded C project)
- CubeMX (to interface with microcontroller development board)

Hardware: 
- STMicroelectronics Nucleo 64 STMF334R8 microcontroller development board
- Potentiometers
- Veroboard
- Resistors, capacitors
- +-5V power supply

---

1. Feinberg, J. and Yang, B. (2018) ‘Natural-frequency splitting of a guitar string caused by a non-uniform magnetic field’, The Journal of the Acoustical Society of America. Acoustical Society of America, 144(5), pp. EL460–EL464. doi: 10.1121/1.5080465.

