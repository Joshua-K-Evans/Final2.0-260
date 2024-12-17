# Final2.0-260


	
 This project initially started from a need I noticed at an old ski resort that I used to work at.
 I noticed that the shuttle was constantly running a loop regardless of if people were there to be picked up. 
 The objective of this project is to stop the shuttle from needing to be “polling” for passengers and can instead use a more efficient method that will save them on fuel and labor time.
 This project will affect public health, environmental factors, and economic factors by helping shuttle services use an "interrupt" method instead of a "polling" method to see if people need to be picked up and shuttled.
 It will impact public safety and public welfare by helping the resort have more clear and solid communication on the mountain so that they will be more safe. As far as global factors, cultural factors and social factors, there is no direct impact negative or positive. 
 The 3 principles that this project uses are Digital Eternal Interrupts via the button being pressed to signify a passenger ready to be picked up, UART communication between the 2 microcontrollers/UART 1 and UART 3, and I2C communication through the LCD 1602 display. 
 There is also a 4th to consider because of the RF modules being an I/O device and system that the microcontroller communicates with, this one proved the most difficult to implement as well.
