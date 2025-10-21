Servo Going to far issue could be fixed with a change in the incremant distance. in th emove function part of the code, it uses 1 increments. The issue could be fixed by changing those incremenents to a lower number like 0.25.

Change the pins in the robot wiring markdown to actual be the right ones on the pi (the current ones are for the old pi zero config)

Update fritzing diagram so l298n and motors are named correctly and the motors go to the correct pins on the correct l298n.