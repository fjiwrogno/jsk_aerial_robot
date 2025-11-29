working frequency: 300hz

pwm port: pwm 5, 6, 7, 8
encoder: pisitive encoder, which means its rotation will refer to the positive angle position
0 angle: I find the 0 angle still needs to be compensated because right now when sending 0 command, its position is not 0 angle visually. So manual offset is essential.
Domain protection: the pulse width is protected within the 500 ~ 2500.
Problem: the definition of this servo is 0 ~ 357 anticlockwise. But the angle range of attitude controller is [-pi, pi]. And rotate to minus angle is a quite usual case but it cannot work directly with this motor. So one solution is just to define logic zero point like 180 deg. 180 deg will be the starting point of the tlting thrust unit. But for this kind of method, because the servo will go back to zero point every time power it on, the servo will experience 180 deg rotate every time, which is definitely bad for your application.
So maybe new bi-directional esc should be used for your application!
But 4 in 1 version is just too hot to sustain. Maybe 1pcs one could be a good option?! AM32 one is better, easy to config!
