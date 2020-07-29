# Virtual-Joystick
...playing JustCause4 PC Game Virtually.

Working:
---------------

1. OpenCV is used to detect skin colour of Hands in steering position.
2. Two hands from front view seems as 2 Circles from webcam.
3. It finds centers of Two Circles.
4. Assignes x and y coordinates of both Circles.
5. Finds Distance betweeen two points to get Speed.
6. Also finds Slope of two points to get Angle and Fixing left and right direction in controller.
7. Accordingly A, D, S, W ...key is Pressed using ctypes Python library.
