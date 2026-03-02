# Dancing-Robot-Project
A dancing robot project using Parallax BoeBot

Line-following robot performs a predefined dance choreography on a grid-based dance floor (maximum size 9×9, coordinates A1 to I9). The robot moves between grid intersections according to timed instructions and synchronises its motion with absolute timestamps. 

Example choreography: A1N, E1 T150, B2 T350, 3A T450, 4C T567, D2 T700

The choreography consists of:
* Starting position and orientation (e.g., A1N)
* A sequence of dance steps, each containing:
  * Target position (e.g., B2, 4C)
  * Absolute time when the robot must leave that position (e.g., T350 → 35.0 seconds)
