# Dancing-Robot-Project
A dancing robot project using Parallax BoeBot

Line-following robot performs a predefined dance choreography on a grid-based dance floor (maximum size 9×9, coordinates A1 to I9). The robot moves between grid intersections according to timed instructions and synchronises its motion with absolute timestamps. 

Example choreography: A1N, E1 T150, B2 T350, 3A T450, 4C T567, D2 T700

The choreography consists of:
* Starting position and orientation (N, E, S, W) (e.g., A1N) 
* A sequence of dance steps, each containing:
  * Target position (e.g., B2, 4C)
  * Absolute time when the robot must leave that position (e.g., T350 → 35.0 seconds)

For each step, the coordinate determines the movement order (letter first → column then row, number first → row then column).

The dance begins on button press and ends after completing all steps; another button press at any time makes the robot return to its starting position and orientation. 

The choreography is predefined at compile time but can be overridden via serial input before starting, with persistent storage (EEPROM) preferred.

The program parses input (case-insensitive, flexible separators, possible typing delays), rejects malformed input with meaningful error messages, and operates within floor boundaries. 
