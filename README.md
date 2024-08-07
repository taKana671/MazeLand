# MazeLand
Find the exit and escape the maze!　Airplanes move automatically. Control a snowman to escape the maze faster than them.　The snowman can jump to see the maze from above or to avoid collision with them.

https://github.com/taKana671/MazeLand/assets/48859041/03b340ad-e070-4811-8095-f65566374e2f

I have wanted to try split screen. For this reason, I made this game. But what was harder than such a screen was to make camera follow the snowman in the narrow passages of the maze. I tried many different ways, but camera went into the 3D objects such as walls and the inside of the objects was displayed on the screen. Finally, it took so many days to find that it was ok just to set the variable 'near_distance' of set_near_far method to less than 1. 

# Requirements
* Panda3D 1.10.13
* Cython 0.29.37
* numpy 1.23.5
  
# Environment
* Python 3.11
* Windows 11

# Usage
* Execute a command below on your command line.
```
>>>python maze_land.py
```

# Controls:
* Press [Esc] to quit.
* Press [up arrow] key to go foward.
* Press [left arrow] key to turn left.
* Press [right arrow] key to turn right.
* Press [down arrow] key to go back.
* Press [Enter] key to jump.
* Press [ D ] key to toggle debug ON and OFF.

# Cython code
To speed up maze algorithm, I used Cython language to convert Python code to C code.
To build your Cython file, use the command below, which will leave in your local a so file in unix or pyd file in Windows.

```
>>>cd cython_code
>>>python setup.py build_ext --inplace
```

```
# python code (maze size is 155 * 155)
36.4 ms ± 1.14 ms per loop (mean ± std. dev. of 7 runs, 10 loops each)
```

```
# cython code (maze size is 155 * 155)
18.2 ms ± 144 µs per loop (mean ± std. dev. of 7 runs, 100 loops each)
```
