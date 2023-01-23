# CS 6610
Interactive Computer Graphics project assignments!

## Building

### Linux
You might need to install a couple packages:
- `libxinerama-dev`
- `libglu1-mesa-dev`
- `libxi-dev`

Then let CMake handle the rest:
```bash
mkdir build && cd build && cmake .. && make -j
```

### Windows
Open project as a CMake project, click "Build All", and hope for the best

## Running
`./build/app` or `.\build\app.exe`

## Dependencies
These dependencies are either included or managed by CMake and built locally:

- [GLFW](https://github.com/glfw/glfw)
- [glm](https://github.com/g-truc/glm)
- [gleq](https://github.com/glfw/gleq)
- [cyCodeBase](http://www.cemyuksel.com/cyCodeBase/code.html)
- [spdlog](https://github.com/gabime/spdlog)
- glad