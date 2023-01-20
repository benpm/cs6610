# CS 6610
Interactive computer graphics project assignments

## Building
All dependencies built via CMake sorcery

### Linux
You might need to install a couple packages:
- `libxinerama-dev`
- `libglu1-mesa-dev`
- `libxi-dev`

```bash
mkdir build && cd build && cmake .. && make -j
```

### Windows
Open project as a CMake project, click "Build All", and hope for the best

## Credits
- GLFW
- glm
- [gleq](https://github.com/glfw/gleq)
- [cyCodeBase](http://www.cemyuksel.com/cyCodeBase/code.html)