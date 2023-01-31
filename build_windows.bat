cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug -SC:./ -Bc:./build -G Ninja
cmake --build build --config Debug --target all --