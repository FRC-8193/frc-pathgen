# `frc-pathgen`
## What's this?
A small simulation designed to test **path generation** and **following** algorithms for FRC robots.
## How can I use it?
`frc-pathgen` is developed and tested on **Arch Linux** and **Debian**. To build it, you will need the following packages installed:
1. `sdl2` (development libraries)
2. `cmake`
3. A working C++ compiler
4. `pkg-config`

> [!NOTE]
> Windows is unsupported but might work through MSYS2.

### Download
```
$ git clone https://github.com/FRC-8193/frc-pathgen.git --recurse-submodules
```
### Build
```
$ cd frc-pathgen
$ cmake -B build
$ cmake --build build -j
```
### Run
```
$ build/frc-pathgen
```
## Contributing
Contributions to `frc-pathgen` are always welcome. Make sure to follow our [style guide](https://github.com/FRC-8193/styleguide), and open a pull request with a detailed explanation of changes.  
> [!NOTE]
> [`.clangd`](.clangd) is provided for use with editors connected to the `clangd` language server. Other IDEs can be configured for our build process.

Made with ❤️ by Fred from team 8193 :)
