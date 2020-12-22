# General Overview
## Prerequisites

A suitable compiler for the RISC-V ISA must be available.
Since the RI5CY RISC-V core supports additional ISA extensions that are not
supported by official toolchain, a special compiler must be used to take
advantage of those.

For the basic RV32I instruction set also the official toolchain can be used.

## Setup

The software compilation flow is based on CMake. A version of CMake >= 2.8.0 is
required, but a version greater than 3.1.0 is recommended due to support for
ninja and `python-is-python2` is recommended.

CMake uses out-of-source builds which means you will need a separate build
folder for the software.

    mkdir pulpino-dev
    cd pulpino-dev
    git clone https://github.com/mrninhvn/pulpino-dev-build
    git clone https://github.com/mrninhvn/pulpino-dev-sw

Now you are ready to start compiling software!


## Compiling

Switch to the build folder and run config file:

    ./cmake_configure.riscv.gcc.sh

Compile the application you are interested in:

    make applicationName && make applicationName.links

Or make all applications:

    make

This command will compile the application and generate SLM to Upload.



# Applications
## How to add a new application

CMake uses the concept of CMakeLists.txt files in each directory that is
managed by the tool. Those files give instructions to the tool about which
applications exist and which files belong to it.

An application is defined like this in a CMakeLists.txt file:

    add_application(helloworld helloworld.c)


If an application consists of multiple source files it has be defined like
this:

    set(SOURCES main.c helper.c)
    add_application(helloworld "${SOURCES}")


For ease-of-use we recommend that each application has its own source
directory. Use the `add_subdirectory` macro of CMake to let the tool know about
folder structures. Those macros are put in the parent folders until you hit a
folder that is already managed by CMake. Each of the folders needs to have a
CMakeLists.txt file. Eg:

    add_subdirectory(Blink)

All applications need to have their own build folders. This means that if you
want to declare multiple applications in the same source folders, you have to
make sure they do not share the same build folder. This can be done by the
optional argument `SUBDIR` for `add_application`

    add_application(helloworld helloworld.c SUBDIR "hello"))

The command above would put the application helloworld in a subdirectory called
hello in the build folder structure.
