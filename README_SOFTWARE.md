You need to provide clear and concise documentation for all the code that you have written. It simply
does not suffice to copy all the code into a folder and rely on comments in your code for documentation.

Please provide the following:

• An overview of each software module

• A flow chart indicating the dependencies between these functions. For instance, if you have a
main.py and LCD.py, you need to show that LCD.py is a module used by main.py.

• Dev/build tool information: Package name and version info. For example, OpenCV 4.0.3 with
Python 3.8.1, using CUDA Toolbox 10.0 and GCC 9.1 and CMake 3.14.2

• How to install the project software stack from scratch (a blank hard drive / cloud instance) Please
provide concise documentation on what installation software is needed, and how to build from
source to binary as applicable.

Example:

• Debian Linux 9.1 into embedded system, with kernel patch to enable the USB controller

• how to use CMake to cross-compile code, how to transfer binary into the target, how target
autoruns binary, etc.

• How to setup/startup cloud instance that collects and processes data from the embedded
system
