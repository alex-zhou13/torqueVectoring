# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/components/bootloader/subproject"
  "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader"
  "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader-prefix"
  "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader-prefix/tmp"
  "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader-prefix/src"
  "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Users/curse/.espressif/frameworks/esp-idf-v5.0/SrDes-MCP4725-master/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()