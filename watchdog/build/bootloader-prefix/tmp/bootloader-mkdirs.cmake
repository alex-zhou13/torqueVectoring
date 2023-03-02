# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/alexzhou13/Documents/spring2023/EC464/esp/esp-idf/components/bootloader/subproject"
  "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader"
  "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader-prefix"
  "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader-prefix/tmp"
  "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader-prefix/src"
  "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/alexzhou13/Documents/spring2023/EC464/esp/watchdog/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
