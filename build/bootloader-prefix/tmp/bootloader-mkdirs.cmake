# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/espressif5.0/frameworks/esp-idf-v5.0.3/components/bootloader/subproject"
  "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader"
  "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader-prefix"
  "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader-prefix/tmp"
  "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader-prefix/src/bootloader-stamp"
  "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader-prefix/src"
  "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/espressif5.0/frameworks/arcadeJoypad/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
