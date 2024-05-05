# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS D:/esp/esp-idf-lib/components D:/esp/FastLED-idf-master/FastLED-idf-master/components/FastLED-idf
                            D:/esp/FastLED-idf-master/FastLED-idf-master/components/WS2812FX-idf
                            $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
set(PROJECT_VER "0.7.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(saniz_mk2)



