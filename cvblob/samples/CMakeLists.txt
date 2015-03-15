# Copyright (C) 2007 by Cristóbal Carnero Liñán
# grendel.ccl@gmail.com
#
# This file is part of cvBlob.
#
# cvBlob is free software: you can redistribute it and/or modify
# it under the terms of the Lesser GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# cvBlob is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# Lesser GNU General Public License for more details.
#
# You should have received a copy of the Lesser GNU General Public License
# along with cvBlob.  If not, see <http://www.gnu.org/licenses/>.
#
cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

include_directories(${cvBlob_SOURCE_DIR}/cvBlob)

# RedObj
set(RED_SRC red_object_tracking.cpp)

set_source_files_properties(${RED_SRC}
                            PROPERTIES
                            COMPILE_FLAGS "-O3"
)

add_executable(redObject ${RED_SRC})

target_link_libraries(redObject cvblob)
target_link_libraries(redObject ${OpenCV_LIBS})


