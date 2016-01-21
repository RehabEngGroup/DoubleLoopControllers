# Copyright 2015 Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include_directories(${OPENSIM_INCLUDE_DIRS} ${SIMTK_INCLUDE_DIRS})

set(DOUBLE_LOOP_SOURCE_FILES DoubleLoopPIController.cpp RegisterTypes_osimDoubleLoopControllers.cpp)
set(DOUBLE_LOOP_HEADER_FILES DoubleLoopPIController.h RegisterTypes_osimDoubleLoopControllers.h)
source_group("Headers" FILES ${DOUBLE_LOOP_HEADER_FILES})
source_group("Sources" FILES ${DOUBLE_LOOP_SOURCE_FILES})

add_library(DoubleLoopController SHARED ${DOUBLE_LOOP_SOURCE_FILES} ${DOUBLE_LOOP_HEADER_FILES})
target_link_libraries(DoubleLoopController ${OPENSIM_LIBRARIES})

set_target_properties(DoubleLoopController PROPERTIES
                      DEFINE_SYMBOL OSIMPLUGIN_EXPORTS)
