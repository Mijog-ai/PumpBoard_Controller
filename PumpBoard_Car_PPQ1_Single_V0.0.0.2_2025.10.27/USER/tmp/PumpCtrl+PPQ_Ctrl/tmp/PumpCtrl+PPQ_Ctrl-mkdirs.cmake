# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl")
  file(MAKE_DIRECTORY "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl")
endif()
file(MAKE_DIRECTORY
  "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/1"
  "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl"
  "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl/tmp"
  "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl/src/PumpCtrl+PPQ_Ctrl-stamp"
  "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl/src"
  "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl/src/PumpCtrl+PPQ_Ctrl-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl/src/PumpCtrl+PPQ_Ctrl-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "H:/PumpBoard_Controller/PumpBoard_Car_PPQ1_Single_V0.0.0.2_2025.10.27/USER/tmp/PumpCtrl+PPQ_Ctrl/src/PumpCtrl+PPQ_Ctrl-stamp${cfgdir}") # cfgdir has leading slash
endif()
