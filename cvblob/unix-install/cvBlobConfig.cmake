# ===================================================================================
#  cvBlob cmake configuration file
#
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(cvBlob REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${cvBlob_LIBS})
#
#    This file will define the following variables:
#      - cvBlob_LIBS          : The list of libraries to links against.
#      - cvBlob_LIB_DIR       : The directory where lib files are. Calling LINK_DIRECTORIES
#                                with this path is NOT needed.
#      - cvBlob_INCLUDE_DIRS  : The cvBlob include directories.
#      - cvBlob_VERSION       : The  version of this cvBlob build. Example: "0.10.3"
#      - cvBlob_VERSION_MAJOR : Major version part of cvBlob_VERSION. Example: "0"
#      - cvBlob_VERSION_MINOR : Minor version part of cvBlob_VERSION. Example: "10"
#      - cvBlob_VERSION_PATCH : Patch version part of cvBlob_VERSION. Example: "3"
#
# ===================================================================================


# Extract the directory where *this* file has been installed (determined at cmake run-time)
#  This variable may or may not be used below, depending on the parsing of cvBlobConfig.cmake
get_filename_component(THIS_CVBLOB_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH)

# ======================================================
# Include directories to add to the user project:
# ======================================================
INCLUDE_DIRECTORIES(/usr/local/include/)
# Provide the include directories to the caller
SET(cvBlob_INCLUDE_DIRS /usr/local/include/)

# ======================================================
# Link directories to add to the user project:
# ======================================================
LINK_DIRECTORIES("/usr/local/lib")
# Provide the libs directory anyway, it may be needed in some cases.
SET(cvBlob_LIB_DIR "/usr/local/lib")

# ====================================================================
# Link libraries: e.g.   cvblob...
# ====================================================================
set(CVBLOB_LIB_COMPONENTS cvblob)
SET(cvBlob_LIBS "")
foreach(__CVLIB ${CVBLOB_LIB_COMPONENTS})
  # CMake>=2.6 supports the notation "debug XXd optimized XX"
  if (CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
    # Modern CMake:
    SET(cvBlob_LIBS ${cvBlob_LIBS} debug ${__CVLIB}  optimized ${__CVLIB} )
  else(CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
    # Old CMake:
    SET(cvBlob_LIBS ${cvBlob_LIBS} ${__CVLIB})
  endif(CMAKE_MAJOR_VERSION GREATER 2  OR  CMAKE_MINOR_VERSION GREATER 4)
endforeach(__CVLIB)


# ======================================================
#  Version variables:
# ======================================================
SET(cvBlob_VERSION 0.10.4)
SET(cvBlob_VERSION_MAJOR  0)
SET(cvBlob_VERSION_MINOR  10)
SET(cvBlob_VERSION_PATCH  4)
