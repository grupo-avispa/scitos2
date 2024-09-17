#
# Find MIRA path
#
# @public
#
macro(find_mira_path)
  message(STATUS "Detecting MIRA root directory")

  # Examines each path in MIRA_PATH in order to find the MIRA root directory, which
  # is identified by the file "make/Insource.make"
  SET(ENV{MIRA_PATH} "$ENV{MIRA_PATH}:/opt/mira:${CMAKE_SOURCE_DIR}")
  SET(FORCE_USE_QT5 1)

  set(FindMIRARoot_MIRA_PATH "$ENV{MIRA_PATH}")

  if(UNIX)
    # note: the "" around "${MIRA_PATH}" are very important !
    string(REPLACE ":" ";" FindMIRARoot_MIRA_PATH "${FindMIRARoot_MIRA_PATH}")
  endif()

  set(FOUND_MIRA_ROOT_DIR 0)
  set(MIRA_ROOT_DIR "")

  FOREACH(path ${FindMIRARoot_MIRA_PATH})
    GET_FILENAME_COMPONENT(pathComponent ${path} ABSOLUTE)

    # strip any trailing slashes from every path in MIRA_PATH env
    MESSAGE(STATUS "Examining ${pathComponent}")

    if(EXISTS "${pathComponent}/mira.root")
      MESSAGE(STATUS "found valid ${pathComponent}")
      set(FOUND_MIRA_ROOT_DIR 1)
      set(MIRA_ROOT_DIR ${pathComponent})
    endif()
  ENDFOREACH(path)

  MESSAGE(STATUS "MIRA_ROOT_DIR=${MIRA_ROOT_DIR}")
  MESSAGE(STATUS "FOUND_MIRA_ROOT_DIR=${FOUND_MIRA_ROOT_DIR}")

  IF(FOUND_MIRA_ROOT_DIR)
    # return the final relative path; must use echo, as cmake does not provide
    # mechanism to output to stdout :(
    execute_process(COMMAND ${CMAKE_COMMAND} -E echo ${MIRA_ROOT_DIR})
  ELSE()
    message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    message("Your MIRA_PATH environment variable is not set properly.\nPlease add the proper path to your MIRA directory.")
    message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    message(FATAL_ERROR)
  ENDIF()
endmacro()

# #############################################################################

#
# Include MIRA packages
#
# @public
#
macro(include_mira_packages)
  set(CMAKE_MODULE_PATH ${MIRA_ROOT_DIR}/make)

  include(Prerequisites)
  include(Dependencies)

  # Require MIRAFramework package
  mira_require_package(MIRAFramework)
  mira_require_package(RobotDataTypes)
endmacro()

# #############################################################################

#
# Target link for MIRA libraries
#
# @public
#
macro(target_link_mira_libraries target)
  # For transitive linking link against all auto liked libraries
  target_link_libraries(${target} PUBLIC ${MIRAAutoLinkLibraries} MIRAFramework)

  # use, i.e. don't skip the full RPATH for the build tree
  SET(CMAKE_SKIP_BUILD_RPATH FALSE)

  # when building, don't use the install RPATH already
  # (but later on when installing)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

  SET(CMAKE_INSTALL_RPATH "${MIRA_ROOT_DIR}/lib")

  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  # the RPATH to be used when installing, but only if it's not a system directory
  LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)

  IF("${isSystemDir}" STREQUAL "-1")
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  ENDIF("${isSystemDir}" STREQUAL "-1")
endmacro()