cmake_minimum_required(VERSION 3.1.0)

# Use newer policies if available, up to most recent tested version of CMake.
if(${CMAKE_VERSION} VERSION_LESS 3.11)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
else()
  cmake_policy(VERSION 3.11)
endif()

# Determine if fmt is built as a subproject (using add_subdirectory)
# or if it is the master project.
set(MASTER_PROJECT OFF)

# Joins arguments and places the results in ${result_var}.
function(join result_var)
  set(result )
  foreach (arg ${ARGN})
    set(result "${result}${arg}")
  endforeach ()
  set(${result_var} "${result}" PARENT_SCOPE)
endfunction()

# Set the default CMAKE_BUILD_TYPE to Release.
# This should be done before the project command since the latter can set
# CMAKE_BUILD_TYPE itself (it does so for nmake).
if (MASTER_PROJECT AND NOT CMAKE_BUILD_TYPE)
  join(doc "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or "
           "CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING ${doc})
endif ()

option(FMT_PEDANTIC "Enable extra warnings and expensive tests." OFF)
option(FMT_WERROR "Halt the compilation with an error on compiler warnings."
       OFF)

# Options that control generation of various targets.
option(FMT_DOC "Generate the doc target." ${MASTER_PROJECT})
option(FMT_INSTALL "Generate the install target." ${MASTER_PROJECT})
option(FMT_TEST "Generate the test target." ${MASTER_PROJECT})
option(FMT_FUZZ "Generate the fuzz target." OFF)

project(FMT CXX)

# Get version from core.h
file(READ include/fmt/core.h core_h)
if (NOT core_h MATCHES "FMT_VERSION ([0-9]+)([0-9][0-9])([0-9][0-9])")
  message(FATAL_ERROR "Cannot get FMT_VERSION from core.h.")
endif ()
# Use math to skip leading zeros if any.
math(EXPR CPACK_PACKAGE_VERSION_MAJOR ${CMAKE_MATCH_1})
math(EXPR CPACK_PACKAGE_VERSION_MINOR ${CMAKE_MATCH_2})
math(EXPR CPACK_PACKAGE_VERSION_PATCH ${CMAKE_MATCH_3})
join(FMT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.
                 ${CPACK_PACKAGE_VERSION_PATCH})
message(STATUS "Version: ${FMT_VERSION}")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/support/cmake")

include(cxx14)
include(CheckCXXCompilerFlag)

set(FMT_REQUIRED_FEATURES cxx_auto_type cxx_variadic_templates)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(PEDANTIC_COMPILE_FLAGS -pedantic-errors -Wall -Wextra -pedantic
      -Wold-style-cast -Wundef
      -Wredundant-decls -Wwrite-strings -Wpointer-arith
      -Wcast-qual -Wformat=2 -Wmissing-include-dirs
      -Wcast-align -Wnon-virtual-dtor
      -Wctor-dtor-privacy -Wdisabled-optimization
      -Winvalid-pch -Woverloaded-virtual
      -Wconversion
      -Wno-ctor-dtor-privacy -Wno-format-nonliteral -Wno-shadow)
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.6)
      set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS} -Wnoexcept
         -Wno-dangling-else -Wno-unused-local-typedefs)
  endif ()
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0)
      set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS} -Wdouble-promotion
          -Wtrampolines -Wzero-as-null-pointer-constant -Wuseless-cast
          -Wvector-operation-performance -Wsized-deallocation)
  endif ()
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0)
      set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS} -Wshift-overflow=2
          -Wnull-dereference -Wduplicated-cond)
  endif ()
  set(WERROR_FLAG -Werror)
endif ()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(PEDANTIC_COMPILE_FLAGS -Wall -Wextra -pedantic -Wconversion
      -Wno-sign-conversion)
  check_cxx_compiler_flag(-Wzero-as-null-pointer-constant HAS_NULLPTR_WARNING)
  if (HAS_NULLPTR_WARNING)
    set(PEDANTIC_COMPILE_FLAGS ${PEDANTIC_COMPILE_FLAGS}
        -Wzero-as-null-pointer-constant)
  endif ()
  set(WERROR_FLAG -Werror)
endif ()

if (MSVC)
  set(PEDANTIC_COMPILE_FLAGS /W3)
  set(WERROR_FLAG /WX)
endif ()

if (MASTER_PROJECT AND CMAKE_GENERATOR MATCHES "Visual Studio")
  # If Microsoft SDK is installed create script run-msbuild.bat that
  # calls SetEnv.cmd to set up build environment and runs msbuild.
  # It is useful when building Visual Studio projects with the SDK
  # toolchain rather than Visual Studio.
  include(FindSetEnv)
  if (WINSDK_SETENV)
    set(MSBUILD_SETUP "call \"${WINSDK_SETENV}\"")
  endif ()
  # Set FrameworkPathOverride to get rid of MSB3644 warnings.
  set(netfxpath "C:\\Program Files\\Reference Assemblies\\Microsoft\\Framework\\.NETFramework\\v4.0")
  file(WRITE run-msbuild.bat "
    ${MSBUILD_SETUP}
    ${CMAKE_MAKE_PROGRAM} -p:FrameworkPathOverride=\"${netfxpath}\" %*")
endif ()

set(strtod_l_headers stdlib.h)
if (APPLE)
  set(strtod_l_headers ${strtod_l_headers} xlocale.h)
endif ()

include(CheckSymbolExists)
if (WIN32)
  check_symbol_exists(open io.h HAVE_OPEN)
  check_symbol_exists(_strtod_l "${strtod_l_headers}" HAVE_STRTOD_L)
else ()
  check_symbol_exists(open fcntl.h HAVE_OPEN)
  check_symbol_exists(strtod_l "${strtod_l_headers}" HAVE_STRTOD_L)
endif ()

function(add_headers VAR)
  set(headers ${${VAR}})
  foreach (header ${ARGN})
    set(headers ${headers} include/fmt/${header})
  endforeach()
  set(${VAR} ${headers} PARENT_SCOPE)
endfunction()

# Define the fmt library, its includes and the needed defines.
add_headers(FMT_HEADERS chrono.h color.h compile.h core.h format.h format-inl.h
                        locale.h ostream.h printf.h ranges.h
                        safe-duration-cast.h)
set(FMT_SOURCES src/format.cc)
if (HAVE_OPEN)
  add_headers(FMT_HEADERS posix.h)
  set(FMT_SOURCES ${FMT_SOURCES} src/posix.cc)
endif ()

add_library(fmt ${FMT_SOURCES} ${FMT_HEADERS} README.rst ChangeLog.rst)
add_library(fmt::fmt ALIAS fmt)

if (HAVE_STRTOD_L)
  target_compile_definitions(fmt PUBLIC FMT_LOCALE)
endif ()

if (FMT_WERROR)
  target_compile_options(fmt PRIVATE ${WERROR_FLAG})
endif ()
if (FMT_PEDANTIC)
  target_compile_options(fmt PRIVATE ${PEDANTIC_COMPILE_FLAGS})
endif ()

target_compile_features(fmt INTERFACE ${FMT_REQUIRED_FEATURES})

target_include_directories(fmt PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set_target_properties(fmt PROPERTIES
  VERSION ${FMT_VERSION} SOVERSION ${CPACK_PACKAGE_VERSION_MAJOR}
  DEBUG_POSTFIX d)

if (FMT_SAFE_DURATION_CAST)
  target_compile_definitions(fmt PUBLIC FMT_SAFE_DURATION_CAST)
endif()

add_library(fmt-header-only INTERFACE)
add_library(fmt::fmt-header-only ALIAS fmt-header-only)

target_compile_definitions(fmt-header-only INTERFACE FMT_HEADER_ONLY=1)

target_compile_features(fmt-header-only INTERFACE ${FMT_REQUIRED_FEATURES})

target_include_directories(fmt-header-only INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


