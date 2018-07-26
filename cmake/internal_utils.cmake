cmake_minimum_required(VERSION 2.8)

# put key value to map
MACRO(MAP_PUT _MAP _KEY _VALUE)
  SET("MAP_${_MAP}_${_KEY}" ${_VALUE})
ENDMACRO()

# get value int map
MACRO(MAP_GET _MAP _KEY _OUTPUT)
  SET(KEY "MAP_${_MAP}_${_KEY}")
  set(${_OUTPUT} "undefined")
  if (${KEY})
    set(${_OUTPUT} ${${KEY}})
  endif ()

ENDMACRO()

# load properties in file and put it into map
MACRO(LOAD_PROPERTY _MAP _FILENAME)
  FILE(READ ${_FILENAME} contents)
  STRING(REGEX REPLACE "\n" ";" lines "${contents}")
  foreach (line ${lines})
    if (NOT (${line} MATCHES "^(#|\t|\n| )"))
      STRING(REGEX REPLACE "\t+| +" ";" fields ${line})
      list(GET fields 0 KEY)
      list(GET fields 1 VALUE)
      MAP_PUT(${_MAP} ${KEY} ${VALUE})
    endif ()
  endforeach ()

ENDMACRO()


MACRO(GETLINES _LINES _FILENAME)
  FILE(READ ${_FILENAME} contents)
  STRING(REGEX REPLACE "\n" ";" ${_LINES} "${contents}")
ENDMACRO()