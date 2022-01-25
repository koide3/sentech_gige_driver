find_path(StApi_INCLUDE_DIR NAMES StApi_C.h
  HINTS /opt/sentech/include/StApi /usr/local/include /usr/include
  DOC "StApi include directory")

find_path(GenICam_INCLUDE_DIR NAMES GenICam.h
  HINTS /opt/sentech/include/GenICam /usr/local/include /usr/include
  DOC "GenICam include directory")

set(StApi_INCLUDE_DIRS ${StApi_INCLUDE_DIR} ${GenICam_INCLUDE_DIR})

set(StApi_LIBRARY_NAMES StApi_C StApi_TL GCBase GenApi)
foreach(LIBRARY_NAME IN LISTS StApi_LIBRARY_NAMES)
  find_library(LIBRARY_PATH ${LIBRARY_NAME}
    HINTS /opt/sentech/lib /opt/sentech/lib/GenICam /usr/local/lib /usr/lib
    Doc "StApi library:${LIBRARY_NAME}"
    NO_CACHE
  )
  message(STATUS "StApi:${LIBRARY_NAME} ${LIBRARY_PATH}")
  set(StApi_LIBRARIES ${StApi_LIBRARIES} ${LIBRARY_PATH})
  set(LIBRARY_PATH "LIBRARY_PATH-NOTFOUND")
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(StApi REQUIRED_VARS StApi_INCLUDE_DIRS StApi_LIBRARIES)
