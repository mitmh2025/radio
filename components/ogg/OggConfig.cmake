get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

set(Ogg_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set(OGG_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
set(Ogg_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")
set(OGG_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")

add_library(Ogg::ogg IMPORTED STATIC)
set_target_properties(Ogg::ogg PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${PACKAGE_PREFIX_DIR}/include"
  IMPORTED_LOCATION "$ENV{ADF_PATH}/components/esp-adf-libs/esp_codec/lib/${IDF_TARGET}/libesp_codec.a"
)

check_required_components(Ogg)
set(OGG_FOUND 1)
