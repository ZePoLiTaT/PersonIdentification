project( vtkMeshGeodesicExamples )

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package( VTK REQUIRED )
include( ${VTK_USE_FILE} )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../Source  )

add_executable( GeodesicDistanceExample GeodesicDistanceExample.cxx )
target_link_libraries( GeodesicDistanceExample vtkMeshGeodesics ${VTK_LIBRARIES} ${PCL_LIBRARIES} )

add_executable( GeodesicPathExample GeodesicPathExample.cxx )
target_link_libraries( GeodesicPathExample vtkMeshGeodesics ${VTK_LIBRARIES} ${PCL_LIBRARIES} )
