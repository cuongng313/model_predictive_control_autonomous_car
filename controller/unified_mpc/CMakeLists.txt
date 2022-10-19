cmake_minimum_required (VERSION 3.5)

project(unified_mpc)

add_executable(unified_mpc      UnifiedControllerNode.cpp
                                src/UnifiedController.cpp
                                src/AcadoUnified.cpp
                                )

target_include_directories(unified_mpc PRIVATE
                    ${RTIDDS_INCLUDE_DIR}
                    ${RTIDDS_INCLUDE_DIR}/ndds
                    ${CMAKE_SOURCE_DIR}/adas_dds/pubsub/include
                    ${CMAKE_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/adas_dds/common/include
                    ${CMAKE_SOURCE_DIR}/adas_dds/nodes
                    ${ADAS_DDS_INCLUDE_DIR}
                    ${ADAS_DDS_INCLUDE_DIR}/idl
                    ${CMAKE_CURRENT_SOURCE_DIR}/include
                    )

target_link_libraries(unified_mpc
                    -L${ADAS_DDS_LIBRARY_DIR} -L${RTIDDS_LIBRARY_DIR} ${RTIDDS_LIBRARIES}
                    -L${CMAKE_CURRENT_SOURCE_DIR}/acado_mpc_export
                    vinai_adas_dds
                    vinai_adas_dds_idl
                    adas_dds_pubsub
                    adas_common
                    acado_exported_rti
                    ${PLATFORM_LIBS}
                    )
