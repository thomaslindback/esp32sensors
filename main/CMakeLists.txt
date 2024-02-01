
set(PRIV_INCLUDE_DIRS_LIST
                      "${CMAKE_CURRENT_LIST_DIR}/include"
)
idf_component_register(PRIV_INCLUDE_DIRS ${PRIV_INCLUDE_DIRS_LIST} 
                        SRCS "main.cpp" "AppTask.cpp" "ASHT31.cpp" "SMAX1704x_fuel_Gauge.cpp"
                    )
