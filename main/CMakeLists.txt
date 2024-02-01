
set(PRIV_INCLUDE_DIRS_LIST
                      "${CMAKE_CURRENT_LIST_DIR}/include"
)
idf_component_register(PRIV_INCLUDE_DIRS ${PRIV_INCLUDE_DIRS_LIST} 
                        SRCS "main.cpp" "AppTask.cpp"
                        PRIV_REQUIRES SHT31 MAX1704x driver)
