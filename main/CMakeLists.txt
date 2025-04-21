idf_component_register(SRCS "main.c"
                       SRCS "${CMAKE_SOURCE_DIR}/src/fr_flash.c"
                       SRCS "${CMAKE_SOURCE_DIR}/src/fr_forward.c"
                       SRCS "${CMAKE_SOURCE_DIR}/src/fd_forward.c"
                       SRCS "${CMAKE_SOURCE_DIR}/src/image_util.c"
                       INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include"
                       REQUIRES esp32-camera esp_partition esp_timer esp_http_server esp_wifi wpa_supplicant nvs_flash) 
                       
target_link_libraries(${COMPONENT_LIB} PRIVATE "${CMAKE_SOURCE_DIR}/lib/libdl.a")