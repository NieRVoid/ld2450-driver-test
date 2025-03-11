# CMakeLists.txt for LD2450 Radar Driver Component
idf_component_register(
    SRCS 
        "src/ld2450.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        "driver"
        "esp_timer"
        "esp_event"
        "log"
        "esp_common"
        "freertos"
)

# Set compiler options for ESP-IDF v5.4 compatibility
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")