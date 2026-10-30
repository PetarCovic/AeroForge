idf_component_register(
    SRCS
        "main.cpp"
        "HilWireProtocol.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_uart
        esp_timer
)

target_compile_features(
    ${COMPONENT_LIB}
    PUBLIC cxx_std_17
)
