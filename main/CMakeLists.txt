set(srcs "main.c"
    "nmea_uart.c"
    )

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ".")

list(APPEND EXTRA_COMPONENT_DIRS components)
