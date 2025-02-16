idf_component_register(SRCS "src/ACAN2515.cpp"
                            "src/ACAN2515Settings.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver freertos esp_timer)
