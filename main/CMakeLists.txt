idf_component_register(SRCS "main.cpp"
                    #Zigbee
                    zb/zb_main.hpp
                    zb/zb_main.cpp
                    #Sensors
                    sensors/aht21.hpp
                    sensors/aht21.cpp
                    sensors/ens160.hpp
                    sensors/ens160.cpp
                    sensors/scd40.hpp
                    sensors/scd40.cpp
                    INCLUDE_DIRS "")
