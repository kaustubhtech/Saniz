set(COMPONENT_SRCS "./MLX90614_API.c" "./MLX90614_SMBus_Driver.c")
set(COMPONENT_ADD_INCLUDEDIRS "./include")

idf_component_register(SRCS "MLX90614_API.c" "MLX90614_SMBus_Driver.c"
                        INCLUDE_DIRS "include .")