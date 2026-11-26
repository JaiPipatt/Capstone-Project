idf_component_register(SRCS "main.c" "robot_logic.c" "kinematics.c" "stm32_link.c" "protocol.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver)
