add_executable(main main.c)

# pull in common dependencies
target_link_libraries(main PRIVATE pico_stdlib hardware_pwm pico_servo freertos hardware_adc)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)

