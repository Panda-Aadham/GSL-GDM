#############################################################
# I DON'T KNOW IF THIS WILL WORK BUT IF IT FAILS RUN EACH   #
# BUILD CMD INDIVIDUALLY AND DO `source install/setup.bash` #
#############################################################


# GADEN Packages
colcon build --symlink-install --packages-select \
gaden_common                                            \
gaden_environment                                       \
gaden_filament_simulator                                \
gaden_msgs                                              \
gaden_player                                            \
gaden_preprocessing                                     \
test_env                                                \

# Dependency for Sensor Packages and GSL
colcon build --symlink-install --packages-select \
olfaction_msgs                                          \
ament_imgui                                             \
vision_msgs

# Simulated Sensor Packages
colcon build --symlink-install --packages-select \
simulated_anemometer                                    \
simulated_gas_sensor                                    \
simulated_tdlas                                         \
voxeland_msgs

colcon build --symlink-install --packages-select voxeland

colcon build --symlink-install --packages-select \
gsl_actions                                             \
gsl_server