@[if DEVELSPACE]@
# Env variables in develspace.
export MININAV2D_CONFIG_DIR="@(CMAKE_CURRENT_SOURCE_DIR)/share/"
@[else]@
# Env variables in installspace.
export MININAV2D_CONFIG_DIR="$CATKIN_ENV_HOOK_WORKSPACE/share/"
@[end if]@