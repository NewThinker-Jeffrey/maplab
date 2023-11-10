@[if DEVELSPACE]@
# Env variables in develspace.
export OPENVINS_CONFIG_DIR="@(CMAKE_CURRENT_SOURCE_DIR)/share/"
@[else]@
# Env variables in installspace.
export OPENVINS_CONFIG_DIR="$CATKIN_ENV_HOOK_WORKSPACE/share/"
@[end if]@