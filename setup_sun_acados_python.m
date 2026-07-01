function setup_sun_acados_python()
%SETUP_SUN_ACADOS_PYTHON Compatibility wrapper for setup_acados_python.

warning("setup_sun_acados_python:renamed", ...
    "setup_sun_acados_python is deprecated; use setup_acados_python.");
setup_acados_python();
end
