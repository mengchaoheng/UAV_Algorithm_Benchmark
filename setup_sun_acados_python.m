function setup_sun_acados_python()
%SETUP_SUN_ACADOS_PYTHON Install Python dependencies for Sun acados NMPC.
%
% Run this from MATLAB when sun_nmpc reports that casadi or
% acados_template is missing from the Python environment used by MATLAB.

preferredPython = string(getenv("SUN_NMPC_PYTHON"));
if strlength(preferredPython) == 0
    candidatePython = "/Users/mchmini/.pyenv/versions/3.12.8/bin/python3";
    if exist(candidatePython, "file") == 2
        preferredPython = candidatePython;
    end
end

acadosDir = string(getenv("ACADOS_SOURCE_DIR"));
if strlength(acadosDir) == 0
    acadosDir = "/private/tmp/acados";
end

pe = pyenv;
if string(pe.Status) == "NotLoaded" && strlength(preferredPython) > 0 ...
        && exist(preferredPython, "file") == 2
    pyenv("Version", preferredPython, "ExecutionMode", "InProcess");
    pe = pyenv;
end

pythonExe = string(pe.Executable);
if strlength(pythonExe) == 0
    error("MATLAB pyenv has no Python executable. Configure pyenv first.");
end

fprintf("MATLAB Python: %s\n", pythonExe);

if exist(fullfile(acadosDir, "interfaces", "acados_template"), "dir") ~= 7
    error(["acados_template was not found under %s.\n" ...
           "Install acados first, for example:\n" ...
           "  git clone --depth 1 --recurse-submodules https://github.com/acados/acados.git %s\n" ...
           "  cmake -S %s -B %s -DACADOS_WITH_QPOASES=ON -DCMAKE_BUILD_TYPE=Release\n" ...
           "  cmake --build %s --target install -j4\n" ...
           "Then set ACADOS_SOURCE_DIR=%s and rerun setup_sun_acados_python."], ...
          acadosDir, acadosDir, acadosDir, fullfile(acadosDir, "build"), ...
          fullfile(acadosDir, "build"), acadosDir);
end

runCommand(sprintf('"%s" -m pip install --user casadi scipy matplotlib cython Deprecated', pythonExe));
runCommand(sprintf('"%s" -m pip install --user -e "%s"', ...
    pythonExe, fullfile(acadosDir, "interfaces", "acados_template")));

tRenderer = fullfile(acadosDir, "bin", "t_renderer");
if exist(tRenderer, "file") ~= 2
    if exist(fileparts(tRenderer), "dir") ~= 7
        mkdir(fileparts(tRenderer));
    end

    url = "https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-osx-arm64";
    fprintf("Downloading acados t_renderer...\n");
    websave(tRenderer, url);
    fileattrib(tRenderer, "+x");
end

py.importlib.invalidate_caches();
py.importlib.import_module("casadi");
py.importlib.import_module("acados_template");

fprintf("Sun acados Python dependencies are available. ");
if string(pyenv().Status) == "Loaded"
    fprintf("If main already failed in this MATLAB session, restart MATLAB and run main again.\n");
else
    fprintf("Run main now.\n");
end
end

function runCommand(cmd)

fprintf("%s\n", cmd);
[status, out] = system(cmd);
if status ~= 0
    error("Command failed:\n%s\n%s", cmd, out);
end
fprintf("%s", out);
end
