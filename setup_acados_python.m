function setup_acados_python()
%SETUP_ACADOS_PYTHON Prepare Python dependencies for this acados checkout.
%
% New-machine default:
%   1. Use ACADOS_PYTHON when it is set.
%   2. Otherwise create/use .venv under this repository.
%   3. Install casadi and acados_template into that Python.
%
% If MATLAB already loaded a different Python in this session, the setup still
% prepares .venv, but MATLAB must be restarted before py.* can use it.

repoDir = fileparts(mfilename('fullpath'));
if isempty(repoDir)
    repoDir = pwd;
end

projectVenvDir = fullfile(repoDir, ".venv");
projectPython = fullfile(projectVenvDir, "bin", "python3");
if ispc
    projectPython = fullfile(projectVenvDir, "Scripts", "python.exe");
end

pythonExe = string(getenv("ACADOS_PYTHON"));
if strlength(pythonExe) == 0
    if exist(projectPython, "file") ~= 2
        bootstrapPython = string(getenv("ACADOS_BOOTSTRAP_PYTHON"));
        if strlength(bootstrapPython) == 0
            bootstrapPython = "python3";
        end

        fprintf("Creating project Python environment:\n  %s\n", projectVenvDir);
        runCommand(sprintf('"%s" -m venv "%s"', ...
            char(bootstrapPython), projectVenvDir));
    end
    pythonExe = string(projectPython);
end

if exist(pythonExe, "file") ~= 2
    error(['acados Python does not exist:\n  %s\n' ...
           'Set ACADOS_PYTHON to a valid python3, or unset it so this ' ...
           'script can create .venv.'], char(pythonExe));
end

acadosDir = string(getenv("ACADOS_SOURCE_DIR"));
if strlength(acadosDir) == 0
    acadosDir = fullfile(repoDir, ".acados", "acados");
end

acadosTemplateDir = fullfile(acadosDir, "interfaces", "acados_template");
if exist(acadosTemplateDir, "dir") ~= 7
    error(["acados_template was not found under %s.\n" ...
           "Run install_acados to clone/build acados and set up Python automatically.\n" ...
           "Manual install example:\n" ...
           "  mkdir -p %s\n" ...
           "  git clone --depth 1 --recurse-submodules https://github.com/acados/acados.git %s\n" ...
           "  cmake -S %s -B %s -DACADOS_WITH_QPOASES=ON -DCMAKE_BUILD_TYPE=Release\n" ...
           "  cmake --build %s --target install -j4\n" ...
           "Then rerun setup_acados_python."], ...
          char(acadosDir), char(fileparts(acadosDir)), char(acadosDir), ...
          char(acadosDir), char(fullfile(acadosDir, "build")), ...
          char(fullfile(acadosDir, "build")));
end

setenv("ACADOS_PYTHON", char(pythonExe));
setenv("ACADOS_SOURCE_DIR", char(acadosDir));
setenv("ACADOS_INSTALL_DIR", char(acadosDir));
setenv("MPLCONFIGDIR", char(fullfile(tempdir, "matplotlib")));

fprintf("acados Python:\n  %s\n", char(pythonExe));
fprintf("acados source:\n  %s\n", char(acadosDir));

runCommand(sprintf('"%s" -m pip install --upgrade pip setuptools wheel', ...
    char(pythonExe)));
runCommand(sprintf(['"%s" -m pip install numpy casadi scipy matplotlib ' ...
    'cython Deprecated'], char(pythonExe)));
runCommand(sprintf('"%s" -m pip install -e "%s"', ...
    char(pythonExe), char(acadosTemplateDir)));

ensureTRenderer(acadosDir);

pe = pyenv;
if string(pe.Status) == "NotLoaded"
    pyenv("Version", char(pythonExe), "ExecutionMode", "InProcess");
    pe = pyenv;
end

if strcmp(char(pe.Executable), char(pythonExe))
    pyPath = py.sys.path;
    pyPath.insert(int32(0), char(acadosTemplateDir));
    py.importlib.invalidate_caches();
    py.importlib.import_module("casadi");
    py.importlib.import_module("acados_template");
    fprintf("acados Python dependencies are available. Run main now.\n");
else
    runCommand(sprintf('"%s" -c "import casadi, acados_template"', ...
        char(pythonExe)));
    fprintf(['acados Python dependencies were installed, but MATLAB ' ...
        'has already loaded a different Python:\n  %s\n' ...
        'Restart MATLAB, cd to this repository, and run main again.\n'], ...
        char(pe.Executable));
end
end

function ensureTRenderer(acadosDir)

if ispc
    rendererName = "t_renderer.exe";
else
    rendererName = "t_renderer";
end

tRenderer = fullfile(acadosDir, "bin", rendererName);
if exist(tRenderer, "file") == 2
    return;
end

if exist(fileparts(tRenderer), "dir") ~= 7
    mkdir(fileparts(tRenderer));
end

url = "https://github.com/acados/tera_renderer/releases/download/v0.2.0/" + ...
    "t_renderer-v0.2.0" + teraRendererSuffix();
fprintf("Downloading acados t_renderer...\n");
downloadedRenderer = websave(tRenderer, url);
if exist(tRenderer, "file") ~= 2
    if exist(downloadedRenderer, "file") == 2
        movefile(downloadedRenderer, tRenderer, "f");
    else
        candidates = dir(fullfile(fileparts(tRenderer), "t_renderer*"));
        candidates = candidates(~[candidates.isdir]);
        if isempty(candidates)
            error("Downloaded t_renderer, but could not find the saved file.");
        end
        [~, newestIdx] = max([candidates.datenum]);
        movefile(fullfile(candidates(newestIdx).folder, ...
            candidates(newestIdx).name), tRenderer, "f");
    end
end

if ~ispc
    fileattrib(tRenderer, "+x");
end
end

function suffix = teraRendererSuffix()

[arch, isArm64] = machineArchitecture();

if ismac
    if isArm64
        suffix = "-osx-arm64";
    else
        suffix = "-osx-amd64";
    end
elseif isunix
    if isArm64
        suffix = "-linux-arm64";
    else
        suffix = "-linux-amd64";
    end
elseif ispc
    if isArm64
        error("acados t_renderer does not provide a Windows arm64 binary.");
    end
    suffix = "-windows-amd64.exe";
else
    error("Unsupported platform for acados t_renderer download.");
end

if strlength(string(suffix)) == 0 || strlength(arch) == 0
    error("Could not determine platform for acados t_renderer download.");
end
end

function [arch, isArm64] = machineArchitecture()

arch = lower(strtrim(string(commandOutput("uname -m"))));
if strlength(arch) == 0
    arch = lower(string(getenv("PROCESSOR_ARCHITECTURE")));
end

isArm64 = any(arch == ["arm64", "aarch64"]);
end

function out = commandOutput(cmd)

[status, outText] = system(char(cmd));
if status ~= 0
    out = "";
else
    out = string(strtrim(outText));
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
