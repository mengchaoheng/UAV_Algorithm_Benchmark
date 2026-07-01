function install_acados(varargin)
%INSTALL_ACADOS Clone/build acados and prepare this repository to use it.
%
% Usage:
%   install_acados
%   install_acados("RunSunSmokeTest", false)
%   install_acados("AcadosDir", "/path/to/acados")
%   install_acados("Python", "/path/to/python3")
%
% The default install is repository-local:
%   .acados/acados   acados source/build/install tree
%   .venv            Python environment used by MATLAB

repoDir = fileparts(mfilename("fullpath"));
if isempty(repoDir)
    repoDir = pwd;
end

parser = inputParser;
parser.addParameter("AcadosDir", fullfile(repoDir, ".acados", "acados"));
parser.addParameter("Python", defaultAcadosPython());
parser.addParameter("Jobs", defaultParallelJobs());
parser.addParameter("RunSunSmokeTest", true);
parser.parse(varargin{:});
opts = parser.Results;

acadosDir = char(string(opts.AcadosDir));
buildDir = fullfile(acadosDir, "build");
jobs = max(1, round(double(opts.Jobs)));
runSunSmokeTest = logical(opts.RunSunSmokeTest);

fprintf("acados local installation\n");
fprintf("Repository: %s\n", repoDir);
fprintf("acados:    %s\n", acadosDir);
fprintf("Jobs:      %d\n", jobs);

ensureCommandAvailable("git --version", "git");
ensureCommandAvailable("cmake --version", "cmake");

ensureAcadosSource(acadosDir);
configureAndBuildAcados(acadosDir, buildDir, jobs);

setenv("ACADOS_SOURCE_DIR", acadosDir);
setenv("ACADOS_INSTALL_DIR", acadosDir);
if strlength(string(opts.Python)) > 0
    setenv("ACADOS_PYTHON", char(string(opts.Python)));
end

setup_acados_python();

if runSunSmokeTest
    runSunNMPCSmokeTest(repoDir);
end

fprintf("acados is installed and ready for this repository.\n");
end

function ensureAcadosSource(acadosDir)

cmakeFile = fullfile(acadosDir, "CMakeLists.txt");
if exist(cmakeFile, "file") == 2
    fprintf("acados source already exists. Updating submodules...\n");
    if isGitCheckout(acadosDir)
        runCommand("git -C " + shellQuote(acadosDir) + ...
            " submodule update --init --recursive");
    end
    return;
end

if exist(acadosDir, "dir") == 7 && ~isempty(dir(acadosDir))
    error(["acados directory exists but does not look like an acados " ...
           "checkout:\n  %s\nChoose another AcadosDir or remove/fix it."], ...
          acadosDir);
end

parentDir = fileparts(acadosDir);
if exist(parentDir, "dir") ~= 7
    mkdir(parentDir);
end

fprintf("Cloning acados...\n");
runCommand("git clone --depth 1 --recurse-submodules " + ...
    "https://github.com/acados/acados.git " + shellQuote(acadosDir));
end

function configureAndBuildAcados(acadosDir, buildDir, jobs)

cmakeOptions = acadosCMakeOptions(acadosDir);
fprintf("Configuring acados with platform options:\n  %s\n", ...
    strjoin(cmakeOptions, " "));

runCommand("cmake -S " + shellQuote(acadosDir) + ...
    " -B " + shellQuote(buildDir) + " " + strjoin(cmakeOptions, " "));
runCommand("cmake --build " + shellQuote(buildDir) + ...
    " --target install -j" + string(jobs));

if ismac
    fixMacOSDylibRPath(acadosDir);
end

requiredLibs = ["libacados", "libblasfeo", "libhpipm"];
for i = 1:numel(requiredLibs)
    libPattern = fullfile(acadosDir, "lib", requiredLibs(i) + ".*");
    if isempty(dir(libPattern))
        error("Expected acados library was not installed: %s", libPattern);
    end
end
end

function fixMacOSDylibRPath(acadosDir)

libDir = fullfile(acadosDir, "lib");
dylibs = dir(fullfile(libDir, "*.dylib"));

for i = 1:numel(dylibs)
    dylibPath = fullfile(dylibs(i).folder, dylibs(i).name);
    [status, out] = system(char("otool -l " + shellQuote(dylibPath)));
    if status ~= 0
        error("Could not inspect dylib rpaths:\n%s", out);
    end
    if ~contains(string(out), "path @loader_path")
        runCommand("install_name_tool -add_rpath @loader_path " + ...
            shellQuote(dylibPath));
    end
end
end

function opts = acadosCMakeOptions(acadosDir)

opts = [
    "-DACADOS_WITH_QPOASES=ON"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_INSTALL_PREFIX=" + shellQuote(acadosDir)
    ];

[arch, isArm64] = machineArchitecture();

if ismac && isArm64
    opts = [opts
        "-DBLASFEO_TARGET=ARMV8A_APPLE_M1"
        "-DCMAKE_OSX_ARCHITECTURES=arm64"];
elseif isunix && isArm64
    opts = [opts
        "-DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57"];
elseif any(arch == ["x86_64", "amd64"])
    opts = [opts
        "-DBLASFEO_TARGET=X64_AUTOMATIC"];
else
    opts = [opts
        "-DBLASFEO_TARGET=GENERIC"];
end
end

function runSunNMPCSmokeTest(repoDir)

fprintf("Running short sun_nmpc smoke test...\n");
clear("main");

UAV_BENCHMARK_BATCH = true; %#ok<NASGU>
override = struct();
override.controllerName = "sun_nmpc";
override.trajName = "fast_circle";
override.Tend = 0.02;
override.progress.mode = "scale_fixed";
override.progress.scale = 1;
override.enablePlots = false;
override.saveResults = false;
override.saveFigures = false;
override.saveMat = false;
override.disturbance.enabled = false;
override.sun.N = 5;
override.sun.dt = 0.05;
override.sun.printSolverTiming = true;
override.sun.acadosCodegenDir = tempname(tempdir);
UAV_BENCHMARK_PAR_OVERRIDE = override; %#ok<NASGU>

run(fullfile(repoDir, "main.m"));
fprintf("sun_nmpc smoke test passed.\n");
end

function ensureCommandAvailable(versionCommand, commandName)

[status, out] = system(char(versionCommand));
if status ~= 0
    error("Required command is not available: %s\n%s", commandName, out);
end
end

function tf = isGitCheckout(pathName)

tf = exist(fullfile(pathName, ".git"), "dir") == 7 ...
    || exist(fullfile(pathName, ".git"), "file") == 2;
end

function pythonExe = defaultAcadosPython()

pythonExe = string(getenv("ACADOS_PYTHON"));
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

function n = defaultParallelJobs()

try
    n = max(1, min(feature("numcores"), 8));
catch
    n = 4;
end
end

function runCommand(cmd)

cmd = string(cmd);
fprintf("%s\n", cmd);
[status, out] = system(char(cmd));
fprintf("%s", out);
if status ~= 0
    error("Command failed:\n%s", cmd);
end
end

function quoted = shellQuote(pathName)

quoted = """" + replace(string(pathName), """", "\""") + """";
end
