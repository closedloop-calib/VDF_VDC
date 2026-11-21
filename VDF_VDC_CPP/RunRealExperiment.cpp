// Visual Studio 2022: add the three .cpp files to a Console App and put Eigen
// on its include path. This code supports C++14. Select Release / x64.
// Press Local Windows Debugger to run all four VDF-VDC implementations.
// The program searches for hand_eye_calibration_input.txt in parent folders
// of the working directory and executable. An input path can also be supplied
// under Project Properties > Debugging > Command Arguments.

#include "RunRealExperiment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <cstdlib>
#include <memory>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#pragma comment(lib, "Psapi.lib")
#else
#include <sys/stat.h>
#endif

namespace vdf_vdc {
namespace {

constexpr OptimizationSettings settings{};
#ifdef _WIN32
constexpr wchar_t defaultInputName[] = L"hand_eye_calibration_input.txt";
constexpr wchar_t matlabFolder[] = L"VDF_VDC";
constexpr wchar_t pathSeparators[] = L"\\/";
constexpr wchar_t pathSeparator = L'\\';
#else
constexpr char defaultInputName[] = "hand_eye_calibration_input.txt";
constexpr char matlabFolder[] = "VDF_VDC";
constexpr char pathSeparators[] = "/";
constexpr char pathSeparator = '/';
#endif

const char* implementationName(SolverMode mode) {
    // UTF-8 en dash.
    switch (mode) {
    case SolverMode::DenseFull: return "Dense\xe2\x80\x93" "full";
    case SolverMode::SparseFull: return "Sparse\xe2\x80\x93" "full";
    case SolverMode::DenseSchur: return "Dense\xe2\x80\x93" "Schur";
    case SolverMode::SparseSchur: return "Sparse\xe2\x80\x93" "Schur (proposed)";
    }
    throw std::invalid_argument("Unknown implementation.");
}

#ifdef _WIN32
double peakWorkingSetMB() {
    PROCESS_MEMORY_COUNTERS info{};
    if (!GetProcessMemoryInfo(GetCurrentProcess(), &info, sizeof(info)))
        throw std::runtime_error("Cannot measure the process peak working set.");
    return static_cast<double>(info.PeakWorkingSetSize) / (1024.0 * 1024.0);
}
#endif

bool matchingResults(const BenchmarkResult& first, const BenchmarkResult& second) {
    const auto& a = first.calibration;
    const auto& b = second.calibration;
    const double referenceCost = first.solverStats.finalCost;
    return (a.boardInBase - b.boardInBase).norm() <= 1e-8 &&
           (a.cameraInTcp - b.cameraInTcp).norm() <= 1e-8 &&
           std::abs(referenceCost - second.solverStats.finalCost) <=
               1e-8 * (1.0 + std::abs(referenceCost));
}

std::string formatMetric(double value, double reference, bool showReduction, std::string suffix = {}) {
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << std::fixed << std::setprecision(3) << value << suffix;
    if (showReduction && value != 0.0) {
        const double reduction = 100.0 * (value - reference) / value;
        text << " (" << std::setprecision(1) << reduction << "%)";
    }
    return text.str();
}

InputPath appendPath(const InputPath& directory, const InputPath& filename) {
    if (directory.empty() || directory.find_last_of(pathSeparators) == directory.size() - 1)
        return directory + filename;
    return directory + pathSeparator + filename;
}

InputPath parentDirectory(const InputPath& path) {
    const auto end = path.find_last_not_of(pathSeparators);
    if (end == InputPath::npos) return {};
    const auto separator = path.find_last_of(pathSeparators, end);
    if (separator == InputPath::npos) return {};
#ifdef _WIN32
    if (separator == 2 && path[1] == L':') return path.substr(0, 3);
#endif
    return path.substr(0, separator == 0 ? 1 : separator);
}

bool isInputFile(const InputPath& path) {
#ifdef _WIN32
    const DWORD attributes = GetFileAttributesW(path.c_str());
    return attributes != INVALID_FILE_ATTRIBUTES && !(attributes & FILE_ATTRIBUTE_DIRECTORY);
#else
    struct stat status{};
    return stat(path.c_str(), &status) == 0 && S_ISREG(status.st_mode);
#endif
}

InputPath absolutePath(const InputPath& path) {
#ifdef _WIN32
    const DWORD capacity = GetFullPathNameW(path.c_str(), 0, nullptr, nullptr);
    if (capacity == 0) return path;
    InputPath fullPath(capacity, L'\0');
    const DWORD length = GetFullPathNameW(path.c_str(), capacity, &fullPath[0], nullptr);
    if (length == 0 || length >= capacity) return path;
    fullPath.resize(length);
    return fullPath;
#else
    const std::unique_ptr<char, decltype(&std::free)> fullPath(realpath(path.c_str(), nullptr), &std::free);
    return fullPath ? InputPath(fullPath.get()) : path;
#endif
}

InputPath locateInput(const InputPath& executable) {
    const std::array<InputPath, 2> startDirectories{
#ifdef _WIN32
        absolutePath(L"."),
#else
        absolutePath("."),
#endif
        parentDirectory(absolutePath(executable))
    };
    for (auto directory : startDirectories) {
        while (!directory.empty()) {
            for (const auto& parent : {directory, appendPath(directory, matlabFolder)}) {
                const auto input = appendPath(parent, defaultInputName);
                if (isInputFile(input)) return input;
            }
            const auto parent = parentDirectory(directory);
            if (parent == directory) break;
            directory = parent;
        }
    }
    throw std::runtime_error(
        "Cannot find hand_eye_calibration_input.txt. Place it in the project folder "
        "or specify its path in the Visual Studio command arguments.");
}

#ifdef _WIN32
class WindowsHandle {
public:
    explicit WindowsHandle(HANDLE value) : value_(value) {}
    WindowsHandle(const WindowsHandle&) = delete;
    WindowsHandle& operator=(const WindowsHandle&) = delete;
    ~WindowsHandle() { close(); }
    HANDLE get() const { return value_; }
    void close() {
        if (value_ && value_ != INVALID_HANDLE_VALUE) CloseHandle(value_);
        value_ = nullptr;
    }
private:
    HANDLE value_;
};

InputPath executablePath() {
    std::wstring path(32768, L'\0');
    const DWORD size = GetModuleFileNameW(nullptr, &path[0], static_cast<DWORD>(path.size()));
    if (size == 0 || size == path.size())
        throw std::runtime_error("Cannot determine the executable path.");
    path.resize(size);
    return path;
}

void writeWorkerResult(std::ostream& output, const BenchmarkResult& result) {
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << result.timeMeanMs << ' ' << result.timeStdMs << ' '
           << result.peakMemoryIncreaseMB << ' ' << result.peakProcessMemoryMB << ' '
           << result.solverStats.finalCost << ' ' << result.solverStats.lmIterations << ' '
           << result.solverStats.acceptedSteps << '\n';
    for (const auto* transform : {&result.calibration.boardInBase, &result.calibration.cameraInTcp}) {
        for (Eigen::Index i = 0; i < transform->size(); ++i) output << transform->data()[i] << ' ';
        output << '\n';
    }
}

BenchmarkResult runWindowsWorker(const InputPath& executable,
                                 const InputPath& inputFile, std::size_t modeIndex) {
    SECURITY_ATTRIBUTES inherit{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    HANDLE readEnd = nullptr;
    HANDLE writeEnd = nullptr;
    if (!CreatePipe(&readEnd, &writeEnd, &inherit, 0))
        throw std::runtime_error("Cannot create the benchmark output pipe.");
    WindowsHandle reader(readEnd);
    WindowsHandle writer(writeEnd);
    if (!SetHandleInformation(reader.get(), HANDLE_FLAG_INHERIT, 0))
        throw std::runtime_error("Cannot configure the benchmark output pipe.");
    WindowsHandle input(CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                    &inherit, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr));
    if (input.get() == INVALID_HANDLE_VALUE)
        throw std::runtime_error("Cannot configure benchmark worker input.");

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = input.get();
    startup.hStdOutput = writer.get();
    startup.hStdError = writer.get();
    PROCESS_INFORMATION processInfo{};
    // Both paths are filenames, which cannot contain a quote on Windows.
    std::wstring command = L"\"" + executable + L"\" --benchmark-worker " +
                           std::to_wstring(modeIndex) + L" \"" + inputFile + L"\"";
    if (!CreateProcessW(executable.c_str(), &command[0], nullptr, nullptr, TRUE, CREATE_NO_WINDOW,
                        nullptr, nullptr, &startup, &processInfo))
        throw std::runtime_error("Cannot start the benchmark for " +
                                 std::string(implementationName(benchmarkModes[modeIndex])) + ".");
    WindowsHandle process(processInfo.hProcess);
    WindowsHandle thread(processInfo.hThread);
    writer.close();

    std::string reply;
    char buffer[4096];
    DWORD bytesRead = 0;
    while (ReadFile(reader.get(), buffer, sizeof(buffer), &bytesRead, nullptr) && bytesRead > 0)
        reply.append(buffer, bytesRead);
    if (WaitForSingleObject(process.get(), INFINITE) != WAIT_OBJECT_0)
        throw std::runtime_error("Cannot wait for a benchmark worker.");
    DWORD exitCode = 0;
    if (!GetExitCodeProcess(process.get(), &exitCode) || exitCode != 0)
        throw std::runtime_error(std::string(implementationName(benchmarkModes[modeIndex])) + ": " + reply);

    BenchmarkResult result;
    result.hasMemoryMeasurement = true;
    std::istringstream rows(reply);
    rows.imbue(std::locale::classic());
    rows >> result.timeMeanMs >> result.timeStdMs >> result.peakMemoryIncreaseMB >> result.peakProcessMemoryMB
         >> result.solverStats.finalCost >> result.solverStats.lmIterations >> result.solverStats.acceptedSteps;
    for (auto* transform : {&result.calibration.boardInBase, &result.calibration.cameraInTcp})
        for (Eigen::Index i = 0; i < transform->size(); ++i) rows >> transform->data()[i];
    if (!rows) throw std::runtime_error("Invalid output from a benchmark worker.");
    return result;
}

void printToDebugger(const std::string& text) {
    if (!IsDebuggerPresent()) return;
    const int length = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, nullptr, 0);
    if (length <= 0) return;
    std::wstring wide(static_cast<std::size_t>(length), L'\0');
    if (MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, &wide[0], length) > 0)
        OutputDebugStringW(wide.c_str());
}
#endif

} // namespace

BenchmarkResult benchmarkCalibration(const CalibrationData& data, SolverMode mode) {
    using Clock = std::chrono::steady_clock;
    BenchmarkResult result;
    BenchmarkResult repeat;
    std::array<double, timedRuns> times{};
#ifdef _WIN32
    // Sample after data preparation, before calibration, as in the paper.
    const double preparedPeak = peakWorkingSetMB();
#endif
    for (int run = -warmupRuns; run < timedRuns; ++run) {
        const auto start = Clock::now();
        const Calibration initial = Li(data.baseInTcp, data.boardInCamera);
        repeat.calibration = VDF_VDC(data, initial, mode, &repeat.solverStats, settings);
        const auto finish = Clock::now();
        if (run < 0) {
            result = repeat;
        } else {
            times[run] = std::chrono::duration<double, std::milli>(finish - start).count();
            if (!matchingResults(result, repeat))
                throw std::runtime_error("Repeated runs produced different calibration results.");
        }
    }
#ifdef _WIN32
    result.peakProcessMemoryMB = peakWorkingSetMB();
    result.peakMemoryIncreaseMB = result.peakProcessMemoryMB - preparedPeak;
    result.hasMemoryMeasurement = true;
#endif
    result.timeMeanMs = std::accumulate(times.begin(), times.end(), 0.0) / timedRuns;
    double squaredDifference = 0.0;
    for (double time : times) {
        const double difference = time - result.timeMeanMs;
        squaredDifference += difference * difference;
    }
    result.timeStdMs = std::sqrt(squaredDifference / (timedRuns - 1));
    return result;
}

void printComputationalCostComparison(
    std::ostream& output, const std::array<BenchmarkResult, benchmarkModes.size()>& results) {
    const auto& proposed = results.back();
    // Widths below are expressed in bytes, except for the first UTF-8 column.
    const auto row = [&](const std::string& name, const std::string& time,
                         const std::string& increase, const std::string& memory) {
        output << std::left << std::setw(name.find("\xe2\x80\x93") == std::string::npos ? 27 : 29) << name
               << std::right << std::setw(25 + (time.find("\xc2\xb1") != std::string::npos)) << time
               << std::setw(28) << increase << std::setw(28) << memory << '\n';
    };
    output << "Computational cost comparison with ablation\n";
    row("Implementation", "Calibration time (ms)", "Peak memory increase (MB)", "Peak process memory (MB)");
    output << std::string(108, '-') << '\n';
    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& result = results[i];
        const bool compare = i + 1 != results.size();
        const std::string time = formatMetric(result.timeMeanMs, proposed.timeMeanMs, compare,
                                               " \xc2\xb1 " + formatMetric(result.timeStdMs, 0.0, false));
        const std::string increase = result.hasMemoryMeasurement
            ? formatMetric(result.peakMemoryIncreaseMB, proposed.peakMemoryIncreaseMB, compare) : "N/A";
        const std::string memory = result.hasMemoryMeasurement
            ? formatMetric(result.peakProcessMemoryMB, proposed.peakProcessMemoryMB, compare) : "N/A";
        row(implementationName(benchmarkModes[i]), time, increase, memory);
    }
    output << '\n'
           << "C++: " << timedRuns << " timed runs after " << warmupRuns
           << " warmup run. Time includes Li initialization.\n"
           << "Parentheses give reductions achieved by Sparse\xe2\x80\x93" "Schur relative to each comparison row.\n";
#ifndef _WIN32
    output << "Windows working-set measurements are unavailable on this platform.\n";
#endif
}

int runRealExperiment(const InputPath& executable, const InputPath& inputFile) {
    const auto data = readCalibrationData(inputFile);
    std::cout << "Input: " << inputPathToUtf8(inputFile) << "\nViews: " << data.baseInTcp.size()
              << " | Points per view: " << data.worldPoints.rows()
              << " | Normal equation dimension: " << 12 + 6 * data.baseInTcp.size()
              << "\nOuter iterations: " << settings.outerLoops << " | LM iterations per round: "
              << settings.lmMaxIterations << " | LM stopping tolerance: " << settings.lmTolerance
              << "\n\n" << std::flush;
    std::array<BenchmarkResult, benchmarkModes.size()> results;
    for (std::size_t i = 0; i < benchmarkModes.size(); ++i) {
        std::cout << "Running " << implementationName(benchmarkModes[i]) << "...\n" << std::flush;
#ifdef _WIN32
        results[i] = runWindowsWorker(executable, inputFile, i);
#else
        static_cast<void>(executable);
        results[i] = benchmarkCalibration(data, benchmarkModes[i]);
#endif
    }
    for (std::size_t i = 0; i + 1 < results.size(); ++i) {
        if (!matchingResults(results.back(), results[i])) {
            std::cerr << "Calibration differs between " << implementationName(benchmarkModes[i])
                      << " and " << implementationName(benchmarkModes.back()) << ".\n";
            throw std::runtime_error("The four implementations must agree to compare their cost.");
        }
    }
    std::ostringstream table;
    table.imbue(std::locale::classic());
    table << '\n';
    printComputationalCostComparison(table, results);
    const auto text = table.str();
    std::cout << text << std::flush;
#ifdef _WIN32
    printToDebugger(text);
#endif
    return 0;
}

} // namespace vdf_vdc

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[]) {
#else
int main(int argc, char* argv[]) {
#endif
    using namespace vdf_vdc;
    try {
#ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);
        if (argc == 4 && std::wstring(argv[1]) == L"--benchmark-worker") {
            const std::wstring mode(argv[2]);
            if (mode.size() != 1 || mode[0] < L'0' || mode[0] > L'3')
                throw std::invalid_argument("Invalid benchmark worker mode.");
            const auto data = readCalibrationData(argv[3]);
            writeWorkerResult(std::cout, benchmarkCalibration(data, benchmarkModes[mode[0] - L'0']));
            return 0;
        }
        const auto executable = executablePath();
#else
        const InputPath executable = argv[0];
#endif
        if (argc > 2) throw std::invalid_argument("Usage: RunRealExperiment [inputFile]");
        const auto inputFile = argc == 2 ? absolutePath(argv[1]) : locateInput(executable);
        return runRealExperiment(executable, inputFile);
    } catch (const std::exception& error) {
        const std::string message = std::string("VDF-VDC: ") + error.what() + '\n';
        std::cerr << message;
#ifdef _WIN32
        printToDebugger(message);
#endif
        return 1;
    }
}
