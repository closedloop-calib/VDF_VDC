// Standalone computational-cost benchmark for HALCON 24.05.
// In a Visual Studio Console App, compile this file and HalconCalibrationData.cpp
// with the HALCON C++ include and link settings. Use Release / x64 and the
// Local Windows Debugger. The default input is hand_eye_calibration_input.txt.
// Command arguments: [inputFile [timedRuns]]

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#pragma comment(lib, "Psapi.lib")
#else
#include <cstdlib>
#include <memory>
#include <sys/stat.h>
#endif

#include "HalconCalibrationData.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int imageWidth = 4384;
constexpr int imageHeight = 3288;
constexpr int defaultTimedRuns = 100;

#ifdef _WIN32
using InputPath = std::wstring;
constexpr wchar_t defaultInputName[] = L"hand_eye_calibration_input.txt";
constexpr wchar_t matlabFolder[] = L"VDF_VDC";
constexpr wchar_t pathSeparators[] = L"\\/";
constexpr wchar_t pathSeparator = L'\\';
#else
using InputPath = std::string;
constexpr char defaultInputName[] = "hand_eye_calibration_input.txt";
constexpr char matlabFolder[] = "VDF_VDC";
constexpr char pathSeparators[] = "/";
constexpr char pathSeparator = '/';
#endif

InputPath appendPath(const InputPath& directory, const InputPath& name) {
    if (directory.empty() || directory.find_last_of(pathSeparators) == directory.size() - 1)
        return directory + name;
    return directory + pathSeparator + name;
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

#ifdef _WIN32
InputPath absolutePath(const InputPath& path) {
    const DWORD capacity = GetFullPathNameW(path.c_str(), 0, nullptr, nullptr);
    if (capacity == 0) return path;
    InputPath fullPath(capacity, L'\0');
    const DWORD length = GetFullPathNameW(path.c_str(), capacity, &fullPath[0], nullptr);
    if (length == 0 || length >= capacity) return path;
    fullPath.resize(length);
    return fullPath;
}

InputPath executablePath(const wchar_t*) {
    InputPath path(32768, L'\0');
    const DWORD size = GetModuleFileNameW(nullptr, &path[0], static_cast<DWORD>(path.size()));
    if (size == 0 || size >= path.size())
        throw std::runtime_error("Cannot determine the executable path.");
    path.resize(size);
    return path;
}

std::string displayPath(const InputPath& path) {
    const int size = WideCharToMultiByte(CP_UTF8, 0, path.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (size <= 0) throw std::runtime_error("Cannot convert the input path to UTF-8.");
    std::string utf8(static_cast<std::size_t>(size), '\0');
    if (!WideCharToMultiByte(CP_UTF8, 0, path.c_str(), -1, &utf8[0], size, nullptr, nullptr))
        throw std::runtime_error("Cannot convert the input path to UTF-8.");
    utf8.pop_back();
    return utf8;
}

double peakWorkingSetMB() {
    PROCESS_MEMORY_COUNTERS counters{};
    if (!GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)))
        throw std::runtime_error("Cannot read the process peak working set.");
    return static_cast<double>(counters.PeakWorkingSetSize) / (1024.0 * 1024.0);
}
#else
InputPath absolutePath(const InputPath& path) {
    const std::unique_ptr<char, decltype(&std::free)> resolved(realpath(path.c_str(), nullptr), &std::free);
    return resolved ? InputPath(resolved.get()) : path;
}

InputPath executablePath(const char* path) { return absolutePath(path); }
const InputPath& displayPath(const InputPath& path) { return path; }
#endif

bool isInputFile(const InputPath& path) {
#ifdef _WIN32
    const DWORD attributes = GetFileAttributesW(path.c_str());
    return attributes != INVALID_FILE_ATTRIBUTES && !(attributes & FILE_ATTRIBUTE_DIRECTORY);
#else
    struct stat status{};
    return stat(path.c_str(), &status) == 0 && S_ISREG(status.st_mode);
#endif
}

InputPath locateInput(const InputPath& executable) {
#ifdef _WIN32
    const InputPath currentDirectory = absolutePath(L".");
#else
    const InputPath currentDirectory = absolutePath(".");
#endif
    for (auto directory : {currentDirectory, parentDirectory(executable)}) {
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
    throw std::runtime_error("Cannot find hand_eye_calibration_input.txt. Put it in the project directory "
                             "or pass its path as the first command argument.");
}

void printToDebugger(const std::string& text) {
#ifdef _WIN32
    if (!IsDebuggerPresent()) return;
    const int length = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, nullptr, 0);
    if (length <= 0) return;
    std::wstring wide(static_cast<std::size_t>(length), L'\0');
    if (MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, &wide[0], length))
        OutputDebugStringW(wide.c_str());
#else
    static_cast<void>(text);
#endif
}

int parseTimedRuns(const InputPath& argument) {
    std::size_t consumed = 0;
    int runs = 0;
    try {
        runs = std::stoi(argument, &consumed);
    } catch (const std::logic_error&) {
        throw std::invalid_argument("timedRuns must be an integer from 2 to 10000.");
    }
    if (consumed != argument.size() || runs < 2 || runs > 10000)
        throw std::invalid_argument("timedRuns must be an integer from 2 to 10000.");
    return runs;
}

int runHalconBenchmark(const InputPath& inputFile, int timedRuns) {
    std::ifstream input(inputFile.c_str());
    if (!input) throw std::runtime_error("Cannot open input file: " + displayPath(inputFile));
    const auto data = halcon_benchmark::readCalibrationData(input);
    input.close();
    const auto handEye = halcon_benchmark::prepareHandEyeData(data, imageWidth, imageHeight);
#ifdef _WIN32
    const double preparedPeak = peakWorkingSetMB();
#endif

    HalconCpp::HTuple cameraPose, calibrationPose, quality;
    const auto calibrate = [&] {
        HalconCpp::HandEyeCalibration(
            handEye.worldX, handEye.worldY, handEye.worldZ,
            handEye.imageRows, handEye.imageColumns, handEye.pointsPerView,
            handEye.robotPoses, handEye.cameraParameters, "nonlinear", "error_pose",
            &cameraPose, &calibrationPose, &quality);
    };

    calibrate(); // One untimed warmup, matching the VDF-VDC benchmark.
    using Clock = std::chrono::steady_clock;
    std::vector<double> times;
    times.reserve(static_cast<std::size_t>(timedRuns));
    for (int run = 0; run < timedRuns; ++run) {
        const auto start = Clock::now();
        calibrate();
        const auto end = Clock::now();
        times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    }
    const double mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
#ifdef _WIN32
    const double processPeak = peakWorkingSetMB();
#endif
    double sumOfSquares = 0.0;
    for (double elapsed : times) {
        const double difference = elapsed - mean;
        sumOfSquares += difference * difference;
    }
    const double sampleStdDev = std::sqrt(sumOfSquares / (times.size() - 1));

    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "Input: " << displayPath(inputFile)
           << "\nViews: " << data.views << " | Points per view: " << data.pointsPerView
           << " | Image size: " << imageWidth << " x " << imageHeight << "\n\n"
           << "Computational cost comparison - HALCON row\n"
           << std::left << std::setw(27) << "Implementation"
           << std::right << std::setw(25) << "Calibration time (ms)"
           << std::setw(28) << "Peak memory increase (MB)"
           << std::setw(28) << "Peak process memory (MB)" << '\n'
           << std::string(108, '-') << '\n';
    std::ostringstream calibrationTime;
    calibrationTime.imbue(std::locale::classic());
    calibrationTime << std::fixed << std::setprecision(3) << mean << " \xc2\xb1 " << sampleStdDev;
    output << std::left << std::setw(27) << "HALCON"
           << std::right << std::setw(26) << calibrationTime.str()
           << std::fixed << std::setprecision(3);
#ifdef _WIN32
    output << std::setw(28) << processPeak - preparedPeak << std::setw(28) << processPeak;
#else
    output << std::setw(28) << "N/A" << std::setw(28) << "N/A";
#endif
    output << "\n\n" << timedRuns << " timed runs after 1 warmup run. "
           << "Time includes the nonlinear calibration call.\n";
#ifdef _WIN32
    output << "Both memory columns use the process peak working set. "
           << "The increase is relative to the peak after input preparation.\n";
#else
    output << "Windows working-set measurements are unavailable on this platform.\n";
#endif
    const auto text = output.str();
    std::cout << text << std::flush;
    printToDebugger(text);
    return 0;
}

} // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[]) {
    SetConsoleOutputCP(CP_UTF8);
#else
int main(int argc, char* argv[]) {
#endif
    try {
        if (argc > 3) throw std::invalid_argument("Usage: RunHalconBenchmark [inputFile [timedRuns]]");
        const auto inputFile = argc >= 2 ? absolutePath(argv[1]) : locateInput(executablePath(argv[0]));
        const int timedRuns = argc == 3 ? parseTimedRuns(argv[2]) : defaultTimedRuns;
        return runHalconBenchmark(inputFile, timedRuns);
    } catch (const HalconCpp::HException& error) {
        const std::string message = std::string("HALCON error: ") + error.ErrorMessage().Text() + '\n';
        std::cerr << message;
        printToDebugger(message);
    } catch (const std::exception& error) {
        const std::string message = std::string("Benchmark error: ") + error.what() + '\n';
        std::cerr << message;
        printToDebugger(message);
    }
    return 1;
}
