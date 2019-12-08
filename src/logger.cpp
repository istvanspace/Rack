#include <common.hpp>
#include <asset.hpp>
#include <settings.hpp>
#include <system.hpp>
#include <mutex>


namespace rack {
namespace logger {


static FILE* outputFile = NULL;
static long startTime = 0;
static std::mutex logMutex;


void init() {
	startTime = system::getNanoseconds();
	if (settings::devMode) {
		outputFile = stderr;
		return;
	}

	outputFile = fopen(asset::logPath.c_str(), "w");
	if (!outputFile) {
		fprintf(stderr, "Could not open log at %s\n", asset::logPath.c_str());
	}
}

void destroy() {
	if (outputFile != stderr) {
		fclose(outputFile);
	}
}

static const char* const levelLabels[] = {
	"debug",
	"info",
	"warn",
	"fatal"
};

static const int levelColors[] = {
	35,
	34,
	33,
	31
};

static void logVa(Level level, const char* filename, int line, const char* format, va_list args) {
	std::lock_guard<std::mutex> lock(logMutex);

	long nowTime = system::getNanoseconds();
	double duration = (nowTime - startTime) / 1e9;
	if (outputFile == stderr)
		fprintf(outputFile, "\x1B[%dm", levelColors[level]);
	fprintf(outputFile, "[%.03f %s %s:%d] ", duration, levelLabels[level], filename, line);
	if (outputFile == stderr)
		fprintf(outputFile, "\x1B[0m");
	vfprintf(outputFile, format, args);
	fprintf(outputFile, "\n");
	fflush(outputFile);
}

void log(Level level, const char* filename, int line, const char* format, ...) {
	va_list args;
	va_start(args, format);
	logVa(level, filename, line, format, args);
	va_end(args);
}


} // namespace logger
} // namespace rack
