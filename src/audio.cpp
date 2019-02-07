#include "audio.hpp"
#include "string.hpp"
#include "math.hpp"
#include "bridge.hpp"
#include "system.hpp"


namespace rack {
namespace audio {


IO::IO() {
	setDriver(RtAudio::UNSPECIFIED);
}

IO::~IO() {
	closeStream();
}

std::vector<int> IO::getDrivers() {
	std::vector<RtAudio::Api> apis;
	RtAudio::getCompiledApi(apis);
	std::vector<int> drivers;
	for (RtAudio::Api api : apis) {
		drivers.push_back((int) api);
	}
	// Add fake Bridge driver
	drivers.push_back(BRIDGE_DRIVER);
	return drivers;
}

std::string IO::getDriverName(int driver) {
	switch (driver) {
		case RtAudio::UNSPECIFIED: return "Unspecified";
		case RtAudio::LINUX_ALSA: return "ALSA";
		case RtAudio::LINUX_PULSE: return "PulseAudio";
		case RtAudio::LINUX_OSS: return "OSS";
		case RtAudio::UNIX_JACK: return "JACK";
		case RtAudio::MACOSX_CORE: return "Core Audio";
		case RtAudio::WINDOWS_WASAPI: return "WASAPI";
		case RtAudio::WINDOWS_ASIO: return "ASIO";
		case RtAudio::WINDOWS_DS: return "DirectSound";
		case RtAudio::RTAUDIO_DUMMY: return "Dummy Audio";
		case BRIDGE_DRIVER: return "Bridge";
		default: return "Unknown";
	}
}

void IO::setDriver(int driver) {
	// Close device
	setDevice(-1, 0);

	// Close driver
	if (rtAudio) {
		delete rtAudio;
		rtAudio = NULL;
	}
	this->driver = 0;

	// Open driver
	if (driver >= 0) {
		rtAudio = new RtAudio((RtAudio::Api) driver);
		this->driver = (int) rtAudio->getCurrentApi();
	}
	else if (driver == BRIDGE_DRIVER) {
		this->driver = BRIDGE_DRIVER;
	}
}

int IO::getDeviceCount() {
	if (rtAudio) {
		return rtAudio->getDeviceCount();
	}
	else if (driver == BRIDGE_DRIVER) {
		return BRIDGE_NUM_PORTS;
	}
	return 0;
}

bool IO::getDeviceInfo(int device, RtAudio::DeviceInfo *deviceInfo) {
	if (!deviceInfo)
		return false;

	if (rtAudio) {
		if (device == this->device) {
			*deviceInfo = this->deviceInfo;
			return true;
		}
		else {
			try {
				*deviceInfo = rtAudio->getDeviceInfo(device);
				return true;
			}
			catch (RtAudioError &e) {
				WARN("Failed to query RtAudio device: %s", e.what());
			}
		}
	}

	return false;
}

int IO::getDeviceChannels(int device) {
	if (device < 0)
		return 0;

	if (rtAudio) {
		RtAudio::DeviceInfo deviceInfo;
		if (getDeviceInfo(device, &deviceInfo))
			return std::max((int) deviceInfo.inputChannels, (int) deviceInfo.outputChannels);
	}
	else if (driver == BRIDGE_DRIVER) {
		return std::max(BRIDGE_OUTPUTS, BRIDGE_INPUTS);
	}
	return 0;
}

std::string IO::getDeviceName(int device) {
	if (device < 0)
		return "";

	if (rtAudio) {
		RtAudio::DeviceInfo deviceInfo;
		if (getDeviceInfo(device, &deviceInfo))
			return deviceInfo.name;
	}
	else if (driver == BRIDGE_DRIVER) {
		return string::f("%d", device + 1);
	}
	return "";
}

std::string IO::getDeviceDetail(int device, int offset) {
	if (device < 0)
		return "";

	if (rtAudio) {
		RtAudio::DeviceInfo deviceInfo;
		if (getDeviceInfo(device, &deviceInfo)) {
			std::string deviceDetail = string::f("%s (", deviceInfo.name.c_str());
			if (offset < (int) deviceInfo.inputChannels)
				deviceDetail += string::f("%d-%d in", offset + 1, std::min(offset + maxChannels, (int) deviceInfo.inputChannels));
			if (offset < (int) deviceInfo.inputChannels && offset < (int) deviceInfo.outputChannels)
				deviceDetail += ", ";
			if (offset < (int) deviceInfo.outputChannels)
				deviceDetail += string::f("%d-%d out", offset + 1, std::min(offset + maxChannels, (int) deviceInfo.outputChannels));
			deviceDetail += ")";
			return deviceDetail;
		}
	}
	else if (driver == BRIDGE_DRIVER) {
		return string::f("Port %d", device + 1);
	}
	return "";
}

void IO::setDevice(int device, int offset) {
	closeStream();
	this->device = device;
	this->offset = offset;
	openStream();
}

std::vector<int> IO::getSampleRates() {
	if (rtAudio) {
		try {
			RtAudio::DeviceInfo deviceInfo = rtAudio->getDeviceInfo(device);
			std::vector<int> sampleRates(deviceInfo.sampleRates.begin(), deviceInfo.sampleRates.end());
			return sampleRates;
		}
		catch (RtAudioError &e) {
			WARN("Failed to query RtAudio device: %s", e.what());
		}
	}
	return {};
}

void IO::setSampleRate(int sampleRate) {
	if (sampleRate == this->sampleRate)
		return;
	closeStream();
	this->sampleRate = sampleRate;
	openStream();
}

std::vector<int> IO::getBlockSizes() {
	if (rtAudio) {
		return {64, 128, 256, 512, 1024, 2048, 4096};
	}
	return {};
}

void IO::setBlockSize(int blockSize) {
	if (blockSize == this->blockSize)
		return;
	closeStream();
	this->blockSize = blockSize;
	openStream();
}

void IO::setChannels(int numOutputs, int numInputs) {
	this->numOutputs = numOutputs;
	this->numInputs = numInputs;
	onChannelsChange();
}


static int rtCallback(void *outputBuffer, void *inputBuffer, unsigned int nFrames, double streamTime, RtAudioStreamStatus status, void *userData) {
	IO *audioIO = (IO*) userData;
	assert(audioIO);
	// Exploit the stream time to run code on startup of the audio thread
	if (streamTime == 0.0) {
		system::setThreadName("Audio");
		system::setThreadRealTime();
	}
	audioIO->processStream((const float *) inputBuffer, (float *) outputBuffer, nFrames);
	return 0;
}

void IO::openStream() {
	if (device < 0)
		return;

	if (rtAudio) {
		// Open new device
		try {
			deviceInfo = rtAudio->getDeviceInfo(device);
		}
		catch (RtAudioError &e) {
			WARN("Failed to query RtAudio device: %s", e.what());
			return;
		}

		if (rtAudio->isStreamOpen())
			return;

		setChannels(math::clamp((int) deviceInfo.outputChannels - offset, 0, maxChannels), math::clamp((int) deviceInfo.inputChannels - offset, 0, maxChannels));

		if (numOutputs == 0 && numInputs == 0) {
			WARN("RtAudio device %d has 0 inputs and 0 outputs", device);
			return;
		}

		RtAudio::StreamParameters outParameters;
		outParameters.deviceId = device;
		outParameters.nChannels = numOutputs;
		outParameters.firstChannel = offset;

		RtAudio::StreamParameters inParameters;
		inParameters.deviceId = device;
		inParameters.nChannels = numInputs;
		inParameters.firstChannel = offset;

		RtAudio::StreamOptions options;
		options.flags |= RTAUDIO_JACK_DONT_CONNECT;
		options.streamName = "VCV Rack";

		int closestSampleRate = deviceInfo.preferredSampleRate;
		for (int sr : deviceInfo.sampleRates) {
			if (std::abs(sr - sampleRate) < std::abs(closestSampleRate - sampleRate)) {
				closestSampleRate = sr;
			}
		}

		try {
			INFO("Opening audio RtAudio device %d with %d in %d out", device, numInputs, numOutputs);
			rtAudio->openStream(
				numOutputs == 0 ? NULL : &outParameters,
				numInputs == 0 ? NULL : &inParameters,
				RTAUDIO_FLOAT32, closestSampleRate, (unsigned int*) &blockSize,
				&rtCallback, this, &options, NULL);
		}
		catch (RtAudioError &e) {
			WARN("Failed to open RtAudio stream: %s", e.what());
			return;
		}

		try {
			INFO("Starting RtAudio stream %d", device);
			rtAudio->startStream();
		}
		catch (RtAudioError &e) {
			WARN("Failed to start RtAudio stream: %s", e.what());
			return;
		}

		// Update sample rate because this may have changed
		this->sampleRate = rtAudio->getStreamSampleRate();
		onOpenStream();
	}
	else if (driver == BRIDGE_DRIVER) {
		setChannels(BRIDGE_OUTPUTS, BRIDGE_INPUTS);
		bridgeAudioSubscribe(device, this);
	}
}

void IO::closeStream() {
	setChannels(0, 0);

	if (rtAudio) {
		if (rtAudio->isStreamRunning()) {
			INFO("Stopping RtAudio stream %d", device);
			try {
				rtAudio->stopStream();
			}
			catch (RtAudioError &e) {
				WARN("Failed to stop RtAudio stream %s", e.what());
			}
		}
		if (rtAudio->isStreamOpen()) {
			INFO("Closing RtAudio stream %d", device);
			try {
				rtAudio->closeStream();
			}
			catch (RtAudioError &e) {
				WARN("Failed to close RtAudio stream %s", e.what());
			}
		}
		deviceInfo = RtAudio::DeviceInfo();
	}
	else if (driver == BRIDGE_DRIVER) {
		bridgeAudioUnsubscribe(device, this);
	}

	onCloseStream();
}

json_t *IO::toJson() {
	json_t *rootJ = json_object();
	json_object_set_new(rootJ, "driver", json_integer(driver));
	std::string deviceName = getDeviceName(device);
	if (!deviceName.empty())
		json_object_set_new(rootJ, "deviceName", json_string(deviceName.c_str()));
	json_object_set_new(rootJ, "offset", json_integer(offset));
	json_object_set_new(rootJ, "maxChannels", json_integer(maxChannels));
	json_object_set_new(rootJ, "sampleRate", json_integer(sampleRate));
	json_object_set_new(rootJ, "blockSize", json_integer(blockSize));
	return rootJ;
}

void IO::fromJson(json_t *rootJ) {
	closeStream();

	json_t *driverJ = json_object_get(rootJ, "driver");
	if (driverJ)
		setDriver(json_number_value(driverJ));

	json_t *deviceNameJ = json_object_get(rootJ, "deviceName");
	if (deviceNameJ) {
		std::string deviceName = json_string_value(deviceNameJ);
		// Search for device ID with equal name
		for (int device = 0; device < getDeviceCount(); device++) {
			if (getDeviceName(device) == deviceName) {
				this->device = device;
				break;
			}
		}
	}

	json_t *offsetJ = json_object_get(rootJ, "offset");
	if (offsetJ)
		offset = json_integer_value(offsetJ);

	json_t *maxChannelsJ = json_object_get(rootJ, "maxChannels");
	if (maxChannelsJ)
		maxChannels = json_integer_value(maxChannelsJ);

	json_t *sampleRateJ = json_object_get(rootJ, "sampleRate");
	if (sampleRateJ)
		sampleRate = json_integer_value(sampleRateJ);

	json_t *blockSizeJ = json_object_get(rootJ, "blockSize");
	if (blockSizeJ)
		blockSize = json_integer_value(blockSizeJ);

	openStream();
}


} // namespace audio
} // namespace rack
