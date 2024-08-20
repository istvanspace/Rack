#include <thread>
#include <vector>
#include <atomic>
#include <string>
#include <algorithm>
#include "serial/serial.h"

#include <osdialog.h>

#include <app/Scene.hpp>
#include <app/Browser.hpp>
#include <app/TipWindow.hpp>
#include <app/MenuBar.hpp>
#include <context.hpp>
#include <system.hpp>
#include <network.hpp>
#include <history.hpp>
#include <settings.hpp>
#include <patch.hpp>
#include <asset.hpp>
#include <queue>
#include <mutex>
#include <functional>
#include <future>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <unordered_map>

namespace {
	constexpr uint8_t START_MARKER = 0xFE;
    constexpr uint8_t END_MARKER = 0xFF;
    std::string hexDump(const uint8_t* buffer, size_t size) {
        std::stringstream ss;
        for (size_t i = 0; i < size; ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
        }
        return ss.str();
    }
}

namespace rack {
namespace app {

struct ResizeHandle : widget::OpaqueWidget {
	math::Vec size;

	void draw(const DrawArgs& args) override {
		nvgBeginPath(args.vg);
		nvgMoveTo(args.vg, box.size.x, box.size.y);
		nvgLineTo(args.vg, 0, box.size.y);
		nvgLineTo(args.vg, box.size.x, 0);
		nvgClosePath(args.vg);
		nvgFillColor(args.vg, nvgRGBAf(1, 1, 1, 0.15));
		nvgFill(args.vg);
	}

	void onDragStart(const DragStartEvent& e) override {
		size = APP->window->getSize();
	}

	void onDragMove(const DragMoveEvent& e) override {
		size = size.plus(e.mouseDelta);
		APP->window->setSize(size.round());
	}
};


struct Scene::Internal {
	ResizeHandle* resizeHandle;

	double lastAutosaveTime = 0.0;

	bool heldArrowKeys[4] = {};

	// New members for serial communication
    std::vector<std::thread> serialThreads;
    std::atomic<bool> running{true};
    std::queue<std::function<void()>> taskQueue;
    std::mutex taskMutex;
    std::unordered_map<uint8_t, std::string> processedCommands;
    std::mutex processedCommandsMutex;

	// Flag to track when the window is fully initialized
    std::atomic<bool> windowInitialized{false};
	std::atomic<int> stepCounter{0};

	bool debug = false;  // Debug flag to control logging
	
};


Scene::Scene() {
	internal = new Internal;

	rackScroll = new RackScrollWidget;
	addChild(rackScroll);

	rack = rackScroll->rackWidget;

	menuBar = createMenuBar();
	addChild(menuBar);

	browser = browserCreate();
	browser->hide();
	addChild(browser);

	if (settings::showTipsOnLaunch) {
		addChild(tipWindowCreate());
	}

	internal->resizeHandle = new ResizeHandle;
	internal->resizeHandle->box.size = math::Vec(15, 15);
	internal->resizeHandle->hide();
	addChild(internal->resizeHandle);
}


Scene::~Scene() {
	// Stop all serial threads
    internal->running = false;
    for (auto& thread : internal->serialThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    delete internal;
}


math::Vec Scene::getMousePos() {
	return mousePos;
}


void Scene::step() {
	if (APP->window->isFullScreen()) {
		// Expand RackScrollWidget to cover entire screen if fullscreen
		rackScroll->box.pos.y = 0;
	}
	else {
		// Always show MenuBar if not fullscreen
		menuBar->show();
		rackScroll->box.pos.y = menuBar->box.size.y;
	}

	internal->resizeHandle->box.pos = box.size.minus(internal->resizeHandle->box.size);

	// Resize owned descendants
	menuBar->box.size.x = box.size.x;
	rackScroll->box.size = box.size.minus(rackScroll->box.pos);

	// Autosave periodically
	if (settings::autosaveInterval > 0.0) {
		double time = system::getTime();
		if (time - internal->lastAutosaveTime >= settings::autosaveInterval) {
			internal->lastAutosaveTime = time;
			APP->patch->saveAutosave();
			settings::save();
		}
	}

	// Scroll RackScrollWidget with arrow keys
	math::Vec arrowDelta;
	if (internal->heldArrowKeys[0]) {
		arrowDelta.x -= 1;
	}
	if (internal->heldArrowKeys[1]) {
		arrowDelta.x += 1;
	}
	if (internal->heldArrowKeys[2]) {
		arrowDelta.y -= 1;
	}
	if (internal->heldArrowKeys[3]) {
		arrowDelta.y += 1;
	}

	if (!arrowDelta.isZero()) {
		int mods = APP->window->getMods();
		float arrowSpeed = 32.f;
		if ((mods & RACK_MOD_MASK) == RACK_MOD_CTRL)
			arrowSpeed /= 4.f;
		if ((mods & RACK_MOD_MASK) == GLFW_MOD_SHIFT)
			arrowSpeed *= 4.f;
		if ((mods & RACK_MOD_MASK) == (RACK_MOD_CTRL | GLFW_MOD_SHIFT))
			arrowSpeed /= 16.f;

		rackScroll->offset += arrowDelta * arrowSpeed;
	}

	// Process tasks from background threads
    std::function<void()> task;
    {
        std::lock_guard<std::mutex> lock(internal->taskMutex);
        if (!internal->taskQueue.empty()) {
            task = internal->taskQueue.front();
            internal->taskQueue.pop();
        }
    }
    if (task) {
        task();
    }

	Widget::step();

	// Set windowInitialized to true after the first step
    if (!internal->windowInitialized.load()) {
        int currentStep = internal->stepCounter.fetch_add(1) + 1;
        if (currentStep >= 30) {
            internal->windowInitialized.store(true);
            INFO("Window initialized after %d steps", currentStep);
			// Start serial communication threads
			startSerialThreads();
        }
    }
}


void Scene::draw(const DrawArgs& args) {
	Widget::draw(args);
}


void Scene::onHover(const HoverEvent& e) {
	mousePos = e.pos;
	if (mousePos.y < menuBar->box.size.y) {
		menuBar->show();
	}
	OpaqueWidget::onHover(e);
}


void Scene::onDragHover(const DragHoverEvent& e) {
	mousePos = e.pos;
	OpaqueWidget::onDragHover(e);
}


void Scene::onHoverKey(const HoverKeyEvent& e) {
	// Key commands that override children
	if (e.action == GLFW_PRESS || e.action == GLFW_REPEAT) {
		// DEBUG("key '%d '%c' scancode %d '%c' keyName '%s'", e.key, e.key, e.scancode, e.scancode, e.keyName.c_str());
		if (e.keyName == "n" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			APP->patch->loadTemplateDialog();
			e.consume(this);
		}
		if (e.keyName == "q" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			APP->window->close();
			e.consume(this);
		}
		if (e.keyName == "o" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			APP->patch->loadDialog();
			e.consume(this);
		}
		if (e.keyName == "o" && (e.mods & RACK_MOD_MASK) == (RACK_MOD_CTRL | GLFW_MOD_SHIFT)) {
			APP->patch->revertDialog();
			e.consume(this);
		}
		if (e.keyName == "s" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			APP->patch->saveDialog();
			e.consume(this);
		}
		if (e.keyName == "s" && (e.mods & RACK_MOD_MASK) == (RACK_MOD_CTRL | GLFW_MOD_SHIFT)) {
			APP->patch->saveAsDialog();
			e.consume(this);
		}
		if (e.keyName == "z" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			APP->history->undo();
			e.consume(this);
		}
		if (e.keyName == "z" && (e.mods & RACK_MOD_MASK) == (RACK_MOD_CTRL | GLFW_MOD_SHIFT)) {
			APP->history->redo();
			e.consume(this);
		}
		if (e.keyName == "-" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			float zoom = std::log2(APP->scene->rackScroll->getZoom());
			zoom *= 2;
			zoom = std::ceil(zoom - 0.01f) - 1;
			zoom /= 2;
			APP->scene->rackScroll->setZoom(std::pow(2.f, zoom));
			e.consume(this);
		}
		// Numpad has a "+" key, but the main keyboard section hides it under "="
		if ((e.keyName == "=" || e.keyName == "+") && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			float zoom = std::log2(APP->scene->rackScroll->getZoom());
			zoom *= 2;
			zoom = std::floor(zoom + 0.01f) + 1;
			zoom /= 2;
			APP->scene->rackScroll->setZoom(std::pow(2.f, zoom));
			e.consume(this);
		}
		if ((e.keyName == "0") && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			APP->scene->rackScroll->setZoom(1.f);
			e.consume(this);
		}
		if (e.key == GLFW_KEY_F1 && (e.mods & RACK_MOD_MASK) == 0) {
			system::openBrowser("https://vcvrack.com/manual/");
			e.consume(this);
		}
		if (e.key == GLFW_KEY_F3 && (e.mods & RACK_MOD_MASK) == 0) {
			settings::cpuMeter ^= true;
			e.consume(this);
		}
		if (e.key == GLFW_KEY_F11 && (e.mods & RACK_MOD_MASK) == 0) {
			APP->window->setFullScreen(!APP->window->isFullScreen());
			// The MenuBar will be hidden when the mouse moves over the RackScrollWidget.
			// menuBar->hide();
			e.consume(this);
		}

		// Module selections
		if (e.keyName == "a" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			rack->selectAll();
			e.consume(this);
		}
		if (e.keyName == "a" && (e.mods & RACK_MOD_MASK) == (RACK_MOD_CTRL | GLFW_MOD_SHIFT)) {
			rack->deselectAll();
			e.consume(this);
		}
		if (e.keyName == "c" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			if (rack->hasSelection()) {
				rack->copyClipboardSelection();
				e.consume(this);
			}
		}
		if (e.keyName == "i" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			if (rack->hasSelection()) {
				rack->resetSelectionAction();
				e.consume(this);
			}
		}
		if (e.keyName == "r" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			if (rack->hasSelection()) {
				rack->randomizeSelectionAction();
				e.consume(this);
			}
		}
		if (e.keyName == "u" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			if (rack->hasSelection()) {
				rack->disconnectSelectionAction();
				e.consume(this);
			}
		}
		if (e.keyName == "e" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			if (rack->hasSelection()) {
				rack->bypassSelectionAction(!rack->isSelectionBypassed());
				e.consume(this);
			}
		}
		if (e.keyName == "d" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			if (rack->hasSelection()) {
				rack->cloneSelectionAction(false);
				e.consume(this);
			}
		}
		if (e.keyName == "d" && (e.mods & RACK_MOD_MASK) == (RACK_MOD_CTRL | GLFW_MOD_SHIFT)) {
			if (rack->hasSelection()) {
				rack->cloneSelectionAction(true);
				e.consume(this);
			}
		}
		if ((e.key == GLFW_KEY_DELETE || e.key == GLFW_KEY_BACKSPACE) && (e.mods & RACK_MOD_MASK) == 0) {
			if (rack->hasSelection()) {
				rack->deleteSelectionAction();
				e.consume(this);
			}
		}
	}

	// Scroll RackScrollWidget with arrow keys
	if (e.action == GLFW_PRESS || e.action == GLFW_RELEASE) {
		if (e.key == GLFW_KEY_LEFT) {
			internal->heldArrowKeys[0] = (e.action == GLFW_PRESS);
			e.consume(this);
		}
		if (e.key == GLFW_KEY_RIGHT) {
			internal->heldArrowKeys[1] = (e.action == GLFW_PRESS);
			e.consume(this);
		}
		if (e.key == GLFW_KEY_UP) {
			internal->heldArrowKeys[2] = (e.action == GLFW_PRESS);
			e.consume(this);
		}
		if (e.key == GLFW_KEY_DOWN) {
			internal->heldArrowKeys[3] = (e.action == GLFW_PRESS);
			e.consume(this);
		}
	}

	if (e.isConsumed())
		return;
	OpaqueWidget::onHoverKey(e);
	if (e.isConsumed())
		return;

	// Key commands that can be overridden by children
	if (e.action == GLFW_PRESS || e.action == GLFW_REPEAT) {
		// Alternative key command for exiting fullscreen, since F11 doesn't work reliably on Mac due to "Show desktop" OS binding.
		if (e.key == GLFW_KEY_ESCAPE && (e.mods & RACK_MOD_MASK) == 0) {
			if (APP->window->isFullScreen()) {
				APP->window->setFullScreen(false);
				e.consume(this);
			}
		}
		if (e.keyName == "v" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			rack->pasteClipboardAction();
			e.consume(this);
		}
		if ((e.key == GLFW_KEY_ENTER || e.key == GLFW_KEY_KP_ENTER) && (e.mods & RACK_MOD_MASK) == 0) {
			browser->show();
			e.consume(this);
		}
	}
}


void Scene::onPathDrop(const PathDropEvent& e) {
	if (e.paths.size() >= 1) {
		const std::string& path = e.paths[0];
		std::string extension = system::getExtension(path);

		if (extension == ".vcv") {
			APP->patch->loadPathDialog(path);
			e.consume(this);
			return;
		}
		if (extension == ".vcvs") {
			APP->scene->rack->loadSelection(path);
			e.consume(this);
			return;
		}
	}

	OpaqueWidget::onPathDrop(e);
}

void Scene::startSerialThreads() {
    std::vector<serial::PortInfo> devices = serial::list_ports();
    for (const auto& device : devices) {
        if (device.hardware_id.find("Arduino") != std::string::npos || 
            device.description.find("Arduino") != std::string::npos) {
            INFO("Found Arduino on port: %s", device.port.c_str());
            internal->serialThreads.emplace_back(&Scene::serialThread, this, device.port);
            break;  // Assuming we only want to use the first Arduino found
        }
    }
    if (internal->serialThreads.empty()) {
        WARN("No Arduino found. Serial communication disabled.");
    }
}

void Scene::serialAcknowledgment(serial::Serial& serial_port, const uint8_t* buffer, size_t bufferSize, bool success) {
    const size_t MAX_BUFFER_SIZE = 256;
    uint8_t responseBuffer[MAX_BUFFER_SIZE];
    uint8_t* ptr = responseBuffer;

    *ptr++ = START_MARKER;
    
    uint16_t messageLength = 4 + buffer[2] + buffer[3];  // 4 for commandId, commandType, and payload lengths
    *ptr++ = (messageLength >> 8) & 0xFF;  // High byte of length
    *ptr++ = messageLength & 0xFF;  // Low byte of length

    *ptr++ = buffer[0]; // Echo back the command ID
    *ptr++ = success ? 0x01 : 0x03; // 0x01 = Acknowledgment, 0x03 = Error
    *ptr++ = buffer[2]; // Payload 1 Length
    *ptr++ = buffer[3]; // Payload 2 Length
    memcpy(ptr, buffer + 4, buffer[2]); // Copy Payload 1
    ptr += buffer[2];
    memcpy(ptr, buffer + 4 + buffer[2], buffer[3]); // Copy Payload 2
    ptr += buffer[3];
    *ptr++ = END_MARKER;

    size_t responseSize = ptr - responseBuffer;

    WARN("Sending response: %s", hexDump(responseBuffer, responseSize).c_str());

    serial_port.write(responseBuffer, responseSize);
}

void Scene::serialThread(std::string port) {
    try {
        serial::Serial serial_port(port, 230400, serial::Timeout::simpleTimeout(500));
		// Clear the serial buffer
		if (serial_port.isOpen()) {
			serial_port.flush(); // Clear any data in the output buffer (just in case)

			// Read and discard all available data from the input buffer
			while (serial_port.available()) {
				serial_port.read();
			}
		}
        const size_t MAX_BUFFER_SIZE = 256;
        uint8_t buffer[MAX_BUFFER_SIZE];
        size_t bufferIndex = 0;
        bool messageStarted = false;
        uint16_t expectedLength = 0;

        while (internal->running) {
            while (serial_port.available()) {
                uint8_t byte;
                serial_port.read(&byte, 1);

                if (!messageStarted && byte == START_MARKER) {
                    messageStarted = true;
                    bufferIndex = 0;
                    continue;
                }

                if (messageStarted) {
                    buffer[bufferIndex++] = byte;

                    if (bufferIndex == 2) {
                        // First two bytes after START_MARKER represent message length
                        expectedLength = (buffer[0] << 8) | buffer[1];
                        continue;
                    }

                    if (bufferIndex == expectedLength + 3) {  // +3 for length bytes and END_MARKER
                        if (byte == END_MARKER) {
                            if (internal->debug) {
								WARN("Buffer contents: %s", hexDump(buffer, bufferIndex - 1).c_str());
							}

                            {
                                // Capture serial_port by reference to use it in the lambda
                                std::lock_guard<std::mutex> lock(internal->taskMutex);
								std::vector<uint8_t> bufferCopy(buffer, buffer + bufferIndex);
								std::string portCopy = port;
                                internal->taskQueue.push([this, portCopy, bufferCopy, bufferIndex]() mutable {
                                    processMessage(bufferCopy.data() + 2, bufferIndex - 3, portCopy);
                                });
                            }
                        } else {
                            WARN("Invalid END_MARKER received");
                        }

                        messageStarted = false;
                        bufferIndex = 0;
                    }
                }

                // Prevent buffer overflow
                if (bufferIndex >= MAX_BUFFER_SIZE) {
                    WARN("Buffer overflow detected. Resetting buffer.");
                    messageStarted = false;
                    bufferIndex = 0;
                }
            }// Yield to allow other threads to run
        }
    }
    catch (serial::IOException& e) {
        WARN("Failed to open serial port: %s", e.what());
    }
}

bool Scene::processMessage(const uint8_t* buffer, size_t size, std::string port) {
	serial::Serial serial_port(port, 230400, serial::Timeout::simpleTimeout(500));
	if (!internal->windowInitialized) {
        WARN("Window not initialized. Ignoring message.");
        return false;
    }

    if (size < 4) {  // At least 4 bytes are needed for commandId, commandType, and payload lengths
        WARN("Received message is too short");
        return false;
    }

    uint8_t commandId = buffer[0];
    uint8_t commandType = buffer[1];
    uint8_t payload1Length = buffer[2];
    uint8_t payload2Length = buffer[3];

    // Calculate the expected total size of the message
    size_t expectedSize = 4 + payload1Length + payload2Length;

    // Ensure the buffer size matches the expected size based on the payload lengths
    if (size != expectedSize) {
        WARN("Received message is shorter than expected based on payload lengths");
        return false;
    }

    const char* payload1 = reinterpret_cast<const char*>(buffer + 4);
    const char* payload2 = reinterpret_cast<const char*>(buffer + 4 + payload1Length);

    // Null-terminate the payload strings (create copies to safely null-terminate)
    std::string payload1Str(payload1, payload1Length);
    std::string payload2Str(payload2, payload2Length);

    if (internal->debug) {
		INFO("Received message: commandId=%u, commandType=%u, payload1=%s, payload2=%s", 
			 commandId, commandType, payload1Str.c_str(), payload2Str.c_str());
	}

    // Handle the received command
    bool success = false;
    switch (commandType) {
        case 0x01:  // 'init' command
        {
			// Check if this command was already processed
			std::string commandSignature = std::to_string(commandId) + std::to_string(commandType) + payload1Str + payload2Str;
			
			{
				std::lock_guard<std::mutex> lock(internal->processedCommandsMutex);
				auto it = internal->processedCommands.find(commandId);
				if (it != internal->processedCommands.end() && it->second == commandSignature) {
					if (internal->debug) {
						INFO("Command with ID %u was already processed. Skipping re-execution.", commandId);
					}
					serialAcknowledgment(serial_port, buffer, size, success);
					return true;  // Command already processed, no need to re-execute
				}
			}

            std::lock_guard<std::mutex> lock(internal->taskMutex);
			// success = true;  // Assume success, replace with actual function call if needed
            success = addVcoModule(payload1Str.c_str(), payload2Str.c_str());
            INFO("VCO module added via Arduino command: payload1=%s, payload2=%s", 
                 payload1Str.c_str(), payload2Str.c_str());

            // Acknowledge only for 'init' command
            serialAcknowledgment(serial_port, buffer, size, success);
			internal->processedCommands[commandId] = commandSignature;
            break;
        }
        case 0x02:  // 'parameter update' command
        {
            try {
				int paramId = std::stoi(payload1Str);
				int rawValue = std::stoi(payload2Str);
				int64_t moduleId = 4; // Hardcoded module ID
				float normalizedValue = rawValue / 1023.0f;

				// Apply the update immediately
				updateModuleParameter(moduleId, paramId, normalizedValue);
				success = true;
			} catch (const std::invalid_argument& e) {
				WARN("Failed to convert payload to integer: %s", e.what());
				return false;
			} catch (const std::out_of_range& e) {
				WARN("Payload out of range for integer conversion: %s", e.what());
				return false;
			}
            break;
        }
        default:
            WARN("Unknown command type: %u", commandType);
            return false;
    }

    return success;
}


bool Scene::addVcoModule(const char* payload1, const char* payload2) {
    // Create the rootJ object
    json_t *rootJ = json_object();

    // Create the modules array
    json_t *modules = json_array();

    // Create the first module object
    json_t *module = json_object();

    // Set the module properties
    json_object_set_new(module, "plugin", json_string(payload1));
    json_object_set_new(module, "model", json_string(payload2));
    json_object_set_new(module, "version", json_string("2.6.0"));
    json_object_set_new(module, "id", json_integer(-1));

    // Create the params array
    json_t *params = json_array();

    // Add params to the array
    json_t *param0 = json_object();
    json_object_set_new(param0, "value", json_real(0.0));
    json_object_set_new(param0, "id", json_integer(0));
    json_array_append_new(params, param0);

    json_t *param1 = json_object();
    json_object_set_new(param1, "value", json_real(1.0));
    json_object_set_new(param1, "id", json_integer(1));
    json_array_append_new(params, param1);

    json_t *param2 = json_object();
    json_object_set_new(param2, "value", json_real(1.0));
    json_object_set_new(param2, "id", json_integer(2));
    json_array_append_new(params, param2);

    json_t *param3 = json_object();
    json_object_set_new(param3, "value", json_real(0.0));
    json_object_set_new(param3, "id", json_integer(3));
    json_array_append_new(params, param3);

    json_t *param4 = json_object();
    json_object_set_new(param4, "value", json_real(0.0));
    json_object_set_new(param4, "id", json_integer(4));
    json_array_append_new(params, param4);

    json_t *param5 = json_object();
    json_object_set_new(param5, "value", json_real(0.5));
    json_object_set_new(param5, "id", json_integer(5));
    json_array_append_new(params, param5);

    json_t *param6 = json_object();
    json_object_set_new(param6, "value", json_real(0.0));
    json_object_set_new(param6, "id", json_integer(6));
    json_array_append_new(params, param6);

    json_t *param7 = json_object();
    json_object_set_new(param7, "value", json_real(0.0));
    json_object_set_new(param7, "id", json_integer(7));
    json_array_append_new(params, param7);

    // Add params array to module object
    json_object_set_new(module, "params", params);

    // Create and add pos array to module object
    json_t *pos = json_array();
    json_array_append_new(pos, json_integer(39));
    json_array_append_new(pos, json_integer(0));
    json_object_set_new(module, "pos", pos);

    // Add module object to modules array
    json_array_append_new(modules, module);

    // Add modules array to rootJ object
    json_object_set_new(rootJ, "modules", modules);

    // Create an empty cables array and add it to rootJ object
    json_t *cables = json_array();

    // Create a new cable object
    json_t *cable = json_object();
    json_object_set_new(cable, "id", json_integer(-1));
    json_object_set_new(cable, "outputModuleId", json_integer(2));
    json_object_set_new(cable, "outputId", json_integer(0));
    json_object_set_new(cable, "inputModuleId", json_integer(7));
    json_object_set_new(cable, "inputId", json_integer(1));
    json_object_set_new(cable, "color", json_string("#f3374b"));
    json_array_append_new(cables, cable);

    // Add cables array to rootJ object
    json_object_set_new(rootJ, "cables", cables);

    bool success = false;

    // Add the module to the scene
    if (APP && APP->scene && APP->scene->rack) {
		APP->scene->rack->addModuleFromJson(module);
        APP->scene->rack->addCableFromJson(cable);
		INFO("VCO module added to rack");
        success = true;
    } else {
        WARN("Scene or Rack is not initialized");
    }

    // Cleanup the JSON object
    json_decref(rootJ);

    return success;  // Return success or failure
}

bool Scene::updateModuleParameter(int64_t moduleId, int paramId, float normalizedValue) {
    if (!APP || !APP->scene || !APP->scene->rack) {
        WARN("Scene or Rack is not initialized - cannot update module parameter");
        return false;
    }
    
    rack::app::ModuleWidget* mw = APP->scene->rack->getModule(moduleId);
    if (!mw || !mw->getModule()) {
        WARN("Module with ID %ld not found", moduleId);
        return false;
    }

    engine::Module* module = mw->getModule();
    if (paramId < 0 || paramId >= (int)module->params.size()) {
        return false;
    }

    engine::ParamQuantity* pq = module->paramQuantities[paramId];
    if (!pq) {
        return false;
    }

    float oldValue = pq->getValue();
    float newValue = pq->getMinValue() + (pq->getMaxValue() - pq->getMinValue()) * normalizedValue;
    
    // Apply all changes, no matter how small
    pq->setValue(newValue);

    // Create a history action for every change
    history::ParamChange* h = new history::ParamChange;
    h->name = "set parameter";
    h->moduleId = moduleId;
    h->paramId = paramId;
    h->oldValue = oldValue;
    h->newValue = newValue;
    APP->history->push(h);

    return true;
}



} // namespace app
} // namespace rack
