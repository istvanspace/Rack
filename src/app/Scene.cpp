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

	// Start serial communication threads
    startSerialThreads();
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
		// Handle DigiMod-specific key press: Ctrl+H
		if (e.keyName == "h" && (e.mods & RACK_MOD_MASK) == RACK_MOD_CTRL) {
			addVcoModule();
			WARN("addVcoModule()");
			e.consume(this);
		}
		if (e.action == GLFW_PRESS && e.key == GLFW_KEY_F2) {
			std::lock_guard<std::mutex> lock(internal->taskMutex);
			internal->taskQueue.push([this]() { 
				addVcoModule(); 
				INFO("VCO module added via F2 key press");
			});
			e.consume(this);
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

void Scene::serialThread(std::string port) {
    try {
        serial::Serial serial_port(port, 9600, serial::Timeout::simpleTimeout(1000));
        
        while (internal->running) {
            if (serial_port.available()) {
                std::string msg = serial_port.readline();
                std::transform(msg.begin(), msg.end(), msg.begin(), ::tolower);
                if (msg.find("execute") != std::string::npos) {
                    std::lock_guard<std::mutex> lock(internal->taskMutex);
                    internal->taskQueue.push([this]() { 
                        addVcoModule(); 
                        INFO("VCO module added via Arduino button press");
                    });
                }
            }
        }
    }
    catch (serial::IOException& e) {
        WARN("Failed to open serial port: %s", e.what());
    }
}

void Scene::addVcoModule() {
    // Create the rootJ object
    json_t *rootJ = json_object();

    // Create the modules array
    json_t *modules = json_array();

    // Create the first module object
    json_t *module = json_object();

    // Set the module properties
    json_object_set_new(module, "plugin", json_string("Fundamental"));
    json_object_set_new(module, "model", json_string("VCO"));
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

    // Add the module to the scene
    if (APP && APP->scene && APP->scene->rack) {
		APP->scene->rack->addModuleFromJson(module);
        APP->scene->rack->addCableFromJson(cable);
		INFO("VCO module added to rack");
    } else {
        WARN("Scene or Rack is not initialized");
    }

    // Cleanup the JSON object
    json_decref(rootJ);
}


} // namespace app
} // namespace rack
