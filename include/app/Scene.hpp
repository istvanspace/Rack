#pragma once
#include <app/common.hpp>
#include <widget/OpaqueWidget.hpp>
#include <app/RackScrollWidget.hpp>
#include <app/RackWidget.hpp>
#include <string>


namespace rack {
namespace app {


struct Scene : widget::OpaqueWidget {
	struct Internal;
	Internal* internal;


	// Convenience variables for accessing important widgets
	RackScrollWidget* rackScroll;
	RackWidget* rack;
	widget::Widget* menuBar;
	widget::Widget* browser;

	/** The last mouse position in the Scene.
	DEPRECATED. Use getMousePos() instead.
	*/
	math::Vec mousePos;

	PRIVATE Scene();
	PRIVATE ~Scene();
	math::Vec getMousePos();
	void step() override;
	void draw(const DrawArgs& args) override;
	void onHover(const HoverEvent& e) override;
	void onDragHover(const DragHoverEvent& e) override;
	void onHoverKey(const HoverKeyEvent& e) override;
	void onPathDrop(const PathDropEvent& e) override;

	// DigiMod methods

	void addVcoModule();

	// New methods for serial communication
    void startSerialThreads();
    void serialThread(std::string port);
};


} // namespace app
} // namespace rack
