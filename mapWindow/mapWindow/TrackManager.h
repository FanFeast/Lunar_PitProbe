#pragma once
/*
Nestor Gomez
Carnegie Mellon University
Eng. Computation, 24-780-B
Prob Set 9
Due Tues. Nov. 22, 2022
*/

#include <vector>
#include <fstream>

#include "fssimplewindow.h"
#include "yssimplesound.h"

#include "ButtonCollection.h"

struct Point2D {
	float x;
	float y;
};

class TrackManager {
private:
	
	bool simulationIsRunning = false;

	int winWidth = 1000, winHeight = 600;
	int panX = 0, panY = winHeight;
	float scale = 6.f;

	std::string fileName;
	std::ifstream inFile;
	char userChoice = 'N';
	int key = FSKEY_NULL; // just to start it up

	bool inEditMode = false;
	bool showThePoints = false;
	bool useFillMode = false;

	// for moving a vertex with the mouse
	int currVertex = -1;
	bool vertexIsMoving = false;
	int mouseEvent = 0, leftButton = 0, middleButton = 0, rightButton = 0,
		screenX = 0, screenY = 0;

	// for panning and zooming
	int prevScreenX = -1000, prevScreenY = -1000;

	// buttons
	ButtonCollection theButtons;

	// audio feedback on inside/outside check (left-over from PS06 :-)
	//YsSoundPlayer theSoundPlayer;
	//YsSoundPlayer::SoundData insideSound;
	//YsSoundPlayer::SoundData outsideSound;

public:
	TrackManager() {};

	TrackManager(int width, int height);

	bool manage();


protected:
	// resets pan and zoom so that all shapes fit on the window

	// do the calculations to set values of panx, pany, and zoom
	// such that given bounding box fills the screen
	
	// if manager is in edit mode, this paints some indication on the graphics
	// screen that this is the case. Otherwise does nothing
	

	Point2D getScreenCoords(Point2D worldCoords);

	// uses current pan and zoom to convert coordinates from screen to world
	Point2D getWorldCoords(Point2D screenCoords);

	// handle mouse input (OpenGL is still in screen coords)
	void manageMouse();

	// creates the buttons to be shown on interface in a vertical column
	// called by constructor
	// 
	void addButtonsVert(GraphicFont* aFont, int xLoc, int buttonWidth);

	// manage slide boxes
	void sendUserToConsole();
	

};