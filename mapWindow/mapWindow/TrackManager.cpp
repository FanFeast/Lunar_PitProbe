#include <iostream>
#include <fstream>
#include <sstream>
#include "DrawingUtilNG.h"
#include "GraphicFont.h"
#include "StringPlus.h"
#include "ysglfontdata.h"
#include "yssimplesound.h"
#include "TrackManager.h"

using namespace std;

TrackManager::TrackManager(int width, int height)
{
	// for testing, just load a track (comment out when done)
	//ifstream inFile("loop2.track");
	//theTrack.readFile(inFile);
	//inFile.close();

	winWidth = width;
	winHeight = height;

	FsOpenWindow(16, 16, winWidth, winHeight, 1, "Track Manager (NG 2022)");

	// set color and width
	glColor3b(255, 0, 0);  // red
	//glLineWidth(3);

		// load sounds and start player
	//if (YSOK != insideSound.LoadWav("cash_register.wav"))
	//	cout << "   ERROR: Unable to load cash_register.wav " << endl;
	//if (YSOK != outsideSound.LoadWav("buzzer.wav"))
	//	cout << "   ERROR: Unable to load cash_register.wav " << endl;
	//theSoundPlayer.Start();

	ButtonCollection* myButtons = new ButtonCollection; // put this AFTER FsOpenWindow()
	GraphicFont* buttonFont = new ComicSansFont;
	buttonFont->setColorRGB(0, 0, 0); // black

	addButtonsVert(buttonFont, winWidth - 108, 100);

}

Point2D TrackManager::getScreenCoords(Point2D worldCoords)
{
	float screenX = worldCoords.x * scale + panX;
	float screenY = worldCoords.y * -scale + panY;
	return { screenX, screenY };
}

Point2D TrackManager::getWorldCoords(Point2D screenCoords)
{
	float worldX = (screenCoords.x - panX) / scale;
	float worldY = (screenCoords.y - panY) / -scale;
	return { worldX, worldY };
}

void TrackManager::manageMouse()
{
	// handle mouse input (OpenGL is still in screen coords)
	stringstream coordStream;     // for displaying coordinates on screen
	FsPollDevice();
	mouseEvent = FsGetMouseEvent(leftButton, middleButton,
		rightButton, screenX, screenY);

	Point2D worldPnt = getWorldCoords({ screenX * 1.f, screenY * 1.f });

	if (leftButton) { // write coords on screen if left button is held down
		coordStream.str("");  // reset stream
		coordStream.precision(4);
		coordStream << worldPnt.x << ", " << worldPnt.y
			<< " (" << screenX << ", " << screenY << ")";
		glColor3ub(60, 230, 60);
		glRasterPos2i(screenX, screenY - 3);  // set position 3 pix above
		YsGlDrawFontBitmap7x10(coordStream.str().c_str());
	}

	if (inEditMode) {
		float hoverDistance = 3.f / scale; // 3 pixels

	}

	// capture location of first button press (needed for panning and zooming)
	if (mouseEvent == FSMOUSEEVENT_LBUTTONDOWN || mouseEvent == FSMOUSEEVENT_MBUTTONDOWN) {
		prevScreenX = screenX; prevScreenY = screenY;
	}

	// disallow panning and zooming with mouse when a vertex is moving
	if (!vertexIsMoving) {

		// pan in x and y axes when Ctrl key is held down and left button is down
		// note: I added middle button (wheel) drag for panning
		if (middleButton || (FsGetKeyState(FSKEY_CTRL) && leftButton)) {
			// no need for scale since the screen-to-model ratio is not applicable
			panX += (screenX - prevScreenX);
			panY += (screenY - prevScreenY);
			prevScreenX = screenX; prevScreenY = screenY; // reset previous values to continue move
		}

		// zoom in and out when Shft key is held down and left button is down
		// note: I added wheel rolling for zoomimg, which accidentally is also
		//       triggered by touchpad pinch and two finger scroll
		else if (key == FSKEY_WHEELUP || key == FSKEY_WHEELDOWN
			|| (FsGetKeyState(FSKEY_SHIFT) && leftButton)) {
			double oldScale = scale;
			if (key == FSKEY_WHEELUP)
				scale *= 1.03; // less jumpy than zooming with +/- keys
			else if (key == FSKEY_WHEELDOWN)
				scale /= 1.02;
			else if (screenY < prevScreenY)
				scale *= max(1.02, (prevScreenY - screenY) * 0.1);
			else if (screenY > prevScreenY)
				scale /= max(1.02, (prevScreenY - screenY) * -0.1);

			// adjust panX and panY so point under mouse does not move
			// i.e., we can zoom in/out on a specific point
			// a bit complicated since you have to convert old origin to screen coords
			// then adjust pan, then convert to model coords. 
			// what you see below is the simplified equation after all substitutions
			// rounding reduces "shifting"
			if (key == FSKEY_WHEELUP || key == FSKEY_WHEELDOWN) {
				panX = (int)round((screenX * (oldScale - scale)
					+ panX * scale) / oldScale);
				panY = (int)round((screenY * (oldScale - scale)
					+ panY * scale) / oldScale);
			}
			prevScreenX = screenX; prevScreenY = screenY; // reset previous values to continue move
		}
	}
}

//void TrackManager::addButtonsVert(GraphicFont* aFont, int xLoc, int wid)
//{
//	int hei = 30;
//	int spacing = 10;
//
//	int currY = 30;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_L, "Load", aFont,
//		"Load a track from a file");
//	
//	currY += hei + spacing;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_S, "Save", aFont,
//		"Save track to a file");
//
//	currY += hei + spacing;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_H, "Hue", aFont,
//		"Cycle through several colors of track");
//
//	//currY += hei + spacing;
//	//theButtons.add(xLoc, currY, wid, hei, FSKEY_Q, "Sound", aFont,
//	//	"Toggle sound feedback on/off");
//
//	currY += hei + spacing;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_Z, "View All", aFont,
//		"Adjust view to fit the whole track");
//
//	currY += hei + spacing * 3;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_E, "Edit", aFont,
//		"Toggle Edit Mode");
//
//	currY += hei + spacing * 3;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_B, "Add Box", aFont,
//		"Add a box to the track");
//
//	currY += hei + spacing;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_R, "Remove", aFont,
//		"Remove box from the track");
//
//	currY += hei + spacing;
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_I, "Initial", aFont,
//		"Initialize model (put all boxes at initial positions)");
//
//	currY += hei + spacing * 3;
//	GraphicFont* boldFont = new JokermanFont;
//	boldFont->setColorRGB(1., 0., 0.);
//	theButtons.add(xLoc, currY, wid, hei, FSKEY_SPACE, "GO/STOP", boldFont,
//		"Start/Stop the simulation");
//
//	// to disable a button (will gray out and won't return its value)
//	//theButtons.disableButton(FSKEY_SPACE);
//
//}





void TrackManager::sendUserToConsole() {
	glColor3f(0, 0, 0);
	glRasterPos2d(100, 200);
	YsGlDrawFontBitmap20x28("Input required on console . . .");
	FsSwapBuffers();
}



bool TrackManager::manage()
{
	ofstream outFile;
	int buttonKey;

	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	// reset all transformations
	//glLoadIdentity();

	FsPollDevice();
	key = FsInkey();

	// handle mouse input (remember that OpenGL is still in screen coords)
	manageMouse();

	// check if a button was clicked
	if (key == FSKEY_NULL && mouseEvent == FSMOUSEEVENT_LBUTTONDOWN) {
		buttonKey = theButtons.checkClick(screenX, screenY);

		if (buttonKey != FSKEY_NULL)
			key = buttonKey;  // pretend the user pressed a key 
	}

	switch (key) {
	case FSKEY_RIGHT: panX += 5;
		break;
	case FSKEY_LEFT: panX -= 5;
		break;
	case FSKEY_UP: panY -= 5;
		break;
	case FSKEY_DOWN: panY += 5;
		break;
	case FSKEY_PLUS: scale *= 1.05;
		break;
	case FSKEY_MINUS: scale /= 1.05;
		break;
	/*case FSKEY_H:
		theTrack.changeColor(10.f);
		break;*/

	}


	// set up axes to "math normal" (origin at lower left, y-axis going up)
	// and pan and scale
	glTranslatef(panX, panY, 0);
	glScalef(scale, -scale, 1);


	int i = 2;
	double red, green, blue;

	// reset all transformations and paint the buttons on top of everything
	glLoadIdentity();
	theButtons.paint();
	theButtons.checkHover(screenX, screenY); // remove hover feedback for better performance ?


	return key != FSKEY_ESC;
}

void TrackManager::addButtonsVert(GraphicFont* aFont, int xLoc, int wid)
{
	int hei = 30;
	int spacing = 10;

	int currY = 30;
	theButtons.add(xLoc, currY, wid, hei, FSKEY_V, "Vantage", aFont,
		"Load the vantage points");

	currY += hei + spacing;
	theButtons.add(xLoc, currY, wid, hei, FSKEY_S, "Slope", aFont,
		"Show the slope");

	currY += hei + spacing;
	theButtons.add(xLoc, currY, wid, hei, FSKEY_L, "LOS%", aFont,
		"shows the LOS percent");

	//currY += hei + spacing;
	//theButtons.add(xLoc, currY, wid, hei, FSKEY_Q, "Sound", aFont,
	//	"Toggle sound feedback on/off");

	currY += hei + spacing;
	theButtons.add(xLoc, currY, wid, hei, FSKEY_P, "Paths", aFont,
		"Shows the possible paths");

	currY += hei + spacing * 3;
	theButtons.add(xLoc, currY, wid, hei, FSKEY_X, "Landing", aFont,
		"Shows the landing site");

	// to disable a button (will gray out and won't return its value)
	//theButtons.disableButton(FSKEY_SPACE);
}
