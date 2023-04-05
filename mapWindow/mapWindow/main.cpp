#include <iostream>

#include "fssimplewindow.h"
#include "ButtonCollection.h"
#include "GraphicFont.h"
#include "TrackManager.h"


using namespace std;

int main(void) {

	int hei = 30;
	int spacing = 10;
	int currY = 30;
	int key = FSKEY_NULL;
	int buttonKey;
	int mouseEvent = 0, leftButton = 0, middleButton = 0, rightButton = 0,
		screenX = 0, screenY = 0;

	//FsOpenWindow(16, 16, 800, 600, 1);
	TrackManager theManager(900, 600);
	/*ButtonCollection theButtons;
	GraphicFont* aFont = new TimesNewRomanFont;
	aFont->setColorRGB(0, 0, 0);
	

	theButtons.add(700, 20, 80, 30, FSKEY_V, "Vantage", aFont, "this displays all vantage points");
	theButtons.add(700, 80, 80, 30, FSKEY_S, "Slope", aFont, "show the slope");
	theButtons.add(700, 140, 80, 30, FSKEY_L, "LOS%", aFont, "show the LOS percent");
	theButtons.add(700, 200, 80, 30, FSKEY_P, "Paths", aFont, "shows the possible paths");
	theButtons.add(700, 260, 80, 30, FSKEY_X, "Landing", aFont, "show the landing site");*/

	//while (FSKEY_ESC != key) {
	//	
	//	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	//	theButtons.paint();
	//	theButtons.checkHover(screenX, screenY);
	//	glLoadIdentity();
	//	FsSwapBuffers();
	//	FsPollDevice();
	//	mouseEvent = FsGetMouseEvent(leftButton, middleButton,
	//		rightButton, screenX, screenY);

	//	key = FsInkey();
	//	if (key == FSKEY_NULL && mouseEvent == FSMOUSEEVENT_LBUTTONDOWN) {
	//		buttonKey = theButtons.checkClick(screenX, screenY);

	//		if (buttonKey != FSKEY_NULL)
	//		{
	//			key = buttonKey;  // pretend the user pressed a key 
	//			cout << screenX << screenY << endl;
	//		}
	//	}
	//	FsSleep(25);
	//}
	while (theManager.manage()) {
		// actually display graphics
		FsSwapBuffers();

		// prepare for next loop
		FsSleep(25);
	}
}