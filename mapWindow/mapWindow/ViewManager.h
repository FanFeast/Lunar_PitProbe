/*
Nestor Gomez
Carnegie Mellon University
Eng. Computation, 24-780-B
Prob Set 6
Due Mon. Oct. 26, 2020
*/
#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <chrono>



#define WIN_WIDTH 800
#define WIN_HEIGHT 600

//using namespace std;  // removed so that other files are not forced into it
class Node;
//class Slider;

class ViewManager {
	// this enum only exists in the context of ViewManager
	enum UserMode { none, editMode, addMode, deleteMode };

private:

	// simulation parameters
	bool simulationIsRunning;

	std::chrono::system_clock::time_point startTime; 
	// above can be set like this:  startTime = std::chrono::system_clock::now();
	// and then can do:             
	//    std::chrono::duration<double> elapsedTime = std::chrono::system_clock::now() - startTime;
	// which can then be used to get elapsed seconds:  elapsedSeconds = elapsedTime.count();

	// view preferences
	int panChange;
	double zoomFactor;

	int sliderColor;  // H value for slider color (S = 1, V = 1)
	int nodeColor;  // H value for node color (S = 1, V = 1)
	int lineWidth;
	bool showNodes;

	int xOrigin, yOrigin;  // screen coords of model coords 0,0
	int prevLocX, prevLocY;  // for zoom and pan, and for node edit
	double viewScale;  // must be greater than zero

	// to help with file names
	std::vector<std::string> allFEMfiles;

	// to manage add/edit
	UserMode currUserMode;
	Node * currNode;

	// manage undo of guide point edit
	Node* lastNode;
	double lastNodeX, lastNodeY;

public:
	ViewManager();
	void showMenu(); 
	// displays keycode choices on console
	
	int getOriginX() { return xOrigin; }
	int getOriginY() { return yOrigin; }
	double getScale() { return viewScale; }

	bool manage();
	// interprets user input and performs actions on the interface
	// returns false only when user presses ESC to end program

	void getScreenCoords(double modelX, double modelY,
		double& screenX, double& screenY);
	// given model coordinates, function calculates screen coordinates
	// converting for translation and scale

	void getModelCoords(double &modelX, double &modelY,
		double screenX, double screenY);
	// given screen coordinates, function calculates model coordinates
	// converting for translation and scale

	void screenVertex(double modelX, double modelY);
	// given model coordinates, function adds a vertex on screen
	// after converting for translation and scale

private:
	void addSlideBox();
	// YOU CODE THIS
	// takes input from user through console to add a box to model

	void removeSlideBox();
	// YOU CODE THIS
	// takes input from user through console to remove a box from model

	void load();
	// asks for a filename and loads a file into model

	void save();
	// asks for a filename and loads a file into model

	void centerOnScreen();
	// sets view parameters so that the model is centered on screen

	bool getConsoleYes();  // allows user to say Y/N on screen
	bool getScreenYes();   // allows user to say Y/N on console
	std::string getFileFromConsole();  // allows user to input file name in console
	
	// allows user to input file name on screen, with a list of available files
	// requires using C++17 compiler (set in project properties)
	std::string getFileFromScreen(std::vector<std::string>& availableFiles,
		const std::string& prompt);
	void getAvailableFiles(std::vector<std::string>& availableFiles);

	void drawModeIndicator();

	void highlightNode(Node& aNode);

};