#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectInpainter.h"
#include "ofxBlobTracker.h"

#include "ofxUI.h"
#include "ofxXmlSettings.h"

#include "ofxOsc.h"

#define BACKGROUND_FRAMES 100

#define MAP_POINTS 4

#define PORT 12000
#define IP "localHost"

class testApp : public ofBaseApp{
public:

	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed (int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    void blobAdded(ofxBlob &_blob);
    void blobMoved(ofxBlob &_blob);
    void blobDeleted(ofxBlob &_blob);

    void getBackground();

    ofImage backgroundImg;

	ofxKinect kinect;
    ofxBlobTracker          blobTracker;
    ofxKinectInpainter inPainter;
    ofFloatPixels background;
    ofxCvGrayscaleImage backgroundTex;
    int dilate;
    int erode;

    unsigned char * tmpMapMask;
    unsigned char * tmpZonesMask;

    float nearThreshold;
    float farThreshold;

    float diffThreshold;

	bool learnBackground;
	int backFrames;
	int angle;

	int minBlobPoints;
    int maxBlobPoints;
    int maxBlobs;

    ofFbo zonesFbo;
    ofPixels zonesPixels;
    int zonesCols;
    int zonesRows;
    int zonesColSpacing;
    int zonesRowSpacing;
    void drawZones(int a=-1);

    ofxCvGrayscaleImage zonesMask;

    ofFbo mapFbo;
    ofPixels mapPixels;
    ofPoint map[MAP_POINTS];
    ofPoint screenRef[MAP_POINTS];
    int mapPoint;
    bool mapOpen;
    void saveMap();
    void loadMap();
    void drawMap();

    ofxCvGrayscaleImage mapMask;
    
    float pitch,roll;
    ofMatrix4x4 rotation;

	ofMatrix4x4 homography;

    ofxOscSender sender;

    ofxUISuperCanvas *gui;
	void guiEvent(ofxUIEventArgs &e);

};
