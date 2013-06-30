#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKinectInpainter.h"
#include "ofxBlobTracker.h"
#include "ofxKinectBlobTracker.h"
#include "ofxKinectBlobTracker.h"

#include "ofxUI.h"
#include "ofxXmlSettings.h"

#include "ofxOsc.h"

#define BACKGROUND_FRAMES 100

#define MAP_POINTS 4

#define PORT 12000
#define IP "localHost"

class testApp : public ofBaseApp, public ofxKinectBlobListener{
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
    void blob2DAdded(ofxBlob &_blob);
    void blob2DMoved(ofxBlob &_blob);
    void blob2DDeleted(ofxBlob &_blob);
    
    void blobOn( ofVec3f centroid, int id, int order );
    void blobMoved( ofVec3f centroid, int id, int order);
    void blobOff( ofVec3f centroid, int id, int order );

	ofxKinect               kinect;
    ofxBlobTracker          blob2DTracker;
    
    ofxKinectBlobFinder     blobFinder;
    ofxKinectBlobTracker    blobTracker;
    ofImage                 blobImage;
    
    ofxKinectInpainter      inPainter;
    ofFloatPixels           background;
    ofxCvGrayscaleImage     backgroundTex;
    
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
    float minBlobVol;
    float maxBlobVol;

    ofFbo zonesFbo;
    vector<ofPoint> zones;
    float zonesDistance;
    bool zonesNew;
    bool zonesClear;
    void saveZones();
    void loadZones();
    void drawZones();
    void addZone(ofPoint z);

    ofFbo mapFbo;
    ofPixels mapPixels;
    ofPoint map[MAP_POINTS];
    ofPoint mapScreen[MAP_POINTS];
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
