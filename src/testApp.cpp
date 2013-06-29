#include "testApp.h"
#include "homography.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

    ofEnableAlphaBlending();
    ofSetPolyMode(OF_POLY_WINDING_NONZERO);

	backgroundImg.loadImage("mapa.jpg");

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

    angle=0;
	//kinect.setCameraTiltAngle(angle);
	//ofSleepMillis(1000);

	kinect.enableDepthNearValueWhite(true);

    ofAddListener(blobTracker.blobAdded, this, &testApp::blobAdded);
    ofAddListener(blobTracker.blobMoved, this, &testApp::blobMoved);
    ofAddListener(blobTracker.blobDeleted, this, &testApp::blobDeleted);
    
    background.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    backgroundTex.allocate(kinect.width,kinect.height);//,OF_IMAGE_GRAYSCALE);
    inPainter.setup(kinect.width, kinect.height);

    tmpMapMask = new unsigned char[kinect.width*kinect.height];
    tmpZonesMask = new unsigned char[kinect.width*kinect.height];

    nearThreshold=10000.;
    farThreshold=10000.;

    diffThreshold=100.;

    maxBlobs = 10;
    minBlobPoints=250;
    maxBlobPoints=1000000;
    
    zonesCols=3;
    zonesRows=3;
    zonesColSpacing=20;
    zonesRowSpacing=20;

    dilate=10;
    erode=10;
    
    pitch=0.;
    roll=0.;
    rotation.makeIdentityMatrix();
    
    screenRef[0]=ofPoint(0,0);
	screenRef[1]=ofPoint(640,0);
	screenRef[2]=ofPoint(640,480);
	screenRef[3]=ofPoint(0,480);
    mapPoint=0;
	mapFbo.allocate(kinect.width,kinect.height);
	mapPixels.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    mapMask.allocate(kinect.width, kinect.height);
    
    zonesFbo.allocate(kinect.width,kinect.height);
	zonesPixels.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    zonesMask.allocate(kinect.width, kinect.height);

    gui = new ofxUISuperCanvas("kinectMap", OFX_UI_FONT_MEDIUM);
    gui->addSpacer();
    gui->addTextArea("CONTROL", "Control de parametros de kinectMap");
    gui->addSpacer();
    gui->addSlider("angle", -30, 30, &angle);
    gui->addLabelToggle("learnBackground", &learnBackground);
    gui->addSlider("backFrames", 0.0, BACKGROUND_FRAMES, &backFrames);
    gui->addSpacer();
    gui->addSlider("maxBlobs", 0, 20, &maxBlobs);
    gui->addSlider("min blob points", 0, 2000, &minBlobPoints);
    gui->addSpacer();
    gui->addRangeSlider("near and far threshold", 0., 5000., &nearThreshold,&farThreshold);
    gui->addSlider("diff threshold", 0., 1000., &diffThreshold);
    gui->addSpacer();
    gui->addLabelToggle("mapOpen", &mapOpen);
    gui->addSpacer();
    gui->addSlider("zonesCols", 1, 10, &zonesCols);
    gui->addSlider("zonesRows", 1, 10, &zonesRows);
    gui->addSlider("zonesColSpacing", 0, 50, &zonesColSpacing);
    gui->addSlider("zonesRowSpacing", 0, 50, &zonesRowSpacing);
    gui->addSpacer();
    gui->addSlider("dilate", 0, 20, &dilate);
    gui->addSlider("erode", 0, 20, &erode);
    gui->autoSizeToFitWidgets();
    ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);

    if(ofFile::doesFileExist("GUI/guiSettings.xml"))
        gui->loadSettings("GUI/guiSettings.xml");

    loadMap();
    
    drawZones();

    sender.setup(IP,PORT);

    mapOpen=false;

    learnBackground = true;
    backFrames=0.;

	ofSetFrameRate(60);

}

//--------------------------------------------------------------
void testApp::update() {

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load depth image from the kinect source
		ofFloatPixels current=kinect.getDistancePixelsRef();

        int numPixels = kinect.width*kinect.height;
        unsigned char * tmpCurrent = kinect.getDepthPixels();
        unsigned char * tmpBackground = backgroundTex.getPixels();

        if(learnBackground || backFrames)
        {
            if(backFrames){
                for(int i=0;i<numPixels;i++)
                {
                    if(background[i]){
                        if(current[i]){
                            background[i]=(background[i]+current[i])/2.;
                            unsigned int mean = tmpCurrent[i]+ tmpBackground[i];
                            tmpBackground[i] = (unsigned char)(mean/2);
                        }
                    }
                    else{
                        background[i]=current[i];
                        tmpBackground[i]=tmpCurrent[i];
                    }
                }
                
                pitch=0.9*pitch+0.1*kinect.getAccelPitch();
                roll=0.9*roll+0.1*kinect.getAccelRoll();
                rotation.makeRotationMatrix(roll,ofVec3f(1,0,0),pitch,ofVec3f(0,1,0),0,ofVec3f(0,0,1));
                backFrames--;
                if(!backFrames){
                    //inPainter.inpaint(backgroundTex);
                }
            }
            if(learnBackground){
                for(int i=0;i<numPixels;i++)
                {
                    background[i]=current[i];
                    tmpBackground[i]= tmpCurrent[i];
                }
                
                pitch=kinect.getAccelPitch();
                roll=kinect.getAccelRoll();
                
                backFrames=BACKGROUND_FRAMES;
                learnBackground = false;
            }
        }

        float backUpdateFast=0.005;
        float backUpdateSlow=0.0005;

        for(int i=0;i<numPixels;i++){
            tmpMapMask[i]=0;
            tmpZonesMask[i]=0;
            float tmpBackDist = background[i];
            float tmpBackImg = tmpBackground[i];
            if(current[i]){
                tmpBackDist = (1.-backUpdateFast)*background[i] + backUpdateFast*current[i];
                tmpBackImg = (1.-backUpdateFast)*((float)tmpBackground[i]) + backUpdateFast*((float)tmpCurrent[i]);
            }
            if(current[i]<farThreshold && current[i]>nearThreshold)
            {
                if(mapPixels[i]){
                    float diff=background[i]-current[i];
                    if(diff>diffThreshold){
                        tmpMapMask[i]=255;
                        tmpBackDist = (1.-backUpdateSlow)*background[i] + backUpdateSlow*current[i];
                        tmpBackImg = (1.-backUpdateSlow)*((float)tmpBackground[i]) + backUpdateSlow*((float)tmpCurrent[i]);
                        if(zonesPixels[i]){
                            tmpZonesMask[i]=255;//(unsigned char)ofMap(diff,touchDiffNearThreshold,touchDiffFarThreshold,100,255);
                        }
                    }
                }
            }
            background[i] = tmpBackDist;
            tmpBackground[i] = (unsigned char)tmpBackImg;
        }
        mapMask.setFromPixels(tmpMapMask, kinect.width, kinect.height);
        zonesMask.setFromPixels(tmpZonesMask, kinect.width, kinect.height);

        cvErode(mapMask.getCvImage(), mapMask.getCvImage(), NULL, erode);
        cvDilate(mapMask.getCvImage(), mapMask.getCvImage(), NULL, dilate);

        cvAnd(zonesMask.getCvImage(),mapMask.getCvImage(),zonesMask.getCvImage());
        
        blobTracker.update( mapMask, -1, minBlobPoints , maxBlobPoints, maxBlobs, 20, false, true);

        backgroundTex.flagImageChanged();

    }

}

//--------------------------------------------------------------
void testApp::draw() {
	if(mapOpen){
        ofSetColor(255);
        kinect.drawDepth(0, 0, ofGetWidth(), ofGetHeight());
        ofSetColor(255, 0 , 0, 60);
        mapFbo.draw(0, 0, ofGetWidth(), ofGetHeight());
    }
    else{
		ofBackground(100, 100, 100);

		ofSetColor(255, 255, 255);
		// draw from the live kinect
		kinect.drawDepth(10, 10, 320, 240);

		backgroundTex.draw(340,10,320,240);

		ofSetColor(255, 0 , 0);
		mapMask.draw(15, 260, 640, 480);

		ofSetColor(255, 255, 255, 80);
		zonesMask.draw(15, 260, 640, 480);

		ofSetColor(255,0,255,60);
		mapFbo.draw(15,260,640,480);

		ofSetColor(255,255,0,100);
		zonesFbo.draw(15,260,640,480);

		blobTracker.draw(15,260,640,480);

		// draw instructions
		ofSetColor(255, 255, 255);
		stringstream reportStream;
		reportStream << "blobs: " << blobTracker.size() << ", pitch: " << pitch << ", roll: " << roll << ", fps: " << ofToString(ofGetFrameRate(),2) << endl
		<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
		ofDrawBitmapString(reportStream.str(),20,750);
		if(backFrames)
		{
			ofDrawBitmapString("WAIT!",360,100);
		}
	}
}

//--------------------------------------------------------------
void testApp::blobAdded(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " added" );
    int x=_blob.centroid.x*kinect.width;
    int y=_blob.centroid.y*kinect.height;
    if(zonesPixels[x+y*kinect.width]){
        int z=255-zonesPixels[x+y*kinect.width];
        ofxOscMessage m;
        m.setAddress("/play");
        m.addIntArg(int(z%zonesCols));
        m.addIntArg(int(z/zonesCols));
        sender.sendMessage(m);
    }
}

//--------------------------------------------------------------
void testApp::blobMoved(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " moved" );
    int x=_blob.centroid.x*kinect.width;
    int y=_blob.centroid.y*kinect.height;
    if(zonesPixels[x+y*kinect.width]){
        int z=255-zonesPixels[x+y*kinect.width];
        ofxOscMessage m;
        m.setAddress("/play");
        m.addIntArg(int(z%zonesCols));
        m.addIntArg(int(z/zonesCols));
        sender.sendMessage(m);
    }
}

//--------------------------------------------------------------
void testApp::blobDeleted(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " deleted" );
}

//--------------------------------------------------------------
void testApp::guiEvent(ofxUIEventArgs &e)
{
    string name = e.widget->getName();
    if(name=="angle")
    {
        if(angle>30.) angle=30.;
        if(angle<-30.) angle=-30.;
        kinect.setCameraTiltAngle(angle);
    }
    else if(name=="near and far threshold"){
        kinect.setDepthClipping(nearThreshold,farThreshold);
    }
    else if(name=="zonesCols" || name=="zonesRows" || name=="zonesColSpacing" || name=="zonesRowSpacing"){
        drawZones();
    }
}

//--------------------------------------------------------------
void testApp::exit(){
    //kinect.setCameraTiltAngle(0);
	kinect.close();

    gui->saveSettings("GUI/guiSettings.xml");

    saveMap();

    delete tmpMapMask;
    delete tmpZonesMask;

    delete gui;
}

//--------------------------------------------------------------
void testApp::loadMap(){
    if(ofFile::doesFileExist("0.map")){
        ofBuffer buf=ofBufferFromFile("0.map");
        int i=0;
        //Read file line by line
        while (!buf.isLastLine() && i<MAP_POINTS) {
            string line = buf.getNextLine();
            //Split line into strings
            vector<string> p = ofSplitString(line, ",");
            map[i++]=ofPoint(ofToInt(p[0]),ofToInt(p[1]));
        }
	}
	homography=findHomography(screenRef,map);
    drawMap();
}

//--------------------------------------------------------------
void testApp::saveMap(){
    ofBuffer buf;
    for(int i=0;i<MAP_POINTS;i++){
        buf.append(ofToString(map[i])+"\n");
    }
    ofBufferToFile("0.map",buf);
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
        case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
    }

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    ofxUIRectangle * guiWindow = gui->getRect();
    if(!guiWindow->inside(x,y)){//mouseX>10 && mouseX<340 && mouseY>260 && mouseY<500){
        if(mapOpen){
            int mX=x*kinect.width/ofGetWidth();//2*(mouseX-10);
            int mY=y*kinect.height/ofGetHeight();;//2*(mouseY-260);
            map[mapPoint++]=ofPoint(mX,mY);
            /*int n=0;
            int r=5;
            while(!n && r<kinect.height){
                for(mY=0;mY<kinect.height;mY++){
                    for(mX=0;mX<kinect.width;mX++){
                        if(map[mapPoint].distance(ofPoint(mX,mY))<r){
                            map[mapPoint].z+=kinect.getDistanceAt(x,y);
                            n++;
                        }
                    }
                }
                r++;
            }
            if(n)
                map[mapPoint++].z/=n;*/
            if(mapPoint>=MAP_POINTS){
                mapOpen=false;
                mapPoint=0;
            }
            homography=findHomography(screenRef,map);
            drawMap();
            drawZones();
        }
    }
}

//--------------------------------------------------------------
void testApp::drawZones(int a)
{
    int ww=(kinect.width-zonesCols*zonesColSpacing)/zonesCols;
    int hh=(kinect.height-zonesRows*zonesRowSpacing)/zonesRows;
    zonesFbo.begin();
    ofClear(0,0);
    int z=0;
    for(int y=0;y<zonesRows;y++){
        for(int x=0;x<zonesCols;x++){
            if (z==a)   ofSetColor(125);
            else        ofSetColor(255-(z++));
            ofPoint p0=homography*ofPoint(zonesColSpacing*(x+0.5)+ww*x,zonesRowSpacing*(y+0.5)+hh*y);
            ofPoint p1=homography*ofPoint(zonesColSpacing*(x+0.5)+ww*(x+1),zonesRowSpacing*(y+0.5)+hh*y);
            ofPoint p2=homography*ofPoint(zonesColSpacing*(x+0.5)+ww*(x+1),zonesRowSpacing*(y+0.5)+hh*(y+1));
            ofPoint p3=homography*ofPoint(zonesColSpacing*(x+0.5)+ww*x,zonesRowSpacing*(y+0.5)+hh*(y+1));
            ofBeginShape();
            ofVertex(p0);
            ofVertex(p1);
            ofVertex(p2);
            ofVertex(p3);
            ofEndShape();
        }
    }
    zonesFbo.end();
    ofPixels tmpPixels;
    zonesFbo.readToPixels(tmpPixels);
    for(int i=0;i<zonesPixels.getWidth()*zonesPixels.getHeight();i++){
        zonesPixels[i]=tmpPixels[i*4];
    }
}

//--------------------------------------------------------------
void testApp::drawMap()
{
    mapFbo.begin();
    ofClear(0,0);
    ofSetColor(255);
    ofBeginShape();
    ofVertex(map[0].x,map[0].y);
    ofVertex(map[1].x,map[1].y);
    ofVertex(map[2].x,map[2].y);
    ofVertex(map[3].x,map[3].y);
    ofEndShape();
    mapFbo.end();
    ofPixels tmpPixels;
    mapFbo.readToPixels(tmpPixels);
    for(int i=0;i<mapPixels.getWidth()*mapPixels.getHeight();i++){
        mapPixels[i]=tmpPixels[i*4];
    }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
}
