#include "testApp.h"
#include "homography.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

    ofEnableAlphaBlending();
    ofSetPolyMode(OF_POLY_WINDING_NONZERO);

    // enable depth->rgb image calibration
	kinect.setRegistration(true);

	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	kinect.open();

    angle=kinect.getCurrentCameraTiltAngle();
	//kinect.setCameraTiltAngle(angle);
	//ofSleepMillis(1000);

	kinect.enableDepthNearValueWhite(true);

    ofAddListener(blob2DTracker.blobAdded, this, &testApp::blob2DAdded);
    ofAddListener(blob2DTracker.blobMoved, this, &testApp::blob2DMoved);
    ofAddListener(blob2DTracker.blobDeleted, this, &testApp::blob2DDeleted);
    
    blobFinder.init(&kinect, true); // standarized coordinate system: z in the direction of gravity
    blobFinder.setResolution(BF_LOW_RES);
    blobFinder.setRotation( ofVec3f( angle, 0, 0) );
    blobFinder.setTranslation(ofVec3f(0,0,0));
    blobFinder.setScale(ofVec3f(0.001, 0.001, 0.001)); // mm to meters
    // bind our kinect to the blob finder
    // in order to do this we need to declare in testApp.h: class testApp : public ofBaseApp, public ofxKinectBlobListener
    blobTracker.setListener( this );
    
    blobImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    
    background.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    backgroundTex.allocate(kinect.width, kinect.height);//,OF_IMAGE_GRAYSCALE);
    inPainter.setup(kinect.width, kinect.height);

    tmpMapMask = new unsigned char[kinect.width*kinect.height];
    tmpZonesMask = new unsigned char[kinect.width*kinect.height];

    nearThreshold=10000.;
    farThreshold=10000.;

    diffThreshold=100.;

    maxBlobs = 10;
    minBlobPoints=250;
    maxBlobPoints=1000000;
    // NOTE: measurement units in meters!!!
    minBlobVol = 0.02f;
    maxBlobVol = 2.0f;
    
    /*zonesCols=3;
    zonesRows=3;
    zonesColSpacing=20;
    zonesRowSpacing=20;*/

    dilate=10;
    erode=10;
    
    pitch=0.;
    roll=0.;
    rotation.makeIdentityMatrix();
    
    mapPoint=0;
	mapFbo.allocate(kinect.width,kinect.height);
	mapPixels.allocate(kinect.width,kinect.height,OF_IMAGE_GRAYSCALE);
    mapMask.allocate(kinect.width, kinect.height);
    
    zonesDistance=10.;
    zonesFbo.allocate(kinect.width,kinect.height);

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
    gui->addSlider("min blob vol", 0.0, 0.2, &minBlobVol);
    gui->addSlider("max blob vol", 1., 10., &maxBlobVol);
    gui->addSpacer();
    gui->addRangeSlider("near and far threshold", 0., 5000., &nearThreshold,&farThreshold);
    gui->addSlider("diff threshold", 0., 1000., &diffThreshold);
    gui->addSpacer();
    gui->addLabelToggle("mapOpen", &mapOpen);
    gui->addSpacer();
    gui->addLabelButton("zonesNew", &zonesNew);
    gui->addLabelButton("zonesClear", &zonesClear);
    gui->addSpacer();
    /*gui->addSlider("zonesCols", 1, 10, &zonesCols);
    gui->addSlider("zonesRows", 1, 10, &zonesRows);
    gui->addSlider("zonesColSpacing", 0, 50, &zonesColSpacing);
    gui->addSlider("zonesRowSpacing", 0, 50, &zonesRowSpacing);
    gui->addSpacer();*/
    gui->addSlider("dilate", 0, 20, &dilate);
    gui->addSlider("erode", 0, 20, &erode);
    gui->autoSizeToFitWidgets();
    ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);

    if(ofFile::doesFileExist("GUI/guiSettings.xml"))
        gui->loadSettings("GUI/guiSettings.xml");

    loadMap();
    
    loadZones();

    sender.setup(IP,PORT);

    mapOpen=false;
    
    zonesNew=false;
    zonesClear=false;

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
                float tmpPitch=kinect.getAccelPitch();
                float tmpRoll=kinect.getAccelRoll();
                if(tmpPitch!=NAN && tmpRoll!=NAN){
                    pitch=0.9*pitch+0.1*tmpPitch;
                    roll=0.9*roll+0.1*tmpRoll;
                }
                //rotation.makeRotationMatrix(-90-pitch,ofVec3f(1,0,0),0,ofVec3f(0,1,0),-roll,ofVec3f(0,0,1));
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
                float diff=background[i]-current[i];
                if(diff>diffThreshold){
                    tmpMapMask[i]=255;
                    tmpBackDist = (1.-backUpdateSlow)*background[i] + backUpdateSlow*current[i];
                    tmpBackImg = (1.-backUpdateSlow)*((float)tmpBackground[i]) + backUpdateSlow*((float)tmpCurrent[i]);
                }
            }
            background[i] = tmpBackDist;
            tmpBackground[i] = (unsigned char)tmpBackImg;
        }
        mapMask.setFromPixels(tmpMapMask, kinect.width, kinect.height);

        cvErode(mapMask.getCvImage(), mapMask.getCvImage(), NULL, erode);
        cvDilate(mapMask.getCvImage(), mapMask.getCvImage(), NULL, dilate);
        
        blob2DTracker.update( mapMask, -1, minBlobPoints , maxBlobPoints, maxBlobs, 20, false, true);
        
        blobImage.setFromPixels(mapMask.getPixels(), kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    

        blobFinder.findBlobs( &blobImage,
                             ofVec3f(-10, -10, -10), ofVec3f(10, 10, 10),
                             ofVec3f(1.,1.,1.), 1,
                             minBlobVol, maxBlobVol, minBlobPoints,  maxBlobs);
        blobTracker.trackBlobs( blobFinder.blobs );

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

		ofSetColor(255,0,255,60);
		mapFbo.draw(15,260,640,480);

		ofSetColor(255,255,0,150);
		zonesFbo.draw(15,260,640,480);

		blob2DTracker.draw(15,260,640,480);
        
        ofSetColor(0,255,0);
        for(int i=0;i<blobTracker.blobs.size();i++){
            ofVec3f b = homography*blobTracker.blobs[i].centroid;
            ofCircle(15+b.x,260+b.y,10);
        }

		// draw instructions
		ofSetColor(255, 255, 255);
		stringstream reportStream;
		reportStream << "zones: " << zones.size() << ", blobs: " << blobFinder.nBlobs << ", pitch: " << pitch << ", roll: " << roll << ", fps: " << ofToString(ofGetFrameRate(),2) << endl
		<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
		ofDrawBitmapString(reportStream.str(),20,750);
		if(backFrames)
		{
			ofDrawBitmapString("WAIT!",360,100);
		}
	}
}

//--------------------------------------------------------------
void testApp::blobOn( ofVec3f centroid, int id, int order ) {
    cout << "blobOn() - id: " << id << " order: " << order << endl;
    cout << "centroid: " << centroid << endl;
    ofVec3f projection=rotation*blobTracker.blobs[blobTracker.getIndexById(id)].centroid;
    blobTracker.blobs[blobTracker.getIndexById(id)].indicator=-1;
    float diff = zonesDistance;
    
    for (int i=0; i < zones.size(); i++) {
        cout<<"proj "<<projection<<" vs zone "<<zones[i]<<endl;
        if(projection.distanceSquared(zones[i])<diff){
            diff=projection.distanceSquared(zones[i]);
            blobTracker.blobs[blobTracker.getIndexById(id)].indicator=i;
        }
    }
    
    if(blobTracker.blobs[blobTracker.getIndexById(id)].indicator!=-1){
        ofxOscMessage m;
        m.setAddress("/play");
        m.addIntArg(blobTracker.blobs[blobTracker.getIndexById(id)].indicator);
        sender.sendMessage(m);
    }
    cout<<"id "<<id<<" is in zone "<<blobTracker.blobs[blobTracker.getIndexById(id)].indicator<<endl;
}

//--------------------------------------------------------------
void testApp::blobMoved( ofVec3f centroid, int id, int order) {
    cout << "blobMoved() - id:" << id << " order:" << order << endl;
    // full access to blob object ( get a reference)
    //  ofxKinectTrackedBlob blob = blobTracker.getById( id );
    // cout << "volume: " << blob.volume << endl;
    ofVec3f projection=rotation*blobTracker.blobs[blobTracker.getIndexById(id)].centroid;
    blobTracker.blobs[blobTracker.getIndexById(id)].indicator=-1;
    float diff = zonesDistance;
    
    for (int i=0; i < zones.size(); i++) {
        cout<<"proj "<<projection<<" vs zone "<<zones[i]<<endl;
        if(projection.distanceSquared(zones[i])<diff){
            diff=projection.distanceSquared(zones[i]);
            blobTracker.blobs[blobTracker.getIndexById(id)].indicator=i;
        }
    }
    
    if(blobTracker.blobs[blobTracker.getIndexById(id)].indicator!=-1){
        ofxOscMessage m;
        m.setAddress("/play");
        m.addIntArg(blobTracker.blobs[blobTracker.getIndexById(id)].indicator);
        sender.sendMessage(m);
    }
    cout<<"id "<<id<<" is in zone "<<blobTracker.blobs[blobTracker.getIndexById(id)].indicator<<endl;
}

//--------------------------------------------------------------
void testApp::blobOff( ofVec3f centroid, int id, int order ) {
    // cout << "blobOff() - id:" << id << " order:" << order << endl;
}

//--------------------------------------------------------------
void testApp::blob2DAdded(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " added" );
}

//--------------------------------------------------------------
void testApp::blob2DMoved(ofxBlob &_blob){
    //ofLog(OF_LOG_NOTICE, "Blob ID " + ofToString(_blob.id) + " moved" );
}

//--------------------------------------------------------------
void testApp::blob2DDeleted(ofxBlob &_blob){
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
    else if(name=="zonesNew"){
        if(zonesNew){
            if(blobFinder.nBlobs){
                ofVec3f projection=rotation*blobTracker.blobs[0].centroid;
                addZone(projection);
                cout<<"zone: "<<zones.size()<<" point:"<<projection<<endl;
            }
            drawZones();
        }
    }
    else if(name=="zonesClear"){
        if(zonesClear){
            zones.clear();
        }
    }
}

//--------------------------------------------------------------
void testApp::exit(){
    //kinect.setCameraTiltAngle(0);
	kinect.close();

    gui->saveSettings("GUI/guiSettings.xml");

    saveMap();
    saveZones();

    delete tmpMapMask;
    delete tmpZonesMask;

    delete gui;
}

//--------------------------------------------------------------
void testApp::loadZones(){
    if(ofFile::doesFileExist("0.zones")){
        ofBuffer buf=ofBufferFromFile("0.zones");
        int i=0;
        //Read file line by line
        while (!buf.isLastLine()) {
            string line = buf.getNextLine();
            //Split line into strings
            vector<string> p = ofSplitString(line, ",");
            addZone(ofPoint(ofToFloat(p[0]),ofToFloat(p[1]),ofToFloat(p[2])));
        }
	}
    drawZones();
}

//--------------------------------------------------------------
void testApp::saveZones(){
    ofBuffer buf;
    for(int i=0;i<zones.size();i++){
        buf.append(ofToString(zones[i])+"\n");
    }
    ofBufferToFile("0.zones",buf);
}

//--------------------------------------------------------------
void testApp::addZone(ofPoint z){
    zones.push_back(z);
    if(zones.size()>1){
        zonesDistance=zones[0].distanceSquared(zones[1]);
        for(int i=0;i<zones.size();i++){
            for(int j=i+1;j<zones.size();j++){
                if(zones[i].distanceSquared(zones[j])<zonesDistance){
                    zonesDistance=zones[i].distanceSquared(zones[j]);
                }
            }
        }
    }
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
            map[i]=ofPoint(ofToFloat(p[0]),ofToFloat(p[1]),ofToFloat(p[2]));
            mapScreen[i++]=ofPoint(ofToInt(p[3]),ofToInt(p[4]),ofToInt(p[5]));
        }
	}
	homography=findHomography(map,mapScreen);
    drawMap();
}

//--------------------------------------------------------------
void testApp::saveMap(){
    ofBuffer buf;
    for(int i=0;i<MAP_POINTS;i++){
        buf.append(ofToString(map[i])+", "+ofToString(mapScreen[i])+"\n");
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
        if(mapOpen && blobFinder.nBlobs){
            int mX=x*kinect.width/ofGetWidth();//2*(mouseX-10);
            int mY=y*kinect.height/ofGetHeight();;//2*(mouseY-260);
            mapScreen[mapPoint]=ofPoint(mX,mY);
            map[mapPoint++]=rotation*blobTracker.blobs[0].centroid;
            if(mapPoint>=MAP_POINTS){
                mapOpen=false;
                mapPoint=0;
            }
            homography=findHomography(map,mapScreen);
            drawMap();
            drawZones();
        }
    }
}

//--------------------------------------------------------------
void testApp::drawZones()
{
    zonesFbo.begin();
    ofClear(0,0);
    int z=0;
    for(int i=0;i<zones.size();i++){
        ofVec3f p = homography*map[i];
        ofSetColor(ofRandom(255),ofRandom(255),ofRandom(255));
        ofCircle(p.x,p.y,30);
    }
    zonesFbo.end();
}

//--------------------------------------------------------------
void testApp::drawMap()
{
    mapFbo.begin();
    ofClear(0,0);
    ofSetColor(255);
    ofBeginShape();
    for(int i=0;i<MAP_POINTS;i++){
        ofVec3f p = homography*map[i];
        ofVertex(p.x,p.y);
    }
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
