#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

        void keyPressed(int key);
//        void keyReleased(int key);
//        void mouseMoved(int x, int y );
//        void mouseDragged(int x, int y, int button);
//        void mousePressed(int x, int y, int button);
//        void mouseReleased(int x, int y, int button);
//        void mouseEntered(int x, int y);
//        void mouseExited(int x, int y);
//        void windowResized(int w, int h);
//        void dragEvent(ofDragInfo dragInfo);
//        void gotMessage(ofMessage msg);
    
    void drawPointCloud();
    
   // int nearClip, farClip;
    ofxKinect kinect;
    
    bool b_drawPointCloud, b_drawGui, b_saving;
    
    ofEasyCam cam;
    
    // declare our GUI items
    ofxPanel gui;
    ofParameter<int> nearClip;
    ofParameter<int> farClip;
    
    ofMesh pointCloud;
    bool hasCloud = false;
    vector<int> pointIndex;


    
    float counter;
    int countDown;
    
    ofSoundPlayer beep;
    ofSoundPlayer click;
    ofTrueTypeFont font;
};
