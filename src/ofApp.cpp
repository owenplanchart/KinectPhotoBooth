#include "ofApp.h"

//--------------------------------------------------------------
static inline bool isValidDepth(const ofVec3f& p, int nearClip, int farClip) {
    return p.z > 0 && p.z >= nearClip && p.z <= farClip;
}

//--------------------------------------------------------------
void rebuildPointCloudMesh(ofxKinect& kinect, ofMesh& mesh, bool& hasCloud,
                           int nearClip, int farClip) {
    const int w = kinect.getWidth();
    const int h = kinect.getHeight();
    const int step = 1; // 1 = full res (≈307 k verts)
    
    mesh.clear();
    mesh.setMode(OF_PRIMITIVE_TRIANGLES);
    
    std::vector<int> idxMap(w * h, -1);
    const bool haveRGB = kinect.getPixels().isAllocated();
    
    // Pass 1: vertices + colors
    for (int y = 0; y < h; y += step) {
        for (int x = 0; x < w; x += step) {
            if (kinect.getDistanceAt(x, y) <= 0) continue;
            ofVec3f p = kinect.getWorldCoordinateAt(x, y);
            if (!isValidDepth(p, nearClip, farClip)) continue;
            
            idxMap[y * w + x] = mesh.getNumVertices();
            mesh.addVertex(p);
            if (haveRGB)
                mesh.addColor(ofFloatColor(kinect.getColorAt(x, y)));
            else
                mesh.addColor(ofFloatColor::white);
        }
    }
    
    // Pass 2: two triangles per 2×2 cell
    auto addTri = [&](int a, int b, int c) {
        mesh.addIndex(a); mesh.addIndex(b); mesh.addIndex(c);
    };
    for (int y = 0; y < h - 1; ++y) {
        for (int x = 0; x < w - 1; ++x) {
            int i00 = idxMap[y * w + x];
            int i10 = idxMap[y * w + (x + 1)];
            int i01 = idxMap[(y + 1) * w + x];
            int i11 = idxMap[(y + 1) * w + (x + 1)];
            
            if (i00 >= 0 && i10 >= 0 && i01 >= 0) addTri(i00, i10, i01);
            if (i10 >= 0 && i11 >= 0 && i01 >= 0) addTri(i10, i11, i01);
        }
    }
    
    hasCloud = mesh.getNumVertices() > 0;
}
void computeAndFlipNormals(ofMesh& mesh, bool flip = true) {
    mesh.clearNormals();
    
    const size_t nV = mesh.getNumVertices();
    if (nV == 0 || mesh.getNumIndices() < 3) return;
    
    std::vector<ofVec3f> acc(nV, ofVec3f(0,0,0));
    const auto& idx = mesh.getIndices();
    
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        int ia = idx[i], ib = idx[i+1], ic = idx[i+2];
        const ofVec3f& a = mesh.getVertex(ia);
        const ofVec3f& b = mesh.getVertex(ib);
        const ofVec3f& c = mesh.getVertex(ic);
        ofVec3f n = (b - a).cross(c - a);           // face normal (not normalized)
        acc[ia] += n; acc[ib] += n; acc[ic] += n;   // accumulate
    }
    
    // normalize + optional flip
    for (auto& n : acc) {
        if (n.lengthSquared() > 0) n.normalize();
        if (flip) n *= -1;
    }
    
    // write into the mesh (0.10.1 style)
    auto& normals = mesh.getNormals();
    normals.resize(nV);
    for (size_t i = 0; i < nV; ++i) normals[i] = acc[i];
}

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofBackground(20);
    
    // Kinect v1 setup
    kinect.setRegistration(true);
    kinect.init();     // depth enabled
    kinect.open();     // open first device
    kinect.listDevices();
    
    // GUI
    gui.setup("panel");
    gui.add(nearClip.set("nearClip", 500, 100, 2000));
    gui.add(farClip.set("farClip", 800, 500, 8000));
    
    // Assets
    font.load("frabk.ttf", 50);
    // beep.load("Beep_Short.mp3");
    // click.load("Camera-shutter-sound-effect.mp3");
    
    b_drawPointCloud = false;
    b_drawGui = true;
    b_saving = false;
    
    pointCloud.setMode(OF_PRIMITIVE_TRIANGLES);
    counter = 0.0f;
    countDown = 0;
    hasCloud = false;
}

//--------------------------------------------------------------
void ofApp::update() {
    kinect.update();
    if (!kinect.isConnected()) return;
    
    if (kinect.isFrameNew()) {
        rebuildPointCloudMesh(kinect, pointCloud, hasCloud,
                              nearClip.get(), farClip.get());
    }
    
    // Countdown + save
    const uint64_t now = ofGetSystemTimeMillis();
    const int delay = 1000;

    if (b_saving) {
        if (now > (uint64_t)counter + delay) {
            if (countDown == 1) {
                if (click.isLoaded()) click.play();
                if (hasCloud) {
                    auto &idx = pointCloud.getIndices();
                    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
                        std::swap(idx[i + 1], idx[i + 2]);
                    }
                    
                    // 2) Recompute normals (no extra flip now)
                    computeAndFlipNormals(pointCloud, /*flip=*/false);
                    std::string fn =
                    "cloud_" + ofGetTimestampString("%Y-%m-%d_%H-%M-%S") + ".ply";
                    auto fullPath = ofToDataPath(fn, true);
                    computeAndFlipNormals(pointCloud, true);  // flip=true makes "underside" darker/top brighter
                    pointCloud.save(fullPath);
                    ofLogNotice() << "Saved " << fullPath
                    << " verts=" << pointCloud.getNumVertices()
                    << " indices=" << pointCloud.getNumIndices();
                } else {
                    ofLogError() << "No vertices in pointCloud; skipping save.";
                }
                countDown = 20;
                b_saving = true;
                counter = now;
            } else {
                countDown--;
                counter = now;
                if (beep.isLoaded()) beep.play();
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    const bool drawCloudNow = b_drawPointCloud || (countDown == 1);
    
    if (drawCloudNow) {
        cam.begin();
        drawPointCloud();
        cam.end();
    } else {
        // debug views
        kinect.drawDepth(10, 10, 400, 300);
        kinect.draw(420, 10, 400, 300);
    }
    
    if (b_drawGui) {
        gui.draw();
        ofDrawBitmapStringHighlight(
                                    "FPS: " + ofToString(ofGetFrameRate(), 1) +
                                    " | verts: " + ofToString(pointCloud.getNumVertices()) +
                                    " | faces: " + ofToString(pointCloud.getNumIndices() / 3),
                                    10, ofGetHeight() - 10);
    }
    
    if (b_saving) {
        const std::string s = ofToString(countDown);
        font.drawString(s,
                        ofGetWidth() / 5 - font.stringWidth(s) / 2,
                        ofGetHeight() / 5);
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
    switch (key) {
        case 'p': b_drawPointCloud = !b_drawPointCloud; break;
        case 'g': b_drawGui = !b_drawGui; break;
        case 'f': ofToggleFullscreen(); break;
        case 's':
            b_saving = true;
            counter = ofGetSystemTimeMillis();
            countDown = 3;
            if (beep.isLoaded()) beep.play();
            break;
        default: break;
    }
}

//--------------------------------------------------------------
void ofApp::drawPointCloud() {
    if (!hasCloud || pointCloud.getNumVertices() == 0) {
        ofDrawBitmapStringHighlight("No cloud yet", 20, 60);
        return;
    }
    glPointSize(3);
    ofEnableDepthTest();
    ofPushMatrix();
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000);
    pointCloud.draw(); // draw with faces
    ofPopMatrix();
    ofDisableDepthTest();
}
//--------------------------------------------------------------

