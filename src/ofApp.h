#include "ofMain.h"
#include "object.h"

class ofApp : public ofBaseApp{
	
	public:
		
		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);		
		
		ofTexture 		texColor;

		int 			width, height;
		float     left, right, top, bottom;

		bool isParallel;
		bool wPressed, aPressed, sPressed, dPressed;
		int movementType;
		Vector3 u_vec, v_vec, w_vec, viewpoint;
		Vector3 lightIntensity, lightSource;
		float d;
		std::vector<std::unique_ptr<Object>> objects;
		int num_rotations, num_centered_rotations;
		float centered_rotation_angle;
		int64_t prev_time_ms;
		const int DEPTH = 5;
		int horizontalRot = 0;
		const Vector3 centerOfRotation{0,0,3};

		Object* selected_object;
		ofTrueTypeFont font;
		Vector3 lastMouse;
		
		ofPixels colorPixels;		
};