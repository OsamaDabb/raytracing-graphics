#include "ofApp.h"
#include "scene.h"
#include "utils.h"
#include "object.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>

//--------------------------------------------------------------
void ofApp::setup() {

	font.load("VERDANAI.TTF", 30);
	
	// true = parallel, false=perspective
	isParallel = false;
	movementType = 0;
	num_rotations = 0;
	selected_object = nullptr;
	int screen_scaling = 6;
	width = 640 * screen_scaling;
	height = 480 * screen_scaling;
	colorPixels.allocate(width, height, OF_PIXELS_RGB);

	left = -2;
	right = 2;
	bottom = -1.5;
	top = 1.5;

	d = 1;
	viewpoint = Vector3(0,0,-1);
	// light parameters
	lightSource = Vector3(0,8,0);
	lightIntensity = Vector3(255,255,255);

	// modified basis vecs so that u/v point right/up
	w_vec = Vector3(0,0,-1);
	Vector3 y_vec(0,1,0);
	u_vec = cross(w_vec, y_vec);
	v_vec = cross(u_vec, w_vec);

	// Spheres

	// Spheres
	objects.push_back(std::make_unique<Sphere>(
		Vector3(-1, -0.4, 3),
		Vector3(1.0, 0, 0) * 0.6,
		Vector3(1,1,1) * 0.2,
		Vector3(1,1,1) * 0.2, Vector3(1,1,1)*0, 1,
		"Red Small Sphere"
	));
	objects.push_back(std::make_unique<Sphere>(
		Vector3(2, 0.6, 4),
		Vector3(0, 1.0, 0) * 0.4,
		Vector3(1,1,1) * 0.2,
		Vector3(1,1,1) * 0.2,
		Vector3(1,1,1) * 0.2, 2,
		"Green Large Sphere"
	));

	// Planes
	objects.push_back(std::make_unique<Plane>(
		Vector3(0, -1.4, 0),
		Vector3(0, -1.4, 1),
		Vector3(1, -1.4, 0),
		Vector3(0.5, 0.5, 0.5) * 0.3,
		Vector3(1,1,1) * 0.1,
		Vector3(1,1,1) * 0.1,
		Vector3(1, 1, 1) * 0.5,
		"Floor Plane"
	));
	objects.push_back(std::make_unique<Plane>(
		Vector3(0, 1, 10),
		Vector3(1, 0, 10),
		Vector3(0, 0, 10),
		Vector3(0.5, 0.5, 0.5) * 0.18,
		Vector3(1,1,1) * 0.06,
		Vector3(1,1,1) * 0.06,
		Vector3(1, 1, 1) * 0.7,
		"Wall Plane"
	));

	// Ellipsoids
	objects.push_back(std::make_unique<Ellipsoid>(
		Vector3(-2, 2, 2),
		Vector3(1, 0.5, 0.5),
		Vector3(0, 0, 1.0) * 0.6,
		Vector3(1,1,1) * 0.2,
		Vector3(1,1,1) * 0.2,
		Vector3(1,1,1) * 0,
		"Blue Ellipsoid"
	));

	auto lightSphere = std::make_unique<Sphere>(
		lightSource,        // position
		Vector3(1,1,1),          // kd (unused)
		Vector3(1,1,1),          // ks (unused)
		Vector3(1,1,1),          // ka (unused)
		Vector3(1,1,1),			 // kr (unused)
		0.5f,                     // radius
		"Light Source"
	);
	lightSphere->isLight = true;
	lightSphere->emission = Vector3(255, 255, 220);
	objects.push_back(std::move(lightSphere));

}

//--------------------------------------------------------------
void ofApp::update() {

	auto start = std::chrono::high_resolution_clock::now();

	auto* p = colorPixels.getData(); // unsigned char*
	unsigned threads = std::max(1u, std::thread::hardware_concurrency());
	unsigned chunk   = (height + threads - 1) / threads;

	if (num_rotations > 0){
		lightSource = RotationMatrix(Vector3(1,0,0), 0.02) * lightSource;
		objects[objects.size() - 1]->center = lightSource;
		num_rotations -= 1;
	}
	if (num_centered_rotations > 0){
		Matrix ROT = RotationMatrix(Vector3(0,1,0), centered_rotation_angle);
		w_vec = ROT * w_vec;
		u_vec = ROT * u_vec;
		v_vec = ROT * v_vec;
		// as calculated in notebook
		viewpoint = centerOfRotation - ROT * (centerOfRotation - viewpoint);
		num_centered_rotations -= 1;
	}

	if (wPressed) {
		viewpoint = viewpoint - w_vec * prev_time_ms / 50;
	}
	if (sPressed){
		viewpoint = viewpoint + w_vec * prev_time_ms / 50;
	}
	if (aPressed){
		viewpoint = viewpoint - u_vec * prev_time_ms / 50;
	}
	if (dPressed){
		viewpoint = viewpoint + u_vec * prev_time_ms / 50;
	}


	std::vector<std::thread> pool;
	pool.reserve(threads);

	for (unsigned t = 0; t < threads; ++t) {
		unsigned y0 = t * chunk;
		unsigned y1 = std::min<unsigned>(height, y0 + chunk);

		pool.emplace_back([this, y0, y1, &p]() {
			for (unsigned y = y0; y < y1; ++y) {
				for (int x = 0; x < width; ++x) {
					float u = left + (right - left) * (x + 0.5f) / width;
					float v = top  - (top  - bottom) * (y + 0.5f) / height;

					Vector3 rayDir = isParallel ? -1 * w_vec
									: (-d * w_vec + u * u_vec + v * v_vec);
					rayDir = rayDir / rayDir.magnitude();
					Vector3 rayOrigin = isParallel ? (viewpoint + u*u_vec + v*v_vec)
												: viewpoint;

					Vector3 pix = detectCollision(rayOrigin, rayDir,
												  lightSource, lightIntensity,
												  objects, DEPTH);

					size_t i = 3 * (y * width + x);
					p[i+0] = (unsigned char)std::clamp(pix.x, 0.f, 255.f);
					p[i+1] = (unsigned char)std::clamp(pix.y, 0.f, 255.f);
					p[i+2] = (unsigned char)std::clamp(pix.z, 0.f, 255.f);
				}
			}
		});
	}
	for (auto& th : pool) th.join();
	texColor.loadData(colorPixels);


	auto stop = std::chrono::high_resolution_clock::now();
	prev_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	std::cout << "Raytracing time: " << prev_time_ms << " ms\n";

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetHexColor(0xffffff);
	texColor.draw(0, 0, width, height);

    string mode = (movementType == 0) ? "Translation" : (movementType == 1)? "Rotation" : "Character Rotation";
    ofSetColor(255);          // white text
	
    font.drawString("Mode: " + mode, 20, 40);

	string obj_name = selected_object != nullptr ? selected_object->name : "none";
	font.drawString("Selected Object: " + obj_name, 20, 90);

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	switch (key)
	{
	case 'p':
		isParallel = !isParallel;
		break;
	
	case OF_KEY_UP:
		if (selected_object != nullptr) {
			if (movementType == 0) {
				selected_object->moveObject(v_vec);
				if (selected_object->isLight){
					lightSource = lightSource + v_vec;
				}
			}
			else if (movementType == 1){
				selected_object->rotateObject(u_vec);
			}
		}
		if (movementType == 2){
			Matrix ROT = RotationMatrix(u_vec, -0.2);
			w_vec = ROT * w_vec;
			u_vec = ROT * u_vec;
			v_vec = ROT * v_vec;
		}
		break;
	case OF_KEY_DOWN:
		if (selected_object != nullptr) {
			if (movementType == 0) {
				selected_object->moveObject(-1 * v_vec);
				if (selected_object->isLight){
					lightSource = lightSource - v_vec;
				}
			}
			else if (movementType == 1){
				selected_object->rotateObject(-1 * u_vec);
			}
		}
		if (movementType == 2) {
				Matrix ROT = RotationMatrix(u_vec, 0.2);
				w_vec = ROT * w_vec;
				u_vec = ROT * u_vec;
				v_vec = ROT * v_vec;
			}
		break;

	case OF_KEY_RIGHT:
		if (selected_object != nullptr) {
			if (movementType == 0) {
				selected_object->moveObject(u_vec);
				if (selected_object->isLight){
					lightSource = lightSource + u_vec;
				}
			}
			else if (movementType == 1){
				selected_object->rotateObject(-1 * v_vec);
			}
		}
		if (movementType == 2) {
				Matrix ROT = RotationMatrix(Vector3(0,1,0), 0.2);
				w_vec = ROT * w_vec;
				u_vec = ROT * u_vec;
				v_vec = ROT * v_vec;
			}
		break;

	case OF_KEY_LEFT:
		if (selected_object != nullptr) {
			if (movementType == 0) {
				selected_object->moveObject(-1 * u_vec);
				if (selected_object->isLight){
					lightSource = lightSource - u_vec;
				}
			}
			else if (movementType == 1){
				selected_object->rotateObject(v_vec);
			}
		}
		if (movementType == 2) {
				Matrix ROT = RotationMatrix(Vector3(0,1,0), -0.2);
				w_vec = ROT * w_vec;
				u_vec = ROT * u_vec;
				v_vec = ROT * v_vec;
			}
		break;

	case 'j':
		if (selected_object != nullptr) {
			if (movementType == 1){
				selected_object->rotateObject(-1 * w_vec);
			}
			else{
				selected_object->moveObject(-1 * w_vec);
				if (selected_object->isLight){
					lightSource = lightSource - w_vec;
				}
			}
		}
		break;

	case 'l':
		if (selected_object != nullptr) {
			if (movementType == 1){
				selected_object->rotateObject(w_vec);
			}
			else{
				selected_object->moveObject(w_vec);
				if (selected_object->isLight){
					lightSource = lightSource + w_vec;
				}
			}
		}
		break;
	case 'm':
		if (movementType == 2){
			movementType = 0;
		}
		else{
			movementType += 1;
		}
		break;

	case '=': //secretly this is '+'
		if (selected_object != nullptr) {
			selected_object->scaleObject(1.1);
		}
		break;

	case '-':
		if (selected_object != nullptr) {
			selected_object->scaleObject(0.9);
		}
		break;	

	case 'r':
		if (selected_object != nullptr) {
			selected_object->resetObject();
		}
		else{
			for (auto& obj : objects){
				obj->resetObject();
			}
		}
		break;
		
	case ' ':
		viewpoint = viewpoint + v_vec;
		break;

	case OF_KEY_SHIFT:
		viewpoint = viewpoint - v_vec;
		break;

	case 'w':
		wPressed = true;
		break;

	case 's':
		sPressed = true;
		break;

	case 'a':
		aPressed = true;
		break;

	case 'd':
		dPressed = true;
		break;

	case 't': {
		viewpoint = Vector3(0,0,-1);
		w_vec = Vector3(0,0,-1);
		Vector3 y_vec(0,1,0);
		u_vec = cross(w_vec, y_vec);
		v_vec = cross(u_vec, w_vec);
		lightSource = Vector3(0,5,0);
		objects[objects.size() - 1]->center = lightSource;
		break;
	}

	case 'o':
		num_rotations = int(floor(8000 / prev_time_ms));
		break;
	
	case '[':
		{
			centered_rotation_angle = 0.1;
			num_centered_rotations = int(round(2*PI / centered_rotation_angle));
			break;
		}

	case ']':
		{
			
			centered_rotation_angle = -0.1;
			num_centered_rotations = int(round(2*PI / abs(centered_rotation_angle)));
			break;
		}
	default:
		break;
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	switch (key) {
		case 'w':
			wPressed = false;
			break;

		case 's':
			sPressed = false;
			break;

		case 'a':
			aPressed = false;
			break;

		case 'd':
			dPressed = false;
			break;
		default:
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
	if (button == 0){
		int diff_x = x - lastMouse.x;
		int diff_y = y - lastMouse.y;

		if (horizontalRot == 0){
			horizontalRot = abs(diff_x) > abs(diff_y) ? 1 : 2;
		}
		
		if (horizontalRot == 1){
			Matrix x_ROT = RotationMatrix(v_vec, diff_x*-0.005);
			w_vec = x_ROT * w_vec;
			u_vec = x_ROT * u_vec;
			v_vec = x_ROT * v_vec;
		}
		else{
			Matrix y_ROT = RotationMatrix(u_vec, diff_y*-0.005);
			w_vec = y_ROT * w_vec;
			u_vec = y_ROT * u_vec;
			v_vec = y_ROT * v_vec;
		}
		lastMouse = Vector3(x,y,0);
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

	if (button == 0) {
		lastMouse = Vector3(x,y,0);
		horizontalRot = 0;
	}
	else{
		float u = left + (right - left) * (x + 0.5f) / width;
		float v = top  - (top  - bottom) * (y + 0.5f) / height;

		Vector3 rayDir = isParallel ? -1 * w_vec
						: (-d * w_vec + u * u_vec + v * v_vec);
		rayDir = rayDir / rayDir.magnitude();
		Vector3 rayOrigin = isParallel ? (viewpoint + u*u_vec + v*v_vec)
									: viewpoint;

		
		selected_object = nullptr;
		float closest_t = INFINITY;
		HitInfo hit;

		for (auto& obj : objects) {
			hit = obj->shadow(rayOrigin, rayDir, closest_t);
			if(hit.wasCollision) {
				selected_object = obj.get();
				closest_t = hit.t;
			}
		}
	}

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}