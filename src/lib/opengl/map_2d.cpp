#include "map_2d.h"

#include <iostream>	//cout
#include <fstream>	//ifstream
#include <stdexcept>

#include <opencv/cv.h> 

#include <GL/glut.h>
#include <GL/freeglut_ext.h>	//glutMainLoopEvent

namespace hub {
namespace opengl {

#define VERBOSE 1

// little debug makro
#if VERBOSE >= 1
#define dout if(1) cout
#else
#define dout if(0) cout
#endif

#define rad2deg(r) ((r) * 180) / pi
#define deg2rad(d) ((d) * pi) / 180

using namespace std;

typedef struct {
	GLfloat x;
	GLfloat y;
	GLfloat z;
	GLfloat roll;
	GLfloat pitch;
	GLfloat yaw;
} position_t;

const double Map2D::pi = 3.1415926535897932384626433832795028841971693993751058209749;

int Map2D::window;
GLfloat rotation_matrix[16];
GLfloat zoom_factor = 1.0;
position_t camera_position;
position_t object_position;


Map2D::Map2D(int *argc, char **argv, const int width, const int height) {
	glutInit(argc, argv);
  
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(width, height); 

	window = glutCreateWindow("OpenGL 2D Map");
	//TODO: check window variable

	// define the color we use to clearscreen 
	glClearColor(0.0, 0.0, 0.0, 0.0);
	// set shading technique to flat
	glShadeModel(GL_FLAT);

	//init camera position
// 	camera_position.x = 0.75;
// 	camera_position.y = -0.75;
// 	camera_position.z = -0.75;
// 	camera_position.roll = 30.0;
// 	camera_position.pitch = -70.0;
// 	camera_position.yaw = 30;

	// init object position
// 	object_position.z = 1.0;
// 	object_position.yaw = 180.0;
// 	object_position.pitch = 180.0;

	glutDisplayFunc(display);
// 	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard_input);
	glutMouseFunc(mouse_input);
	glutPassiveMotionFunc(mouse_movement);
// 	glutIdleFunc(display);

}

void Map2D::bind_textures(std::vector<unsigned int> &ids) {
	// allocate a texture name
	glGenTextures(1, &ids[0]);

	//select texture
	for(std::vector<unsigned int>::iterator id_it = ids.begin(); id_it != ids.end(); ++id_it) {
		if(*id_it <= 0) continue;
		glBindTexture(GL_TEXTURE_2D, *id_it);
	}

	// select modulate to mix texture with color for shading
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, true);

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			GL_LINEAR_MIPMAP_NEAREST);
	// when texture area is large, bilinear filter the first mipmap
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// the texture ends at the edges (clamp)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
}

int Map2D::calc_rotation_matrix(float *roation_matrix, const float roll_deg, const float pitch_deg, const float yaw_deg) {
	if(!roation_matrix) return -1;

	//FIXME: use function/macro to convert between deg and rad
	GLfloat roll_rad = roll_deg * pi / 180;
	GLfloat pitch_rad = pitch_deg * pi / 180;
	GLfloat yaw_rad = yaw_deg * pi / 180;

	rotation_matrix[0] = cos(pitch_rad)*cos(yaw_rad);
	rotation_matrix[1] = -cos(roll_rad)*sin(yaw_rad) + sin(roll_rad)*sin(pitch_rad)*cos(yaw_rad);
	rotation_matrix[2] = sin(roll_rad)*sin(yaw_rad) + cos(roll_rad)*sin(pitch_rad)*cos(yaw_rad);
	rotation_matrix[3] = 0.0f;

	rotation_matrix[4] = cos(pitch_rad)*sin(yaw_rad);
	rotation_matrix[5] = cos(roll_rad)*cos(yaw_rad) + sin(roll_rad)*sin(pitch_rad)*sin(yaw_rad);
	rotation_matrix[6] = -sin(roll_rad)*cos(yaw_rad) + cos(roll_rad)*sin(pitch_rad)*sin(yaw_rad);
	rotation_matrix[7] = 0.0f;

	rotation_matrix[8] = -sin(pitch_rad);
	rotation_matrix[9] = sin(roll_rad)*cos(pitch_rad);
	rotation_matrix[10] = cos(roll_rad)*cos(pitch_rad);
	rotation_matrix[11] = 0.0f;

	rotation_matrix[12] = 0.0f;
	rotation_matrix[13] = 0.0f;
	rotation_matrix[14] = 0.0f;
	rotation_matrix[15] = 1.0f;
	
	return 0;
}

void Map2D::camera_direction(const float roll, const float pitch, const float yaw, bool deg) {
	//FIXME: use assignment method/macro fow angles
	if(deg) {
		camera_position.roll = roll;
		camera_position.pitch = pitch;
		camera_position.yaw = yaw;
	} else { //assume rad
		camera_position.roll = rad2deg(roll);
		camera_position.pitch = rad2deg(pitch);
		camera_position.yaw = rad2deg(yaw);
	}

	if(camera_position.roll >= 360.0)
		camera_position.roll -= 360.0;
	else if(camera_position.roll < 0.0)
		camera_position.roll += 360.0;

	if(camera_position.pitch >= 360.0)
		camera_position.pitch -= 360.0;
	else if(camera_position.pitch < 0.0)
		camera_position.pitch += 360.0;

	if(camera_position.yaw >= 360.0)
		camera_position.yaw -= 360.0;
	else if(camera_position.yaw < 0.0)
		camera_position.yaw += 360.0;
}

void Map2D::camera_view() {

	glTranslatef(-camera_position.x, -camera_position.y, -camera_position.z);
	calc_rotation_matrix(rotation_matrix, camera_position.roll, camera_position.pitch, camera_position.yaw);
	glMultMatrixf(rotation_matrix);

	// 	glScalef(x_scale, y_scale, z_scale);
}

void Map2D::display() {
	//clear color buffer and depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//clear transformation matrix
	glLoadIdentity();
	// transform OpenGL coordinate system to aeronautic system
	calc_rotation_matrix(rotation_matrix, 90.0, 180.0, 270.0);
	glMultMatrixf(rotation_matrix);

	// set camera view
	camera_view();

	display_grid();

	display_textures();

	glutSwapBuffers();

	post_process();
}

void Map2D::display_grid() {
	float max_x = 100.0;
	float max_y = 100.0;
	float delta = 0.1;

	//draw world grid
	glLineWidth(1.0);
	glDisable(GL_TEXTURE_2D);
	glBegin(GL_LINES);
	//lines from left to right
	glColor3f(1.0f, 1.0f, 1.0f);	//set grid color to white
	for(float i=0.0; i<=max_y; i+=delta) {
		glVertex2f(0.0f, i);
		glVertex2f(max_x, i);
	}
	//lines from back to front
	for(float i=0.0; i<=max_x; i+=delta) {
		glVertex2f(i, 0.0f);
		glVertex2f(i, max_y);
	}
	glEnd();

	//TODO: draw arrow tips
	glLineWidth(2.0);
	glBegin(GL_LINES);
	//draw arrows
	glColor3f(0, 0, 1);	//x axis is blue
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(1.0f, 0.0f, 0.0f); 
	glColor3f(1, 0, 0);	//y axis is red
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 1.0f, 0.0f); 
	glColor3f(0, 1, 0);	//z axis is green
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 1.0f); 
	glEnd();
}

void Map2D::display_textures() {
	glPushMatrix();

	calc_rotation_matrix(rotation_matrix, object_position.roll, object_position.pitch, object_position.yaw);
	glMultMatrixf(rotation_matrix);
	glTranslatef(object_position.x, object_position.y, object_position.z);

	//FIXME: use texture dimension instead
	GLfloat param = 1.0;

	glColor3f(1.0f, 1.0f, 1.0f);	//set canvas color to white
	glEnable(GL_TEXTURE_2D);
	/* texture
	 * bottom left: 0, 1
	 * bottom right: 1, 1
	 * top right: 1, 0
	 * top left: 0, 0
	 * */
	glBegin(GL_QUADS);
	glTexCoord2d(0.0, 1.0); glVertex2d(0.0, 0.0);		//bottom left
	glTexCoord2d(1.0, 1.0); glVertex2d(0.0, param);		//top left
	glTexCoord2d(1.0, 0.0); glVertex2d(param, param);	//top right
	glTexCoord2d(0.0, 0.0); glVertex2d(param, 0.0);		//bottom right
	glEnd();

	glPopMatrix();
}

void Map2D::keyboard_input(unsigned char key, int x, int y) {
	dout << "Pressed key " << key << " on coordinates (" << x << "," << y << ")" << endl;
	if(key == 'a') {	//sideway movement to left
		camera_position.y -= 0.1;
	} else if(key == 'd') {	//sideway movement to right
		camera_position.y += 0.1;
	} else if(key == 'e') {	//upwards movement
		camera_position.z -= 0.1;
	} else if(key == 'f') {	//downwards movement
		camera_position.z += 0.1;
	} else if(key == 's') {	//move backwards
		camera_position.x -=  0.1;
	} else if(key == 'w') {	//move forwards
		camera_position.x +=  0.1;
	} else if(key == 'q' || key == 27) {
		cout << "Got q,so quitting " << endl;
		glutDestroyWindow(window);
		exit(0);
	}

// 	display();
}

void Map2D::mouse_input(int button, int state, int x, int y) {
	if(state != GLUT_DOWN) return;

	switch(button) {
		case GLUT_LEFT_BUTTON:
// 			glutIdleFunc(rotate);
			break;
		case GLUT_RIGHT_BUTTON:
			glutIdleFunc(NULL);
			break;
	}
}

void Map2D::mouse_movement(int x, int y) {
	static int last_x, last_y;

	int delta_x = last_x - x; last_x = x;
	int delta_y = last_y - y; last_y = y;
	
	camera_position.yaw += static_cast<float>(delta_x);
	camera_position.pitch += static_cast<float>(delta_y);
	
// 	display();
}

void Map2D::load_texture(const unsigned int id, const char *image, const unsigned int width, const unsigned int height) throw(const std::exception&) {
	if(!image) throw std::domain_error("image argument is NULL pointer");

	//select texture with given id
	glBindTexture(GL_TEXTURE_2D, id);

	//FIXME: use glTexSumImage2D instead
//   glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE, pixels);

	//build texture mipmaps
	glTexImage2D(GL_TEXTURE_2D,	//target
		0,			//level
		3,			//internalformat
		width,			//width
		height,			//height
		0,			//border
		GL_BGR,			//format
		GL_UNSIGNED_BYTE,	//_type
		image);

// 	display();
/*	glutMainLoopEvent();*/
}

void Map2D::load_texture(const unsigned int id, const std::string &filename, const unsigned int width, const unsigned int height) throw(const std::exception&) {

	ifstream ifs( filename.c_str(), ifstream::in|ifstream::binary);
	if( !ifs.is_open() ) return;

	int image_size = width*height*3;

	//get length of file
	ifs.seekg (0, ios::end);
	int length = ifs.tellg();
	if(length < image_size)
		return;
	//rewind to beginning
	ifs.seekg (0, ios::beg);

	char *image = static_cast<char*>( malloc(image_size) );
	if(!image) return;

	ifs.read(image, image_size);
	ifs.close();

	load_texture(id, image, width, height);
	free(image);
}

void Map2D::post_process() {
	glutMainLoopEvent();
}

int Map2D::release_textures(const std::vector<unsigned int> &ids) {
	for(std::vector<unsigned int>::const_iterator id_it = ids.begin(); id_it != ids.end(); ++id_it) {
		if(*id_it <= 0) continue;
		glDeleteTextures(1, &(*id_it));
	}

	return 0;
}

// void Map2D::reshape(int width, int height) {
// 	if(height == 0) return;
// 
// 	glMatrixMode(GL_PROJECTION); //set the matrix to projection
// 	glLoadIdentity();
// 	//gluPerspective(fovy, aspect, zNear, zFar
// 	gluPerspective(50.0*zoom_factor, (GLfloat)width / (GLfloat)height, 0.0, 100.0);
// 	glMatrixMode(GL_MODELVIEW); //set the matrix back to model
// }

void Map2D::rotate(const float roll, const float pitch, const float yaw, bool deg) {
	if(deg) {
		camera_position.roll += roll;
		camera_position.pitch += pitch;
		camera_position.yaw += yaw;
	} else { //assume rad
		camera_position.roll += rad2deg(roll);
		camera_position.pitch += rad2deg(pitch);
		camera_position.yaw += rad2deg(yaw);
	}

	if(camera_position.roll >= 360.0)
		camera_position.roll -= 360.0;
	else if(camera_position.roll < 0.0)
		camera_position.roll += 360.0;

	if(camera_position.pitch >= 360.0)
		camera_position.pitch -= 360.0;
	else if(camera_position.pitch < 0.0)
		camera_position.pitch += 360.0;

	if(camera_position.yaw >= 360.0)
		camera_position.yaw -= 360.0;
	else if(camera_position.yaw < 0.0)
		camera_position.yaw += 360.0;

// 	display();
}

// int Map2D::translate(const float x, const float y, const float z) {
// 	//FIXME
// // 	x_translate = x;
// 	return 0;
// }

} // namespace opengl
} // namespace hub
