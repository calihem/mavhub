#ifndef _HUB_OPENGL_MAP_2D_H_
#define _HUB_OPENGL_MAP_2D_H_

#include <string>
#include <vector>
// #include <list>
// #include <map>
// #include <sstream> //istringstream

//#include <opencv/cxtypes.h>
// #include <opencv/cxcore.h>

#include <GL/glut.h>
#include <GL/freeglut_ext.h>	//glutMainLoopEvent

namespace hub {
namespace opengl {

class Map2D  {
	typedef struct {
		GLfloat x;
		GLfloat y;
		GLfloat z;
		GLfloat roll;
		GLfloat pitch;
		GLfloat yaw;
	} position_t;

	public:
		Map2D(int *argc, char **argv, const int width = 640, const int height = 480);

		static const double pi;

		static void bind_textures(std::vector<unsigned int> &ids);
		static void camera_direction(const float roll, const float pitch, const float yaw, bool deg = true);
		static void display();
		static void load_texture(const unsigned int id,
			const char *image,
			const unsigned int width,
			const unsigned int height,
			const unsigned int bpp = 24) throw(const std::exception&); 
		static void load_texture(const unsigned int id,
			const std::string &filename,
			const unsigned int width,
			const unsigned int height) throw(const std::exception&);
		static int release_textures(const std::vector<unsigned int> &ids);
		static void rotate(const float roll, const float pitch, const float yaw, bool deg = true);
// 		static int translate(const float x, const float y, const float z);

	protected:

	private:
		static int m_width, m_height;
		static int window;
		static GLfloat rotation_matrix[16];
		static GLfloat zoom_factor;
		static position_t camera_position;
		static position_t object_position;

		Map2D(const Map2D &);
		void operator=(const Map2D &);

		static int calc_rotation_matrix(float *rotation_matrix, const float roll_deg, const float pitch_deg, const float yaw_deg);
		static void camera_view();
/*		static void display();*/
		static void display_grid();
		static void display_camera_orientation();
		static void display_textures();
		static void keyboard_input(unsigned char key, int x, int y);
		static void mouse_input(int button, int state, int x, int y);
		static void mouse_movement(int x, int y);
		static void print(const std::string &text);
		static void post_process();
// 		static void reshape(int width, int height);
};

} // namespace opengl
} // namespace hub

#endif
