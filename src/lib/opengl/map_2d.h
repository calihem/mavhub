#ifndef _HUB_OPENGL_MAP_2D_H_
#define _HUB_OPENGL_MAP_2D_H_

#include <string>
#include <vector>
// #include <list>
// #include <map>
// #include <sstream> //istringstream

#include <opencv/cxtypes.h>

namespace hub {
namespace opengl {

class Map2D  {
	public:
		Map2D(int *argc, char **argv, const int width = 640, const int height = 480);

		static const double pi;

		static void bind_textures(std::vector<unsigned int> &ids);
		static void camera_direction(const float roll, const float pitch, const float yaw, bool deg = true);
		static void display();
		static void load_texture(const unsigned int id, const char *image, const unsigned int width, const unsigned height) throw(const std::exception&); 
		static void load_texture(const unsigned int id, const IplImage &image) throw(const std::exception&);
		static void load_texture(const unsigned int id, const std::string &filename, const unsigned int width, const unsigned int height) throw(const std::exception&);
		static int release_textures(const std::vector<unsigned int> &ids);
		static void rotate(const float roll, const float pitch, const float yaw, bool deg = true);
// 		static int translate(const float x, const float y, const float z);

	protected:

	private:
		Map2D(const Map2D &);
		void operator=(const Map2D &);

		static int window;
		static int calc_rotation_matrix(float *rotation_matrix, const float roll_deg, const float pitch_deg, const float yaw_deg);
		static void camera_view();
/*		static void display();*/
		static void display_grid();
		static void display_textures();
		static void keyboard_input(unsigned char key, int x, int y);
		static void mouse_input(int button, int state, int x, int y);
		static void mouse_movement(int x, int y);
		static void post_process();
// 		static void reshape(int width, int height);
};

inline void Map2D::load_texture(const unsigned int id, const IplImage &image) throw(const std::exception&) {
	load_texture(id, image.imageData, image.width, image.height);
}

} // namespace opengl
} // namespace hub

#endif
