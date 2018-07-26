#include <opencv2/opencv.hpp>
#include <GL/gl.h>
namespace OpenGLModule {
class XOpenGLModule {

 public:
  XOpenGLModule(const std::string &window_name_,
                const cv::Mat &intrinsic,
                const cv::Mat &extrinsic);
  ~XOpenGLModule();
  void Init(const cv::Mat &img);
  void UpdateWindow();
  void UpdateFrame( cv::Mat &img);
  void DrawTexture();

 private:
  bool draw_is_ready;
  std::string openGL_window_name;
  cv::Mat image_texture;
  cv::Mat camera_intrinsic ;
  cv::Mat camera_distortion;
  std::vector<std::vector<double> > projection_vector_mat;

  void RenderBackgroundTexture();
  void Render3Dmodul();
  std::vector<cv::Point2f> MarkerDetect(cv::Mat &img);
  bool ProjectionMatrix(const cv::Mat &camera_intrinsic,
                        const cv::Mat &camera_distortion,
                        std::vector<cv::Point2f> &marker_points);

  friend void render(void *prama);

};
}
