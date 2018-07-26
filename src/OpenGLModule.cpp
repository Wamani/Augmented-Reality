#include <iostream>
#include "opencv_opengl_demo/OpenGLModule.h"
#define WINDOW_SIZE 0.5
GLfloat projectionMatrix[16];

void render(void * param)
{
  OpenGLModule::XOpenGLModule * ptr = static_cast<OpenGLModule::XOpenGLModule*>(param);
  if(ptr != NULL)
    ptr->DrawTexture();
}
namespace OpenGLModule {
OpenGLModule::XOpenGLModule::XOpenGLModule(const std::string &window_name_,
                                           const cv::Mat &intrinsic,
                                           const cv::Mat &extrinsic)
    : openGL_window_name(window_name_),
      camera_intrinsic(intrinsic),
      camera_distortion(extrinsic) {
}

OpenGLModule::XOpenGLModule::~XOpenGLModule(){

  cv::setOpenGlDrawCallback(openGL_window_name, 0, 0);
}
void OpenGLModule::XOpenGLModule::Init(const cv::Mat &img) {

  cv::namedWindow(openGL_window_name, cv::WINDOW_OPENGL);
  cv::resizeWindow(openGL_window_name, img.cols, img.rows);
  cv::setOpenGlContext(openGL_window_name);
  cv::setOpenGlDrawCallback(openGL_window_name, render,this);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_FRONT);


  int width = img.rows;
  int height= img.cols;

  float near = 0.01;  // Near clipping distance
  float far = 100;  // Far clipping distance

  float f_x = 579.814392; // Focal length in x axis
  float f_y = 598.97229; // Focal length in y axis (usually the same?)
  float c_x = 309.472961; // Camera primary point x
  float c_y = 280.152069; // Camera primary point y
  projectionMatrix[0] =  - 2.0 * f_x / height;
  projectionMatrix[1] = 0.0;
  projectionMatrix[2] = 0.0;
  projectionMatrix[3] = 0.0;

  projectionMatrix[4] = 0.0;
  projectionMatrix[5] = 2.0 * f_y / width;
  projectionMatrix[6] = 0.0;
  projectionMatrix[7] = 0.0;

  projectionMatrix[8] = 2.0 * c_x / height - 1.0;
  projectionMatrix[9] = 2.0 * c_y / width - 1.0;
  projectionMatrix[10] = -( far+near ) / ( far - near );
  projectionMatrix[11] = -1.0;

  projectionMatrix[12] = 0.0;
  projectionMatrix[13] = 0.0;
  projectionMatrix[14] = -2.0 * far * near / ( far - near );
  projectionMatrix[15] = 0.0;

}
void OpenGLModule::XOpenGLModule::DrawTexture() {
  //获取纹理对象
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // Clear entire screen:
  RenderBackgroundTexture();
  // draw
  Render3Dmodul();
  glFlush();
}
void OpenGLModule::XOpenGLModule::UpdateWindow() {
  cv::updateWindow(openGL_window_name);
}
void OpenGLModule::XOpenGLModule::UpdateFrame( cv::Mat &img){
  image_texture = img;
  std::vector<cv::Point2f> marker_points_;

  marker_points_ = MarkerDetect(img);
  draw_is_ready = ProjectionMatrix(camera_intrinsic,camera_distortion,marker_points_);
}

std::vector<cv::Point2f> OpenGLModule::XOpenGLModule::MarkerDetect(cv::Mat &image) {

  cv::Mat grayImage, tempImage;
  cv::cvtColor(image, grayImage, CV_BGR2GRAY);

  threshold(grayImage, tempImage, 0, 255, cv::THRESH_OTSU);
  cv::Mat OpenElement = cv::getStructuringElement(cv::MORPH_RECT,
                                                  cv::Size(3, 3), cv::Point(1, 1));
  cv::Mat CloseElement = cv::getStructuringElement(cv::MORPH_RECT,
                                                   cv::Size(3, 3), cv::Point(1, 1));
  //morphologyEx(tempImage, tempImage, MORPH_OPEN, OpenElement, Point(-1, -1), 1);
  cv::morphologyEx(tempImage, tempImage, cv::MORPH_CLOSE, CloseElement, cv::Point(-1, -1), 1);
  std::vector<std::vector<cv::Point> > all_contours;
  std::vector<std::vector<cv::Point> > contours;
  //Rect RectArea;
  //求外接圆
  cv::findContours(tempImage, all_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  for (int i = 0; i < all_contours.size(); ++i) {
    if (all_contours[i].size() > 10) {
      contours.push_back(all_contours[i]);
    }
  }
  std::vector<cv::Point> approxCurve;//返回结果为多边形，用点集表示//相似形状
  cv::Rect RectArea;
  for (size_t i = 0; i < contours.size(); i++) {
    /*近似一个多边形逼近，为了减少轮廓的像素。这样比较好，可筛选出非标记区域，因为标记总能被四个顶点的多边形表示。如果多边形的顶点多于或少于四个，就绝对不是本项目想要的标记。通过点集近似多边形，第三个参数为epsilon代表近似程度，即原始轮廓及近似多边形之间的距离，第四个参数表示多边形是闭合的。*/
    double eps = contours[i].size() * 0.05;
    //输入图像的2维点集，输出结果，估计精度，是否闭合。输出多边形的顶点组成的点集//使多边形边缘平滑，得到近似的多边形
    approxPolyDP(contours[i], approxCurve, eps, true);
    //drawContours(srcImage, contours, i, Scalar(0, 0, 255), 1, 8);
    //我们感兴趣的多边形只有四个顶点
    if (approxCurve.size() != 4)
      continue;

    //检查轮廓是否是凸边形
    if (!isContourConvex(approxCurve))
      continue;

    //确保连续点之间的距离是足够大的。//确保相邻的两点间的距离“足够大”－大到是一条边而不是短线段就是了
    //float minDist = numeric_limits<float>::max();//代表float可以表示的最大值，numeric_limits就是模板类，这里表示max（float）;3.4e038
    float minDist = 1e10;//这个值就很大了
    float maxDist = 0;//这个值就很大了
    //求当前四边形各顶点之间的最短距离
    //求当前四边形各顶点之间的最短距离
    for (int j = 0; j < 4; j++) {
      cv::Point side = approxCurve[j] - approxCurve[(j + 1) % 4];//这里应该是2维的相减
      //Point side1 = approxCurve[j+1] - approxCurve[(j + 2) % 4];//这里应该是2维的相减
      float squaredSideLength = side.dot(side);//求2维向量的点积，就是XxY
      //float squaredSideLength1 = side1.dot(side1);//求2维向量的点积，就是XxY
      minDist = cv::min(minDist, squaredSideLength);//找出最小的距离
      maxDist = cv::max(maxDist, squaredSideLength);//找出最小的距离
    }
    float delta = (maxDist - minDist) / maxDist;
    float MIN_LENGTH = 50;
    //检查最长边与最短边的差值，小于一定阈值继续
    if ((delta > MIN_LENGTH) || (minDist < 500))
      continue;
    std::vector<cv::Point2f> Ximage;                        //图像坐标 用来求解透视矩阵
//    cv::drawContours(image, contours, i, cv::Scalar(255, 0, 255), 1, 8);
    for (int j = 0; j < 4; j++) {
      Ximage.push_back(approxCurve[j]);

    }
    //标记信息处理
    std::vector<cv::Point2f> MarkerPoint;                        //图像坐标 用来求解透视矩阵
    //透视变换后的Marker大小
    MarkerPoint.push_back(cv::Point2f(0, 0));
    MarkerPoint.push_back(cv::Point2f(0, 69));
    MarkerPoint.push_back(cv::Point2f(69, 69));
    MarkerPoint.push_back(cv::Point2f(69, 0));
    cv::Mat marker_image;
    cv::Mat M = getPerspectiveTransform(Ximage, MarkerPoint);
    Ximage.clear();
    warpPerspective(grayImage, marker_image, M, cv::Size(70, 70));
    threshold(marker_image,
              marker_image,
              125,
              255,
              cv::THRESH_BINARY | cv::THRESH_OTSU); //OTSU determins threshold automatically.
    //A marker must has a whole black border.
    bool badMarker = 0;
    for (int y = 0; y < 7; ++y) {
      int inc = (y == 0 || y == 6) ? 1 : 6;
      int cell_y = y * 10;

      for (int x = 0; x < 7; x += inc) {
        int cell_x = x * 10;
        int none_zero_count = countNonZero(marker_image(cv::Rect(cell_x, cell_y, 10, 10)));
        if (none_zero_count > 10 * 10 / 4) {
          badMarker = 1;
          break;
        }

      }
    }
    if (badMarker) continue;
    if (countNonZero(marker_image(cv::Rect(10, 10, 10, 10))) < 10 * 10 / 2) {
      Ximage.push_back(approxCurve[0]);
      Ximage.push_back(approxCurve[1]);
      Ximage.push_back(approxCurve[2]);
      Ximage.push_back(approxCurve[3]);
    } else if (countNonZero(marker_image(cv::Rect(10, 50, 10, 10))) < 10 * 10 / 2) {
      Ximage.push_back(approxCurve[1]);
      Ximage.push_back(approxCurve[2]);
      Ximage.push_back(approxCurve[3]);
      Ximage.push_back(approxCurve[0]);
    } else if (countNonZero(marker_image(cv::Rect(50, 50, 10, 10))) < 10 * 10 / 2) {
      Ximage.push_back(approxCurve[2]);
      Ximage.push_back(approxCurve[3]);
      Ximage.push_back(approxCurve[0]);
      Ximage.push_back(approxCurve[1]);
    } else if (countNonZero(marker_image(cv::Rect(50, 10, 10, 10))) < 10 * 10 / 2) {
      Ximage.push_back(approxCurve[3]);
      Ximage.push_back(approxCurve[0]);
      Ximage.push_back(approxCurve[1]);
      Ximage.push_back(approxCurve[2]);
    } else continue;
//    circle(image, Ximage[0], 10, cv::Scalar(0, 255, 255), -1, 8);

//    cv::imshow("s",image);
    //circle(srcImage, Ximage[0], 10, Scalar(0, 0, 255), -1, 8);
    // Sort the points in anti - clockwise

    cv::Point2f v1 = Ximage[1] - Ximage[0];
    cv::Point2f v2 = Ximage[2] - Ximage[0];
    if (v1.cross(v2) > 0)    //由于图像坐标的Y轴向下，所以大于零才代表逆时针
    {
      std::swap(Ximage[1], Ximage[3]);
    }
//    cv::cornerSubPix(Ximage, Ximage, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));
    //possible_markers.push_back(marker);
//    circle(image, Ximage[0], 10, cv::Scalar(0, 255, 255), -1, 8);
//    circle(image, Ximage[1], 10, cv::Scalar(0, 255, 255), -1, 8);
//    circle(image, Ximage[2], 10, cv::Scalar(0, 255, 255), -1, 8);
//    circle(image, Ximage[3], 10, cv::Scalar(0, 255, 255), -1, 8);
    return Ximage;
  }
  std::vector<cv::Point2f> return_value;

  return return_value;
}

bool OpenGLModule::XOpenGLModule::ProjectionMatrix(const cv::Mat &camera_intrinsic,
                                                   const cv::Mat &camera_distortion,
                                                   std::vector<cv::Point2f> &marker_points) {

  if (camera_intrinsic.empty() || camera_distortion.empty()) return 0;
  if (0 == marker_points.size()) return 0;

  cv::Mat rvec, tvec;
  std::vector<cv::Point3f> Xworld;                        //世界坐标系
  Xworld.push_back(cv::Point3f(-0.5, -0.5, 0));
  Xworld.push_back(cv::Point3f(0.5, -0.5, 0));
  Xworld.push_back(cv::Point3f(0.5, 0.5, 0));
  Xworld.push_back(cv::Point3f(-0.5, 0.5, 0));

  cv::solvePnP(Xworld, marker_points, camera_intrinsic, camera_distortion, rvec, tvec);
  cv::Mat rmat;
//  cv::Mat extinsic;
  cv::Rodrigues(rvec, rmat);
//  cv::hconcat(rmat,tvec,extinsic);

//  std::cout<<"extrinsic:"<<extinsic<<std::endl;
//  std::cout<<"tmat"<<tvec<<std::endl;
//    Ximage.clear();
  //绕X轴旋转180度，从OpenCV坐标系变换为OpenGL坐标系
  static double d[] =
      {
          -1, 0, 0,
          0, -1, 0,
          0, 0, -1
      };
  cv::Mat_<double> rx(3, 3, d);
  rmat = rx * rmat;
  tvec = rx * tvec;
//  std::cout<<"tvec:"<<tvec<<std::endl;
  std::vector<double> projection_vector;
  projection_vector.push_back(rmat.at<double>(0, 0));
  projection_vector.push_back(rmat.at<double>(1, 0));
  projection_vector.push_back(rmat.at<double>(2, 0));
  projection_vector.push_back(0.0f);

  projection_vector.push_back(rmat.at<double>(0, 1));
  projection_vector.push_back(rmat.at<double>(1, 1));
  projection_vector.push_back(rmat.at<double>(2, 1));
  projection_vector.push_back(0.0f);

  projection_vector.push_back(rmat.at<double>(0, 2));
  projection_vector.push_back(rmat.at<double>(1, 2));
  projection_vector.push_back(rmat.at<double>(2, 2));
  projection_vector.push_back(0.0f);

  projection_vector.push_back(tvec.at<double>(0, 0));
  projection_vector.push_back(tvec.at<double>(0, 1));
  projection_vector.push_back(tvec.at<double>(0, 2));

//  projection_vector.push_back(rmat.at<double>(0, 0));
//  projection_vector.push_back(rmat.at<double>(0, 1));
//  projection_vector.push_back(rmat.at<double>(0, 2));
//  projection_vector.push_back(0.0f);
//
//  projection_vector.push_back(rmat.at<double>(1, 0));
//  projection_vector.push_back(rmat.at<double>(1, 1));
//  projection_vector.push_back(rmat.at<double>(1, 2));
//  projection_vector.push_back(0.0f);
//
//  projection_vector.push_back(rmat.at<double>(2, 0));
//  projection_vector.push_back(rmat.at<double>(2, 1));
//  projection_vector.push_back(rmat.at<double>(2, 2));
//  projection_vector.push_back(0.0f);
//
//  projection_vector.push_back(tvec.at<double>(0, 0));
//  projection_vector.push_back(tvec.at<double>(0, 1));
//  projection_vector.push_back(tvec.at<double>(0, 2));
  projection_vector.push_back(1.0f);
  //RotMat.pop_back(rotMatrix);
  projection_vector_mat.push_back(projection_vector);
  projection_vector.clear();
  return 1;
}

void OpenGLModule::XOpenGLModule::RenderBackgroundTexture() {
  cv::Mat img =  image_texture;
  //OpenGL纹理用整型数表示
  GLuint texture_ID;

  float w = img.cols;
  float h = img.rows;
//  GLubyte* pixels;
//
//  //获取图像指针
//  int pixellength = width*height * 3;
//  pixels = new GLubyte[pixellength];
//  memcpy(pixels, img, pixellength * sizeof(char));
//  imshow("OpenCV", img);

  //将texture_ID设置为2D纹理信息
  glGenTextures(1, &texture_ID);
  glBindTexture(GL_TEXTURE_2D, texture_ID);
  //纹理放大缩小使用线性插值
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //纹理水平竖直方向外扩使用重复贴图
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//  纹理水平竖直方向外扩使用边缘像素贴图(与重复贴图二选一)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glBindTexture(GL_TEXTURE_2D, texture_ID);
  //将图像内存用作纹理信息
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, img.data);

  const GLfloat bgTextureVertices[] = { 0, 0, w, 0, 0, h, w, h };
  const GLfloat bgTextureCoords[] = { 1, 0, 1, 1, 0, 0, 0, 1 };
  const GLfloat proj[] = { 0, -2.f / w, 0, 0, -2.f / h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_ID);

  // Update attribute values.
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);

  glVertexPointer(2, GL_FLOAT, 0, bgTextureVertices);
  glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);

  glColor4f(1, 1, 1, 1);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisable(GL_TEXTURE_2D);
  glDeleteTextures(1,&texture_ID);
}
void XOpenGLModule::XOpenGLModule::Render3Dmodul() {
  //draw 3D model
  if (draw_is_ready) {
    draw_is_ready = 0;
    GLfloat model_matrix[16];
    std::vector<double> temp=projection_vector_mat[0];
    for (int index = 0; index < 16; index++) {
      model_matrix[index] = temp[index];
    }
    projection_vector_mat.clear();

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projectionMatrix);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glLoadMatrixf(model_matrix);

//    glTranslatef(0, 0, -WINDOW_SIZE);						//将物体向外移动WINDOW_SIZE个单位

//    glBegin(GL_QUADS);//绘制图形接口，与glEnd()对应
//    glNormal3f(0.0, 0.0, 1.0);
//    glTexCoord2f(0.0, 0.0); glVertex3f(-WINDOW_SIZE, WINDOW_SIZE, WINDOW_SIZE);//
//    glTexCoord2f(1.0, 0.0); glVertex3f(WINDOW_SIZE, -WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(1.0, 1.0); glVertex3f(WINDOW_SIZE, WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(0.0, 1.0); glVertex3f(-WINDOW_SIZE, WINDOW_SIZE, WINDOW_SIZE);
//
//    glNormal3f(0.0, 0.0, -1.0);
//    glTexCoord2f(1.0, 0.0); glVertex3f(-WINDOW_SIZE, -WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(1.0, 1.0); glVertex3f(-WINDOW_SIZE, WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(0.0, 1.0); glVertex3f(WINDOW_SIZE, WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(0.0, 0.0); glVertex3f(WINDOW_SIZE, -WINDOW_SIZE, -WINDOW_SIZE);
//
//    glNormal3f(0.0, 1.0, 0.0);
//    glTexCoord2f(0.0, 1.0); glVertex3f(-WINDOW_SIZE, WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(0.0, 0.0); glVertex3f(-WINDOW_SIZE, WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(1.0, 0.0); glVertex3f(WINDOW_SIZE, WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(1.0, 1.0); glVertex3f(WINDOW_SIZE, WINDOW_SIZE, -WINDOW_SIZE);
//
//    glNormal3f(0.0, -1.0, 0.0);
//    glTexCoord2f(1.0, 1.0); glVertex3f(-WINDOW_SIZE, -WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(0.0, 1.0); glVertex3f(WINDOW_SIZE, -WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(0.0, 0.0); glVertex3f(WINDOW_SIZE, -WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(1.0, 0.0); glVertex3f(-WINDOW_SIZE, -WINDOW_SIZE, WINDOW_SIZE);
//
//    glNormal3f(1.0, 0.0, 0.0);
//    glTexCoord2f(1.0, 0.0); glVertex3f(WINDOW_SIZE, -WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(1.0, 1.0); glVertex3f(WINDOW_SIZE, WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(0.0, 1.0); glVertex3f(WINDOW_SIZE, WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(0.0, 0.0); glVertex3f(WINDOW_SIZE, -WINDOW_SIZE, WINDOW_SIZE);
//
//    glNormal3f(-1.0, 0.0, 0.0);
//    glTexCoord2f(0.0, 0.0); glVertex3f(-WINDOW_SIZE, -WINDOW_SIZE, -WINDOW_SIZE);
//    glTexCoord2f(1.0, 0.0); glVertex3f(-WINDOW_SIZE, -WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(1.0, 1.0); glVertex3f(-WINDOW_SIZE, WINDOW_SIZE, WINDOW_SIZE);
//    glTexCoord2f(0.0, 1.0); glVertex3f(-WINDOW_SIZE, WINDOW_SIZE, -WINDOW_SIZE);
    static float lineX[] = { 0, 0, 0, 1, 0, 0 };
    static float lineY[] = { 0, 0, 0, 0, 1, 0 };
    static float lineZ[] = { 0, 0, 0, 0, 0, 1 };

    glLineWidth(4);

    glBegin(GL_LINES);

    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3fv(lineX);
    glVertex3fv(lineX + 3);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3fv(lineY);
    glVertex3fv(lineY + 3);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3fv(lineZ);
    glVertex3fv(lineZ + 3);

    glEnd();
  }
}
}