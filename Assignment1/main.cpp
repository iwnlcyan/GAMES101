#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(Vector3f axis,float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float c0s = cos(rotation_angle);
    float s1n = sin(rotation_angle);
    float m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15,m16;
    m1  = axis[0]*axis[0]*(1 - c0s) + c0s;
    m2  = axis[0]*axis[1]*(1 - c0s) + axis[2]*s1n;
    m3  = axis[0]*axis[2]*(1 - c0s) - axis[1]*s1n;
    m4  = 0;
    m5  = axis[0]*axis[1]*(1 - c0s) - axis[2]*s1n;
    m6  = axis[1]*axis[1]*(1 - c0s) + c0s;
    m7  = axis[1]*axis[2]*(1 - c0s) + axis[0]*s1n;
    m8  = 0;
    m9  = axis[0]*axis[2]*(1 - c0s) + axis[1]*s1n;
    m10 = axis[1]*axis[2]*(1 - c0s) - axis[0]*s1n;
    m11 = axis[2]*axis[2]*(1 - c0s) + c0s;
    m12 = 0;
    m13 = 0;
    m14 = 0;
    m15 = 0;
    m16 = 1;
    Eigen::Matrix4f rotation;
    rotation << m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15,m16;
    
    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f persp;
    persp << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear*zFar, 
        0, 0, 1, 0;
    
    float halfE = eye_fov/2.0/180.0*MY_PI;
    float t = zNear*std::tan(halfE);//top
    float r=t*aspect_ratio;//right 
    float l=(-1)*r;//left 
    float b=(-1)*t;//bottom

    Eigen::Matrix4f scal=Eigen::Matrix4f::Identity();
    scal<<2/(r-l),0,0,0,
        0,2/(t-b),0,0,
        0,0,2/(zNear-zFar),0,
        0,0,0,1;
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans<<1,0,0,(-1)*(r+l)/2,
        0,1,0,(-1)*(t+b)/2,
        0,0,1,(-1)*(zNear+zFar)/2,
        0,0,0,1;
    Eigen::Matrix4f ortho = scal * trans;

    projection = ortho * persp * projection;

    return projection;
}

Eigen::Matrix4f get_scale_matrix(int factor)
{
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f zoom;
    zoom << factor, 0, 0, 0, 0, factor, 0, 0, 0, 0, factor, 0, 
        0, 0, 0, 1;
    
    scale = zoom * scale;

    return scale;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    Eigen::Vector3f Axis = {0,0,1};
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};
    int factor = 1;

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_scale(get_scale_matrix(factor));
        r.set_model(get_model_matrix(Axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_scale(get_scale_matrix(factor));
        r.set_model(get_model_matrix(Axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle -= 10;
        }
        else if (key == 'd') {
            angle += 10;
        }
        else if (key == 'z') {
            factor -= 2;
        }
        else if (key == 'x') {
            factor += 2;
        }
    }

    return 0;
}
