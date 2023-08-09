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

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float rotation_angle)
{
    float rad = float(rotation_angle * MY_PI) / 180.0f;
    float cosa = cos(rad);
    float sina = sin(rad);

    axis.normalize();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f nhat = Eigen::Matrix3f::Identity();
    nhat << 
        0, -axis.z(), axis.y(),
        axis.z(), 0, -axis.x(),
        -axis.y(), axis.x(), 0;

    // rodrigues rotation formula
    Eigen::Matrix3f m = cosa * I + (1 - cosa) * (axis * axis.transpose()) + sina * nhat;
    
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Zero();
    rotation.block<3, 3>(0, 0) = m;
    rotation.row(3) << 0, 0, 0, 1;

    return rotation;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // float rad = float(rotation_angle * MY_PI) / 180.0f;
    // Eigen::Matrix4f rotation;
    // rotation << 
    //     cos(rad), -sin(rad), 0, 0, 
    //     sin(rad), cos(rad),  0, 0, 
    //     0,         0,        1, 0, 
    //     0,         0,        0, 1;

    Eigen::Vector3f axis{1, 0, 0};
    Eigen::Matrix4f rotation = get_rotation(axis, rotation_angle);

    model = rotation * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f ortho_translate = Eigen::Matrix4f::Identity(); // ortho translate to origin
    Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity(); // ortho scale to [-1, 1]
    Eigen::Matrix4f ortho_normal = Eigen::Matrix4f::Identity(); // ortho normal
    Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Identity();

    float half_fov_radian = eye_fov / 2 * float(MY_PI) / 180.0f;
    float top = -zNear * tan(half_fov_radian);
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;

    ortho_translate << 
        1, 0, 0, -(left + right) / 2,
        0, 1, 0, -(top + bottom) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    ortho_scale << 
        2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    ortho_normal = ortho_scale * ortho_translate;

    persp_to_ortho <<
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;
    
    projection = ortho_normal * persp_to_ortho;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
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

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
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

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
