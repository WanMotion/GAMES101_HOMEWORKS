#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
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

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = rotation_angle / 180.0 * MY_PI;
    Eigen::Matrix4f rotation_matrix;
    rotation_matrix << 
        cos(angle), -sin(angle), 0, 0,
        sin(angle), cos(angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return rotation_matrix;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // eye_fov为视角，aspect_ratio为长宽比，zNear为近处z的坐标，zFar为远处z的坐标
    float A = zNear + zFar;
    float B = -zNear * zFar;
    Eigen::Matrix4f persp;
    persp << 
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, A, B,
        0, 0, 1.0, 0;
    projection = projection * persp;
    // 计算近处平面的高
    float h = zNear * tan(eye_fov / 2 / 180.0 * MY_PI) * 2;
    // 宽
    float w = h / aspect_ratio;
    // 缩放为正方形
    Eigen::Matrix4f scale;
    scale <<
        2.0 / w, 0, 0, 0,
        0, 2.0 / h, 0, 0,
        0, 0, 2.0 / (zFar - zNear), 0,
        0, 0, 0, 1;
    Eigen::Matrix4f move;
    move <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -0.5*(zFar+zNear),
        0, 0, 0, 1;
    return scale * move * projection;

}


// 得到绕任意过原点的轴的旋转变换矩阵。
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    Vector3f axis_iden = axis/axis.norm();// 求出单位向量
    // 首先将轴绕x轴旋转到xoz平面
    float a = axis_iden[0];
    float b = axis_iden[1];
    float c = axis_iden[2];
    float d = sqrt(b * b + c * c);
    Eigen::Matrix4f rotateToXOZ;
    rotateToXOZ <<
        1.0, 0, 0, 0,
        0, c / d, -b / d, 0,
        0, b / d, c / d, 0,
        0, 0, 0, 1.0;
    // 绕着y轴旋转至Z轴
    Eigen::Matrix4f rotateToZ;
    rotateToZ <<
        d, 0.0, -a, 0,
        0, 1, 0, 0,
        a, 0, d, 0,
        0, 0, 0, 1;
    // 最后绕着z轴旋转cita
    float cita = angle / 180.0 * MY_PI;
    Eigen::Matrix4f rotateFinal;
    rotateFinal <<
        cos(cita), -sin(cita), 0, 0,
        sin(cita), cos(cita), 0, 0,
        0, 0, 1.0, 0,
        0, 0, 0, 1;
    return rotateToXOZ.inverse() * rotateToZ.inverse() * rotateFinal * rotateToZ * rotateToXOZ;
        
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
        else
            return 0;
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
