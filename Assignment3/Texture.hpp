//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {

        u = std::fmin(1, std::fmax(u, 0));
        v = std::fmin(1, std::fmax(v, 0));

        auto u_img = u * width, v_img = (1 - v) * width;
        float u0 = std::fmax(1, floor(u_img - 0.5)), u1 = floor(u_img + 0.5);
        float v0 = std::fmax(1, floor(v_img - 0.5)), v1 = floor(v_img + 0.5);
        float s = (u_img - u0) / (u1 - u0), t = (v_img - v0) / (v1 - v0);
        
        auto c00 = image_data.at<cv::Vec3b>(v0, u0);
        auto c10 = image_data.at<cv::Vec3b>(v0, u1);
        auto c01 = image_data.at<cv::Vec3b>(v1, u0);
        auto c11 = image_data.at<cv::Vec3b>(v1, u1);
        auto c0 = c00 + s * (c10 - c00);
        auto c1 = c01 + s * (c11 - c01);
        auto color = c0 + t * (c1 - c0);

        return Eigen::Vector3f(color[0], color[1], color[2]);

    }


};
#endif //RASTERIZER_TEXTURE_H
