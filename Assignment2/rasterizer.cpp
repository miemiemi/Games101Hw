// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


#define SSAA_Freq 4       // SSAAx2

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f a = _v[0];
    Eigen::Vector3f b = _v[1];
    Eigen::Vector3f c = _v[2];
    Eigen::Vector3f p(x, y, 0);

    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f ap = p - a;
    Eigen::Vector3f bc = c - b;
    Eigen::Vector3f bp = p - b;
    Eigen::Vector3f ca = a - c;
    Eigen::Vector3f cp = p - c;

    if(ab.cross(ap).z() > 0 && bc.cross(bp).z() > 0 && ca.cross(cp).z() > 0)
        return true;

    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }

    down_sampling();
}

//降低分辨率
void rst::rasterizer::down_sampling(){

    int sampling_frequency, sampling_times;
	sampling_frequency = SSAA_Freq;
	sampling_times = sampling_frequency * sampling_frequency;
	float sampling_period = 1.0f / sampling_frequency;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {

			Eigen::Vector3f color = { 0,0,0 };
			Eigen::Vector3f point = { j * 1.0f,i * 1.0f,0 };

			for (int m = 0; m < sampling_frequency; m++) {
				for (int n = 0; n < sampling_frequency; n++) {
					int depth_buf_x, depth_buf_y;
					depth_buf_x = j * SSAA_Freq + n;
					depth_buf_y = i * SSAA_Freq + m;
					color += SSAA_frame_buf[get_SSAA_index(depth_buf_x, depth_buf_y)];
				}
			}
			color /= sampling_times;
			set_pixel(point, color);
		}
	}
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {

    auto v = t.toVector4();
    float supersample_step = 1.0f / SSAA_Freq;

    // TODO : Find out the bounding box of current triangle.
    float Max_x = 0;
    float Max_y = 0;
    float Min_x = width;
    float Min_y = height;
    for (int i = 0; i < 3; i++)
    {
        Max_x = std::max(Max_x, t.v[i].x());
        Max_y = std::max(Max_y, t.v[i].y());
        Min_x = std::min(Min_x, t.v[i].x());
        Min_y = std::min(Min_y, t.v[i].y());
    }
    int Max_x_int = std::ceil(Max_x);
    int Max_y_int = std::ceil(Max_y);
    int Min_x_int = std::floor(Min_x);
    int Min_y_int = std::floor(Min_y);

    // for(auto & vector : v)
    //     std::cout << " x: "<< vector.x() << " y: "<< vector.y() << " z: "<< vector.z() << std::endl;
    // std::cout << std::endl;
    // std::cout << "Max_x = " << Max_x_int << " Max_y = " << Max_y_int << std::endl;
    // std::cout << "Min_x = " << Min_x_int << " Min_y = " << Min_y_int << std::endl;
    // std::cout << std::endl;


    Vector3f color = t.getColor();
    
    for (int i = Min_x_int; i < Max_x_int; i++)
    {
        for (int j = Min_y_int; j < Max_y_int; j++)
        {
            float blend_rate;
            int sample_counter = 0;
            for (int m = 0; m < SSAA_Freq; m++)
                for (int n = 0; n < SSAA_Freq; n++){
                    float x = i + (m + 0.5f) * supersample_step;
                    float y = j + (n + 0.5f) * supersample_step;
                    
                    if(insideTriangle(x, y, t.v)){
                        auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        int x_index = i * SSAA_Freq + m;
                        int y_index = j * SSAA_Freq + n;
                        int buf_index = get_SSAA_index(x_index, y_index);
                        if(z_interpolated < SSAA_depth_buf[buf_index]){
                            SSAA_depth_buf[buf_index] = z_interpolated;
                            Vector3f SSAA_point = {x_index * 1.0f, y_index * 1.0f, 0.0f};
                            set_SSAA_pixel(SSAA_point, color);
                        }
                    }
                }
            
        }
        
    }

    
    
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(SSAA_frame_buf.begin(), SSAA_frame_buf.end(), Eigen::Vector3f{0, 0, 0});

    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(SSAA_depth_buf.begin(), SSAA_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    //depth_buf.resize(w * h);
    SSAA_frame_buf.resize(w * h * SSAA_Freq * SSAA_Freq);
    SSAA_depth_buf.resize(w * h * SSAA_Freq * SSAA_Freq);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_SSAA_index(int x, int y)
{
    return (height * SSAA_Freq - 1 - y)* width * SSAA_Freq + x;
}


void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::set_SSAA_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    auto ind = (height * SSAA_Freq - 1 - point.y()) * width * SSAA_Freq + point.x();
    SSAA_frame_buf[ind] = color;
}

// clang-format on