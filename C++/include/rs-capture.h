#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <map>
#include <functional>

#include "stb_easy_font.h"
#include "example-utils.hpp"
#include "imgui_impl_glfw.h"

#ifndef PI
#define PI  3.14159265358979323846
#define PI_FL  3.141592f
#endif
const float IMU_FRAME_WIDTH = 1280.f;
const float IMU_FRAME_HEIGHT = 720.f;
enum class Priority { high = 0, medium = -1, low = -2 };

struct float3 {
    float x, y, z;
    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};

struct float2 { float x, y; };

struct rect
{
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const
    {
        auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
        if (W > w)
        {
            auto scale = w / W;
            W *= scale;
            H *= scale;
        }

        return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
    }
};

inline void draw_text(int x, int y, const char* text)
{
    std::vector<char> buffer;
    buffer.resize(60000); // ~300 chars
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, &(buffer[0]) );
    glDrawArrays( GL_QUADS,
                  0,
                  4
                      * stb_easy_font_print( (float)x,
                                             (float)( y - 7 ),
                                             (char *)text,
                                             nullptr,
                                             &( buffer[0] ),
                                             int( sizeof( char ) * buffer.size() ) ) );
    glDisableClientState(GL_VERTEX_ARRAY);
}

void set_viewport(const rect& r)
{
    glViewport((int)r.x, (int)r.y, (int)r.w, (int)r.h);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, r.w, r.h, 0, -1, +1);
}

class imu_renderer
{
public:
    void render(const rs2::motion_frame& frame, const rect& r)
    {
        draw_motion(frame, r.adjust_ratio({ IMU_FRAME_WIDTH, IMU_FRAME_HEIGHT }));
    }

    GLuint get_gl_handle() { return _gl_handle; }

private:
    GLuint _gl_handle = 0;

    void draw_motion(const rs2::motion_frame& f, const rect& r)
    {
        if (!_gl_handle)
            glGenTextures(1, &_gl_handle);

        set_viewport(r);
        draw_text(int(0.05f * r.w), int(0.05f * r.h), f.get_profile().stream_name().c_str());

        auto md = f.get_motion_data();
        auto x = md.x;
        auto y = md.y;
        auto z = md.z;

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();

        glOrtho(-2.8, 2.8, -2.4, 2.4, -7, 7);

        glRotatef(25, 1.0f, 0.0f, 0.0f);

        glTranslatef(0, -0.33f, -1.f);

        glRotatef(-135, 0.0f, 1.0f, 0.0f);

        glRotatef(180, 0.0f, 0.0f, 1.0f);
        glRotatef(-90, 0.0f, 1.0f, 0.0f);

        draw_axes(1, 2);

        draw_circle(1, 0, 0, 0, 1, 0);
        draw_circle(0, 1, 0, 0, 0, 1);
        draw_circle(1, 0, 0, 0, 0, 1);

        const auto canvas_size = 230;
        const auto vec_threshold = 0.01f;
        float norm = std::sqrt(x * x + y * y + z * z);
        if (norm < vec_threshold)
        {
            const auto radius = 0.05;
            static const int circle_points = 100;
            static const float angle = 2.0f * 3.1416f / circle_points;

            glColor3f(1.0f, 1.0f, 1.0f);
            glBegin(GL_POLYGON);
            double angle1 = 0.0;
            glVertex2d(radius * cos(0.0), radius * sin(0.0));
            int i;
            for (i = 0; i < circle_points; i++)
            {
                glVertex2d(radius * cos(angle1), radius * sin(angle1));
                angle1 += angle;
            }
            glEnd();
        }
        else
        {
            auto vectorWidth = 3.f;
            glLineWidth(vectorWidth);
            glBegin(GL_LINES);
            glColor3f(1.0f, 1.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(x / norm, y / norm, z / norm);
            glEnd();

            // Save model and projection matrix for later
            GLfloat model[16];
            glGetFloatv(GL_MODELVIEW_MATRIX, model);
            GLfloat proj[16];
            glGetFloatv(GL_PROJECTION_MATRIX, proj);

            glLoadIdentity();
            glOrtho(-canvas_size, canvas_size, -canvas_size, canvas_size, -1, +1);

            std::ostringstream s1;
            const auto precision = 3;

            glRotatef(180, 1.0f, 0.0f, 0.0f);

            s1 << "(" << std::fixed << std::setprecision(precision) << x << "," << std::fixed << std::setprecision(precision) << y << "," << std::fixed << std::setprecision(precision) << z << ")";
            print_text_in_3d(x, y, z, s1.str().c_str(), false, model, proj, 1 / norm);

            std::ostringstream s2;
            s2 << std::setprecision(precision) << norm;
            print_text_in_3d(x / 2, y / 2, z / 2, s2.str().c_str(), true, model, proj, 1 / norm);
        }
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
    }

    //IMU drawing helper functions
    void multiply_vector_by_matrix(GLfloat vec[], GLfloat mat[], GLfloat* result)
    {
        const auto N = 4;
        for (int i = 0; i < N; i++)
        {
            result[i] = 0;
            for (int j = 0; j < N; j++)
            {
                result[i] += vec[j] * mat[N * j + i];
            }
        }
        return;
    }

    float2 xyz_to_xy(float x, float y, float z, GLfloat model[], GLfloat proj[], float vec_norm)
    {
        GLfloat vec[4] = { x, y, z, 0 };
        float tmp_result[4];
        float result[4];

        const auto canvas_size = 230;

        multiply_vector_by_matrix(vec, model, tmp_result);
        multiply_vector_by_matrix(tmp_result, proj, result);

        return{ canvas_size * vec_norm * result[0], canvas_size * vec_norm * result[1] };
    }

    void print_text_in_3d(float x, float y, float z, const char* text, bool center_text, GLfloat model[], GLfloat proj[], float vec_norm)
    {
        auto xy = xyz_to_xy(x, y, z, model, proj, vec_norm);
        auto w = (center_text) ? stb_easy_font_width((char*)text) : 0;
        glColor3f(1.0f, 1.0f, 1.0f);
        draw_text((int)(xy.x - w / 2), (int)xy.y, text);
    }

    static void  draw_axes(float axis_size = 1.f, float axisWidth = 4.f)
    {
        // Triangles For X axis
        glBegin(GL_TRIANGLES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(axis_size * 1.1f, 0.f, 0.f);
        glVertex3f(axis_size, -axis_size * 0.05f, 0.f);
        glVertex3f(axis_size, axis_size * 0.05f, 0.f);
        glVertex3f(axis_size * 1.1f, 0.f, 0.f);
        glVertex3f(axis_size, 0.f, -axis_size * 0.05f);
        glVertex3f(axis_size, 0.f, axis_size * 0.05f);
        glEnd();

        // Triangles For Y axis
        glBegin(GL_TRIANGLES);
        glColor3f(0.f, 1.f, 0.f);
        glVertex3f(0.f, axis_size * 1.1f, 0.0f);
        glVertex3f(0.f, axis_size, 0.05f * axis_size);
        glVertex3f(0.f, axis_size, -0.05f * axis_size);
        glVertex3f(0.f, axis_size * 1.1f, 0.0f);
        glVertex3f(0.05f * axis_size, axis_size, 0.f);
        glVertex3f(-0.05f * axis_size, axis_size, 0.f);
        glEnd();

        // Triangles For Z axis
        glBegin(GL_TRIANGLES);
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 1.1f * axis_size);
        glVertex3f(0.0f, 0.05f * axis_size, 1.0f * axis_size);
        glVertex3f(0.0f, -0.05f * axis_size, 1.0f * axis_size);
        glVertex3f(0.0f, 0.0f, 1.1f * axis_size);
        glVertex3f(0.05f * axis_size, 0.f, 1.0f * axis_size);
        glVertex3f(-0.05f * axis_size, 0.f, 1.0f * axis_size);
        glEnd();

        glLineWidth(axisWidth);

        // Drawing Axis
        glBegin(GL_LINES);
        // X axis - Red
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(axis_size, 0.0f, 0.0f);

        // Y axis - Green
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, axis_size, 0.0f);

        // Z axis - Blue
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, axis_size);
        glEnd();
    }

    // intensity is grey intensity
    static void draw_circle(float xx, float xy, float xz, float yx, float yy, float yz, float radius = 1.1, float3 center = { 0.0, 0.0, 0.0 }, float intensity = 0.5f)
    {
        const auto N = 50;
        glColor3f(intensity, intensity, intensity);
        glLineWidth(2);
        glBegin(GL_LINE_STRIP);

        for (int i = 0; i <= N; i++)
        {
            const double theta = (2 * PI / N) * i;
            const auto cost = static_cast<float>(cos(theta));
            const auto sint = static_cast<float>(sin(theta));
            glVertex3f(
                center.x + radius * (xx * cost + yx * sint),
                center.y + radius * (xy * cost + yy * sint),
                center.z + radius * (xz * cost + yz * sint)
            );
        }
        glEnd();
    }

};

class pose_renderer
{
public:
    void render(const rs2::pose_frame& frame, const rect& r)
    {
        draw_pose(frame, r.adjust_ratio({ IMU_FRAME_WIDTH, IMU_FRAME_HEIGHT }));
    }

    GLuint get_gl_handle() { return _gl_handle; }

private:
    mutable GLuint _gl_handle = 0;

    // Provide textual representation only
    void draw_pose(const rs2::pose_frame& f, const rect& r)
    {
        if (!_gl_handle)
            glGenTextures(1, &_gl_handle);

        set_viewport(r);
        std::string caption(f.get_profile().stream_name());
        if (f.get_profile().stream_index())
            caption += std::to_string(f.get_profile().stream_index());
        draw_text(int(0.05f * r.w), int(0.05f * r.h), caption.c_str());

        auto pose = f.get_pose_data();
        std::stringstream ss;
        ss << "Pos (meter): \t\t" << std::fixed << std::setprecision(2) << pose.translation.x << ", " << pose.translation.y << ", " << pose.translation.z;
        draw_text(int(0.05f * r.w), int(0.2f * r.h), ss.str().c_str());
        ss.clear(); ss.str("");
        ss << "Orient (quaternion): \t" << pose.rotation.x << ", " << pose.rotation.y << ", " << pose.rotation.z << ", " << pose.rotation.w;
        draw_text(int(0.05f * r.w), int(0.3f * r.h), ss.str().c_str());
        ss.clear(); ss.str("");
        ss << "Lin Velocity (m/sec): \t" << pose.velocity.x << ", " << pose.velocity.y << ", " << pose.velocity.z;
        draw_text(int(0.05f * r.w), int(0.4f * r.h), ss.str().c_str());
        ss.clear(); ss.str("");
        ss << "Ang. Velocity (rad/sec): \t" << pose.angular_velocity.x << ", " << pose.angular_velocity.y << ", " << pose.angular_velocity.z;
        draw_text(int(0.05f * r.w), int(0.5f * r.h), ss.str().c_str());
    }
};


////////////////////////
// Image display code //
////////////////////////
/// \brief The texture class

class texture
{
public:

    void upload(const rs2::video_frame& frame)
    {
        if (!frame) return;

        if (!_gl_handle)
            glGenTextures(1, &_gl_handle);
        GLenum err = glGetError();

        auto format = frame.get_profile().format();
        auto width = frame.get_width();
        auto height = frame.get_height();
        _stream_type = frame.get_profile().stream_type();
        _stream_index = frame.get_profile().stream_index();

        glBindTexture(GL_TEXTURE_2D, _gl_handle);

        switch (format)
        {
        case RS2_FORMAT_RGB8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.get_data());
            break;
        case RS2_FORMAT_RGBA8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, frame.get_data());
            break;
        case RS2_FORMAT_Y8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, frame.get_data());
            break;
        case RS2_FORMAT_Y10BPACK:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_SHORT, frame.get_data());
            break;
        default:
            throw std::runtime_error("The requested format is not supported by this demo!");
        }

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void show(const rect& r, float alpha = 1.f) const
    {
        if (!_gl_handle)
            return;

        set_viewport(r);

        glBindTexture(GL_TEXTURE_2D, _gl_handle);
        glColor4f(1.0f, 1.0f, 1.0f, alpha);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2f(0, 0);
        glTexCoord2f(0, 1); glVertex2f(0, r.h);
        glTexCoord2f(1, 1); glVertex2f(r.w, r.h);
        glTexCoord2f(1, 0); glVertex2f(r.w, 0);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
        draw_text(int(0.05f * r.w), int(0.05f * r.h), rs2_stream_to_string(_stream_type));
    }

    GLuint get_gl_handle() { return _gl_handle; }

    void render(const rs2::frame& frame, const rect& rect, float alpha = 1.f)
    {
        if (auto vf = frame.as<rs2::video_frame>())
        {
            upload(vf);
            show(rect.adjust_ratio({ (float)vf.get_width(), (float)vf.get_height() }), alpha);
        }
        else if (auto mf = frame.as<rs2::motion_frame>())
        {
            _imu_render.render(frame, rect.adjust_ratio({ IMU_FRAME_WIDTH, IMU_FRAME_HEIGHT }));
        }
        else if (auto pf = frame.as<rs2::pose_frame>())
        {
            _pose_render.render(frame, rect.adjust_ratio({ IMU_FRAME_WIDTH, IMU_FRAME_HEIGHT }));
        }
        else
            throw std::runtime_error("Rendering is currently supported for video, motion and pose frames only");
    }

private:
    GLuint          _gl_handle = 0;
    rs2_stream      _stream_type = RS2_STREAM_ANY;
    int             _stream_index{};
    imu_renderer    _imu_render;
    pose_renderer   _pose_render;
};

struct tile_properties
{
   
    unsigned int x, y; //location of tile in the grid
    unsigned int w, h; //width and height by number of tiles
    Priority priority; //when should the tile be drawn?: high priority is on top of all, medium is a layer under top layer, low is a layer under medium layer

};

//name aliasing the map of pairs<frame, tile_properties>
using frame_and_tile_property = std::pair<rs2::frame, tile_properties>;
using frames_mosaic = std::map<int, frame_and_tile_property>;

class window
{
public:
    std::function<void(bool)>           on_left_mouse = [](bool) {};
    std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
    std::function<void(double, double)> on_mouse_move = [](double, double) {};
    std::function<void(int)>            on_key_release = [](int) {};

    window(int width, int height, const char* title)
        : _width(width), _height(height), _canvas_left_top_x(0), _canvas_left_top_y(0), _canvas_width(width), _canvas_height(height)
    {
        glfwInit();
        win = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (!win)
            throw std::runtime_error("Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools");
        glfwMakeContextCurrent(win);

        glfwSetWindowUserPointer(win, this);
        // glfwSetMouseButtonCallback(win, [](GLFWwindow* w, int button, int action, int mods)
        //     {
        //         if (ImGui::GetCurrentContext() != nullptr)
        //         {
        //             ImGui_ImplGlfw_MouseButtonCallback(w, button, action, mods);// Forward the event to ImGui's GLFW implementation
        //         }
        //         auto s = (window*)glfwGetWindowUserPointer(w);
        //         if (button == 0) s->on_left_mouse(action == GLFW_PRESS);
        //     });

        // glfwSetScrollCallback(win, [](GLFWwindow* w, double xoffset, double yoffset)
        //     {
        //         if (ImGui::GetCurrentContext() != nullptr)
        //         {
        //             ImGui_ImplGlfw_ScrollCallback(w, xoffset, yoffset); // Forwards scroll events to ImGui
        //         }
        //         auto s = (window*)glfwGetWindowUserPointer(w);
        //         s->on_mouse_scroll(xoffset, yoffset);
        //     });

        // glfwSetCursorPosCallback(win, [](GLFWwindow* w, double x, double y)
        //     {
        //         if (ImGui::GetCurrentContext() != nullptr)
        //         {
        //             ImGui_ImplGlfw_CursorPosCallback(w, x, y); // Forward the cursor position to ImGui
        //         }
        //         auto s = (window*)glfwGetWindowUserPointer(w);
        //         s->on_mouse_move(x, y);
        //     });

        // glfwSetKeyCallback(win, [](GLFWwindow* w, int key, int scancode, int action, int mods)
        //     {
        //         if (ImGui::GetCurrentContext() != nullptr)
        //         {
        //             ImGui_ImplGlfw_KeyCallback(w, key, scancode, action, mods);
        //         }
        //         auto s = (window*)glfwGetWindowUserPointer(w);
        //         if (0 == action) // on key release
        //         {
        //             s->on_key_release(key);
        //         }
        //     });
    }

    void show(rs2::frame frame)
    {
        show(frame, { 0, 0, (float)_width, (float)_height });
    }

    operator bool()
    {
        glPopMatrix();
        glfwSwapBuffers(win);

        auto res = !glfwWindowShouldClose(win);

        glfwPollEvents();
        glfwGetFramebufferSize(win, &_width, &_height);

        // Clear the framebuffer
        glClear(GL_COLOR_BUFFER_BIT);
        glViewport(0, 0, _width, _height);

        // Draw the images
        glPushMatrix();
        glfwGetWindowSize(win, &_width, &_height);
        glOrtho(0, _width, _height, 0, -1, +1);

        return res;
    }

    void show(const frames_mosaic& frames)
    {
        // Render openGl mosaic of frames
        if (frames.size())
        {
            // create vector of frames from map, and sort it by priority
            std::vector <frame_and_tile_property> vector_frames;
            //copy: map (values) -> vector
            for (const auto& frame : frames) { vector_frames.push_back(frame.second); }
            //sort in ascending order of the priority
            std::sort(vector_frames.begin(), vector_frames.end(),
                [](const frame_and_tile_property& frame1, const frame_and_tile_property& frame2)
                {
                    return frame1.second.priority < frame2.second.priority;
                });
            //create margin to the shown frame on tile
            float frame_width_size_from_tile_width = 1.0f;
            //iterate over frames in ascending priority order (so that lower priority frame is drawn first, and can be over-written by higher priority frame )
            for (const auto& frame : vector_frames)
            {
                tile_properties attr = frame.second;
                rect viewport_loc{ _tile_width_pixels * attr.x + _canvas_left_top_x, _tile_height_pixels * attr.y + _canvas_left_top_y,
                    _tile_width_pixels * attr.w * frame_width_size_from_tile_width, _tile_height_pixels * attr.h };
                show(frame.first, viewport_loc);
            }
        }
        else
        {
            // _main_win.put_text("Connect one or more Intel RealSense devices and rerun the example",
            //     0.3f, 0.5f, { float(_canvas_left_top_x), float(_canvas_left_top_y), float(_canvas_width) , float(_canvas_height) });
        }
    }

    void show(const rs2::frame& frame, const rect& rect)
    {
        if (auto fs = frame.as<rs2::frameset>()){
            //  std::cout << "Check frameset" << std::endl;
             render_frameset(fs, rect);}
        // else if (auto vf = frame.as<rs2::depth_frame>()) {
        //     std::cout << "Check depth_frame" << std::endl;
        //     // render_video_frame(vf.apply_filter(_colorizer), rect);
        // }
        else if (auto vf = frame.as<rs2::video_frame>()){
            // std::cout << "Check video_frame" << std::endl;
            render_video_frame(vf, rect);
        }
        // else if (auto mf = frame.as<rs2::motion_frame>()){
        //     std::cout << "Check motion_frame" << std::endl;
        //     // render_motion_frame(mf, rect);
        // }
        // else if (auto pf = frame.as<rs2::pose_frame>()){
        //     std::cout << "Check pose_frame" << std::endl;
        //     // render_pose_frame(pf, rect);
        // }
    }
    void render_frameset(const rs2::frameset& frames, const rect& r)
    {
        std::vector<rs2::frame> supported_frames;
        for (auto f : frames)
        {
            if (can_render(f))
                supported_frames.push_back(f);
        }
        if (supported_frames.empty())
            return;

        std::sort(supported_frames.begin(), supported_frames.end(), [](rs2::frame first, rs2::frame second)
            { return first.get_profile().stream_type() < second.get_profile().stream_type();  });

        auto image_grid = calc_grid(r, supported_frames);

        int image_index = 0;
        for (auto f : supported_frames)
        {
            auto r = image_grid.at(image_index);
            show(f, r);
            image_index++;
        }
    }
    bool can_render(const rs2::frame& f) const
    {
        auto format = f.get_profile().format();
        switch (format)
        {
        case RS2_FORMAT_RGB8:
        case RS2_FORMAT_RGBA8:
        case RS2_FORMAT_Y8:
        case RS2_FORMAT_MOTION_XYZ32F:
        case RS2_FORMAT_Y10BPACK:
            return true;
        default:
            return false;
        }
    }

    std::vector<rect> calc_grid(rect r, std::vector<rs2::frame>& frames)
    {
        auto grid = calc_grid(r, frames.size());

        std::vector<rect> rv;
        int curr_line = -1;

        for (int i = 0; i < frames.size(); i++)
        {
            auto mod = i % (int)grid.x;
            float fw = IMU_FRAME_WIDTH;
            float fh = IMU_FRAME_HEIGHT;
            if (auto vf = frames[i].as<rs2::video_frame>())
            {
                fw = (float)vf.get_width();
                fh = (float)vf.get_height();
            }
            float cell_x_postion = (float)(mod * grid.w);
            if (mod == 0) curr_line++;
            float cell_y_position = curr_line * grid.h;
            float2 margin = { grid.w * 0.02f, grid.h * 0.02f };
            auto r = rect{ cell_x_postion + margin.x, cell_y_position + margin.y, grid.w - 2 * margin.x, grid.h };
            rv.push_back(r.adjust_ratio(float2{ fw, fh }));
        }

        return rv;
    }

    rect calc_grid(rect r, size_t streams)
    {
        if (r.w <= 0 || r.h <= 0 || streams <= 0)
            throw std::runtime_error("invalid window configuration request, failed to calculate window grid");
        float ratio = r.w / r.h;
        auto x = sqrt(ratio * (float)streams);
        auto y = (float)streams / x;
        auto w = round(x);
        auto h = round(y);
        if (w == 0 || h == 0)
            throw std::runtime_error("invalid window configuration request, failed to calculate window grid");
        while (w * h > streams)
            h > w ? h-- : w--;
        while (w* h < streams)
            h > w ? w++ : h++;
        auto new_w = round(r.w / w);
        auto new_h = round(r.h / h);
        // column count, line count, cell width cell height
        return rect{ static_cast<float>(w), static_cast<float>(h), static_cast<float>(new_w), static_cast<float>(new_h) };
    }
    void render_video_frame(const rs2::video_frame& f, const rect& r)
    {
        auto& t = _textures[f.get_profile().unique_id()];
        t.render(f, r);
    }
    private: 
    GLFWwindow* win;
    std::map<int, texture> _textures;
    int _width, _height;
    float _canvas_left_top_x, _canvas_left_top_y;
    int _canvas_width, _canvas_height;
    unsigned _tiles_in_row, _tiles_in_col;
    float _tile_width_pixels, _tile_height_pixels;
    rs2::colorizer _colorizer;
};

