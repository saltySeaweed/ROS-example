// rs-capture
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "rs-capture.h"  

// rs-depth
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>
#include "rs-depth.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
// rs-save-to-disk
#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "rs-save-to-disk.h"
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_Z16    // rs2_format identifies how binary data is encoded within a frame      //
#define WIDTH           640               // Defines the number of columns for each frame or zero for auto resolve//
#define HEIGHT          0                 // Defines the number of lines for each frame or zero for auto resolve  //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
#define HEIGHT_RATIO    20                // Defines the height ratio between the original frame to the new frame //
#define WIDTH_RATIO     10                // Defines the width ratio between the original frame to the new frame  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// The number of meters represented by a single depth unit
float get_depth_unit_value(const rs2_device* const dev)
{
    rs2_error* e = 0;
    rs2_sensor_list* sensor_list = rs2_query_sensors(dev, &e);
    check_error(e);

    int num_of_sensors = rs2_get_sensors_count(sensor_list, &e);
    check_error(e);

    float depth_scale = 0.001f; // Default to mm
    int is_depth_sensor_found = 0;
    int i;
    for (i = 0; i < num_of_sensors; ++i)
    {
        rs2_sensor* sensor = rs2_create_sensor(sensor_list, i, &e);
        check_error(e);

        // Check if the given sensor can be extended to depth sensor interface
        is_depth_sensor_found = rs2_is_sensor_extendable_to(sensor, RS2_EXTENSION_DEPTH_SENSOR, &e);
        check_error(e);

        if (1 == is_depth_sensor_found)
        {
            if( rs2_supports_option((const rs2_options*)sensor, RS2_OPTION_DEPTH_UNITS, &e) )
                depth_scale = rs2_get_option((const rs2_options*)sensor, RS2_OPTION_DEPTH_UNITS, &e);
            check_error(e);
            rs2_delete_sensor(sensor);
            break;
        }
        rs2_delete_sensor(sensor);
    }
    rs2_delete_sensor_list(sensor_list);

    if (0 == is_depth_sensor_found)
    {
        printf("Depth sensor not found!\n");
        exit(EXIT_FAILURE);
    }

    return depth_scale;
}


int rs_capture() try {    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start();

    while (app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
                             apply_filter(printer).     // Print each enabled stream frame rate
                             apply_filter(color_map);   // Find and colorize the depth data

        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        app.show(data);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

int rs_depth() {
    rs2_error* e = 0;
    // Create a context object. This object owns the handles to all connected realsense devices.
    // The returned object should be released with rs2_delete_context(...)
    rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
    check_error(e);

    /* Get a list of all the connected devices. */
    // The returned object should be released with rs2_delete_device_list(...)
    rs2_device_list* device_list = rs2_query_devices(ctx, &e);
    check_error(e);

    int dev_count = rs2_get_device_count(device_list, &e);
    check_error(e);
    printf("There are %d connected RealSense devices.\n", dev_count);
    if (0 == dev_count)
        return EXIT_FAILURE;

    // Get the first connected device
    // The returned object should be released with rs2_delete_device(...)
    rs2_device* dev = rs2_create_device(device_list, 0, &e);
    check_error(e);

    print_device_info(dev);

    /* Determine depth value corresponding to one meter */
    uint16_t one_meter = (uint16_t)(1.0f / get_depth_unit_value(dev));

    // Create a pipeline to configure, start and stop camera streaming
    // The returned object should be released with rs2_delete_pipeline(...)
    rs2_pipeline* pipeline =  rs2_create_pipeline(ctx, &e);
    check_error(e);

    // Create a config instance, used to specify hardware configuration
    // The retunred object should be released with rs2_delete_config(...)
    rs2_config* config = rs2_create_config(&e);
    check_error(e);

    // Request a specific configuration
    rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
    check_error(e);

    // Start the pipeline streaming
    // The returned object should be released with rs2_delete_pipeline_profile(...)
    rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
    if (e)
    {
        printf("The connected device doesn't support depth streaming!\n");
        exit(EXIT_FAILURE);
    }

    rs2_stream_profile_list* stream_profile_list = rs2_pipeline_profile_get_streams(pipeline_profile, &e);
    if (e)
    {
        printf("Failed to create stream profile list!\n");
        exit(EXIT_FAILURE);
    }

    rs2_stream_profile* stream_profile = (rs2_stream_profile*)rs2_get_stream_profile(stream_profile_list, 0, &e);
    if (e)
    {
        printf("Failed to create stream profile!\n");
        exit(EXIT_FAILURE);
    }

    rs2_stream stream; rs2_format format; int index; int unique_id; int framerate;
    rs2_get_stream_profile_data(stream_profile, &stream, &format, &index, &unique_id, &framerate, &e);
    if (e)
    {
        printf("Failed to get stream profile data!\n");
        exit(EXIT_FAILURE);
    }

    int width; int height;
    rs2_get_video_stream_resolution(stream_profile, &width, &height, &e);
    if (e)
    {
        printf("Failed to get video stream resolution data!\n");
        exit(EXIT_FAILURE);
    }
    int rows = height / HEIGHT_RATIO;
    int row_length = width / WIDTH_RATIO;
    int display_size = (rows + 1) * (row_length + 1);
    int buffer_size = display_size * sizeof(char);

    char* buffer = (char*) calloc(display_size, sizeof(char));
    char* out = NULL;

    while (1)  // Until user presses Ctrl+C
    {   
        // This call waits until a new composite_frame is available
        // composite_frame holds a set of frames. It is used to prevent frame drops
        // The returned object should be released with rs2_release_frame(...)
        rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, &e);
        check_error(e);

        // Returns the number of frames embedded within the composite frame
        int num_of_frames = rs2_embedded_frames_count(frames, &e);
        check_error(e);

        int i;
        for (i = 0; i < num_of_frames; ++i)
        {
            // The retunred object should be released with rs2_release_frame(...)
            rs2_frame* frame = rs2_extract_frame(frames, i, &e);
            check_error(e);

            // Check if the given frame can be extended to depth frame interface
            // Accept only depth frames and skip other frames
            if (0 == rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &e))
            {
                rs2_release_frame(frame);
                continue;
            }

            /* Retrieve depth data, configured as 16-bit depth values */
            const uint16_t* depth_frame_data = (const uint16_t*)(rs2_get_frame_data(frame, &e));
            check_error(e);

            /* Print a simple text-based representation of the image, by breaking it into 10x5 pixel regions and approximating the coverage of pixels within one meter */
            out = buffer;
            int x, y, i;
            int* coverage = (int*) calloc(row_length, sizeof(int));

            for (y = 0; y < height; ++y)
            {
                for (x = 0; x < width; ++x)
                {
                    // Create a depth histogram to each row
                    int coverage_index = x / WIDTH_RATIO;
                    int depth = *depth_frame_data++;
                    if (depth > 0 && depth < one_meter)
                        ++coverage[coverage_index];
                }

                if ((y % HEIGHT_RATIO) == (HEIGHT_RATIO-1))
                {
                    for (i = 0; i < (row_length); ++i)
                    {
                        static const char* pixels = " .:nhBXWW";
                        int pixel_index = (coverage[i] / (HEIGHT_RATIO * WIDTH_RATIO / sizeof(pixels)));
                        *out++ = pixels[pixel_index];
                        coverage[i] = 0;
                    }
                    *out++ = '\n';
                }
            }
            *out++ = 0;
            printf("\n%s", buffer);

            free(coverage);
            rs2_release_frame(frame);
        }

        rs2_release_frame(frames);
    }
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}

int rs_save_to_disk() try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

    // Wait for the next set of frames from the camera. Now that autoexposure, etc.
    // has settled, we will write these to disk
    for (auto&& frame : pipe.wait_for_frames())
    {
        // We can only save video frames as pngs, so we skip the rest
        if (auto vf = frame.as<rs2::video_frame>())
        {
            auto stream = frame.get_profile().stream_type();
            // Use the colorizer to get an rgb image for the depth stream
            // if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

            // Write images to disk
            std::stringstream png_file;
            png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                           vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            std::cout << "Saved " << png_file.str() << std::endl;

            // Record per-frame metadata for UVC streams
            std::stringstream csv_file;
            csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
                     << "-metadata.csv";
            metadata_to_csv(vf, csv_file.str());
        }
    }

    return EXIT_SUCCESS;
}
catch(const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

int main () {
    // if(rs_capture() == EXIT_FAILURE) {
    //     std::cout << "rs-capture error !!!" << std::endl; 
    // }
    // if(rs_depth() == EXIT_FAILURE) {
    //     std::cout << "rs-depth error !!!" << std::endl; 
    // }
    if(rs_save_to_disk() == EXIT_FAILURE) {
        std::cout << "rs-save-to-disk error !!!" << std::endl; 
    }

    return 0;
}

