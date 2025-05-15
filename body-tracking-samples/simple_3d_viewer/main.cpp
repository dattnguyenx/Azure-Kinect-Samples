// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <array>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <k4arecord/record.h>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
//#include <recorder.h>
using namespace cv;
std::stringstream ss;

void PrintUsage()
{
#ifdef _WIN32
    printf("\nUSAGE: (k4abt_)simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU, CUDA, DIRECTML, TENSORRT](optional) -model MODEL_PATH(optional)\n");
#else
    printf("\nUSAGE: (k4abt_)simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU, CUDA, TENSORRT](optional)\n");
#endif
    printf("  - SensorMode: \n");
    printf("      NFOV_UNBINNED (default) - Narrow Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]\n");
    printf("      WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]\n");
    printf("  - RuntimeMode: \n");
    printf("      CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower\n");
    printf("      CUDA - Use CUDA for processing.\n");
#ifdef _WIN32
    printf("      DIRECTML - Use the DirectML processing mode.\n");
#endif
    printf("      TENSORRT - Use the TensorRT processing mode.\n");
    printf("      OFFLINE - Play a specified file. Does not require Kinect device\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe OFFLINE MyFile.mkv\n");
}

void PrintAppUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

// Global State and Key Process Function
bool s_isRunning = true;
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;
// global variable
std::string output_png_file_name = "C:\\Users\\ke76boqe\\Projects\\Body_tracking\\body-tracking-samples\\FullDepth\\d_";
std::string point_cloud_file_name = "C:\\Users\\ke76boqe\\Projects\\Body_tracking\\body-tracking-samples\\PointCloud\\frame";
char const* output_mkv = "C:\\Users\\ke76boqe\\Projects\\Body_tracking\\body-tracking-samples\\modified_video.mkv";
int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

struct InputSettings
{
    k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
#ifdef _WIN32
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
#else
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
#endif
    bool Offline = false;
    std::string FileName;
    std::string ModelPath;
};

bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings)
{
    for (int i = 1; i < argc; i++)
    {
        std::string inputArg(argv[i]);
        if (inputArg == std::string("NFOV_UNBINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (inputArg == std::string("WFOV_BINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (inputArg == std::string("CPU"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
        }
        else if (inputArg == std::string("TENSORRT"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT;
        }
        else if (inputArg == std::string("CUDA"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
        }
#ifdef _WIN32
        else if (inputArg == std::string("DIRECTML"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
        }
#endif
        else if (inputArg == std::string("OFFLINE"))
        {
            inputSettings.Offline = true;
            if (i < argc - 1) {
                // Take the next argument after OFFLINE as file name
                //std::string filename = "C:\\Users\\ke76boqe\\Projects\\DepthCompression\\test.mkv" ;
                inputSettings.FileName =  argv[i + 1];
                i++;
            }
            else {
                return false;
            }
        }
        else if (inputArg == std::string("-model"))
        {
            if (i < argc - 1)
                inputSettings.ModelPath = argv[++i];
            else
            {
                printf("Error: model path missing\n");
                return false;
            }
        }
        else
        {
            printf("Error: command not understood: %s\n", inputArg.c_str());
            return false;
        }
    }
    return true;
}

k4a_image_t VisualizeResult(k4abt_frame_t bodyFrame, Window3dWrapper& window3d, int depthWidth, int depthHeight, int count) {

    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);

    std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });

    // Read body index map and assign colors
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
    const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
    //int size = pointCloudColors.size();
    //printf("size of point cloud: %d\n", size);
    size_t sizet = k4a_image_get_size(depthImage);
    uint16_t* depth_image_data = (uint16_t*)(void*)k4a_image_get_buffer(depthImage);
    int32_t width = k4a_image_get_width_pixels(depthImage);
    int32_t height = k4a_image_get_height_pixels(depthImage);
    
    uint16_t minDepthValue=UINT_MAX;
    uint16_t maxDepthValue=0;
    for (int i = 0; i < depthWidth * depthHeight; i++)
    {
        uint8_t bodyIndex = bodyIndexMapBuffer[i];
        if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
        {
            uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
            pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
            //if (depth_image_data[i] < minDepthValue)
            //{
            //    minDepthValue = depth_image_data[i];
            //}

            //if (depth_image_data[i] > maxDepthValue)
            //{
            //    maxDepthValue = depth_image_data[i];
            //}
        }
        /* comment out this Else if you want to save the whole depth image/point cloud scene */
        else
        {   // store only points/pixels on the tracked human body
           
            depth_image_data[i] = 0;
        }
        // 
        
    }

    // SAVE PNG IMAGE HERE
    k4a_capture_set_depth_image(originalCapture, depthImage);
    char str[5];
    snprintf(str, 5, "%04d", count);
    std::string s = "";
    s = output_png_file_name + str + ".png";

    cv::Mat depth_mat = cv::Mat(height, width, CV_16UC1, depth_image_data);
    cv::imwrite(s, depth_mat);

    // CONVERT 16 bits to 8bits image for visualization
    //depth_mat.convertTo(depth_mat, CV_8U, 255.0 / 5000.0, 0.0);
    //cv::imshow("depth image", depth_mat);
    //cv::waitKey(0);

    // 
    //std::cout << "Vao loop";
    //k4a_image_t depth_image = NULL;
    //k4a_image_t xy_table = NULL;
    //k4a_image_t point_cloud = NULL;
    //int point_count = 0;
    //k4a_calibration_t calibration;
    //create_xy_table(&calibration, xy_table);

    //k4a_image_t segmentedDepthImage = k4a_image_create_from_buffer(depth_image_data);
    //std::cout << minDepthValue << '\t' << maxDepthValue << '\n';

    k4a_image_release(bodyIndexMap);
    window3d.UpdatePointClouds(depthImage, pointCloudColors);
    return depthImage;
    // Visualize point cloud
    //

    // Visualize the skeleton data
    //window3d.CleanJointsAndBones();
    //uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    //for (uint32_t i = 0; i < numBodies; i++)
    //{
    //    k4abt_body_t body;
    //    VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
    //    body.id = k4abt_frame_get_body_id(bodyFrame, i);

    //    // Assign the correct color based on the body id
    //    Color color = g_bodyColors[body.id % g_bodyColors.size()];
    //    color.a = 0.4f;
    //    Color lowConfidenceColor = color;
    //    lowConfidenceColor.a = 0.1f;

        // Visualize joints
        //for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
        //{
        //    if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
        //    {
        //        const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
        //        const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

        //        window3d.AddJoint(
        //            jointPosition,
        //            jointOrientation,
        //            body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
        //    }
        //}

        // Visualize bones
     /*   for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
        {
            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

            if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
                body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                    body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
            }
        }*/
   /* }

    k4a_capture_release(originalCapture);
    k4a_image_release(depthImage);*/

}

static void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table)
{
    k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}



static void generate_point_cloud(const k4a_image_t depth_image,
    const k4a_image_t xy_table,
    k4a_image_t point_cloud,
    int* point_count)
{
    //std::cout << "Generating point cloud";
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);

    uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
    k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

    *point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            (*point_count)++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
}

static void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count)
{
    //std::cout << "Writing point cloud";
    int width = k4a_image_get_width_pixels(point_cloud);
    int height = k4a_image_get_height_pixels(point_cloud);

    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            continue;
        }

        ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
            << (float)point_cloud_data[i].xyz.z << std::endl;
        //std::cout << "asdfas"; // (float)point_cloud_data[i].xyz.x;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

void PlayFile(InputSettings inputSettings)
{
    // Initialize the 3d window controller
    Window3dWrapper window3d;

    //create the tracker and playback handle
    k4a_calibration_t sensorCalibration;
    k4abt_tracker_t tracker = nullptr;
    k4a_playback_t playbackHandle = nullptr;

    const char* file = inputSettings.FileName.c_str();
    if (k4a_playback_open(file, &playbackHandle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording: %s\n", file);
        return;
    }

    if (k4a_playback_get_calibration(playbackHandle, &sensorCalibration) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to get calibration\n");
        return;
    }

    k4a_capture_t capture = nullptr;
    k4a_stream_result_t playbackResult = K4A_STREAM_RESULT_SUCCEEDED;

    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = inputSettings.processingMode;
    trackerConfig.model_path = inputSettings.ModelPath.c_str();
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!");

    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);
    int frame_count = 0;

    k4a_device_t device = NULL;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG; // or enumeration 1 or 2
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    k4a_record_t record_handle;
    if (K4A_FAILED(k4a_record_create(output_mkv, device, config, &record_handle)))
    {
        printf("Failed to create recording \n");
    }
    k4a_record_add_imu_track(record_handle);
    printf("Created record");

    if (K4A_FAILED(k4a_record_write_header(record_handle)))
    {
        printf("Failed to write header \n");
    }
    
    k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
    k4a_imu_sample_t imu_sample ;
    while (playbackResult == K4A_STREAM_RESULT_SUCCEEDED && s_isRunning)
    {
        playbackResult = k4a_playback_get_next_capture(playbackHandle, &capture);
        k4a_playback_get_next_imu_sample(playbackHandle, &imu_sample);
        if (playbackResult == K4A_STREAM_RESULT_EOF)
        {
            // End of file reached
            break;
        }

        if (playbackResult == K4A_STREAM_RESULT_SUCCEEDED)
        {
         

            frame_count+=1;
            // check to make sure we have a depth image
            k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
            if (depthImage == nullptr) {
                //If no depth image, print a warning and skip to next frame
                std::cout << "Warning: No depth image, skipping frame!" << std::endl;
                k4a_capture_release(capture);
                continue;
            }
            // Release the Depth image
            k4a_image_release(depthImage);

            //enque capture and pop results - synchronous
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
         



            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }

            k4abt_frame_t bodyFrame = nullptr;
            k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, K4A_WAIT_INFINITE);
            if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
            {
                k4a_image_t depth_image = NULL;
                k4a_image_t xy_table = NULL;
                k4a_image_t point_cloud = NULL;
                int point_count = 0;
                //std::string point_cloud_file_name = "C:\\Users\\ke76boqe\\Projects\\Body_tracking\\body-tracking-samples\\PointCloud\\frame";
                k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    sensorCalibration.depth_camera_calibration.resolution_width,
                    sensorCalibration.depth_camera_calibration.resolution_height,
                    sensorCalibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                    &xy_table);

                create_xy_table(&sensorCalibration, xy_table);

                k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                    sensorCalibration.depth_camera_calibration.resolution_width,
                    sensorCalibration.depth_camera_calibration.resolution_height,
                    sensorCalibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
                    &point_cloud);

                /************* Successfully get a body tracking result, process the result here ***************/
                depth_image=VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight, frame_count); 

                // GENERATE AND SAVE POINT CLOUD HERE
                generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);
                char str[5];
                snprintf(str, 5, "%04d", frame_count);
                std::string s = "";

                s = point_cloud_file_name + str + ".ply";
                write_point_cloud(s.c_str(), point_cloud, point_count);
                //Release the bodyFrame
            /*    k4a_capture_t originalCapture = k4abt_frame_get_capture(capture);
                k4a_record_write_capture(record_handle, originalCapture);
                k4a_capture_release(originalCapture);*/

                if (K4A_FAILED(k4a_record_write_capture(record_handle, capture)))
                {
                    printf("Failed to write to mkv file");

                }
                
     
                k4a_result_t write_result = k4a_record_write_imu_sample(record_handle, imu_sample);
                if (K4A_FAILED(write_result))
                {
                    std::cerr << "Runtime error: k4a_record_write_imu_sample() returned " << write_result << std::endl;
                    break;
                }
                // Release the sensor capture once it is no longer needed.
                k4a_capture_release(capture);
                //k4a_capture_release(imu);
                k4abt_frame_release(bodyFrame);
            }
            else
            {
                std::cout << "Pop body frame result failed!" << std::endl;
                break;
            }
            if(frame_count==140)
            {
                break;
            }
        }

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    k4a_record_flush(record_handle);
    k4a_record_close(record_handle);

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    window3d.Delete();
    printf("Finished body tracking processing!\n");
    k4a_playback_close(playbackHandle);
}

void PlayFromDevice(InputSettings inputSettings) 
{
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = inputSettings.processingMode;
    trackerConfig.model_path = inputSettings.ModelPath.c_str();
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!");

    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_image_t depth_image = NULL;
            k4a_image_t xy_table = NULL;
            k4a_image_t point_cloud = NULL;
            int point_count = 0;
            //std::string file_name="C:\\Users\\ke76boqe\\Projects\\Body_tracking\\body-tracking-samples\\simple_3d_viewer\\build\\bin\\Debug\\out.ply";
            k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                sensorCalibration.depth_camera_calibration.resolution_width,
                sensorCalibration.depth_camera_calibration.resolution_height,
                sensorCalibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                &xy_table);

            create_xy_table(&sensorCalibration, xy_table);

            k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                sensorCalibration.depth_camera_calibration.resolution_width,
                sensorCalibration.depth_camera_calibration.resolution_height,
                sensorCalibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
                &point_cloud);

            /************* Successfully get a body tracking result, process the result here ***************/
            depth_image = VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight,0);

            generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);

            //write_point_cloud(file_name.c_str(), point_cloud, point_count);



            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }
       
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    std::cout << "Finished body tracking processing!" << std::endl;

    window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
}

int main(int argc, char** argv)
{
    InputSettings inputSettings;
   
    if (!ParseInputSettingsFromArg(argc, argv, inputSettings))
    {
        // Print app usage if user entered incorrect arguments.
        PrintUsage();
        return -1;
    }

    // Either play the offline file or play from the device
    if (inputSettings.Offline == true)
    {
        PlayFile(inputSettings);
    }
    else
    {
        PlayFromDevice(inputSettings);
    }

    return 0;
}
