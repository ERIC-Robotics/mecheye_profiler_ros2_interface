#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/string.hpp>
#include <profiler/api_util.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <MechMindProfiler4.h>

namespace {

constexpr double kUmToM = 1e-6;
constexpr double kMmToM = 1e-3;

sensor_msgs::msg::PointField createPointField4(const std::string& name, uint32_t offset,
                                              uint8_t datatype, uint32_t count)
{
    sensor_msgs::msg::PointField point_field;
    point_field.name = name;
    point_field.offset = offset;
    point_field.datatype = datatype;
    point_field.count = count;
    return point_field;
}

void convertToROSMsg4(const mmind::eye::ProfileBatch& batch, float xResolution, float yResolution,
                     bool useEncoderValues, int triggerInterval,
                     sensor_msgs::msg::PointCloud2& cloud,
                     sensor_msgs::msg::PointCloud2& texturedCloud)
{
    const auto height = batch.height();
    const auto width = batch.width();
    cloud.height = height;
    texturedCloud.height = height;
    cloud.width = width;
    texturedCloud.width = width;
    cloud.is_dense = false;
    texturedCloud.is_dense = false;
    cloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    texturedCloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    cloud.point_step = sizeof(float) * 3;
    texturedCloud.point_step = sizeof(float) * 4;
    cloud.row_step = cloud.point_step * cloud.width;
    texturedCloud.row_step = texturedCloud.point_step * texturedCloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);
    texturedCloud.data.resize(texturedCloud.row_step * texturedCloud.height);
    cloud.fields.reserve(3);
    cloud.fields.push_back(createPointField4("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
    cloud.fields.push_back(
        createPointField4("y", sizeof(float), sensor_msgs::msg::PointField::FLOAT32, 1));
    cloud.fields.push_back(
        createPointField4("z", sizeof(float) * 2, sensor_msgs::msg::PointField::FLOAT32, 1));

    texturedCloud.fields.reserve(4);
    texturedCloud.fields.push_back(
        createPointField4("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
    texturedCloud.fields.push_back(
        createPointField4("y", sizeof(float), sensor_msgs::msg::PointField::FLOAT32, 1));
    texturedCloud.fields.push_back(
        createPointField4("z", sizeof(float) * 2, sensor_msgs::msg::PointField::FLOAT32, 1));
    texturedCloud.fields.push_back(
        createPointField4("intensity", sizeof(float) * 3, sensor_msgs::msg::PointField::UINT8, 1));

    memcpy(cloud.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZ*>(
               batch
                   .getUntexturedPointCloud(xResolution, yResolution, useEncoderValues,
                                            triggerInterval, mmind::eye::CoordinateUnit::Meter)
                   .data())),
           (cloud.row_step * cloud.height));
    memcpy(texturedCloud.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZI*>(
               batch
                   .getTexturedPointCloud(xResolution, yResolution, useEncoderValues,
                                          triggerInterval, mmind::eye::CoordinateUnit::Meter)
                   .data())),
           (texturedCloud.row_step * texturedCloud.height));
}

void callbackFunc4(const mmind::eye::ProfileBatch& batch, void* pUser)
{
    auto* mechMindProfiler = static_cast<mechmind4::MechMindProfiler4*>(pUser);
    if (mechMindProfiler)
        mechMindProfiler->handleCallbackBatch(batch);
}
} // namespace

namespace mechmind4 {

MechMindProfiler4::MechMindProfiler4(const rclcpp::NodeOptions & options)
: rclcpp::Node("mechmind_profiler_publisher_service_4", options)
{
    declare_parameter<std::string>("profiler_ip", "192.168.0.40");
    declare_parameter<bool>("save_file", false);

    get_parameter("profiler_ip", profiler_ip);
    get_parameter("save_file", save_file);

    pub_intensity =
        create_publisher<sensor_msgs::msg::Image>("/mechmind_profiler_4/intensity_image", 1);
    pub_depth = create_publisher<sensor_msgs::msg::Image>("/mechmind_profiler_4/depth_map", 1);
    pub_pcl =
        create_publisher<sensor_msgs::msg::PointCloud2>("/mechmind_profiler_4/point_cloud", 1);
    pub_textured_pcl = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mechmind_profiler_4/textured_point_cloud", 1);

    mmind::eye::ErrorStatus status;
    mmind::eye::ProfilerInfo info;
    info.firmwareVersion = mmind::eye::Version("2.5.4");
    info.ipAddress = profiler_ip;
    info.port = 5577;
    status = profiler.connect(info);
    if (!status.isOK())
    {
        throw status;
    }
    std::cout << "Profiler 4: Connected to the profiler successfully." << std::endl;

    mmind::eye::ProfilerInfo profilerInfo;
    showError(profiler.getProfilerInfo(profilerInfo));
    printProfilerInfo(profilerInfo);

    status = profiler.registerAcquisitionCallback(callbackFunc4, this);
    if (!status.isOK()) {
        throw status;
    }

    start_acquisition_service =
        create_service<mecheye_profiler_ros_interface::srv::StartAcquisition>(
            "start_acquisition_4", std::bind(&MechMindProfiler4::start_acquisition_callback, this,
                                           std::placeholders::_1, std::placeholders::_2));
    stop_acquisition_service =
        create_service<mecheye_profiler_ros_interface::srv::StopAcquisition>(
            "stop_acquisition_4", std::bind(&MechMindProfiler4::stop_acquisition_callback, this,
                                          std::placeholders::_1, std::placeholders::_2));
    trigger_software_service =
        create_service<mecheye_profiler_ros_interface::srv::TriggerSoftware>(
            "trigger_software_4", std::bind(&MechMindProfiler4::trigger_software_callback, this,
                                          std::placeholders::_1, std::placeholders::_2));

    add_user_set_service = create_service<mecheye_profiler_ros_interface::srv::AddUserSet>(
        "add_user_set_4", std::bind(&MechMindProfiler4::add_user_set_callback, this,
                                  std::placeholders::_1, std::placeholders::_2));

    delete_user_set_service =
        create_service<mecheye_profiler_ros_interface::srv::DeleteUserSet>(
            "delete_user_set_4", std::bind(&MechMindProfiler4::delete_user_set_callback, this,
                                         std::placeholders::_1, std::placeholders::_2));
    profiler_info_service = create_service<mecheye_profiler_ros_interface::srv::ProfilerInfo>(
        "profiler_info_4", std::bind(&MechMindProfiler4::profiler_info_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

    get_all_user_sets_service =
        create_service<mecheye_profiler_ros_interface::srv::GetAllUserSets>(
            "get_all_user_sets_4", std::bind(&MechMindProfiler4::get_all_user_sets_callback, this,
                                           std::placeholders::_1, std::placeholders::_2));

    get_current_user_set_service =
        create_service<mecheye_profiler_ros_interface::srv::GetCurrentUserSet>(
            "get_current_user_set_4", std::bind(&MechMindProfiler4::get_current_user_set_callback,
                                              this, std::placeholders::_1, std::placeholders::_2));

    save_all_settings_to_user_sets_service =
        create_service<mecheye_profiler_ros_interface::srv::SaveAllSettingsToUserSets>(
            "save_all_settings_to_user_sets_4",
            std::bind(&MechMindProfiler4::save_all_settings_to_user_sets_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

    set_current_user_set_service =
        create_service<mecheye_profiler_ros_interface::srv::SetCurrentUserSet>(
            "set_current_user_set_4", std::bind(&MechMindProfiler4::set_current_user_set_callback,
                                              this, std::placeholders::_1, std::placeholders::_2));

    set_int_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::SetIntParameter>(
            "set_int_parameter_4", std::bind(&MechMindProfiler4::set_int_parameter_callback, this,
                                           std::placeholders::_1, std::placeholders::_2));

    get_int_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::GetIntParameter>(
            "get_int_parameter_4", std::bind(&MechMindProfiler4::get_int_parameter_callback, this,
                                           std::placeholders::_1, std::placeholders::_2));

    set_float_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::SetFloatParameter>(
            "set_float_parameter_4", std::bind(&MechMindProfiler4::set_float_parameter_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

    get_float_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::GetFloatParameter>(
            "get_float_parameter_4", std::bind(&MechMindProfiler4::get_float_parameter_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

    set_bool_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::SetBoolParameter>(
            "set_bool_parameter_4", std::bind(&MechMindProfiler4::set_bool_parameter_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));

    get_bool_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::GetBoolParameter>(
            "get_bool_parameter_4", std::bind(&MechMindProfiler4::get_bool_parameter_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));

    set_enum_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::SetEnumParameter>(
            "set_enum_parameter_4", std::bind(&MechMindProfiler4::set_enum_parameter_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));

    get_enum_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::GetEnumParameter>(
            "get_enum_parameter_4", std::bind(&MechMindProfiler4::get_enum_parameter_callback, this,
                                            std::placeholders::_1, std::placeholders::_2));

    set_profile_roi_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::SetProfileROIParameter>(
            "set_profile_roi_parameter_4",
            std::bind(&MechMindProfiler4::set_profile_roi_parameter_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

    get_profile_roi_parameter_service =
        create_service<mecheye_profiler_ros_interface::srv::GetProfileROIParameter>(
            "get_profile_roi_parameter_4",
            std::bind(&MechMindProfiler4::get_profile_roi_parameter_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
}

void MechMindProfiler4::handleCallbackBatch(const mmind::eye::ProfileBatch& batch)
{
    auto batchBackup = batch;
    if (!batchBackup.getErrorStatus().isOK()) {
        showError(batch.getErrorStatus());
        return;
    }
    if (batchBackup.checkFlag(mmind::eye::ProfileBatch::BatchFlag::Incomplete))
        std::cout << "Part of the batch's data is lost, the number of valid profiles is: "
                  << batchBackup.validHeight() << "." << std::endl;
    publishIntensityImage(batchBackup.getIntensityImage());
    publishDepthMap(batchBackup.getDepthMap());
    publishPointClouds(batchBackup, profiler.currentUserSet());
}

void MechMindProfiler4::publishIntensityImage(
    mmind::eye::ProfileBatch::IntensityImage&& intensityImage)
{
    cv::Mat intensity =
        cv::Mat(intensityImage.height(), intensityImage.width(), CV_8UC1, intensityImage.data());
    cv_bridge::CvImage cv_image;
    cv_image.image = intensity;
    cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    sensor_msgs::msg::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "mechmind_profiler_4/intensity_image";
    ros_image.header.stamp = this->now();
    pub_intensity->publish(ros_image);
    if (!save_file)
        return;
    if (cv::imwrite("/tmp/time.time_intensity_image_4.png", intensity))
        std::cout << "The intensity image is saved to /tmp" << std::endl;
    else
        std::cerr << "Failed to save the intensity image." << std::endl;
}

void MechMindProfiler4::publishDepthMap(mmind::eye::ProfileBatch::DepthMap&& depthMap)
{
    cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    cv_bridge::CvImage cv_depth;
    cv_depth.image = depth;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::msg::Image ros_depth;
    cv_depth.toImageMsg(ros_depth);
    ros_depth.header.frame_id = "mechmind_profiler_4/depth_map";
    ros_depth.header.stamp = this->now();
    pub_depth->publish(ros_depth);
    if (!save_file)
        return;
    if (cv::imwrite("/tmp/depth_map_4.tiff", depth))
        std::cout << "The depth map is saved to /tmp." << std::endl;
    else
        std::cerr << "Failed to save the depth map." << std::endl;
}

void MechMindProfiler4::publishPointClouds(const mmind::eye::ProfileBatch& batch,
                                          const mmind::eye::UserSet& userSet)
{
    if (batch.isEmpty())
        return;

    double xResolution{};
    auto status = userSet.getFloatValue(mmind::eye::point_cloud_resolutions::XAxisResolution::name,
                                        xResolution);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    double yResolution{};
    status =
        userSet.getFloatValue(mmind::eye::point_cloud_resolutions::YResolution::name, yResolution);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    int lineScanTriggerSource{};
    status = userSet.getEnumValue(mmind::eye::trigger_settings::LineScanTriggerSource::name,
                                  lineScanTriggerSource);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    bool useEncoderValues =
        lineScanTriggerSource ==
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::Encoder);

    int triggerInterval{};
    status = userSet.getIntValue(mmind::eye::trigger_settings::EncoderTriggerInterval::name,
                                 triggerInterval);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    sensor_msgs::msg::PointCloud2 ros_cloud;
    ros_cloud.header.frame_id = "mechmind_profiler_4/point_cloud";
    ros_cloud.header.stamp = this->now();
    sensor_msgs::msg::PointCloud2 ros_textured_cloud;
    ros_textured_cloud.header.frame_id = "mechmind_profiler_4/textured_point_cloud";
    ros_textured_cloud.header.stamp = this->now();
    convertToROSMsg4(batch, xResolution, yResolution, useEncoderValues, triggerInterval, ros_cloud,
                    ros_textured_cloud);
    pub_pcl->publish(ros_cloud);
    pub_textured_pcl->publish(ros_textured_cloud);
}

void MechMindProfiler4::start_acquisition_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::StartAcquisition::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::StartAcquisition::Response> res)
{
    auto status = profiler.startAcquisition();
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::stop_acquisition_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::StopAcquisition::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::StopAcquisition::Response> res)
{
    auto status = profiler.stopAcquisition();
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::trigger_software_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::TriggerSoftware::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::TriggerSoftware::Response> res)
{
    auto status = profiler.triggerSoftware();
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::profiler_info_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::ProfilerInfo::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::ProfilerInfo::Response> res)
{
    mmind::eye::ProfilerInfo profilerInfo;
    mmind::eye::ErrorStatus status = profiler.getProfilerInfo(profilerInfo);
    showError(status);
    res->model = profilerInfo.model.c_str();
    res->controller_sn = profilerInfo.controllerSN.c_str();
    res->sensor_sn = profilerInfo.sensorSN.c_str();
    res->hardware_version = profilerInfo.hardwareVersion.toString();
    res->firmware_version = profilerInfo.firmwareVersion.toString();
    res->ip_address = profilerInfo.ipAddress.c_str();
    res->subnet_mask = profilerInfo.subnetMask.c_str();
    res->ip_assignment_method = ipAssignmentMethodToString(profilerInfo.ipAssignmentMethod);
    res->port = profilerInfo.port;
}

void MechMindProfiler4::get_all_user_sets_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetAllUserSets::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::GetAllUserSets::Response> res)
{
    std::vector<std::string> sequence;
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.getAllUserSetNames(sequence);
    showError(status);
    res->sequence = sequence;
}

void MechMindProfiler4::get_current_user_set_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetCurrentUserSet::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::GetCurrentUserSet::Response> res)
{
    std::string value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getName(value);
    showError(status);
    res->value = value.c_str();
}

void MechMindProfiler4::set_current_user_set_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetCurrentUserSet::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::SetCurrentUserSet::Response> res)
{
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.selectUserSet(req->value.c_str());
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::add_user_set_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::AddUserSet::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::AddUserSet::Response> res)
{
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.addUserSet({req->value.c_str()});
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::delete_user_set_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::DeleteUserSet::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::DeleteUserSet::Response> res)
{
    mmind::eye::UserSetManager userSetManager = profiler.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.deleteUserSet({req->value.c_str()});
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::save_all_settings_to_user_sets_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::SaveAllSettingsToUserSets::Request>
        req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::SaveAllSettingsToUserSets::Response> res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.saveAllParametersToDevice();
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::get_profile_roi_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetProfileROIParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::GetProfileROIParameter::Response> res)
{
    mmind::eye::ProfileROI roi;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getProfileRoiValue(req->name, roi);
    showError(status);
    res->x_axis_center = roi.xAxisCenter;
    res->width = roi.width;
    res->height = roi.height;
}

void MechMindProfiler4::set_profile_roi_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetProfileROIParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::SetProfileROIParameter::Response> res)
{
    mmind::eye::ProfileROI roi{req->x_axis_center, req->width, req->height};
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setProfileRoiValue(req->name, roi);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::get_int_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetIntParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::GetIntParameter::Response> res)
{
    int value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getIntValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindProfiler4::set_int_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetIntParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::SetIntParameter::Response> res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setIntValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::get_bool_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetBoolParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::GetBoolParameter::Response> res)
{
    bool value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getBoolValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindProfiler4::set_bool_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetBoolParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::SetBoolParameter::Response> res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setBoolValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::get_enum_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetEnumParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::GetEnumParameter::Response> res)
{
    std::string value;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getEnumValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindProfiler4::set_enum_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetEnumParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::SetEnumParameter::Response> res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setEnumValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindProfiler4::get_float_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::GetFloatParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::GetFloatParameter::Response> res)
{
    double value = -1;
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getFloatValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindProfiler4::set_float_parameter_callback(
    const std::shared_ptr<mecheye_profiler_ros_interface::srv::SetFloatParameter::Request> req,
    std::shared_ptr<mecheye_profiler_ros_interface::srv::SetFloatParameter::Response> res)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setFloatValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

} // namespace mechmind4

RCLCPP_COMPONENTS_REGISTER_NODE(mechmind4::MechMindProfiler4)
