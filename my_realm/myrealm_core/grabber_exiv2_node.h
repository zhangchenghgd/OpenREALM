#ifndef PROJECT_GRABBER_EXIV_NODE_H
#define PROJECT_GRABBER_EXIV_NODE_H

#include "MyREALM_Core_Exports.h"
#include <iostream>

#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>

//#include <realm_ros/conversions.h>
#include <realm_io/realm_import.h>
#include <realm_io/exif_import.h>
#include <realm_io/utilities.h>
#include "MyPublisher.h"
#include "NodeParas.h"
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

namespace MyREALM
{

/*!
 * @brief Simple "image from folder"-grabber for Exiv2 tagged images to convert into REALM input frames. Images can be
 * added dynamically, however the order accoding to the image names should be considered. Therefore DON'T add images in
 * wrong order. Additionally a pose file in TUM format can be provided to feed external pose data into the frame.
 * Exiv2 tags should at least contain the GNSS position. Tested name format is name_000000.suffix
 */

class MyREALM_Core_API Exiv2GrabberNode : public OpenThreads::Thread
{
  public:
    Exiv2GrabberNode(const NodeParas& node_paras);
    void spin();
    bool isOkay();

    virtual int cancel();
    virtual void run();

  private:

      OpenThreads::Mutex _mutex;
      bool _done;
      bool _dirty;

    bool _do_set_all_keyframes;
    NodeParas _node_paras;

    // External pose can be provided in the format of a TUM trajectory file
    bool _use_apriori_pose;
    std::string _filepath_poses;
    std::unordered_map<uint64_t, cv::Mat> _poses;

    // External georeference can be provided in YAML format
    bool _use_apriori_georeference;
    std::string _filepath_georeference;
    cv::Mat _georeference;

    // External surface points can be provided in simple txt format with x, y, z
    bool _use_apriori_surface_pts;
    std::string _filepath_surface_pts;
    cv::Mat _surface_pts;

    double _fps;

    std::string _id_node;

    std::string _profile;
    std::string _topic_prefix;

    std::string _path_grab;
    std::string _path_working_directory;
    std::string _path_profile;

    std::string _file_settings_camera;

    realm::io::Exiv2FrameReader _exiv2_reader;

    // ros::NodeHandle _nh;
    MyPublisher* _pub_frame;
    MyPublisher* _pub_image;

    realm::camera::Pinhole::Ptr _cam;
    // realm_msgs::Pinhole _cam_msg;

    size_t _id_curr_file;
    std::vector<std::string> _file_list;

    void readParams();
    void setPaths();
    void pubFrame(const realm::Frame::Ptr &frame);
    std::vector<std::string> getFileList(const std::string& path);
};

} // namespace MyREALM

#endif //PROJECT_GRABBER_EXIV_NODE
