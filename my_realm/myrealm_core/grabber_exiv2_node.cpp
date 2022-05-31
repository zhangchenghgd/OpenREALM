#include "grabber_exiv2_node.h"
#include "MyREALM.h"
#include <glog/logging.h>
#include <chrono>
#include <thread>

using namespace realm;

namespace MyREALM
{
    Exiv2GrabberNode::Exiv2GrabberNode(const NodeParas& node_paras)
        :OpenThreads::Thread(), _node_paras(node_paras), _do_set_all_keyframes(false),
        _use_apriori_pose(false),
        _use_apriori_georeference(false),
        _use_apriori_surface_pts(false),
        _fps(0.0),
        _id_curr_file(0),
        _cam(nullptr)
    {
        readParams();
        setPaths();

        LOG(INFO) << "Start";

        _exiv2_reader = io::Exiv2FrameReader(
            io::Exiv2FrameReader::FrameTags::loadFromFile(_path_profile + "/config/exif.yaml"));

        if (io::fileExists(_file_settings_camera))
            _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_file_settings_camera));
        else
            throw(std::invalid_argument("Error loading camera file: Provided path does not exist."));
        //_cam_msg = to_ros::pinhole(_cam);

        // Loading poses from file if provided
        if (_use_apriori_pose)
        {
            _poses = io::loadTrajectoryFromTxtTUM(_filepath_poses);
            LOG(INFO) << "Succesfully loaded " << _poses.size() << " external poses.";
        }

        // Loading georeference from file if provided
        if (_use_apriori_georeference)
        {
            _georeference = io::loadGeoreferenceFromYaml(_filepath_georeference);
            LOG(INFO) << "Succesfully loaded external georeference:\n" << _georeference;
        }

        // Loading surface points from file if provided
        if (_use_apriori_surface_pts)
        {
            _surface_pts = io::loadSurfacePointsFromTxt(_filepath_surface_pts);
            LOG(INFO) << "Succesfully loaded " << _surface_pts.rows << " external surface points.";
        }

        LOG(INFO) <<
            "Exiv2 Grabber Node [Ankommen]: Successfully loaded camera: "
            << "\n\tcx = " << _cam->cx()
            << "\n\tcy = " << _cam->cy()
            << "\n\tfx = " << _cam->fx()
            << "\n\tfy = " << _cam->fy()
            << "\n\tk1 = " << _cam->k1()
            << "\n\tk2 = " << _cam->k2()
            << "\n\tp1 = " << _cam->p1()
            << "\n\tp2 = " << _cam->p2();

        // ROS related inits
        _topic_prefix = "/realm/" + _id_node;

        _pub_frame = MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "/input");
        
        // _pub_frame = _nh.advertise<realm_msgs::Frame>(_topic_prefix + "/input", 5);
        // _pub_image = _nh.advertise<sensor_msgs::Image>(_topic_prefix + "/img", 5);


        // Start grabbing images
        _file_list = getFileList(_path_grab);

        // Check if the exif tags in the config exist
        LOG(INFO) << "Scanning input image for provided meta tags...";
        std::map<std::string, bool> tag_existence = _exiv2_reader.probeImage(_file_list[0]);
        for (const auto& tag : tag_existence)
        {
            if (tag.second)
                LOG(INFO) << "[FOUND]\t\t'" << tag.first.c_str() << "'";

            else
                LOG(INFO) << "[NOT FOUND]\t'" << tag.first.c_str() << "'";
        }
    }

    void Exiv2GrabberNode::readParams()
    {
        std::string _fps_str;
        std::string _do_set_all_keyframes_str = "false";

        _node_paras.param("config/id", _id_node, std::string("uninitialised"));
        _node_paras.param("config/input", _path_grab, std::string("uninitialised"));
        _node_paras.param("config/rate", _fps_str, "0.0");
        _node_paras.param("config/profile", _profile, std::string("uninitialised"));
        _node_paras.param("config/opt/poses", _filepath_poses, std::string("uninitialised"));
        _node_paras.param("config/opt/georeference", _filepath_georeference, std::string("uninitialised"));
        _node_paras.param("config/opt/surface_pts", _filepath_surface_pts, std::string("uninitialised"));
        _node_paras.param("config/opt/set_all_keyframes", _do_set_all_keyframes_str, std::string("false"));
        _node_paras.param("config/opt/working_directory", _path_working_directory, std::string("uninitialised"));


        _fps = atof(_fps_str.c_str());
        _do_set_all_keyframes = _do_set_all_keyframes_str.compare("true") == 0;

        /*_id_node = "CAMERA_1";
        _path_grab = "F:/OpenREALM2/data/edm_big_overlap_50p";
        _fps = atof(_fps_str.c_str());
        _profile = std::string("alexa_reco");
        _filepath_poses = std::string("uninitialised");
        _filepath_georeference = std::string("uninitialised");
        _filepath_surface_pts = std::string("uninitialised");
        _do_set_all_keyframes = false;
        _path_working_directory = "F:/OpenREALM2/data";*/

        if (_fps < 0.01)
            throw(std::invalid_argument("Error reading exiv2 grabber parameters: Frame rate is too low!"));
        if (_filepath_poses != "uninitialised")
            _use_apriori_pose = true;
        if (_filepath_georeference != "uninitialised")
            _use_apriori_georeference = true;
        if (_filepath_surface_pts != "uninitialised")
            _use_apriori_surface_pts = true;
        if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
            throw(std::invalid_argument("Error: Working directory does not exist!"));
    }

    void Exiv2GrabberNode::setPaths()
    {
        // if (_path_working_directory == "uninitialised")
        //   _path_working_directory = ros::package::getPath("realm_ros");
        _path_profile = _path_working_directory + "/profiles/" + _profile;

        _file_settings_camera = _path_profile + "/camera/calib.yaml";

        if (!io::dirExists(_path_profile))
            throw(std::invalid_argument("Error: Config folder path '" + _path_profile + "' does not exist!"));
        if (!io::dirExists(_path_grab))
            throw(std::invalid_argument("Error grabbing Exiv2 images: Folder '" + _path_grab + "' does not exist!"));
    }

    void Exiv2GrabberNode::spin()
    {
        std::chrono::milliseconds sleep_interval((int)(1000 / _fps));
        //ros::Rate rate(_fps);
        if (_id_curr_file < _file_list.size())
        {
            LOG(INFO) << "Image #" << _id_curr_file << ", image Path: " << _file_list[_id_curr_file];

            realm::camera::Pinhole::Ptr frame_cam = std::make_shared<realm::camera::Pinhole>(*_cam.get());
            Frame::Ptr frame = _exiv2_reader.loadFrameFromExiv2(_id_node, frame_cam, _file_list[_id_curr_file]);

            // External pose can be provided
            if (_use_apriori_pose)
            {
                cv::Mat pose = _poses[frame->getTimestamp()];
                if (pose.empty())
                    throw(std::runtime_error("Error adding external pose informations: "\
                        "No pose was found. Maybe images or provided pose file do not match?"));
                frame->setVisualPose(pose);
                if (_do_set_all_keyframes)
                    frame->setKeyframe(true);
            }

            if (_use_apriori_georeference)
            {
                frame->updateGeoreference(_georeference);
            }

            // External surface points can be provided
            if (_use_apriori_surface_pts)
            {
                //if (_surface_pts.empty())
                throw(std::runtime_error("Error adding external surface points: "\
                    "No points were found!"));
                //frame->setSparseCloud(_surface_pts, false);
            }

            pubFrame(frame);
            _id_curr_file++;
        }

        std::vector<std::string> new_file_list = getFileList(_path_grab);
        if (new_file_list.size() != _file_list.size())
        {
            LOG(INFO) << "Processed images in folder: " << _file_list.size() << " / " << new_file_list.size();
            std::set_difference(new_file_list.begin(), new_file_list.end(), _file_list.begin(), _file_list.end(), std::back_inserter(_file_list));
            std::sort(_file_list.begin(), _file_list.end());
        }

        //rate.sleep();
        std::this_thread::sleep_for(sleep_interval);
    }

    void Exiv2GrabberNode::pubFrame(const Frame::Ptr& frame)
    {
        //// Create message header
        //std_msgs::Header header;
        //header.stamp = ros::Time::now();
        //header.frame_id = "map";

        //// Create message informations
        //realm_msgs::Frame msg_frame = to_ros::frame(header, frame);
        //sensor_msgs::Image msg_img = *to_ros::imageDisplay(header, frame->getImageRaw()).toImageMsg();

        //_pub_frame.publish(msg_frame);
        //_pub_image.publish(msg_img);

        _pub_frame->pubFrame(frame);
    }

    bool Exiv2GrabberNode::isOkay()
    {
        return MyRealmSys::get_instance().ok();
    }

    int Exiv2GrabberNode::cancel()
    {
        _done = true;
        while (isRunning()) YieldCurrentThread();
        return 0;
    }

    void Exiv2GrabberNode::run()
    {
        _done = false;
        _dirty = true;
        do
        {
            //YieldCurrentThread();
            this->spin();

        } while (!_done);
    }

    std::vector<std::string> Exiv2GrabberNode::getFileList(const std::string& path)
    {
        std::vector<std::string> file_names;
        if (!path.empty())
        {
            boost::filesystem::path apk_path(path);
            boost::filesystem::recursive_directory_iterator end;

            for (boost::filesystem::recursive_directory_iterator it(apk_path); it != end; ++it)
            {
                const boost::filesystem::path cp = (*it);
                file_names.push_back(cp.string());
            }
        }
        std::sort(file_names.begin(), file_names.end());
        return file_names;
    }
}