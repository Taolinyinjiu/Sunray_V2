// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/base_realsense_node.h"
// #include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <regex>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

RealSenseNodeFactory::RealSenseNodeFactory()
    : _initial_reset(false), _is_alive(true)
{
	ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
	ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

	auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
	tryGetLogSeverity(severity);
	if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	rs2::log_to_console(severity);
}

void RealSenseNodeFactory::closeDevice()
{
    if (!_device)
    {
        ROS_INFO("closeDevice skipped: device handle is empty.");
        return;
    }

    ROS_INFO_STREAM("closeDevice begin. serial_no=" << _serial_no);
    for(rs2::sensor sensor : _device.query_sensors())
	{
        std::string sensor_name(sensor.get_info(RS2_CAMERA_INFO_NAME));
        try
        {
            ROS_INFO_STREAM("Stopping sensor in closeDevice: " << sensor_name);
		    sensor.stop();
            ROS_INFO_STREAM("Stopped sensor in closeDevice: " << sensor_name);
        }
        catch (const rs2::wrong_api_call_sequence_error& ex)
        {
            ROS_DEBUG_STREAM("closeDevice stop " << sensor_name << ": " << ex.what());
        }
        catch (const std::exception& ex)
        {
            ROS_WARN_STREAM("closeDevice stop failed for " << sensor_name << ": " << ex.what());
        }

        try
        {
            ROS_INFO_STREAM("Closing sensor in closeDevice: " << sensor_name);
		    sensor.close();
            ROS_INFO_STREAM("Closed sensor in closeDevice: " << sensor_name);
        }
        catch (const rs2::wrong_api_call_sequence_error& ex)
        {
            ROS_DEBUG_STREAM("closeDevice close " << sensor_name << ": " << ex.what());
        }
        catch (const std::exception& ex)
        {
            ROS_WARN_STREAM("closeDevice close failed for " << sensor_name << ": " << ex.what());
        }
	}
    ROS_INFO_STREAM("closeDevice end. serial_no=" << _serial_no);
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
	ROS_INFO_STREAM("RealSenseNodeFactory destructor begin. serial_no=" << _serial_no);
	_is_alive = false;
    try
    {
        ROS_INFO("Clearing device change callback.");
        _ctx.set_devices_changed_callback([](rs2::event_information&) {});
    }
    catch (const std::exception& ex)
    {
        ROS_DEBUG_STREAM("Failed clearing device change callback: " << ex.what());
    }

    if (_query_thread.joinable())
    {
        ROS_INFO("Joining query thread.");
        _query_thread.join();
        ROS_INFO("Query thread joined.");
    }

    if (_realSenseNode)
    {
        ROS_INFO("Calling node shutdown from factory destructor.");
        _realSenseNode->shutdown();
        ROS_INFO("Node shutdown completed.");
    }
	_realSenseNode.reset(nullptr);
    ROS_INFO("Node instance destroyed. Releasing device handle.");
	closeDevice();
    _device = rs2::device();
    ROS_INFO_STREAM("RealSenseNodeFactory destructor end. serial_no=" << _serial_no);
}

void RealSenseNodeFactory::getDevice(rs2::device_list list)
{
	if (!_device)
	{
		if (0 == list.size())
		{
			ROS_WARN("No RealSense devices were found!");
		}
		else
		{
			bool found = false;
      		ROS_INFO_STREAM(" ");
			for (auto&& dev : list)
			{
				auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				ROS_INFO_STREAM("Device with serial number " << sn << " was found."<<std::endl);
				std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
				std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
				ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
				std::string port_id;
				std::vector<std::string> results;
				ROS_INFO_STREAM("Device with name " << name << " was found.");
				std::regex self_regex;
				if(name == std::string("Intel RealSense T265"))
				{
					self_regex = std::regex(".*?bus_([0-9]+) port_([0-9]+).*", std::regex_constants::ECMAScript);
				}
				else// if(strcmp(name, "Intel RealSense D435") == 0)
				{
					self_regex = std::regex("[^ ]+/usb[0-9]+[0-9./-]*/([0-9.-]+):[^ ]*", std::regex_constants::ECMAScript);
				}
				std::smatch base_match;
				bool found_usb_desc = std::regex_match(pn, base_match, self_regex);
				if (found_usb_desc)
				{
					std::ssub_match base_sub_match = base_match[1];
					port_id = base_sub_match.str();
					for (unsigned int mi = 2; mi < base_match.size(); mi++)
					{
						std::ssub_match base_sub_match = base_match[mi];
						port_id += "-" + base_sub_match.str();
					}
					ROS_INFO_STREAM("Device with port number " << port_id << " was found.");
				}
				else
				{
					std::stringstream msg;
					msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
					if (_usb_port_id.empty())
					{
						ROS_WARN_STREAM(msg.str());
					}
					else
					{
						ROS_ERROR_STREAM(msg.str());
						ROS_ERROR_STREAM("Please use serial number instead of usb port.");
					}
				}
				bool found_device_type(true);
				if (!_device_type.empty())
				{
					std::regex device_type_regex(_device_type.c_str(), std::regex::icase);
					found_device_type = std::regex_search(name, base_match, device_type_regex);
				}

				if ((_serial_no.empty() || sn == _serial_no) && (_usb_port_id.empty() || port_id == _usb_port_id) && found_device_type)
				{
					_device = dev;
					_serial_no = sn;
					found = true;
					break;
				}
			}
			if (!found)
			{
				// T265 could be caught by another node.
				std::string msg ("The requested device with ");
				bool add_and(false);
				if (!_serial_no.empty())
				{
					msg += "serial number " + _serial_no;
					add_and = true;
				}
				if (!_usb_port_id.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "usb port id " + _usb_port_id;
					add_and = true;
				}
				if (!_device_type.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "device name containing " + _device_type;
				}
				msg += " is NOT found. Will Try again.";
				ROS_ERROR_STREAM(msg);
			}
		}
	}

	bool remove_tm2_handle(_device && RS_T265_PID != std::stoi(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
	if (remove_tm2_handle)
	{
		_ctx.unload_tracking_module();
	}

	if (_device && _initial_reset)
	{
		_initial_reset = false;
		try
		{
			ROS_INFO("Resetting device...");
			_device.hardware_reset();
			// _device = rs2::device();
			
		}
		catch(const std::exception& ex)
		{
			ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
		}
	}
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
	if (!_is_alive)
	{
		return;
	}

    ROS_INFO_STREAM("Device change callback triggered. has_device=" << static_cast<bool>(_device));

	if (info.was_removed(_device))
	{
		ROS_ERROR_STREAM("The device has been disconnected! serial_no=" << _serial_no);
		_realSenseNode.reset(nullptr);
		_device = rs2::device();
	}
	if (!_device)
	{
		rs2::device_list new_devices = info.get_new_devices();
		if (new_devices.size() > 0)
		{
			ROS_INFO("Checking new devices...");
			getDevice(new_devices);
			if (_device)
			{
				StartDevice();
			}
		}
	}
}

void RealSenseNodeFactory::onInit()
{
	try
	{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif
		ros::NodeHandle nh = getNodeHandle();
		auto privateNh = getPrivateNodeHandle();
		privateNh.param("serial_no", _serial_no, std::string(""));
    	privateNh.param("usb_port_id", _usb_port_id, std::string(""));
    	privateNh.param("device_type", _device_type, std::string(""));

		std::string rosbag_filename("");
		privateNh.param("rosbag_filename", rosbag_filename, std::string(""));
		if (!rosbag_filename.empty())
		{
			{
				ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
				auto pipe = std::make_shared<rs2::pipeline>();
				rs2::config cfg;
				cfg.enable_device_from_file(rosbag_filename.c_str(), false);
				cfg.enable_all_streams();
				pipe->start(cfg); //File will be opened in read mode at this point
				_device = pipe->get_active_profile().get_device();
				_serial_no = _device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
			}
			if (_device)
			{
				StartDevice();
			}
		}
		else
		{
			privateNh.param("initial_reset", _initial_reset, false);

			_query_thread = std::thread([this]()
						{
							std::chrono::milliseconds timespan(6000);
                            ROS_INFO("Query thread started.");
							while (_is_alive && !_device)
							{
								// _ctx.init_tracking_module(); // Unavailable function.
								getDevice(_ctx.query_devices());
								if (_device && _is_alive)
								{
                                    ROS_INFO_STREAM("Device acquired in query thread. serial_no=" << _serial_no);
									std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
									_ctx.set_devices_changed_callback(change_device_callback_function);
									StartDevice();
								}
								else if (_is_alive)
								{
                                    ROS_DEBUG("Query thread did not find a matching device. Sleeping before retry.");
									std::this_thread::sleep_for(timespan);
								}
							}
                            ROS_INFO_STREAM("Query thread exiting. is_alive=" << _is_alive << ", has_device=" << static_cast<bool>(_device));
						});
			}
	}
	catch(const std::exception& ex)
	{
		ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
		exit(1);
	}
	catch(...)
	{
		ROS_ERROR_STREAM("Unknown exception has occured!");
		exit(1);
	}
}

void RealSenseNodeFactory::StartDevice()
{
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle privateNh = getPrivateNodeHandle();
	// TODO
	std::string pid_str(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
	uint16_t pid = std::stoi(pid_str, 0, 16);
	switch(pid)
	{
	case SR300_PID:
	case SR300v2_PID:
	case RS400_PID:
	case RS405_PID:
	case RS410_PID:
	case RS460_PID:
	case RS415_PID:
	case RS420_PID:
	case RS420_MM_PID:
	case RS430_PID:
	case RS430_MM_PID:
	case RS430_MM_RGB_PID:
	case RS435_RGB_PID:
	case RS435i_RGB_PID:
	case RS_USB2_PID:
		_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
		break;
	// case RS_T265_PID:
	// 	_realSenseNode = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _device, _serial_no));
	// 	break;
	default:
		ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
		ros::shutdown();
		exit(1);
	}
	assert(_realSenseNode);
	_realSenseNode->publishTopics();
}

void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
	static const char* severity_var_name = "LRS_LOG_LEVEL";
	auto content = getenv(severity_var_name);

	if (content)
	{
		std::string content_str(content);
		std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

		for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
		{
			auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
			std::transform(current.begin(), current.end(), current.begin(), ::toupper);
			if (content_str == current)
			{
				severity = (rs2_log_severity)i;
				break;
			}
		}
	}
}
