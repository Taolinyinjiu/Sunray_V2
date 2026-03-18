/**
 * @file external_fusion.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-18
 * @see https://yundrone.feishu.cn/wiki/NukDw1pCKiLD4rkhX5gcOGjWnAh
 * @copyright Copyright (c) 2026
 * 
 */

#include <ros/node_handle.h>

class ExternalFusion{
	public:
	explicit ExternalFusion(ros::NodeHandle &nh);
	~ExternalFusion() = default;
	
	// 局部定位回调函数
	
	// 全局定位回调函数
	
	// loop_check 回环检测函数


	// 检查local和global的坐标转换关系并发布

};