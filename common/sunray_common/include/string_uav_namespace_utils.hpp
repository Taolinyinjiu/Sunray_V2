/**
 * @file string_uav_namespace_utils.hpp
 * @author your name (you@domain.com)
 * @brief string_uav_namespace_utils.hpp 设计意图为，简化对uav_ns的处理
 * @version 0.1
 * @date 2026-03-23
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <string>

namespace sunray_common {
// 规范化输入的uav_namespace,目标uav_ns = /uav_name+uav_id
inline std::string normalize_uav_ns(std::string uav_ns) {
  // 首先判断传入的uav_ns是否为空
  if (uav_ns.empty()) {
    // 考虑到本文件的工具属性，异常，报错等应该交给上层调用层来处理，因此此处返回空串
		return uav_ns;
  }
  // 如果uav_ns开头缺少”/“则进行补充
  if (uav_ns.front() != '/') {
    uav_ns.insert(uav_ns.begin(), '/');
  }
  // 如果结尾有'/'则删掉
  if (uav_ns.size() > 1 && uav_ns.back() == '/') {
    uav_ns.pop_back();
  }

  return uav_ns;
}

inline std::string replace_string(std::string raw_string,const std::string &occupy_string, const std::string &replace_string)
{
	// 第一个参数是原始字符串，使用值拷贝，因此我们直接使用raw_string作为返回值
	// 首先判断三个串都非空
	if(raw_string.empty())
	{
		// 考虑到本文件的工具属性，异常，报错等应该交给上层调用层来处理，因此此处返回空串
		return raw_string;
	}
	else if(occupy_string.empty())
	{
		return occupy_string;
	}
	else if(replace_string.empty())
	{
		return replace_string;
	}

	// 三个输入都非空，在raw_string中寻找是否存在占位符
	if(raw_string.find(occupy_string) == std::string::npos)
	{
		// 原串中不存在占位符，则返回原串
		return raw_string;
	}
	// 原串中存在占位符，切循环
	std::size_t pos = 0;
	while ( (pos = raw_string.find(occupy_string,pos)) != std::string::npos ){
		// 替换
		raw_string.replace(pos,occupy_string.length(),replace_string);
		pos += replace_string.length();
	}
	// 返回修改后的raw_string
	return raw_string;
}

// 将输入的字符串中的"${uav_ns}"转译为输入的uav_ns
inline std::string replace_uav_ns(std::string input_string, const std::string &uav_ns){
	// 首先拉出字符串常量，"${uav_ns}”占位符
	const std::string uav_string = "${uav_ns}";
	// 如果存在uav_string的话，他在原字符串中的位置
	std::size_t pos = 0;
	// 首先判断两者都非空串
	if(input_string.empty() || uav_ns.empty())
	{
		return "";
	}
	// 是否有占位符
	if(input_string.find(uav_string) == std::string::npos)
	{
		// 没有发现占位符的话，返回原字符串
		return input_string;
	}
	// 存在占位符，进行转译
	// input_string.fing(uav_string,pos) 表示从pos位置开始查找，uav_string字串，如果查找到,则返回查找到的位置，如果没有查找到，会返回std::string::npos
	// 如果查找到一次，就使用replace方法，将
	while ((pos = input_string.find(uav_string,pos)) != std::string::npos) {
		// 从input.string的pos位置开始，将长度为uav_string字符串长度的内容，替换为uav_ns
		input_string.replace(pos, uav_string.length(), uav_ns);
		// 替换完成后，把 pos 向后移动到刚刚替换进去的新字符串后面
		pos += uav_ns.length();
	}	
	// 结束while循环，则表示查找替换完了所有的uav_string
	return input_string;
};

}; // namespace sunray_common
