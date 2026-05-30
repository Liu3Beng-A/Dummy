#ifndef __ASCII_PROTOCOL_H
#define __ASCII_PROTOCOL_H

/* 包含的头文件 ------------------------------------------------------------------*/
#include <fibre/protocol.hpp>   // 包含 Fibre 协议相关的头文件（假设用于处理通信协议）
#include <stdlib.h>              // 包含标准库头文件，提供常用功能，如内存管理、字符串处理等
#include <stdint.h>              // 包含标准整数类型定义头文件（如 uint8_t, uint32_t 等）
#include <stdbool.h>             // 包含布尔类型定义头文件（如 bool 类型）

/* 导出的类型 ------------------------------------------------------------*/
/* 导出的常量 ------------------------------------------------------------*/
/* 导出的变量 ------------------------------------------------------------*/
/* 导出的宏 ------------------------------------------------------------*/
/* 导出的函数 ------------------------------------------------------------*/

/* 导出的函数原型 --------------------------------------------------------*/
// @brief 解析 ASCII 协议流并处理每一行命令
// @param buffer 包含 ASCII 编码字符的缓冲区
// @param len 缓冲区的大小
// @param response_channel 响应通道，用于返回处理结果
void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink& response_channel);

// @brief 处理 USB 通道的 ASCII 命令
// @param _cmd ASCII 命令
// @param _len 命令长度
// @param _responseChannel 响应通道
void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);

// @brief 处理 UART4 通道的 ASCII 命令
// @param _cmd ASCII 命令
// @param _len 命令长度
// @param _responseChannel 响应通道
void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);

// @brief 处理 UART5 通道的 ASCII 命令
// @param _cmd ASCII 命令
// @param _len 命令长度
// @param _responseChannel 响应通道
void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);

// @brief 通过特定的通道（如 UART 或 USB-VCP）发送消息
// 使用此函数代替 printf，因为 printf 会通过所有通道发送消息
// @param output 响应输出通道（UART 或 USB-VCP）
// @param fmt 格式化字符串
// @param args 格式化字符串的参数（变参模板）
template<typename ... TArgs>
void Respond(StreamSink &output, const char *fmt, TArgs &&... args)
{
    char response[64];  // 定义一个字符数组用于存储格式化后的响应消息
    size_t len = snprintf(response, sizeof(response), fmt, std::forward<TArgs>(args)...);  // 格式化字符串，存入 response 中
    output.process_bytes((uint8_t *) response, len, nullptr);  // 通过输出通道发送格式化后的响应消息
    output.process_bytes((const uint8_t *) "\r\n", 2, nullptr);  // 发送换行符
}

#endif /* __ASCII_PROTOCOL_H */
