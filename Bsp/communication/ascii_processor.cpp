/*
* ASCII 协议是主本地协议的一种简化版，具有可读性，便于人类理解。
* 未来此协议可能会扩展以支持选择性的 GCode 命令。
* 支持的命令列表请参见 doc/ascii-protocol.md 文件。
*/

/* 包含的头文件 ------------------------------------------------------------------*/

#include "common_inc.h"        // 包含公共的头文件
#include "ascii_processor.hpp" // 包含 ASCII 协议处理相关头文件

/* 私有宏定义 -----------------------------------------------------------*/
#define MAX_LINE_LENGTH 256  // 定义最大行长度为 256 字节
#define TO_STR_INNER(s) #s   // 宏：将传入的参数转为字符串
#define TO_STR(s) TO_STR_INNER(s) // 宏：将传入的参数转为字符串并调用内部宏

/* 私有类型定义 -----------------------------------------------------------*/
/* 全局常量数据 ---------------------------------------------------------*/
/* 全局变量 -------------------------------------------------------------*/
/* 私有常量数据 ----------------------------------------------------------*/
/* 私有变量 -------------------------------------------------------------*/
/* 私有函数原型声明 -------------------------------------------------------*/
/* 函数实现 --------------------------------------------------------------*/


// @brief 执行一个 ASCII 协议命令
// @param buffer 包含 ASCII 编码字符的缓冲区
// @param len 缓冲区的大小
// @param response_channel 响应通道，用于返回处理结果
void ASCII_protocol_process_line(const uint8_t* buffer, size_t len, StreamSink &response_channel)
{
    static_assert(sizeof(char) == sizeof(uint8_t));  // 确保 char 类型与 uint8_t 类型大小相同

    // 将所有数据复制到本地缓冲区，以便插入 null 终止符
    char cmd[MAX_LINE_LENGTH + 1];
    if (len > MAX_LINE_LENGTH) len = MAX_LINE_LENGTH;  // 如果命令长度大于最大行长度，截断为最大行长度
    memcpy(cmd, buffer, len);  // 将输入的命令数据复制到本地缓冲区

    cmd[len] = 0;  // 为命令添加 null 终止符

    // 根据响应通道的类型，调用不同的命令处理函数
    if (response_channel.channelType == StreamSink::CHANNEL_TYPE_USB)
        OnUsbAsciiCmd(cmd, len, response_channel);  // 处理 USB 通道的 ASCII 命令
    else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART4)
        OnUart4AsciiCmd(cmd, len, response_channel);  // 处理 UART4 通道的 ASCII 命令
    else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART5)
        OnUart5AsciiCmd(cmd, len, response_channel);  // 处理 UART5 通道的 ASCII 命令
}

// @brief 解析 ASCII 协议流并处理每一行命令
// @param buffer 包含 ASCII 编码字符的缓冲区
// @param len 缓冲区的大小
// @param response_channel 响应通道，用于返回处理结果
void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink &response_channel)
{
    static uint8_t parse_buffer[MAX_LINE_LENGTH];  // 解析缓冲区，用于存储每一行的命令
    static bool read_active = true;  // 读取状态，表示是否处于有效的命令读取状态
    static uint32_t parse_buffer_idx = 0;  // 解析缓冲区的索引

    while (len--)
    {
        // 如果命令行过长，重置缓冲区并等待下一行命令
        if (parse_buffer_idx >= MAX_LINE_LENGTH)
        {
            read_active = false;  // 停止读取
            parse_buffer_idx = 0;  // 重置索引
        }

        // 获取下一个字符
        uint8_t c = *(buffer++);
        bool is_end_of_line = (c == '\r' || c == '\n');  // 判断当前字符是否是行结束符（回车或换行）

        if (is_end_of_line)
        {
            // 如果当前行有效，处理该行命令
            if (read_active)
                ASCII_protocol_process_line(parse_buffer, parse_buffer_idx, response_channel);
            parse_buffer_idx = 0;  // 重置缓冲区索引，准备接收下一行命令
            read_active = true;  // 恢复读取状态
        } else
        {
            // 如果当前字符不是行结束符且处于有效状态，将字符添加到解析缓冲区
            if (read_active)
            {
                parse_buffer[parse_buffer_idx++] = c;  // 将字符存入缓冲区并递增索引
            }
        }
    }
}
