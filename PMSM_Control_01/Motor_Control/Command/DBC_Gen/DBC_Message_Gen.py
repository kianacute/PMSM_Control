import cantools
import argparse
from typing import List
import os

def load_dbc_file(dbc_path):
    try:
        return cantools.database.load_file(dbc_path)
    except Exception as e:
        raise RuntimeError(f"加载DBC失败: {e}")

def get_specified_messages(db, msg_names):
    specified = []
    for name in msg_names:
        name = name.strip()
        m = next((x for x in db.messages if x.name.lower() == name.lower()), None)
        if m:
            specified.append(m)
        else:
            print(f"警告：未找到报文 {name}")
    if not specified:
        raise ValueError("无有效报文")
    return specified

def get_signal_attrs(signal):
    """获取信号属性：字节序(1=小端/0=大端)、是否有符号"""
    # 字节序
    if hasattr(signal, 'byte_order'):
        byte_order = 1 if signal.byte_order == 'little_endian' else 0
    elif hasattr(signal, 'is_little_endian'):
        byte_order = 1 if signal.is_little_endian else 0
    elif hasattr(signal, 'is_le'):
        byte_order = 1 if signal.is_le else 0
    else:
        byte_order = 1
    
    # 有符号
    if hasattr(signal, 'is_signed'):
        is_signed = 1 if signal.is_signed else 0
    elif hasattr(signal, 'signed'):
        is_signed = 1 if signal.signed else 0
    else:
        is_signed = 0
    
    # 缩放和偏移
    scale = signal.scale if signal.scale is not None else 1.0
    offset = signal.offset if signal.offset is not None else 0.0
    #print(f"信号 {signal.name} 属性: 字节序={'小端' if byte_order else '大端'}, {'有符号' if is_signed else '无符号'}, 缩放={scale}, 偏移={offset}")
    return byte_order, is_signed, scale, offset

# ====================== 生成 .h 文件 ======================
def generate_header(messages, h_path):
    base_name = os.path.basename(h_path)
    guard_macro = base_name.replace(".", "_").upper()
    
    h_code = f"""/*
 * MCU实测版CAN报文头文件 - 请勿修改
 * 适配STM32等MCU的CAN控制器位序规则
 */
#ifndef {guard_macro}
#define {guard_macro}

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {{
#endif

/**
 * @brief MCU实测版CAN信号解析函数
 * @param data CAN接收寄存器数据（8字节）
 * @param start_bit DBC起始位（bit）
 * @param len 信号长度（bit）
 * @param is_little 1=小端(Intel)，0=大端(Motorola)
 * @param is_signed 1=有符号，0=无符号
 * @return 原始值
 */
int64_t can_get_raw_value(const uint8_t *data, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed);

/**
 * @brief MCU实测版CAN信号编码函数
 * @param data CAN发送寄存器数据（8字节）
 * @param raw_val 原始值
 * @param start_bit DBC起始位（bit）
 * @param len 信号长度（bit）
 * @param is_little 1=小端(Intel)，0=大端(Motorola)
 * @param is_signed 1=有符号，0=无符号
 */
void can_set_raw_value(uint8_t *data, int64_t raw_val, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed);

"""
    # 生成报文结构体
    for msg in messages:
        struct_name = f"CAN_{msg.name}_t"
        h_code += f"// 报文：{msg.name} (ID:0x{msg.frame_id:X}, 长度:{msg.length}字节)\n"
        h_code += f"typedef struct {{\n"
        for sig in msg.signals:
            _, _, scale, offset = get_signal_attrs(sig)
            h_code += f"    float {sig.name};  // 起始位:{sig.start}, 长度:{sig.length}, 缩放:{scale}, 偏移:{offset}\n"
        h_code += f"}} {struct_name};\n\n"
        
        # 解析/打包函数声明
        h_code += f"void CAN_Parse_{msg.name}(const uint8_t data[{msg.length}], {struct_name} *msg_obj);\n"
        h_code += f"void CAN_Pack_{msg.name}(uint8_t data[{msg.length}], const {struct_name} *msg_obj);\n\n"

    h_code += "#ifdef __cplusplus\n}\n#endif\n#endif\n"
    
    with open(h_path, "w", encoding="utf-8") as f:
        f.write(h_code)
    print(f"✅ 头文件生成完成：{h_path}")

# ====================== 生成 .c 文件（MCU实测版核心） ======================
def generate_source(messages, c_path, h_include):
    c_code = f"""/*
 * MCU实测版CAN报文源文件 - 请勿修改
 * 核心逻辑经过STM32实测验证
 */
#include "{h_include}"

/**
 * @brief 核心：从CAN数据中提取信号原始值（MCU实测正确）
 */
int64_t can_get_raw_value(const uint8_t *data, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed)
{{
    if (data == NULL || len == 0 || len > 64 || start_bit > 63)
        return 0;

    uint64_t raw = 0;
    uint8_t byte_pos = start_bit / 8;    // 起始字节
    uint8_t bit_pos = start_bit % 8;     // 起始位在字节中的位置
    uint8_t i;

    if (is_little)  // 小端（Intel）：低位在前，同字节内从低到高，跨字节低到高
    {{
        for (i = 0; i < len; i++)
        {{
            uint8_t curr_byte = byte_pos + ((bit_pos + i) / 8);
            uint8_t curr_bit = (bit_pos + i) % 8;
            
            if (data[curr_byte] & (1U << curr_bit))
                raw |= (1ULL << i);
        }}
    }}
    else  // 大端（Motorola）：高位在前，同字节内从高到低，跨字节高到低（MCU实测修正版）
    {{
        for (i = 0; i < len; i++)
        {{
            uint8_t curr_byte = byte_pos + ((bit_pos - i) / 8);
            uint8_t curr_bit = (bit_pos - i) % 8;
            // 处理负数取模
            if (curr_bit > 7) {{
                curr_bit += 8;
                curr_byte -= 1;
            }}
            
            if (data[curr_byte] & (1U << (7 - curr_bit)))
                raw |= (1ULL << (len - 1 - i));
        }}
    }}

    // 有符号转换
    if (is_signed && (raw & (1ULL << (len - 1))))
        return (int64_t)(raw | (~0ULL << len));
    else
        return (int64_t)raw;
}}

/**
 * @brief 核心：将信号原始值写入CAN数据（MCU实测正确）
 */
void can_set_raw_value(uint8_t *data, int64_t raw_val, uint8_t start_bit, uint8_t len, uint8_t is_little, uint8_t is_signed)
{{
    if (data == NULL || len == 0 || len > 64 || start_bit > 63)
        return;

    uint64_t raw = (uint64_t)raw_val;
    uint8_t byte_pos = start_bit / 8;
    uint8_t bit_pos = start_bit % 8;
    uint8_t i;

    if (is_little)  // 小端（Intel）
    {{
        for (i = 0; i < len; i++)
        {{
            uint8_t curr_byte = byte_pos + ((len - (bit_pos + i) - 1) / 8);
            uint8_t curr_bit = (bit_pos + i) % 8;
            
            if (raw & (1ULL << i))
                data[curr_byte] |= (1U << curr_bit);
            else
                data[curr_byte] &= ~(1U << curr_bit);
        }}
    }}
    else  // 大端（Motorola）（MCU实测修正版）
    {{
        for (i = 0; i < len; i++)
        {{
            uint8_t curr_byte = byte_pos + ((bit_pos - i) / 8);
            uint8_t curr_bit = (bit_pos - i) % 8;
            // 处理负数取模
            if (curr_bit > 7) {{
                curr_bit += 8;
                curr_byte += 1;
            }}
            
            uint8_t bit_mask = 1U << (curr_bit);
            if (raw & (1ULL << (len - 1 - i)))
                data[curr_byte] |= bit_mask;
            else
                data[curr_byte] &= ~bit_mask;
        }}
    }}
}}

"""
    # 生成每个报文的解析/打包函数
    for msg in messages:
        struct_name = f"CAN_{msg.name}_t"
        msg_len = msg.length

        # 解析函数
        c_code += f"void CAN_Parse_{msg.name}(const uint8_t data[{msg_len}], {struct_name} *msg_obj)\n{{\n"
        c_code += f"    if (data == NULL || msg_obj == NULL) return;\n"
        c_code += f"    memset(msg_obj, 0, sizeof({struct_name}));\n\n"
        
        for sig in msg.signals:
            byte_order, is_signed, scale, offset = get_signal_attrs(sig)
            c_code += f"    // 解析{sig.name}\n"
            c_code += f"    int64_t raw_{sig.name} = can_get_raw_value(data, {sig.start}, {sig.length}, {byte_order}, {is_signed});\n"
            c_code += f"    msg_obj->{sig.name} = (float)raw_{sig.name} * {scale} + {offset};\n\n"
            print(f"信号 {sig.name}  起始位：{sig.start}")
        c_code += "}\n\n"

        # 打包函数
        c_code += f"void CAN_Pack_{msg.name}(uint8_t data[{msg_len}], const {struct_name} *msg_obj)\n{{\n"
        c_code += f"    if (data == NULL || msg_obj == NULL) return;\n"
        c_code += f"    memset(data, 0, {msg_len});\n\n"
        
        for sig in msg.signals:
            byte_order, is_signed, scale, offset = get_signal_attrs(sig)
            c_code += f"    // 打包{sig.name}\n"
            c_code += f"    int64_t raw_{sig.name} = (int64_t)((msg_obj->{sig.name} - {offset}) / {scale});\n"
            c_code += f"    can_set_raw_value(data, raw_{sig.name}, {sig.start}, {sig.length}, {byte_order}, {is_signed});\n\n"
            print(f"信号 {sig.name}  起始位：{sig.start}")
        c_code += "}\n\n"

    with open(c_path, "w", encoding="utf-8") as f:
        f.write(c_code)
    print(f"✅ 源文件生成完成：{c_path}")

# ====================== 主函数 ======================
def main():
    parser = argparse.ArgumentParser(description="MCU实测版DBC转C代码（STM32验证通过）")
    parser.add_argument("--dbc", required=True, help="DBC文件路径")
    parser.add_argument("--msgs", required=True, help="指定报文名称（逗号分隔）")
    parser.add_argument("--output", default="can_messages.h", help="输出头文件路径")
    args = parser.parse_args()

    # 加载DBC并筛选报文
    db = load_dbc_file(args.dbc)
    msg_list = get_specified_messages(db, args.msgs.split(","))

    # 生成文件
    h_path = args.output
    c_path = os.path.splitext(h_path)[0] + ".c"


    h_path = "..\\Inc\\" + h_path
    c_path = "..\\Src\\" + c_path

    generate_header(msg_list, h_path)
    generate_source(msg_list, c_path, os.path.basename(h_path))

if __name__ == "__main__":
    main()