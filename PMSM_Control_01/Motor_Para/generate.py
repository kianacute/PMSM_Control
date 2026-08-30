#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 Motor_Para/Motor_Parameters.csv 生成查表头文件 Motor_Lookup_Tables_Float.h。

CSV 中存储的是**调谐参数**（如阻尼系数、带宽比），
脚本根据电机参数和公式计算出最终的 PI/PLL 增益值。

输出:
  Motor_Task/Control_Task/Float/Inc/Motor_Lookup_Tables_Float.h  （Keil 工程引用）
  Motor_Task/Control_Task/Float/Src/Motor_Lookup_Tables_Float.h  （同步副本）

电机参数/配置初始化已改为手写在 System/Src/Motor_Control.c，
脚本不再生成 Motor_Config.c。

用法: python generate.py [输出目录]
"""

import csv
import math
import os
import re
import sys
from collections import OrderedDict

# ─── 常量 ───────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
CSV_DIR = SCRIPT_DIR
MERGED_CSV_FILENAME = "Motor_Parameters.csv"
DEFAULT_OUTPUT_DIR = os.path.join(PROJECT_DIR, "Motor_Task", "Control_Task", "Float", "Inc")
SYNC_OUTPUT_DIR = os.path.join(PROJECT_DIR, "Motor_Task", "Control_Task", "Float", "Src")
OUTPUT_FILENAME = "Motor_Lookup_Tables_Float.h"

PI = math.pi

# C 标识符正则
_RE_C_IDENT = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


# ─── 公式 ───────────────────────────────────────────────────────────────

def elec_speed_rpm(rpm, pole_pairs):
    """转速 rpm → 电角速度 rad/s"""
    return rpm * 2.0 * PI / 60.0 * pole_pairs


def compute_nonflux_observer(params, rows):
    """
    非磁链观测器 PLL 参数计算。

    输入: speed_rpm, damping, cutoff_coeff, nonflux_gamma, efflux_gamma, is_index
    公式:
      we   = speed_rpm * 2*pi/60 * pole_pairs          (电角速度)
      wc   = cutoff_coeff * we                           (截止频率)
      Kp   = 2 * damping * wc                           (PLL比例增益)
      Ki   = wc^2 * Ts                                   (PLL积分增益)

    输出列:
      NonFlux_Lookup_Speed_index, NonFlux_PLL_Kp_Lookup_1D, NonFlux_PLL_Ki_Lookup_1D,
      NonFlux_Gama_Lookup_1D, EfFlux_Gama_Lookup_1D, NonFlux_Lookup_Is_index
    """
    Ts = 1.0 / params['current_loop_hz']
    pp = params['pole_pairs']

    output = OrderedDict()
    output['NonFlux_Lookup_Speed_index'] = []
    output['NonFlux_PLL_Kp_Lookup_1D'] = []
    output['NonFlux_PLL_Ki_Lookup_1D'] = []
    output['NonFlux_Gama_Lookup_1D'] = []
    output['EfFlux_Gama_Lookup_1D'] = []
    output['NonFlux_Lookup_Is_index'] = []

    for r in rows:
        rpm = float(r[0])
        damping = float(r[1])
        cutoff_coeff = float(r[2])
        nf_gamma = float(r[3])
        ef_gamma = float(r[4])
        is_idx = float(r[5])

        we = elec_speed_rpm(rpm, pp)
        wc = cutoff_coeff * we
        kp = 2.0 * damping * wc
        ki = wc * wc * Ts

        output['NonFlux_Lookup_Speed_index'].append(rpm)
        output['NonFlux_PLL_Kp_Lookup_1D'].append(kp)
        output['NonFlux_PLL_Ki_Lookup_1D'].append(ki)
        output['NonFlux_Gama_Lookup_1D'].append(nf_gamma)
        output['EfFlux_Gama_Lookup_1D'].append(ef_gamma)
        output['NonFlux_Lookup_Is_index'].append(is_idx)

    return output


def compute_smo_observer(params, rows):
    """
    滑模观测器 PLL 参数计算。

    输入: speed_rpm, damping, cutoff_coeff, smo_gain
    公式:
      we      = speed_rpm * 2*pi/60 * pole_pairs
      wc      = cutoff_coeff * we
      back_emf = flux_rpm_per_v * speed_rpm / 1000     (反电动势 V)
      Kp      = 2 * damping * wc / back_emf             (PLL比例增益, 归一化到反电动势)
      Ki      = wc^2 * Ts / back_emf                    (PLL积分增益)

    输出列:
      SMO_Lookup_Speed_index, SMO_PLL_Kp_Lookup_1D, SMO_PLL_Ki_Lookup_1D,
      SMO_Gain_Lookup_1D
    """
    Ts = 1.0 / params['current_loop_hz']
    pp = params['pole_pairs']
    ke = params['flux_rpm_per_v']  # V/(krpm)

    output = OrderedDict()
    output['SMO_Lookup_Speed_index'] = []
    output['SMO_PLL_Kp_Lookup_1D'] = []
    output['SMO_PLL_Ki_Lookup_1D'] = []
    output['SMO_Gain_Lookup_1D'] = []

    for r in rows:
        rpm = float(r[0])
        damping = float(r[1])
        cutoff_coeff = float(r[2])
        smo_gain = float(r[3])

        we = elec_speed_rpm(rpm, pp)
        wc = cutoff_coeff * we
        back_emf = ke * rpm / 1000.0

        kp = 2.0 * damping * wc / back_emf
        ki = wc * wc * Ts / back_emf

        output['SMO_Lookup_Speed_index'].append(rpm)
        output['SMO_PLL_Kp_Lookup_1D'].append(kp)
        output['SMO_PLL_Ki_Lookup_1D'].append(ki)
        output['SMO_Gain_Lookup_1D'].append(smo_gain)

    return output


def compute_current_loop(params, rows):
    """
    电流环 PI 参数计算。

    输入: speed_rpm, bandwidth_ratio
    公式:
      wc     = 2*pi * f_pwm / bandwidth_ratio           (电流环带宽 rad/s)
      Id_Kp  = Ld * wc                                   (D轴比例增益)
      Id_Ki  = Rs * wc * Ts                              (D轴积分增益)
      Iq_Kp  = Lq * wc                                   (Q轴比例增益)
      Iq_Ki  = Rs * wc * Ts                              (Q轴积分增益, 与D轴相同)

    输出列:
      Current_Lookup_Speed_index, Current_ID_PI_Kp_Lookup_1D,
      Current_ID_PI_Ki_Lookup_1D, Current_IQ_PI_Kp_Lookup_1D,
      Current_IQ_PI_Ki_Lookup_1D
    """
    f_pwm = params['current_loop_hz']
    Ts = 1.0 / f_pwm
    Rs = params['Rs']
    Ld = params['Ld']
    Lq = params['Lq']

    output = OrderedDict()
    output['Current_Lookup_Speed_index'] = []
    output['Current_ID_PI_Kp_Lookup_1D'] = []
    output['Current_ID_PI_Ki_Lookup_1D'] = []
    output['Current_IQ_PI_Kp_Lookup_1D'] = []
    output['Current_IQ_PI_Ki_Lookup_1D'] = []

    for r in rows:
        rpm = float(r[0])
        bw_ratio = float(r[1])

        wc = 2.0 * PI * f_pwm / bw_ratio

        id_kp = Ld * wc
        id_ki = Rs * wc * Ts
        iq_kp = Lq * wc
        iq_ki = Rs * wc * Ts

        output['Current_Lookup_Speed_index'].append(rpm)
        output['Current_ID_PI_Kp_Lookup_1D'].append(id_kp)
        output['Current_ID_PI_Ki_Lookup_1D'].append(id_ki)
        output['Current_IQ_PI_Kp_Lookup_1D'].append(iq_kp)
        output['Current_IQ_PI_Ki_Lookup_1D'].append(iq_ki)

    return output


def compute_speed_loop(params, rows):
    """
    速度环 PI 参数（经验调谐值，非公式计算）。

    输入: speed_rpm, speed_kp, speed_ki  (直接存储调谐后的值)

    输出列:
      Speed_Loop_Speed_Index, Speed_Loop_Speed_PI_Kp_1D, Speed_Loop_Speed_PI_Ki_1D
    """
    output = OrderedDict()
    output['Speed_Loop_Speed_Index'] = []
    output['Speed_Loop_Speed_PI_Kp_1D'] = []
    output['Speed_Loop_Speed_PI_Ki_1D'] = []

    for r in rows:
        output['Speed_Loop_Speed_Index'].append(float(r[0]))
        output['Speed_Loop_Speed_PI_Kp_1D'].append(float(r[1]))
        output['Speed_Loop_Speed_PI_Ki_1D'].append(float(r[2]))

    return output


# 区块名 → (计算函数, 预期的调谐参数列数)
MODULE_PIPELINE = OrderedDict({
    'NonFluxObserver': (compute_nonflux_observer, 6),
    'SMOObserver':     (compute_smo_observer,     4),
    'CurrentLoop':     (compute_current_loop,     2),
})

RAW_MODULES = {
    'SpeedLoop':  (compute_speed_loop, 3),
    'IFStartup':  (None, 3),  # 无公式, 直接用 read_raw_1d
}

# ─── 工具函数 ──────────────────────────────────────────────────────────

def fmt_float(v):
    """数值 → C float 字面量"""
    if isinstance(v, int) or (isinstance(v, float) and v == int(v)):
        return f"{float(v):.1f}f"
    s = f"{v:.8g}"
    if 'e' in s or 'E' in s:
        return s + 'f'
    if '.' in s:
        return s + 'f'
    return s + '.0f'


def check_duplicates(values, label):
    """检查重复值并警告"""
    seen = set()
    for v in values:
        if v in seen:
            print(f"  [WARN] duplicate X value '{v}' in '{label}'")
        seen.add(v)


# ─── CSV 读取 ──────────────────────────────────────────────────────────

def read_csv_text(path):
    """读取 CSV 文本，自动识别编码：UTF-8(-BOM) 优先，失败回退 GBK（Excel ANSI 另存）。"""
    with open(path, 'rb') as f:
        raw = f.read()
    for enc in ('utf-8-sig', 'gbk'):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode('utf-8-sig')


def read_merged_csv(path):
    """读取合并 CSV → OrderedDict {区块名: rows}。

    区块以 "#Module: <名称>" 行分隔；空行与 "#" 注释行跳过；
    "#2D ..." 行保留（2D 表标记，属于区块内容）。
    编码: 自动识别 UTF-8/GBK，中文注释不依赖系统区域设置。
    容错: 忽略每行尾部的空列（Excel 保存 CSV 时会补齐逗号）。
    """
    modules = OrderedDict()
    current = None
    text = read_csv_text(path)
    for row in csv.reader(text.splitlines()):
        if not row or all(not c.strip() for c in row):
            continue
        # 去掉行尾空列（Excel 补逗号产生）
        while row and not row[-1].strip():
            row.pop()
        if not row:
            continue
        first = row[0].strip()
        if first.startswith('#Module:'):
            current = first[len('#Module:'):].strip()
            if not current:
                raise ValueError(f"Empty #Module marker in '{path}'")
            modules[current] = []
            continue
        # 注释行（#2D 除外，它是 2D 表标记，属于区块内容）
        if first.startswith('#') and not first.startswith('#2D '):
            continue
        if current is None:
            raise ValueError(f"Data row before first #Module marker in '{path}'")
        modules[current].append(row)
    if not modules:
        raise ValueError(f"No #Module sections found in '{path}'")
    return modules


def read_motor_base(rows, label):
    """读取 MotorBase 区块 → dict"""
    params = {}
    for row in rows[1:]:
        if len(row) < 2:
            continue
        name = row[0].strip()
        if not name:
            continue
        params[name] = float(row[1].strip())
    return params


def read_raw_1d(rows, label):
    """读取无公式的 1D 区块 → OrderedDict {col_name: [values]}"""
    headers = [h.strip() for h in rows[0]]
    for h in headers:
        if not _RE_C_IDENT.match(h):
            raise ValueError(f"'{h}' is not a valid C identifier in '{label}'")

    data = OrderedDict()
    for h in headers:
        data[h] = []

    for i, row in enumerate(rows[1:], 2):
        if len(row) < len(headers):
            raise ValueError(f"Row {i}: expected {len(headers)} cols, got {len(row)} in '{label}'")
        for j, h in enumerate(headers):
            data[h].append(float(row[j].strip()))

    check_duplicates(data[headers[0]], f"{label} -> {headers[0]}")
    return data


def read_tuning_params(rows, expected_cols, label):
    """读取调谐参数区块 → list of lists (仅数据行，无表头)"""
    headers = [h.strip() for h in rows[0]]
    if len(headers) != expected_cols:
        raise ValueError(
            f"'{label}': expected {expected_cols} tuning columns, got {len(headers)}. "
            f"Headers: {headers}"
        )
    data_rows = []
    for i, row in enumerate(rows[1:], 2):
        if len(row) < expected_cols:
            raise ValueError(f"Row {i}: expected {expected_cols} cols, got {len(row)} in '{label}'")
        data_rows.append(row)
    return data_rows


def read_2d_table(rows, label):
    """读取 2D 查表区块 → (var_name, [[float]])"""
    marker = rows[0][0].strip()
    var_name = marker[4:].strip()
    if not _RE_C_IDENT.match(var_name):
        raise ValueError(f"'{var_name}' invalid C identifier for 2D table in '{label}'")

    col_headers = []
    for c in rows[1][1:]:
        val = c.strip()
        if val:
            col_headers.append(float(val))
    if not col_headers:
        raise ValueError(f"No column headers in '{label}'")

    matrix = []
    for i, row in enumerate(rows[2:], 3):
        if len(row) < len(col_headers) + 1:
            raise ValueError(f"Row {i}: insufficient columns in '{label}'")
        matrix.append([float(c) for c in row[1:1 + len(col_headers)]])

    if not matrix:
        raise ValueError(f"No data rows in '{label}'")

    return var_name, matrix


# ─── C 代码生成 ────────────────────────────────────────────────────────

# 期望生成的查表（用于验证 CSV 是否齐全）
EXPECTED_1D = {
    # IF 启动
    "IF_Start_Ramp_Sec", "IF_Start_Speed_RPM", "IF_Start_Iq_A",
    # 电流环
    "Current_Lookup_Speed_index",
    "Current_ID_PI_Kp_Lookup_1D", "Current_ID_PI_Ki_Lookup_1D",
    "Current_IQ_PI_Kp_Lookup_1D", "Current_IQ_PI_Ki_Lookup_1D",
    # 速度环
    "Speed_Loop_Speed_Index",
    "Speed_Loop_Speed_PI_Kp_1D", "Speed_Loop_Speed_PI_Ki_1D",
    # 非磁链观测器
    "NonFlux_Lookup_Speed_index", "NonFlux_Lookup_Is_index",
    "NonFlux_PLL_Kp_Lookup_1D", "NonFlux_PLL_Ki_Lookup_1D",
    "NonFlux_Gama_Lookup_1D", "EfFlux_Gama_Lookup_1D",
    # SMO 观测器
    "SMO_Lookup_Speed_index",
    "SMO_PLL_Kp_Lookup_1D", "SMO_PLL_Ki_Lookup_1D", "SMO_Gain_Lookup_1D",
}

EXPECTED_2D = {
    "EFFlux_Angle_Comp_table_2D",
}

# MotorBase 参数名 → (C 宏名, 结构体字段名)
# current_loop_hz 仅用于脚本内计算，不生成；rs_identified 结构体无字段，不生成
SCALAR_MACROS = OrderedDict([
    ("pole_pairs",      ("MOTOR_POLE_PAIRS",       "pole_pairs")),
    ("max_current_a",   ("MOTOR_MAX_CURRENT_A",    "max_current_a")),
    ("voltage_limit_v", ("MOTOR_VOLTAGE_LIMIT_V",  "voltage_limit_v")),
    ("flux_rpm_per_v",  ("MOTOR_FLUX_RPM_PER_V",   "flux_rpm_per_v")),
    ("Rs",              ("MOTOR_RS",               "Rs")),
    ("Ld",              ("MOTOR_LD",               "Ld")),
    ("Lq",              ("MOTOR_LQ",               "Lq")),
    ("power_limit_w",   ("MOTOR_POWER_LIMIT",      "Power_Limit")),
])


def gen_motor_param_macros(motor_params):
    """生成电机基本参数宏定义（数值只出现在此头文件中，保持纯 ASCII）"""
    lines = ['// Motor base parameters (from MotorBase section, values generated by generate.py)']
    for csv_name, (macro, field) in SCALAR_MACROS.items():
        if csv_name not in motor_params:
            print(f"  [WARN] '{csv_name}' missing in MotorBase, macro {macro} not generated")
            continue
        lines.append(f'#define {macro:<26} ({fmt_float(motor_params[csv_name])})')
    return lines


# 生成头文件的包含守卫，须与 Motor_Control.h 中定义的宏一致
HEADER_GUARD = "MOTOR_LOOKUP_TABLES_FLOAT"


def gen_lookup_tables_h(motor_params, all_1d, all_2d):
    """生成 Motor_Lookup_Tables_Float.h（查表 + 电机基本参数）"""
    lines = [
        '// Auto-generated by generate.py from Motor_Para/Motor_Parameters.csv',
        '// DO NOT EDIT manually.',
        '',
        f'#ifdef {HEADER_GUARD}',
        '',
    ]

    # 电机基本参数宏
    lines.extend(gen_motor_param_macros(motor_params))
    lines.append('')

    for var_name in sorted(all_1d.keys()):
        values = all_1d[var_name]
        count = len(values)
        fmt_lines = []
        for i in range(0, count, 6):
            chunk = values[i:i + 6]
            fmt_lines.append('    ' + ', '.join(fmt_float(v) for v in chunk) + ',')
        lines.append(f'const float {var_name}[{count}] = {{')
        lines.extend(fmt_lines)
        lines.append('};')
        lines.append('')

    for var_name in sorted(all_2d.keys()):
        values = all_2d[var_name]
        rows = len(values)
        cols = len(values[0])
        fmt_rows = []
        for row in values:
            fmt_vals = ', '.join(fmt_float(v) for v in row)
            fmt_rows.append(f'    {{{fmt_vals}}},')
        lines.append(f'const float {var_name}[{rows}][{cols}] = {{')
        lines.extend(fmt_rows)
        lines.append('};')
        lines.append('')

    lines.append('#endif')
    return '\n'.join(lines) + '\n'


# ─── 主流程 ────────────────────────────────────────────────────────────

def extract_all(merged_path):
    """读取合并 CSV 的各区块 → (motor_params, all_1d_arrays, all_2d_arrays)"""
    motor_params = {}
    all_1d = OrderedDict()
    all_2d = OrderedDict()

    modules = read_merged_csv(merged_path)

    # MotorBase 必须最先处理（后续计算依赖它）
    if 'MotorBase' in modules:
        modules.move_to_end('MotorBase', last=False)

    for name, rows in modules.items():

        # MotorBase
        if name == 'MotorBase':
            motor_params = read_motor_base(rows, name)
            print(f'  {name}: {len(motor_params)} params')
            for k, v in motor_params.items():
                print(f'    {k} = {v}')
            continue

        # 2D 表
        if rows and rows[0][0].strip().startswith('#2D '):
            var_name, matrix = read_2d_table(rows, name)
            all_2d[var_name] = matrix
            print(f'  {name}: 2D table -> {var_name}[{len(matrix)}][{len(matrix[0])}]')
            continue

        # 公式计算模块
        if name in MODULE_PIPELINE:
            compute_fn, expected_cols = MODULE_PIPELINE[name]
            tuning_rows = read_tuning_params(rows, expected_cols, name)
            result = compute_fn(motor_params, tuning_rows)
            print(f'  {name}: {len(tuning_rows)} rows -> {len(result)} output vars')
            for var, vals in result.items():
                print(f'    {var}[{len(vals)}]')
                all_1d[var] = vals
            continue

        # 无公式模块（直接读取数组值）
        if name in RAW_MODULES:
            compute_fn, expected_cols = RAW_MODULES[name]
            if compute_fn is not None:
                result = compute_fn(motor_params, read_tuning_params(rows, expected_cols, name))
            else:
                result = read_raw_1d(rows, name)
            print(f'  {name}: {len(result)} vars (raw values)')
            for var, vals in result.items():
                print(f'    {var}[{len(vals)}]')
                all_1d[var] = vals
            continue

        # 回退: 作为无公式 1D 表处理
        data = read_raw_1d(rows, name)
        print(f'  {name}: {len(data)} vars x {len(list(data.values())[0])} rows (raw)')
        for var, vals in data.items():
            print(f'    {var}[{len(vals)}]')
            all_1d[var] = vals

    return motor_params, all_1d, all_2d


def main():
    if sys.stdout.encoding != 'utf-8':
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')

    if len(sys.argv) > 1:
        output_dirs = [sys.argv[1]]
    else:
        output_dirs = [DEFAULT_OUTPUT_DIR, SYNC_OUTPUT_DIR]
    for d in output_dirs:
        os.makedirs(d, exist_ok=True)

    merged_path = os.path.join(CSV_DIR, MERGED_CSV_FILENAME)
    print(f"CSV 文件: {merged_path}")
    print(f"输出目录: {', '.join(output_dirs)}")
    print()

    motor_params, all_1d, all_2d = extract_all(merged_path)
    print()

    if not all_1d and not all_2d:
        print("Error: no lookup tables extracted")
        sys.exit(1)

    # 验证期望查表是否齐全
    for var_name in sorted(EXPECTED_1D):
        if var_name not in all_1d:
            print(f"  [WARN] '{var_name}' expected but not found in any CSV")
    for var_name in sorted(EXPECTED_2D):
        if var_name not in all_2d:
            print(f"  [WARN] '{var_name}' expected but not found in any CSV")

    # 检查并警告 X 轴重复
    x_columns = [
        ('CurrentLookup', 'Current_Lookup_Speed_index'),
        ('SpeedLoop', 'Speed_Loop_Speed_Index'),
        ('NonFluxObserver', 'NonFlux_Lookup_Speed_index'),
        ('SMOObserver', 'SMO_Lookup_Speed_index'),
    ]
    for label, var_name in x_columns:
        if var_name in all_1d:
            check_duplicates(all_1d[var_name], label)

    # 生成查表头文件
    content = gen_lookup_tables_h(motor_params, all_1d, all_2d)
    for output_dir in output_dirs:
        lookup_path = os.path.join(output_dir, OUTPUT_FILENAME)
        with open(lookup_path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(content)
        print(f'Generated: {lookup_path}')


if __name__ == '__main__':
    main()
