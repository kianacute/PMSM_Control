#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 Motor_Para/*.csv 生成 Motor_Lookup_Tables.c 和 Motor_Config.c。

CSV 中存储的是**调谐参数**（如阻尼系数、带宽比），
脚本根据电机参数和公式计算出最终的 PI/PLL 增益值。

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
DEFAULT_OUTPUT_DIR = os.path.join(PROJECT_DIR, "Motor_Control", "Motor_Config", "Src")

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


# 模块名 → (计算函数, 预期的调谐参数列数)
MODULE_PIPELINE = OrderedDict({
    'NonFluxObserver.csv': (compute_nonflux_observer, 6),
    'SMOObserver.csv':     (compute_smo_observer,     4),
    'CurrentLoop.csv':     (compute_current_loop,     2),
})

RAW_MODULES = {
    'SpeedLoop.csv':  (compute_speed_loop, 3),
    'IFStartup.csv':  (None, 3),  # 无公式, 直接用 read_raw_1d
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

def read_csv(path):
    """读取 CSV，返回 (headers, data_rows)。跳过空行和注释行。"""
    with open(path, 'r', encoding='utf-8-sig', newline='') as f:
        reader = csv.reader(f)
        rows = []
        for row in reader:
            if not row or all(not c.strip() for c in row):
                continue
            if row[0].strip().startswith('#') and not row[0].strip().startswith('#2D '):
                continue
            rows.append(row)
    if not rows:
        raise ValueError(f"CSV file is empty: {path}")
    return rows


def read_motor_base(path):
    """读取 MotorBase.csv → dict"""
    rows = read_csv(path)
    params = {}
    for row in rows[1:]:
        if len(row) < 2:
            continue
        name = row[0].strip()
        if not name:
            continue
        params[name] = float(row[1].strip())
    return params


def read_raw_1d(path):
    """读取无公式的 1D CSV → OrderedDict {col_name: [values]}"""
    rows = read_csv(path)
    headers = [h.strip() for h in rows[0]]
    for h in headers:
        if not _RE_C_IDENT.match(h):
            raise ValueError(f"'{h}' is not a valid C identifier in '{path}'")

    data = OrderedDict()
    for h in headers:
        data[h] = []

    for i, row in enumerate(rows[1:], 2):
        if len(row) < len(headers):
            raise ValueError(f"Row {i}: expected {len(headers)} cols, got {len(row)} in '{path}'")
        for j, h in enumerate(headers):
            data[h].append(float(row[j].strip()))

    check_duplicates(data[headers[0]], f"{path} -> {headers[0]}")
    return data


def read_tuning_params(path, expected_cols):
    """读取调谐参数 CSV → list of lists (仅数据行，无表头)"""
    rows = read_csv(path)
    headers = [h.strip() for h in rows[0]]
    if len(headers) != expected_cols:
        raise ValueError(
            f"'{path}': expected {expected_cols} tuning columns, got {len(headers)}. "
            f"Headers: {headers}"
        )
    data_rows = []
    for i, row in enumerate(rows[1:], 2):
        if len(row) < expected_cols:
            raise ValueError(f"Row {i}: expected {expected_cols} cols, got {len(row)} in '{path}'")
        data_rows.append(row)
    return data_rows


def read_2d_table(path):
    """读取 2D 查表 CSV → (var_name, [[float]])"""
    rows = read_csv(path)
    marker = rows[0][0].strip()
    var_name = marker[4:].strip()
    if not _RE_C_IDENT.match(var_name):
        raise ValueError(f"'{var_name}' invalid C identifier for 2D table in '{path}'")

    col_headers = []
    for c in rows[1][1:]:
        val = c.strip()
        if val:
            col_headers.append(float(val))
    if not col_headers:
        raise ValueError(f"No column headers in '{path}'")

    matrix = []
    for i, row in enumerate(rows[2:], 3):
        if len(row) < len(col_headers) + 1:
            raise ValueError(f"Row {i}: insufficient columns in '{path}'")
        matrix.append([float(c) for c in row[1:1 + len(col_headers)]])

    if not matrix:
        raise ValueError(f"No data rows in '{path}'")

    return var_name, matrix


# ─── C 代码生成 ────────────────────────────────────────────────────────

# 1D 查表 Y 变量名 → (Motor_Config 字段名, X 轴变量名)
TABLE_BINDINGS = OrderedDict({
    # IF 启动
    "IF_Start_Speed_RPM":     ("IF_Start_Speed_Lookup",  "IF_Start_Ramp_Sec"),
    "IF_Start_Iq_A":          ("IF_Start_Iq_Lookup",     "IF_Start_Ramp_Sec"),
    # 电流环
    "Current_ID_PI_Kp_Lookup_1D": ("ID_PI_Kp_Lookup",   "Current_Lookup_Speed_index"),
    "Current_IQ_PI_Kp_Lookup_1D": ("IQ_PI_Kp_Lookup",   "Current_Lookup_Speed_index"),
    "Current_ID_PI_Ki_Lookup_1D": ("ID_PI_Ki_Lookup",   "Current_Lookup_Speed_index"),
    "Current_IQ_PI_Ki_Lookup_1D": ("IQ_PI_Ki_Lookup",   "Current_Lookup_Speed_index"),
    # 速度环
    "Speed_Loop_Speed_PI_Kp_1D": ("Speed_PI_Kp_Lookup", "Speed_Loop_Speed_Index"),
    "Speed_Loop_Speed_PI_Ki_1D": ("Speed_PI_Ki_Lookup", "Speed_Loop_Speed_Index"),
    # 非磁链观测器
    "NonFlux_PLL_Kp_Lookup_1D": ("NonFlux_PLL_Kp_Lookup", "NonFlux_Lookup_Speed_index"),
    "NonFlux_PLL_Ki_Lookup_1D": ("NonFlux_PLL_Ki_Lookup", "NonFlux_Lookup_Speed_index"),
    "NonFlux_Gama_Lookup_1D":   ("NonFlux_Gama_Lookup",   "NonFlux_Lookup_Speed_index"),
    "EfFlux_Gama_Lookup_1D":    ("EfFlux_Gama_Lookup",    "NonFlux_Lookup_Speed_index"),
    "NonFlux_Lookup_Is_index":  ("EfFlux_Angle_Comp",     None),
    # SMO 观测器
    "SMO_PLL_Kp_Lookup_1D":     ("SMO_PLL_Kp_Lookup",    "SMO_Lookup_Speed_index"),
    "SMO_PLL_Ki_Lookup_1D":     ("SMO_PLL_Ki_Lookup",    "SMO_Lookup_Speed_index"),
    "SMO_Gain_Lookup_1D":       ("SMO_Gain_Lookup",      "SMO_Lookup_Speed_index"),
})

TABLE_2D_BINDINGS = OrderedDict({
    "EFFlux_Angle_Comp_table_2D": {
        "field":   "EfFlux_Angle_Comp",
        "x_table": "NonFlux_Lookup_Speed_index",
        "y_table": "NonFlux_Lookup_Is_index",
    },
})

SCALAR_FIELD_MAP = {
    "pole_pairs":      "pole_pairs",
    "max_current_a":   "max_current_a",
    "voltage_limit_v": "voltage_limit_v",
    "flux_rpm_per_v":  "flux_rpm_per_v",
    "Rs":              "Rs",
    "Ld":              "Ld",
    "Lq":              "Lq",
    "power_limit_w":   "Power_Limit",
    "rs_identified":   "rs_identified",
}

SCALAR_TYPE_OVERRIDES = {
    "rs_identified": ("uint8_t", lambda v: str(int(float(v)))),
}


def gen_lookup_tables_c(all_1d, all_2d):
    """生成 Motor_Lookup_Tables.c"""
    lines = [
        '// Auto-generated by generate.py from Motor_Para/*.csv',
        '// DO NOT EDIT manually.',
        '',
        '#ifdef MOTOR_CONFIG_H',
        '',
    ]

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


def gen_motor_config_c(motor_params, all_1d, all_2d):
    """生成 Motor_Config.c"""
    lines = [
        '// Auto-generated by generate.py from Motor_Para/*.csv',
        '// DO NOT EDIT manually.',
        '',
        '#include "Motor_Config.h"',
        '',
        '#include "Motor_Lookup_Tables.c"',
        '',
        '',
        'Motor_Parameter_t PMSM_42JS_Parameter;',
        'Motor_Config_t PMSM_42JS_Config;',
        '',
        'void Motor_Parameter_Init(void)',
        '{',
    ]

    for csv_name, field_name in SCALAR_FIELD_MAP.items():
        if csv_name not in motor_params:
            continue
        val = motor_params[csv_name]
        if csv_name in SCALAR_TYPE_OVERRIDES:
            _, fmt_fn = SCALAR_TYPE_OVERRIDES[csv_name]
            lines.append(f'    PMSM_42JS_Parameter.{field_name} = {fmt_fn(val)};')
        else:
            lines.append(f'    PMSM_42JS_Parameter.{field_name} = {fmt_float(val)};')

    lines.extend([
        '    PMSM_42JS_Parameter.Ls = (PMSM_42JS_Parameter.Ld + PMSM_42JS_Parameter.Lq) / 2;',
        '    PMSM_42JS_Parameter.flux_linkage_wb = (PMSM_42JS_Parameter.flux_rpm_per_v / PMSM_42JS_Parameter.pole_pairs',
        '                         / 100.0f / PI * 3.0f);',
        '    /*磁链计算参考文章：https://www.zhihu.com/question/606311981/answer/3091158625 */',
        '    PMSM_42JS_Parameter.Flux_Flux  = PMSM_42JS_Parameter.flux_linkage_wb * PMSM_42JS_Parameter.flux_linkage_wb;',
        '    PMSM_42JS_Parameter.Ld_Lq = PMSM_42JS_Parameter.Ld - PMSM_42JS_Parameter.Lq;',
        '}',
        '',
        'void Motor_Config_Init(void)',
        '{',
        '    Motor_Parameter_Init();',
        '    PMSM_42JS_Config.motor_param = &PMSM_42JS_Parameter;',
    ])

    sections = [
        ("IF启动参数",    ["IF_Start_Iq_A", "IF_Start_Speed_RPM"]),
        ("电流环参数",    ["Current_ID_PI_Kp_Lookup_1D", "Current_ID_PI_Ki_Lookup_1D",
                           "Current_IQ_PI_Kp_Lookup_1D", "Current_IQ_PI_Ki_Lookup_1D"]),
        ("速度环参数",    ["Speed_Loop_Speed_PI_Kp_1D", "Speed_Loop_Speed_PI_Ki_1D"]),
        ("磁链观测器参数", ["NonFlux_PLL_Kp_Lookup_1D", "NonFlux_PLL_Ki_Lookup_1D",
                           "NonFlux_Gama_Lookup_1D", "EfFlux_Gama_Lookup_1D",
                           "NonFlux_Lookup_Is_index"]),
        ("SMO观测器参数",  ["SMO_PLL_Kp_Lookup_1D", "SMO_PLL_Ki_Lookup_1D",
                           "SMO_Gain_Lookup_1D"]),
    ]

    for section_name, y_vars in sections:
        lines.append('')
        lines.append(f'    // {section_name}查表初始化')
        for y_var in y_vars:
            if y_var not in TABLE_BINDINGS or y_var not in all_1d:
                continue
            field_name, x_var = TABLE_BINDINGS[y_var]
            if x_var is None:
                continue
            lines.append(f'    PMSM_42JS_Config.{field_name}.x_table = {x_var};')
            lines.append(f'    PMSM_42JS_Config.{field_name}.y_table = {y_var};')
            lines.append(f'    PMSM_42JS_Config.{field_name}.table_size = sizeof({y_var}) / sizeof(float);')

    for var_name, binding in TABLE_2D_BINDINGS.items():
        if var_name not in all_2d:
            continue
        lines.append('')
        lines.append(f'    // 角度补偿2D查表初始化')
        lines.append(f'    PMSM_42JS_Config.{binding["field"]}.x_table = {binding["x_table"]};')
        lines.append(f'    PMSM_42JS_Config.{binding["field"]}.y_table = {binding["y_table"]};')
        lines.append(f'    PMSM_42JS_Config.{binding["field"]}.z_table = (const float *){var_name};')
        lines.append(f'    PMSM_42JS_Config.{binding["field"]}.nx_size = sizeof({binding["x_table"]}) / sizeof(float);')
        lines.append(f'    PMSM_42JS_Config.{binding["field"]}.ny_size = sizeof({binding["y_table"]}) / sizeof(float);')

    lines.append('}')
    return '\n'.join(lines) + '\n'


# ─── 主流程 ────────────────────────────────────────────────────────────

def extract_all(csv_dir):
    """读取所有 CSV → (motor_params, all_1d_arrays, all_2d_arrays)"""
    motor_params = {}
    all_1d = OrderedDict()
    all_2d = OrderedDict()

    csv_files = sorted(f for f in os.listdir(csv_dir) if f.endswith('.csv') and not f.startswith('~'))

    # MotorBase.csv 必须最先处理
    if 'MotorBase.csv' in csv_files:
        csv_files.remove('MotorBase.csv')
        csv_files.insert(0, 'MotorBase.csv')

    for filename in csv_files:
        path = os.path.join(csv_dir, filename)

        # MotorBase
        if filename == 'MotorBase.csv':
            motor_params = read_motor_base(path)
            print(f'  {filename}: {len(motor_params)} params')
            for k, v in motor_params.items():
                print(f'    {k} = {v}')
            continue

        # 2D 表
        with open(path, 'r', encoding='utf-8-sig', newline='') as f:
            first = next(csv.reader(f), [])
        if first and first[0].strip().startswith('#2D '):
            var_name, matrix = read_2d_table(path)
            all_2d[var_name] = matrix
            print(f'  {filename}: 2D table -> {var_name}[{len(matrix)}][{len(matrix[0])}]')
            continue

        # 公式计算模块
        if filename in MODULE_PIPELINE:
            compute_fn, expected_cols = MODULE_PIPELINE[filename]
            tuning_rows = read_tuning_params(path, expected_cols)
            result = compute_fn(motor_params, tuning_rows)
            print(f'  {filename}: {len(tuning_rows)} rows -> {len(result)} output vars')
            for name, vals in result.items():
                print(f'    {name}[{len(vals)}]')
                all_1d[name] = vals
            continue

        # 无公式模块（直接读取数组值）
        if filename in RAW_MODULES:
            compute_fn, expected_cols = RAW_MODULES[filename]
            if compute_fn is not None:
                result = compute_fn(motor_params, read_tuning_params(path, expected_cols))
            else:
                result = read_raw_1d(path)
            print(f'  {filename}: {len(result)} vars (raw values)')
            for name, vals in result.items():
                print(f'    {name}[{len(vals)}]')
                all_1d[name] = vals
            continue

        # 回退: 作为无公式 1D 表处理
        data = read_raw_1d(path)
        print(f'  {filename}: {len(data)} vars x {len(list(data.values())[0])} rows (raw)')
        for name, vals in data.items():
            print(f'    {name}[{len(vals)}]')
            all_1d[name] = vals

    return motor_params, all_1d, all_2d


def main():
    if sys.stdout.encoding != 'utf-8':
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')

    output_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)

    print(f"CSV 目录: {CSV_DIR}")
    print(f"输出目录: {output_dir}")
    print()

    motor_params, all_1d, all_2d = extract_all(CSV_DIR)
    print()

    if not all_1d and not all_2d:
        print("Error: no lookup tables extracted")
        sys.exit(1)

    # 验证 binding
    for y_var in TABLE_BINDINGS:
        if y_var not in all_1d:
            print(f"  [WARN] '{y_var}' in TABLE_BINDINGS but not found in any CSV")
    for var_name in TABLE_2D_BINDINGS:
        if var_name not in all_2d:
            print(f"  [WARN] '{var_name}' in TABLE_2D_BINDINGS but not found in any CSV")

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

    # 生成
    lookup_path = os.path.join(output_dir, 'Motor_Lookup_Tables.c')
    with open(lookup_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(gen_lookup_tables_c(all_1d, all_2d))
    print(f'Generated: {lookup_path}')

    config_path = os.path.join(output_dir, 'Motor_Config.c')
    with open(config_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(gen_motor_config_c(motor_params, all_1d, all_2d))
    print(f'Generated: {config_path}')


if __name__ == '__main__':
    main()
