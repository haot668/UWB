#!/usr/bin/env python3
"""Generate 3 Keil project files from the original template."""
import os
import re

BASE = os.path.dirname(os.path.abspath(__file__))
SRC  = os.path.join(BASE, "API")

def read_xml(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

def write_xml(path, content):
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)

# Read original
original = read_xml(os.path.join(SRC, "DWS3000_STM32L412.uvprojx"))

# New shared include paths
NEW_INCLUDE = (
    "..\\shared\\Inc;"
    "..\\shared\\Drivers\\CMSIS\\Include;"
    "..\\shared\\Drivers\\CMSIS\\Device\\ST\\STM32L4xx\\Include;"
    "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Inc;"
    "..\\shared\\decadriver;"
    "..\\shared\\platform;"
    "..\\shared\\MAC_802_15_4;"
    "..\\shared\\MAC_802_15_8;"
    "..\\shared\\shared_data;"
    "..\\shared\\examples_info;"
    "..\\shared\\config;"
    "..\\shared\\fusion_anchor;"
    "..\\shared\\fusion_tag;"
    "..\\shared\\system;"
    ".\\Src"
)

# Old C compiler include paths (to be replaced entirely)
OLD_CADS_INCLUDE = (
    ".\\Inc;"
    ".\\Src;"
    ".\\Src\\platform;"
    ".\\Src\\decadriver;"
    ".\\Src\\examples\\examples_info;"
    ".\\Src\\examples\\shared_data;"
    ".\\Src\\examples\\ex_10a_downlink_tag;"
    ".\\Src\\examples\\ex_10b_downlink_anchor;"
    ".\\Drivers\\CMSIS\\Include;"
    ".\\Drivers\\CMSIS\\Device\\ST\\STM32L4xx\\Include;"
    ".\\Drivers\\STM32L4xx_HAL_Driver\\Inc"
)

# --- Project-specific configurations ---
projects = {
    "uwb_anchor": {
        "target_name": "DWS3000_Anchor",
        "output_name": "DWS3000_Anchor",
        "defines":     "USE_HAL_DRIVER, STM32L412xx, TEST_FUSION_ANCHOR",
        "scatter":     ".\\DWS3000_Anchor.sct",
        "replace_source_paths": {
            "main.c":                        ".\\Src\\main.c",
            "stm32l4xx_hal_msp.c":           ".\\Src\\stm32l4xx_hal_msp.c",
            "stm32l4xx_it.c":                ".\\Src\\stm32l4xx_it.c",
            "system_stm32l4xx.c":            "..\\shared\\system\\system_stm32l4xx.c",
            "config_options.c":              "..\\shared\\config\\config_options.c",
            "port.c":                        "..\\shared\\platform\\port.c",
            "deca_spi.c":                    "..\\shared\\platform\\deca_spi.c",
            "deca_mutex.c":                  "..\\shared\\platform\\deca_mutex.c",
            "deca_sleep.c":                  "..\\shared\\platform\\deca_sleep.c",
            "deca_device.c":                 "..\\shared\\decadriver\\deca_device.c",
            "mac_802_15_4.c":                "..\\shared\\MAC_802_15_4\\mac_802_15_4.c",
            "mac_802_15_8.c":                "..\\shared\\MAC_802_15_8\\mac_802_15_8.c",
            "example_info.c":                "..\\shared\\examples_info\\example_info.c",
            "shared_functions.c":            "..\\shared\\shared_data\\shared_functions.c",
            "fusion_anchor.c":               "..\\shared\\fusion_anchor\\fusion_anchor.c",
            "startup_stm32l412xx.s":         "..\\shared\\startup\\startup_stm32l412xx.s",
            # HAL driver files
            "stm32l4xx_hal.c":              "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal.c",
            "stm32l4xx_hal_cortex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_cortex.c",
            "stm32l4xx_hal_dma.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_dma.c",
            "stm32l4xx_hal_exti.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_exti.c",
            "stm32l4xx_hal_flash.c":         "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash.c",
            "stm32l4xx_hal_flash_ex.c":      "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash_ex.c",
            "stm32l4xx_hal_gpio.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_gpio.c",
            "stm32l4xx_hal_pwr.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr.c",
            "stm32l4xx_hal_pwr_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr_ex.c",
            "stm32l4xx_hal_rcc.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc.c",
            "stm32l4xx_hal_rcc_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc_ex.c",
            "stm32l4xx_hal_spi.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_spi.c",
            "stm32l4xx_hal_tim.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim.c",
            "stm32l4xx_hal_tim_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim_ex.c",
            "stm32l4xx_hal_uart.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart.c",
        },
        "remove_files": ["gateway_cmd.c"],
    },
    "uwb_tag": {
        "target_name": "DWS3000_Tag",
        "output_name": "DWS3000_Tag",
        "defines":     "USE_HAL_DRIVER, STM32L412xx, TEST_FUSION_TAG",
        "scatter":     ".\\DWS3000_Tag.sct",
        "replace_source_paths": {
            "main.c":                        ".\\Src\\main.c",
            "stm32l4xx_hal_msp.c":           ".\\Src\\stm32l4xx_hal_msp.c",
            "stm32l4xx_it.c":                ".\\Src\\stm32l4xx_it.c",
            "system_stm32l4xx.c":            "..\\shared\\system\\system_stm32l4xx.c",
            "config_options.c":              "..\\shared\\config\\config_options.c",
            "port.c":                        "..\\shared\\platform\\port.c",
            "deca_spi.c":                    "..\\shared\\platform\\deca_spi.c",
            "deca_mutex.c":                  "..\\shared\\platform\\deca_mutex.c",
            "deca_sleep.c":                  "..\\shared\\platform\\deca_sleep.c",
            "deca_device.c":                 "..\\shared\\decadriver\\deca_device.c",
            "mac_802_15_4.c":                "..\\shared\\MAC_802_15_4\\mac_802_15_4.c",
            "mac_802_15_8.c":                "..\\shared\\MAC_802_15_8\\mac_802_15_8.c",
            "example_info.c":                "..\\shared\\examples_info\\example_info.c",
            "shared_functions.c":            "..\\shared\\shared_data\\shared_functions.c",
            "startup_stm32l412xx.s":         "..\\shared\\startup\\startup_stm32l412xx.s",
            # HAL driver files
            "stm32l4xx_hal.c":              "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal.c",
            "stm32l4xx_hal_cortex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_cortex.c",
            "stm32l4xx_hal_dma.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_dma.c",
            "stm32l4xx_hal_exti.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_exti.c",
            "stm32l4xx_hal_flash.c":         "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash.c",
            "stm32l4xx_hal_flash_ex.c":      "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash_ex.c",
            "stm32l4xx_hal_gpio.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_gpio.c",
            "stm32l4xx_hal_pwr.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr.c",
            "stm32l4xx_hal_pwr_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr_ex.c",
            "stm32l4xx_hal_rcc.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc.c",
            "stm32l4xx_hal_rcc_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc_ex.c",
            "stm32l4xx_hal_spi.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_spi.c",
            "stm32l4xx_hal_tim.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim.c",
            "stm32l4xx_hal_tim_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim_ex.c",
            "stm32l4xx_hal_uart.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart.c",
        },
        "remove_files": ["fusion_anchor.c", "gateway_cmd.c"],
        # Tag needs fusion_tag.c added as a new file in the Examples group
        "add_files_to_group": {
            "Examples": [
                '            <File>\n'
                '              <FileName>fusion_tag.c</FileName>\n'
                '              <FileType>1</FileType>\n'
                '              <FilePath>..\\shared\\fusion_tag\\fusion_tag.c</FilePath>\n'
                '            </File>\n'
            ],
        },
    },
    "uwb_gateway": {
        "target_name": "DWS3000_Gateway",
        "output_name": "DWS3000_Gateway",
        "defines":     "USE_HAL_DRIVER, STM32L412xx, TEST_FUSION_ANCHOR, ENABLE_GATEWAY",
        "scatter":     ".\\DWS3000_Gateway.sct",
        "replace_source_paths": {
            "main.c":                        ".\\Src\\main.c",
            "stm32l4xx_hal_msp.c":           ".\\Src\\stm32l4xx_hal_msp.c",
            "stm32l4xx_it.c":                ".\\Src\\stm32l4xx_it.c",
            "gateway_cmd.c":                 ".\\Src\\gateway_cmd.c",
            "system_stm32l4xx.c":            "..\\shared\\system\\system_stm32l4xx.c",
            "config_options.c":              "..\\shared\\config\\config_options.c",
            "port.c":                        "..\\shared\\platform\\port.c",
            "deca_spi.c":                    "..\\shared\\platform\\deca_spi.c",
            "deca_mutex.c":                  "..\\shared\\platform\\deca_mutex.c",
            "deca_sleep.c":                  "..\\shared\\platform\\deca_sleep.c",
            "deca_device.c":                 "..\\shared\\decadriver\\deca_device.c",
            "mac_802_15_4.c":                "..\\shared\\MAC_802_15_4\\mac_802_15_4.c",
            "mac_802_15_8.c":                "..\\shared\\MAC_802_15_8\\mac_802_15_8.c",
            "example_info.c":                "..\\shared\\examples_info\\example_info.c",
            "shared_functions.c":            "..\\shared\\shared_data\\shared_functions.c",
            "fusion_anchor.c":               "..\\shared\\fusion_anchor\\fusion_anchor.c",
            "startup_stm32l412xx.s":         "..\\shared\\startup\\startup_stm32l412xx.s",
            # HAL driver files
            "stm32l4xx_hal.c":              "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal.c",
            "stm32l4xx_hal_cortex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_cortex.c",
            "stm32l4xx_hal_dma.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_dma.c",
            "stm32l4xx_hal_exti.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_exti.c",
            "stm32l4xx_hal_flash.c":         "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash.c",
            "stm32l4xx_hal_flash_ex.c":      "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_flash_ex.c",
            "stm32l4xx_hal_gpio.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_gpio.c",
            "stm32l4xx_hal_pwr.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr.c",
            "stm32l4xx_hal_pwr_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_pwr_ex.c",
            "stm32l4xx_hal_rcc.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc.c",
            "stm32l4xx_hal_rcc_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_rcc_ex.c",
            "stm32l4xx_hal_spi.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_spi.c",
            "stm32l4xx_hal_tim.c":           "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim.c",
            "stm32l4xx_hal_tim_ex.c":        "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_tim_ex.c",
            "stm32l4xx_hal_uart.c":          "..\\shared\\Drivers\\STM32L4xx_HAL_Driver\\Src\\stm32l4xx_hal_uart.c",
        },
        "remove_files": [],
    },
}

for proj_dir, cfg in projects.items():
    proj = original

    # Replace target name
    proj = proj.replace("<TargetName>DWS3000_STM32L412</TargetName>",
                         f"<TargetName>{cfg['target_name']}</TargetName>")
    proj = proj.replace("<OutputName>DWS3000_STM32L412</OutputName>",
                         f"<OutputName>{cfg['output_name']}</OutputName>")
    proj = proj.replace("<LayName>DWS3000_STM32L412</LayName>",
                         f"<LayName>{cfg['target_name']}</LayName>")

    # Replace defines
    proj = proj.replace("<Define>USE_HAL_DRIVER, STM32L412xx</Define>",
                         f"<Define>{cfg['defines']}</Define>")

    # Replace scatter file
    proj = proj.replace(".\\DWS3000_STM32L412.sct", cfg['scatter'])

    # --- Fix IncludePaths correctly ---
    # 1. TargetCommonOption: replace empty <IncludePath></IncludePath> with new path
    proj = proj.replace(
        "<IncludePath></IncludePath>",
        f"<IncludePath>{NEW_INCLUDE}</IncludePath>"
    )
    # 2. Cads: replace old C compiler include paths with new ones
    proj = proj.replace(
        f"<IncludePath>{OLD_CADS_INCLUDE}</IncludePath>",
        f"<IncludePath>{NEW_INCLUDE}</IncludePath>"
    )

    # Replace all source file paths
    for filename, new_path in cfg['replace_source_paths'].items():
        escaped_path = new_path.replace('\\', '\\\\')
        pattern = re.compile(
            r'(<FileName>' + re.escape(filename) + r'</FileName>\s*<FileType>\d+</FileType>\s*)'
            r'<FilePath>[^<]*</FilePath>',
            re.MULTILINE
        )
        proj = pattern.sub(r'\1<FilePath>' + escaped_path + r'</FilePath>', proj)

    # Remove excluded files (entire <File> blocks)
    for fname in cfg['remove_files']:
        pattern = re.compile(
            r'\s*<File>\s*<FileName>' + re.escape(fname) +
            r'</FileName>.*?</File>',
            re.DOTALL
        )
        proj = pattern.sub('', proj)

    # Add new files to groups (e.g. fusion_tag.c for tag project)
    if 'add_files_to_group' in cfg:
        for group_name, file_blocks in cfg['add_files_to_group'].items():
            for file_block in file_blocks:
                # Find the group's <Files> tag and insert new file block after it
                group_tag = f"<GroupName>{group_name}</GroupName>"
                idx = proj.find(group_tag)
                if idx != -1:
                    # Find the <Files> start tag after this group name
                    files_idx = proj.find("<Files>", idx)
                    if files_idx != -1:
                        # Insert right after <Files>\n
                        insert_pos = files_idx + len("<Files>")
                        proj = proj[:insert_pos] + "\n" + file_block + proj[insert_pos:]

    # Write project file
    out_path = os.path.join(BASE, proj_dir, f"{cfg['target_name']}.uvprojx")
    write_xml(out_path, proj)
    print(f"Created: {out_path}")

    # Also copy uvoptx (debug settings) from original
    src_opt = os.path.join(SRC, "DWS3000_STM32L412.uvoptx")
    dst_opt = os.path.join(BASE, proj_dir, f"{cfg['target_name']}.uvoptx")
    if os.path.exists(src_opt):
        opt_content = read_xml(src_opt)
        opt_content = opt_content.replace("DWS3000_STM32L412", cfg['target_name'])
        write_xml(dst_opt, opt_content)
        print(f"Created: {dst_opt}")

print("\nDone! 3 projects generated.")
