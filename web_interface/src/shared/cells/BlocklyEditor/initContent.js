export const VARIABLES_TOOLBOX_CATEGORIES = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "基本指令",
      contents: [{ kind: "block", type: "base_variable" }],
    },
    {
      kind: "category",
      name: "结构体变量",
      contents: [
        { kind: "block", type: "load_variable" },
        { kind: "block", type: "jointtarget_variable" },
        { kind: "block", type: "robtarget_variable" },
        { kind: "block", type: "speed_variable" },
        { kind: "block", type: "zone_variable" },
      ],
    },
  ],
};
export const PROGRAM_TOOLBOX_CATEGORIES = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "数学指令",
      contents: [
        { kind: "block", type: "math_function" },
        { kind: "block", type: "math_opr" },
        { kind: "block", type: "logic_opr" },
        { kind: "block", type: "compare_opr" },
        { kind: "block", type: "number" },
      ],
    },
    {
      kind: "category",
      name: "逻辑指令",
      contents: [
        { kind: "block", type: "if" },
        { kind: "block", type: "while" },
        { kind: "block", type: "function" },
        { kind: "block", type: "main" },
        { kind: "block", type: "call" },
      ],
    },
    {
      kind: "category",
      name: "IO指令",
      contents: [
        { kind: "block", type: "sl" },
        { kind: "block", type: "setdo" },
        { kind: "block", type: "waitdi" },
      ],
    },
    {
      kind: "category",
      name: "运动指令",
      contents: [
        { kind: "block", type: "mvaj" },
        { kind: "block", type: "mvc" },
        { kind: "block", type: "mvl" },
        { kind: "block", type: "mvj" },
      ],
    },
    {
      kind: "category",
      name: "力控指令",
      contents: [
        { kind: "block", type: "FCPressP" },
        { kind: "block", type: "FCPressL" },
        { kind: "block", type: "FCPressS" },
        { kind: "block", type: "admitinit" },
        { kind: "block", type: "coroffset" },
        { kind: "block", type: "FCMonitor" },
        { kind: "block", type: "saveproxml" },
        { kind: "block", type: "loadproxml" },
      ],
    },
    {
      kind: "category",
      name: "SET指令",
      contents: [
        { kind: "block", type: "set_robtarget" },
        { kind: "block", type: "set_jointtarget" },
        { kind: "block", type: "set_speed" },
        { kind: "block", type: "set_zone" },
      ],
    },
  ],
};

const ConfigFiles = {
  VARIABLES_TOOLBOX_CATEGORIES,
  PROGRAM_TOOLBOX_CATEGORIES,
};

export default ConfigFiles;
