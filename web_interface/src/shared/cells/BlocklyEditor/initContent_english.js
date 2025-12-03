// import { getlocalestate } from "../../../selectors";
// import store from '../../../store'

// store.subscribe(() => {
//   let state = store.getState();   //这就是你获取到的数据state tree，由于使用了subscribe，当数据更改时会重新获取
//   console.log("initcontent中的state是：",state)
// });

export const VARIABLES_TOOLBOX_CATEGORIES_ENGLISH = [
  {
    name: "基本指令",
    blocks: [
      {
        type: "base_variate",
        fields: {
          NAME: "VAR",
          DROPDOWN1: "int",
          INPUT1: "p1",
          RESULT: "result",
        },
      },
    ],
  },
  {
    name: "结构体变量",
    blocks: [
      {
        type: "load_variate",
        fields: {
          VARIABLE: "piece1",
          MASS: "mass",
          COGX: "cogx",
          COGY: "cogy",
          COGZ: "cogz",
          Q1: "q1",
          Q2: "q2",
          Q3: "q3",
          Q4: "q4",
          IX: "ix",
          IY: "iy",
          IZ: "iz",
        },
      },
      // {
      //   type: 'pose_variate',
      //   fields: {
      //     pose1: 'pose1',
      //     X: 'x',
      //     Y: 'y',
      //     Z: 'z',
      //     Q1: 'q1',
      //     Q2: 'q2',
      //     Q3: 'q3',
      //     Q4: 'q4',
      //   }
      // },
      {
        type: "jointtarget_variate",
        fields: {
          HOME: "home",
          J1: "J1",
          J2: "J2",
          J3: "J3",
          J4: "J4",
          J5: "J5",
          J6: "J6",
          EJ: "EJ",
        },
      },
      {
        type: "robtarget_variate",
        fields: {
          INPUT1: "input1",
          X: "x",
          Y: "y",
          Z: "z",
          Q1: "q1",
          Q2: "q2",
          Q3: "q3",
          Q4: "q4",
        },
      },
      {
        type: "speed_variate_define",
        fields: {},
      },
      {
        type: "zone_variate",
        fields: {
          INPUT1: "z100",
          DIS: "dis",
          PER: "per",
        },
      },
    ],
  },
];
export const PROGRAM_TOOLBOX_CATEGORIES_ENGLISH = [
  {
    name: "数学指令",
    blocks: [
      {
        type: "math_function",
        fields: {
          FROPDOWN1: "sin",
          INPUT1: "default",
        },
      },
      {
        type: "simple_arithmetic",
        fields: {
          VALUE1: "value1",
          NAME: "sum",
          VALUE2: "value2",
        },
      },
      { type: "wait", fields: { NUM: 0 } },
      {
        type: "logic_and_or",
        fields: {
          DROPDOWN: "NAME_AND",
        },
      },
      {
        type: "comparison_operator",
        fields: {
          DROPDOWN: "NAME_GREATER",
        },
      },
      {
        type: "INPUT_NUMBER",
        fields: {
          NAME_NUMBER: "number",
        },
      },
    ],
  },
  {
    name: "逻辑指令",
    blocks: [
      { type: "controls_if_self" },
      {
        type: "function_block",
        fields: {
          FUNCTION_NAME: "default",
        },
      },
      {
        type: "func_main_block",
      },
      // {
      //   type: "func_return_block",
      //   fields:{
      //     FUNCTION_NAME:"default",
      //   }
      // },
      {
        type: "while_block",
      },
      {
        type: "call_function",
      },
    ],
  },
  {
    name: "IO指令",
    blocks: [
      {
        type: "wait_until",
      },
      {
        type: "setdo",
        fields: {
          INPUT1: "default",
          DROPDOWN1: "True",
        },
      },
    ],
  },
  {
    name: "运动指令",
    blocks: [
      {
        type: "moveabsj",
        fields: {
          DROPDOWN1: "ToJointPos",
          DROPDOWN2: "Speed",
          DROPDOWN3: "Zone",
          DROPDOWN4: "Tool",
          DROPDOWN5: "Wobj",
        },
      },

      // {
      //   type: 'moveabsj_1',
      //   fields:{
      //     CHOOSE: 'ToJointPos',
      //     DROPDOWN2: 'Speed',
      //     DROPDOWN3: 'Zone',
      //   }
      // },

      {
        type: "movec",
        fields: {
          DROPDOWN1: "AuxPoint",
          DROPDOWN2: "ToPoint",
          DROPDOWN3: "Speed",
          DROPDOWN4: "Zone",
          DROPDOWN5: "Tool",
          DROPDOWN6: "Wobj",
        },
      },
      {
        type: "robotcontrol_movel",
        fields: {},
      },
      {
        type: "robotcontrol_movej",
        fields: {},
      },
    ],
  },
  {
    name: "力控指令",
    blocks: [
      {
        type: "fcpressp",
        fields: {
          NAME: "name",
          P1: "p1",
          P2: "p2",
          P3: "p3",
          P4: "p4",
          P5: "p5",
          DROPDOWN1: "Force",
          DROPDOWN2: "Tool",
          DROPDOWN5: "Wobj",
        },
      },
      {
        type: "fcpressl",
        fields: {
          FCPressL: "FCPressL",
          NAME: "name",
          DROPDOWN1: "Force",
          DROPDOWN2: "Tool",
          DROPDOWN3: "Wobj",
        },
      },
      {
        type: "fcpresss",
        fields: {
          FCPressS: "FCPressS",
          NAME: "name",
          DROPDOWN1: "Force",
          DROPDOWN2: "Tool",
          DROPDOWN3: "Wobj",
        },
      },
    ],
  },
  {
    name: "SET指令",
    blocks: [
      {
        type: "speed_variate",
        fields: {
          V1: "v1",
          TER: "ter",
          TCP: "tcp",
          ORI: "ori",
          EXJ_R: "exj_r",
          EXJ_L: "exj_l",
        },
      },
      {
        type: "tool_variate",
        fields: {
          INPUT1: "tool2",
          CHOOSE: "true",
          X: "x",
          Y: "y",
          Z: "z",
          A1: "q1",
          A2: "q2",
          A3: "q3",
          A4: "q4",
          MASS: "mass",
          COGX: "cogx",
          COGY: "cogy",
          COGZ: "cogz",
          Q1: "q1",
          Q2: "q2",
          Q3: "q3",
          Q4: "q4",
          IX: "ix",
          IY: "iy",
          IZ: "iz",
        },
      },
      {
        type: "wobj_variate",
        fields: {
          VALUE: "wobj2",
          DROPDOWN1: "FALSE",
          DROPDOWN2: "TRUE",
          NAME_STRING: "robot",
          X: "x",
          Y: "y",
          Z: "z",
          Q1: "q1",
          Q2: "q2",
          Q3: "q3",
          Q4: "q4",
          ID: "id",
        },
      },
      {
        type: "set_zone_variate",
        fields: {
          // NAME_NUMBER: "number",
        },
      },
      {
        type: "set_function",
        fields: {
          // NAME_NUMBER: "number",
        },
      },
    ],
  },
];

const ConfigFiles = {
  VARIABLES_TOOLBOX_CATEGORIES_ENGLISH,
  PROGRAM_TOOLBOX_CATEGORIES_ENGLISH,
};

export default ConfigFiles;
