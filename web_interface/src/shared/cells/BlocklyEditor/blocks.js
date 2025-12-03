import Blockly from "blockly";
import store from "../../../index";

let robtargetNumber = 1;
const validateP = function (newValue) {
  const value = newValue.split("p");
  // 名字数值自增
  if (
    !!Number(value[value.length - 1]) &&
    Number(value[value.length - 1]) >= robtargetNumber
  ) {
    robtargetNumber = Number(value[value.length - 1]) + 1;
  }
  const reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
  return reg.test(newValue) ? newValue : "error";
};

let jointtargetNumber = 1;
const validateJ = function (newValue) {
  const value = newValue.split("j");
  // 名字数值自增
  if (
    !!Number(value[value.length - 1]) &&
    Number(value[value.length - 1]) >= jointtargetNumber
  ) {
    jointtargetNumber = Number(value[value.length - 1]) + 1;
  }
  const reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
  return reg.test(newValue) ? newValue : "error";
};

let speedNumber = 1;
const validateV = function (newValue) {
  const value = newValue.split("v");
  // 名字数值自增
  if (
    !!Number(value[value.length - 1]) &&
    Number(value[value.length - 1]) >= speedNumber
  ) {
    speedNumber = Number(value[value.length - 1]) + 1;
  }
  const reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
  return reg.test(newValue) ? newValue : "error";
};

let zoneNumber = 1;
const validateZ = function (newValue) {
  const value = newValue.split("z");
  // 名字数值自增
  if (
    !!Number(value[value.length - 1]) &&
    Number(value[value.length - 1]) >= zoneNumber
  ) {
    zoneNumber = Number(value[value.length - 1]) + 1;
  }
  const reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
  return reg.test(newValue) ? newValue : "error";
};

function popupFunction(block) {
  console.log("popup:", block);

  const xml = Blockly.Xml.blockToDom(block);

  console.log("xml:", xml);
  console.log("block.workspace:", block.workspace);
  console.log(block.workspace.dspCopyBlock);
  block.workspace.dspCopyBlock(xml);
}

const toolDropdown = [
  ["tool0", "tool0"],
  ["tool1", "tool1"],
  ["tool2", "tool2"],
  ["tool3", "tool3"],
  ["tool4", "tool4"],
  ["tool5", "tool5"],
  ["tool6", "tool6"],
  ["tool7", "tool7"],
  ["tool8", "tool8"],
  ["tool9", "tool9"],
  ["tool10", "tool10"],
  ["tool11", "tool11"],
  ["tool12", "tool12"],
  ["tool13", "tool13"],
  ["tool14", "tool14"],
  ["tool15", "tool15"],
];

const wobjDropdown = [
  ["wobj0", "wobj0"],
  ["wobj1", "wobj1"],
  ["wobj2", "wobj2"],
  ["wobj3", "wobj3"],
  ["wobj4", "wobj4"],
  ["wobj5", "wobj5"],
  ["wobj6", "wobj6"],
  ["wobj7", "wobj7"],
  ["wobj8", "wobj8"],
  ["wobj9", "wobj9"],
  ["wobj10", "wobj10"],
  ["wobj11", "wobj11"],
  ["wobj12", "wobj12"],
  ["wobj13", "wobj13"],
  ["wobj14", "wobj14"],
  ["wobj15", "wobj15"],
  ["wobj16", "wobj16"],
  ["wobj17", "wobj17"],
  ["wobj18", "wobj18"],
  ["wobj19", "wobj19"],
  ["wobj20", "wobj20"],
  ["wobj21", "wobj21"],
  ["wobj22", "wobj22"],
  ["wobj23", "wobj23"],
  ["wobj24", "wobj24"],
  ["wobj25", "wobj25"],
  ["wobj26", "wobj26"],
  ["wobj27", "wobj27"],
  ["wobj28", "wobj28"],
  ["wobj29", "wobj29"],
  ["wobj30", "wobj30"],
  ["wobj31", "wobj31"],
];

let isLoadingXml = false;
export function setLoadingBlocks() {
  isLoadingXml = true;
}
export function setNotLoadingBlocks() {
  isLoadingXml = false;
}

// 变量区
Blockly.Blocks.base_variable = {
  init: function () {
    this.appendDummyInput()
      .appendField("Var")
      .appendField(
        new Blockly.FieldDropdown([
          ["int", "int"],
          ["string", "string"],
          ["boolen", "boolen"],
          ["double", "double"],
          ["array", "array"],
        ]),
        "type"
      )
      .appendField(new Blockly.FieldTextInput("a"), "name")
      .appendField("=")
      .appendField(new Blockly.FieldTextInput("0"), "value");

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.load_variable = {
  customContextMenu: function (options) {
    console.log(options);
    const option = {};
    option.enabled = { enabled: true };
    option.text = "Create Function";
    option.callback = () => popupFunction(this);
    options.push(option);
  },

  init: function () {
    this.appendDummyInput()
      .appendField("Var load")
      .appendField(new Blockly.FieldTextInput("l"), "name")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("mass"), "MASS")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("cogx"), "COGX")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("cogy"), "COGY")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("cogz"), "COGZ")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("q1"), "Q1")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("q2"), "Q2")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("q3"), "Q3")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("q4"), "Q4")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("ix"), "IX")
      .appendField(new Blockly.FieldTextInput("iy"), "IY")
      .appendField(new Blockly.FieldTextInput("iz"), "IZ")
      .appendField("}");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.jointtarget_variable = {
  init: function () {
    let motionPos = [0, 0, 0, 0, 0, 0];
    const state = store.getState();
    if (state.server && state.server.motion_pos) {
      motionPos = state.server.motion_pos;
    }

    this.appendDummyInput()
      .appendField("JointTarget")
      .appendField(
        new Blockly.FieldTextInput(`j${jointtargetNumber}`, validateJ),
        "name"
      )
      .appendField("={")
      .appendField(
        new Blockly.FieldTextInput((motionPos[0] / Math.PI) * 180),
        "j1"
      )
      .appendField(",")
      .appendField(
        new Blockly.FieldTextInput((motionPos[1] / Math.PI) * 180),
        "j2"
      )
      .appendField(",")
      .appendField(
        new Blockly.FieldTextInput((motionPos[2] / Math.PI) * 180),
        "j3"
      )
      .appendField(",")
      .appendField(
        new Blockly.FieldTextInput((motionPos[3] / Math.PI) * 180),
        "j4"
      )
      .appendField(",")
      .appendField(
        new Blockly.FieldTextInput((motionPos[4] / Math.PI) * 180),
        "j5"
      )
      .appendField(",")
      .appendField(
        new Blockly.FieldTextInput((motionPos[5] / Math.PI) * 180),
        "j6"
      )
      .appendField("}");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.robtarget_variable = {
  init: function () {
    if (!isLoadingXml) {
      let endPE = [0, 0, 0, 0, 0, 0];
      const state = store.getState();
      if (state.server && state.server.end_pe) endPE = state.server.end_pe;

      this.appendDummyInput()
        .appendField("P")
        .appendField(
          new Blockly.FieldTextInput(`p${robtargetNumber}`, validateP),
          "name"
        )
        .appendField(":")
        .appendField(
          new Blockly.FieldNumber(endPE[0] * 1000, -3000, 3000, 0.001),
          "x"
        )
        .appendField(
          new Blockly.FieldNumber(endPE[1] * 1000, -3000, 3000, 0.001),
          "y"
        )
        .appendField(
          new Blockly.FieldNumber(endPE[2] * 1000, -3000, 3000, 0.001),
          "z"
        )
        .appendField("mm")
        .appendField(
          new Blockly.FieldNumber((endPE[3] * 180) / Math.PI, -360, 360, 0.001),
          "a"
        )
        .appendField(
          new Blockly.FieldNumber((endPE[4] * 180) / Math.PI, -360, 360, 0.001),
          "b"
        )
        .appendField(
          new Blockly.FieldNumber((endPE[5] * 180) / Math.PI, -360, 360, 0.001),
          "c"
        )
        .appendField("°");
    } else {
      this.appendDummyInput()
        .appendField("P")
        .appendField(new Blockly.FieldTextInput(), "name")
        .appendField(":")
        .appendField(new Blockly.FieldNumber(0, -3000, 3000, 0.001), "x")
        .appendField(new Blockly.FieldNumber(0, -3000, 3000, 0.001), "y")
        .appendField(new Blockly.FieldNumber(0, -3000, 3000, 0.001), "z")
        .appendField("mm")
        .appendField(new Blockly.FieldNumber(0, -360, 360, 0.001), "a")
        .appendField(new Blockly.FieldNumber(0, -360, 360, 0.001), "b")
        .appendField(new Blockly.FieldNumber(0, -360, 360, 0.001), "c")
        .appendField("°");
    }

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    // this.setTooltip("");
    // this.setHelpUrl("");
  },
};

Blockly.Blocks.speed_variable = {
  init: function () {
    this.appendDummyInput()
      .appendField("V")
      .appendField(
        new Blockly.FieldTextInput(`v${speedNumber}`, validateV),
        "name"
      )
      .appendField(":")
      .appendField(new Blockly.FieldNumber(1, 0, 100, 1), "percent")
      .appendField("%")
      .appendField(new Blockly.FieldNumber(0.01, 0, 10, 0.001), "linear")
      .appendField("m/s")
      .appendField(new Blockly.FieldNumber(1, 0, 720, 0.001), "angular")
      .appendField("°/s")
      .appendField(new Blockly.FieldNumber(0.01, 0, 10, 0.001), "exj_linear")
      .appendField("m/s")
      .appendField(new Blockly.FieldNumber(1, 0, 720, 0.001), "exj_angular")
      .appendField("°/s");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.zone_variable = {
  init: function () {
    this.appendDummyInput()
      .appendField("Z")
      .appendField(
        new Blockly.FieldTextInput(`z${zoneNumber}`, validateZ),
        "name"
      )
      .appendField(":")
      .appendField(new Blockly.FieldNumber(10, 0, 10000, 0.001), "dis")
      .appendField("mm")
      .appendField(new Blockly.FieldNumber(10, 0, 100, 1), "per")
      .appendField("%");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    // this.setTooltip("");
    // this.setHelpUrl("");
  },
};

// 数学指令区
Blockly.Blocks.math_function = {
  init: function () {
    this.appendDummyInput()
      .appendField(
        new Blockly.FieldDropdown([
          ["sin", "sin"],
          ["cos", "cos"],
          ["tan", "tan"],
          ["cot", "cot"],
          ["asin", "asin"],
          ["atan", "atan"],
          ["atan2", "atan2"],
          ["acot", "acot"],
          ["abs", "abs"],
          ["exp", "exp"],
          ["sqrt", "aqrt"],
        ]),
        "function"
      )
      .appendField(new Blockly.FieldTextInput("default"), "param");
    this.setInputsInline(true);
    this.setOutput(true, null);
    this.setColour(290);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.math_opr = {
  init: function () {
    this.appendValueInput("left").setCheck(null);
    this.appendValueInput("right")
      .setCheck(null)
      .appendField(
        new Blockly.FieldDropdown([
          ["+", "+"],
          ["-", "-"],
          ["*", "*"],
          ["/", "/"],
        ]),
        "opr"
      );
    this.setInputsInline(true);
    this.setOutput(true, null);
    this.setColour(290);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.logic_opr = {
  init: function () {
    this.appendValueInput("left").setCheck(null);
    this.appendValueInput("right")
      .setCheck(null)
      .appendField(
        new Blockly.FieldDropdown([
          ["and", "and"],
          ["or", "or"],
          ["not", "not"],
        ]),
        "opr"
      );
    this.setInputsInline(true);
    this.setOutput(true, null);
    this.setColour(290);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.compare_opr = {
  init: function () {
    this.appendValueInput("left").setCheck(null);
    this.appendValueInput("right")
      .setCheck(null)
      .appendField(
        new Blockly.FieldDropdown([
          [">", ">"],
          ["<", "<"],
          ["==", "=="],
          [">=", ">="],
          ["<=", "<="],
          ["!=", "!="],
        ]),
        "opr"
      );
    this.setInputsInline(true);
    this.setOutput(true, null);
    this.setColour(290);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.number = {
  init: function () {
    this.appendDummyInput().appendField(
      new Blockly.FieldTextInput("number"),
      "number"
    );
    this.setInputsInline(true);
    this.setOutput(true, null);
    this.setColour(290);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

// 逻辑指令区
const getFunctionLists = (block, type) => {
  const dropList = block.workspace[type]
    ? JSON.parse(JSON.stringify(block.workspace[type]))
    : [];
  dropList.unshift(["#", "select"]);

  if (block.workspace) {
    block.workspace.getBlocksByType(type).forEach((blk) => {
      const name = blk.getFieldValue("name");
      dropList.push([name, block.workspace.fileName + "." + name]);
    });
  }

  return dropList;
};

Blockly.Blocks.if = {
  init: function () {
    this.appendValueInput("condition").setCheck("Boolean").appendField("if");
    this.appendStatementInput("body").setCheck(null).appendField("do");
    this.appendDummyInput().appendField("endif");

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.while = {
  init: function () {
    this.appendValueInput("condition").setCheck(null).appendField("while");
    this.appendStatementInput("body").setCheck(null).appendField("do");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.function = {
  validate: function (newValue) {
    const reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
    if (reg.test(newValue)) {
      return newValue;
    } else {
      return null;
    }
  },
  init: function () {
    this.appendDummyInput()
      .appendField("function")
      .appendField(
        new Blockly.FieldTextInput("default", this.validate),
        "name"
      );
    this.appendStatementInput("body").setCheck(null);
    this.appendDummyInput().appendField("endfunction");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.main = {
  init: function () {
    this.appendDummyInput().appendField("main");
    this.appendStatementInput("body").setCheck(null);
    this.appendDummyInput().appendField("endmain");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.call = {
  init: function () {
    this.functionList = [["select", "select"]];
    this.appendDummyInput()
      .appendField("call")
      .appendField(
        new Blockly.FieldDropdown((e) => getFunctionLists(this, "function")),
        "func_name"
      );
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

// 运动指令区
const getVariableLists = (block, type) => {
  const dropList = block.workspace[type]
    ? JSON.parse(JSON.stringify(block.workspace[type]))
    : [];
  dropList.unshift(["#", "select"]);

  if (block.workspace.variablesWorkspace) {
    block.workspace.variablesWorkspace.getBlocksByType(type).forEach((blk) => {
      const name = blk.getFieldValue("name");
      dropList.push([name, block.workspace.fileName + "." + name]);
    });
  }

  return dropList;
};

Blockly.Blocks.mvaj = {
  init: function () {
    this.appendDummyInput("DummyInput")
      .appendField("Moveabsj JointTarget=")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "jointtarget_variable")
        ),
        "pos"
      )
      .appendField("v:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "speed_variable")
        ),
        "vel"
      )
      .appendField("z:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "zone_variable")
        ),
        "zone"
      )
      .appendField("");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(340);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.mvc = {
  init: function () {
    this.appendDummyInput()
      .appendField("C p1:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "robtarget_variable")
        ),
        "mid_pe"
      )
      .appendField("p2:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "robtarget_variable")
        ),
        "end_pe"
      )
      .appendField("v:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "speed_variable")
        ),
        "vel"
      )
      .appendField("z:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "zone_variable")
        ),
        "zone"
      )
      .appendField(new Blockly.FieldDropdown(toolDropdown), "tool")
      .appendField(new Blockly.FieldDropdown(wobjDropdown), "wobj")
      .appendField("FC=")
      .appendField(
        new Blockly.FieldDropdown([
          ["FALSE", "FALSE"],
          ["TRUE", "TRUE"],
        ]),
        "fc"
      );
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(340);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.mvl = {
  init: function () {
    this.appendDummyInput("DUMMYINPUT1")
      .appendField("L p:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "robtarget_variable")
        ),
        "pe"
      )
      .appendField("v:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "speed_variable")
        ),
        "vel"
      )
      .appendField("z:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "zone_variable")
        ),
        "zone"
      )
      .appendField(new Blockly.FieldDropdown(toolDropdown), "tool")
      .appendField(new Blockly.FieldDropdown(wobjDropdown), "wobj")
      .appendField("FC=")
      .appendField(
        new Blockly.FieldDropdown([
          ["FALSE", "FALSE"],
          ["TRUE", "TRUE"],
        ]),
        "fc"
      );
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(340);
    this.setTooltip("Move L");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.mvj = {
  init: function () {
    this.appendDummyInput("DUMMYINPUT1")
      .appendField("J p:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "robtarget_variable")
        ),
        "pe"
      )
      .appendField("v:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "speed_variable")
        ),
        "vel"
      )
      .appendField("z:")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "zone_variable")
        ),
        "zone"
      )
      .appendField(new Blockly.FieldDropdown(toolDropdown), "tool")
      .appendField(new Blockly.FieldDropdown(wobjDropdown), "wobj");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(340);
    this.setTooltip("Move J");
    this.setHelpUrl("");
  },
};

// IO
Blockly.Blocks.sl = {
  init: function () {
    this.appendDummyInput()
      .appendField("wait")
      .appendField(new Blockly.FieldTextInput("0"), "NUM")
      .appendField("ms");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.setdo = {
  init: function () {
    this.appendDummyInput()
      .appendField("SetDO")
      .appendField(
        new Blockly.FieldDropdown(() => {
          if (!isLoadingXml) {
            const state = store.getState();
            let num = 0;
            if (
              state.robot &&
              state.robot.resultsByChannel &&
              state.robot.resultsByChannel.get &&
              state.robot.resultsByChannel.get.do
            ) {
              state.robot.resultsByChannel.get.do.forEach(
                (d) => (num = num + d.length)
              );
            }

            const options = [["#", "select"]];

            for (let i = 0; i < num; i++) {
              options.push([String(i), String(i)]);
            }

            return options;
          } else {
            const options = [["#", "select"]];

            for (let i = 0; i < 128; i++) {
              options.push([String(i), String(i)]);
            }

            return options;
          }
        }),
        "INPUT1"
      )
      .appendField(
        new Blockly.FieldDropdown([
          ["TRUE", "TRUE"],
          ["FALSE", "FALSE"],
        ]),
        "DROPDOWN1"
      );
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.waitdi = {
  init: function () {
    this.Input = [["select", "select"]];
    this.appendDummyInput()
      .appendField("WaitDI")
      .appendField(
        new Blockly.FieldDropdown(() => {
          if (!isLoadingXml) {
            const state = store.getState();
            let num = 0;
            if (
              state.robot &&
              state.robot.resultsByChannel &&
              state.robot.resultsByChannel.get &&
              state.robot.resultsByChannel.get.di
            ) {
              state.robot.resultsByChannel.get.di.forEach(
                (d) => (num = num + d.length)
              );
            }

            const options = [["#", "select"]];

            for (let i = 0; i < num; i++) {
              options.push([String(i), String(i)]);
            }

            return options;
          } else {
            const options = [["#", "select"]];

            for (let i = 0; i < 128; i++) {
              options.push([String(i), String(i)]);
            }

            return options;
          }
        }),
        "index"
      )
      .appendField(
        new Blockly.FieldDropdown([
          ["True", "True"],
          ["False", "False"],
        ]),
        "status"
      );
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

// 力控指令区
Blockly.Blocks.FCPressP = {
  init: function () {
    this.appendDummyInput()
      .appendField("FCPressP")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("p1"), "P1")
      .appendField(new Blockly.FieldTextInput("p2"), "P2")
      .appendField(new Blockly.FieldTextInput("p3"), "P3")
      .appendField(new Blockly.FieldTextInput("p4"), "P4")
      .appendField(new Blockly.FieldTextInput("p5"), "P5")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("Force"), "FORCE")
      .appendField(new Blockly.FieldTextInput("Tool"), "TOOL")
      .appendField(new Blockly.FieldTextInput("Wobj"), "WOBJ")
      .appendField("}");
    this.setColour(230);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.FCPressL = {
  init: function () {
    this.appendDummyInput()
      .appendField("FCPressL")
      .appendField(new Blockly.FieldTextInput("Force"), "FORCE")
      .appendField(new Blockly.FieldTextInput("Tool"), "TOOL")
      .appendField(new Blockly.FieldTextInput("Wobj"), "WOBJ");
    this.setColour(230);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.FCPressS = {
  init: function () {
    this.appendDummyInput()
      .appendField("FCPressS")
      .appendField(new Blockly.FieldTextInput("Force"), "FORCE")
      .appendField(new Blockly.FieldTextInput("Tool"), "TOOL")
      .appendField(new Blockly.FieldTextInput("Wobj"), "WOBJ");
    this.setColour(230);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.admitinit = {
  mutationToDom: function () {
    const container = document.createElement("mutation");
    const toolNameList = this.getFieldValue("DROPDOWN4");
    const wobjNameList = this.getFieldValue("DROPDOWN5");
    container.setAttribute("tool", toolNameList);
    container.setAttribute("wobj", wobjNameList);
    return container;
  },

  domToMutation: function (xmlElement) {
    // This attribute should match the one you used in mutationToDom
    const tool = xmlElement.getAttribute("tool");
    const wobj = xmlElement.getAttribute("wobj");
    // tool变量名称
    if (tool === "tool0") {
    } else if (tool === "null") {
      this.toolList = [["tool0", "tool0"]];
    } else {
      this.toolList.pop();
      this.toolList.push([tool, tool], ["tool0", "tool0"]);
    }
    // wobj变量名称
    if (wobj === "wobj0") {
    } else if (wobj === "null") {
      this.wobjList = [["wobj0", "wobj0"]];
    } else {
      this.wobjList.pop();
      this.wobjList.push([wobj, wobj], ["wobj0", "wobj0"]);
    }
  },

  init: function () {
    this.toolList = [["tool0", "tool0"]];
    this.wobjList = [["wobj0", "wobj0"]];
    this.appendDummyInput()
      .appendField("AdmintInit", "ADMINTINIT_FIELD")
      .appendField("b", "FIRST_FIELD_B")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("100000"), "B1")
      .appendField(new Blockly.FieldTextInput("100000"), "B2")
      .appendField(new Blockly.FieldTextInput("100000"), "B3")
      .appendField(new Blockly.FieldTextInput("100000"), "B4")
      .appendField(new Blockly.FieldTextInput("100000"), "B5")
      .appendField(new Blockly.FieldTextInput("100000"), "B6")
      .appendField("}");
    this.appendDummyInput()
      .appendField("ftset", "FTSET_FIELD")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET1")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET2")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET3")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET4")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET5")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET6")
      .appendField("}")
      .appendField("deadzone", "DEADZONE_FIELD")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("1"), "DEADZONE1")
      .appendField(new Blockly.FieldTextInput("1"), "DEADZONE2")
      .appendField(new Blockly.FieldTextInput("1"), "DEADZONE3")
      .appendField(new Blockly.FieldTextInput("1"), "DEADZONE4")
      .appendField(new Blockly.FieldTextInput("1"), "DEADZONE5")
      .appendField(new Blockly.FieldTextInput("1"), "DEADZONE6")
      .appendField("}");
    this.appendDummyInput()
      .appendField("vellimit", "VELLIMIT_FIELD")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("1"), "VELLIMIT1")
      .appendField(new Blockly.FieldTextInput("1"), "VELLIMIT2")
      .appendField(new Blockly.FieldTextInput("1"), "VELLIMIT3")
      .appendField(new Blockly.FieldTextInput("1"), "VELLIMIT4")
      .appendField(new Blockly.FieldTextInput("1"), "VELLIMIT5")
      .appendField(new Blockly.FieldTextInput("1"), "VELLIMIT6")
      .appendField("}")
      .appendField("corposlimit", "CORPOSLIMIT_FIELD")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("1"), "CORPOSLIMIT1")
      .appendField(new Blockly.FieldTextInput("1"), "CORPOSLIMIT2")
      .appendField(new Blockly.FieldTextInput("1"), "CORPOSLIMIT3")
      .appendField(new Blockly.FieldTextInput("1"), "CORPOSLIMIT4")
      .appendField(new Blockly.FieldTextInput("1"), "CORPOSLIMIT5")
      .appendField(new Blockly.FieldTextInput("1"), "CORPOSLIMIT6")
      .appendField("}");
    this.appendDummyInput()
      .appendField("tool=")
      .appendField(
        new Blockly.FieldDropdown(() => {
          if (this.toolList.length <= 1) {
            for (let i = 1; i < 16; i++) {
              this.toolList.push([`tool${i}`, `tool${i}`]);
            }
          }
          return this.toolList;
        }),
        "DROPDOWN4"
      )
      .appendField("wobj=")
      .appendField(
        new Blockly.FieldDropdown(() => {
          if (this.wobjList.length <= 1) {
            for (let i = 1; i < 35; i++) {
              this.wobjList.push([`wobj${i}`, `wobj${i}`]);
            }
          }
          return this.wobjList;
        }),
        "DROPDOWN5"
      );
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.coroffset = {
  init: function () {
    this.appendDummyInput()
      .appendField("coroffset", "COROFFSET_FIELD")
      .appendField("offset", "OFFSET_FIELD")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("0"), "B1")
      .appendField(new Blockly.FieldTextInput("0"), "B2")
      .appendField(new Blockly.FieldTextInput("0"), "B3")
      .appendField(new Blockly.FieldTextInput("0"), "B4")
      .appendField(new Blockly.FieldTextInput("0"), "B5")
      .appendField(new Blockly.FieldTextInput("0"), "B6")
      .appendField("}")
      .appendField("icount", "ICOUNT_FIELD")
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET1")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET2")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET3")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET4")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET5")
      .appendField(new Blockly.FieldTextInput("0"), "FTSET6")
      .appendField("}");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.FCMonitor = {
  mutationToDom: function () {
    const container = document.createElement("mutation");
    // Do not use camelCase values for attribute names
    const modeNameList = this.getFieldValue("DROPDOWN1");
    const offsetNameList = this.getFieldValue("DROPDOWN2");
    container.setAttribute("mode", modeNameList);
    container.setAttribute("offset", offsetNameList);
    // ALWAYS return container; this will be the input for domToMutation.
    return container;
  },

  domToMutation: function (xmlElement) {
    // This attribute should match the one you used in mutationToDom
    const mode = xmlElement.getAttribute("mode");
    const offset = xmlElement.getAttribute("offset");
    // tool变量名称
    if (mode === "0") {
    } else if (mode === "null") {
      this.modeList = [
        ["0", "mode0"],
        ["1", "mode1"],
      ];
    } else {
      this.modeList.pop();
      this.modeList.pop();
      this.modeList.push(["1", "mode1"], ["0", "mode0"]);
    }
    // offset变量名称
    if (offset === "offset0") {
    } else if (offset === "null") {
    } else if (offset === "select") {
    } else {
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      this.offsetList.pop();
      let newOffset = "0";
      try {
        newOffset = offset.split("").pop();
      } catch {
        console.error("newOffset错误");
      }
      this.offsetList.push(
        [newOffset, `offset${newOffset}`],
        ["select", "select"],
        ["0", "offset0"],
        ["1", "offset1"],
        ["2", "offset2"],
        ["3", "offset3"],
        ["4", "offset4"],
        ["5", "offset5"],
        ["6", "offset6"],
        ["7", "offset7"],
        ["8", "offset8"],
        ["9", "offset9"]
      );
    }
  },
  init: function () {
    this.modeList = [
      ["0", "mode0"],
      ["1", "mode1"],
    ];
    this.offsetList = [
      ["select", "select"],
      ["0", "offset0"],
      ["1", "offset1"],
      ["2", "offset2"],
      ["3", "offset3"],
      ["4", "offset4"],
      ["5", "offset5"],
      ["6", "offset6"],
      ["7", "offset7"],
      ["8", "offset8"],
      ["9", "offset9"],
    ];
    this.appendDummyInput()
      .appendField("FCMonitor", "FCMONITER_FIELD")
      .appendField("--mode", "MODE_FIELD")
      .appendField("=")
      .appendField(
        new Blockly.FieldDropdown(() => {
          return this.modeList;
        }),
        "DROPDOWN1"
      )
      .appendField("--offset", "OFFSET_FIELD")
      .appendField("=")
      .appendField(
        new Blockly.FieldDropdown(() => {
          return this.offsetList;
        }),
        "DROPDOWN2"
      )
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("x"), "B1")
      .appendField(new Blockly.FieldTextInput("y"), "B2")
      .appendField(new Blockly.FieldTextInput("z"), "B3")
      .appendField(new Blockly.FieldTextInput("a"), "B4")
      .appendField(new Blockly.FieldTextInput("b"), "B5")
      .appendField(new Blockly.FieldTextInput("c"), "B6")
      .appendField("}");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.saveproxml = {
  init: function () {
    this.appendDummyInput()
      .appendField("saveproxml", "COROFFSET_FIELD")
      .appendField("--name", "OFFSET_FIELD")
      .appendField("=")
      .appendField(new Blockly.FieldTextInput("address"), "B1");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.loadproxml = {
  init: function () {
    this.appendDummyInput()
      .appendField("loadproxml", "COROFFSET_FIELD")
      .appendField("--name", "OFFSET_FIELD")
      .appendField("=")
      // .appendField("{")
      .appendField(new Blockly.FieldTextInput("address"), "B1");
    // .appendField("}")
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

// set
Blockly.Blocks.set_speed = {
  validator_1: function (newValue) {
    // let reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
    if (newValue >= 0 && newValue <= 100) {
      return newValue;
    } else {
      return null;
    }
  },
  validator_2: function (newValue) {
    // let reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
    if (newValue >= 0 && newValue <= 6) {
      return newValue;
    } else {
      return null;
    }
  },
  validator_3: function (newValue) {
    // let reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
    if (newValue >= 0 && newValue <= 360) {
      return newValue;
    } else {
      return null;
    }
  },

  init: function () {
    this.nameList = [["select", "select"]];
    this.appendDummyInput()
      .appendField("set speed", "first_field")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "speed_variable")
        ),
        "speed"
      )
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("0", this.validator_1), "SPEED0")
      .appendField("%")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("0", this.validator_2), "SPEED1")
      .appendField("m/s")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("0", this.validator_3), "SPEED2")
      .appendField("°/s")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("0", this.validator_3), "SPEED3")
      .appendField("°/s")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("0", this.validator_2), "SPEED4")
      .appendField("m/s")
      .appendField("}");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.set_zone = {
  validator_1: function (newValue) {
    // let reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
    if (newValue >= 0 && newValue <= 10000) {
      return newValue;
    } else {
      return null;
    }
  },
  validator_2: function (newValue) {
    // let reg = /^[a-zA-Z][a-zA-Z0-9_]*$/;
    if (newValue >= 0 && newValue <= 100) {
      return newValue;
    } else {
      return null;
    }
  },

  init: function () {
    this.appendDummyInput()
      .appendField("set zone", "first_field")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "zone_variable")
        ),
        "zone"
      )
      .appendField("=")
      .appendField("{")
      .appendField(new Blockly.FieldTextInput("0", this.validator_1), "DIS")
      .appendField("mm")
      .appendField(",")
      .appendField(new Blockly.FieldTextInput("0", this.validator_2), "PER")
      .appendField("%")
      .appendField("}");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.set_jointtarget = {
  init: function () {
    this.appendValueInput("NAME")
      .setCheck(null)
      .appendField("set")
      .appendField(new Blockly.FieldTextInput("default"), "Input")
      .appendField("=");

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};

Blockly.Blocks.set_robtarget = {
  init: function () {
    this.appendDummyInput()
      .appendField("Set RobTarget")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "robtarget_variable")
        ),
        "RobTarget"
      )
      .appendField("=")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "robtarget_variable")
        ),
        "RobTarget1"
      )
      .appendField("+")
      .appendField(
        new Blockly.FieldDropdown((e) =>
          getVariableLists(this, "robtarget_variable")
        ),
        "RobTarget2"
      );

    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(120);
    this.setTooltip("");
    this.setHelpUrl("");
  },
};
