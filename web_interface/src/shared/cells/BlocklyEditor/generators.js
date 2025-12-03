import Blockly from "blockly";

// 变量区
Blockly.Robot.base_variable = function (block) {
  const type = block.getFieldValue("type");
  const name = block.workspace.fileName + "." + block.getFieldValue("name");
  const value = block.getFieldValue("value");

  let code = "";
  if (type === "array") {
    code = "var " + type + " " + name + " = " + value + "\n";
  } else if (type === "string") {
    code = "var " + type + " " + name + ' = "' + value + '"\n';
  } else {
    code = "var Number " + name + " = " + value + "\n";
  }

  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.load_variable = function (block) {
  const textVariable =
    block.workspace.fileName + "." + block.getFieldValue("name");
  const textMass = block.getFieldValue("MASS");
  const textCogx = block.getFieldValue("COGX");
  const textCogy = block.getFieldValue("COGY");
  const textCogz = block.getFieldValue("COGZ");
  const textQ1 = block.getFieldValue("Q1");
  const textQ2 = block.getFieldValue("Q2");
  const textQ3 = block.getFieldValue("Q3");
  const textQ4 = block.getFieldValue("Q4");
  const textIx = block.getFieldValue("IX");
  const textIy = block.getFieldValue("IY");
  const textIz = block.getFieldValue("IZ");
  const code =
    "var load " +
    textVariable +
    " = " +
    "{" +
    textMass +
    "," +
    textCogx +
    "," +
    textCogy +
    "," +
    textCogz +
    "," +
    textQ1 +
    "," +
    textQ2 +
    "," +
    textQ3 +
    "," +
    textQ4 +
    "," +
    textIx +
    "," +
    textIy +
    "," +
    textIz +
    "}" +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.jointtarget_variable = function (block) {
  const name = block.workspace.fileName + "." + block.getFieldValue("name");
  const j1 = ((Number(block.getFieldValue("j1")) * Math.PI) / 180).toString();
  const j2 = ((Number(block.getFieldValue("j2")) * Math.PI) / 180).toString();
  const j3 = ((Number(block.getFieldValue("j3")) * Math.PI) / 180).toString();
  const j4 = ((Number(block.getFieldValue("j4")) * Math.PI) / 180).toString();
  const j5 = ((Number(block.getFieldValue("j5")) * Math.PI) / 180).toString();
  const j6 = ((Number(block.getFieldValue("j6")) * Math.PI) / 180).toString();
  const code =
    "var jointtarget " +
    name +
    " = {" +
    j1 +
    "," +
    j2 +
    "," +
    j3 +
    "," +
    j4 +
    "," +
    j5 +
    "," +
    j6 +
    "}" +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.robtarget_variable = function (block) {
  const name = block.workspace.fileName + "." + block.getFieldValue("name");
  const x = (Number(block.getFieldValue("x")) / 1000).toString();
  const y = (Number(block.getFieldValue("y")) / 1000).toString();
  const z = (Number(block.getFieldValue("z")) / 1000).toString();
  const a = ((Number(block.getFieldValue("a")) / 180) * Math.PI).toString();
  const b = ((Number(block.getFieldValue("b")) / 180) * Math.PI).toString();
  const c = ((Number(block.getFieldValue("c")) / 180) * Math.PI).toString();
  const code =
    "var robtarget " +
    name +
    " = {" +
    x +
    "," +
    y +
    "," +
    z +
    "," +
    a +
    "," +
    b +
    "," +
    c +
    "}" +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.speed_variable = function (block) {
  const name = block.workspace.fileName + "." + block.getFieldValue("name");
  const percent = (Number(block.getFieldValue("percent")) / 100).toString();
  const linear = Number(block.getFieldValue("linear")).toString();
  const angular = (
    (Number(block.getFieldValue("angular")) / 180) *
    Math.PI
  ).toString();
  const exjAngular = (
    (Number(block.getFieldValue("exj_angular")) / 180) *
    Math.PI
  ).toString();
  const exjLinear = Number(block.getFieldValue("exj_linear")).toString();
  const code =
    "var speed " +
    name +
    " = {" +
    percent +
    "," +
    linear +
    "," +
    angular +
    "," +
    exjAngular +
    "," +
    exjLinear +
    "}" +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.zone_variable = function (block) {
  const name = block.workspace.fileName + "." + block.getFieldValue("name");
  const dis = (Number(block.getFieldValue("dis")) / 1000).toString();
  const per = (Number(block.getFieldValue("per")) / 100).toString();
  const code = "var zone " + name + " = {" + dis + "," + per + "}\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

// 数学指令区
Blockly.Robot.math_function = function (block) {
  const code =
    block.getFieldValue("function") + "(" + block.getFieldValue("param") + ")";
  return [code, Blockly.Robot.ORDER_NONE];
};

Blockly.Robot.math_opr = function (block) {
  const opr = block.getFieldValue("opr");
  const left = Blockly.Robot.valueToCode(
    block,
    "left",
    Blockly.Robot.ORDER_ATOMIC
  );
  const right = Blockly.Robot.valueToCode(
    block,
    "right",
    Blockly.Robot.ORDER_ATOMIC
  );
  const code = left + opr + right;
  return [code, Blockly.Robot.ORDER_NONE];
};

Blockly.Robot.logic_opr = function (block) {
  const left = Blockly.Robot.valueToCode(
    block,
    "left",
    Blockly.Robot.ORDER_ATOMIC
  );
  const opr = block.getFieldValue("opr");
  const right = Blockly.Robot.valueToCode(
    block,
    "right",
    Blockly.Robot.ORDER_ATOMIC
  );
  const code = left + opr + right;
  return [code, Blockly.Robot.ORDER_NONE];
};

Blockly.Robot.compare_opr = function (block) {
  const left = Blockly.Robot.valueToCode(
    block,
    "left",
    Blockly.Robot.ORDER_ATOMIC
  );
  const opr = block.getFieldValue("opr");
  const right = Blockly.Robot.valueToCode(
    block,
    "right",
    Blockly.Robot.ORDER_ATOMIC
  );
  const code = left + opr + right;
  return [code, Blockly.Robot.ORDER_NONE];
};

Blockly.Robot.number = function (block) {
  const branchCode = block.getFieldValue("number");
  const code =
    (branchCode[0] < "0" || branchCode[0] > "9") &&
    branchCode.indexOf(".") === -1
      ? block.workspace.fileName + "." + branchCode
      : branchCode;

  return [code, Blockly.Robot.ORDER_NONE];
};

// 运动指令
Blockly.Robot.mvaj = function (block) {
  const pos = block.getFieldValue("pos");
  const vel = block.getFieldValue("vel");
  const zone = block.getFieldValue("zone");

  const code =
    "mvaj" +
    (pos === "select" ? "" : " --pos=" + pos) +
    (vel === "select" ? "" : " --vel=" + vel) +
    (zone === "select" ? "" : " --zone=" + zone) +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.mvc = function (block) {
  const midPE = block.getFieldValue("mid_pe");
  const endPE = block.getFieldValue("end_pe");
  const vel = block.getFieldValue("vel");
  const zone = block.getFieldValue("zone");
  const tool = block.getFieldValue("tool");
  const wobj = block.getFieldValue("wobj");
  const fc = block.getFieldValue("fc");

  const code =
    "mvc" +
    (midPE === "select" ? "" : " --mid_pe=" + midPE) +
    (endPE === "select" ? "" : " --end_pe=" + endPE) +
    (vel === "select" ? "" : " --vel=" + vel) +
    (zone === "select" ? "" : " --zone=" + zone) +
    (tool === "select" ? "" : " --tool=" + tool) +
    (wobj === "select" ? "" : " --wobj=" + wobj) +
    (" --fc=" + (fc === "FALSE" ? "0" : "1")) +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.mvl = function (block) {
  const pe = block.getFieldValue("pe");
  const vel = block.getFieldValue("vel");
  const zone = block.getFieldValue("zone");
  const tool = block.getFieldValue("tool");
  const wobj = block.getFieldValue("wobj");
  const fc = block.getFieldValue("fc");

  const code =
    "mvl" +
    (pe === "select" ? "" : " --pe=" + pe) +
    (vel === "select" ? "" : " --vel=" + vel) +
    (zone === "select" ? "" : " --zone=" + zone) +
    (tool === "select" ? "" : " --tool=" + tool) +
    (wobj === "select" ? "" : " --wobj=" + wobj) +
    (" --fc=" + (fc === "FALSE" ? "0" : "1")) +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.mvj = function (block) {
  const pe = block.getFieldValue("pe");
  const vel = block.getFieldValue("vel");
  const zone = block.getFieldValue("zone");
  const tool = block.getFieldValue("tool");
  const wobj = block.getFieldValue("wobj");

  const code =
    "mvj" +
    (pe === "select" ? "" : " --pe=" + pe) +
    (vel === "select" ? "" : " --vel=" + vel) +
    (zone === "select" ? "" : " --zone=" + zone) +
    (tool === "select" ? "" : " --tool=" + tool) +
    (wobj === "select" ? "" : " --wobj=" + wobj) +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

// 逻辑指令区
Blockly.Robot.if = function (block) {
  const conditionCode =
    Blockly.Robot.valueToCode(block, "condition", Blockly.Robot.ORDER_NONE) ||
    "false";
  block.data = ++block.workspace.generateId;
  const bodyCode = Blockly.Robot.statementToCode(block, "body");
  block.workspace.generateId++;
  return (
    block.data +
    ":if (" +
    conditionCode +
    ") \n" +
    bodyCode +
    block.workspace.generateId +
    ":endif\n"
  );
};

Blockly.Robot.while = function (block) {
  const conditionCode =
    Blockly.Robot.valueToCode(block, "condition", Blockly.Robot.ORDER_ATOMIC) ||
    false;
  block.data = ++block.workspace.generateId;
  const bodyCode = Blockly.Robot.statementToCode(block, "body");
  block.workspace.generateId++;
  return (
    block.data +
    ":while" +
    "(" +
    conditionCode +
    ")" +
    "\n" +
    bodyCode +
    block.workspace.generateId +
    ":endwhile\n"
  );
};

Blockly.Robot.function = function (block) {
  const funcName = block.workspace.fileName + "." + block.getFieldValue("name");
  block.data = ++block.workspace.generateId;
  const bodyCode = Blockly.Robot.statementToCode(block, "body");
  block.workspace.generateId++;
  return (
    block.data +
    ":function " +
    funcName +
    "\n" +
    bodyCode +
    block.workspace.generateId +
    ":endfunction\n"
  );
};

Blockly.Robot.main = function (block) {
  block.data = ++block.workspace.generateId;
  const bodyCode = Blockly.Robot.statementToCode(block, "body");
  block.workspace.generateId++;
  return (
    block.data +
    ":main\n" +
    bodyCode +
    block.workspace.generateId +
    ":endmain\n"
  );
};

Blockly.Robot.call = function (block) {
  const code = block.getFieldValue("func_name") + "()\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

// IO
Blockly.Robot.sl = function (block) {
  const NUM = block.getFieldValue("NUM");
  const code = "sl --count=" + NUM + "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.setdo = function (block) {
  let input = block.getFieldValue("INPUT1");
  let dropdown = block.getFieldValue("DROPDOWN1");
  if (input === "select") {
    input = "0";
  }
  if (dropdown === "FALSE") {
    dropdown = "0";
  } else {
    dropdown = "1";
  }
  // TODO: Assemble Robot into code variable.
  const code = "setdo --index=" + input + " --status=" + dropdown;
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}\n`;
};

Blockly.Robot.waitdi = function (block) {
  const i = block.getFieldValue("index");
  const s = block.getFieldValue("status");
  const code = "waitdi --index=" + i + " --status=" + (s ? 1 : 0) + "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

// 力控指令
Blockly.Robot.FCPressP = function (block) {
  // let text_name = block.getFieldValue('name');
  // let text_p1 = block.getFieldValue("P1");
  // let text_p2 = block.getFieldValue("P2");
  // let text_p3 = block.getFieldValue("P3");
  // let text_p4 = block.getFieldValue("P4");
  // let text_p5 = block.getFieldValue("P5");
  // let Force = block.getFieldValue("FORCE");
  // let Tool = block.getFieldValue("TOOL");
  // let Wobj = block.getFieldValue("WOBJ");
  // TODO: Assemble Robot into code variable.
  const code = "FCPressP\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.FCPressL = function (block) {
  // let Force = block.getFieldValue("FORCE");
  // let Tool = block.getFieldValue("TOOL");
  // let Wobj = block.getFieldValue("WOBJ");
  // TODO: Assemble Robot into code variable.
  const code = "FCPressL\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.FCPressS = function (block) {
  // let Force = block.getFieldValue("FORCE");
  // let Tool = block.getFieldValue("TOOL");
  // let Wobj = block.getFieldValue("WOBJ");
  // TODO: Assemble Robot into code variable.
  const code = "FCPressS\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.admitinit = function (block) {
  const B1 = this.getFieldValue("B1");
  const B2 = this.getFieldValue("B2");
  const B3 = this.getFieldValue("B3");
  const B4 = this.getFieldValue("B4");
  const B5 = this.getFieldValue("B5");
  const B6 = this.getFieldValue("B6");
  const codeB = ` --b={${B1},${B2},${B3},${B4},${B5},${B6}}`;
  const FTSET1 = this.getFieldValue("FTSET1");
  const FTSET2 = this.getFieldValue("FTSET2");
  const FTSET3 = this.getFieldValue("FTSET3");
  const FTSET4 = this.getFieldValue("FTSET4");
  const FTSET5 = this.getFieldValue("FTSET5");
  const FTSET6 = this.getFieldValue("FTSET6");
  const codeFTSET = ` --ftset={${FTSET1},${FTSET2},${FTSET3},${FTSET4},${FTSET5},${FTSET6}}`;
  const DEADZONE1 = this.getFieldValue("DEADZONE1");
  const DEADZONE2 = this.getFieldValue("DEADZONE2");
  const DEADZONE3 = this.getFieldValue("DEADZONE3");
  const DEADZONE4 = this.getFieldValue("DEADZONE4");
  const DEADZONE5 = this.getFieldValue("DEADZONE5");
  const DEADZONE6 = this.getFieldValue("DEADZONE6");
  const codeDEADZONE = ` --deadzone={${DEADZONE1},${DEADZONE2},${DEADZONE3},${DEADZONE4},${DEADZONE5},${DEADZONE6}}`;

  const VELLIMIT1 = this.getFieldValue("VELLIMIT1");
  const VELLIMIT2 = this.getFieldValue("VELLIMIT2");
  const VELLIMIT3 = this.getFieldValue("VELLIMIT3");
  const VELLIMIT4 = this.getFieldValue("VELLIMIT4");
  const VELLIMIT5 = this.getFieldValue("VELLIMIT5");
  const VELLIMIT6 = this.getFieldValue("VELLIMIT6");
  const codeVELLIMIT = ` --vellimit={${VELLIMIT1},${VELLIMIT2},${VELLIMIT3},${VELLIMIT4},${VELLIMIT5},${VELLIMIT6}}`;

  const CORPOSLIMIT1 = this.getFieldValue("CORPOSLIMIT1");
  const CORPOSLIMIT2 = this.getFieldValue("CORPOSLIMIT2");
  const CORPOSLIMIT3 = this.getFieldValue("CORPOSLIMIT3");
  const CORPOSLIMIT4 = this.getFieldValue("CORPOSLIMIT4");
  const CORPOSLIMIT5 = this.getFieldValue("CORPOSLIMIT5");
  const CORPOSLIMIT6 = this.getFieldValue("CORPOSLIMIT6");
  const codeCORPOSLIMIT = ` --corposlimit={${CORPOSLIMIT1},${CORPOSLIMIT2},${CORPOSLIMIT3},${CORPOSLIMIT4},${CORPOSLIMIT5},${CORPOSLIMIT6}}`;
  const tool = this.getFieldValue("DROPDOWN4");
  const codeTool = ` --tool=${tool}`;
  const wobj = this.getFieldValue("DROPDOWN5");
  const codeWobj = ` --wobj=${wobj}`;
  // TODO: Assemble Robot into code variable.
  const code =
    "admitinit" +
    codeB +
    codeFTSET +
    codeDEADZONE +
    codeVELLIMIT +
    codeCORPOSLIMIT +
    codeTool +
    codeWobj;
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}\n`;
};

Blockly.Robot.coroffset = function (block) {
  const B1 = this.getFieldValue("B1");
  const B2 = this.getFieldValue("B2");
  const B3 = this.getFieldValue("B3");
  const B4 = this.getFieldValue("B4");
  const B5 = this.getFieldValue("B5");
  const B6 = this.getFieldValue("B6");
  const codeB = ` --offset={${B1},${B2},${B3},${B4},${B5},${B6}}`;
  const FTSET1 = this.getFieldValue("FTSET1");
  const FTSET2 = this.getFieldValue("FTSET2");
  const FTSET3 = this.getFieldValue("FTSET3");
  const FTSET4 = this.getFieldValue("FTSET4");
  const FTSET5 = this.getFieldValue("FTSET5");
  const FTSET6 = this.getFieldValue("FTSET6");
  const codeFTSET = ` --icount={${FTSET1},${FTSET2},${FTSET3},${FTSET4},${FTSET5},${FTSET6}}`;
  // TODO: Assemble Robot into code variable.
  const code = "coroffset" + codeB + codeFTSET;
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}\n`;
};

Blockly.Robot.FCMonitor = function (block) {
  const MDOE = this.getFieldValue("DROPDOWN1");
  const OFFSET = this.getFieldValue("DROPDOWN2");
  const B1 = this.getFieldValue("B1");
  const B2 = this.getFieldValue("B2");
  const B3 = this.getFieldValue("B3");
  const B4 = this.getFieldValue("B4");
  const B5 = this.getFieldValue("B5");
  const B6 = this.getFieldValue("B6");
  const codeA = ` --mode=${MDOE.split("").pop()}`;
  let codeB = "";
  if (OFFSET === "select") {
    codeB = ` --offset={${B1},${B2},${B3},${B4},${B5},${B6}}`;
  } else {
    codeB = ` --offset=${OFFSET.split("").pop()}`;
  }
  // TODO: Assemble Robot into code variable.
  const code = "FCMonitor" + codeA + codeB;
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}\n`;
};

Blockly.Robot.saveproxml = function (block) {
  const B1 = this.getFieldValue("B1");
  const code = `saveproxml --name=${B1}`;
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}\n`;
};

Blockly.Robot.loadproxml = function (block) {
  const B1 = this.getFieldValue("B1");
  const code = `loadproxml --name=${B1}`;
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}\n`;
};

// set指令
Blockly.Robot.set_speed = function (block) {
  const textV1 = block.getFieldValue("speed");
  let textTer = block.getFieldValue("SPEED0");
  const textTcp = block.getFieldValue("SPEED1");
  let textOri = block.getFieldValue("SPEED2");
  let textExjR = block.getFieldValue("SPEED3");
  const textExjL = block.getFieldValue("SPEED4");

  textTer = (parseFloat(textTer) / 100).toString();
  textOri = ((parseFloat(textOri) / 360) * Math.PI).toString();
  textExjR = ((parseFloat(textExjR) / 360) * Math.PI).toString();

  const code =
    "set " +
    textV1 +
    "={" +
    textTer +
    "," +
    textTcp +
    "," +
    textOri +
    "," +
    textExjR +
    "," +
    textExjL +
    "}" +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.set_zone = function (block) {
  const textV1 = block.getFieldValue("zone");
  const textTer = block.getFieldValue("DIS");
  const textPer = block.getFieldValue("PER");
  const switchTer = (parseFloat(textTer) / 1000).toString();
  const switchPer = (parseFloat(textPer) / 100).toString();

  // TODO: Assemble Robot into code variable.
  const code = "set " + textV1 + "={" + switchTer + "," + switchPer + "}\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};

Blockly.Robot.set_jointtarget = function (block) {
  let textInput = block.getFieldValue("Input");
  const valueName = Blockly.Robot.valueToCode(
    block,
    "NAME",
    Blockly.Robot.ORDER_ATOMIC
  );

  textInput =
    (textInput[0] < "0" || textInput[0] > "9") && textInput.indexOf(".") === -1
      ? block.workspace.fileName + "." + textInput
      : textInput;
  const code = "set " + textInput + "=" + valueName;
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}\n`;
};

Blockly.Robot.set_robtarget = function (block) {
  const code =
    "set " +
    block.getFieldValue("RobTarget") +
    " = " +
    block.getFieldValue("RobTarget1") +
    " + " +
    block.getFieldValue("RobTarget2") +
    "\n";
  block.data = ++block.workspace.generateId;
  return `${block.data}:${code}`;
};
