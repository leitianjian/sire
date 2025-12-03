import { createSelector, createSelectorCreator } from "reselect";
import memoize from "lodash/memoize";
import isNumber from "lodash/isNumber";
import isArray from "lodash/isArray";
import { Model } from "../ethercat/Model";
import { Display3dConfig } from "~/shared/cells/Display3d/types";
import { RobotState, RootState } from "./reducers";
import { Matrix4Array, SireGeometry } from "~/types/lib";

// hash equal //
const hashFn = (...args: any): string => {
  return args.reduce(
    (acc: string, val: any) => acc + "-" + JSON.stringify(val),
    ""
  );
};
export const hashEqualSelectorCreator = createSelectorCreator(memoize, hashFn);

/// ///////////////////////////////////////////// dashboard //////////////////////////////
export const getDashboards = ({ config }: { config: any }): any =>
  config?.dashboards;
export const getLayoutCols = ({ config }: { config: any }): any =>
  config?.layoutConfig?.cols;
export const getLayoutRowHeight = ({ config }: { config: any }): any =>
  config?.layoutConfig?.rowHeight;
export const getLayoutMargin = ({ config }: { config: any }): any =>
  config?.layoutConfig?.margin;
export const getLayoutContainerPadding = ({ config }: { config: any }): any =>
  config?.layoutConfig?.containerPadding;

/// ////////////////////////////////////////////////////////////////////////////////////
export const getPathname = (state: any): string => state.router.pathname;

const getFullDashboards = (state: any): any => state.config.dashboards;
export const getDashboardAndCells = (state: any, id: string): any => {
  const dashboards = getFullDashboards(state);
  return dashboards?.filter((d: any) => d.i === id)[0];
};

export const getCurrentDashboardAndCells = createSelector(
  getFullDashboards,
  getPathname,
  (dashboards, pathname) => {
    if (pathname !== "/" && pathname.includes("/dashboards/")) {
      const id = pathname.replace("/dashboards/", "");
      const dashboard = dashboards?.filter((d: any) => d.i === id)[0];
      // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
      if (dashboard) {
        return dashboard;
      }
    }
    return dashboards?.[2];
  }
);

const getlocalestateselector = (state: any): any => state.config.localestate;
export const getlocalestate = createSelector(
  getlocalestateselector,
  (localestate): string => {
    if (localestate == null) {
      localestate = window.localStorage.getItem("locale") ?? "chinese";
      return localestate;
    } else {
      return localestate;
    }
  }
);

export const getModelPath = (state: any): any => state.config.Path;
export const getPath = createSelector(getModelPath, (path) => {
  if (path == null) {
    path = "";
    return path;
  } else {
    return path;
  }
});

export const getConfigModelState = (state: any): any =>
  state.config.configModel;

export const getConfigModel = createSelector(
  getConfigModelState,
  (ConfigModel) => {
    if (ConfigModel == null) {
      console.log("执行getConfigModel失败");
      return ConfigModel;
    } else {
      console.log("执行getConfigModel成功");
      return ConfigModel;
    }
  }
);

/// ///////////////////////////////////////////// robot selector //////////////////////////////
export const getDataSources = (state: any, { cell }: any): any[] => {
  const options = { ...cell.options };
  const channel = options.channel === "command" ? cell.i : "get";
  const result = state.robot.resultsByChannel[channel];
  // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
  if (result) {
    return Object.keys(result).filter((key) => {
      if (isNumber(result[key])) {
        return true;
      }
      if (isArray(result[key]) && isNumber(result[key][0])) {
        return true;
      }
      return false;
    });
  } else {
    return [];
  }
};

export const getMemoizeDataSources = hashEqualSelectorCreator(
  getDataSources,
  (result) => result
);

export const getDataIndexesBySource = (state: any, { cell }: any): any => {
  const options = { ...cell.options };
  const channel = options.channel === "command" ? cell.i : "get";
  const result = state.robot.resultsByChannel[channel];
  const indexes: any = {};
  // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
  if (result) {
    const sources = getDataSources(state, { cell });
    sources.forEach((source: string) => {
      if (isArray(result[source]) && isNumber(result[source][0])) {
        indexes[source] = result[source].map((d: any, i: number) => i);
      }
    });
  }
  return indexes;
};

export const getMemoizeDataIndexesBySource = hashEqualSelectorCreator(
  getDataIndexesBySource,
  (result) => result
);

/// //////////////////////////////////////////// SOCKET //////////////////////////////////////////////////////////
export const getWebsocketConnected = (state: any): any =>
  state.socket.websocketConnected;

export const getCurrentCommand = (state: any): any =>
  state.socket.currentCommand;

export const getCommandSendInterval = ({ config }: any): any =>
  config.ws?.commandSendInterval;
export const getGetInterval = ({ config }: any): any => config.ws?.getInterval;
export const getCommandSendDelay = ({ config }: any): any =>
  config.ws?.commandSendDelay;

/// //////////////////////////////////////////// SERVER //////////////////////////////////////////////////////////
const getCsStarted = (state: any): any => state.server.cs_is_started;
export const getMemoizeCsStarted = hashEqualSelectorCreator(
  getCsStarted,
  (started) => started
);

const getAI = (state: any): any => state.server.ai;
export const getMemoizeAI = hashEqualSelectorCreator(getAI, (ai) => ai);
const getDI = (state: any): any => state.server.di;
export const getMemoizeDI = hashEqualSelectorCreator(getDI, (di) => di);
const getDO = (state: any): any => state.server.do;
export const getMemoizeDO = hashEqualSelectorCreator(getDO, (DO) => DO);

const getEndPE = (state: any): any => state.server.end_pe;
export const getMemoizeEndPE = hashEqualSelectorCreator(
  getEndPE,
  (endPE) => endPE
);
const getEndPQ = (state: any): any => state.server.end_pq;
export const getMemoizeEndPQ = createSelector(getEndPQ, (endPQ) => endPQ);

const getMoitionState = (state: any): any => state.server.motion_state;
export const getMemoizeMotionState = hashEqualSelectorCreator(
  getMoitionState,
  (motionState) => motionState
);
const getMotionPos = (state: any): any => state.server.motion_pos;
export const getMemoizeMotionPos = hashEqualSelectorCreator(
  getMotionPos,
  (motionPos) => motionPos
);
const getMotionVel = (state: any): any => state.server.motion_vel;
export const getMemoizeMotionVel = hashEqualSelectorCreator(
  getMotionVel,
  (motionVel) => motionVel
);
const getMotionAcc = (state: any): any => state.server.motion_acc;
export const getMemoizeMotionAcc = createSelector(
  getMotionAcc,
  (motionAcc) => motionAcc
);
const getMotionToq = (state: any): any => state.server.motion_toq;
export const getMemoizeMotionToq = hashEqualSelectorCreator(
  getMotionToq,
  (motionToq) => motionToq
);

const getForceData = (state: any): any => state.server.force_data;
export const getMemoizeForceData = hashEqualSelectorCreator(
  getForceData,
  (forceData) => forceData
);

const getPartPQ = (state: any): any => state.server.part_pq;
export const getMemoizePartPQ = hashEqualSelectorCreator(
  getPartPQ,
  (partPQ) => partPQ
);

const getSlaveAlState = (state: any): any => state.server.slave_al_state;
export const getMemoizeSlaveAlState = hashEqualSelectorCreator(
  getSlaveAlState,
  (slaveAlState) => slaveAlState
);
const getSlaveLinkNum = (state: any): any => state.server.slave_link_num;
export const getMemoizeSlaveLinkNum = hashEqualSelectorCreator(
  getSlaveLinkNum,
  (slaveLinkNum) => slaveLinkNum
);
const getSlaveOnlineState = (state: any): any =>
  state.server.slave_online_state;
export const getMemoizeSlaveOnlineState = hashEqualSelectorCreator(
  getSlaveOnlineState,
  (slaveOnlineState) => slaveOnlineState
);

export const getReturnCode = (state: any): any => state.server.return_code;

export const getCsErrCode = (state: any): any => state.server.cs_err_code;
export const getMemoizeCsErrCode = hashEqualSelectorCreator(
  getCsErrCode,
  (csErrCode) => csErrCode
);
export const getCsErrMsg = (state: any): any => state.server.cs_err_msg;
export const getMemoizeCsErrMsg = hashEqualSelectorCreator(
  getCsErrMsg,
  (csErrMsg) => csErrMsg
);

export const getStateCode = (state: any): any => state.server.state_code;
export const getCurrentPlan = (state: any): any => state.server.current_plan;
export const getCurrentPlanId = (state: any): any =>
  state.server.current_plan_id;

export const getProgramFile = (state: any): any => state.server.file;
export const getProgramLine = (state: any): any => state.server.line;
export const getProErrCode = (state: any): any => state.server.pro_err_code;
export const getProErrMsg = (state: any): any => state.server.pro_err_msg;

/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
export const getToolIds = (state: any): any => state.robot.toolIds;
export const getWobjIds = (state: any): any => state.robot.wobjIds;

export const getCurrentToolId = (state: any): any => state.robot.currentToolId;
export const getCurrentWobjId = (state: any): any => state.robot.currentWobjId;
export const getCurrentMotionVelPercent = (state: any): any =>
  // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
  state.robot.currentMotionVelPercent || 1;

export const getRobotModels = (state: any): any => state.robot.robotModels;

export const getCreatingRobotModel = (state: any): any =>
  state.robot.creatingRobotModel;
export const getCreatingRobotModelError = (state: any): any =>
  state.robot.creatingRobotModelError;
export const getCreatingRobotModelResult = (state: any): any =>
  state.robot.creatingRobotModelResult;
export const getCreateRobotModelModalVisible = (state: any): any =>
  state.robot.createRobotModelModalVisible;
export const getCurrentRobotBrand = (state: any): any =>
  state.robot.currentRobotBrand;
export const getCurrentRobotModel = (state: any): any =>
  state.robot.currentRobotModel;

export const getXml = (state: any): any => state.robot.xml;

export const GetPartsPath = createSelector(getXml, (xml) => {
  // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
  if (xml) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(xml, "text/xml");
    const masterNode = xmlDoc.getElementsByTagName("Model"); // 获取到EthercatController节点
    // console.log("master node", masterNode.partPoolElement);
    // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
    if (masterNode?.[0]) {
      // 如果存在
      const model = Model.FromXML(masterNode[0]).toJSON(); // 转化为json数据格式
      // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
      if (model?.partPoolElement?.parts) {
        return model.partPoolElement.parts
          .filter(
            (p: any) => p.geometryPoolElement?.fileGeometry?.graphicFilePath
          )
          .map((p: any) => p.geometryPoolElement.fileGeometry.graphicFilePath);
      }
    }
  }
  return null;
});

export const getGeometryPool = (state: RootState): SireGeometry[] | null => state.robot.geometry_pool;
export const getGeometryPm = (state: RootState): Matrix4Array[] | null => state.robot.geometry_pm;
export const getPartPQInit = (state: RootState): number[][] | null => state.robot.part_pq_init;
export const getViewerConfig = (state: RootState, defaultConfig: Display3dConfig): Display3dConfig => {
  if (state.robot.viewerConfig == null) {
    return defaultConfig;
  } else {
    return state.robot.viewerConfig;
  }
};

export const getLogNameList = (state: any): any => state.robot.logFileNameList;
export const getLogPath = (state: any): any => state.robot.logFilePath;
export const getLogFileContent = (state: any): any => state.robot.logContent;
