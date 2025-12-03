import { createSelector } from "reselect";
import EthercatController from "../../../ethercat/EthercatController";
import { EthercatMaster } from "../../../ethercat/EthercatMaster";
import EthercatSlave from "../../../ethercat/EthercatSlave";

export const getSetEsiPathResult = (state, { cell }) =>
  state.robot.resultsByChannel[`${cell.i}-setesipath`];
export const getScanSlaveResult = (state, { cell }) =>
  state.robot.resultsByChannel[`${cell.i}-scanslave`]; // || (state.robot.xml && {controller_xml:state.robot.xml})
export const getDefaultXmlResult = (state) => state.robot.xml;
export const getPdoListResultForDevice = (state, { cell }) =>
  state.robot.resultsByChannel[`${cell.i}-pdolist-for-device`];
export const getPdoListResultForSlave = (state, { cell }) =>
  state.robot.resultsByChannel[`${cell.i}-pdolist-for-slave`];
export const getSaveResult = (state, { cell }) =>
  state.robot.resultsByChannel[`${cell.i}-save`];

export const getEsiFileUploadingRequest = (state) =>
  state.robot.esiFileUploadingRequest;
export const getEsiFileUploadingSuccess = (state) =>
  state.robot.esiFileUploadingSuccess;
export const getEsiFileUploadingFailure = (state) =>
  state.robot.esiFileUploadingFailure;

export const getScanSlaveJsonResult = createSelector(
  getScanSlaveResult,
  getDefaultXmlResult,
  (result, xml) => {
    if (!!result && !!result.controller_xml) {
      xml = result.controller_xml;
    }
    if (xml) {
      const parser = new DOMParser();
      const xmlDoc = parser.parseFromString(xml, "text/xml");
      const masterNode = xmlDoc.getElementsByTagName("EthercatController"); // 获取到EthercatController节点
      if (masterNode && masterNode[0]) {
        // 如果存在
        return EthercatController.FromXML(masterNode[0]).toJSON(); // 转化为json数据格式
      }
    }
    return null;
  }
);

export const getPosRange = createSelector(getDefaultXmlResult, (xml) => {
  if (xml) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(xml, "text/xml");
    const masterNode = xmlDoc.getElementsByTagName("EthercatController"); // 获取到EthercatController节点
    if (masterNode && masterNode[0]) {
      // 如果存在
      return EthercatController.FromXML(masterNode[0]).toJSON(); // 转化为json数据格式
    }
  }
  return null;
});

export const getSetEsiPathJSONResult = createSelector(
  getSetEsiPathResult,
  (result) => {
    if (!!result && !!result.device_list_xml) {
      // console.log("result",result)
      const parser = new DOMParser();
      const xmlDoc = parser.parseFromString(result.device_list_xml, "text/xml");
      const masterNode = xmlDoc.getElementsByTagName("ethercat_controller");
      if (masterNode && masterNode[0]) {
        return EthercatMaster.FromXML(masterNode[0]).toJSON();
      }
    }
    return null;
  }
);

export const getPdoListForDeviceJSONResult = createSelector(
  getPdoListResultForDevice,
  (result) => {
    if (!!result && !!result.pdo_list_xml) {
      console.log("result", result);
      const parser = new DOMParser();
      const xmlDoc = parser.parseFromString(result.pdo_list_xml, "text/xml");
      const masterNode = xmlDoc.getElementsByTagName("EthercatSlave");
      if (masterNode && masterNode[0]) {
        return EthercatSlave.FromXML(masterNode[0]).toJSON();
      }
    }
    return null;
  }
);

export const getPdoListForSlaveJSONResult = createSelector(
  getPdoListResultForSlave,
  (result) => {
    // if (!!result && !!result.device_list_xml) {
    // }
    return null;
  }
);
