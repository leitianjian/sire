/* eslint-disable @typescript-eslint/strict-boolean-expressions */
/* eslint-disable @typescript-eslint/explicit-function-return-type */
import {
  take,
  put,
  call,
  fork,
  select,
  all,
  delay,
  takeLatest,
  takeEvery,
  SelectEffect,
  CallEffect,
  PutEffect,
} from "redux-saga/effects";
import isEmpty from "lodash/isEmpty";

import * as api from "./api";
import { history } from "./api";
import * as actionType from "./constants";
import * as actions from "./actions";
import store from "../index";

import { getDashboardAndCells } from "./selectors";

import {
  cellCreate,
  cellDelete,
  dashboardUpdate,
  configXml,
  configModel,
  programsLoad,
  programCreate,
  programDelete,
  programUpdate,
  programRename,
  getLog,
  getLogContent,
  // fetchObjPictureList,
} from "./actions";

import { websock } from "./websocket";
import { CellConfig, FetchError, FetchResponse } from "~/types/lib";
import { PayloadAction } from "@reduxjs/toolkit";

/// ///////////////////////////////////////////config sagas ////////////////////////////////////////
const { interfaceConfig } = actions;

function* fetchConfig() {
  yield put(interfaceConfig.request());
  const { response, error }: { response: any; error: string } = yield call(
    api.fetchInterfaceConfig
  );
  if (response != null) yield put(interfaceConfig.success(response));
  else yield put(interfaceConfig.failure(error));
}

function* loadConfig() {
  yield delay(50);
  yield call(fetchConfig);
}

function* loadConfigModel() {
  yield put(configModel.request());
  const { response, error }: { response: any; error: string } = yield call(
    api.loadConfigModel
  );
  if (response != null) yield put(configModel.success(response));
  else yield put(configModel.failure(error));
}

function* loadConfigXml() {
  yield put(configXml.request());
  const { response, error }: { response: string; error: string } = yield call(
    api.fetchConfigXml
  );
  if (response != null) yield put(configXml.success(response));
  else yield put(configXml.failure(error));
}

function* updateDashboard({
  payload,
}: PayloadAction<any, string>): Generator<
  | SelectEffect
  | CallEffect<true | FetchResponse<any> | FetchError>
  | PutEffect<PayloadAction<string, string>>,
  void,
  any
> {
  // delay可以产生debounce effect
  yield delay(500);
  const dashboard = yield select(getDashboardAndCells, payload.dashboard.i);
  if (!isEmpty(dashboard)) {
    yield put(dashboardUpdate.request(dashboard));
    const { response, error }: { response: any; error: string } = yield call(
      api.updateDashboard,
      dashboard
    );
    if (response != null) yield put(dashboardUpdate.success(response));
    else yield put(dashboardUpdate.failure(error));
  }
}

function* createCell({ payload }: PayloadAction<any, string>) {
  yield delay(50);
  const { dashboardId, cell }: { dashboardId: string; cell: CellConfig } =
    payload;
  yield put(cellCreate.request(dashboardId));
  const { response, error }: { response: any; error: string } = yield call(
    api.createCell,
    dashboardId,
    cell
  );
  if (response != null) yield put(cellCreate.success(dashboardId, response));
  else yield put(cellCreate.failure(error));
}

function* deleteCell({ payload }: PayloadAction<any, string>) {
  yield delay(50);
  const { dashboardId, cellId }: { dashboardId: string; cellId: string } =
    payload;
  yield put(cellDelete.request(dashboardId, cellId));
  const { response, error } = yield call(api.deleteCell, dashboardId, cellId);
  if (response != null) yield put(cellDelete.success(dashboardId, response));
  else yield put(cellDelete.failure(error));
}

function* watchUpdateDashboard() {
  yield takeLatest(actionType.UPDATE_DASHBOARD, updateDashboard);
}

function* watchCreateCell() {
  yield takeLatest(actionType.CREATE_CELL, createCell);
}

function* watchLoadConfig() {
  yield takeLatest(actionType.LOAD_INTERFACE_CONFIG, loadConfig);
}

function* watchDeleteCell() {
  yield takeLatest(actionType.DELETE_CELL, deleteCell);
}

function* watchLoadConfigXml() {
  yield takeLatest(actionType.LOAD_CONFIG_XML, loadConfigXml);
}

// 获取到speed和wobj参数
function* watchLoadConfigModel() {
  yield takeLatest(actionType.FETCH_CONFIG_MODEL, loadConfigModel);
}

/// /////////////////////////////////////////////////robot sagas//////////////////////////////////////////
const {
  robotModelPartDelete,
  robotModelDelete,
  robotModelCreate,
  robotModels,
  esiPath,
} = actions;

function* fetchRobotModels() {
  yield put(robotModels.request());
  const { response, error } = yield call(api.fetchRobotModels);
  if (response != null) yield put(robotModels.success(response));
  else yield put(robotModels.failure(error));
}

function* fetchESIPath() {
  yield put(esiPath.request());
  const { response, error } = yield call(api.fetchESIPath);
  if (response != null) yield put(esiPath.success(response.path));
  else yield put(esiPath.failure(error));
}

// 程序相关 //
function* fetchPrograms() {
  yield put(programsLoad.request());
  const { response, error } = yield call(api.fetchPrograms);

  if (response) yield put(programsLoad.success(response));
  else yield put(programsLoad.failure(error));
}

function* createProgram({ payload }: PayloadAction<any, string>) {
  yield put(programCreate.request());
  const { name }: { name: string } = payload;
  const { response, error } = yield call(api.createProgram, name);
  if (response) yield put(programCreate.success(response));
  else yield put(programCreate.failure(error));
}

function* updateProgram({ payload }: PayloadAction<any, string>) {
  yield delay(500);
  const { name, program }: { name: string; program: any } = payload;
  yield put(programUpdate.request(name, program));
  const { response, error } = yield call(api.updateProgram, name, program);
  if (response) yield put(programUpdate.success(response));
  else yield put(programUpdate.failure(error));
}

function* deleteProgram({ payload }: PayloadAction<any, string>) {
  const { name }: { name: string } = payload;
  yield put(programDelete.request(name));
  const { response, error } = yield call(api.deleteProgram, name);
  if (response) yield put(programDelete.success(response));
  else yield put(programDelete.failure(error));
}

function* renameProgram({ payload }: PayloadAction<any, string>) {
  const {
    name,
    newName,
    copy,
  }: { name: string; newName: string; copy: boolean } = payload;
  yield put(programRename.request(name, newName, copy));
  const { response, error } = yield call(
    api.renameProgram,
    name,
    newName,
    copy
  );
  if (response) yield put(programRename.success(name, response, copy));
  else yield put(programRename.failure(error));
}

// log 相关 //
function* fetchLog({ payload }: PayloadAction<any, string>) {
  yield put(getLog.request());
  const object: any = payload;
  const { response, error }: { response: any; error: string } = yield call(
    api.fetchLogName,
    object.num
  );
  if (response) yield put(getLog.success(response));
  else yield put(getLog.failure(error));
}

function* fetchLogContent({ payload }: PayloadAction<any, string>) {
  yield put(getLogContent.request());
  const { name, pageNum }: { name: string; pageNum: number } = payload;
  const { response, error }: { response: any; error: string } = yield call(
    api.fetchLogContent,
    name,
    pageNum
  );
  if (response) yield put(getLogContent.success(response));
  else yield put(getLogContent.failure(error));
}

function* deleteRobotModelPart({ payload }: PayloadAction<any, string>) {
  yield put(robotModelPartDelete.request());
  const { brand, model, part }: { brand: string; model: string; part: string } =
    payload;
  const { response, error }: { response: any; error: string } = yield call(
    api.deleteRobotModelPart,
    brand,
    model,
    part
  );
  if (response) yield put(robotModelPartDelete.success(response));
  else yield put(robotModelPartDelete.failure(error));
}

function* deleteRobotModel({ payload }: PayloadAction<any, string>) {
  yield put(robotModelDelete.request());
  const { brand, model }: { brand: string; model: string } = payload;
  const { response, error }: { response: any; error: string } = yield call(
    api.deleteRobotModel,
    brand,
    model
  );
  if (response) yield put(robotModelDelete.success(response));
  else yield put(robotModelDelete.failure(error));
}

function* createRobotModel({ payload }: PayloadAction<any, string>) {
  const { model }: { model: string } = payload;
  yield put(robotModelCreate.request(model));
  const { response, error }: { response: any; error: string } = yield call(
    api.createRobotModel,
    model
  );
  if (response) yield put(robotModelCreate.success(response));
  else yield put(robotModelCreate.failure(error));
}

// DE techs
// function* fetchObjPictureListFunc() {
//   yield put(fetchObjPictureList.request());
//   const { response, error }: { response: any; error: string } = yield call(
//     api.fetchObjPictureList
//   );
//   if (response) yield put(fetchObjPictureList.success(response));
//   else yield put(fetchObjPictureList.failure(error));
// }

/// ////////////////////////////////////watch//////////////////////////////////////////////////

function* watchLoadRobotModels() {
  yield takeLatest(actionType.LOAD_ROBOT_MODELS, fetchRobotModels);
}

function* watchDeleteRobotModelPart() {
  yield takeLatest(actionType.DELETE_ROBOT_MODEL_PART, deleteRobotModelPart);
}

function* watchDeleteRobotModel() {
  yield takeLatest(actionType.DELETE_ROBOT_MODEL, deleteRobotModel);
}

function* watchCreateRobotModel() {
  yield takeLatest(actionType.CREATE_ROBOT_MODEL, createRobotModel);
}

function* watchLoadESIPath() {
  yield takeLatest(actionType.LOAD_ESI_PATH, fetchESIPath);
}

function* watchLoadPrograms() {
  yield takeLatest(actionType.LOAD_PROGRAMS, fetchPrograms);
}

function* watchCreateProgram() {
  yield takeLatest(actionType.CREATE_PROGRAM, createProgram);
}

function* watchUpdateProgram() {
  yield takeLatest(actionType.UPDATE_PROGRAM, updateProgram);
}

function* watchDeleteProgram() {
  yield takeLatest(actionType.DELETE_PROGRAM, deleteProgram);
}

function* watchRenameProgram() {
  yield takeLatest(actionType.RENAME_PROGRAM, renameProgram);
}

function* watchFetchLog() {
  yield takeLatest(actionType.DISPATCH_LOG_FILE_NAME, fetchLog);
}

function* watchFetchLogContent() {
  yield takeLatest(actionType.DISPATH_LOG_FILE_CONTENT, fetchLogContent);
}

// function* watchFetchObjPictureList() {
//   yield takeLatest(
//     actionType.DISPATH_FETCH_OBJ_PICTURE_LIST,
//     fetchObjPictureListFunc
//   );
// }

/// ////////////////////////////////////////////////////websockets///////////////////////////////////////////
function* websocketSagaCallback({ payload }: PayloadAction<any, string>) {
  const { cmd, successCallback } = payload;
  websock.sendCmd(cmd, (msg) => {
    store.dispatch(
      actions.sendCmdResponse({
        cmd,
        data: msg.jsData,
        cmdResponseTime: Number(Date.now() - msg.reserved1),
      })
    );
    if (successCallback) successCallback(msg);
  });
  yield put(actions.recordSendCmd(cmd));
}

function* watchSendCmd() {
  yield takeEvery(actionType.SEND_CMD, websocketSagaCallback);
}

function* websocketSlienceSagaCallback({
  payload,
}: PayloadAction<any, string>) {
  const { cmd, successCallback } = payload;
  websock.sendCmd(cmd, (msg) => {
    if (successCallback) successCallback(msg);
  });
}

function* watchSendCmdSilence() {
  yield takeEvery(actionType.SEND_CMD_SILENCE, websocketSlienceSagaCallback);
}

function* updateServerStateCallback() {
  yield websock.sendCmd("get", (msg) => {
    store.dispatch(
      actions.updateServerStateSuccess({ server_state: msg.jsData })
    );
  });
}

function* watchUpdateServerStateSaga() {
  yield takeLatest(
    actionType.UPDATE_SERVER_STATE_REQUEST,
    updateServerStateCallback
  );
}

function* updatePartPQCallback() {
  yield websock.sendCmd("get --part_pq", (msg) => {
    store.dispatch(actions.updatePartPQSuccess({ server_state: msg.jsData }));
  });
}

function* watchUpdatePartPQSaga() {
  yield takeLatest(actionType.UPDATE_PARTPQ_REQUEST, updatePartPQCallback);
}

function* initDisplay3dCallback() {
  yield websock.sendCmd("display3d_init", (msg) => {
    store.dispatch(actions.display3dInitSuccess({ ...msg.jsData }));
  });
}

function* watchInitDisplay3d() {
  yield takeEvery(actionType.DISPLAY3D_INIT_REQUEST, initDisplay3dCallback);
}

/// /////////////////////////////////////////////////robot sagas//////////////////////////////////////////

// resuable fetch Subroutine
// entity :  dashboards | dashboards
// apiFn  : api.fetchDashboards | api.fetchRepo | ...
// id     : login | fullName
// url    : next page url. If not provided will use pass id to apiFn
// function* fetchEntity(entity, apiFn, id, url) {
//   yield put(entity.request(id));
//   const { response, error } = yield call(apiFn, url || id);
//   if (response) yield put(entity.success(id, response));
//   else yield put(entity.failure(id, error));
// }

// function* fetchEntities(entities, apiFn, url) {
//   yield put(entities.request());
//   const { response, error } = yield call(apiFn, url);
//   if (response) yield put(entities.success(response));
//   else yield put(entities.failure(error));
// }

// trigger router navigation via history
function* watchNavigate() {
  while (true) {
    const { pathname } = yield take(actionType.NAVIGATE);
    yield history.push(pathname);
  }
}

export default function* root() {
  yield all([
    fork(watchLoadConfig),
    fork(watchUpdateDashboard),
    fork(watchCreateCell),
    fork(watchDeleteCell),
    fork(watchLoadConfigXml),
    fork(watchInitDisplay3d),
    fork(watchLoadConfigModel),

    fork(watchLoadPrograms),
    fork(watchCreateProgram),
    fork(watchUpdateProgram),
    fork(watchDeleteProgram),
    fork(watchRenameProgram),

    fork(watchLoadESIPath),

    fork(watchLoadRobotModels),
    fork(watchDeleteRobotModelPart),
    fork(watchDeleteRobotModel),
    fork(watchCreateRobotModel),

    fork(watchFetchLog),
    fork(watchFetchLogContent),

    fork(watchSendCmd),
    fork(watchSendCmdSilence),
    fork(watchUpdateServerStateSaga),
    fork(watchUpdatePartPQSaga),

    fork(watchNavigate),

    // fork(watchFetchObjPictureList),
  ]);
}
