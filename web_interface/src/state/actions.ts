// const cmdTimestamp = Date.now()
import { ActionCreator, PayloadAction } from "@reduxjs/toolkit";
import { CellConfig, DashboardConfig } from "~/types/lib";
import * as actions from "./constants";

function action<T>(
  actionCreator: ActionCreator<PayloadAction<T, string>>,
  payload?: T
): PayloadAction<T, string> {
  return actionCreator(payload);
}

// configs //
interface InterfaceConfig {
  request: () => PayloadAction<void, string>;
  success: (response: any) => PayloadAction<any, string>;
  failure: (error: string) => PayloadAction<string, string>;
}
export const interfaceConfig: InterfaceConfig = {
  request: () => actions.InterfaceConfigFetchRequestAction(),
  success: (response: any) => {
    response = { ...response };
    if (response.dashboards != null) {
      console.log("test response.dashboard: ", response.dashboards);
      response.dashboards = response.dashboards.map((d: DashboardConfig) => {
        d = { ...d };
        d.cells = d.cells.map((c) => {
          c = { ...c };
          try {
            c.optionsJson = JSON.parse(c.options);
          } catch (e) {
            c.optionsJson = {};
          }
          return c;
        });
        return d;
      });
    }
    return actions.InterfaceConfigFetchSuccessAction({ response });
  },
  failure: (error: string): PayloadAction<string, string> =>
    actions.InterfaceConfigFetchFailureAction(error),
};
export const loadInterfaceConfig = (
  requiredFields = []
): PayloadAction<any, string> =>
  actions.LoadInterfaceConfigAction({ requiredFields });

// full screen //
export const actFullScreen = (): PayloadAction<void, string> => {
  void document.getElementById("body")?.requestFullscreen();
  return actions.FullScreenAction();
};

export const actFullScreenInverse = (): PayloadAction<void, string> => {
  void document.exitFullscreen();
  return actions.FullScreenInverseAction();
};
// dashboards //
export const dashboardUpdate = {
  request: (dashboard: DashboardConfig) =>
    action(actions.DashboardUpdateRequestAction, { dashboard }),
  success: (dashboard: DashboardConfig) => {
    dashboard = { ...dashboard };
    dashboard.cells = dashboard.cells.map((cell: CellConfig) => {
      cell = { ...cell };
      try {
        cell.optionsJson = JSON.parse(cell.options);
      } catch (e) {
        cell.optionsJson = {};
      }
      return cell;
    });
    return action(actions.DashboardUpdateSuccessAction, { dashboard });
  },
  failure: (error: string) =>
    action(actions.DashboardUpdateFailureAction, error),
};
export const updateDashboard = (
  dashboard: DashboardConfig
): PayloadAction<any, string> => {
  return action(actions.DashboardUpdateAction, { dashboard });
};

// create cell //
export const cellCreate = {
  request: (dashboardId: string) =>
    action(actions.CellCreateRequestAction, { dashboardId }),
  success: (dashboardId: string, cell: CellConfig) => {
    cell = { ...cell };
    try {
      cell.optionsJson = JSON.parse(cell.options);
    } catch (e) {
      cell.optionsJson = {};
    }
    return action(actions.CellCreateSuccessAction, { dashboardId, cell });
  },
  failure: (error: string) => action(actions.CellCreateFailureAction, error),
};
export const actCreateCell = (
  dashboardId: string,
  cell: CellConfig
): PayloadAction<any, string> => {
  return action(actions.CellCreateAction, { dashboardId, cell });
};

export const actToggleCreateCellModal = (): PayloadAction<void, string> =>
  action(actions.ToggleCreateCellModalAction);

// delete cell //
export const cellDelete = {
  request: (dashboardId: string, cellId: string): PayloadAction<any, string> =>
    action(actions.CellDeleteRequestAction, { dashboardId, cellId }),
  success: (dashboardId: string, cell: CellConfig) =>
    action(actions.CellDeleteSuccessAction, { dashboardId, cell }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.CellDeleteFailureAction, error),
};
export const deleteCell = (
  dashboardId: string,
  cellId: string
): PayloadAction<any, string> =>
  action(actions.CellDeleteAction, { dashboardId, cellId });

// routers //
export const updateRouterState = (state: any): PayloadAction<any, string> =>
  action(actions.UpdateRouterStateAction, { state });

// error msgs //
export const resetErrorMessage = (): PayloadAction<void, string> =>
  action(actions.ResetErrorMsgAction);

// export const RESET_LOADING_MESSAGE = 'RESET_LOADING_MESSAGE'
// NAVIGATE //
export const actNavigate = (pathname: string): PayloadAction<any, string> =>
  action(actions.NavigateAction, pathname);

// server actions //
export const sendCmd = (
  cmd: string,
  successCallback: Function,
  failCallback: Function
): PayloadAction<any, string> => {
  return action(actions.SendCmdAction, { cmd, successCallback, failCallback });
};

export const sendCmdSilence = (
  cmd: string,
  successCallback?: Function,
  failCallback?: Function
): PayloadAction<any, string> => {
  return action(actions.SendCmdSilenceAction, { cmd, successCallback, failCallback });
};

export const recordSendCmd = (cmd: string): PayloadAction<any, string> => {
  return action(actions.RecordSendCmdAction, { cmd });
};

export const sendCmdResponse = (data: any): PayloadAction<any, string> => {
  return action(actions.SendCmdResponseAction, data);
};

export const updateWebsocketStatus = ({
  websocketConnected,
}: any): PayloadAction<any, string> =>
  action(actions.UpdateWebsocketStatusAction, { websocketConnected });

export const updateServerStateRequest = ({
  toolId,
  wobjId,
}: {
  toolId: number;
  wobjId: number;
}): PayloadAction<any, string> =>
  action(actions.UpdateServerStateRequestAction, { toolId, wobjId });
export const updateServerStateSuccess = (
  result: any
): PayloadAction<any, string> =>
  action(actions.UpdateServerStateSuccessAction, { ...result });
export const updateServerStateFailure = (
  error: any
): PayloadAction<string, string> =>
  action(actions.UpdateServerStateFailureAction, error);

export const updatePartPQRequest = (payload: any): PayloadAction<any, string> => action(actions.UpdatePartPQRequestAction);
export const updatePartPQSuccess = (
  result: any
): PayloadAction<any, string> =>
  action(actions.UpdatePartPQSuccessAction, { ...result });
export const updatePartPQFailure = (
  error: any
): PayloadAction<string, string> =>
  action(actions.UpdatePartPQFailureAction, error);

export const display3dInitRequest = (): PayloadAction<any, string> => action(actions.Display3dInitRequestAction);
export const display3dInitSuccess = (
  result: any
): PayloadAction<any, string> =>
  action(actions.Display3dInitSuccessAction, { ...result });
export const display3dInitFailure = (
  error: any
): PayloadAction<string, string> =>
  action(actions.Display3dInitFailureAction, error);

// robots related variables //
export const actUpdateMotionVelPercent = (
  motionVelPercent: number
): PayloadAction<any, string> =>
  action(actions.UpdateMotionVelPercentAction, { motionVelPercent });
export const actUpdateToolId = (toolId: number): PayloadAction<any, string> =>
  action(actions.UpdateToolIdAction, { toolId });
export const actUpdateWobjId = (wobjId: number): PayloadAction<any, string> =>
  action(actions.UpdateWobjIdAction, { wobjId });
export const actUpdateCoordinateId = (
  coordinateId: number
): PayloadAction<any, string> =>
  action(actions.UpdateCoordinateIdAction, { coordinateId });

// auto or manual state //
export const dispatchButtonState = (name: string): PayloadAction<any, string> =>
  action(actions.DispatchButtonStateAction, { name });
export const dispatchAutoState = (name: string): PayloadAction<any, string> =>
  action(actions.DispatchAutoStateAction, { name });

// 加载kaanh里面的speed和wobj等变量
export const configModel = {
  request: () => action(actions.FetchConfigModelRequestAction, {}),
  success: (model: any) =>
    action(actions.FetchConfigModelSuccessAction, { model }),
  failure: (error: string) =>
    action(actions.FetchConfigModelFailureAction, error),
};
export const fetchConfigModel = (): PayloadAction<void, string> =>
  action(actions.FetchConfigModelAction);

// models //
export const robotModels = {
  request: () => action(actions.RobotModelsFetchRequestAction, {}),
  success: (robotModels: any) =>
    action(actions.RobotModelsFetchSuccessAction, { robotModels }),
  failure: (error: string) =>
    action(actions.RobotModelsFetchFailureAction, error),
};
export const loadRobotModels = (
  requiredFields: any[] = []
): PayloadAction<any, string> =>
  action(actions.LoadRobotModelsAction, { requiredFields });
export const robotModelPartUploadSuccess = (
  model: any
): PayloadAction<any, string> =>
  action(actions.RobotModelPartUploadSuccessAction, { model });

export const robotModelPartDelete = {
  request: () => action(actions.RobotModelPartDeleteRequestAction),
  success: (model: any) =>
    action(actions.RobotModelPartDeleteSuccessAction, { model }),
  failure: (error: string) =>
    action(actions.RobotModelPartDeleteFailureAction, error),
};
export const deleteRobotModelPart = (
  brand: string,
  model: string,
  part: string
): PayloadAction<any, string> =>
  action(actions.DeleteRobotModelPartAction, { brand, model, part });

export const robotModelDelete = {
  request: () => action(actions.RobotModelDeleteRequestAction, {}),
  success: (model: any) =>
    action(actions.RobotModelDeleteSuccessAction, { model }),
  failure: (error: string) =>
    action(actions.RobotModelDeleteFailureAction, error),
};
export const deleteRobotModel = (
  brand: string,
  model: string
): PayloadAction<any, string> =>
  action(actions.DeleteRobotModelAction, { brand, model });

export const robotModelCreate = {
  request: (model: any) =>
    action(actions.RobotModelCreateRequestAction, { model }),
  success: (model: any) =>
    action(actions.RobotModelCreateSuccessAction, { model }),
  failure: (error: string) =>
    action(actions.RobotModelCreateFailureAction, error),
};
export const createRobotModel = (model: any): PayloadAction<any, string> =>
  action(actions.CreateRobotModelAction, { model });

export const showCreateRobotModelModal = (): PayloadAction<void, string> =>
  action(actions.ShowCreateRobotModelModalAction);
export const hideCreateRobotModelModal = (): PayloadAction<void, string> =>
  action(actions.HideCreateRobotModelModalAction);

export const selectCurrentRobotBrand = (
  brand: string
): PayloadAction<any, string> =>
  action(actions.SelectCurrentRobotBrandAction, { brand });
export const selectCurrentRobotModel = (
  model: string
): PayloadAction<any, string> =>
  action(actions.SelectCurrentRobotModelAction, { model });

// ethercat configs //
export const esiFileUpload = {
  request: (): PayloadAction<void, string> =>
    action(actions.UploadESIFileRequestAction),
  success: (path: any): PayloadAction<any, string> =>
    action(actions.UploadESIFileSuccessAction, { path }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.UploadESIFileFailureAction, error),
};

export const esiPath = {
  request: () => action(actions.ESIPathFetchRequestAction),
  success: (path: any): PayloadAction<any, string> =>
    action(actions.ESIPathFetchSuccessAction, { path }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.ESIPathFetchFailureAction, error),
};
export const loadEsiPath = (): PayloadAction<void, string> =>
  action(actions.LoadESIPathAction);

export const configXml = {
  request: () => action(actions.ConfigXMLFetchRequestAction),
  success: (xml: any) => action(actions.ConfigXMLFetchSuccessAction, { xml }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.ConfigXMLFetchFailureAction, error),
};
export const loadConfigXml = (): PayloadAction<void, string> =>
  action(actions.LoadConfigXMLAction);

// programs //
export const programsLoad = {
  request: () => action(actions.ProgramsFetchRequestAction),
  success: (programs: any) =>
    action(actions.ProgramsFetchSuccessAction, { programs }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.ProgramsFetchFailureAction, error),
};
export const loadPrograms = (): PayloadAction<void, string> =>
  action(actions.LoadProgramsAction);

export const programCreate = {
  request: (): PayloadAction<void, string> =>
    action(actions.ProgramCreateRequestAction),
  success: (program: any) =>
    action(actions.ProgramCreateSuccessAction, { program }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.ProgramCreateFailureAction, error),
};
export const createProgram = ({ name }: any): PayloadAction<any, string> =>
  action(actions.CreateProgramAction, { name });
export const toggleCreateProgramModal = (): PayloadAction<void, string> =>
  action(actions.ToggleCreateProgramModalAction);

export const programUpdate = {
  request: (name: string, program: any) =>
    action(actions.ProgramUpdateRequestAction, { name, program }),
  success: (program: any) =>
    action(actions.ProgramUpdateSuccessAction, { program }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.ProgramUpdateFailureAction, error),
};
export const updateProgram = (
  name: string,
  program: any
): PayloadAction<any, string> =>
  action(actions.UpdateProgramAction, { name, program });

export const programDelete = {
  request: (program: any): PayloadAction<any, string> =>
    action(actions.ProgramDeleteRequestAction, { program }),
  success: (program: any): PayloadAction<any, string> =>
    action(actions.ProgramDeleteSuccessAction, { program }),
  failure: (error: string): PayloadAction<string, string> =>
    action(actions.ProgramDeleteFailureAction, error),
};
export const deleteProgram = (name: string): PayloadAction<any, string> =>
  action(actions.DeleteProgramAction, { name });
export const toggleDeleteProgramModal = (
  name: string
): PayloadAction<any, string> =>
  action<any>(actions.ToggleDeleteProgramModalAction, { name });

export const programRename = {
  request: (name: string, newName: string, copy: boolean) =>
    action(actions.ProgramRenameRequestAction, { name, newName, copy }),
  success: (name: string, program: string, copy: boolean) =>
    action(actions.ProgramRenameSuccessAction, { name, program, copy }),
  failure: (error: string) => action(actions.ProgramRenameFailureAction, error),
};
export const renameProgram = (
  name: string,
  newName: string,
  copy: boolean
): PayloadAction<any, string> =>
  action(actions.RenameProgramAction, { name, newName, copy });
export const toggleEditProgramName = (
  name: string
): PayloadAction<any, string> =>
  action(actions.ToggleEditProgramAction, { name });
export const resetRenameProgramStatus = (): PayloadAction<void, string> =>
  action(actions.ResetRenameProgramStatusAction);

export const actCopyBlock = (blkXml: any): PayloadAction<any, string> => {
  return action(actions.CopyBlockAction, { blkXml });
};

// locale //
export const dispatchLocaleState = (value: any): PayloadAction<any, string> =>
  action(actions.LocaleStateAction, { value });

// 获取log日志的列表
export const getLog = {
  request: () => action(actions.LogRequestAction),
  success: (logFiles: any) => action(actions.LogSuccessAction, { logFiles }),
  failure: (error: string) => action(actions.LogFailureAction, error),
};
export const dispatchLogName = (num: any): PayloadAction<any, string> =>
  action(actions.DispatchLogFileNameAction, { num });

// 获取log日志的content
export const getLogContent = {
  request: () => action(actions.LogContentRequestAction),
  success: (content: any) =>
    action(actions.LogContentSuccessAction, { content }),
  failure: (error: string) => action(actions.LogContentFailureAction, error),
};
export const diapatchLogFileContent = (
  name: string,
  pageNum: number
): PayloadAction<any, string> =>
  action(actions.DispatchLogFileContentAction, { name, pageNum });

// DE techs
// export const FETCH_OBJ_PICTURE_LIST_REQUEST = 'FETCH_OBJ_PICTURE_LIST_REQUEST'
// export const FETCH_OBJ_PICTURE_LIST_SUCCESS = 'FETCH_OBJ_PICTURE_LIST_SUCCESS'
// export const FETCH_OBJ_PICTURE_LIST_FAILURE = 'FETCH_OBJ_PICTURE_LIST_FAILURE'
// export const fetchObjPictureList = {
//   request: () => action(FETCH_OBJ_PICTURE_LIST_REQUEST),
//   success: (content) => action(FETCH_OBJ_PICTURE_LIST_SUCCESS, { content: content.fileList }),
//   failure: (error) => action(FETCH_OBJ_PICTURE_LIST_FAILURE, { error })
// }
// export const DISPATH_FETCH_OBJ_PICTURE_LIST = 'DISPATH_FETCH_OBJ_PICTURE_LIST'
// export const dispatchFetchObjPictureList = () => action(DISPATH_FETCH_OBJ_PICTURE_LIST)

// export const UPDATE_OBJ_PICTURE_LIST = 'UPDATE_OBJ_PICTURE_LIST'
// export const dispatchUpdateObjPictureList = (value) => action(UPDATE_OBJ_PICTURE_LIST, { value })
