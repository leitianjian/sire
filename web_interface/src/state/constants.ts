import { createAction } from "@reduxjs/toolkit";

// configs //
export const LOAD_INTERFACE_CONFIG = "LOAD_INTERFACE_CONFIG";
export const LoadInterfaceConfigAction = createAction<any>(
  LOAD_INTERFACE_CONFIG
);
export const INTERFACE_CONFIG_FETCH_REQUEST = "INTERFACE_CONFIG_FETCH_REQUEST";
export const InterfaceConfigFetchRequestAction = createAction(
  INTERFACE_CONFIG_FETCH_REQUEST
);
export const INTERFACE_CONFIG_FETCH_FAILURE = "INTERFACE_CONFIG_FETCH_FAILURE";
export const InterfaceConfigFetchFailureAction = createAction<string>(
  INTERFACE_CONFIG_FETCH_FAILURE
);
export const INTERFACE_CONFIG_FETCH_SUCCESS = "INTERFACE_CONFIG_FETCH_SUCCESS";
export const InterfaceConfigFetchSuccessAction = createAction<any>(
  INTERFACE_CONFIG_FETCH_SUCCESS
);

// full screen //
export const FULL_SCREEN = "FULL_SCREEN";
export const FullScreenAction = createAction(FULL_SCREEN);
export const FULL_SCREEN_INVERSE = "FULL_SCREEN_INVERSE";
export const FullScreenInverseAction = createAction(FULL_SCREEN_INVERSE);

// dashboards //
export const UPDATE_DASHBOARD = "UPDATE_DASHBOARD";
export const DashboardUpdateAction = createAction<any>(UPDATE_DASHBOARD);
export const DASHBOARD_UPDATE_REQUEST = "DASHBOARD_UPDATE_REQUEST";
export const DashboardUpdateRequestAction = createAction<any>(
  DASHBOARD_UPDATE_REQUEST
);
export const DASHBOARD_UPDATE_FAILURE = "DASHBOARD_UPDATE_FAILURE";
export const DashboardUpdateFailureAction = createAction<string>(
  DASHBOARD_UPDATE_REQUEST
);
export const DASHBOARD_UPDATE_SUCCESS = "DASHBOARD_UPDATE_SUCCESS";
export const DashboardUpdateSuccessAction = createAction<any>(
  DASHBOARD_UPDATE_REQUEST
);
// create cell //
export const CREATE_CELL = "CREATE_CELL";
export const CellCreateAction = createAction<any>(CREATE_CELL);
export const CELL_CREATE_REQUEST = "CELL_CREATE_REQUEST";
export const CellCreateRequestAction = createAction<any>(CELL_CREATE_REQUEST);
export const CELL_CREATE_FAILURE = "CELL_CREATE_FAILURE";
export const CellCreateFailureAction =
  createAction<string>(CELL_CREATE_FAILURE);
export const CELL_CREATE_SUCCESS = "CELL_CREATE_SUCCESS";
export const CellCreateSuccessAction = createAction<any>(CELL_CREATE_SUCCESS);

export const TOGGLE_CREATE_CELL_MODAL = "TOGGLE_CREATE_CELL_MODAL";
export const ToggleCreateCellModalAction = createAction(
  TOGGLE_CREATE_CELL_MODAL
);

// delete cell //
export const DELETE_CELL = "DELETE_CELL";
export const CellDeleteAction = createAction<any>(DELETE_CELL);
export const CELL_DELETE_REQUEST = "CELL_DELETE_REQUEST";
export const CellDeleteRequestAction = createAction<any>(CELL_DELETE_REQUEST);
export const CELL_DELETE_FAILURE = "CELL_DELETE_FAILURE";
export const CellDeleteFailureAction =
  createAction<string>(CELL_DELETE_FAILURE);
export const CELL_DELETE_SUCCESS = "CELL_DELETE_SUCCESS";
export const CellDeleteSuccessAction = createAction<any>(CELL_DELETE_SUCCESS);

// routers //
export const UPDATE_ROUTER_STATE = "UPDATE_ROUTER_STATE";
export const UpdateRouterStateAction = createAction<any>(UPDATE_ROUTER_STATE);

// error msgs //
export const RESET_ERROR_MESSAGE = "RESET_ERROR_MESSAGE";
export const ResetErrorMsgAction = createAction<any>(UPDATE_ROUTER_STATE);

// NAVIGATE //
export const NAVIGATE = "NAVIGATE";
export const NavigateAction = createAction<string>(NAVIGATE);

// server actions //
export const SEND_CMD = "SEND_CMD";
export const SendCmdAction = createAction<any>(SEND_CMD);
export const SEND_CMD_SILENCE = "SEND_CMD_SILENCE";
export const SendCmdSilenceAction = createAction<any>(SEND_CMD_SILENCE);
export const RECORD_SEND_CMD = "RECORD_SEND_CMD";
export const RecordSendCmdAction = createAction<any>(RECORD_SEND_CMD);
export const SEND_CMD_RESPONSE = "SEND_CMD_RESPONSE";
export const SendCmdResponseAction = createAction<any>(SEND_CMD_RESPONSE);
export const UPDATE_WEBSOCKET_STATUS = "UPDATE_WEBSOCKET_STATUS";
export const UpdateWebsocketStatusAction = createAction<any>(
  UPDATE_WEBSOCKET_STATUS
);
// get
export const UPDATE_SERVER_STATE_REQUEST = "UPDATE_SERVER_STATE_REQUEST";
export const UpdateServerStateRequestAction = createAction<any>(
  UPDATE_SERVER_STATE_REQUEST
);
export const UPDATE_SERVER_STATE_SUCCESS = "UPDATE_SERVER_STATE_SUCCESS";
export const UpdateServerStateSuccessAction = createAction<any>(
  UPDATE_SERVER_STATE_SUCCESS
);
export const UPDATE_SERVER_STATE_FAILURE = "UPDATE_SERVER_STATE_FAILURE";
export const UpdateServerStateFailureAction = createAction<string>(
  UPDATE_SERVER_STATE_FAILURE
);

// get --part_pq
export const UPDATE_PARTPQ_REQUEST = "UPDATE_PARTPQ_REQUEST";
export const UpdatePartPQRequestAction = createAction<any>(
  UPDATE_PARTPQ_REQUEST
);
export const UPDATE_PARTPQ_SUCCESS = "UPDATE_PARTPQ_SUCCESS";
export const UpdatePartPQSuccessAction = createAction<any>(
  UPDATE_PARTPQ_SUCCESS
);
export const UPDATE_PARTPQ_FAILURE = "UPDATE_PARTPQ_FAILURE";
export const UpdatePartPQFailureAction = createAction<string>(
  UPDATE_PARTPQ_FAILURE
);

// display3d_init
export const DISPLAY3D_INIT_REQUEST = "DISPLAY3D_INIT_REQUEST";
export const Display3dInitRequestAction = createAction(
  DISPLAY3D_INIT_REQUEST
);
export const DISPLAY3D_INIT_FAILURE = "DISPLAY3D_INIT_FAILURE";
export const Display3dInitFailureAction = createAction<string>(
  DISPLAY3D_INIT_FAILURE
);
export const DISPLAY3D_INIT_SUCCESS = "DISPLAY3D_INIT_SUCCESS";
export const Display3dInitSuccessAction = createAction<any>(
  DISPLAY3D_INIT_SUCCESS
);

// robots related variables //
export const UPDATE_MOTION_VEL_PERCENT = "UPDATE_MOTION_VEL_PERCENT";
export const UpdateMotionVelPercentAction = createAction<any>(
  UPDATE_MOTION_VEL_PERCENT
);
export const UPDATE_TOOL_ID = "UPDATE_TOOL_ID";
export const UpdateToolIdAction = createAction<any>(UPDATE_TOOL_ID);
export const UPDATE_WOBJ_ID = "UPDATE_WOBJ_ID";
export const UpdateWobjIdAction = createAction<any>(UPDATE_WOBJ_ID);
export const UPDATE_COORDINATE_ID = "UPDATE_COORDINATE_ID";
export const UpdateCoordinateIdAction = createAction<any>(UPDATE_COORDINATE_ID);

// auto or manual state //
export const DISPATCH_BUTTON_STATE = "DISPATCH_BUTTON_STATE";
export const DispatchButtonStateAction = createAction<any>(
  DISPATCH_BUTTON_STATE
);
export const DISPATCH_AUTO_STATE = "DISPATCH_AUTO_STATE";
export const DispatchAutoStateAction = createAction<any>(DISPATCH_AUTO_STATE);

// 加载kaanh里面的speed和wobj等变量
export const FETCH_CONFIG_MODEL_REQUEST = "FETCH_CONFIG_MODEL_REQUEST";
export const FetchConfigModelRequestAction = createAction(
  FETCH_CONFIG_MODEL_REQUEST
);
export const FETCH_CONFIG_MODEL_SUCCESS = "FETCH_CONFIG_MODEL_SUCCESS";
export const FetchConfigModelSuccessAction = createAction<any>(
  FETCH_CONFIG_MODEL_SUCCESS
);
export const FETCH_CONFIG_MODEL_FAILURE = "FETCH_CONFIG_MODEL_FAILURE";
export const FetchConfigModelFailureAction = createAction<string>(
  FETCH_CONFIG_MODEL_FAILURE
);
export const FETCH_CONFIG_MODEL = "FETCH_CONFIG_MODEL";
export const FetchConfigModelAction = createAction<any>(FETCH_CONFIG_MODEL);

// models //
export const LOAD_ROBOT_MODELS = "LOAD_ROBOT_MODELS";
export const LoadRobotModelsAction = createAction<any>(LOAD_ROBOT_MODELS);
export const ROBOT_MODELS_FETCH_REQUEST = "ROBOT_MODELS_FETCH_REQUEST";
export const RobotModelsFetchRequestAction = createAction(
  ROBOT_MODELS_FETCH_REQUEST
);
export const ROBOT_MODELS_FETCH_FAILURE = "ROBOT_MODELS_FETCH_FAILURE";
export const RobotModelsFetchFailureAction = createAction<string>(
  ROBOT_MODELS_FETCH_FAILURE
);
export const ROBOT_MODELS_FETCH_SUCCESS = "ROBOT_MODELS_FETCH_SUCCESS";
export const RobotModelsFetchSuccessAction = createAction<any>(
  ROBOT_MODELS_FETCH_SUCCESS
);
export const ROBOT_MODEL_PART_UPLOAD_SUCCESS =
  "ROBOT_MODEL_PART_UPLOAD_SUCCESS";
export const RobotModelPartUploadSuccessAction = createAction<any>(
  ROBOT_MODEL_PART_UPLOAD_SUCCESS
);
export const ROBOT_MODEL_PART_DELETE_REQUEST =
  "ROBOT_MODEL_PART_DELETE_REQUEST";
export const RobotModelPartDeleteRequestAction = createAction(
  ROBOT_MODEL_PART_DELETE_REQUEST
);
export const ROBOT_MODEL_PART_DELETE_FAILURE =
  "ROBOT_MODEL_PART_DELETE_FAILURE";
export const RobotModelPartDeleteFailureAction = createAction<string>(
  ROBOT_MODELS_FETCH_FAILURE
);
export const ROBOT_MODEL_PART_DELETE_SUCCESS =
  "ROBOT_MODEL_PART_DELETE_SUCCESS";
export const RobotModelPartDeleteSuccessAction = createAction<any>(
  ROBOT_MODEL_PART_DELETE_SUCCESS
);
export const DELETE_ROBOT_MODEL_PART = "DELETE_ROBOT_MODEL_PART";
export const DeleteRobotModelPartAction = createAction<any>(
  DELETE_ROBOT_MODEL_PART
);
export const ROBOT_MODEL_DELETE_REQUEST = "ROBOT_MODEL_DELETE_REQUEST";
export const RobotModelDeleteRequestAction = createAction(
  ROBOT_MODEL_DELETE_REQUEST
);
export const ROBOT_MODEL_DELETE_FAILURE = "ROBOT_MODEL_DELETE_FAILURE";
export const RobotModelDeleteFailureAction = createAction<string>(
  ROBOT_MODEL_DELETE_FAILURE
);
export const ROBOT_MODEL_DELETE_SUCCESS = "ROBOT_MODEL_DELETE_SUCCESS";
export const RobotModelDeleteSuccessAction = createAction<any>(
  ROBOT_MODEL_DELETE_SUCCESS
);
export const DELETE_ROBOT_MODEL = "DELETE_ROBOT_MODEL";
export const DeleteRobotModelAction = createAction<any>(DELETE_ROBOT_MODEL);
export const CREATE_ROBOT_MODEL = "CREATE_ROBOT_MODEL";
export const CreateRobotModelAction = createAction<any>(CREATE_ROBOT_MODEL);
export const ROBOT_MODEL_CREATE_REQUEST = "ROBOT_MODEL_CREATE_REQUEST";
export const RobotModelCreateRequestAction = createAction<any>(
  ROBOT_MODEL_CREATE_REQUEST
);
export const ROBOT_MODEL_CREATE_FAILURE = "ROBOT_MODEL_CREATE_FAILURE";
export const RobotModelCreateFailureAction = createAction<string>(
  ROBOT_MODEL_CREATE_FAILURE
);
export const ROBOT_MODEL_CREATE_SUCCESS = "ROBOT_MODEL_CREATE_SUCCESS";
export const RobotModelCreateSuccessAction = createAction<any>(
  ROBOT_MODEL_CREATE_SUCCESS
);
export const SHOW_CREATE_ROBOT_MODEL_MODAL = "SHOW_CREATE_ROBOT_MODEL_MODAL";
export const ShowCreateRobotModelModalAction = createAction(
  SHOW_CREATE_ROBOT_MODEL_MODAL
);
export const HIDE_CREATE_ROBOT_MODEL_MODAL = "HIDE_CREATE_ROBOT_MODEL_MODAL";
export const HideCreateRobotModelModalAction = createAction(
  HIDE_CREATE_ROBOT_MODEL_MODAL
);
export const SELECT_CURRENT_ROBOT_BRAND = "SELECT_CURRENT_ROBOT_BRAND";
export const SelectCurrentRobotBrandAction = createAction<any>(
  SELECT_CURRENT_ROBOT_BRAND
);
export const SELECT_CURRENT_ROBOT_MODEL = "SELECT_CURRENT_ROBOT_MODEL";
export const SelectCurrentRobotModelAction = createAction<any>(
  SELECT_CURRENT_ROBOT_MODEL
);

// ethercat configs //
export const UPLOAD_ESI_FILE_REQUEST = "UPLOAD_ESI_FILE_REQUEST";
export const UploadESIFileRequestAction = createAction(UPLOAD_ESI_FILE_REQUEST);
export const UPLOAD_ESI_FILE_FAILURE = "UPLOAD_ESI_FILE_FAILURE";
export const UploadESIFileFailureAction = createAction<string>(
  UPLOAD_ESI_FILE_FAILURE
);
export const UPLOAD_ESI_FILE_SUCCESS = "UPLOAD_ESI_FILE_SUCCESS";
export const UploadESIFileSuccessAction = createAction<any>(
  UPLOAD_ESI_FILE_SUCCESS
);
export const ESI_PATH_FETCH_REQUEST = "ESI_PATH_FETCH_REQUEST";
export const ESIPathFetchRequestAction = createAction(ESI_PATH_FETCH_REQUEST);
export const ESI_PATH_FETCH_FAILURE = "ESI_PATH_FETCH_FAILURE";
export const ESIPathFetchFailureAction = createAction<string>(
  ESI_PATH_FETCH_FAILURE
);
export const ESI_PATH_FETCH_SUCCESS = "ESI_PATH_FETCH_SUCCESS";
export const ESIPathFetchSuccessAction = createAction<any>(
  ESI_PATH_FETCH_SUCCESS
);
export const LOAD_ESI_PATH = "LOAD_ESI_PATH";
export const LoadESIPathAction = createAction<any>(LOAD_ESI_PATH);
export const CONFIG_XML_FETCH_REQUEST = "CONFIG_XML_FETCH_REQUEST";
export const ConfigXMLFetchRequestAction = createAction(
  CONFIG_XML_FETCH_REQUEST
);
export const CONFIG_XML_FETCH_FAILURE = "CONFIG_XML_FETCH_FAILURE";
export const ConfigXMLFetchFailureAction = createAction<string>(
  CONFIG_XML_FETCH_FAILURE
);
export const CONFIG_XML_FETCH_SUCCESS = "CONFIG_XML_FETCH_SUCCESS";
export const ConfigXMLFetchSuccessAction = createAction<any>(
  CONFIG_XML_FETCH_SUCCESS
);
export const LOAD_CONFIG_XML = "LOAD_CONFIG_XML";
export const LoadConfigXMLAction = createAction(LOAD_CONFIG_XML);

// programs //
export const PROGRAMS_FETCH_REQUEST = "PROGRAMS_FETCH_REQUEST";
export const ProgramsFetchRequestAction = createAction(PROGRAMS_FETCH_REQUEST);
export const PROGRAMS_FETCH_FAILURE = "PROGRAMS_FETCH_FAILURE";
export const ProgramsFetchFailureAction = createAction<string>(
  PROGRAMS_FETCH_FAILURE
);
export const PROGRAMS_FETCH_SUCCESS = "PROGRAMS_FETCH_SUCCESS";
export const ProgramsFetchSuccessAction = createAction<any>(
  PROGRAMS_FETCH_SUCCESS
);
export const LOAD_PROGRAMS = "LOAD_PROGRAMS";
export const LoadProgramsAction = createAction<any>(LOAD_PROGRAMS);
export const PROGRAM_CREATE_REQUEST = "PROGRAM_CREATE_REQUEST";
export const ProgramCreateRequestAction = createAction(PROGRAM_CREATE_REQUEST);
export const PROGRAM_CREATE_FAILURE = "PROGRAM_CREATE_FAILURE";
export const ProgramCreateFailureAction = createAction<string>(
  PROGRAM_CREATE_FAILURE
);
export const PROGRAM_CREATE_SUCCESS = "PROGRAM_CREATE_SUCCESS";
export const ProgramCreateSuccessAction = createAction<any>(
  PROGRAM_CREATE_SUCCESS
);
export const TOGGLE_CREATE_PROGRAM_MODAL = "TOGGLE_CREATE_PROGRAM_MODAL";
export const ToggleCreateProgramModalAction = createAction<any>(
  TOGGLE_CREATE_PROGRAM_MODAL
);
export const CREATE_PROGRAM = "CREATE_PROGRAM";
export const CreateProgramAction = createAction<any>(CREATE_PROGRAM);
export const PROGRAM_UPDATE_REQUEST = "PROGRAM_UPDATE_REQUEST";
export const ProgramUpdateRequestAction = createAction(PROGRAM_UPDATE_REQUEST);
export const PROGRAM_UPDATE_FAILURE = "PROGRAM_UPDATE_FAILURE";
export const ProgramUpdateFailureAction = createAction<string>(
  PROGRAM_UPDATE_FAILURE
);
export const PROGRAM_UPDATE_SUCCESS = "PROGRAM_UPDATE_SUCCESS";
export const ProgramUpdateSuccessAction = createAction<any>(
  PROGRAM_UPDATE_SUCCESS
);
export const UPDATE_PROGRAM = "UPDATE_PROGRAM";
export const UpdateProgramAction = createAction<any>(UPDATE_PROGRAM);
export const PROGRAM_DELETE_REQUEST = "PROGRAM_DELETE_REQUEST";
export const ProgramDeleteRequestAction = createAction(PROGRAM_DELETE_REQUEST);
export const PROGRAM_DELETE_FAILURE = "PROGRAM_DELETE_FAILURE";
export const ProgramDeleteFailureAction = createAction<string>(
  PROGRAM_DELETE_FAILURE
);
export const PROGRAM_DELETE_SUCCESS = "PROGRAM_DELETE_SUCCESS";
export const ProgramDeleteSuccessAction = createAction<any>(
  PROGRAM_DELETE_SUCCESS
);
export const DELETE_PROGRAM = "DELETE_PROGRAM";
export const DeleteProgramAction = createAction<any>(DELETE_PROGRAM);
export const TOGGLE_DELETE_PROGRAM_MODAL = "TOGGLE_DELETE_PROGRAM_MODAL";
export const ToggleDeleteProgramModalAction = createAction<any>(
  TOGGLE_DELETE_PROGRAM_MODAL
);
export const PROGRAM_RENAME_REQUEST = "PROGRAM_RENAME_REQUEST";
export const ProgramRenameRequestAction = createAction(PROGRAM_RENAME_REQUEST);
export const PROGRAM_RENAME_FAILURE = "PROGRAM_RENAME_FAILURE";
export const ProgramRenameFailureAction = createAction<string>(
  PROGRAM_RENAME_FAILURE
);
export const PROGRAM_RENAME_SUCCESS = "PROGRAM_RENAME_SUCCESS";
export const ProgramRenameSuccessAction = createAction<any>(
  PROGRAM_RENAME_SUCCESS
);
export const RENAME_PROGRAM = "RENAME_PROGRAM";
export const RenameProgramAction = createAction<any>(RENAME_PROGRAM);
export const TOGGLE_EDIT_PROGRAM_NAME = "TOGGLE_EDITING_PROGRAM_NAME";
export const ToggleEditProgramAction = createAction<any>(
  TOGGLE_EDIT_PROGRAM_NAME
);
export const RESET_RENAME_PROGRAM_STATUS = "RESET_RENAME_PROGRAM_STATUS";
export const ResetRenameProgramStatusAction = createAction(
  RESET_RENAME_PROGRAM_STATUS
);
export const COPY_BLOCK = "COPY_BLOCK";
export const CopyBlockAction = createAction<any>(COPY_BLOCK);
// locale //
export const LOCALESTATE = "LOCALESTATE";
export const LocaleStateAction = createAction<any>(LOCALESTATE);
// 获取log日志的列表
export const LOG_REQUEST = "LOG_REQUEST";
export const LogRequestAction = createAction(LOG_REQUEST);
export const LOG_SUCCESS = "LOG_SUCCESS";
export const LogSuccessAction = createAction<any>(LOG_SUCCESS);
export const LOG_FAILURE = "LOG_FAILURE";
export const LogFailureAction = createAction<string>(LOG_FAILURE);
export const DISPATCH_LOG_FILE_NAME = "DISPATCH_LOG_FILE_NAME";
export const DispatchLogFileNameAction = createAction<any>(
  DISPATCH_LOG_FILE_NAME
);

// 获取log日志的content
export const LOG_CONTENT_REQUEST = "LOG_CONTENT_REQUEST";
export const LogContentRequestAction = createAction(LOG_CONTENT_REQUEST);
export const LOG_CONTENT_SUCCESS = "LOG_CONTENT_SUCCESS";
export const LogContentSuccessAction = createAction<any>(LOG_CONTENT_SUCCESS);
export const LOG_CONTENT_FAILURE = "LOG_CONTENT_FAILURE";
export const LogContentFailureAction =
  createAction<string>(LOG_CONTENT_FAILURE);
export const DISPATH_LOG_FILE_CONTENT = "DISPATH_LOG_FILE_CONTENT";
export const DispatchLogFileContentAction = createAction<any>(
  DISPATH_LOG_FILE_CONTENT
);
