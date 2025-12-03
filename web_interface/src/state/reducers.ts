import { END } from "@redux-saga/types";
import { combineReducers, PayloadAction } from "@reduxjs/toolkit";
import { number } from "prop-types";
import { DashboardConfig, Matrix4Array, SireGeometry } from "~/types/lib";

import * as ActionTypes from "./constants";
import { Display3dConfig } from "~/shared/cells/Display3d/types";

let currentToolId = 0;
const toolIdStr: string | null = window.localStorage.getItem("currentToolId");
if (toolIdStr != null) {
  try {
    currentToolId = JSON.parse(toolIdStr);
  } catch (e) {}
}
let currentWobjId = 0;
const wobjIdStr: string | null = window.localStorage.getItem("currentWobjId");
if (wobjIdStr != null) {
  try {
    currentWobjId = JSON.parse(wobjIdStr);
  } catch (e) {}
}

export interface RobotState {
  readonly robotStatus: number;
  readonly toolIds: number[];
  readonly wobjIds: number[];
  readonly currentToolId: number;
  readonly currentWobjId: number;
  readonly currentCoordinateId: number;
  readonly currentMotionVelPercent: number;
  readonly firstGet: boolean;
  readonly resultsByChannel: any[];
  readonly createRobotModelModalVisible: boolean;
  readonly currentRobotBrand: string | null;
  readonly currentRobotModel: string | null;
  readonly esiFileUploadingSuccess: boolean | null;
  readonly esiFileUploadingRequest: boolean | null;
  readonly esiFileUploadingFailure: boolean | null;
  readonly esiPathFetchingSuccess: boolean | null;
  readonly esiPathFetchingRequest: boolean | null;
  readonly esiPathFetchingFailure: boolean | null;
  readonly esiPath: string | null;
  readonly xml: string | null;
  readonly part_pq_init: number[][] | null;
  readonly geometry_pm: Matrix4Array[] | null;
  readonly geometry_pool: SireGeometry[] | null;
  readonly viewerConfig: Display3dConfig | null;
  readonly creatingRobotModel: boolean;
  readonly creatingRobotModelError: string | null;
  readonly creatingRobotModelResult: string | null;
  readonly programs: any;
  readonly showCreatingProgramModal: boolean;
  readonly showCreateProgramModal: boolean;
  readonly loadingProgram: boolean;
  readonly loadingProgramError: string | null;
  readonly updatingProgram: boolean;
  readonly updatingProgramFailure: string | null;
  readonly creatingProgram: boolean;
  readonly creatingProgramFailure: boolean | null;
  readonly creatingProgramSuccess: boolean | null;
  readonly renamingProgram: boolean;
  readonly renamingProgramFailure: boolean | null;
  readonly renamingProgramSuccess: boolean | null;

  readonly showDeleteProgramModal: boolean;
  readonly showDeletingProgramModal: boolean;
  readonly deletingProgram: boolean;
  readonly deletingProgramFailure: boolean | null;
  readonly deletingProgramSuccess: boolean | null;
  readonly editingProgramName: string | null;
  readonly fetchingObjPictureList: boolean;
  readonly fetchingObjPictureListError: string | null;
  readonly fetchingObjPictureListResult: any[];

  readonly copy_blk_xml: string;
  readonly buttonState: string;
  readonly autoState: string;
  readonly logFileNameList: string[];
  readonly logFilePath: string[];
  readonly logContent: string;
}
/// ////////////////////////////////////// ROBOT //////////////////////////////////////////////////
const robotInitState: RobotState = {
  robotStatus: 4,

  // robot tools and wobjs
  toolIds: Array.from(Array(16).keys()),
  wobjIds: Array.from(Array(32).keys()),
  currentToolId,
  currentWobjId,
  currentMotionVelPercent: 0,

  firstGet: true,
  resultsByChannel: [],

  createRobotModelModalVisible: false,
  currentRobotBrand: null,
  currentRobotModel: null,
  currentCoordinateId: 0,

  esiFileUploadingSuccess: null,
  esiFileUploadingRequest: null,
  esiFileUploadingFailure: null,

  esiPathFetchingSuccess: null,
  esiPathFetchingRequest: null,
  esiPathFetchingFailure: null,
  esiPath: null,

  xml: null,
  part_pq_init: [],
  geometry_pm: [],
  geometry_pool: [],
  viewerConfig: null,

  creatingRobotModel: false,
  creatingRobotModelError: null,
  creatingRobotModelResult: null,

  programs: {},
  showCreatingProgramModal: false,
  showCreateProgramModal: false,
  loadingProgram: false,
  loadingProgramError: null,
  updatingProgram: false,
  updatingProgramFailure: null,

  creatingProgram: false,
  creatingProgramFailure: null,
  creatingProgramSuccess: null,
  renamingProgram: false,
  renamingProgramFailure: null,
  renamingProgramSuccess: null,

  showDeleteProgramModal: false,
  showDeletingProgramModal: false,
  deletingProgram: false,
  deletingProgramFailure: null,
  deletingProgramSuccess: null,

  editingProgramName: null,
  
  // detechs
  fetchingObjPictureList: false,
  fetchingObjPictureListError: null,
  fetchingObjPictureListResult: [],
  copy_blk_xml: "",
  buttonState: "",
  autoState: "",
  logFileNameList: [],
  logFilePath: [],
  logContent: "",
};

const robot = (
  state: RobotState = robotInitState,
  action: PayloadAction<any, string>
): RobotState => {
  switch (action.type) {
    case ActionTypes.UPDATE_MOTION_VEL_PERCENT: {
      return {
        ...state,
        currentMotionVelPercent: action.payload.motionVelPercent,
      };
    }
    case ActionTypes.UPDATE_COORDINATE_ID: {
      window.localStorage.setItem(
        "currentCoordinateId",
        action.payload.coordinateId
      );
      return {
        ...state,
        currentCoordinateId: action.payload.coordinateId,
      };
    }
    case ActionTypes.UPDATE_TOOL_ID: {
      window.localStorage.setItem("currentToolId", action.payload.toolId);
      return {
        ...state,
        currentToolId: action.payload.toolId,
      };
    }
    case ActionTypes.UPDATE_WOBJ_ID: {
      window.localStorage.setItem("currentWobjId", action.payload.wobjId);
      return {
        ...state,
        currentWobjId: action.payload.wobjId,
      };
    }
    // model brand相关，现在不需要-------------------------------------------------
    // case ActionTypes.ROBOT_MODELS_FETCH_SUCCESS: {
    //   const { robotModels } = action
    //   return { ...state, robotModels }
    // }
    // case ActionTypes.ROBOT_MODEL_DELETE_SUCCESS: {
    //   const { model } = action
    //   const robotModels = { ...state.robotModels }
    //   robotModels[model.brand] = { ...robotModels[model.brand] }
    //   delete robotModels[model.brand][model.model]
    //   if (isEmpty(robotModels[model.brand])) {
    //     delete robotModels[model.brand]
    //   }
    //   return { ...state, robotModels }
    // }
    // case ActionTypes.ROBOT_MODEL_PART_UPLOAD_SUCCESS: {
    //   const { model } = action
    //   const robotModels = { ...state.robotModels }
    //   robotModels[model.brand] = { ...robotModels[model.brand] }
    //   robotModels[model.brand][model.model] = model
    //   return { ...state, robotModels }
    // }
    // case ActionTypes.ROBOT_MODEL_PART_DELETE_SUCCESS: {
    //   const { model } = action
    //   const robotModels = { ...state.robotModels }
    //   robotModels[model.brand] = { ...robotModels[model.brand] }
    //   robotModels[model.brand][model.model] = model
    //   return { ...state, robotModels }
    // }
    // case ActionTypes.ROBOT_MODEL_CREATE_REQUEST: {
    //   return {
    //     ...state,
    //     creatingRobotModel: true,
    //     creatingRobotModelError: false,
    //     creatingRobotModelResult: null
    //   }
    // }
    // case ActionTypes.ROBOT_MODEL_CREATE_SUCCESS: {
    //   const { model } = action
    //   const robotModels = { ...state.robotModels }
    //   robotModels[model.brand] = { ...robotModels[model.brand] }
    //   robotModels[model.brand][model.model] = model
    //   return {
    //     ...state,
    //     robotModels,
    //     currentRobotBrand: model.brand,
    //     currentRobotModel: model.model,
    //     createRobotModelModalVisible: false,
    //     creatingRobotModel: false,
    //     creatingRobotModelError: null,
    //     creatingRobotModelResult: model
    //   }
    // }
    // case ActionTypes.ROBOT_MODEL_CREATE_FAILURE: {
    //   const { error } = action
    //   return {
    //     ...state,
    //     creatingRobotModel: false,
    //     creatingRobotModelError: error,
    //     creatingRobotModelResult: null
    //   }
    // }
    // case ActionTypes.SHOW_CREATE_ROBOT_MODEL_MODAL: {
    //   return {
    //     ...state,
    //     createRobotModelModalVisible: true,
    //     creatingRobotModel: false,
    //     creatingRobotModelError: null,
    //     creatingRobotModelResult: null
    //   }
    // }
    // case ActionTypes.HIDE_CREATE_ROBOT_MODEL_MODAL: {
    //   return {
    //     ...state,
    //     createRobotModelModalVisible: false
    //   }
    // }
    // case ActionTypes.SELECT_CURRENT_ROBOT_BRAND: {
    //   const { brand } = action
    //   return {
    //     ...state,
    //     currentRobotBrand: brand
    //   }
    // }
    // case ActionTypes.SELECT_CURRENT_ROBOT_MODEL: {
    //   const { model } = action
    //   return {
    //     ...state,
    //     currentRobotModel: model
    //   }
    // }
    case ActionTypes.UPLOAD_ESI_FILE_REQUEST: {
      return {
        ...state,
        esiFileUploadingSuccess: null,
        esiFileUploadingRequest: true,
        esiFileUploadingFailure: null,
      };
    }
    case ActionTypes.UPLOAD_ESI_FILE_FAILURE: {
      return {
        ...state,
        esiFileUploadingSuccess: null,
        esiFileUploadingRequest: false,
        esiFileUploadingFailure: action.payload.error,
      };
    }
    case ActionTypes.UPLOAD_ESI_FILE_SUCCESS: {
      return {
        ...state,
        esiFileUploadingRequest: false,
        esiFileUploadingSuccess: true,
        esiFileUploadingFailure: false,
      };
    }
    case ActionTypes.ESI_PATH_FETCH_REQUEST: {
      return {
        ...state,
        esiPath: null,
        esiPathFetchingRequest: true,
        esiPathFetchingSuccess: false,
        esiPathFetchingFailure: false,
      };
    }
    case ActionTypes.ESI_PATH_FETCH_FAILURE: {
      return {
        ...state,
        esiPath: null,
        esiPathFetchingRequest: false,
        esiPathFetchingSuccess: false,
        esiPathFetchingFailure: action.payload.error,
      };
    }
    case ActionTypes.ESI_PATH_FETCH_SUCCESS: {
      return {
        ...state,
        esiPath: action.payload.path,
        esiPathFetchingRequest: false,
        esiPathFetchingSuccess: true,
        esiPathFetchingFailure: false,
      };
    }
    case ActionTypes.CONFIG_XML_FETCH_SUCCESS: {
      return {
        ...state,
        xml: action.payload.xml,
      };
    }
    case ActionTypes.DISPLAY3D_INIT_SUCCESS: {
      return {
        ...state,
        geometry_pool: action.payload.geometry_pool,
        geometry_pm: action.payload.geometry_pm,
        part_pq_init: action.payload.part_pq_init,
      };
    }
    case ActionTypes.PROGRAMS_FETCH_REQUEST: {
      return {
        ...state,
        loadingProgram: true,
        loadingProgramError: null,
      };
    }
    case ActionTypes.PROGRAMS_FETCH_FAILURE: {
      return {
        ...state,
        loadingProgram: false,
        loadingProgramError: action.payload.error,
      };
    }
    case ActionTypes.PROGRAMS_FETCH_SUCCESS: {
      return {
        ...state,
        loadingProgram: false,
        loadingProgramError: null,
        programs: action.payload.programs,
      };
    }
    case ActionTypes.PROGRAM_UPDATE_REQUEST: {
      const programs = { ...state.programs };
      programs[action.payload.program.name] = action.payload.program;
      return {
        ...state,
        updatingProgram: true,
        updatingProgramFailure: null,
        programs,
      };
    }
    case ActionTypes.PROGRAM_UPDATE_FAILURE: {
      return {
        ...state,
        updatingProgram: false,
        updatingProgramFailure: action.payload.error,
      };
    }
    case ActionTypes.PROGRAM_UPDATE_SUCCESS: {
      const programs = { ...state.programs };
      programs[action.payload.program.name] = action.payload.program;
      return {
        ...state,
        updatingProgram: false,
        updatingProgramFailure: null,
        programs,
      };
    }
    case ActionTypes.TOGGLE_CREATE_PROGRAM_MODAL: {
      return {
        ...state,
        showCreateProgramModal: !state.showCreateProgramModal,
        creatingProgram: false,
        creatingProgramFailure: null,
        creatingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_CREATE_REQUEST: {
      return {
        ...state,
        creatingProgram: false,
        creatingProgramFailure: null,
        creatingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_CREATE_FAILURE: {
      return {
        ...state,
        creatingProgram: false,
        creatingProgramFailure: action.payload.error,
        creatingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_CREATE_SUCCESS: {
      const { program } = action.payload;
      const programs = { ...state.programs };
      programs[program.name] = program;
      return {
        ...state,
        programs,
        showCreateProgramModal: false,
        creatingProgramFailure: null,
        creatingProgram: false,
        creatingProgramSuccess: program,
      };
    }
    case ActionTypes.TOGGLE_DELETE_PROGRAM_MODAL: {
      return {
        ...state,
        showDeleteProgramModal:
          action.payload.name != null && Boolean(action.payload.name)
            ? action.payload.name
            : !state.showDeleteProgramModal,
        deletingProgram: false,
        deletingProgramFailure: null,
        deletingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_DELETE_REQUEST: {
      return {
        ...state,
        deletingProgram: false,
        deletingProgramFailure: null,
        deletingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_DELETE_FAILURE: {
      return {
        ...state,
        deletingProgram: false,
        deletingProgramFailure: action.payload.error,
        deletingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_DELETE_SUCCESS: {
      const { program } = action.payload;
      const programs = { ...state.programs };
      // eslint-disable-next-line @typescript-eslint/no-dynamic-delete
      delete programs[program.name];
      return {
        ...state,
        programs,
        showDeleteProgramModal: false,
        deletingProgramFailure: null,
        deletingProgram: false,
        deletingProgramSuccess: program,
      };
    }
    case ActionTypes.TOGGLE_EDIT_PROGRAM_NAME: {
      const { name } = action.payload;
      return {
        ...state,
        renamingProgram: false,
        renamingProgramFailure: null,
        renamingProgramSuccess: null,
        editingProgramName: state.editingProgramName === name ? null : name,
      };
    }
    case ActionTypes.PROGRAM_RENAME_REQUEST: {
      return {
        ...state,
        renamingProgram: false,
        renamingProgramFailure: null,
        renamingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_RENAME_FAILURE: {
      return {
        ...state,
        renamingProgram: false,
        renamingProgramFailure: action.payload.error,
        renamingProgramSuccess: null,
      };
    }
    case ActionTypes.PROGRAM_RENAME_SUCCESS: {
      const { program, name, copy } = action.payload;
      const programs = { ...state.programs };
      // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
      if (!copy) {
        // eslint-disable-next-line @typescript-eslint/no-dynamic-delete
        delete programs[name];
      }
      programs[program.name] = program;
      return {
        ...state,
        programs,
        editingProgramName: null,
        renamingProgram: false,
        renamingProgramFailure: null,
        renamingProgramSuccess: program,
      };
    }
    case ActionTypes.RESET_RENAME_PROGRAM_STATUS: {
      return {
        ...state,
        renamingProgram: false,
        renamingProgramFailure: null,
        renamingProgramSuccess: null,
      };
    }
    case ActionTypes.COPY_BLOCK: {
      return {
        ...state,
        copy_blk_xml: action.payload.blk_xml,
      };
    }
    case ActionTypes.DISPATCH_BUTTON_STATE: {
      window.localStorage.setItem("switchState", action.payload.name);
      return {
        ...state,
        buttonState: action.payload.name,
      };
    }
    case ActionTypes.DISPATCH_AUTO_STATE: {
      window.localStorage.setItem("autoState", action.payload.name);
      return {
        ...state,
        autoState: action.payload.name,
      };
    }
    case ActionTypes.LOG_REQUEST: {
      return {
        ...state,
        logFileNameList: [],
        esiPathFetchingRequest: true,
        esiPathFetchingSuccess: false,
        esiPathFetchingFailure: false,
      };
    }
    case ActionTypes.LOG_SUCCESS: {
      return {
        ...state,
        logFileNameList: action.payload.logFiles.logFileList,
        logFilePath: action.payload.logFiles.logFilePath,
      };
    }
    case ActionTypes.LOG_CONTENT_REQUEST: {
      return { ...state };
    }
    case ActionTypes.LOG_CONTENT_SUCCESS: {
      return {
        ...state,
        logContent: action.payload.content.fileData,
      };
    }
    // case ActionTypes.FETCH_OBJ_PICTURE_LIST_REQUEST: {
    //   return {
    //     ...state,
    //     fetchingObjPictureList: true,
    //     fetchingObjPictureListError: null,
    //     fetchingObjPictureListResult: []
    //   }
    // }
    // case ActionTypes.FETCH_OBJ_PICTURE_LIST_SUCCESS: {
    //   return {
    //     ...state,
    //     fetchingObjPictureList: false,
    //     fetchingObjPictureListError: null,
    //     fetchingObjPictureListResult: action.payload.content
    //   }
    // }
    // case ActionTypes.FETCH_OBJ_PICTURE_LIST_FAILURE: {
    //   return {
    //     ...state,
    //     fetchingObjPictureList: false,
    //     fetchingObjPictureListError: action.payload.error,
    //     fetchingObjPictureListResult: []
    //   }
    // }
    // case ActionTypes.UPDATE_OBJ_PICTURE_LIST: {
    //   return {
    //     ...state,
    //     fetchingObjPictureListResult: action.payload.value
    //   }
    // }
    default:
      return state;
  }
};

export interface ConfigState {
  readonly fullscreen: boolean;
  readonly showCreateCellModal: boolean;
  readonly creatingCell: boolean;
  readonly creatingCellFailure: boolean | null;
  readonly creatingCellSuccess: boolean | null;
  readonly dashboards: DashboardConfig[];
  readonly localestate: string;
  readonly configModel: string | null;
}

/// ////////////////////////////////////// CONFIG //////////////////////////////////////////////////
const configInitState: ConfigState = {
  fullscreen: false,
  showCreateCellModal: false,
  creatingCell: false,
  creatingCellFailure: null,
  creatingCellSuccess: null,
  dashboards: [],
  localestate: "zh",
  configModel: null,
};
const config = (
  state: ConfigState = configInitState,
  action: PayloadAction<any, string>
): ConfigState => {
  switch (action.type) {
    case ActionTypes.INTERFACE_CONFIG_FETCH_REQUEST:
      return state;
    case ActionTypes.INTERFACE_CONFIG_FETCH_FAILURE:
      return state;
    case ActionTypes.INTERFACE_CONFIG_FETCH_SUCCESS:
      console.log(action.payload.response);
      return { ...state, ...action.payload.response };
    case ActionTypes.FULL_SCREEN:
      return { ...state, fullscreen: true };
    case ActionTypes.FULL_SCREEN_INVERSE:
      return { ...state, fullscreen: false };
    case ActionTypes.UPDATE_DASHBOARD: {
      const { dashboard } = action.payload;
      const dashboards = [...state.dashboards];
      dashboards.forEach((d, i) => {
        if (d.i === dashboard.i) {
          dashboards[i] = { ...dashboards[i], ...dashboard };
        }
      });
      return { ...state, dashboards };
    }
    case ActionTypes.DASHBOARD_UPDATE_SUCCESS: {
      const { dashboard } = action.payload;
      const dashboards = [...state.dashboards];
      dashboards.forEach((d, i) => {
        if (d.i === dashboard.i) {
          dashboards[i] = { ...dashboards[i], ...dashboard };
        }
      });
      return { ...state, dashboards };
    }
    case ActionTypes.TOGGLE_CREATE_CELL_MODAL: {
      return {
        ...state,
        showCreateCellModal: !state.showCreateCellModal,
        creatingCell: false,
        creatingCellFailure: null,
        creatingCellSuccess: null,
      };
    }
    case ActionTypes.CELL_CREATE_REQUEST: {
      return {
        ...state,
        creatingCell: false,
        creatingCellFailure: null,
        creatingCellSuccess: null,
      };
    }
    case ActionTypes.CELL_CREATE_FAILURE: {
      return {
        ...state,
        creatingCell: false,
        creatingCellFailure: action.payload.error,
        creatingCellSuccess: null,
      };
    }
    case ActionTypes.CELL_CREATE_SUCCESS: {
      const { dashboardId, cell } = action.payload;
      const dashboards = [...state.dashboards];
      dashboards.forEach((d, i) => {
        if (d.i === dashboardId) {
          dashboards[i] = { ...dashboards[i] };
          dashboards[i].cells = dashboards[i].cells.map((cell) => ({
            ...cell,
            y: cell.y + 1,
          }));
          dashboards[i].cells = [...dashboards[i].cells, cell];
        }
      });
      return {
        ...state,
        dashboards,
        showCreateCellModal: false,
        creatingCellFailure: null,
        creatingCell: false,
        creatingCellSuccess: cell,
      };
    }
    case ActionTypes.CELL_DELETE_SUCCESS: {
      const { dashboardId, cell } = action.payload;
      const dashboards = [...state.dashboards];
      dashboards.forEach((d, i) => {
        if (d.i === dashboardId) {
          dashboards[i] = { ...dashboards[i] };
          dashboards[i].cells = dashboards[i].cells.filter(
            (c) => c.i !== cell.i
          );
        }
      });
      return { ...state, dashboards };
    }
    case ActionTypes.LOCALESTATE: {
      return {
        ...state,
        localestate: action.payload.value,
      };
    }
    case ActionTypes.FETCH_CONFIG_MODEL_REQUEST:
      return { ...state };
    case ActionTypes.FETCH_CONFIG_MODEL_FAILURE:
      return { ...state };
    case ActionTypes.FETCH_CONFIG_MODEL_SUCCESS: {
      if (action.payload.model != null) {
        return { ...state, configModel: action.payload.model };
      } else {
        return state;
      }
    }
    default:
      return state;
  }
};

export interface ErrorState {
  readonly type: string;
  readonly errorType: string;
  readonly errorMsg: string;
}

const errorInitState: ErrorState = {
  type: "",
  errorType: "",
  errorMsg: "",
};

/// ///////////////////////////////////////ERROR//////////////////////////////////////////////////
const errors = (
  state: ErrorState = errorInitState,
  action: PayloadAction<any, string>
): ErrorState => {
  const type = action.type;
  const error = action.payload as string;
  if (type === ActionTypes.RESET_ERROR_MESSAGE) {
    // if (errorType) {
    //   if (state[errorType]) {
    //     state = { ...state }
    //     delete state[errorType]
    //   }
    // } else {
    state = {
      ...state,
      errorType: "",
      errorMsg: "",
    };
    // }
    // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
  } else if (error) {
    state = { ...state, type: error };
  }
  return state;
};

export interface RouterState {
  pathname: string;
}

const routerInitState: RouterState = {
  pathname: window.location.hash.slice(1),
};

const router = (
  state: RouterState = routerInitState,
  action: PayloadAction<string, string>
): RouterState => {
  switch (action.type) {
    case ActionTypes.UPDATE_ROUTER_STATE:
      return { ...state };
    case ActionTypes.NAVIGATE: {
      const pathname = action.payload;
      return { ...state, pathname };
    }
    default:
      return state;
  }
};

/// ////////////////////////////////// SOCKET //////////////////////////////////////////////////////////
export interface CommandState {
  cmd: string | null;
  isRunning: boolean;
  cmdResponseTime: number | null;
  return_code: number | null;
  return_message: string | null;
}

export interface SocketState {
  websocketConnected: boolean;
  currentCommand: CommandState;
}

const socketInitState: SocketState = {
  websocketConnected: false,
  currentCommand: {
    cmd: null,
    isRunning: false,
    cmdResponseTime: null,
    return_code: null,
    return_message: null,
  },
};
const socket = (
  state: SocketState = socketInitState,
  action: PayloadAction<any, string>
): SocketState => {
  switch (action.type) {
    case ActionTypes.SEND_CMD_RESPONSE: {
      return {
        ...state,
        currentCommand: {
          cmd: action.payload.cmd,
          isRunning: false,
          cmdResponseTime: action.payload.cmdResponseTime,
          // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
          return_code: action.payload.data
            ? action.payload.data.return_code
            : null,
          // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
          return_message: action.payload.data
            ? action.payload.data.return_message
            : null,
        },
      };
    }
    case ActionTypes.RECORD_SEND_CMD: {
      return {
        ...state,
        currentCommand: {
          cmd: action.payload.cmd,
          isRunning: true,
          cmdResponseTime: null,
          return_code: null,
          return_message: null,
        },
      };
    }
    case ActionTypes.UPDATE_WEBSOCKET_STATUS:
      return {
        ...state,
        websocketConnected: action.payload.websocketConnected,
      };
    default:
      return state;
  }
};

/// ////////////////////////////////// SERVER //////////////////////////////////////////////////////////
export interface ServerState {
  server_state: string;
}

const serverInitState: ServerState = {
  server_state: "",
};
const server = (
  state: ServerState = serverInitState,
  action: PayloadAction<any, string>
): ServerState => {
  switch (action.type) {
    case ActionTypes.UPDATE_SERVER_STATE_SUCCESS:
      return { ...state, ...action.payload.server_state };
    default:
      return state;
  }
};

const EndSaga = (state: Object = {}, action: END): Object => {
  return {};
};

/// /////////////////////////////////// COMBINE ///////////////////////////////////////////////////////////////
const rootReducer = combineReducers({
  config,
  errors,
  router,
  robot,
  socket,
  server,
  EndSaga,
});

export type RootState = ReturnType<typeof rootReducer>;
export default rootReducer;
