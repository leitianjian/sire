// import * as _api from './api'
import { createHashHistory } from "history";

import "isomorphic-fetch";
import {
  CellConfig,
  DashboardConfig,
  FetchError,
  FetchResponse,
} from "~/types/lib";
import { ROBOT_PREFIX } from "../utils";

export const history = createHashHistory();

async function callRawGetApi(
  endpoint: string
): Promise<FetchResponse<string> | FetchError> {
  return await fetch(endpoint)
    .then(
      async (response: Response) =>
        await response.text().then((text: string) => ({ text, response }))
    )
    .then(({ text, response }) => {
      if (!response.ok) {
        return Promise.reject(text);
      }
      return text;
    })
    .then(
      (response): FetchResponse<string> => ({ response }),
      (): FetchError => ({
        error: "Something bad happened",
      })
    );
}

async function callGetApi(
  endpoint: string
): Promise<FetchResponse<any> | FetchError> {
  return await fetch(endpoint)
    .then(
      async (response: Response) =>
        await response.json().then((json) => ({ json, response }))
    )
    .then(({ json, response }) => {
      if (!response.ok) {
        return Promise.reject(json);
      }
      return json;
    })
    .then(
      (response): FetchResponse<any> => ({ response }),
      (error): FetchError => ({
        // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
        error: error.message || error.error || "Something bad happened",
      })
    );
}

async function callPatchApi(
  endpoint: string,
  data: any
): Promise<FetchResponse<any> | FetchError> {
  return await fetch(endpoint, {
    method: "PATCH",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(data),
  })
    .then(
      async (response) =>
        await response.json().then((json) => ({ json, response }))
    )
    .then(({ json, response }) => {
      if (!response.ok) {
        return Promise.reject(json);
      }
      return json;
    })
    .then(
      (response) => ({ response }),
      (error) => ({
        // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
        error: error.message || error.error || "Something bad happened",
      })
    );
}

async function callPutApi(
  endpoint: string,
  data: any
): Promise<FetchResponse<any> | FetchError> {
  return await fetch(endpoint, {
    method: "PUT",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(data),
  })
    .then(
      async (response) =>
        await response.json().then((json) => ({ json, response }))
    )
    .then(({ json, response }) => {
      if (!response.ok) {
        return Promise.reject(json);
      }
      return json;
    })
    .then(
      (response) => ({ response }),
      (error) => ({
        // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
        error: error.message || error.error || "Something bad happened",
      })
    );
}

async function callPostApi(
  endpoint: string,
  data: any
): Promise<FetchResponse<any> | FetchError> {
  return await fetch(endpoint, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(data),
  })
    .then(
      async (response) =>
        await response.json().then((json) => ({ json, response }))
    )
    .then(({ json, response }) => {
      if (!response.ok) {
        return Promise.reject(json);
      }
      return json;
    })
    .then(
      (response) => ({ response }),
      (error) => ({
        // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
        error: error.message || error.error || "Something bad happened",
      })
    );
}

async function callDeleteApi(
  endpoint: string,
  data: any
): Promise<FetchResponse<any> | FetchError> {
  return await fetch(endpoint, {
    method: "DELETE",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(data),
  })
    .then(
      async (response) =>
        await response.json().then((json) => ({ json, response }))
    )
    .then(({ json, response }) => {
      if (!response.ok) {
        return Promise.reject(json);
      }
      return json;
    })
    .then(
      (response) => ({ response }),
      (error) => ({
        // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
        error: error.message || error.error || "Something bad happened",
      })
    );
}

// api services
export const fetchInterfaceConfig = async (): Promise<
  FetchResponse<any> | FetchError
> => await callGetApi(`${ROBOT_PREFIX}/api/config/interface`);
export const updateDashboard = async (
  dashboard: DashboardConfig
): Promise<FetchResponse<any> | FetchError> => {
  dashboard = { ...dashboard };
  dashboard.cells = dashboard.cells.map((cell) => {
    cell = { ...cell, options: JSON.stringify(cell.optionsJson) };
    return cell;
  });
  return await callPutApi(
    `${ROBOT_PREFIX}/api/dashboards/${dashboard.i}`,
    dashboard
  );
};
export const createCell = async (
  dashboardId: string,
  cell: CellConfig
): Promise<FetchResponse<any> | FetchError> => {
  cell = { ...cell, options: JSON.stringify(cell.optionsJson) };
  return await callPostApi(
    `${ROBOT_PREFIX}/api/dashboards/${dashboardId}/cells`,
    {
      ...cell,
    }
  );
};
export const deleteCell = async (
  dashboardId: string,
  cellId: string
): Promise<FetchResponse<any> | FetchError> =>
  await callDeleteApi(
    `${ROBOT_PREFIX}/api/dashboards/${dashboardId}/cells/${cellId}`,
    {}
  );

export const fetchESIPath = async (): Promise<
  FetchResponse<any> | FetchError
> => await callGetApi(`${ROBOT_PREFIX}/api/esi/path`);
export const fetchConfigXml = async (): Promise<
  FetchResponse<string> | FetchError
> => await callRawGetApi(`${ROBOT_PREFIX}/api/config/xml`);
export const loadConfigModel = async (): Promise<
  FetchResponse<any> | FetchError
> => await callGetApi(`${ROBOT_PREFIX}/api/config/robot/model`);

export const fetchPrograms = async (): Promise<
  FetchResponse<any> | FetchError
> => await callGetApi(`${ROBOT_PREFIX}/api/programs`);
export const createProgram = async (
  name: string
): Promise<FetchResponse<any> | FetchError> =>
  await callPostApi(`${ROBOT_PREFIX}/api/programs`, { name });
export const updateProgram = async (
  name: string,
  program: any
): Promise<FetchResponse<any> | FetchError> =>
  await callPutApi(
    `${ROBOT_PREFIX}/api/programs/${encodeURIComponent(name)}`,
    program
  );
export const deleteProgram = async (
  name: string
): Promise<FetchResponse<any> | FetchError> =>
  await callDeleteApi(
    `${ROBOT_PREFIX}/api/programs/${encodeURIComponent(name)}`,
    {}
  );
export const renameProgram = async (
  name: string,
  newName: string,
  copy: boolean
): Promise<FetchResponse<any> | FetchError> =>
  await callPatchApi(
    copy
      ? `${ROBOT_PREFIX}/api/programs/${encodeURIComponent(name)}?copy=true`
      : `${ROBOT_PREFIX}/api/programs/${encodeURIComponent(name)}`,
    { name: newName }
  );

export const fetchRobotModels = async (): Promise<
  FetchResponse<any> | FetchError
> => await callGetApi(`${ROBOT_PREFIX}/api/robots`); // 获取model接口
export const deleteRobotModelPart = async (
  brand: string,
  model: string,
  part: string
): Promise<FetchResponse<any> | FetchError> =>
  await callDeleteApi(
    `${ROBOT_PREFIX}/api/robots/${brand}/${model}/parts/${part}`,
    {}
  );
export const deleteRobotModel = async (
  brand: string,
  model: string
): Promise<FetchResponse<any> | FetchError> =>
  await callDeleteApi(`${ROBOT_PREFIX}/api/robots/${brand}/${model}`, {});
export const createRobotModel = async (
  model: string
): Promise<FetchResponse<any> | FetchError> =>
  await callPostApi(`${ROBOT_PREFIX}/api/robots`, model);

export const fetchLogName = async (
  pageNum: number
): Promise<FetchResponse<any> | FetchError> =>
  await callGetApi(`${ROBOT_PREFIX}/api/logdata/${pageNum}`);
export const fetchLogContent = async (
  name: string,
  pageNum: number
): Promise<FetchResponse<any> | FetchError> =>
  await callPostApi(`${ROBOT_PREFIX}/api/logContent/${name}/${pageNum}`, {});

export const fetchObjPictureList = async (): Promise<
  FetchResponse<any> | FetchError
> => await callGetApi(`${ROBOT_PREFIX}/api/obj_picture_list`);
