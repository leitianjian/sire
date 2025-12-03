import { Path } from "path-parser";

export const ROBOTS_PATH = new Path("/robots/:robotID");
export const ROBOT_ID = getRobotID();
export const ROBOT_PREFIX = ROBOT_ID != null ? `/robots/${ROBOT_ID}` : "";

export const DASHBOARD_PATH = new Path("/dashboards/:dashboardID");
export const DASHBOARD_ID = getDashboardID();

export function getRobotID(): string | null {
  const r = ROBOTS_PATH.partialTest(window.location.pathname);
  console.log(r);
  return r != null && Boolean(r.robotID) ? r.robotID : null;
}

export function getDashboardID(): string | null {
  const r = DASHBOARD_PATH.partialTest(window.location.hash.slice(1));
  return r != null && Boolean(r.dashboardID) ? r.dashboardID : null;
}

console.log(window.location.pathname);

console.log(window.location.hash);

console.log("ROBOT_PREFIX", ROBOT_PREFIX);
console.log("DASHBOARD_ID", DASHBOARD_ID);
console.log("ROBOT_ID", ROBOT_ID);
