import React from "react";
import { Route, Routes } from "react-router-dom";
import DashboardPage from "./containers/DashboardPage";
import ConfigurationPage from "./containers/ConfigurationPage";

export default (
  <Routes>
    <Route path={"/"} element={<DashboardPage />} />
    <Route path={"/configuration"} element={<ConfigurationPage />} />
    <Route path={"/dashboards/:id"} element={<DashboardPage />} />
  </Routes>
);
