const CracoLessPlugin = require("craco-less");

let modifyVars = {
  "@ant-prefix": "rchmi",

  "@font-size-sm": "12px",
  "@font-size-base": "12px",
  "@primary-color": "#0070cc",
  "@border-radius-base": "5px",
  "@border-radius-sm": "3px",

  "@text-color": "fade(#000, 65%)",
  "@text-color-secondary": "fade(#000, 45%)",
  "@background-color-base": "hsv(0, 0, 96%)",

  "@success-color": "#1e8e3e",
  "@error-color": "#d93026",
  "@warning-color": "#ffc440",
  "@info-color": "@primary-color",
  "@danger-color": "@error-color",
  "@processing-color": "@primary-color",

  "@border-color-base": "#dedede",
  "@border-color-split": "#dedede",

  "@outline-width": "0",
  "@outline-color": "#737373",

  "@input-height-lg": "36px",
  "@input-height-base": "32px",
  "@input-height-sm": "24px",
  "@input-hover-border-color": "#737373",

  "@form-item-margin-bottom": "16px",

  "@btn-default-bg": "#fafafa",
  "@btn-default-border": "#dedede",
  "@btn-danger-color": "#fff",
  "@btn-danger-bg": "@error-color",
  "@btn-danger-border": "@error-color",

  "@switch-color": "@success-color",

  "@table-header-bg": "#fafafa",
  "@table-row-hover-bg": "#fafafa",
  "@table-padding-vertical": "8px",

  "@badge-color": "@error-color",

  "@breadcrumb-base-color": "@text-color",
  "@breadcrumb-last-item-color": "@text-color-secondary",

  "@slider-rail-background-color": "@background-color-base",
  "@slider-rail-background-color-hover": "#e1e1e1",
  "@slider-track-background-color": "@primary-color",
  "@slider-track-background-color-hover": "@primary-color",
  "@slider-handle-border-width": "1px",
  "@slider-handle-color": "#dedede",
  "@slider-handle-color-hover": "#dedede",
  "@slider-handle-color-focus": "#dedede",
  "@slider-handle-color-tooltip-open": "#ddd",
  "@slider-handle-color-focus-shadow": "transparent",
  "@slider-handle-shadow": "1px 1px 4px 0 rgba(0,0,0,.13)",

  "@alert-success-border-color": "#dff4e5",
  "@alert-success-bg-color": "#dff4e5",
  "@alert-info-border-color": "#e5f3ff",
  "@alert-info-bg-color": "#e5f3ff",
  "@alert-error-border-color": "#fcebea",
  "@alert-error-bg-color": "#fcebea",
  "@alert-warning-border-color": "#fff7db",
  "@alert-warning-bg-color": "#fff7db",

  "@radio-button-bg": "transparent",
  "@radio-button-checked-bg": "transparent",

  "@progress-radius": "0",

  "@tabs-card-gutter": "-1px",
  "@tabs-card-tab-active-border-top": "2px solid @primary-color",
};
if (!process.env.WHITE_THEME) {
  modifyVars = {
    "@ant-prefix": "rchmi",
    "@primary-color": "#22adf6",
    "@component-background": "#292933",
    "@background-color-base": "#000",
    "@heading-color": "#ababab",
    "@text-color": "#fff",
    "@text-color-secondary": "#aaa",
    "@text-color-inverse": "#333",
    "@icon-color-hover": "#fff",
    "@heading-color-dark": "#aaa",
    "@text-color-dark": "#aaa",
    "@text-color-secondary-dark": "#999",
    "@item-active-bg": "#555",
    "@item-hover-bg": "#000",
    "@border-color-base": "#292933",
    "@border-color-split": "#565656",
    "@background-color-light": "#222",
    "@disabled-color": "#777",
    "@disabled-bg": "#555",
    "@disabled-color-dark": "#444",
    "@btn-default-bg": "#333",
    "@checkbox-check-color": "#333",
    "@input-bg": "#333",
    "@input-number-handler-active-bg": "#454545",
    "@table-selected-row-bg": "#343434",
    "@table-expanded-row-bg": "#353535",
    "@tree-directory-selected-color": "#111",
    "@table-row-hover-bg": "#111",
    "@popover-bg": "#222",
    "@shadow-color": "rgba(255, 255, 255, 0.15)",
    "@btn-shadow": "0 2px 0 rgba(0, 0, 0, 0.015)",
    "@btn-primary-shadow": "0 2px 0 rgba(0, 0, 0, 0.045)",
    "@btn-text-shadow": "0 -1px 0 rgba(0, 0, 0, 0.12)",
    "@alert-success-border-color": "#03300c",
    "@alert-success-bg-color": "#03300f",
    "@alert-info-border-color": "#061f70",
    "@alert-info-bg-color": "#061f72",
    "@alert-warning-border-color": "#706705",
    "@alert-warning-bg-color": "#706705",
    "@alert-error-border-color": "#702b06",
    "@alert-error-bg-color": "#702b05",
  };
}

modifyVars = {
  "@ant-prefix": "rchmi",
};

module.exports = {
  eslint: {
    enable: false,
  },
  plugins: [
    {
      plugin: CracoLessPlugin,
      options: {
        lessLoaderOptions: {
          lessOptions: {
            modifyVars,
            javascriptEnabled: true,
          },
        },
      },
    },
  ],
};
