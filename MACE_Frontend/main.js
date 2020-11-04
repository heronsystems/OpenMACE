const electron = require("electron");
const url = require("url");
require("events").EventEmitter.prototype._maxListeners = 100;
require("events").prototype._maxListeners = 100;
require("events").defaultMaxListeners = 100;
const fs = require("fs");
const moment = require("moment");
const path = require("path");

let setSizeAndPosition = false;

const { Menu, BrowserWindow, app, ipcMain } = electron;

const { join } = require("path");
const isDev = require("electron-is-dev");

let mainWindow;
let interval;

global.mainWindow = mainWindow;
global.interval = interval;

const devIconPath = `${__dirname}/src/assets`;
const productionIconPath = `${__dirname}/build/icons`;
const iconPath = isDev ? devIconPath : productionIconPath;
// const icon = `${iconPath}/128x128.png`;
const icon = `${iconPath}/mace-icon.png`;
const isMac = process.platform === "darwin";

const template = [
  { label: "File", submenu: [isMac ? { role: "close" } : { role: "quit" }] },
  {
    label: "Edit",
    submenu: [
      {
        role: "undo"
      },
      {
        role: "redo"
      },
      {
        type: "separator"
      },
      {
        role: "cut"
      },
      {
        role: "copy"
      },
      {
        role: "paste"
      }
    ]
  },

  {
    label: "View",
    submenu: [
      {
        role: "reload"
      },
      {
        role: "toggledevtools"
      },
      {
        type: "separator"
      },
      {
        role: "resetzoom"
      },
      {
        role: "zoomin"
      },
      {
        role: "zoomout"
      },
      {
        type: "separator"
      },
      {
        role: "togglefullscreen"
      },
      {
        label: "Toggle HUD",
        accelerator: "CmdOrCtrl + H",
        click: () => {
          mainWindow.webContents.send("toggle_hud");
        }
      }
    ]
  },

  {
    role: "window",
    submenu: [
      {
        role: "minimize"
      },
      {
        role: "close"
      }
    ]
  },
  {
    label: "Tools",
    submenu: [
      {
        label: "Start",
        click: () => {
          mainWindow.webContents.send("hmi", { event: "START" });
        }
      },
      {
        label: "Stop",
        click: () => {
          mainWindow.webContents.send("hmi", { event: "STOP" });
        }
      },
      {
        label: "Restart",
        click: () => {
          mainWindow.webContents.send("hmi", { event: "RESTART" });
        }
      },
      {
        label: "Add Takeoff",
        accelerator: "CmdOrCtrl + T",
        click: () => {
          mainWindow.webContents.send("takeoff_land");
        }
      }
    ]
  },
  {
    label: "Configure",
    submenu: [
      {
        label: "Ports",
        accelerator: "CmdOrCtrl + P",
        click: (item, window, options) => {
            mainWindow.webContents.send("settings", { type: "Ports" });
        }
      }
    ]
  }
];
const menu = Menu.buildFromTemplate(template);
Menu.setApplicationMenu(menu);

function createWindow(layout) {
  mainWindow = new BrowserWindow({
    width: layout.primary.width,
    height: layout.primary.height,
    x: layout.primary.x,
    y: layout.primary.y,
    icon,
    show: true,
    webPreferences: {
      nodeIntegration: true,
      preload: __dirname + "/preload.js"
    },
    id: "map"
  });
  mainWindow.setTitle("Map View");
  mainWindow.loadURL(
    isDev
      ? "http://localhost:3000"
      : url.format({
          pathname: join(__dirname, "./build/index.html"),
          protocol: "file:",
          slashes: true
        })
  );
//   mainWindow.on("")
  mainWindow.on("page-title-updated", function (e) {
    e.preventDefault();
    mainWindow.setIcon(icon);
    if (!setSizeAndPosition) {
      const mainWindowBounds = mainWindow.getBounds();
      const screenElectron = electron.screen;
      const { workArea } = screenElectron.getPrimaryDisplay();
      const half = workArea.width / 2;
      const halfA = Math.floor(half);
    //   const halfB = Math.ceil(half);
      mainWindow.setBounds({ width: halfA });
      setSizeAndPosition = true;
    }
  });
  mainWindow.on("ready-to-show", () => {
    mainWindow.show();
  });
  mainWindow.on("close", (e) => {
    if (mainWindow) {
      e.preventDefault();
      mainWindow.webContents.send("app-close");
    }
  });
  mainWindow.on("closed", () => {
    mainWindow = null;
  });

  global.mainWindow = mainWindow;
}

app.on("ready", () => {
  const screenElectron = electron.screen;
  const { workArea, size } = screenElectron.getPrimaryDisplay();
  const layout = {
    primary: {
      x: 0,
      y: 0,
      width: workArea.width / 2,
      height: workArea.height
    },
    secondary: {
      x: size.width - workArea.width / 2,
      y: 0,
      width: workArea.width / 2,
      height: workArea.height
    }
  };
  createWindow(layout);
  require("./socket-server");
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") {
    app.quit();
  }
});

app.on("will-quit", () => {
  if (mainWindow) {
    e.preventDefault();
    mainWindow.webContents.send("app-close");
  }
});

app.on("before-quit", () => {
  if (mainWindow) {
    e.preventDefault();
    mainWindow.webContents.send("app-close");
  }
});

app.on("activate", () => {
  const screenElectron = electron.screen;
  const { workArea, size } = screenElectron.getPrimaryDisplay();
  const layout = {
    primary: {
      x: 0,
      y: 0,
      width: workArea.width / 2,
      height: workArea.height
    },
    secondary: {
      x: size.width - workArea.width / 2,
      y: 0,
      width: workArea.width / 2,
      height: workArea.height
    }
  };
  if (mainWindow === null) {
    createWindow(layout);
    require("./socket-server");
  }
});

ipcMain.on("reload", () => {
  mainWindow.reload();
});

ipcMain.on("closed", (event, { messages }) => {
  const filename = moment().format();
  try {
    const dir = "./logs";
    if (!fs.existsSync(dir)) {
      fs.mkdirSync(dir);
    }
    fs.writeFileSync(`./logs/${filename}.json`, messages);
    console.log("wrote logfile");
  } catch (error) {
    console.log("Error creating logfile");
  }
  mainWindow = null;
  clearInterval(global.interval);
  if (process.platform !== "darwin") {
    app.quit();
  }
});
