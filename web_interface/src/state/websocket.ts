import store from "../index";
import { updateWebsocketStatus } from "./actions";

export class Msg {
  buffer: ArrayBufferLike;
  constructor(buffer: ArrayBufferLike) {
    this.buffer = buffer;
  }

  size(): number {
    return new Uint32Array(this.buffer, 0, 4)[0];
  }

  get id(): number {
    return new Uint32Array(this.buffer, 4, 8)[0];
  }

  set id(value) {
    new Uint32Array(this.buffer, 4, 8)[0] = value;
  }

  get type(): number {
    return new Float64Array(this.buffer, 8, 16)[0];
  }

  set type(value) {
    new Float64Array(this.buffer, 8, 16)[0] = value;
  }

  get reserved1(): number {
    return new Float64Array(this.buffer, 16, 1)[0];
  }

  set reserved1(value) {
    new Float64Array(this.buffer, 16, 1)[0] = value;
  }

  get reserved2(): number {
    return new Float64Array(this.buffer, 24, 8)[0];
  }

  set reserved2(value) {
    new Float64Array(this.buffer, 24, 8)[0] = value;
  }

  get reserved3(): number {
    return new Float64Array(this.buffer, 32, 8)[0];
  }

  set reserved3(value) {
    new Float64Array(this.buffer, 32, 8)[0] = value;
  }

  get data(): string {
    return String.fromCharCode.apply(
      null,
      // @ts-expect-error
      new Uint8Array(this.buffer.slice(40))
    );
  }

  get jsData(): any {
    return JSON.parse(this.data);
  }

  static make(str: number[], headBuffer: ArrayBuffer | null = null): Msg {
    if (headBuffer === null) headBuffer = new ArrayBuffer(40);
    const headView = new Int32Array(headBuffer, 0, 1);
    headView[0] = str.length;
    const dataSent = new Uint8Array(str.length + 40);
    dataSent.set(new Uint8Array(headBuffer), 0);
    for (let i = 0, strLen = str.length; i < strLen; i++) {
      dataSent[i + 40] = str[i];
    }
    return new Msg(dataSent.buffer);
  }
}
class Command {
  msg: Msg;
  callback: Function | null;

  constructor(msg: Msg, callback: Function | null) {
    this.msg = msg;
    this.callback = callback;
  }
}
class Robot {
  robotWebsocket: WebSocket | null;
  cmdMap: Map<number, Command>;
  cmdReserved1: number;
  onConnect: Function | null;
  onReceivedMsg: Function | null;
  onSendingMsg: Function | null;
  onClose: Function | null;

  constructor() {
    this.robotWebsocket = null;
    this.cmdMap = new Map();
    this.cmdReserved1 = 1000;
    this.onConnect = null;
    this.onReceivedMsg = null;
    this.onSendingMsg = null;
    this.onClose = null;
  }

  setOnConnect(onConnect: Function | null): void {
    this.onConnect = onConnect;
  }

  setOnReceivedMsg(onReceivedMsg: Function | null): void {
    this.onReceivedMsg = onReceivedMsg;
  }

  setOnSendingMsg(onSendingMsg: Function | null): void {
    this.onSendingMsg = onSendingMsg;
  }

  setOnClose(onClose: Function | null): void {
    this.onClose = onClose;
  }

  connect(): void {
    try {
      this.cmdMap = new Map();
      this.cmdReserved1 = Date.now();
      this.robotWebsocket = new WebSocket(
        "ws://" + window.location.host.split(":")[0] + ":5867"
      );
    } catch (e) {
      console.log(e);
    }
    if (this.robotWebsocket == null) {
      console.log("initialize websocket error!");
      return;
    }
    this.robotWebsocket.binaryType = "arraybuffer";

    this.robotWebsocket.onopen = function () {
      store.dispatch(updateWebsocketStatus({ websocketConnected: true }));
    };

    this.robotWebsocket.onmessage = function (event) {
      if (typeof event.data === "string") {
        console.log("Received data string");
      }

      if (event.data instanceof ArrayBuffer) {
        const msg = new Msg(event.data);
        const cmd = websock.cmdMap.get(msg.reserved1);
        websock.cmdMap.delete(msg.reserved1);
        if (cmd?.callback === null) return;
        cmd?.callback(msg);
      }
    };

    this.robotWebsocket.onclose = (event) => {
      store.dispatch(updateWebsocketStatus({ websocketConnected: false }));
      setTimeout(() => {
        this.connect();
      }, 500);
      // this.connect();
    };
    this.robotWebsocket.onerror = (event) => {
      console.error('Socket encountered error: ', 'Closing socket');
      this.robotWebsocket?.close();
    };
  }

  sendCmd(
    cmdStr: string,
    successCallback: ((msg: Msg) => void) | null = null,
    failCallback: ((msg: Msg) => void) | null = null
  ): void {
    if (this.robotWebsocket == null) {
      // alert("websocket not initialized...");
      return;
    }
    if (this.robotWebsocket.readyState !== WebSocket.OPEN) {
      // alert("robot not connected...");
      return;
    }

    // encode to utf8
    function encodeUtf8(text: string): number[] {
      const code: string = encodeURIComponent(text);
      const bytes: number[] = [];
      for (let i = 0; i < code.length; i++) {
        const c = code.charAt(i);
        if (c === "%") {
          const hex = code.charAt(i + 1) + code.charAt(i + 2);
          const hexVal: number = parseInt(hex, 16);
          bytes.push(hexVal);
          i += 2;
        } else bytes.push(c.charCodeAt(0));
      }
      return bytes;
    }

    const str2utf8: number[] = encodeUtf8(cmdStr);

    // 构造msg //
    const msg = Msg.make(str2utf8);
    msg.reserved1 = Date.now();
    if (msg.reserved1 <= this.cmdReserved1) {
      msg.reserved1 = this.cmdReserved1 + 1;
    }
    this.cmdReserved1 = msg.reserved1;

    // 保存到map中 //
    this.cmdMap.set(msg.reserved1, new Command(msg, successCallback));
    if (this.onSendingMsg != null) this.onSendingMsg(msg);
    this.robotWebsocket.send(msg.buffer);
  }
}

export const websock = new Robot();
websock.connect();
