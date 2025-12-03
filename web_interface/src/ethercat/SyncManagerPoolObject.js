import XMLJSONBase from "./XMLJSONBase";
import SyncManager from "./SyncManager";

export class SyncManagerPoolObject extends XMLJSONBase {
  constructor(syncManagers) {
    super();
    this.syncManagers = syncManagers;
  }

  static FromXML = (node) =>
    new SyncManagerPoolObject(
      [...node.getElementsByTagName("SyncManager")]
        .filter((node) => node.nodeType === 1)
        .map(SyncManager.FromXML)
    );

  static FromJSON = ({ syncManagers }) =>
    new SyncManagerPoolObject(
      syncManagers && syncManagers.map(SyncManager.FromJSON)
    );

  toXML = () =>
    `<SyncManagerPoolObject>${
      this.syncManagers
        ? this.syncManagers.map((sm) => sm.toXML()).join("")
        : ""
    }</SyncManagerPoolObject>`;

  toJSON = () => ({
    syncManagers:
      this.syncManagers && this.syncManagers.map((sm) => sm.toJSON()),
  });
}

export default SyncManagerPoolObject;
