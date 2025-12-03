import XMLJSONBase from "./XMLJSONBase";
import Pdo from "./Pdo";

export class SyncManager extends XMLJSONBase {
  constructor(isTx, pdoes) {
    super();
    if (isTx == null) {
      isTx = "false";
    }
    this.isTx = isTx;
    this.pdoes = pdoes;
  }

  static FromXML = (node) =>
    new SyncManager(
      node.getAttribute("is_tx"),
      [...node.getElementsByTagName("Pdo")]
        .filter((node) => node.nodeType === 1)
        .map(Pdo.FromXML)
    );

  static FromJSON = ({ isTx, pdoes }) =>
    new SyncManager(isTx, pdoes && pdoes.map(Pdo.FromJSON));

  toXML = () =>
    `<SyncManager ${SyncManager.ToXMLAttrString("is_tx", this.isTx)}>${
      this.pdoes ? this.pdoes.map((p) => p.toXML()).join("") : ""
    }</SyncManager>`;

  toJSON = () => ({
    isTx: this.isTx,
    pdoes: this.pdoes && this.pdoes.map((p) => p.toJSON()),
  });
}

export default SyncManager;
