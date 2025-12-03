import XMLJSONBase from "./XMLJSONBase";
import SyncManagerPoolObject from "./SyncManagerPoolObject";

export class EthercatSlave extends XMLJSONBase {
  constructor(
    uuid,
    isVirtual,
    phyId,
    vendorId,
    productCode,
    sync0ShiftNs,
    revisionNum,
    dcAssignActivate,
    syncManagerPoolObjects
  ) {
    super();
    this.uuid = uuid;
    try {
      this.isVirtual = JSON.parse(isVirtual);
    } catch (e) {
      this.isVirtual = false;
    }
    try {
      this.phyId = JSON.parse(phyId);
    } catch (e) {
      this.phyId = 0;
    }

    this.vendorId = vendorId;
    this.productCode = productCode;
    this.sync0ShiftNs = sync0ShiftNs;
    this.revisionNum = revisionNum;
    this.dcAssignActivate = dcAssignActivate;
    this.syncManagerPoolObjects = syncManagerPoolObjects;
  }

  static FromXML = (node) =>
    new EthercatSlave(
      Math.random().toString(36),
      node.getAttribute("is_virtual"),
      node.getAttribute("phy_id"),
      node.getAttribute("vendor_id"),
      node.getAttribute("product_code"),
      node.getAttribute("sync0_shift_ns"),
      node.getAttribute("revision_num"),
      node.getAttribute("dc_assign_activate"),
      [...node.getElementsByTagName("SyncManagerPoolObject")]
        .filter((node) => node.nodeType === 1)
        .map(SyncManagerPoolObject.FromXML)
    );

  static FromJSON = ({
    uuid,
    isVirtual,
    phyId,
    vendorId,
    productCode,
    sync0ShiftNs,
    revisionNum,
    dcAssignActivate,
    syncManagerPoolObjects,
  }) =>
    new EthercatSlave(
      uuid || Math.random().toString(36),
      isVirtual,
      phyId,
      vendorId,
      productCode,
      sync0ShiftNs,
      revisionNum,
      dcAssignActivate,
      syncManagerPoolObjects &&
        syncManagerPoolObjects.map(SyncManagerPoolObject.FromJSON)
    );

  toXML = () =>
    `<EthercatSlave ${EthercatSlave.ToXMLAttrString(
      "is_virtual",
      this.isVirtual
    )} ${EthercatSlave.ToXMLAttrString(
      "phy_id",
      this.phyId
    )} ${EthercatSlave.ToXMLAttrString(
      "vendor_id",
      this.vendorId
    )} ${EthercatSlave.ToXMLAttrString(
      "product_code",
      this.productCode
    )} ${EthercatSlave.ToXMLAttrString(
      "sync0_shift_ns",
      this.sync0ShiftNs
    )} ${EthercatSlave.ToXMLAttrString(
      "revision_num",
      this.revisionNum
    )} ${EthercatSlave.ToXMLAttrString(
      "dc_assign_activate",
      this.dcAssignActivate
    )}> ${
      this.syncManagerPoolObjects
        ? this.syncManagerPoolObjects.map((smpo) => smpo.toXML()).join("")
        : ""
    }</EthercatSlave>`;

  toJSON = () => ({
    uuid: this.uuid,
    isVirtual: this.isVirtual,
    isMotion: false,
    phyId: this.phyId,
    vendorId: this.vendorId,
    productCode: this.productCode,
    sync0ShiftNs: this.sync0ShiftNs,
    revisionNum: this.revisionNum,
    dcAssignActivate: this.dcAssignActivate,
    syncManagerPoolObjects:
      this.syncManagerPoolObjects &&
      this.syncManagerPoolObjects.map((smpo) => smpo.toJSON()),
  });
}

export default EthercatSlave;
