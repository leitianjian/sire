import XMLJSONBase from "./XMLJSONBase";
import SyncManagerPoolObject from "./SyncManagerPoolObject";

export class EthercatMotor extends XMLJSONBase {
  constructor(
    uuid,
    isVirtual,
    phyId,
    vendorId,
    productCode,
    sync0ShiftNs,
    revisionNum,
    dcAssignActivate,
    maxPos,
    minPos,
    maxVel,
    minVel,
    maxAcc,
    minAcc,
    maxPosFollowingError,
    maxVelFollowingError,
    posFactor,
    posOffset,
    homePos,
    syncManagerPoolObjects
  ) {
    super();
    this.uuid = uuid;
    this.isVirtual = isVirtual;
    this.phyId = phyId;
    this.vendorId = vendorId;
    this.productCode = productCode;
    this.sync0ShiftNs = sync0ShiftNs;
    this.revisionNum = revisionNum;
    this.dcAssignActivate = dcAssignActivate;

    this.maxPos = maxPos;
    this.minPos = minPos;
    this.maxVel = maxVel;
    this.minVel = minVel;
    this.maxAcc = maxAcc;
    this.minAcc = minAcc;
    this.maxPosFollowingError = maxPosFollowingError;
    this.maxVelFollowingError = maxVelFollowingError;
    this.posFactor = posFactor;
    this.posOffset = posOffset;
    this.homePos = homePos;
    this.syncManagerPoolObjects = syncManagerPoolObjects;
  }

  static FromXML = (node) => {
    const isVirtual = EthercatMotor.GetBool(node.getAttribute("is_virtual"));
    const phyId = EthercatMotor.GetNumber(node.getAttribute("phy_id"));
    const vendorId = node.getAttribute("vendor_id");
    const productCode = node.getAttribute("product_code");
    const revisionNum = node.getAttribute("revision_num");
    const sync0ShiftNs = node.getAttribute("sync0_shift_ns");
    const dcAssignActivate = node.getAttribute("dc_assign_activate");
    const maxPos = EthercatMotor.GetDegreeNumber(node.getAttribute("max_pos"));
    const minPos = EthercatMotor.GetDegreeNumber(node.getAttribute("min_pos"));
    const maxVel = EthercatMotor.GetDegreeNumber(node.getAttribute("max_vel"));
    const minVel = EthercatMotor.GetDegreeNumber(node.getAttribute("min_vel"));
    const maxAcc = EthercatMotor.GetDegreeNumber(node.getAttribute("max_acc"));
    const minAcc = EthercatMotor.GetDegreeNumber(node.getAttribute("min_acc"));
    const maxPosFollowingError = EthercatMotor.GetNumber(
      node.getAttribute("max_pos_following_error")
    );
    const maxVelFollowingError = EthercatMotor.GetNumber(
      node.getAttribute("max_vel_following_error")
    );
    const posFactor = EthercatMotor.GetNumber(node.getAttribute("pos_factor"));
    const posOffset = EthercatMotor.GetDegreeNumber(
      node.getAttribute("pos_offset")
    );
    const homePos = EthercatMotor.GetDegreeNumber(
      node.getAttribute("home_pos")
    );

    return new EthercatMotor(
      Math.random().toString(36),
      isVirtual,
      phyId,
      vendorId,
      productCode,
      sync0ShiftNs,
      revisionNum,
      dcAssignActivate,
      maxPos,
      minPos,
      maxVel,
      minVel,
      maxAcc,
      minAcc,
      maxPosFollowingError,
      maxVelFollowingError,
      posFactor,
      posOffset,
      homePos,
      [...node.getElementsByTagName("SyncManagerPoolObject")]
        .filter((node) => node.nodeType === 1)
        .map(SyncManagerPoolObject.FromXML)
    );
  };

  // 将js对象数据转化为json数据
  static FromJSON = ({
    uuid,
    isVirtual,
    phyId,
    vendorId,
    productCode,
    sync0ShiftNs,
    revisionNum,
    dcAssignActivate,
    maxPos,
    minPos,
    maxVel,
    minVel,
    maxAcc,
    minAcc,
    maxPosFollowingError,
    maxVelFollowingError,
    posFactor,
    posOffset,
    homePos,
    syncManagerPoolObjects,
  }) =>
    new EthercatMotor(
      uuid || Math.random().toString(36),
      isVirtual,
      phyId,
      vendorId,
      productCode,
      sync0ShiftNs,
      revisionNum,
      dcAssignActivate,
      maxPos,
      minPos,
      maxVel,
      minVel,
      maxAcc,
      minAcc,
      maxPosFollowingError,
      maxVelFollowingError,
      posFactor,
      posOffset,
      homePos,
      syncManagerPoolObjects &&
        syncManagerPoolObjects.map(SyncManagerPoolObject.FromJSON)
    );

  toXML = () =>
    `<EthercatMotor ${EthercatMotor.ToXMLAttrString(
      "is_virtual",
      this.isVirtual
    )} ${EthercatMotor.ToXMLAttrString(
      "phy_id",
      this.phyId
    )} ${EthercatMotor.ToXMLAttrString(
      "vendor_id",
      this.vendorId
    )} ${EthercatMotor.ToXMLAttrString(
      "product_code",
      this.productCode
    )} ${EthercatMotor.ToXMLAttrString(
      "sync0_shift_ns",
      this.sync0ShiftNs
    )} ${EthercatMotor.ToXMLAttrString(
      "revision_num",
      this.revisionNum
    )} ${EthercatMotor.ToXMLAttrString(
      "dc_assign_activate",
      this.dcAssignActivate
    )} ${EthercatMotor.ToXMLAttrString(
      "max_pos",
      EthercatMotor.ToRadian(this.maxPos)
    )} ${EthercatMotor.ToXMLAttrString(
      "min_pos",
      EthercatMotor.ToRadian(this.minPos)
    )}  ${EthercatMotor.ToXMLAttrString(
      "max_vel",
      EthercatMotor.ToRadian(this.maxVel)
    )} ${EthercatMotor.ToXMLAttrString(
      "min_vel",
      EthercatMotor.ToRadian(this.minVel)
    )} ${EthercatMotor.ToXMLAttrString(
      "max_acc",
      EthercatMotor.ToRadian(this.maxAcc)
    )} ${EthercatMotor.ToXMLAttrString(
      "min_acc",
      EthercatMotor.ToRadian(this.minAcc)
    )} ${EthercatMotor.ToXMLAttrString(
      "max_pos_following_error",
      this.maxPosFollowingError
    )} ${EthercatMotor.ToXMLAttrString(
      "max_vel_following_error",
      this.maxVelFollowingError
    )} ${EthercatMotor.ToXMLAttrString(
      "pos_factor",
      this.posFactor
    )} ${EthercatMotor.ToXMLAttrString(
      "pos_offset",
      EthercatMotor.ToRadian(this.posOffset)
    )} ${EthercatMotor.ToXMLAttrString(
      "home_pos",
      EthercatMotor.ToRadian(this.homePos)
    )}>${
      this.syncManagerPoolObjects
        ? this.syncManagerPoolObjects.map((smpo) => smpo.toXML()).join("")
        : ""
    }</EthercatMotor>`;

  toJSON = () => ({
    isVirtual: this.isVirtual,
    uuid: this.uuid,
    isMotion: true,
    phyId: this.phyId,
    vendorId: this.vendorId,
    productCode: this.productCode,
    sync0ShiftNs: this.sync0ShiftNs,
    revisionNum: this.revisionNum,
    dcAssignActivate: this.dcAssignActivate,
    maxPos: this.maxPos,
    minPos: this.minPos,
    maxVel: this.maxVel,
    minVel: this.minVel,
    maxAcc: this.maxAcc,
    minAcc: this.minAcc,
    maxPosFollowingError: this.maxPosFollowingError,
    maxVelFollowingError: this.maxVelFollowingError,
    posFactor: this.posFactor,
    posOffset: this.posOffset,
    homePos: this.homePos,
    syncManagerPoolObjects:
      this.syncManagerPoolObjects &&
      this.syncManagerPoolObjects.map((smpo) => smpo.toJSON()),
  });
}

export default EthercatMotor;
