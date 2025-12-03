import XMLJSONBase from "./XMLJSONBase";
import Pdo from "./Pdo";

export class PdoEntry extends XMLJSONBase {
  constructor(name, index, subIndex, size) {
    super();
    this.name = name;
    this.index = index;
    this.subIndex = subIndex;
    this.size = size;
  }

  static FromXML = (node) =>
    new PdoEntry(
      node.getAttribute("name"),
      node.getAttribute("index"),
      node.getAttribute("subindex"),
      node.getAttribute("size")
    );

  static FromJSON = ({ name, index, subIndex, size }) =>
    new PdoEntry(name, index, subIndex, size);

  toXML = () =>
    `<PdoEntry ${PdoEntry.ToXMLAttrString(
      "name",
      this.name
    )} ${PdoEntry.ToXMLAttrString(
      "index",
      this.index
    )}  ${PdoEntry.ToXMLAttrString(
      "subindex",
      this.subIndex
    )} ${PdoEntry.ToXMLAttrString("size", this.size)} />`;

  toJSON = () => ({
    name: this.name,
    index: this.index,
    subIndex: this.subIndex,
    size: this.size,
  });
}
export default PdoEntry;
