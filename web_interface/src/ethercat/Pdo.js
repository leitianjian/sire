import XMLJSONBase from "./XMLJSONBase";
import PdoEntry from "./PdoEntry";

export class Pdo extends XMLJSONBase {
  constructor(index, name, entries) {
    super();
    this.index = index;
    this.name = name;
    this.entries = entries;
  }

  static FromXML = (node) =>
    new Pdo(
      node.getAttribute("index"),
      node.getAttribute("name"),
      [...node.getElementsByTagName("PdoEntry")]
        .filter((node) => node.nodeType === 1)
        .map(PdoEntry.FromXML)
    );

  static FromJSON = ({ index, name, entries }) =>
    new Pdo(index, name, entries && entries.map(PdoEntry.FromJSON));

  toXML = () => {
    return `<Pdo ${Pdo.ToXMLAttrString(
      "name",
      this.name
    )} ${Pdo.ToXMLAttrString("index", this.index)}>${
      this.entries ? this.entries.map((e) => e.toXML()).join("") : ""
    }</Pdo>`;
  };

  toJSON = () => ({
    name: this.name,
    index: this.index,
    entries: this.entries && this.entries.map((e) => e.toJSON()),
  });
}

export default Pdo;
