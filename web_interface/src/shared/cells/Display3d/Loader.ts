import {
  Group,
  LoadingManager,
  BufferGeometryLoader,
  ObjectLoader,
  PointsMaterial,
  MeshStandardMaterial,
  Mesh,
  Points,
  MeshBasicMaterial,
  ShapeGeometry,
} from "three";
import { TGALoader, Chunk } from "three/examples/jsm/Addons";
import { unzipSync, strFromU8 } from "three/examples/jsm/libs/fflate.module";

type FilesMap = { [key: string]: File };

function formatNumber(number: number) {
  return new Intl.NumberFormat("en-us", { useGrouping: true }).format(number);
}

const getFilesFromItemList = function (
  items: DataTransferItemList,
  onDone: (files: File[], filesMap: FilesMap) => void
) {
  // TOFIX: setURLModifier() breaks when the file being loaded is not in root
  let itemsCount = 0;
  let itemsTotal = 0;
  const files: File[] = [];
  const filesMap: FilesMap = {};
  function onEntryHandled() {
    itemsCount++;
    if (itemsCount === itemsTotal) {
      onDone(files, filesMap);
    }
  }

  function handleEntry(entry: FileSystemEntry) {
    if (entry.isDirectory) {
      const reader = (entry as FileSystemDirectoryEntry).createReader();
      reader.readEntries(function (entries) {
        for (let i = 0; i < entries.length; i++) {
          handleEntry(entries[i]);
        }
        onEntryHandled();
      });
    } else if (entry.isFile) {
      (entry as FileSystemFileEntry).file(function (file) {
        files.push(file);
        filesMap[(entry as FileSystemFileEntry).fullPath.slice(1)] = file;
        onEntryHandled();
      });
    }
    itemsTotal++;
  }
  for (let i = 0; i < items.length; i++) {
    const item = items[i];
    if (item.kind === "file") {
      handleEntry(item.webkitGetAsEntry() as FileSystemEntry);
    }
  }
};

const loadItemList = function (items: DataTransferItemList) {
  getFilesFromItemList(items, function (files, filesMap) {
    loadFiles(files, filesMap);
  });
};

const createFilesMap = function (files: File[]) {
  const map: FilesMap = {};
  for (let i = 0; i < files.length; i++) {
    const file = files[i];
    map[file.name] = file;
  }
  return map;
};

const loadFiles = function (files: File[], filesMap: FilesMap) {
  if (files.length > 0) {
    filesMap = filesMap || createFilesMap(files);
    const manager = new LoadingManager();
    manager.setURLModifier(function (url: string) {
      url = url.replace(/^(\.?\/)/, ""); // remove './'
      const file = filesMap[url];
      if (file) {
        console.log("Loading", url);
        return URL.createObjectURL(file);
      }
      return url;
    });
    manager.addHandler(/\.tga$/i, new TGALoader());
    for (let i = 0; i < files.length; i++) {
      loadFile(files[i], manager);
    }
  }
};

const loadFile = function (file: File, manager: LoadingManager) {
  const filename = file.name;
  const extension = filename.split(".").pop()?.toLowerCase();
  const reader = new FileReader();

  reader.addEventListener("progress", function (event) {
    const size = "(" + formatNumber(Math.floor(event.total / 1000)) + " KB)";
    const progress = Math.floor((event.loaded / event.total) * 100) + "%";
    console.log("Loading", filename, size, progress);
  });

  switch (extension) {
    case "3dm": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result;
          const { Rhino3dmLoader } = await import(
            "three/examples/jsm/loaders/3DMLoader"
          );
          const loader = new Rhino3dmLoader();
          loader.setLibraryPath("../examples/jsm/libs/rhino3dm/");
          loader.parse(
            contents as ArrayBuffer,
            function (object) {
              object.name = filename;
              // // editor.execute( new AddObjectCommand( editor, object ) );
            },
            function (error) {
              console.error(error);
            }
          );
        },
        false
      );
      reader.readAsArrayBuffer(file);
      break;
    }
    case "3ds": {
      reader.addEventListener(
        "load",
        async function (event) {
          const { TDSLoader } = await import(
            "three/examples/jsm/loaders/TDSLoader"
          );
          const loader = new TDSLoader();
          if (!event.target) {
            console.error("Event target is null.");
            return;
          }
          const object = loader.parse(event.target?.result as ArrayBuffer, "");
          // // editor.execute(new AddObjectCommand(editor, object));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }
    case "3mf": {
      reader.addEventListener(
        "load",
        async function (event) {
          const { ThreeMFLoader } = await import(
            "three/examples/jsm/loaders/3MFLoader"
          );
          const loader = new ThreeMFLoader();
          const object = loader.parse(event.target?.result as ArrayBuffer);
          // // editor.execute(new AddObjectCommand(editor, object));
        },
        false
      );
      reader.readAsArrayBuffer(file);
      break;
    }

    case "amf": {
      reader.addEventListener(
        "load",
        async function (event) {
          const { AMFLoader } = await import(
            "three/examples/jsm/loaders/AMFLoader"
          );

          const loader = new AMFLoader();
          const amfobject = loader.parse(event.target?.result as ArrayBuffer);

          // editor.execute(new AddObjectCommand(editor, amfobject));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    case "dae": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as string;

          const { ColladaLoader } = await import(
            "three/examples/jsm/loaders/ColladaLoader"
          );

          const loader = new ColladaLoader(manager);
          const collada = loader.parse(contents, "");

          collada.scene.name = filename;

          // editor.execute(new AddObjectCommand(editor, collada.scene));
        },
        false
      );
      reader.readAsText(file);

      break;
    }

    case "drc": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as ArrayBuffer;

          const { DRACOLoader } = await import(
            "three/examples/jsm/loaders/DRACOLoader"
          );

          const loader = new DRACOLoader();
          loader.setDecoderPath("../examples/jsm/libs/draco/");
          loader.parse(contents, function (geometry) {
            let object;

            if (geometry.index !== null) {
              const material = new MeshStandardMaterial();

              object = new Mesh(geometry, material);
              object.name = filename;
            } else {
              const material = new PointsMaterial({ size: 0.01 });
              material.vertexColors = geometry.hasAttribute("color");

              object = new Points(geometry, material);
              object.name = filename;
            }

            loader.dispose();
            // editor.execute(new AddObjectCommand(editor, object));
          });
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    case "fbx": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as ArrayBuffer;

          const { FBXLoader } = await import(
            "three/examples/jsm/loaders/FBXLoader"
          );

          const loader = new FBXLoader(manager);
          const object = loader.parse(contents, "");

          // editor.execute(new AddObjectCommand(editor, object));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    // case "glb": {
    //   reader.addEventListener(
    //     "load",
    //     async function (event) {
    //       const contents = event.target?.result as ArrayBuffer;

    //       const loader = await createGLTFLoader();

    //       loader.parse(contents, "", function (result) {
    //         const scene = result.scene;
    //         scene.name = filename;

    //         scene.animations.push(...result.animations);
    //         // editor.execute(new AddObjectCommand(editor, scene));

    //         loader.dracoLoader?.dispose();
    //         loader.ktx2Loader?.dispose();
    //       });
    //     },
    //     false
    //   );
    //   reader.readAsArrayBuffer(file);

    //   break;
    // }

    // case "gltf": {
    //   reader.addEventListener(
    //     "load",
    //     async function (event) {
    //       const contents = event.target?.result as ArrayBuffer;

    //       const loader = await createGLTFLoader(manager);

    //       loader.parse(contents, "", function (result) {
    //         const scene = result.scene;
    //         scene.name = filename;

    //         scene.animations.push(...result.animations);
    //         // editor.execute(new AddObjectCommand(editor, scene));

    //         loader.dracoLoader?.dispose();
    //         loader.ktx2Loader?.dispose();
    //       });
    //     },
    //     false
    //   );
    //   reader.readAsArrayBuffer(file);

    //   break;
    // }

    case "js":
    case "json": {
      reader.addEventListener(
        "load",
        function (event) {
          const contents = event.target?.result as string;

          // 2.0

          if (contents.indexOf("postMessage") !== -1) {
            const blob = new Blob([contents], { type: "text/javascript" });
            const url = URL.createObjectURL(blob);

            const worker = new Worker(url);

            worker.onmessage = function (event) {
              event.data.metadata = { version: 2 };
              handleJSON(event.data);
            };

            worker.postMessage(Date.now());

            return;
          }

          // >= 3.0

          let data;

          try {
            data = JSON.parse(contents);
          } catch (error) {
            alert(error);
            return;
          }

          handleJSON(data);
        },
        false
      );
      reader.readAsText(file);

      break;
    }

    case "kmz": {
      reader.addEventListener(
        "load",
        async function (event) {
          const { KMZLoader } = await import(
            "three/examples/jsm/loaders/KMZLoader"
          );

          const loader = new KMZLoader();
          const collada = loader.parse(event.target?.result as ArrayBuffer);

          collada.scene.name = filename;

          // editor.execute(new AddObjectCommand(editor, collada.scene));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    // case "ldr":
    // case "mpd": {
    //   reader.addEventListener(
    //     "load",
    //     async function (event) {
    //       const { LDrawLoader } = await import(
    //         "three/examples/jsm/loaders/LDrawLoader"
    //       );

    //       const loader = new LDrawLoader();
    //       loader.setPath("../../examples/models/ldraw/officialLibrary/");
    //       loader.parse(
    //         event.target?.result as string,
    //         "",
    //         function (group: Group) {
    //           group.name = filename;
    //           // Convert from LDraw coordinates: rotate 180 degrees around OX
    //           group.rotation.x = Math.PI;

    //           // editor.execute(new AddObjectCommand(editor, group));
    //         },
    //         () => {}
    //       );
    //     },
    //     false
    //   );
    //   reader.readAsText(file);

    //   break;
    // }
    // case "md2": {
    //   reader.addEventListener(
    //     "load",
    //     async function (event) {
    //       const contents = event.target?.result as ArrayBuffer;

    //       const { MD2Loader } = await import(
    //         "three/examples/jsm/loaders/MD2Loader"
    //       );

    //       const geometry = new MD2Loader().parse(contents);
    //       const material = new THREE.MeshStandardMaterial();

    //       const mesh = new THREE.Mesh(geometry, material);
    //       mesh.mixer = new THREE.AnimationMixer(mesh);
    //       mesh.name = filename;

    //       mesh.animations.push(...geometry.animations);
    //       // editor.execute(new AddObjectCommand(editor, mesh));
    //     },
    //     false
    //   );
    //   reader.readAsArrayBuffer(file);

    //   break;
    // }
    case "obj": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as string;

          const { OBJLoader } = await import(
            "three/examples/jsm/loaders/OBJLoader"
          );

          const object = new OBJLoader().parse(contents);
          object.name = filename;

          // editor.execute(new AddObjectCommand(editor, object));
        },
        false
      );
      reader.readAsText(file);

      break;
    }

    case "pcd": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as ArrayBuffer;

          const { PCDLoader } = await import(
            "three/examples/jsm/loaders/PCDLoader"
          );

          const points = new PCDLoader().parse(contents);
          points.name = filename;

          // editor.execute(new AddObjectCommand(editor, points));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    case "ply": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as ArrayBuffer;

          const { PLYLoader } = await import(
            "three/examples/jsm/loaders/PLYLoader"
          );

          const geometry = new PLYLoader().parse(contents);
          let object;

          if (geometry.index !== null) {
            const material = new MeshStandardMaterial();

            object = new Mesh(geometry, material);
            object.name = filename;
          } else {
            const material = new PointsMaterial({ size: 0.01 });
            material.vertexColors = geometry.hasAttribute("color");

            object = new Points(geometry, material);
            object.name = filename;
          }

          // editor.execute(new AddObjectCommand(editor, object));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    case "stl": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as ArrayBuffer;

          const { STLLoader } = await import(
            "three/examples/jsm/loaders/STLLoader"
          );

          const geometry = new STLLoader().parse(contents);
          const material = new MeshStandardMaterial();

          const mesh = new Mesh(geometry, material);
          mesh.name = filename;

          // editor.execute(new AddObjectCommand(editor, mesh));
        },
        false
      );

      if (reader.readAsBinaryString !== undefined) {
        reader.readAsBinaryString(file);
      } else {
        reader.readAsArrayBuffer(file);
      }

      break;
    }

    case "svg": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as string;

          const { SVGLoader } = await import(
            "three/examples/jsm/loaders/SVGLoader"
          );

          const loader = new SVGLoader();
          const paths = loader.parse(contents).paths;

          //

          const group = new Group();
          group.name = filename;
          group.scale.multiplyScalar(0.1);
          group.scale.y *= -1;

          for (let i = 0; i < paths.length; i++) {
            const path = paths[i];

            const material = new MeshBasicMaterial({
              color: path.color,
              depthWrite: false,
            });

            const shapes = SVGLoader.createShapes(path);

            for (let j = 0; j < shapes.length; j++) {
              const shape = shapes[j];

              const geometry = new ShapeGeometry(shape);
              const mesh = new Mesh(geometry, material);

              group.add(mesh);
            }
          }

          // editor.execute(new AddObjectCommand(editor, group));
        },
        false
      );
      reader.readAsText(file);

      break;
    }

    case "usdz": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as ArrayBuffer;

          const { USDZLoader } = await import(
            "three/examples/jsm/loaders/USDZLoader"
          );

          const group = new USDZLoader().parse(contents);
          group.name = filename;

          // editor.execute(new AddObjectCommand(editor, group));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    case "vox": {
      reader.addEventListener(
        "load",
        async function (event) {
          const contents = event.target?.result as ArrayBuffer;

          const { VOXLoader, VOXMesh } = await import(
            "three/examples/jsm/loaders/VOXLoader"
          );

          const chunks = new VOXLoader().parse(contents);

          const group = new Group();
          group.name = filename;

          for (let i = 0; i < chunks.length; i++) {
            const chunk = chunks[i];

            const mesh = new VOXMesh(chunk as Chunk);
            group.add(mesh);
          }

          // editor.execute(new AddObjectCommand(editor, group));
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    // case "vtk":
    // case "vtp": {
    //   reader.addEventListener(
    //     "load",
    //     async function (event) {
    //       const contents = event.target?.result as ArrayBuffer;

    //       const { VTKLoader } = await import(
    //         "three/examples/jsm/loaders/VTKLoader"
    //       );

    //       const geometry = new VTKLoader().parse(contents, "");
    //       const material = new MeshStandardMaterial();

    //       const mesh = new Mesh(geometry, material);
    //       mesh.name = filename;

    //       // editor.execute(new AddObjectCommand(editor, mesh));
    //     },
    //     false
    //   );
    //   reader.readAsArrayBuffer(file);

    //   break;
    // }

    // case "wrl": {
    //   reader.addEventListener(
    //     "load",
    //     async function (event) {
    //       const contents = event.target?.result as ArrayBuffer;

    //       const { VRMLLoader } = await import(
    //         "three/examples/jsm/loaders/VRMLLoader"
    //       );

    //       const result = new VRMLLoader().parse(contents);

    //       // editor.execute(new AddObjectCommand(editor, result));
    //     },
    //     false
    //   );
    //   reader.readAsText(file);

    //   break;
    // }

    // case "xyz": {
    //   reader.addEventListener(
    //     "load",
    //     async function (event) {
    //       const contents = event.target?.result as ArrayBuffer;

    //       const { XYZLoader } = await import(
    //         "three/examples/jsm/loaders/XYZLoader"
    //       );

    //       const geometry = new XYZLoader().parse(contents);

    //       const material = new PointsMaterial();
    //       material.vertexColors = geometry.hasAttribute("color");

    //       const points = new Points(geometry, material);
    //       points.name = filename;

    //       // editor.execute(new AddObjectCommand(editor, points));
    //     },
    //     false
    //   );
    //   reader.readAsText(file);

    //   break;
    // }

    case "zip": {
      reader.addEventListener(
        "load",
        function (event) {
          handleZIP(event.target?.result as ArrayBuffer);
        },
        false
      );
      reader.readAsArrayBuffer(file);

      break;
    }

    default:
      console.error("Unsupported file format (" + extension + ").");

      break;
  }
};

function handleJSON(data: any) {
  if (data.metadata === undefined) {
    // 2.0

    data.metadata = { type: "Geometry" };
  }

  if (data.metadata.type === undefined) {
    // 3.0

    data.metadata.type = "Geometry";
  }

  if (data.metadata.formatVersion !== undefined) {
    data.metadata.version = data.metadata.formatVersion;
  }

  switch (data.metadata.type.toLowerCase()) {
    case "buffergeometry": {
      const loader = new BufferGeometryLoader();
      const result = loader.parse(data);

      const mesh = new Mesh(result);

      // editor.execute( new AddObjectCommand( editor, mesh ) );

      break;
    }

    case "geometry":
      console.error('Loader: "Geometry" is no longer supported.');

      break;

    case "object": {
      const loader = new ObjectLoader();
      // loader.setResourcePath( scope.texturePath );

      loader.parse(data, function (result) {
        // editor.execute( new AddObjectCommand( editor, result ) );
      });

      break;
    }

    case "app":
      // editor.fromJSON( data );

      break;
  }
}

async function handleZIP(contents: ArrayBuffer) {
  const zip = unzipSync(new Uint8Array(contents));

  const manager = new LoadingManager();
  manager.setURLModifier(function (url) {
    const file = zip[url];

    if (file) {
      console.log("Loading", url);

      const blob = new Blob([file.buffer as ArrayBuffer], {
        type: "application/octet-stream",
      });
      return URL.createObjectURL(blob);
    }

    return url;
  });

  // Poly

  if (zip["model.obj"] && zip["materials.mtl"]) {
    const { MTLLoader } = await import("three/examples/jsm/loaders/MTLLoader");
    const { OBJLoader } = await import("three/examples/jsm/loaders/OBJLoader");

    const materials = new MTLLoader(manager).parse(
      strFromU8(zip["materials.mtl"]),
      ""
    );
    const object = new OBJLoader()
      .setMaterials(materials)
      .parse(strFromU8(zip["model.obj"]));

    // editor.execute(new AddObjectCommand(editor, object));
    return;
  }

  //

  for (const path in zip) {
    const file = zip[path];

    const extension = path?.split(".")?.pop()?.toLowerCase();

    switch (extension) {
      case "fbx": {
        const { FBXLoader } = await import(
          "three/examples/jsm/loaders/FBXLoader"
        );

        const loader = new FBXLoader(manager);
        const object = loader.parse(file.buffer as ArrayBuffer, "");

        // editor.execute(new AddObjectCommand(editor, object));

        break;
      }

      case "glb": {
        const loader = await createGLTFLoader();

        loader.parse(file.buffer as ArrayBuffer, "", function (result) {
          const scene = result.scene;

          scene.animations.push(...result.animations);
          // editor.execute(new AddObjectCommand(editor, scene));

          loader.dracoLoader?.dispose();
          loader.ktx2Loader?.dispose();
        });

        break;
      }

      case "gltf": {
        const loader = await createGLTFLoader(manager);

        loader.parse(strFromU8(file), "", function (result) {
          const scene = result.scene;

          scene.animations.push(...result.animations);
          // editor.execute(new AddObjectCommand(editor, scene));

          loader.dracoLoader?.dispose();
          loader.ktx2Loader?.dispose();
        });

        break;
      }
    }
  }
}

async function createGLTFLoader(manager?: LoadingManager) {
  const { GLTFLoader } = await import("three/examples/jsm/loaders/GLTFLoader");
  const { DRACOLoader } = await import(
    "three/examples/jsm/loaders/DRACOLoader"
  );
  const { KTX2Loader } = await import("three/examples/jsm/loaders/KTX2Loader");
  const { MeshoptDecoder } = await import(
    "three/examples/jsm/libs/meshopt_decoder.module"
  );

  const dracoLoader = new DRACOLoader();
  dracoLoader.setDecoderPath("three/examples/jsm/libs/draco/gltf/");

  const ktx2Loader = new KTX2Loader(manager);
  ktx2Loader.setTranscoderPath("three/examples/jsm/libs/basis/");

  // editor.signals.rendererDetectKTX2Support.dispatch(ktx2Loader);

  const loader = new GLTFLoader(manager);
  loader.setDRACOLoader(dracoLoader);
  loader.setKTX2Loader(ktx2Loader);
  loader.setMeshoptDecoder(MeshoptDecoder);

  return loader;
}
