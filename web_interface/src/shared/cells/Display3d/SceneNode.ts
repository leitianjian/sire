export{}
// import { Camera, Color, Group, Light, Material, Mesh, Object3D, Object3DEventMap, PointLight, Texture, Vector3Like, Vector3Tuple } from "three";
// import { Matrix4Array, PositionType, QuaternionType } from "~/types/lib";

// type WrappedMaterial = Material & {
//   base_opacity: number
// } 

// function isMesh(object: Object3D): object is Mesh{
//   return (object as Mesh).isMesh !== undefined && (object as Mesh).isMesh;
// }

// function isTexture(t: unknown): t is Texture {
//   return (t as Texture).isTexture !== undefined && (t as Texture).isTexture;
// }

// function isArray(arr: unknown): arr is Array<unknown>{
//   return Array.isArray(arr);
// } 

// function isColor(c: unknown): c is Color {
//   return (c as Color).isColor !== undefined && (c as Color).isColor;
// }

// function isObject(o: unknown): o is Object3D {
//   return (o as Object3D).isObject3D !== undefined && (o as Object3D).isObject3D;
// }


// function dispose(object: Object3D) {
//   if (!object) {
//       return;
//   }
//   if (isMesh(object) && object.geometry) {
//     (object as Mesh).geometry.dispose();
//   }
//   if (isMesh(object) && object.material) {
//       if (isArray(object.material)) {
//           for (let material of object.material) {
//               if ("map" in material && isTexture(material.map)) {
//                   material.map.dispose();
//               }
//               material.dispose();
//           }
//       } else {
//           if ("map" in object.material && isTexture(object.material.map)) {
//               object.material.map.dispose();
//           }
//           object.material.dispose();
//       }
//   }
// }

// // class Background extends THREE.Object3D {
// //   constructor() {
// //       super();
// //       this.isBackground = true;
// //       this.type = 'Background';

// //       // The controllable properties that can be set either via control or
// //       // set_property().
// //       this.top_color = new dat.color.Color(135, 206, 250);  // lightskyblue
// //       this.bottom_color = new dat.color.Color(25, 25, 112);  // midnightlblue
// //       this.render_environment_map = true;
// //       this.environment_map = null;
// //       this.visible = true;
// //       this.use_ar_background = false;

// //       // The textures associated with the background: either the map, the
// //       // gradient, or a white texture (for when the background isn't visible).
// //       this.textures = {
// //           "env_map": null,  // no default environment.
// //           "round": {
// //               "gradient": env_texture(this.top_color, this.bottom_color, true),
// //               "white": env_texture([255, 255, 255], [255, 255, 255], true)
// //           },
// //           "flat": {
// //               "gradient": env_texture(this.top_color, this.bottom_color, false),
// //               "white": env_texture([255, 255, 255], [255, 255, 255], false)
// //           }
// //       };
// //       // The state values that contributed to the current values of
// //       // scene.background and scene.environment. When the controllable
// //       // properties get twiddled, this state will be compared with that state
// //       // to determine the work necessary to update the background.
// //       this.state = {
// //           "top_color": null,
// //           "bottom_color": null,
// //           "environment_map": null,
// //           "render_map": null,
// //           "visible": true
// //       };
// //   }

// //   // Updates the background's state in the scene based on its requested
// //   // properties, its internal state, and the indication of whether the
// //   // background is visible or not.
// //   //
// //   // Changes to underlying maps (e.g., gradient or environment map) happen
// //   // regardless of the type of camera. However, the textures applied to
// //   // scene.background depend on whether the camera is perspective or not.
// //   update(scene, is_visible, is_perspective) {
// //       // TODO(SeanCurtis-TRI): If the background simply isn't visible, defer
// //       // the work until it is visible, perhaps?
// //       this.state.visible = is_visible;
// //       this.state.render_map = this.render_environment_map;
// //       // If the named environment map has changed, we need to load appropriately.
// //       if (this.environment_map !== this.state.environment_map) {
// //           if (this.environment_map == "" || this.environment_map == null) {
// //               this.environment_map = this.state.environment_map = null;
// //               this.textures.env_map = null;
// //           } else {
// //               this.textures.env_map =
// //                   load_env_texture(this.environment_map, this, scene,
// //                                    is_visible, is_perspective);
// //               if (this.textures.env_map == null) {
// //                   this.state.environment_map = this.environment_map = null;
// //               } else {
// //                   this.state.environment_map = this.environment_map;
// //               }
// //           }
// //       }
// //       // Possibly update the gradient textures if requested top/bottom colors
// //       // are different from the current state. But only if we're *using*
// //       // the gradient. Note: "using" is not the same as obviously drawing it
// //       // as a background. We use the gradient in the following circumstances:
// //       //
// //       //    - The background is visible and
// //       //         - there is no environment map (using gradient as environment) or
// //       //         - the state has been requested to not draw the env map as
// //       //           background, or
// //       //         - the camera is orthographic (can't use env_map as background).
// //       let using_gradient = !is_perspective ||
// //                            !this.render_environment_map ||
// //                            this.textures.env_map == null;
// //       if (is_visible && using_gradient &&
// //           (this.top_color !== this.state.top_color ||
// //            this.bottom_color !== this.state.bottom_color)) {
// //           this.state.top_color = this.top_color;
// //           this.state.bottom_color = this.bottom_color;
// //           let t = [this.state.top_color.r, this.state.top_color.g,
// //                    this.state.top_color.b];
// //           let b = [this.state.bottom_color.r, this.state.bottom_color.g,
// //                    this.state.bottom_color.b];
// //           this.textures.flat.gradient = env_texture(t, b, false);
// //           this.textures.round.gradient = env_texture(t, b, true);
// //       }
// //       // Both background and environment are white if the background isn't
// //       // visible.

// //       // A visible background is either the environment map or the gradient:
// //       // To be the map, the map must be defined, state.render_map is true,
// //       // and the camera is perspective. Otherwise gradient.
// //       // In immersive AR mode, we need the background to be transparent (so
// //       // that the camera comes through). So, we set the background to null,
// //       // but leave the normal semantics for environment so things keep
// //       // rendering the same.
// //       let cam_key = is_perspective ? "round" : "flat";
// //       scene.background =
// //           this.use_ar_background ?
// //               null :
// //               this.state.visible ?
// //                   (this.state.render_map && this.textures.env_map != null && is_perspective ?
// //                       this.textures.env_map :
// //                       this.textures[cam_key].gradient) :
// //                   this.textures[cam_key].white;
// //       // The environment logic is simpler. It only depends on the background
// //       // being visible and an environment map being defined. We'll always
// //       // use an available environment map for illumination, regardless of
// //       // camera projection type.
// //       scene.environment =
// //           this.state.visible ?
// //               (this.textures.env_map != null ?
// //                   this.textures.env_map :
// //                   this.textures.round.gradient) :
// //               this.textures.round.white;
// //   }
// // }

// class SceneNode {
//   object: Object3D;
//   folder: any;
//   children: {[name:string]: SceneNode};
//   controllers: any;
//   onUpdate: any;
//   // isMaterial: boolean;
//   // material: WrappedMaterial;

//   constructor(object: Object3D, folder: any, onUpdate: any) {
//       this.object = object;
//       this.folder = folder;
//       this.children = {};
//       this.controllers = [];
//       this.onUpdate = onUpdate;
//       // this.create_controls();
//       for (let c of this.object.children) {
//           this.add_child(c);
//       }
//   }

//   add_child(object: Object3D): SceneNode {
//       let f = this.folder.addFolder(object.name);
//       let node = new SceneNode(object, f, this.onUpdate);
//       this.children[object.name] = node;
//       return node;
//   }

//   create_child(name: string): SceneNode {
//       let obj: Object3D = new Group();
//       obj.name = name;
//       this.object.add(obj);
//       return this.add_child(obj);
//   }

//   find(path: string[]): undefined | SceneNode {
//       if (path.length == 0) {
//           return this;
//       } else {
//           let name: string = path[0];
//           let child = this.children[name];
//           if (child === undefined) {
//               child = this.create_child(name);
//           }
//           return child.find(path.slice(1));
//       }
//   }

//   // create_controls() {
//   //     for (let c of this.controllers) {
//   //         this.folder.remove(c);
//   //     }
//   //     this.controllers = [];
//   //     if (this.vis_controller !== undefined) {
//   //         this.folder.domElement.removeChild(this.vis_controller.domElement);
//   //     }
//   //     this.vis_controller = new dat.controllers.BooleanController(this.object, "visible");
//   //     this.vis_controller.onChange(() => this.onUpdate());
//   //     this.folder.domElement.prepend(this.vis_controller.domElement);
//   //     this.vis_controller.domElement.style.height = "0";
//   //     this.vis_controller.domElement.style.float = "right";
//   //     this.vis_controller.domElement.classList.add("meshcat-visibility-checkbox");
//   //     this.vis_controller.domElement.children[0].addEventListener("change", (evt) => {
//   //         if (evt.target.checked) {
//   //             this.folder.domElement.classList.remove("meshcat-hidden-scene-element");
//   //         } else {
//   //             this.folder.domElement.classList.add("meshcat-hidden-scene-element");
//   //         }
//   //     });
//   //     if ((this.object as Light).isLight) {
//   //       let light = this.object as Light;
//   //         let intensity_controller = this.folder.add(light, "intensity")
//   //             .min(0).step(0.01).name("intensity (cd)");
//   //         intensity_controller.onChange(() => this.onUpdate());
//   //         this.controllers.push(intensity_controller);
//   //         if (light.castShadow !== undefined){
//   //             let cast_shadow_controller = this.folder.add(light, "castShadow");
//   //             cast_shadow_controller.onChange(() => this.onUpdate());
//   //             this.controllers.push(cast_shadow_controller);

//   //             if (light.shadow !== undefined) {
//   //                 // Light source radius
//   //                 let radius_controller = this.folder.add(light.shadow, "radius").min(0).step(0.05).max(3.0);
//   //                 radius_controller.onChange(() => this.onUpdate());
//   //                 this.controllers.push(radius_controller);
//   //             }
//   //         }
//   //         // Point light falloff distance
//   //         if ((light as PointLight).distance !== undefined){
//   //             let distance_controller = this.folder.add(this.object, "distance").min(0).step(0.1).max(100.0);
//   //             distance_controller.onChange(() => this.onUpdate());
//   //             this.controllers.push(distance_controller);
//   //         }
//   //     }
//   //     if ((this.object as Camera).isCamera) {
//   //         let controller = this.folder.add(this.object, "zoom").min(0).step(0.1);
//   //         controller.onChange(() => {
//   //             // this.object.updateProjectionMatrix();
//   //             this.onUpdate()
//   //         });
//   //         this.controllers.push(controller);
//   //     }
//   //     // if (this.object.isEnvironment) {
//   //     //     let intensity_controller = this.folder.add(this.object, "intensity").min(0).step(0.1).max(100);
//   //     //     intensity_controller.onChange(() => this.onUpdate());
//   //     //     this.controllers.push(intensity_controller);
//   //     // }
//   //     if (this.object.isBackground) {
//   //         // Changing the background gradient is cheap, so we'll change the
//   //         // color in the onChange() callback (instead of the onChangeFinished)
//   //         // callback -- it makes a more interactive experience.
//   //         let top_controller = this.folder.addColor(this.object, "top_color");
//   //         top_controller.onChange(() => this.onUpdate());
//   //         this.controllers.push(top_controller);

//   //         let bottom_controller = this.folder.addColor(this.object, "bottom_color");
//   //         bottom_controller.onChange(() => this.onUpdate());
//   //         this.controllers.push(bottom_controller);

//   //         let map_controller = this.folder.add(this.object, "render_environment_map");
//   //         map_controller.onChange(() => this.onUpdate());
//   //         this.controllers.push(map_controller);
//   //     }
//   // }

//   // To *modulate* opacity, we need to store the baseline value. This should
//   // be called before the "opacity" property of a Material is written to.
//   cache_original_opacity(material: WrappedMaterial) {
//       if (material.base_opacity === undefined) {
//           material.base_opacity = material.opacity;
//       }
//   }

//   // Changing opacity involves coordinating multiple properties.
//   set_opacity(material: WrappedMaterial, opacity: number) {
//       this.cache_original_opacity(material);
//       material.opacity = opacity;
//       material.transparent = opacity < 1;
//       material.depthWrite = true;
//       // Transparency changes may require changes to the compiled shaders.
//       // Setting needsUpdate will trigger that. See
//       // https://github.com/mrdoob/three.js/issues/25307#issuecomment-1398151913
//       material.needsUpdate = true;
//   }

//   // Visits all the materials in the graph rooted at node (including if node
//   // is, itself, a material). For each material, applies the mat_operator
//   // to that material.
//   visit_materials(node: Object3D | Material, mat_operator: any) {
//       if ((node as Material).isMaterial) {
//           mat_operator(node);
//       } else if ((node as Mesh).material) {
//           mat_operator((node as Mesh).material);
//       }
//       for (let child of (node as Group).children) {
//           this.visit_materials(child, mat_operator);
//       }
//   }

//   set_property(property: string, value: number[] | number, target_path: string[]) {
//       if (property === "position") {
//           let position = value as PositionType
//           this.object.position.set(position[0], position[1], position[2]);
//       } else if (property === "quaternion") {
//           let quaternion = value as QuaternionType
//           this.object.quaternion.set(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
//       } else if (property === "scale") {
//           let scale = value as Vector3Tuple;
//           this.object.scale.set(scale[0], scale[1], scale[2]);
//       } else if (property === "color") {
//           const _this = this;
//           const setNodeColor = (mat: WrappedMaterial) => {
//             if ("color" in mat && isColor(mat.color)){
//               mat.color.setRGB((value as number[])[0], (value as number[])[1], (value as number[])[2]);
//             }
//             _this.set_opacity(mat, (value as number[])[3]);
//           };
//           this.visit_materials(this.object, setNodeColor);
//       } else if (property == "opacity") {
//           var _this = this;
//           const setNodeOpacity = (mat: WrappedMaterial) => {
//               _this.set_opacity(mat, (value as number));
//           };
//           this.visit_materials(this.object, setNodeOpacity);
//       } else if (property == "modulated_opacity") {
//           var _this = this;
//           const setModulatedNodeOpacity = (mat: WrappedMaterial) => {
//               // In case set_opacity() has never been called before, we'll
//               // call cache_original_opacity() to be safe.
//               _this.cache_original_opacity(mat);
//               _this.set_opacity(mat, mat.base_opacity * (value as number));
//           };
//           this.visit_materials(this.object, setModulatedNodeOpacity);
//       } else if (property == "top_color" || property == "bottom_color") {
//           // Top/bottom colors are stored as dat.color.Color
//           // this.object[property] = new dat.color.Color(value.map((x) => x * 255));
//       } else {
//           this.set_property_chain(property, value, target_path);
//       }
//       // if (this.object.isBackground) {
//       //     // If we've set values on the Background, we need to fire its on_update()).
//       //     this.onUpdate();
//       // }
//       // this.vis_controller.updateDisplay();
//       // this.controllers.forEach(c => c.updateDisplay());
//   }

//   set_property_chain(property: string, value: number[] | number, target_path: string[]) {
//       // Break the property `obj0.obj1[obj2].foo` into the list
//       // `[obj0, obj1, obj2]` and the property name `foo`.

//       // Array [x] becomes .x.
//       property = property.replace(/\[(\w+)\]/g, '.$1');
//       // Strip a leading dot.
//       property = property.replace(/^\./, '');
//       let objects = property.split(".");
//       const final_property = objects.pop();

//       // Traverse the object sequence.

//       // For loop invariant: `parent` starts as an object (by construction)
//       // the for loop only updates it to another object.
//       let error_detail = null;
//       let parent = this.object;
//       let parent_path = this.folder.name;
//       for (const child of objects) {
//           // Loop invariant: parent is object implies this test is always safe.
//           if (child in parent) {
//               parent_path += "." + child;
//               let temp = parent[child as keyof typeof parent];
//               // if (!!temp && typeof temp === 'object') {
//               if (isObject(temp)) {
//                   parent = temp;
//                   continue;
//               }
//               error_detail = `'${parent_path}' is not an Object and has no properties`;
//           } else {
//               error_detail = `'${parent_path}' has no property '${child}'`;
//           }
//           break;
//       }

//       // Now assign the final property value (if possible). (We know that
//       // parent is an object.)

//       if (error_detail === null && final_property && !(final_property in parent)) {
//           error_detail = `'${parent_path}' has no property '${final_property}'`;
//       }

//       if (error_detail != null) {
//           // Note: full_path may not be an exact reproduction of the path
//           // passed via msgpack.
//           const full_path = "/" + target_path.join('/');
//           const value_str = JSON.stringify(value);
//           console.error(
//               `Error in set_property("${full_path}", "${property}", ${value_str})\n` +
//               `${error_detail}. The value will not be set.`);
//           return;
//       }
//       parent[final_property as keyof typeof parent] = value;
//   }

//   set_transform(matrix: Matrix4Array) {
//       let mat = new THREE.Matrix4();
//       mat.fromArray(matrix);
//       mat.decompose(this.object.position, this.object.quaternion, this.object.scale);
//   }

//   set_object(object: Object3D) {
//       let parent = this.object?.parent;
//       this.dispose_recursive();
//       this.object?.parent?.remove(this.object);
//       this.object = object;
//       parent?.add(object);
//       // this.create_controls();
//   }

//   dispose_recursive() {
//       for (let name of Object.keys(this.children)) {
//           this.children[name].dispose_recursive();
//       }
//       dispose(this.object);
//   }

//   delete(path) {
//       if (path.length == 0) {
//           console.error("Can't delete an empty path");
//       } else {
//           let parent = this.find(path.slice(0, path.length - 1));
//           let name = path[path.length - 1];
//           let child = parent.children[name];
//           if (child !== undefined) {
//               child.dispose_recursive();
//               parent.object.remove(child.object);
//               remove_folders(child.folder);
//               parent.folder.removeFolder(child.folder);
//               delete parent.children[name];
//           }
//       }
//   }
// }