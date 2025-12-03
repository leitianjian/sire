import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";
import { connect } from "react-redux";

import isEqual from "lodash/isEqual";
import {
  RedoOutlined
} from "@ant-design/icons";
import "./zUp";
import { FBXLoader } from "./FBXLoader";
import { OrbitControls } from "./controls/OrbitControls";
import { withSize } from "react-sizeme";

import { ROBOT_PREFIX } from "../../../utils";

import { getGeometryPm, getGeometryPool, getMemoizePartPQ, getPartPQInit, GetPartsPath, getViewerConfig, getWebsocketConnected } from "../../../state/selectors";
import {
  loadConfigXml, sendCmd,
  display3dInitRequest,
  updatePartPQRequest,
  sendCmdSilence,
  // deleteResultByChannel
} from "../../../state/actions";

const THREE = window.THREE;
// TODO(ltj): 优化代码，场景和灯光
// TODO(ltj): 这里面的状态更新都在这里面处理，不要放到redux里面，否则频率上不去（已解决）
class ViewThree2 extends Component {
  constructor(props) {
    super(props);
    this.initialized = false;
    this.geometries = [];
    this.state = {
      loadings: {
        0: false,
        1: false,
      },
      errors: [],
      partLength: 2,
      fps: 0,
      lups: 0,
    };
    this.updates = 0, 
    this.prevUpdateLocationTime = 0;
  }

  componentDidMount() {
    console.log("send display3d init");
    // this.props.deleteResultByChannel('xml')
    this.props.display3dInitRequest();
    this.props.display3dInitRequest();
    this.props.display3dInitRequest();
    // this.props.loadConfigXml();
    this.intervalTimer = setInterval(() => {
      this.props.sendCmdSilence("get --part_pq", (msg) => {
        if (msg && msg.jsData && msg.jsData.return_code === 0) {
          this.updateLocation(msg.jsData.part_pq);
        }
      })
      // this.props.updatePartPQRequest();
    }, 1000.0 / this.props.viewer3dConfig.frame_rate);
  }

  getEnvScene() {
    const envScene = new THREE.Scene();

    const geometry = new THREE.BoxBufferGeometry();
    geometry.deleteAttribute("uv");
    const roomMaterial = new THREE.MeshStandardMaterial({
      metalness: 0,
      side: THREE.BackSide,
    });
    const room = new THREE.Mesh(geometry, roomMaterial);
    room.scale.setScalar(10);
    envScene.add(room);

    // let mainLight = new THREE.PointLight(0xffffff, 50, 0, 2);
    const mainLight = new THREE.PointLight(0xffffff, 5, 0, 0.2);
    envScene.add(mainLight);

    const lightMaterial = new THREE.MeshLambertMaterial({
      color: 0x000000,
      emissive: 0xffffff,
      emissiveIntensity: 10,
    });

    const light1 = new THREE.Mesh(geometry, lightMaterial);
    light1.position.set(-0.5, 0.2, 0);
    light1.scale.set(0.1, 1, 1);
    envScene.add(light1);

    const light2 = new THREE.Mesh(geometry, lightMaterial);
    light2.position.set(0, 0.5, 0);
    light2.scale.set(1, 0.1, 1);
    envScene.add(light2);

    const light3 = new THREE.Mesh(geometry, lightMaterial);
    light3.position.set(0.2, 0.1, 0.5);
    light3.scale.set(1.5, 2, 0.1);
    envScene.add(light3);

    return envScene;
  }

  init(container, paths) {
    // console.log("called init1", this.props.geometry_pool);
    if (this.initialized || !container) {
      return;
    }
    const self = this;
    const {
      size: { width, height },
    } = this.props;
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    this.camera.position.set(3, 3, 10);
    this.camera.lookAt(new THREE.Vector3(0, 0, -10));
    this.scene = new THREE.Scene();

    this.scene.background = new THREE.Color(0x000000);
    // this.scene.fog = new THREE.Fog(0x000000, 2, 10);

    // let axesHelper = new THREE.AxesHelper(1);
    // axesHelper.position.set(-0.5, -0.5, 0)
    // this.scene.add(axesHelper);
    this.light = new THREE.AmbientLight( 0x404040, 5); // soft white light
    this.scene.add( this.light );

    this.light = new THREE.HemisphereLight(0xffffff, 0x444444);
    this.light.position.set(0, 0, 200);
    this.scene.add(this.light);

    this.light = new THREE.DirectionalLight(0xffffff, 1);
    this.light.position.set(0, 10, 10);
    this.scene.add(this.light);

    const mesh = new THREE.Mesh(
      new THREE.PlaneBufferGeometry(2000, 2000),
      new THREE.MeshPhongMaterial({
        // color: 0x6e6a62,
        color: 0x111111,
        depthWrite: false,
      })
    );
    // mesh.rotation.x = -Math.PI / 2;
    mesh.receiveShadow = true;
    this.scene.add(mesh);

    // let grid = new THREE.GridHelper(2000, 10, 0x000000, 0x000000);
    const grid = new THREE.GridHelper(200, 200, 0xffffff, 0xffffff);
    grid.material.opacity = 0.1;
    grid.material.depthWrite = false;
    grid.material.transparent = true;

    grid.rotateX(Math.PI / 2);
    this.scene.add(grid);

    this.loaderFBX = new FBXLoader();
    // .log("loading",paths)
    const newPath = [];
    console.log("called init2", this.props.geometry_pool);
    this.props.geometry_pool.forEach((geometry, i) => {
      if (geometry.shape_type === "sphere") {
        const sphere_geometry = new THREE.SphereGeometry(geometry.radius);
        if (i % 2 === 1) {
          let material = new THREE.MeshStandardMaterial({
            color: "rgb(6,96,255)",
            roughness: 0.1,
            metalness: 0.1,
          });
          let obj = new THREE.Mesh(sphere_geometry, material);
          obj["relative_to_part"] = geometry.relative_to_part;
          obj["geometry_id"] = geometry.geometry_id;
          obj["part_id"] = geometry.part_id;
          this.geometries[i] = obj;
          this.scene.add(obj);
        } else {
          let material = new THREE.MeshStandardMaterial({
            color: "rgb(212,212,212)",
            roughness: 0.1,
            metalness: 0.1,
          });
          let obj = new THREE.Mesh(sphere_geometry, material);
          obj["relative_to_part"] = geometry.relative_to_part;
          obj["geometry_id"] = geometry.geometry_id;
          obj["part_id"] = geometry.part_id;
          this.geometries[i] = obj;
          this.scene.add(obj);
        }
      } else if (geometry.shape_type === "box") {
        const box_geometry = new THREE.BoxGeometry(geometry.length, geometry.width, geometry.height);
        if (i % 2 === 1) {
          let material = new THREE.MeshStandardMaterial({
            color: "rgb(6,96,255)",
            roughness: 0.1,
            metalness: 0.1,
          });
          let obj = new THREE.Mesh(box_geometry, material);
          obj["relative_to_part"] = geometry.relative_to_part;
          obj["geometry_id"] = geometry.geometry_id;
          obj["part_id"] = geometry.part_id;
          this.geometries[i] = obj;
          this.scene.add(obj);
        } else {
          let material = new THREE.MeshStandardMaterial({
            color: "rgb(212,212,212)",
            roughness: 0.1,
            metalness: 0.1,
          });
          let obj = new THREE.Mesh(box_geometry, material);
          obj["relative_to_part"] = geometry.relative_to_part;
          obj["geometry_id"] = geometry.geometry_id;
          obj["part_id"] = geometry.part_id;
          this.geometries[i] = obj;
          this.scene.add(obj);
        }
      } else if (geometry.shape_type === "mesh") {
        const path = geometry.resource_path;
        if (ROBOT_PREFIX) {
          path = ROBOT_PREFIX + path;
        }
        console.log("path", i, path);
        self.loaderFBX.load(
          path,
          function onLoad(object) {
            object["part_id"] = geometry.part_id;
            object["relative_to_part"] = geometry.relative_to_part;
            object["geometry_id"] = geometry.geometry_id;
            const loadings = { ...self.state.loadings };
            loadings[i] = false;
            self.setState({ loadings });
            self.scene.add(object);
            self.geometries[i] = object;
            if (i % 2 === 1) {
              object.traverse((o) => {
                o.material = new THREE.MeshStandardMaterial({
                  color: "rgb(6,96,255)",
                  roughness: 0.1,
                  metalness: 0.1,
                });
              });
            } else {
              object.traverse((o) => {
                o.material = new THREE.MeshStandardMaterial({
                  color: "rgb(212,212,212)",
                  roughness: 0.1,
                  metalness: 0.1,
                });
              });
            }
          },
          function onProgress(progress) { },
          function onError(error) {
            const errors = [...self.state.errors];
            errors.push(error);
            self.setState({ errors });
            console.error(error);
          }
        );
      }
      if (i == 0) {
        if (self.geometries[i] && 'material' in self.geometries[i]) {
          self.geometries[i].material = new THREE.MeshStandardMaterial({
            color: "rgb(117, 83, 56)",
            roughness: 1.0,
            metalness: 0.1,
          });
        }
      }
    });
    this.props.sendCmdSilence("get --part_pq", (msg) => {
      if (msg && msg.jsData && msg.jsData.ret_code === 0) {
        this.updateLocation(msg.jsData.part_pq);
      }
    });
    this.prevUpdateLocationTime = performance.now();
    // for (let k = 0; k < paths.length; k++) {
    //   // console.log("loading1",paths[k].indexOf(".data"))
    //   if (paths[k].indexOf(".data") !== -1) {
    //     // console.log("paths",paths[k])
    //     newPath.push(paths[k]);
    //   }
    // }
    // this.setState({
    //   partLength: newPath.length,
    // });
    // console.log("newpaths",newPath)
    // newPath.forEach((path, i) => {
    //   // console.log("newpaths",newPath)
    //   if (ROBOT_PREFIX) {
    //     path = ROBOT_PREFIX + path;
    //   }
    //   console.log("path", i, path);
    //   self.loaderFBX.load(
    //     path,
    //     function onLoad(object) {
    //       const loadings = { ...self.state.loadings };
    //       loadings[i] = false;
    //       self.setState({ loadings });
    //       self.scene.add(object);
    //       self.geometries[i] = object;
    //       if (i % 2 === 1) {
    //         object.traverse((o) => {
    //           o.material = new THREE.MeshStandardMaterial({
    //             color: "rgb(6,96,255)",
    //             roughness: 0.1,
    //             metalness: 0.1,
    //           });
    //         });
    //       } else {
    //         object.traverse((o) => {
    //           o.material = new THREE.MeshStandardMaterial({
    //             color: "rgb(212,212,212)",
    //             roughness: 0.1,
    //             metalness: 0.1,
    //           });
    //         });
    //       }
    //     },
    //     function onProgress(progress) { },
    //     function onError(error) {
    //       const errors = [...self.state.errors];
    //       errors.push(error);
    //       self.setState({ errors });
    //       console.error(error);
    //     }
    //   );
    // });
    this.initialized = true;
    let frames = 0, prevTime = performance.now();
    function animate() {
      // 执行循环，渲染新的图形，使物体动起来
      self.animationFrame = requestAnimationFrame(animate);
      
      frames ++;
      const time = performance.now();
      
      if ( time >= prevTime + 500 ) {
        self.setState({fps: Math.round( ( frames * 1000 ) / ( time - prevTime ) )});
        frames = 0;
        prevTime = time;
      }
      // var delta = clock.getDelta();

      // if (mixer) mixer.update(delta);

      self.renderer.render(self.scene, self.camera);

      // stats.update();
    }

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(width, height);
    // this.renderer.shadowMap.enabled = true;
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, 0);
    this.controls.update();
    animate();
  }

  componentWillUnmount() {
    cancelAnimationFrame(this.animationFrame);
  }

  updateLocation = (newPartPQ) => {
    const { part_pq_init, geometry_pm } = this.props;
    for (let i = 0; i < this.geometries.length; ++i) {
      let geometry = this.geometries[i];
      let part_id = geometry.part_id;
      if (geometry) {
        // 如果geometry相对于杆件
        if (geometry.relative_to_part) {
          let part_pm = new THREE.Matrix4().compose(new THREE.Vector3(newPartPQ[part_id][0],
            newPartPQ[part_id][1],
            newPartPQ[part_id][2]), new THREE.Quaternion(newPartPQ[part_id][3],
              newPartPQ[part_id][4],
              newPartPQ[part_id][5],
              newPartPQ[part_id][6]), new THREE.Vector3(1, 1, 1));
          let geo_pm = new THREE.Matrix4().set(...geometry_pm[part_id]);
            part_pm.multiply(geo_pm);
            geometry.matrix.copy(part_pm);
            geometry.matrixAutoUpdate = false;
          // 如果geometry相对于地面
        } else {
          let init_q = new THREE.Quaternion(part_pq_init[part_id][3], part_pq_init[part_id][4], part_pq_init[part_id][5], part_pq_init[part_id][6]);
          let init_p = new THREE.Vector3(part_pq_init[part_id][0], part_pq_init[part_id][1], part_pq_init[part_id][2]);
          let part_p = new THREE.Vector3(newPartPQ[part_id][0], newPartPQ[part_id][1], newPartPQ[part_id][2]);
          let part_q = new THREE.Quaternion(newPartPQ[part_id][3], newPartPQ[part_id][4], newPartPQ[part_id][5], newPartPQ[part_id][6]);
          part_q.multiply(init_q);
          init_p.applyQuaternion(part_q).add(part_p);
          geometry.position.copy(init_p);
          geometry.quaternion.copy(part_q);
        }
      }
    }
    this.updates ++;
    const time = performance.now();
    
    if ( time >= this.prevUpdateLocationTime + 500 ) {
      this.setState({lups: Math.round( ( this.updates * 1000 ) / ( time - this.prevUpdateLocationTime ) )});
      // console.log(Math.round( ( this.updates * 1000 ) / ( time - this.prevUpdateLocationTime ) ));
      this.updates = 0;
      this.prevUpdateLocationTime = time;
    }
  }

  shouldComponentUpdate(nextProps, nextState, nextContext) {
    const { loadings, errors } = this.state;
    const { loadings: nextLoadings, errors: nextErrors } = nextState;
    if (!isEqual(loadings, nextLoadings) || !isEqual(errors, nextErrors)) {
      return true;
    }

    if (
      (nextProps.size.width !== this.props.size.width ||
        nextProps.size.height !== this.props.size.height) &&
      this.camera &&
      this.renderer
    ) {
      this.camera.aspect = nextProps.size.width / nextProps.size.height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(nextProps.size.width, nextProps.size.height);
      return true;
    }
    const { partPQ: newPartPQ, partsPath } = nextProps;
    // const { partPQ: oldPartPQ } = this.props;
    // //console.log(new THREE.Quaternion(-0.843, 0.002, 0.510, -0.170).multiply(new THREE.Quaternion(0.451, 0.003, 0.847, -0.282)), "hello")
    // if (newPartPQ !== oldPartPQ && !!newPartPQ) {
    //   // console.log("partpq",oldPartPQ,newPartPQ)
    //   this.updateLocation(newPartPQ);
    //   return false;
    // }
    if (!!this.container && !this.initialized && this.props.geometry_pool.length !== 0) {
      this.init(this.container, partsPath);
      return true;
    }
    if (nextProps.geometry_pool.length !== this.props.geometry_pool.length) {
      return true;
    }
    if (nextState.fps !== this.state.fps) {
      return true;
    }
    return false;
  }

  renderThree = ({ getPrefixCls }) => {
    const { prefixCls: customizePrefixCls } = this.props;
    let loading = false;
    for (const key in this.state.partLength) {
      if (this.state.loadings[key]) {
        loading = true;
      }
    }
    if (Object.keys(this.state.loadings).length === 0) {
      loading = true;
    }
    this.state.loadings &&
      Object.keys(this.state.loadings).filter((key) => {
        if (this.state.loadings[key]) {
          loading = true;
        }
      });

    // console.log("loading",this.state.loadings)
    const prefixCls = getPrefixCls("view3d", customizePrefixCls);
    return (
      <div className={`${prefixCls}`}>
        <div
          className={`${prefixCls}-three`}
          style={{
            opacity: loading || this.state.errors.length !== 0 ? 0 : 1,
          }}
          ref={(ref) => {
            this.container = ref;
            if (!!this.container && this.props.geometry_pool.length !== 0) {
              this.init(this.container, this.props.partsPath);
            }
          }}
        >
          <RedoOutlined
            onClick={() => {
              if (this.camera && this.controls) {
                this.camera.position.set(10, 10, 5);
                // this.camera.position.set(0, 0, 0);
                this.camera.lookAt(new THREE.Vector3(0, 0, 0.5));
                this.camera.aspect =
                  this.props.size.width / this.props.size.height;
                this.camera.updateProjectionMatrix();
                this.controls.target.set(0, 0, 0.5);
                this.controls.update();
              }
            }}
          />
          <div className={`${prefixCls}-three-info`}>
            <div className={`${prefixCls}-three-info-fps`}>fps: {this.state.fps}</div>
            <div className={`${prefixCls}-three-info-lups`}>LUPS: {this.state.lups}</div>
          </div>
        </div>
        {this.state.errors.length !== 0 && (
          <div className={`${prefixCls}-errors`}>
            <p>ERROR</p>
            {this.state.errors.map((e, i) => (
              <p key={i}>{e.message}</p>
            ))}
          </div>
        )}
        {loading && this.state.errors.length === 0 && (
          <div className={`${prefixCls}-loading`}>载入中...</div>
        )}
      </div>
    );
  };

  render() {
    return <ConfigConsumer>{this.renderThree}</ConfigConsumer>;
  }
}

ViewThree2.propTypes = {
  cell: PropTypes.object.isRequired,
  xml: PropTypes.string,
};

const DefaultConfig = {
  frame_rate: 10,
}

function mapStateToProps(state, props) {
  return {
    viewer3dConfig: getViewerConfig(state, DefaultConfig),
    partsPath: GetPartsPath(state),
    partPQ: getMemoizePartPQ(state),
    geometry_pool: getGeometryPool(state),
    geometry_pm: getGeometryPm(state),
    part_pq_init: getPartPQInit(state),
    websocketConnected: getWebsocketConnected(state),
  };
}

function mapDispatchToProps(dispatch) {
  return {
    sendCmd: (...args) => dispatch(sendCmd(...args)),
    sendCmdSilence: (...args) => dispatch(sendCmdSilence(...args)),
    loadConfigXml: (...args) => dispatch(loadConfigXml(...args)),
    display3dInitRequest: (...args) => dispatch(display3dInitRequest(...args)),
    updatePartPQRequest: (...args) => dispatch(updatePartPQRequest(...args)),
  };
}

const ConnectedViewThree2 = connect(mapStateToProps, mapDispatchToProps)(withSize({ monitorHeight: true, refreshRate: 30 })(ViewThree2));

ConnectedViewThree2.NAME = "三维视图2";
export default ConnectedViewThree2;
