import React, { Component } from "react";
import { ConfigConsumer } from "antd/lib/config-provider";
import PropTypes from "prop-types";
import { connect } from "react-redux";

import isEqual from "lodash/isEqual";
import Icon from "antd/lib/icon";
import "./zUp";
import { FBXLoader } from "./FBXLoader";
import { OrbitControls } from "./controls/OrbitControls";
import { withSize } from "react-sizeme";

import { ROBOT_PREFIX } from "../../../utils";

import { getMemoizePartPQ, GetPartsPath } from "../../../state/selectors";
import {
  loadConfigXml,
  // deleteResultByChannel
} from "../../../state/actions";

const THREE = window.THREE;

class ViewThree extends Component {
  constructor(props) {
    super(props);
    this.initialized = false;
    this.parts = {};
    this.state = {
      loadings: {
        0: true,
        1: true,
        2: true,
        3: true,
        4: true,
        5: true,
        6: true,
      },
      errors: [],
      partLength: 7,
    };
  }

  componentDidMount() {
    // this.props.deleteResultByChannel('xml')
    this.props.loadConfigXml();
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
    if (this.initialized) {
      return;
    }
    if (!container || !paths) {
      return;
    }
    const self = this;
    this.initialized = true;
    const {
      size: { width, height },
    } = this.props;
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    this.camera.position.set(-1, -1, 0.1);
    this.camera.lookAt(new THREE.Vector3(0, 0, 0.5));
    this.scene = new THREE.Scene();

    this.scene.background = new THREE.Color(0x000000);
    this.scene.fog = new THREE.Fog(0x000000, 2, 10);

    // let axesHelper = new THREE.AxesHelper(1);
    // axesHelper.position.set(-0.5, -0.5, 0)
    // this.scene.add(axesHelper);

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

    this.loader = new FBXLoader();
    // .log("loading",paths)
    const newPath = [];
    for (let k = 0; k < paths.length; k++) {
      // console.log("loading1",paths[k].indexOf(".data"))
      if (paths[k].indexOf(".data") !== -1) {
        // console.log("paths",paths[k])
        newPath.push(paths[k]);
      }
    }
    this.setState({
      partLength: newPath.length,
    });
    // console.log("newpaths",newPath)
    newPath.forEach((path, i) => {
      // console.log("newpaths",newPath)
      if (ROBOT_PREFIX) {
        path = ROBOT_PREFIX + path;
      }
      console.log("path", i, path);
      self.loader.load(
        path,
        function onLoad(object) {
          const loadings = { ...self.state.loadings };
          loadings[i] = false;
          self.setState({ loadings });
          self.scene.add(object);
          self.parts[i] = object;
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
        function onProgress(progress) {},
        function onError(error) {
          const errors = [...self.state.errors];
          errors.push(error);
          self.setState({ errors });
          console.error(error);
        }
      );
    });

    function animate() {
      // 执行循环，渲染新的图形，使物体动起来
      self.animationFrame = requestAnimationFrame(animate);

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
    this.controls.target.set(0, 0, 0.5);
    this.controls.update();
    animate();
  }

  componentWillUnmount() {
    cancelAnimationFrame(this.animationFrame);
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
    }

    const { partPQ: newPartPQ, partsPath } = nextProps;
    const { partPQ: oldPartPQ } = this.props;
    if (newPartPQ !== oldPartPQ && !!newPartPQ) {
      // console.log("partpq",oldPartPQ,newPartPQ)
      for (let i = 0; i < newPartPQ.length / 7; i++) {
        if (this.parts[i]) {
          // 重新定位
          this.parts[i].position.set(
            newPartPQ[i * 7],
            newPartPQ[i * 7 + 1],
            newPartPQ[i * 7 + 2]
          );
          this.parts[i].quaternion.set(
            newPartPQ[i * 7 + 3],
            newPartPQ[i * 7 + 4],
            newPartPQ[i * 7 + 5],
            newPartPQ[i * 7 + 6]
          );
        }
      }
    }
    if (!!partsPath && !!this.container && !this.initialized) {
      this.init(this.container, partsPath);
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
            if (!!this.container && !!this.props.partsPath) {
              this.init(this.container, this.props.partsPath);
            }
          }}
        >
          <Icon
            type="redo"
            onClick={() => {
              if (this.camera && this.controls) {
                this.camera.position.set(1, 1, 0.1);
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

ViewThree.propTypes = {
  cell: PropTypes.object.isRequired,
  xml: PropTypes.string,
};

function mapStateToProps(state) {
  return {
    partsPath: GetPartsPath(state),
    partPQ: getMemoizePartPQ(state),
  };
}

const ConnectedViewThree = connect(mapStateToProps, {
  loadConfigXml,
  // deleteResultByChannel,
})(withSize({ monitorHeight: true, refreshRate: 30 })(ViewThree));

ConnectedViewThree.NAME = "三维视图";
export default ConnectedViewThree;
