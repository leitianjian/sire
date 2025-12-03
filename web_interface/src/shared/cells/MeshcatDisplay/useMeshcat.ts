import { Viewer, msgpack } from "./meshcat";
import React from "react";

export function testMeshcatCommand(ref: any): Viewer {
  var scene_json = {
    metadata: { version: 4.5, type: "Object", generator: "Object3D.toJSON" },
    geometries: [
      {
        uuid: "F00197C6-9B9F-475C-88F4-F18C046A0D7F",
        type: "BufferGeometry",
        data: {
          attributes: {
            position: {
              itemSize: 3,
              type: "Float32Array",
              array: [
                -10, 0, -10, 10, 0, -10, -10, 0, -10, -10, 0, 10, -10, 0, -9.5,
                10, 0, -9.5, -9.5, 0, -10, -9.5, 0, 10, -10, 0, -9, 10, 0, -9,
                -9, 0, -10, -9, 0, 10, -10, 0, -8.5, 10, 0, -8.5, -8.5, 0, -10,
                -8.5, 0, 10, -10, 0, -8, 10, 0, -8, -8, 0, -10, -8, 0, 10, -10,
                0, -7.5, 10, 0, -7.5, -7.5, 0, -10, -7.5, 0, 10, -10, 0, -7, 10,
                0, -7, -7, 0, -10, -7, 0, 10, -10, 0, -6.5, 10, 0, -6.5, -6.5,
                0, -10, -6.5, 0, 10, -10, 0, -6, 10, 0, -6, -6, 0, -10, -6, 0,
                10, -10, 0, -5.5, 10, 0, -5.5, -5.5, 0, -10, -5.5, 0, 10, -10,
                0, -5, 10, 0, -5, -5, 0, -10, -5, 0, 10, -10, 0, -4.5, 10, 0,
                -4.5, -4.5, 0, -10, -4.5, 0, 10, -10, 0, -4, 10, 0, -4, -4, 0,
                -10, -4, 0, 10, -10, 0, -3.5, 10, 0, -3.5, -3.5, 0, -10, -3.5,
                0, 10, -10, 0, -3, 10, 0, -3, -3, 0, -10, -3, 0, 10, -10, 0,
                -2.5, 10, 0, -2.5, -2.5, 0, -10, -2.5, 0, 10, -10, 0, -2, 10, 0,
                -2, -2, 0, -10, -2, 0, 10, -10, 0, -1.5, 10, 0, -1.5, -1.5, 0,
                -10, -1.5, 0, 10, -10, 0, -1, 10, 0, -1, -1, 0, -10, -1, 0, 10,
                -10, 0, -0.5, 10, 0, -0.5, -0.5, 0, -10, -0.5, 0, 10, -10, 0, 0,
                10, 0, 0, 0, 0, -10, 0, 0, 10, -10, 0, 0.5, 10, 0, 0.5, 0.5, 0,
                -10, 0.5, 0, 10, -10, 0, 1, 10, 0, 1, 1, 0, -10, 1, 0, 10, -10,
                0, 1.5, 10, 0, 1.5, 1.5, 0, -10, 1.5, 0, 10, -10, 0, 2, 10, 0,
                2, 2, 0, -10, 2, 0, 10, -10, 0, 2.5, 10, 0, 2.5, 2.5, 0, -10,
                2.5, 0, 10, -10, 0, 3, 10, 0, 3, 3, 0, -10, 3, 0, 10, -10, 0,
                3.5, 10, 0, 3.5, 3.5, 0, -10, 3.5, 0, 10, -10, 0, 4, 10, 0, 4,
                4, 0, -10, 4, 0, 10, -10, 0, 4.5, 10, 0, 4.5, 4.5, 0, -10, 4.5,
                0, 10, -10, 0, 5, 10, 0, 5, 5, 0, -10, 5, 0, 10, -10, 0, 5.5,
                10, 0, 5.5, 5.5, 0, -10, 5.5, 0, 10, -10, 0, 6, 10, 0, 6, 6, 0,
                -10, 6, 0, 10, -10, 0, 6.5, 10, 0, 6.5, 6.5, 0, -10, 6.5, 0, 10,
                -10, 0, 7, 10, 0, 7, 7, 0, -10, 7, 0, 10, -10, 0, 7.5, 10, 0,
                7.5, 7.5, 0, -10, 7.5, 0, 10, -10, 0, 8, 10, 0, 8, 8, 0, -10, 8,
                0, 10, -10, 0, 8.5, 10, 0, 8.5, 8.5, 0, -10, 8.5, 0, 10, -10, 0,
                9, 10, 0, 9, 9, 0, -10, 9, 0, 10, -10, 0, 9.5, 10, 0, 9.5, 9.5,
                0, -10, 9.5, 0, 10, -10, 0, 10, 10, 0, 10, 10, 0, -10, 10, 0,
                10,
              ],
              normalized: false,
            },
            color: {
              itemSize: 3,
              type: "Float32Array",
              array: [
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.2666666805744171, 0.2666666805744171, 0.2666666805744171,
                0.2666666805744171, 0.2666666805744171, 0.2666666805744171,
                0.2666666805744171, 0.2666666805744171, 0.2666666805744171,
                0.2666666805744171, 0.2666666805744171, 0.2666666805744171,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
                0.5333333611488342, 0.5333333611488342, 0.5333333611488342,
              ],
              normalized: false,
            },
          },
          boundingSphere: { center: [0, 0, 0], radius: 14.142135623730951 },
        },
      },
      {
        uuid: "3C839FA7-7D54-41F7-B30A-0AB7875E78F9",
        type: "BufferGeometry",
        data: {
          attributes: {
            position: {
              itemSize: 3,
              type: "Float32Array",
              array: [
                0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.5,
              ],
              normalized: false,
            },
            color: {
              itemSize: 3,
              type: "Float32Array",
              array: [
                1, 0, 0, 1, 0.6000000238418579, 0, 0, 1, 0, 0.6000000238418579,
                1, 0, 0, 0, 1, 0, 0.6000000238418579, 1,
              ],
              normalized: false,
            },
          },
          boundingSphere: {
            center: [0.25, 0.25, 0.25],
            radius: 0.4330127018922193,
          },
        },
      },
      {
        uuid: "688cba36-120a-11e8-1a9d-0367a991b8a9",
        type: "BoxGeometry",
        width: 0.1,
        height: 0.2,
        depth: 0.3,
      },
    ],
    materials: [
      {
        uuid: "C6134141-28E6-4C81-9952-FE6282BD91CC",
        type: "LineBasicMaterial",
        color: 16777215,
        vertexColors: 2,
        depthFunc: 3,
        depthTest: true,
        depthWrite: true,
      },
      {
        uuid: "37322D07-0AE9-459D-8ED1-C531059E8F0A",
        type: "LineBasicMaterial",
        color: 16777215,
        vertexColors: 2,
        depthFunc: 3,
        depthTest: true,
        depthWrite: true,
      },
      {
        uuid: "688cba7c-120a-11e8-1ccd-dbbc48084379",
        type: "MeshLambertMaterial",
        color: 16777215,
        emissive: 0,
        depthFunc: 3,
        depthTest: true,
        depthWrite: true,
      },
    ],
    object: {
      uuid: "AE3F39DC-1E7E-4EE6-A60F-B4C36F7EE7CA",
      type: "Scene",
      name: "Scene",
      matrix: [
        1, 0, 0, 0, 0, 2.220446049250313e-16, -1, 0, 0, 1,
        2.220446049250313e-16, 0, 0, 0, 0, 1,
      ],
      children: [
        {
          uuid: "621A4A0B-F05D-4F87-B275-D84BDEC4555F",
          type: "Group",
          name: "Lights",
          matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
          children: [
            {
              uuid: "9001C5AF-00ED-42A0-B99F-F7AB6A8C158F",
              type: "DirectionalLight",
              name: "DirectionalLight",
              matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 5, 10, 1],
              color: 16777215,
              intensity: 0.5,
              shadow: {
                camera: {
                  uuid: "838A7B26-779F-4790-96E5-D5A8491196C7",
                  type: "OrthographicCamera",
                  zoom: 1,
                  left: -5,
                  right: 5,
                  top: 5,
                  bottom: -5,
                  near: 0.5,
                  far: 500,
                },
              },
            },
            {
              uuid: "5285C3E1-D8E4-44B6-A1B4-79FDD53ABCD1",
              type: "AmbientLight",
              name: "AmbientLight",
              matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
              color: 16777215,
              intensity: 0.3,
            },
          ],
        },
        {
          uuid: "97F5751B-CD47-4702-9332-ECF2918AFAF9",
          type: "LineSegments",
          name: "Grid",
          matrix: [
            1, 0, 0, 0, 0, 2.220446049250313e-16, 1, 0, 0, -1,
            2.220446049250313e-16, 0, 0, 0, 0, 1,
          ],
          geometry: "F00197C6-9B9F-475C-88F4-F18C046A0D7F",
          material: "C6134141-28E6-4C81-9952-FE6282BD91CC",
        },
        {
          uuid: "8B6F33B2-021E-40AD-B47D-BF3259373786",
          type: "LineSegments",
          name: "Axes",
          matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
          geometry: "3C839FA7-7D54-41F7-B30A-0AB7875E78F9",
          material: "37322D07-0AE9-459D-8ED1-C531059E8F0A",
        },
        {
          uuid: "0876A211-CF4E-4DF0-93CB-0FE1B03CAB47",
          type: "Object3D",
          name: "meshcat",
          matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
          children: [
            {
              uuid: "0C4D84EE-5826-436C-8329-B4F68876FC70",
              type: "Object3D",
              name: "boxes",
              matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1],
              children: [
                {
                  uuid: "688cb6b2-120a-11e8-32d3-49ba11c3bb93",
                  type: "Mesh",
                  name: "<object>",
                  matrix: [
                    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.05, 0.1, 0.15, 1,
                  ],
                  geometry: "688cba36-120a-11e8-1a9d-0367a991b8a9",
                  material: "688cba7c-120a-11e8-1ccd-dbbc48084379",
                },
              ],
            },
          ],
        },
      ],
    },
  };

  var viewer = new Viewer(ref);
  // viewer.connect();
  viewer.load_scene_from_json(scene_json);

  let cmd = {
    type: "set_animation",
    animations: [
      {
        path: "/Cameras/default",
        clip: {
          fps: 30,
          name: "default",
          tracks: [
            {
              name: ".position",
              type: "vector3",
              keys: [
                {
                  time: 0,
                  value: [0, 1, 0.3],
                },
                {
                  time: 80,
                  value: [0, 1, 2],
                },
              ],
            },
          ],
        },
      },
      {
        path: "/Cameras/default/rotated/<object>",
        clip: {
          fps: 30,
          name: "default",
          tracks: [
            {
              name: ".zoom",
              type: "number",
              keys: [
                {
                  time: 0,
                  value: 1,
                },
                {
                  time: 80,
                  value: 0.5,
                },
              ],
            },
          ],
        },
      },
      {
        path: "/meshcat/boxes",
        clip: {
          fps: 30,
          name: "default",
          tracks: [
            {
              name: ".position",
              type: "vector3",
              keys: [
                {
                  time: 0,
                  value: [0, 1, 0],
                },
                {
                  time: 80,
                  value: [0, -1, 0],
                },
              ],
            },
          ],
        },
      },
    ],
    options: {
      play: true,
      repetitions: 1,
    },
  };

  viewer.handle_command(cmd);
  return viewer;
}



export function useMeshcat(ref: any): Viewer {
    // TODO(#16486): add tooltips to Stats to describe chart contents
    // var stats = new Stats();
    // var realtimeRatePanel = stats.addPanel(
    //         new Stats.Panel('rtr%', '#ff8', '#221')
    // );
    // document.body.appendChild(stats.dom);
    // stats.dom.id = "stats-plot";
    // We want to show the realtime rate panel by default
    // it is the last element in the stats.dom.children list
    // stats.showPanel(stats.dom.children.length - 1)
    let latestRealtimeRate = 0;
    let viewer = new Viewer(document.getElementById("meshcat-pane"));
    viewer.animate = function() {
      // viewer.animator.update();
      if (viewer.needs_render) {
        viewer.render();
      }
    }
  
    const clamp = (num: number, min: number, max: number) => Math.min(Math.max(num, min), max);
  
    // Gamepad support - first check if the gamepad feature is available.
    const gamepads_supported = !!navigator.getGamepads;
    if (!gamepads_supported) {
      if (!window.isSecureContext) {
        console.warn("Gamepads are not supported outside of a secure context. "
        + "See https://developer.mozilla.org/en-US/docs/Web/Security/Secure_Contexts"
        + " for details. Some browsers support localhost and allowlists.");
      } else {
        console.warn("Gamepads are not supported in this browser session.");
      }
    }
    // See https://beej.us/blog/data/javascript-gamepad/ for a tutorial.
    var last_gamepad = {};
    function handle_gamepads() {
      let gamepads = navigator.getGamepads();
      let gamepad = {};
      for (let i = 0; i < gamepads.length; i++) {
        let gp = gamepads[i];
        if (!gp || !gp.connected) {
          continue;
        }
  
        // Only send a subset of the available information. Also, the floating
        // point values are not constant; they are constantly changing in the
        // least significant digits even when the gamepad is untouched by the
        // user. We truncate the floating point values to two significant
        // digits to reject this noise.
        gamepad = {
          'index': gp.index,
          'button_values': gp.buttons.map(
            a => clamp(Math.round(a.value * 100) / 100, 0, 1)),
          'axes': gp.axes.map(
            a => clamp(Math.round(a * 100) / 100, -1, 1)),
        };
        break;  // Just take the first connected gamepad.
      }
      if (viewer.connection && viewer.connection.readyState == WebSocket.OPEN &&
        JSON.stringify(gamepad) !== JSON.stringify(last_gamepad)) {
          viewer.connection.send(msgpack.encode(
          { 'type': 'gamepad', 'name': '', 'gamepad': gamepad }));
        last_gamepad = gamepad;
      }
    }
  
    function animate() {
      // stats.begin();
      // convert realtime rate to percentage so it is easier to read
      // realtimeRatePanel.update(latestRealtimeRate*100, 100);
      viewer.animate()
      // stats.end();
      if (gamepads_supported) {
        handle_gamepads();
      }
      requestAnimationFrame(animate);
    }
    // TODO(#16486): Replace this function with more robust custom command
    //  handling in Meshcat
    function handle_message(ws_message: any) {
      let decoded: any = viewer.decode(ws_message);
      if (decoded.type == "realtime_rate") {
        latestRealtimeRate = decoded.rate;
      } else if (decoded.type == "show_realtime_rate") {
        // stats.dom.style.display = decoded.show ? "block" : "none";
      } else {
        viewer.handle_command(decoded)
      }
    }

    requestAnimationFrame( animate );
    // Set background to match Drake Visualizer.
    viewer.set_property(['Background'], "top_color", [.95, .95, 1.0]);
    viewer.set_property(['Background'], "bottom_color", [.32, .32, .35]);
    // Set the initial view looking up the y-axis.
    viewer.set_property(['Cameras', 'default', 'rotated', '<object>'],
                        "position", [0.0, 1.0, 3.0]);

    // <!-- CONNECTION BLOCK BEGIN -->
    // The lifespan of the server may be much shorter than this visualizer
    // client. We'd like the user to not have to explicitly reload when they
    // start a new server. So, we automatically try to reconnect at some given
    // rate. However, due to the split of visualizer state between server and
    // client, simply reconnecting may leave the *existing* visualizer in a
    // strange state with various stale artifacts. So, when we detect a
    // *reconnection*, we simply reload the page, so that every *meaningful*
    // connection is accompanied by a fresh client state. Upon loading the
    // page, it can accept a connection. After that first connection, any
    // new connection is interpreted as a signal to reload.
    var accepting_connections = true;
    // status_dom = document.getElementById("status-message");
    // When the connection closes, try creating a new connection automatically.
    function make_connection(url: string, reconnect_ms: number) {
      try {
        let connection = new WebSocket(url);
        connection.binaryType = "arraybuffer";
        connection.onmessage = (msg) => handle_message(msg);
        connection.onopen = (evt) => {
          if (!accepting_connections) location.reload();
          accepting_connections = false
        };
        connection.onclose = function(evt) {
          // status_dom.style.display = "block";
          if (do_reconnect) {
            // Immediately schedule an attempt to reconnect.
            setTimeout(() => {make_connection(url, reconnect_ms);}, reconnect_ms);
          }
        }
        viewer.connection = connection
      } catch (e) {
        console.info("Not connected to MeshCat websocket server: ", e);
        if (do_reconnect) {
          setTimeout(() => {make_connection(url, reconnect_ms);}, reconnect_ms);
        }
      }
    }

    const queryString = window.location.search;
    const urlParams = new URLSearchParams(queryString);
    let reconnect_ms_string = urlParams.get('reconnect_ms');
    let reconnect_ms = parseInt(reconnect_ms_string ? reconnect_ms_string : "1000");
    var do_reconnect = reconnect_ms > 0;
    if (do_reconnect) {
      // status_dom.textContent = "No connection to server. Attempting to reconnect...";
    }

    let url = location.toString();
    url = url.replace("http://", "ws://");
    url = url.replace("https://", "wss://");
    url = url.replace("/index.html", "/");
    url = url.replace("/meshcat.html", "/");
    url = url.replace(":8001", ":8003");
    url = url.replace(":8002", ":8003");
    make_connection(url, reconnect_ms);
    console.log(url);
    // <!-- CONNECTION BLOCK END -->
    return viewer;
}
