// @flow
//
import type {Purpose} from './lib/Node';

import Node, {JOINT, EFFECTOR} from './lib/Node';
import {
  VectorR3,
  VectorR3_UnitX,
  VectorR3_UnitY,
  VectorR3_UnitZ,
  VectorR3_Zero,
} from './lib/LinearR3';

import Jacobian from './lib/Jacobian';
import Tree from './lib/Tree';

const THREE = require('three');
const TransformControls = require('three-transform-controls')(THREE);

function makeBox(
  size: number,
  color: number | string,
  wireframe: boolean = false
) {
  const geometry = new THREE.BoxGeometry(size, size, size);
  const material = new THREE.MeshBasicMaterial({color, wireframe});
  const cube = new THREE.Mesh(geometry, material);
  return cube;
}

function toFixed(num, width) {
  return `${num < 0 ? '' : ' '}${num.toFixed(width)}`;
}

function debugPrintVector3(vec: VectorR3 | THREE.Vector3) {
  return `{x:${toFixed(vec.x, 3)},y:${toFixed(vec.y, 3)},z:${toFixed(
    vec.z,
    3
  )}}`;
}

function radToDeg(angleRadians: number): number {
  return angleRadians * (180.0 / Math.PI);
}

function degToRad(angleDegrees: number): number {
  return angleDegrees * (Math.PI / 180.0);
}

function makeArc(startAngle: number, size: number) {
  return new THREE.CylinderGeometry(
    1,
    1,
    0.2, // geom thickness
    32, // geom segments
    1,
    false,
    startAngle,
    size
  );
}

function makeNode(
  startPos: VectorR3,
  rotationAxis: VectorR3,
  purpose: Purpose,
  minJointAngle: number = -Math.PI, // radians
  maxJointAngle: number = Math.PI // radians
) {
  return new Node(
    startPos,
    rotationAxis,
    0.08, // 'size', seems to be unused
    purpose,
    minJointAngle,
    maxJointAngle
  );
}

function last<T>(arr: Array<T>): T {
  return arr[arr.length - 1];
}

export default class Robot {
  scene: THREE.Scene;
  camera: THREE.Camera;
  renderer: THREE.Renderer;
  ikTree: Tree = new Tree();
  ikNodes: Array<Node> = [];
  ikJacobian: Jacobian; // the ik solver
  targetVectors: Array<VectorR3> = [];
  // ik destination controlled by targets or end effectors?
  useTargets = true;
  targetProxy: THREE.Object3D;
  debugPoints: Array<THREE.Object3D> = [];
  lineGeometry: THREE.Geometry;
  debugLog: string => void;
  debugTextAtPosition: (string, THREE.Object3D) => void;

  constructor(
    scene: THREE.Scene,
    camera: THREE.Camera,
    renderer: THREE.Renderer,
    debugLog: string => void,
    debugTextAtPosition: (string, THREE.Object3D) => void
  ) {
    this.scene = scene;
    this.camera = camera;
    this.renderer = renderer;
    this.debugLog = debugLog;
    this.debugTextAtPosition = debugTextAtPosition;

    // base
    this._insertIKNode(
      makeNode(
        VectorR3_Zero(), // startPos
        VectorR3_UnitY(), // rotationAxis
        JOINT, // purpose (joint or effector)
        4 * -Math.PI, // minJointAngle in radians
        4 * Math.PI // maxJointAngle in radians
      )
    );

    this.addJoint(0, 1, 0);
    this.addJoint(0, 3, 0);
    this.addJoint(0, 4, 0);
    this.addEndEffector(0, 3, 0);
    this.targetVectors.push(new VectorR3(0, 5, 0));

    this.ikJacobian = new Jacobian(this.ikTree);

    this._resetIKState();
    this._stepIKState();
    this._createDebugPoints();
    this._updateDebugPoints();

    // create some graphics proxy for the tracking target
    this.targetProxy = this._makeTargetProxy(this.targetVectors[0]);
  }

  update() {
    // can't go below ground
    if (this.targetProxy.position.y < 0) {
      this.targetProxy.position.y = 0;
    }
    // copy current position from target proxy object
    this.targetVectors[0].Set(
      this.targetProxy.position.x,
      this.targetProxy.position.y,
      this.targetProxy.position.z
    );
    // update ik solution
    this._stepIKState();
    // move threejs geometries as a result of ik update
    this._updateDebugPoints();

    this._debugLogging();
  }

  _debugLogging() {
    this.debugLog('\n');
    this.debugLog(
      this.targetProxy &&
        `targetProxy.position: ${debugPrintVector3(
          this.targetProxy.position
        )}\n`
    );

    this.debugLog(
      'ikNodes:\n' +
        this.ikNodes
          .map(
            node =>
              `s=${debugPrintVector3(node.s)} a=${debugPrintVector3(
                node.attach
              )} θ=${radToDeg(node.theta)
                .toFixed(1)
                .padStart(6, ' ')}deg ${node.purpose}`
          )
          .join('\n') +
        '\n'
    );

    this.debugLog(
      'debugPoints:\n' +
        this.debugPoints
          .map(
            (p, i) =>
              debugPrintVector3(p.position) +
              ` valid=${this._validatePoint(p, i) ? 'y' : 'n'}`
          )
          .join('\n') +
        '\n'
    );

    this.debugPoints.forEach((point, i) => {
      this.debugTextAtPosition(`${i}`, point);
    });

    this.debugTextAtPosition(
      `${debugPrintVector3(this.targetProxy.position)}`,
      this.targetProxy
    );
  }

  _validatePoint(p: THREE.Object3D, i: number) {
    if (i > 0) {
      return p.position.y >= 0;
    }
    return true;
  }

  _stepIKState() {
    if (this.useTargets) {
      this.ikJacobian.SetJtargetActive();
    } else {
      this.ikJacobian.SetJendActive();
    }

    // Set up ik solver (Jacobian) and position offset (deltaS) vectors
    this.ikJacobian.ComputeJacobian(this.targetVectors);

    // Calculate the change in joint rotation (theta) values
    this.ikJacobian.CalcDeltaThetasSDLS(); // using selectively damped least squares method

    // Apply the change in the joint rotation (theta) values
    this.ikJacobian.UpdateThetas();

    // update position offset (deltaS) vectors
    this.ikJacobian.UpdatedSClampValue(this.targetVectors);
  }

  addJoint(x: number, y: number, z: number) {
    const ikNode = makeNode(
      new VectorR3(x, y, z), // startPos
      VectorR3_UnitZ(), // rotationAxis
      JOINT, // purpose (joint or effector)
      degToRad(-180), // minJointAngle in radians
      degToRad(180) // maxJointAngle in radians
    );

    this._insertIKNode(ikNode);
  }

  addEndEffector(x: number, y: number, z: number) {
    const ikNode = makeNode(
      new VectorR3(x, y, z), // startPos
      VectorR3_UnitZ(), // rotationAxis
      EFFECTOR // purpose (joint or effector)
    );
    this._insertIKNode(ikNode);
  }

  _insertIKNode(ikNode: Node) {
    if (this.ikNodes.length === 0) {
      this.ikTree.InsertRoot(ikNode);
    } else {
      this.ikTree.InsertLeftChild(
        this.ikNodes[this.ikNodes.length - 1],
        ikNode
      );
    }
    this.ikNodes.push(ikNode);
  }

  _resetIKState() {
    this.ikTree.Init();
    this.ikTree.Compute();
    this.ikJacobian.Reset();
  }

  _makeTargetProxy(pos: VectorR3): THREE.Object3D {
    const transformControls = new TransformControls(
      this.camera,
      this.renderer.domElement
    );

    const cube = makeBox(0.5, 0x00ff00, true);
    cube.position.set(pos.x, pos.y, pos.z);
    this.scene.add(cube);
    transformControls.attach(cube);
    return cube;
  }

  _createDebugPoints() {
    // ground
    const groundGeometry = new THREE.PlaneGeometry(10, 10, 10, 10);
    const groundMaterial = new THREE.MeshBasicMaterial({
      color: (0xaabbcc: number | string),
      wireframe: true,
    });
    const plane = new THREE.Mesh(groundGeometry, groundMaterial);
    plane.rotation.x = degToRad(-90);
    this.scene.add(plane);

    this.ikNodes.forEach(node => {
      const point = makeBox(0.1, Math.random() * 0xffffff);
      this.debugPoints.push(point);
      this.scene.add(point);
    });

    // build line
    const lineGeom = new THREE.Geometry();
    this.ikNodes.forEach(node => {
      const vert = new THREE.Vector3(0, 0, 0);
      lineGeom.vertices.push(vert);
    });
    const lineMaterial = new THREE.LineBasicMaterial({
      color: (0x0000ff: number | string),
    });

    const line = new THREE.Line(lineGeom, lineMaterial);
    this.scene.add(line);
    this.lineGeometry = lineGeom;
  }

  _updateDebugPoints() {
    for (var i = 0; i < this.debugPoints.length; i++) {
      const pos = this.ikNodes[i].s;
      this.debugPoints[i].position.set(pos.x, pos.y, pos.z);
      this.lineGeometry.vertices[i].set(pos.x, pos.y, pos.z);
    }
    this.lineGeometry.verticesNeedUpdate = true;
  }
}
