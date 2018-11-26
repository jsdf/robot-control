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

function debugPrintVector3(vec: VectorR3 | THREE.Vector3) {
  return `{x:${vec.x.toFixed(3)},y:${vec.y.toFixed(3)},z:${vec.z.toFixed(3)}}`;
}

function degToRad(angleDegrees: number): number {
  return angleDegrees * Math.PI / 180.0;
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
  transformControls: Object;
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

  constructor(
    scene: THREE.Scene,
    transformControls: Object,
    debugLog: string => void
  ) {
    this.scene = scene;
    this.transformControls = transformControls;
    this.debugLog = debugLog;

    // base
    this._insertIKNode(
      makeNode(
        VectorR3_Zero(), // startPos
        VectorR3_Zero(), // rotationAxis
        JOINT, // purpose (joint or effector)
        0, // minJointAngle in radians
        0 // maxJointAngle in radians
      )
    );
    this._insertIKNode(
      makeNode(
        VectorR3_Zero(), // startPos
        VectorR3_UnitY(), // rotationAxis
        JOINT, // purpose (joint or effector)
        4 * -Math.PI, // minJointAngle in radians
        4 * Math.PI // maxJointAngle in radians
      )
    );
    last(this.ikNodes).Freeze();

    this.addJoint(0, 0, 0);
    last(this.ikNodes).Freeze();
    this.addJoint(0, 2, 0);
    this.addJoint(0, 4, 0);
    this.addEndEffector(0, 0, 0); // wtf is this coordinate system?
    this.targetVectors.push(new VectorR3(0, 4, 0));

    this.ikJacobian = new Jacobian(this.ikTree);

    this._resetIKState();
    this._stepIKState();
    this._createDebugPoints();
    this._updateDebugPoints();

    // create some graphics proxy for the tracking target
    this.targetProxy = this._makeTargetProxy(this.targetVectors[0]);
  }

  update() {
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
              `${node.s.toString()} ${node.attach.toString()} θ=${node.theta} ${
                node.purpose
              }`
          )
          .join('\n') +
        '\n'
    );

    this.debugLog(
      'debugPoints:\n' +
        this.debugPoints.map(p => debugPrintVector3(p.position)).join('\n') +
        '\n'
    );
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
      degToRad(-60), // minJointAngle in radians
      degToRad(60) // maxJointAngle in radians
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
    const cube = makeBox(0.5, 0x00ff00, true);
    cube.position.set(pos.x, pos.y, pos.z);
    this.scene.add(cube);
    this.transformControls.attach(cube);
    return cube;
  }

  _createDebugPoints() {
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
