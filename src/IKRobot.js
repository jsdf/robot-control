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

type Vec3Interface = {
  x: number,
  y: number,
  z: number,
};

const THREE = require('three');
const TransformControls = require('three-transform-controls')(THREE);

function setColor(obj: THREE.Mesh, color: number) {
  const {material} = obj;
  if (material instanceof THREE.MeshBasicMaterial) {
    material.color.setHex(color);
  }
}

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

class ArmSolution {
  ikTree: Tree = new Tree();
  ikNodes: Array<Node> = [];
  ikJacobian: Jacobian; // the ik solver
  targetVectors: Array<VectorR3> = [];
  // ik destination controlled by targets or end effectors?
  useTargets = true;

  constructor(initialSolution?: Array<number>) {
    const baseRototatorJoint = this._insertIKNode(
      makeNode(
        VectorR3_Zero(), // startPos
        VectorR3_UnitY(), // rotationAxis
        JOINT, // purpose (joint or effector)
        4 * -Math.PI, // minJointAngle in radians
        4 * Math.PI // maxJointAngle in radians
      ),
      null // root
    );

    const armBaseTiltJoint = this.addJoint(0, 1, 0, baseRototatorJoint);
    const armMidJoint = this.addJoint(0, 3, 0, armBaseTiltJoint);
    const gripperTiltJoint = this.addJoint(0, 4, 0, armMidJoint);
    this.addEndEffector(0, 3, 0, gripperTiltJoint);
    this.targetVectors.push(new VectorR3(0, 5, 0));

    this.ikJacobian = new Jacobian(this.ikTree);

    this._resetIKState();
    if (initialSolution) {
      this.applySolution(initialSolution);
    } else {
      this.stepIKState();
    }
  }

  stepIKState() {
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

  addJoint(x: number, y: number, z: number, parent: Node) {
    const ikNode = makeNode(
      new VectorR3(x, y, z), // startPos
      VectorR3_UnitZ(), // rotationAxis
      JOINT, // purpose (joint or effector)
      degToRad(-180), // minJointAngle in radians
      degToRad(180) // maxJointAngle in radians
    );

    this._insertIKNode(ikNode, parent);
    return ikNode;
  }

  addEndEffector(
    x: number,
    y: number,
    z: number,
    parent: Node,
    rightChild: boolean = false
  ) {
    const ikNode = makeNode(
      new VectorR3(x, y, z), // startPos
      VectorR3_UnitZ(), // rotationAxis
      EFFECTOR // purpose (joint or effector)
    );

    if (rightChild) {
      this.ikTree.InsertRightSibling(parent, ikNode);
    } else {
      this.ikTree.InsertLeftChild(parent, ikNode);
    }
    this.ikNodes.push(ikNode);
    return ikNode;
  }

  _insertIKNode(ikNode: Node, parent: ?Node) {
    if (parent == null) {
      this.ikTree.InsertRoot(ikNode);
    } else {
      this.ikTree.InsertLeftChild(parent, ikNode);
    }
    this.ikNodes.push(ikNode);
    return ikNode;
  }

  _resetIKState() {
    this.ikTree.Init();
    this.ikTree.Compute();
    this.ikJacobian.Reset();
  }

  validatePoint(p: Vec3Interface, i: number) {
    if (i > 0) {
      return p.y >= 0;
    }
    return true;
  }

  solutionIsValid() {
    return this.ikNodes.every((node, i) => this.validatePoint(node.s, i));
  }

  serialize(): Array<number> {
    return this.ikNodes.map(node => node.theta);
  }

  applySolution(solution: Array<number>) {
    this.ikNodes.forEach((node, i) => {
      node.theta = solution[i];
    });
    this.stepIKState();
  }
}

class ArmRenderer {
  armSolution: ArmSolution;
  debugPoints: Array<THREE.Mesh> = [];
  lineGeometry: THREE.Geometry;
  scene: THREE.Scene;
  constructor(armSolution: ArmSolution, scene: THREE.Scene) {
    this.scene = scene;
    this.armSolution = armSolution;

    this.armSolution.ikNodes.forEach(node => {
      const point = makeBox(0.1, Math.random() * 0xffffff);
      this.debugPoints.push(point);
      this.scene.add(point);
    });

    // build line
    const lineGeom = new THREE.Geometry();
    this.armSolution.ikNodes.forEach(node => {
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

  update() {
    for (var i = 0; i < this.debugPoints.length; i++) {
      const pos = this.armSolution.ikNodes[i].s;
      this.debugPoints[i].position.set(pos.x, pos.y, pos.z);

      setColor(
        this.debugPoints[i],
        this.armSolution.validatePoint(this.debugPoints[i].position, i)
          ? 0x00ff00
          : 0xff0000
      );
      this.lineGeometry.vertices[i].set(pos.x, pos.y, pos.z);
    }
    this.lineGeometry.verticesNeedUpdate = true;
  }
}

export default class Robot {
  scene: THREE.Scene;
  camera: THREE.Camera;
  renderer: THREE.Renderer;
  targetProxies: Array<THREE.Object3D> = [];
  debugLog: string => void;
  debugTextAtPosition: (string, THREE.Object3D) => void;

  plannedArmSolution: ArmSolution;
  plannedRenderer: ArmRenderer;
  committedArmSolution: ArmSolution;
  committedRenderer: ArmRenderer;

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

    this.plannedArmSolution = new ArmSolution();
    this.committedArmSolution = new ArmSolution(
      this.plannedArmSolution.serialize()
    );

    this.plannedRenderer = new ArmRenderer(this.plannedArmSolution, this.scene);
    this.committedRenderer = new ArmRenderer(
      this.committedArmSolution,
      this.scene
    );

    this._createStaticGraphics();
    this._updateGraphics();

    // create some graphics proxy for the tracking target

    for (var i = 0; i < this.plannedArmSolution.targetVectors.length; i++) {
      this.targetProxies[i] = this._makeTargetProxy(
        this.plannedArmSolution.targetVectors[i]
      );
    }
  }

  update() {
    for (var i = 0; i < this.targetProxies.length; i++) {
      // can't go below ground
      if (this.targetProxies[i].position.y < 0) {
        this.targetProxies[i].position.y = 0;
      }
      // copy current position from target proxy object
      this.plannedArmSolution.targetVectors[i].Set(
        this.targetProxies[i].position.x,
        this.targetProxies[i].position.y,
        this.targetProxies[i].position.z
      );
    }
    // update ik solution
    this.plannedArmSolution.stepIKState();
    // move threejs geometries as a result of ik update
    this._updateGraphics();

    this._debugLogging();
  }

  _debugLogging() {
    this.debugLog('\n');
    for (var i = 0; i < this.targetProxies.length; i++) {
      this.debugLog(
        this.targetProxies[i] &&
          `targetProxies[${i}].position: ${debugPrintVector3(
            this.targetProxies[i].position
          )}\n`
      );
    }
    this.debugLog(
      `solution=${
        this.plannedArmSolution.solutionIsValid() ? 'valid' : 'invalid'
      }\n`
    );

    this.debugLog(
      'ikNodes:\n' +
        this.plannedArmSolution.ikNodes
          .map(
            node =>
              `s=${debugPrintVector3(node.s)} a=${debugPrintVector3(
                node.attach
              )} θ=${radToDeg(node.theta)
                .toFixed(1)
                .padStart(6, ' ')}deg ${node.purpose} valid=${
                this.plannedArmSolution.validatePoint(node.s, i) ? 'y' : 'n'
              }`
          )
          .join('\n') +
        '\n'
    );

    this.plannedRenderer.debugPoints.forEach((point, i) => {
      this.debugTextAtPosition(`${i}`, point);
    });

    for (var i = 0; i < this.targetProxies.length; i++) {
      this.debugTextAtPosition(
        `${debugPrintVector3(this.targetProxies[i].position)}`,
        this.targetProxies[i]
      );
    }
  }

  _makeTargetProxy(pos: VectorR3): THREE.Object3D {
    const transformControls = new TransformControls(
      this.camera,
      this.renderer.domElement
    );
    this.scene.add(transformControls);

    const cube = makeBox(0.5, 0x00ff00, true);
    cube.position.set(pos.x, pos.y, pos.z);
    this.scene.add(cube);
    transformControls.attach(cube);
    return cube;
  }

  _createStaticGraphics() {
    // ground
    const groundGeometry = new THREE.PlaneGeometry(10, 10, 10, 10);
    const groundMaterial = new THREE.MeshBasicMaterial({
      color: (0xaabbcc: number | string),
      wireframe: true,
    });
    const plane = new THREE.Mesh(groundGeometry, groundMaterial);
    plane.rotation.x = degToRad(-90);
    this.scene.add(plane);
  }

  _updateGraphics() {
    this.plannedRenderer.update();
  }
}
