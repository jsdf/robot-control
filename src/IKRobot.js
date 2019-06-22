// @flow
//
import type {Purpose} from './lib/Node';
import type {Vec3Interface} from './Vec3';

import Node, {JOINT, EFFECTOR} from './lib/Node';
import {
  VectorR3,
  VectorR3_UnitX,
  VectorR3_UnitY,
  VectorR3_UnitZ,
  VectorR3_Zero,
} from './lib/LinearR3';
import * as Vec3 from './Vec3';
import Jacobian from './lib/Jacobian';
import Tree from './lib/Tree';

import Collision from './Collision';

const THREE = require('three');
const TransformControls = require('three-transform-controls')(THREE);

const RED = 0xff0000;
const GREEN = 0x00ff00;
const BLUE = 0x0000ff;
const ORANGE = 0xff9900;
const WHITE = 0xfffffff;

/*
TODO:
- validation
  - collision detection for arm-arm collision
- movement solution
  - candidate integral steps
    - could use IK steps to find intermediate positions
  - ghost render for integral steps
  - find alternate options to invalid integral steps
    - simulated annealing
    - pathfinding algo + target pos?
- motion control
- hand grip pos and rotation
- server communication
- waypoint-based movement paths
 */

const SHOW_TARGET_POS = false;
const SHOW_JOINT_POS = false;

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

function makeSphere(
  radius: number,
  color: number | string,
  wireframe: boolean = false
) {
  const geometry = new THREE.SphereGeometry(radius, 5, 5);
  const material = new THREE.MeshBasicMaterial({color, wireframe});
  const sphere = new THREE.Mesh(geometry, material);
  return sphere;
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

function makeTHREEVector3(init: Vec3Interface) {
  return new THREE.Vector3(init.x, init.y, init.z);
}

const annealingE = 2.71828;
function annealingAcceptanceProbability(oldCost, newCost, T) {
  return annealingE * ((oldCost - newCost) / T);
}

type SolutionAndCost = {
  solution: Array<number>,
  cost: number,
  positions: Array<Vec3Interface>,
};

class ArmSolution {
  ikTree: Tree = new Tree();
  ikNodes: Array<Node> = [];
  ikJacobian: Jacobian; // the ik solver
  targetVectors: Array<VectorR3> = [];
  // ik destination controlled by targets or end effectors?
  useTargets = true;
  collision: Collision;

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

    const armBaseTiltJoint = this.addJoint(
      0,
      1,
      0,
      baseRototatorJoint,
      -90,
      90
    );
    const armMidJoint = this.addJoint(0, 3, 0, armBaseTiltJoint, -130, 130);
    const gripperTiltJoint = this.addJoint(0, 4, 0, armMidJoint, -130, 130);
    this.addEndEffector(0, 3, 0, gripperTiltJoint);
    this.targetVectors.push(new VectorR3(0, 6, 0));

    this.ikJacobian = new Jacobian(this.ikTree);

    this._resetIKState();
    if (initialSolution) {
      this.applySolution(initialSolution);
    }
    this.stepIKState();

    this.collision = new Collision(this.ikNodes.map(node => node.s));
  }

  update() {
    this.stepIKState();
    this.collision.update();
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

  addJoint(
    x: number,
    y: number,
    z: number,
    parent: Node,
    minAngle: number = -180,
    maxAngle: number = 180
  ) {
    const ikNode = makeNode(
      new VectorR3(x, y, z), // startPos
      VectorR3_UnitZ(), // rotationAxis
      JOINT, // purpose (joint or effector)
      degToRad(minAngle), // minJointAngle in radians
      degToRad(maxAngle) // maxJointAngle in radians
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
    this.collision.update();
    return (
      !this.collision.areAnyColliding() &&
      this.ikNodes.every((node, i) => this.validatePoint(node.s, i))
    );
  }

  serialize(): Array<number> {
    return this.ikNodes.map(node => node.theta);
  }

  applySolution(solution: Array<number>) {
    this.ikNodes.forEach((node, i) => {
      node.theta = solution[i];
    });

    this.ikTree.Compute();
  }

  // simulated annealing stuff

  _annealSolution(sol: SolutionAndCost): SolutionAndCost {
    let oldSol = sol;
    let T = 1.0;
    const T_min = 0.00001;
    const alpha = 0.9;
    while (T > T_min) {
      for (let i = 1; i < 50; i++) {
        let newSol = this._generateNeighborSolution(oldSol);
        let ap = annealingAcceptanceProbability(oldSol.cost, newSol.cost, T);
        if (ap > Math.random()) {
          oldSol = newSol;
        }
      }
      T = T * alpha;
    }
    return oldSol;
  }

  _generateNeighborSolution(sol: SolutionAndCost): SolutionAndCost {
    let solution;
    do {
      solution = this._generateNeighbourSolution(sol.solution);
      this.applySolution(solution);
    } while (!this.solutionIsValid());
    const positions = this._getWorldPositions();
    const cost = this._getCostFor(positions, solution);
    return {
      solution,
      cost,
      positions,
    };
  }

  lastSolution: ?SolutionAndCost = null;
  _getCostFor(positions: Array<Vec3Interface>, solution: Array<number>) {
    const lastSolution = this.lastSolution;
    const lastPosImportance = 0.5;
    return (
      Vec3.distanceTo(last(this.ikNodes).s, this.targetVectors[0]) +
      (lastSolution
        ? solution.reduce(
            (acc, v, i) => acc + (solution[i] - lastSolution.solution[i]),
            0
          ) /
            solution.length +
          (positions.reduce(
            (acc, v, i) => acc + Vec3.distanceTo(v, lastSolution.positions[i]),
            0
          ) /
            positions.length) *
            lastPosImportance
        : 0)
    );
  }

  _getWorldPositions(): Array<Vec3Interface> {
    const positions = [];
    for (var i = 0; i < this.ikNodes.length; i++) {
      positions.push(this.ikNodes[i].s);
    }
    return positions;
  }

  _generateNeighbourSolution(prev: Array<number>) {
    const i = Math.floor(this.ikNodes.length * Math.random());
    const solution: Array<number> = prev.slice(0);

    solution[i] =
      this.ikNodes[i].minTheta +
      Math.random() * (this.ikNodes[i].maxTheta - this.ikNodes[i].minTheta);

    return solution;
  }
}

type ArmRendererState = 'planned' | 'committed';
class ArmRenderer {
  armSolution: ArmSolution;
  jointMeshes: Array<THREE.Mesh> = [];
  collisionMeshes: Array<THREE.Mesh> = [];
  lineGeometry: THREE.Geometry;
  scene: THREE.Scene;
  state: ArmRendererState;
  constructor(
    armSolution: ArmSolution,
    scene: THREE.Scene,
    state: ArmRendererState
  ) {
    this.state = state;
    this.scene = scene;
    this.armSolution = armSolution;

    this.armSolution.ikNodes.forEach(node => {
      const point = makeBox(0.1, WHITE, state === 'committed');
      this.jointMeshes.push(point);
      this.scene.add(point);
    });

    // build line
    const lineGeom = new THREE.Geometry();
    this.armSolution.ikNodes.forEach(node => {
      const vert = new THREE.Vector3(0, 0, 0);
      lineGeom.vertices.push(vert);
    });
    const lineMaterial = new THREE.LineBasicMaterial({
      color: ((state === 'planned' ? BLUE : ORANGE): number | string),
    });

    const line = new THREE.Line(lineGeom, lineMaterial);
    this.scene.add(line);
    this.lineGeometry = lineGeom;

    if (this.state === 'planned') {
      // build collision viz
      this.armSolution.collision.volumes.forEach(vol => {
        const sphere = makeSphere(vol.radius, GREEN);
        this.collisionMeshes.push(sphere);
        this.scene.add(sphere);
      });
    }
  }

  update() {
    this.jointMeshes.forEach((jointMesh, i) => {
      const pos = this.armSolution.ikNodes[i].s;
      jointMesh.position.set(pos.x, pos.y, pos.z);

      if (this.state === 'planned') {
        setColor(
          jointMesh,
          this.armSolution.validatePoint(jointMesh.position, i) ? GREEN : RED
        );
      }
      this.lineGeometry.vertices[i].set(pos.x, pos.y, pos.z);
    });
    this.lineGeometry.verticesNeedUpdate = true;

    if (this.state === 'planned') {
      this.collisionMeshes.forEach((collisionMesh, i) => {
        const volume = this.armSolution.collision.volumes[i];
        const volCenter = volume.center;
        collisionMesh.position.set(volCenter.x, volCenter.y, volCenter.z);
        setColor(collisionMesh, volume.isColliding ? RED : GREEN);
      });
    }
  }
}

type AnimationKeyframe = {
  interval: number,
  plan: {plan: Array<number>, target: Vec3Interface},
};
class Animation {
  _framesTimer: ?AnimationFrameID = null;
  currentFrame = 0;
  startTime = Date.now();
  elapsedFrameIntervalSum = 0;
  frames: Array<AnimationKeyframe>;
  robot: Robot;
  constructor(
    robot: Robot,
    unloopedAnimation: Array<AnimationKeyframe>,
    loop: boolean = false
  ) {
    this.robot = robot;
    this.frames = unloopedAnimation;
    if (loop) {
      this.frames = unloopedAnimation.concat(last(unloopedAnimation));
    }

    this.currentFrame = 0;
    this.startTime = Date.now();
    this.elapsedFrameIntervalSum = 0;

    const enqueueFrame = () => {
      const animTimer = requestAnimationFrame(() => {
        if (this._framesTimer != animTimer) {
          return; // kill this anim
        }
        this._framesTimer = null;
        let sinceStart = (Date.now() - this.startTime) / 1000;

        while (
          this.frames[this.currentFrame] &&
          this.elapsedFrameIntervalSum +
            this.frames[this.currentFrame].interval <
            sinceStart
        ) {
          this.elapsedFrameIntervalSum += this.frames[
            this.currentFrame
          ].interval;
          this.currentFrame++;
          const nextFrame = this.frames[this.currentFrame];
          if (nextFrame) {
            this.robot.loadPlan(nextFrame.plan);
          }
        }

        if (this.frames[this.currentFrame + 1]) {
          const frameCompletion =
            sinceStart -
            this.elapsedFrameIntervalSum /
              this.frames[this.currentFrame].interval;
          const interpolated = Vec3.lerp(
            this.frames[this.currentFrame].plan.target,
            this.frames[this.currentFrame + 1].plan.target,
            frameCompletion
          );

          Vec3.copyFrom(this.robot.targetProxies[0].position, interpolated);
        }

        if (loop && !this.frames[this.currentFrame + 1]) {
          this.currentFrame = 0;
          this.startTime = Date.now();
          this.elapsedFrameIntervalSum = 0;
        }

        if (this.frames[this.currentFrame + 1]) {
          enqueueFrame();
        }
      });
      this._framesTimer = animTimer;
    };
    if (this.frames[this.currentFrame] == null) {
      return;
    }
    enqueueFrame();
    this.robot.loadPlan(this.frames[this.currentFrame].plan);
  }

  destroy() {
    const activeTimer = this._framesTimer;

    if (activeTimer != null) {
      cancelAnimationFrame(activeTimer);
    }
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
  initialConfiguration: Array<number>;
  initialTarget: Vec3Interface;

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
    // fix for stuck initial state
    this.plannedArmSolution.targetVectors[0].x += Math.random() * 0.01;
    this.plannedArmSolution.targetVectors[0].y += Math.random() * 0.01;
    this.plannedArmSolution.targetVectors[0].z += Math.random() * 0.01;
    this.plannedArmSolution.stepIKState();

    this.initialConfiguration = this.plannedArmSolution.serialize();
    this.initialTarget = Vec3.clone(this.plannedArmSolution.targetVectors[0]);
    this.committedArmSolution = new ArmSolution(this.initialConfiguration);

    this.plannedRenderer = new ArmRenderer(
      this.plannedArmSolution,
      this.scene,
      'planned'
    );
    this.committedRenderer = new ArmRenderer(
      this.committedArmSolution,
      this.scene,
      'committed'
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
      if (this.targetProxies[i].position.y <= 0) {
        this.targetProxies[i].position.y = 0.001;
      }
      // copy current position from target proxy object
      this.plannedArmSolution.targetVectors[i].Set(
        this.targetProxies[i].position.x,
        this.targetProxies[i].position.y,
        this.targetProxies[i].position.z
      );
    }
    // update ik solution
    this.plannedArmSolution.update();
    // move threejs geometries as a result of ik update
    this._updateGraphics();

    this._debugLogging();
  }

  commitPlan(): Array<number> {
    const plan = this.plannedArmSolution.serialize();
    this.committedArmSolution.applySolution(plan);
    return plan;
  }

  getPlan(): {plan: Array<number>, target: Vec3Interface} {
    return {
      plan: this.plannedArmSolution.serialize(),
      target: Vec3.clone(this.plannedArmSolution.targetVectors[0]),
    };
  }

  loadPlan(args: {plan: Array<number>, target: Vec3Interface}): void {
    this.plannedArmSolution.applySolution(args.plan);
    Vec3.copyFrom(this.targetProxies[0].position, args.target);
  }

  resetToInitial(alsoResetTarget: boolean = false): void {
    this.plannedArmSolution.applySolution(this.initialConfiguration);
    if (alsoResetTarget) {
      Vec3.copyFrom(this.targetProxies[0].position, this.initialTarget);
    }
  }

  animation: ?Animation = null;
  playAnimation(
    unloopedAnimation: Array<AnimationKeyframe>,
    loop: boolean = false
  ) {
    if (this.animation) {
      this.animation.destroy();
    }
    this.animation = new Animation(this, unloopedAnimation, loop);
  }

  _debugLogging() {
    this.debugLog('\n');
    for (let i = 0; i < this.targetProxies.length; i++) {
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

    ['planned', 'committed'].forEach(type => {
      const armSolution =
        type === 'planned'
          ? this.plannedArmSolution
          : this.committedArmSolution;

      this.debugLog(
        type +
          ':\n' +
          armSolution.ikNodes
            .map((node, i) => {
              const formattedPos = SHOW_JOINT_POS
                ? `s=${debugPrintVector3(node.s)}`
                : '';

              const formattedPurpose = node.purpose.padEnd(8);
              const formattedAngle = radToDeg(node.theta)
                .toFixed(1)
                .padStart(6, ' ');

              const attach = false ? `a=${debugPrintVector3(node.attach)}` : '';
              return `${formattedPurpose} ${i} ${formattedPos}${attach} θ=${formattedAngle}deg ${node.theta.toFixed(
                1
              )} ok=${armSolution.validatePoint(node.s, i) ? 'y' : 'n'}`;
            })
            .join('\n') +
          '\n'
      );
    });

    this.plannedRenderer.jointMeshes.forEach((point, i) => {
      this.debugTextAtPosition(`${i}`, point);
    });

    // this.plannedRenderer.collisionMeshes.forEach((point, i) => {
    //   this.debugTextAtPosition(
    //     `${this.plannedArmSolution.collision.volumes[i].radius.toFixed(2)}`,
    //     point
    //   );
    // });
    if (SHOW_TARGET_POS) {
      for (var i = 0; i < this.targetProxies.length; i++) {
        this.debugTextAtPosition(
          `${debugPrintVector3(this.targetProxies[i].position)}`,
          this.targetProxies[i]
        );
      }
    }
  }

  _makeTargetProxy(pos: VectorR3): THREE.Object3D {
    const transformControls = new TransformControls(
      this.camera,
      this.renderer.domElement
    );
    this.scene.add(transformControls);

    const cube = makeBox(0.5, GREEN, true);
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
    this.committedRenderer.update();
  }
}
