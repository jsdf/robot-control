// @flow
//
import type {Purpose} from './lib/Node';

import {JOINT, EFFECTOR} from './lib/Node';
import {
  VectorR3,
  VectorR3_UnitX,
  VectorR3_UnitY,
  VectorR3_UnitZ,
  VectorR3_Zero,
} from './lib/LinearR3';

import Jacobian from './lib/Jacobian';

const THREE = require('three');
const TransformControls = require('three-transform-controls')(THREE);

const SLEEP_TIME_BETWEEN_SOLVING = 0;
const SOLVER_ITERATIONS = 10;

class Node extends THREE.Object3D {
  rotationAxis: THREE.Vector3;
  purpose: Purpose;
  minJointAngle: number; // in radians
  maxJointAngle: number; // in radians
  jointAngle = 0;
  isFrozen = false;
  parentNode: ?Node;
  childNode: ?Node;
  constructor(
    startPos: VectorR3,
    startJointAngle: number,
    rotationAxis: VectorR3,
    purpose: Purpose,
    minJointAngle: number = -Math.PI, // radians
    maxJointAngle: number = Math.PI // radians
  ) {
    super();
    this.position.set(startPos.x, startPos.y, startPos.z);
    this.rotationAxis = new THREE.Vector3(
      rotationAxis.x,
      rotationAxis.y,
      rotationAxis.z
    );
    this.setJointAngle(startJointAngle);
    this.purpose = purpose;
    this.minJointAngle = minJointAngle;
    this.maxJointAngle = maxJointAngle;
  }

  setJointAngle(angle: number) {
    this.jointAngle = angle;
    this.rotation.set(
      this.rotationAxis.x * angle,
      this.rotationAxis.y * angle,
      this.rotationAxis.z * angle
    );
  }

  Freeze() {
    this.isFrozen = true;
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

function debugPrintVector3(vec: VectorR3 | THREE.Vector3) {
  return `{x:${vec.x.toFixed(3)},y:${vec.y.toFixed(3)},z:${vec.z.toFixed(3)}}`;
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
    minJointAngle + (maxJointAngle - minJointAngle) / 2,
    rotationAxis,
    purpose,
    minJointAngle,
    maxJointAngle
  );
}

function last<T>(arr: Array<T>): T {
  return arr[arr.length - 1];
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function getWorldPos(node: Node) {
  if (node.parent) {
    node.parent.updateMatrixWorld(false);
  }
  const pos = new THREE.Vector3();
  pos.setFromMatrixPosition(node.matrixWorld);
  return pos;
}

const E = 2.71828;
function acceptanceProbability(oldCost, newCost, T) {
  return E * ((oldCost - newCost) / T);
}

type SolutionAndCost = {
  solution: Array<number>,
  cost: number,
  positions: Array<THREE.Vector3>,
};

export default class Robot {
  scene: THREE.Scene;
  camera: THREE.Camera;
  renderer: THREE.Renderer;
  ikTree: Node;
  ikNodes: Array<Node> = [];
  ikJacobian: Jacobian; // the ik solver
  targetVectors: Array<VectorR3> = [];
  // ik destination controlled by targets or end effectors?
  useTargets = true;
  targetProxy: THREE.Object3D;
  debugPoints: Array<THREE.Object3D> = [];
  lineGeometry: THREE.Geometry;
  debugLog: string => void;

  lastSolutionTime = 0;
  lastSolution: ?SolutionAndCost = null;

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
    this.addJoint(0, 2, 0);
    this.addJoint(0, 2, 0);
    this.addJoint(0, 1, 0);
    this.targetVectors.push(new VectorR3(0, 6, 0));

    // create some graphics proxy for the tracking target
    this.targetProxy = this._makeTargetProxy(this.targetVectors[0]);

    this._stepIKState();
    this._createDebugPoints();
    this._updateDebugPoints();
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
              `${debugPrintVector3(node.position)} θ=${radToDeg(
                node.jointAngle
              ).toFixed(2)}deg ${node.purpose}`
          )
          .join('\n') +
        '\n'
    );

    this.debugLog(
      'debugPoints:\n' +
        this.debugPoints.map(p => debugPrintVector3(p.position)).join('\n') +
        '\n'
    );

    const lastSolution = this.lastSolution;
    this.debugLog(
      'solution:\n' +
        (lastSolution
          ? `${lastSolution.solution.map(n => n.toFixed(2)).join()}\n` +
            `cost=${lastSolution.cost.toFixed(5)}\n`
          : '') +
        '\n'
    );
  }

  _stepIKState() {
    // this.ikNodes[0].setJointAngle(this.ikNodes[0].jointAngle + 0.1);
    // this.ikNodes[1].setJointAngle(this.ikNodes[1].minJointAngle);
    // this.ikNodes[2].setJointAngle(this.ikNodes[2].minJointAngle);
    // this.ikNodes[3].setJointAngle(this.ikNodes[3].minJointAngle);

    if (Date.now() > this.lastSolutionTime + SLEEP_TIME_BETWEEN_SOLVING) {
      // this._randomIterativeSolve();
      // this._iterativeSolveWithConvergence();
      this._simulatedAnnealingSolve();
      this.lastSolutionTime = Date.now();
    }
  }

  _randomIterativeSolve() {
    const candidateSolutions = [];

    if (this.lastSolution) {
      const lastSolution = this.lastSolution;
      candidateSolutions.push({
        solution: lastSolution.solution,
        cost: this._getCost(lastSolution.positions, lastSolution.solution),
        positions: lastSolution.positions,
      });
    }
    for (var i = 0; i < SOLVER_ITERATIONS; i++) {
      candidateSolutions.push(this._generateValidRandomSolution());
    }
    candidateSolutions.sort((a, b) => a.cost - b.cost);

    this._applySolution(candidateSolutions[0].solution);
    this.lastSolution = candidateSolutions[0];
  }

  _iterativeSolveWithConvergence() {
    let candidateSolutions = [];

    let updatedLastSolution;
    if (this.lastSolution) {
      const lastSolution = this.lastSolution;
      updatedLastSolution = {
        solution: lastSolution.solution,
        cost: this._getCost(lastSolution.positions, lastSolution.solution),
        positions: lastSolution.positions,
      };
    }
    for (let i = 0; i < SOLVER_ITERATIONS; i++) {
      candidateSolutions.push(this._generateValidRandomSolution());
    }
    if (updatedLastSolution) {
      candidateSolutions.push(updatedLastSolution);
    }
    candidateSolutions.sort((a, b) => a.cost - b.cost);

    let bestLastGen = candidateSolutions[0];
    candidateSolutions = [];
    for (let i = 0; i < 5; i++) {
      for (let k = 3; k > 0; k--) {
        candidateSolutions.push(
          this._generateValidSolutionInDistance(bestLastGen.solution, k / 10)
        );
      }

      if (updatedLastSolution) {
        candidateSolutions.push(updatedLastSolution);
      }
      candidateSolutions.sort((a, b) => a.cost - b.cost);
      bestLastGen = candidateSolutions[0];
    }

    this._applySolution(candidateSolutions[0].solution);
    this.lastSolution = candidateSolutions[0];
  }

  _simulatedAnnealingSolve() {
    let updatedLastSolution = null;
    const lastSolution = this.lastSolution;
    if (lastSolution) {
      this._applySolution(lastSolution.solution);

      updatedLastSolution = {
        solution: lastSolution.solution,
        cost: this._getCost(lastSolution.positions, lastSolution.solution),
        positions: lastSolution.positions,
      };
    }
    const annealed = this._annealSolution(this._generateValidRandomSolution());
    this._applySolution(annealed.solution);

    if (updatedLastSolution && updatedLastSolution.cost < annealed.cost) {
      this._applySolution(updatedLastSolution.solution);
      this.lastSolution = updatedLastSolution;
    } else {
      this.lastSolution = annealed;
    }
  }

  _annealSolution(sol: SolutionAndCost): SolutionAndCost {
    let oldSol = sol;
    let T = 1.0;
    const T_min = 0.00001;
    const alpha = 0.9;
    while (T > T_min) {
      for (let i = 1; i < 50; i++) {
        let newSol = this._generateNeighborSolution(oldSol);
        let ap = acceptanceProbability(oldSol.cost, newSol.cost, T);
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
      this._applySolution(solution);
    } while (!this._isValidIKSolution());
    const positions = this._getPositions();
    const cost = this._getCost(positions, solution);
    return {
      solution,
      cost,
      positions,
    };
  }

  _getCost(positions: Array<THREE.Vector3>, solution: Array<number>) {
    const lastSolution = this.lastSolution;
    const lastPosImportance = 0.5;
    return (
      getWorldPos(last(this.ikNodes)).distanceTo(this.targetProxy.position) +
      (lastSolution
        ? solution.reduce(
            (acc, v, i) => acc + (solution[i] - lastSolution.solution[i]),
            0
          ) /
            solution.length +
          (positions.reduce(
            (acc, v, i) => acc + v.distanceTo(lastSolution.positions[i]),
            0
          ) /
            positions.length) *
            lastPosImportance
        : 0)
    );
  }

  _getPositions() {
    const positions = [];
    for (var i = 0; i < this.ikNodes.length; i++) {
      positions.push(getWorldPos(this.ikNodes[i]));
    }
    return positions;
  }

  _generateNeighbourSolution(prev: Array<number>) {
    const i = Math.floor(this.ikNodes.length * Math.random());
    const solution: Array<number> = prev.slice(0);

    solution[i] =
      this.ikNodes[i].minJointAngle +
      Math.random() *
        (this.ikNodes[i].maxJointAngle - this.ikNodes[i].minJointAngle);

    return solution;
  }

  _generateValidRandomSolution() {
    let solution;
    do {
      solution = this._generateRandomSolution();
      this._applySolution(solution);
    } while (!this._isValidIKSolution());
    const positions = this._getPositions();
    const cost = this._getCost(positions, solution);
    return {
      solution,
      cost,
      positions,
    };
  }

  _generateRandomSolution() {
    const solution = [];
    for (var i = 0; i < this.ikNodes.length; i++) {
      solution.push(
        this.ikNodes[i].minJointAngle +
          Math.random() *
            (this.ikNodes[i].maxJointAngle - this.ikNodes[i].minJointAngle)
      );
    }
    return solution;
  }

  _generateValidSolutionInDistance(prev: Array<number>, distance: number) {
    let solution;
    do {
      solution = this._generateSolutionInDistance(prev, distance);
      this._applySolution(solution);
    } while (!this._isValidIKSolution());
    const positions = this._getPositions();
    const cost = this._getCost(positions, solution);
    return {
      solution,
      cost,
      positions,
    };
  }

  _generateSolutionInDistance(prev: Array<number>, distance: number) {
    const solution = [];
    for (var i = 0; i < this.ikNodes.length; i++) {
      const range =
        (this.ikNodes[i].maxJointAngle - this.ikNodes[i].minJointAngle) *
        distance;
      solution.push(
        clamp(
          // TODO: clamp range before applying random
          prev[i] + Math.random() * range,
          this.ikNodes[i].minJointAngle,
          this.ikNodes[i].maxJointAngle
        )
      );
    }
    return solution;
  }

  _applySolution(solution: Array<number>) {
    for (var i = 0; i < this.ikNodes.length; i++) {
      this.ikNodes[i].setJointAngle(solution[i]);
    }
  }

  _isValidIKSolution() {
    for (var i = 0; i < this.ikNodes.length; i++) {
      const node = this.ikNodes[i];
      const pos = getWorldPos(node);
      if (pos.y < 0) {
        return false;
      }
    }
    return true;
  }

  addJoint(x: number, y: number, z: number) {
    const ikNode = makeNode(
      new VectorR3(x, y, z), // startPos
      VectorR3_UnitZ(), // rotationAxis
      JOINT, // purpose (joint or effector)
      degToRad(-90), // minJointAngle in radians
      degToRad(270) // maxJointAngle in radians
    );

    this._insertIKNode(ikNode);
  }

  _insertIKNode(ikNode: Node) {
    if (this.ikNodes.length === 0) {
      this.ikTree = ikNode;
    } else {
      const tail = this.ikNodes[this.ikNodes.length - 1];
      tail.add(ikNode);
      tail.childNode = ikNode;
      ikNode.parentNode = tail;
    }
    this.ikNodes.push(ikNode);
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
      const point = makeBox(0.1, Math.random() * 0xcccccc);
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
      const node = this.ikNodes[i];
      const pos = getWorldPos(node);

      this.debugPoints[i].position.set(pos.x, pos.y, pos.z);
      this.lineGeometry.vertices[i].set(pos.x, pos.y, pos.z);
    }
    this.lineGeometry.verticesNeedUpdate = true;
  }
}
