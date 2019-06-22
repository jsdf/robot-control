// @flow
import IKRobot from './IKRobot';
import AnnealingRobot from './AnnealingRobot';
import renderControls from './Controls';
const THREE = require('three');
const OrbitControls = require('three-orbit-controls')(THREE);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(
  45,
  window.innerWidth / window.innerHeight,
  1,
  10000
);

camera.position.z = 5;

camera.position.set(0, 20, 10);

const renderer = new THREE.WebGLRenderer();
renderer.setClearColor(0xeeeeee, 1);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body && document.body.appendChild(renderer.domElement);

const debugTextEl = document.createElement('pre');
Object.assign(debugTextEl.style, {
  pointerEvents: 'none',
  position: 'absolute',
  top: '0',
  left: '0',
});

document.body && document.body.appendChild(debugTextEl);

const debugAtPosEl = document.createElement('div');
Object.assign(debugAtPosEl.style, {
  pointerEvents: 'none',
  position: 'absolute',
  top: '0',
  left: '0',
  position: 'absolute',
  fontFamily: 'monospace',
  color: 'red',
});
document.body && document.body.appendChild(debugAtPosEl);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.update();

const Robot = window.location.search.slice(1).includes('annealing')
  ? AnnealingRobot
  : IKRobot;

function toScreenPosition(obj, camera, renderer) {
  var vector = new THREE.Vector3();

  var widthHalf = 0.5 * renderer.context.canvas.width;
  var heightHalf = 0.5 * renderer.context.canvas.height;

  obj.updateMatrixWorld(false);
  vector.setFromMatrixPosition(obj.matrixWorld);
  vector.project(camera);

  vector.x = vector.x * widthHalf + widthHalf;
  vector.y = -(vector.y * heightHalf) + heightHalf;

  return {
    x: vector.x,
    y: vector.y,
  };
}

function createOrUpdatePositionedDebugTextNode(item, i) {
  let node = debugAtPosEl.childNodes[i];
  if (node == null) {
    node = document.createElement('div');
    Object.assign(node.style, {
      position: 'absolute',
      whiteSpace: 'pre',
    });
    debugAtPosEl.appendChild(node);
  }

  if (!(node instanceof HTMLDivElement)) {
    throw new Error('expected div');
  }

  Object.assign(node.style, {
    left: `${item.screenPos.x}px`,
    top: `${item.screenPos.y}px`,
  });

  node.textContent = item.text;
}

const reactRoot = document.createElement('div');
document.body && document.body.appendChild(reactRoot);

const debug = {log: 'hi', positionedDebugText: []};
const robot = new Robot(
  scene,
  camera,
  renderer,
  (msg: string) => {
    debug.log += msg;
  },
  (text: string, obj: THREE.Object3D) => {
    const screenPos = toScreenPosition(obj, camera, renderer);
    debug.positionedDebugText.push({text, screenPos});
  }
);

if (robot instanceof IKRobot) {
  renderControls(reactRoot, robot);
}

function animate(timestamp: number) {
  orbitControls.update();
  robot.update();
  renderer.render(scene, camera);
  debugTextEl.textContent = debug.log;
  debug.log = '';
  debug.positionedDebugText.forEach(createOrUpdatePositionedDebugTextNode);
  debug.positionedDebugText = [];
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize(window.innerWidth, window.innerHeight);
});

window.demo = {
  renderer,
  camera,
  scene,
  robot,
};
