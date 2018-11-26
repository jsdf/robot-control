// @flow
import Robot from './Robot';
const THREE = require('three');
const OrbitControls = require('three-orbit-controls')(THREE);
const TransformControls = require('three-transform-controls')(THREE);

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
debugTextEl.id = 'debug';
document.body && document.body.appendChild(debugTextEl);

const transformControls = new TransformControls(camera, renderer.domElement);
const orbitControls = new OrbitControls(camera);
orbitControls.update();
scene.add(transformControls);

const debug = {log: 'hi'};
const robot = new Robot(scene, transformControls, (msg: string) => {
  debug.log += msg;
});

function animate(timestamp: number) {
  orbitControls.update();
  robot.update();
  renderer.render(scene, camera);
  debugTextEl.textContent = debug.log;
  debug.log = '';
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

window.demo = {
  renderer,
  camera,
  scene,
};
