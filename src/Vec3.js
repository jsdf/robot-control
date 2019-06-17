// @flow

export type Vec3Interface = {
  x: number,
  y: number,
  z: number,
};

function lerpScalar(v0: number, v1: number, t: number): number {
  return v0 * (1 - t) + v1 * t;
}

export function distanceTo(source: Vec3Interface, dest: Vec3Interface) {
  const dx = source.x - dest.x;
  const dy = source.y - dest.y;
  const dz = source.z - dest.z;

  const distanceToSquared = dx * dx + dy * dy + dz * dz;
  return Math.sqrt(distanceToSquared);
}

export function clone(v: Vec3Interface) {
  return {
    x: v.x,
    y: v.y,
    z: v.z,
  };
}

export function sub(v1: Vec3Interface, v2: Vec3Interface) {
  v1.x -= v2.x;
  v1.y -= v2.y;
  v1.z -= v2.z;

  return v1;
}

export function add(v1: Vec3Interface, v2: Vec3Interface) {
  v1.x += v2.x;
  v1.y += v2.y;
  v1.z += v2.z;

  return v1;
}

export function mulScalar(v: Vec3Interface, scalar: number) {
  v.x *= scalar;
  v.y *= scalar;
  v.z *= scalar;

  return v;
}

export function normalise(v: Vec3Interface) {
  if (v.x === 0 && v.y === 0 && v.z === 0) return v;

  const magnitude = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  v.x /= magnitude;
  v.y /= magnitude;
  v.z /= magnitude;
  return v;
}

export function directionTo(v1: Vec3Interface, v2: Vec3Interface) {
  return normalise(sub(clone(v2), v1));
}

export function lerp(v1: Vec3Interface, v2: Vec3Interface, t: number) {
  return {
    x: lerpScalar(v1.x, v2.x, t),
    y: lerpScalar(v1.y, v2.y, t),
    z: lerpScalar(v1.z, v2.z, t),
  };
}

export function copyFrom(v1: Vec3Interface, v2: Vec3Interface) {
  v1.x = v2.x;
  v1.y = v2.y;
  v1.z = v2.z;

  return v1;
}
