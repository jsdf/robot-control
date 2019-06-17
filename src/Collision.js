// @flow
import type {Vec3Interface} from './Vec3';
import * as Vec3 from './Vec3';

const SPHERE_RADIUS = 0.1;
const SPHERE_INTERVAL = SPHERE_RADIUS / 4;
const GAP = 0.001;

type CollisionVolume = {
  isColliding: boolean,
  center: Vec3Interface,
  radius: number,
  distanceAlongArmSegment: number,
};

type ArmSegment = {
  indexRange: [number, number],
  start: Vec3Interface,
  end: Vec3Interface,
  volumes: Array<CollisionVolume>,
  span: number,
};

export default class Collision {
  armSegments: Array<ArmSegment>;
  volumes: Array<CollisionVolume> = [];
  constructor(arm: Array<Vec3Interface>) {
    this.makeCollisionVolumes(arm);
    this._updateCollisions();
  }

  makeCollisionVolumes(arm: Array<Vec3Interface>) {
    const allVolumes = [];
    const armSegments = [];
    let prev: Vec3Interface = arm[0];
    for (let i = 1; i < arm.length; i++) {
      const current = arm[i];
      const start = prev;
      const end = current;
      const span = Vec3.distanceTo(start, end);

      // offset spheres from ends by radius
      const startWithGap = Vec3.add(
        Vec3.clone(prev),
        // move towards current by radius
        Vec3.mulScalar(Vec3.directionTo(prev, current), span * GAP)
      );
      const endWithGap = Vec3.add(
        Vec3.clone(current),
        // move towards prev by radius
        Vec3.mulScalar(Vec3.directionTo(current, prev), span * GAP)
      );

      const armSegment = {
        indexRange: [i - 1, i],
        start,
        end,
        volumes: [],
        span,
      };

      const spanWithGap = Vec3.distanceTo(startWithGap, endWithGap);
      const spheresCount = Math.floor(spanWithGap / SPHERE_INTERVAL);
      for (let k = 0; k < spheresCount; k++) {
        const center = Vec3.lerp(startWithGap, endWithGap, k / spheresCount);

        const distanceAlongArmSegment =
          Vec3.distanceTo(armSegment.start, center) / armSegment.span;

        // 0.2 => -0.3
        const radiusScaling = 1 - Math.abs(distanceAlongArmSegment - 0.5) * 2;

        const volume = {
          isColliding: false,
          center,
          radius: SPHERE_RADIUS * radiusScaling,
          // used as 't' for interpolating along arm
          distanceAlongArmSegment,
        };
        armSegment.volumes.push(volume);
        allVolumes.push(volume);
      }
      armSegments.push(armSegment);

      prev = current;
    }
    this.armSegments = armSegments;
    this.volumes = allVolumes;
  }

  update() {
    this._updatePositions();
    this._updateCollisions();
  }

  areAnyColliding() {
    return this.volumes.some(v => v.isColliding);
  }

  _updatePositions() {
    for (let i = 0; i < this.armSegments.length; i++) {
      const armSegment = this.armSegments[i];
      for (var k = 0; k < armSegment.volumes.length; k++) {
        const volume = armSegment.volumes[k];

        Vec3.copyFrom(
          volume.center,
          Vec3.lerp(
            armSegment.start,
            armSegment.end,
            volume.distanceAlongArmSegment
          )
        );
      }
    }
  }

  _updateSegmentCollisions(segA: ArmSegment, segB: ArmSegment) {
    for (let a = 0; a < segA.volumes.length; a++) {
      const volA = segA.volumes[a];

      for (let b = 0; b < segB.volumes.length; b++) {
        const volB = segB.volumes[b];

        if (
          volA != volB &&
          Vec3.distanceTo(volA.center, volB.center) < volA.radius + volB.radius
        ) {
          // collision
          volA.isColliding = volA.isColliding || true;
          volB.isColliding = volB.isColliding || true;
        }
      }
    }
  }

  _updateCollisions() {
    this.volumes.forEach(vol => {
      vol.isColliding = false;
    });
    const armSegments = this.armSegments;
    for (let a = 0; a < armSegments.length; a++) {
      const segA = armSegments[a];

      for (let b = 0; b < armSegments.length; b++) {
        const segB = armSegments[b];
        if (
          !(
            segA === segB ||
            // adjacent segments
            segA.indexRange[0] === segB.indexRange[1] ||
            segB.indexRange[0] === segA.indexRange[1]
          )
        ) {
          this._updateSegmentCollisions(segA, segB);
        }
      }
    }
  }
}
