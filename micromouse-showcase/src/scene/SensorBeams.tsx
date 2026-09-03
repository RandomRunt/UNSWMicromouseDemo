import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { getResponsiveRobotScale } from '../config/responsive';

interface SensorBeamsProps {
  activeChapter: number;
  reducedMotion: boolean;
  assetAvailable: boolean;
}

// Beam tuning. These stay deliberately short and narrow so they read as local
// ToF measurements rather than spotlights crossing the whole scene.
const BEAM_LENGTH = 0.9;
export const SENSOR_BEAM_OPACITY = 0.5;
// This pushes the cylinder just beyond the sensor face. Increase it slightly
// if a future GLB export makes the beam start inside the ToF sensor casing.
const BEAM_ORIGIN_NUDGE = 0.35;
const BEAM_UP = new THREE.Vector3(0, 1, 0);
type SensorName = 'tof_front' | 'tof_left' | 'tof_right';

// Direction vectors use [X, Y, Z]. For my current GLB orientation:
//   X: -1 points left,  +1 points right
//   Y: -1 points down,  +1 points up
//   Z: -1 points away from the opening camera, +1 points towards it
// To flip one beam, I multiply all three numbers in that row by -1.
// These directions are kept at length 1 because BEAM_LENGTH controls reach.
const GLB_DIRECTIONS: Record<SensorName, THREE.Vector3> = {
  // Front now projects away from the viewer through the front sensor.
  tof_front: new THREE.Vector3(-0.1, 0, 1),
  // The side sensors are also reversed so all three beams point outwards.
  tof_left: new THREE.Vector3(1, 0, 0.1),
  tof_right: new THREE.Vector3(-1, 0, 0.1),
};
const PROCEDURAL_DIRECTIONS: Record<SensorName, THREE.Vector3> = {
  tof_front: new THREE.Vector3(0, 0, -1),
  tof_left: new THREE.Vector3(-1, 0, 0),
  tof_right: new THREE.Vector3(1, 0, 0),
};

function findEmitterAnchor(
  sensor: THREE.Object3D,
  componentName: SensorName,
  assetAvailable: boolean,
) {
  const namedEmitter = sensor.getObjectByName(`${componentName}_emitter`);
  if (namedEmitter) return namedEmitter;
  if (!assetAvailable) return sensor;

  // The imported CAD hierarchy keeps each black VL6180X package in a named
  // wrapper. Its origin is a much closer beam anchor than the whole ToF board.
  let importedEmitter: THREE.Object3D | null = null;
  sensor.traverse((object) => {
    if (!importedEmitter && /^VL6180X.*:1$/.test(object.name)) {
      importedEmitter = object;
    }
  });
  return importedEmitter ?? sensor;
}

export function SensorBeams({ activeChapter, assetAvailable }: SensorBeamsProps) {
  const { scene, size } = useThree();
  const responsiveScale = getResponsiveRobotScale(size.width);
  const group = useRef<THREE.Group>(null);
  const frontBeam = useRef<THREE.Mesh>(null);
  const leftBeam = useRef<THREE.Mesh>(null);
  const rightBeam = useRef<THREE.Mesh>(null);
  const reveal = useRef(0.001);
  const trackedObjects = useRef<{
    modelRoot: THREE.Object3D | null;
    tof_front: THREE.Object3D | null;
    tof_left: THREE.Object3D | null;
    tof_right: THREE.Object3D | null;
    emitters: Record<SensorName, THREE.Object3D | null>;
  }>({
    modelRoot: null,
    tof_front: null,
    tof_left: null,
    tof_right: null,
    emitters: { tof_front: null, tof_left: null, tof_right: null },
  });
  const scratch = useRef({
    anchor: new THREE.Vector3(),
    direction: new THREE.Vector3(),
    midpoint: new THREE.Vector3(),
    modelRotation: new THREE.Quaternion(),
  });

  useEffect(() => {
    trackedObjects.current = {
      modelRoot: null,
      tof_front: null,
      tof_left: null,
      tof_right: null,
      emitters: { tof_front: null, tof_left: null, tof_right: null },
    };
  }, [assetAvailable]);

  useFrame((_, delta) => {
    if (!group.current) return;
    const targetReveal = activeChapter === 2 ? 1 : 0.001;
    // Increase 7 for a snappier beam reveal, or reduce it for a slower fade-in.
    reveal.current = THREE.MathUtils.damp(reveal.current, targetReveal, 7, delta);
    group.current.visible = activeChapter === 2 || reveal.current > 0.01;

    const modelRoot = trackedObjects.current.modelRoot
      ?? scene.getObjectByName(assetAvailable ? 'showcase_model_root' : 'mouse_root');
    if (!modelRoot) return;
    trackedObjects.current.modelRoot = modelRoot;
    modelRoot.getWorldQuaternion(scratch.current.modelRotation);

    const directions = assetAvailable ? GLB_DIRECTIONS : PROCEDURAL_DIRECTIONS;

    const placeBeam = (
      beam: THREE.Mesh | null,
      componentName: SensorName,
    ) => {
      if (!beam) return;
      const sensor = trackedObjects.current[componentName]
        ?? scene.getObjectByName(componentName);
      if (!sensor) return;
      trackedObjects.current[componentName] = sensor;

      const emitter = trackedObjects.current.emitters[componentName]
        ?? findEmitterAnchor(sensor, componentName, assetAvailable);
      trackedObjects.current.emitters[componentName] = emitter;
      emitter.getWorldPosition(scratch.current.anchor);
      scratch.current.direction
        .copy(directions[componentName])
        .applyQuaternion(scratch.current.modelRotation)
        .normalize();
      // The emitter's world position already includes the robot's responsive
      // scale. Scale only the beam's local tuning values to keep it attached.
      const length = BEAM_LENGTH * responsiveScale * reveal.current;
      scratch.current.anchor.addScaledVector(
        scratch.current.direction,
        BEAM_ORIGIN_NUDGE * responsiveScale,
      );
      scratch.current.midpoint
        .copy(scratch.current.anchor)
        .addScaledVector(scratch.current.direction, length / 2);
      beam.position.copy(scratch.current.midpoint);
      beam.quaternion.setFromUnitVectors(BEAM_UP, scratch.current.direction);
      beam.scale.set(responsiveScale, length, responsiveScale);
    };

    placeBeam(frontBeam.current, 'tof_front');
    placeBeam(leftBeam.current, 'tof_left');
    placeBeam(rightBeam.current, 'tof_right');
  });

  return (
    <group ref={group} visible={activeChapter === 2}>
      <mesh ref={frontBeam}>
        <cylinderGeometry args={[0.01, 0.01, 1, 12]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={SENSOR_BEAM_OPACITY} depthWrite={false} />
      </mesh>
      <mesh ref={leftBeam}>
        <cylinderGeometry args={[0.01, 0.01, 1, 12]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={SENSOR_BEAM_OPACITY} depthWrite={false} />
      </mesh>
      <mesh ref={rightBeam}>
        <cylinderGeometry args={[0.01, 0.01, 1, 12]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={SENSOR_BEAM_OPACITY} depthWrite={false} />
      </mesh>
    </group>
  );
}
