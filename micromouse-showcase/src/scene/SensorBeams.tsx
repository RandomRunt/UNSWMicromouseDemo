import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useRef } from 'react';
import * as THREE from 'three';

interface SensorBeamsProps {
  activeChapter: number;
  reducedMotion: boolean;
  assetAvailable: boolean;
}

// Beam tuning. I keep these deliberately short so they read as local ToF
// measurements rather than spotlights crossing the whole scene.
const BEAM_LENGTH = 0.9;
// This pushes the narrow end just beyond the sensor face. Increase it slightly
// if a future GLB export makes the beam start inside the ToF sensor casing.
const BEAM_ORIGIN_NUDGE = 0.03;
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
  tof_front: new THREE.Vector3(0, 0, 1),
  // The side sensors are also reversed so all three beams point outwards.
  tof_left: new THREE.Vector3(1, 0, 0),
  tof_right: new THREE.Vector3(-1, 0, 0),
};
const PROCEDURAL_DIRECTIONS: Record<SensorName, THREE.Vector3> = {
  tof_front: new THREE.Vector3(0, 0, -1),
  tof_left: new THREE.Vector3(-1, 0, 0),
  tof_right: new THREE.Vector3(1, 0, 0),
};

export function SensorBeams({ activeChapter, reducedMotion, assetAvailable }: SensorBeamsProps) {
  const { scene } = useThree();
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
  }>({ modelRoot: null, tof_front: null, tof_left: null, tof_right: null });
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
    };
  }, [assetAvailable]);

  useFrame((state, delta) => {
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

      sensor.getWorldPosition(scratch.current.anchor);
      scratch.current.direction
        .copy(directions[componentName])
        .applyQuaternion(scratch.current.modelRotation)
        .normalize();
      const length = BEAM_LENGTH * reveal.current;
      scratch.current.anchor.addScaledVector(scratch.current.direction, BEAM_ORIGIN_NUDGE);
      scratch.current.midpoint
        .copy(scratch.current.anchor)
        .addScaledVector(scratch.current.direction, length / 2);
      beam.position.copy(scratch.current.midpoint);
      beam.quaternion.setFromUnitVectors(BEAM_UP, scratch.current.direction);
      beam.scale.set(1, length, 1);
    };

    placeBeam(frontBeam.current, 'tof_front');
    placeBeam(leftBeam.current, 'tof_left');
    placeBeam(rightBeam.current, 'tof_right');

    if (!reducedMotion) {
      group.current.children.forEach((child, index) => {
        const material = (child as THREE.Mesh).material as THREE.MeshBasicMaterial;
        // 0.28 is average opacity, 3 is pulse speed, and 0.12 is pulse strength.
        if (material) material.opacity = 0.28 + Math.sin(state.clock.elapsedTime * 3 + index) * 0.12;
      });
    }
  });

  return (
    <group ref={group} visible={activeChapter === 2}>
      <mesh ref={frontBeam}>
        {/* Top is the far end and bottom sits at the sensor, so the beam widens outwards. */}
        <cylinderGeometry args={[0.075, 0.018, 1, 8]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={0.38} depthWrite={false} />
      </mesh>
      <mesh ref={leftBeam}>
        <cylinderGeometry args={[0.07, 0.018, 1, 8]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={0.32} depthWrite={false} />
      </mesh>
      <mesh ref={rightBeam}>
        <cylinderGeometry args={[0.07, 0.018, 1, 8]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={0.32} depthWrite={false} />
      </mesh>
    </group>
  );
}
