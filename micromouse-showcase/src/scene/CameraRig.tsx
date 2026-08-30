import { OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useMemo } from 'react';
import * as THREE from 'three';

interface CameraRigProps {
  progress: number;
  explorationEnabled: boolean;
  reducedMotion: boolean;
}

const cameraKeyframes: Array<[number, number, number]> = [
  [4.5, 2.8, 5.4],
  [4.2, 3.4, 4.5],
  [3.7, 2.25, 4.1],
  [4.7, 3.7, 5.8],
  [5.4, 4.6, 6.8],
  [4.8, 3.2, 5.2],
];

export function CameraRig({ progress, explorationEnabled, reducedMotion }: CameraRigProps) {
  const { camera } = useThree();
  const target = useMemo(() => new THREE.Vector3(0, 0.35, 0), []);
  const desired = useMemo(() => new THREE.Vector3(), []);

  useFrame((_, delta) => {
    if (explorationEnabled) return;
    const chapterProgress = progress * cameraKeyframes.length;
    const index = Math.min(cameraKeyframes.length - 1, Math.floor(chapterProgress));
    const nextIndex = Math.min(cameraKeyframes.length - 1, index + 1);
    const local = reducedMotion ? 0 : chapterProgress - index;
    const from = cameraKeyframes[index];
    const to = cameraKeyframes[nextIndex];
    desired.set(
      THREE.MathUtils.lerp(from[0], to[0], local),
      THREE.MathUtils.lerp(from[1], to[1], local),
      THREE.MathUtils.lerp(from[2], to[2], local),
    );
    camera.position.lerp(desired, 1 - Math.exp(-delta * (reducedMotion ? 18 : 3.8)));
    camera.lookAt(target);
  });

  return (
    <OrbitControls
      enabled={explorationEnabled}
      enablePan={false}
      minDistance={3.2}
      maxDistance={9}
      minPolarAngle={0.45}
      maxPolarAngle={1.45}
      target={[0, 0.35, 0]}
      makeDefault
    />
  );
}
