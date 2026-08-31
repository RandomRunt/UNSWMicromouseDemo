import { OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useCallback, useMemo, useRef } from 'react';
import * as THREE from 'three';

interface CameraRigProps {
  progress: number;
  explorationEnabled: boolean;
  onInspectionInput: () => void;
  reducedMotion: boolean;
}

// One [X, Y, Z] camera position per story chapter. I adjust these when a
// chapter needs a clearer view: X moves sideways, Y changes height, and Z
// changes front/back distance. Larger overall values usually zoom out.
const cameraKeyframes: Array<[number, number, number]> = [
  [4.5, 2.8, 5.4],
  [4.2, 3.4, 4.5],
  [3.7, 2.25, 4.1],
  [4.7, 3.7, 5.8],
  [5.4, 4.6, 6.8],
  [4.8, 3.2, 5.2],
];

export function CameraRig({
  progress,
  explorationEnabled,
  onInspectionInput,
  reducedMotion,
}: CameraRigProps) {
  const { camera } = useThree();
  // All guided camera views look at this point. Raising Y aims higher on the robot.
  const target = useMemo(() => new THREE.Vector3(0, 0.35, 0), []);
  const desired = useMemo(() => new THREE.Vector3(), []);
  const isUsingControlsRef = useRef(false);

  const handleControlStart = useCallback(() => {
    if (!explorationEnabled) return;
    isUsingControlsRef.current = true;
    onInspectionInput();
  }, [explorationEnabled, onInspectionInput]);

  const handleControlChange = useCallback(() => {
    if (isUsingControlsRef.current) onInspectionInput();
  }, [onInspectionInput]);

  const handleControlEnd = useCallback(() => {
    if (!isUsingControlsRef.current) return;
    isUsingControlsRef.current = false;
    onInspectionInput();
  }, [onInspectionInput]);

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
    // 3.8 controls guided camera transition speed; a larger value settles faster.
    camera.position.lerp(desired, 1 - Math.exp(-delta * (reducedMotion ? 18 : 3.8)));
    camera.lookAt(target);
  });

  return (
    <OrbitControls
      enabled={explorationEnabled}
      enablePan={false}
      // These are the closest/furthest zoom distances allowed in free inspection.
      minDistance={2}
      maxDistance={9}
      // Polar angles limit how far visitors can orbit over or under the robot.
      minPolarAngle={0.15}
      maxPolarAngle={2.75}
      // Keep this close to the guided target above so the handover feels natural.
      target={[0, 0.2, 0]}
      onStart={handleControlStart}
      onChange={handleControlChange}
      onEnd={handleControlEnd}
      makeDefault
    />
  );
}
