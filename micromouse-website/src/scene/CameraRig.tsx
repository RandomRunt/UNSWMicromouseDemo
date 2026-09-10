import { OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useCallback, useEffect, useMemo, useRef, type ElementRef } from 'react';
import * as THREE from 'three';

interface CameraRigProps {
  progress: number;
  explorationEnabled: boolean;
  onInspectionInput: () => void;
  onOrbitInteraction: () => void;
  reducedMotion: boolean;
  viewResetKey: number;
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

const DESKTOP_SENSE_CAMERA: [number, number, number] = [4.4, 2.7, 4.9];
const DESKTOP_BREAKPOINT = 960;
const GUIDED_TARGET_Y = 0.35;
const DESKTOP_SENSE_TARGET_Y = 0.6;
const RESET_VIEW_DURATION_SECONDS = 0.6;

export function CameraRig({
  progress,
  explorationEnabled,
  onInspectionInput,
  onOrbitInteraction,
  reducedMotion,
  viewResetKey,
}: CameraRigProps) {
  const { camera, scene, size } = useThree();
  // All guided camera views look at this point. Raising Y aims higher on the robot.
  const target = useMemo(() => new THREE.Vector3(0, GUIDED_TARGET_Y, 0), []);
  // Match the guided camera's target exactly so enabling OrbitControls does
  // not re-aim the camera and make the robot jump vertically on screen.
  const controlsTarget = useMemo(() => new THREE.Vector3(0, GUIDED_TARGET_Y, 0), []);
  const desired = useMemo(() => new THREE.Vector3(), []);
  const isUsingControlsRef = useRef(false);
  const hasReportedInteractionRef = useRef(false);
  const interactionStartOffsetRef = useRef(new THREE.Vector3());
  const currentOffsetRef = useRef(new THREE.Vector3());
  const controlsRef = useRef<ElementRef<typeof OrbitControls>>(null);
  const isResettingViewRef = useRef(false);
  const resetElapsedRef = useRef(0);
  const resetStartPositionRef = useRef(new THREE.Vector3());
  const resetStartTargetRef = useRef(new THREE.Vector3());
  const wasExplorationEnabledRef = useRef(explorationEnabled);
  const resetPosition = useMemo(
    () => new THREE.Vector3(...cameraKeyframes[cameraKeyframes.length - 1]),
    [],
  );
  const resetTarget = useMemo(() => new THREE.Vector3(0, GUIDED_TARGET_Y, 0), []);

  useEffect(() => {
    const orbitTarget = controlsRef.current?.target ?? controlsTarget;

    if (!explorationEnabled) {
      if (wasExplorationEnabledRef.current) {
        resetTarget.set(0, GUIDED_TARGET_Y, 0);
        resetPosition.set(...cameraKeyframes[cameraKeyframes.length - 1]);
        resetStartPositionRef.current.copy(camera.position);
        resetStartTargetRef.current.copy(orbitTarget);
        resetElapsedRef.current = 0;
        isResettingViewRef.current = true;
      }
      wasExplorationEnabledRef.current = false;
      return;
    }

    wasExplorationEnabledRef.current = true;

    const model = scene.getObjectByName('showcase_model_root')
      ?? scene.getObjectByName('mouse_root');
    if (!model) return;

    const modelCenter = new THREE.Box3().setFromObject(model).getCenter(new THREE.Vector3());
    const targetOffset = modelCenter.sub(new THREE.Vector3(0, GUIDED_TARGET_Y, 0));
    resetTarget.set(0, GUIDED_TARGET_Y, 0).add(targetOffset);
    resetPosition
      .set(...cameraKeyframes[cameraKeyframes.length - 1])
      .add(targetOffset);
    resetStartPositionRef.current.copy(camera.position);
    resetStartTargetRef.current.copy(orbitTarget);
    resetElapsedRef.current = 0;
    isResettingViewRef.current = true;
  }, [camera, controlsTarget, explorationEnabled, resetPosition, resetTarget, scene]);

  useEffect(() => {
    if (viewResetKey === 0) return;

    resetStartPositionRef.current.copy(camera.position);
    resetStartTargetRef.current.copy(controlsRef.current?.target ?? controlsTarget);
    resetElapsedRef.current = 0;
    isResettingViewRef.current = true;
  }, [camera, controlsTarget, viewResetKey]);

  const handleControlStart = useCallback(() => {
    if (!explorationEnabled) return;
    isResettingViewRef.current = false;
    isUsingControlsRef.current = true;
    hasReportedInteractionRef.current = false;
    interactionStartOffsetRef.current
      .copy(camera.position)
      .sub(controlsRef.current?.target ?? controlsTarget);
    onInspectionInput();
  }, [camera, controlsTarget, explorationEnabled, onInspectionInput]);

  const handleControlChange = useCallback(() => {
    if (!isUsingControlsRef.current) return;
    onInspectionInput();

    if (hasReportedInteractionRef.current) return;

    const startOffset = interactionStartOffsetRef.current;
    const currentOffset = currentOffsetRef.current
      .copy(camera.position)
      .sub(controlsRef.current?.target ?? controlsTarget);
    const startDistance = startOffset.length();
    const currentDistance = currentOffset.length();
    const zoomedIn = currentDistance < startDistance - 0.01;
    const dragged = startDistance > 0
      && currentDistance > 0
      && startOffset.angleTo(currentOffset) > 0.002;

    if (dragged || zoomedIn) {
      hasReportedInteractionRef.current = true;
      onOrbitInteraction();
    }
  }, [camera, controlsTarget, onInspectionInput, onOrbitInteraction]);

  const handleControlEnd = useCallback(() => {
    if (!isUsingControlsRef.current) return;
    isUsingControlsRef.current = false;
    onInspectionInput();
  }, [onInspectionInput]);

  useFrame((_, delta) => {
    if (isResettingViewRef.current) {
      const orbitTarget = controlsRef.current?.target ?? controlsTarget;
      resetElapsedRef.current += delta;
      const progress = Math.min(1, resetElapsedRef.current / RESET_VIEW_DURATION_SECONDS);
      // Smootherstep starts and finishes at zero velocity, avoiding a visible jump.
      const easing = progress * progress * progress * (
        progress * (progress * 6 - 15) + 10
      );
      camera.position.lerpVectors(resetStartPositionRef.current, resetPosition, easing);
      orbitTarget.lerpVectors(resetStartTargetRef.current, resetTarget, easing);
      controlsRef.current?.update();

      if (progress >= 1) {
        camera.position.copy(resetPosition);
        orbitTarget.copy(resetTarget);
        controlsRef.current?.update();
        isResettingViewRef.current = false;
      }
      return;
    }

    if (explorationEnabled) return;
    const chapterProgress = progress * cameraKeyframes.length;
    const index = Math.min(cameraKeyframes.length - 1, Math.floor(chapterProgress));
    const nextIndex = Math.min(cameraKeyframes.length - 1, index + 1);
    const local = reducedMotion ? 0 : chapterProgress - index;
    const desktopView = size.width > DESKTOP_BREAKPOINT;
    const from = desktopView && index === 2 ? DESKTOP_SENSE_CAMERA : cameraKeyframes[index];
    const to = desktopView && nextIndex === 2 ? DESKTOP_SENSE_CAMERA : cameraKeyframes[nextIndex];
    desired.set(
      THREE.MathUtils.lerp(from[0], to[0], local),
      THREE.MathUtils.lerp(from[1], to[1], local),
      THREE.MathUtils.lerp(from[2], to[2], local),
    );
    const fromTargetY = desktopView && index === 2 ? DESKTOP_SENSE_TARGET_Y : GUIDED_TARGET_Y;
    const toTargetY = desktopView && nextIndex === 2 ? DESKTOP_SENSE_TARGET_Y : GUIDED_TARGET_Y;
    target.y = THREE.MathUtils.lerp(fromTargetY, toTargetY, local);
    // 3.8 controls guided camera transition speed; a larger value settles faster.
    camera.position.lerp(desired, 1 - Math.exp(-delta * (reducedMotion ? 18 : 3.8)));
    camera.lookAt(target);
  });

  return (
    <OrbitControls
      ref={controlsRef}
      enabled={explorationEnabled}
      enablePan
      // Keep wheel/pinch zoom close enough that the robot remains the focus.
      maxDistance={9}
      // Keep this close to the guided target above so the handover feels natural.
      target={controlsTarget}
      onStart={handleControlStart}
      onChange={handleControlChange}
      onEnd={handleControlEnd}
      makeDefault
    />
  );
}
