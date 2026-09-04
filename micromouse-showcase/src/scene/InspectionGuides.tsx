import { Html } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { getResponsiveRobotScale } from '../config/responsive';
import { SENSOR_BEAM_OPACITY } from './SensorBeams';

interface InspectionGuidesProps {
  activeChapter: number;
  assetAvailable: boolean;
}

const AXIS_LENGTH = 0.28;
const ROTATION_RADIUS = 0.07;

interface AxisArrowProps {
  direction: 'x' | 'y' | 'z';
  label: 'x' | 'y' | 'z';
  active: boolean;
}

const AXIS_COLORS: Record<AxisArrowProps['label'], string> = {
  x: '#ff645f',
  y: '#76e56d',
  z: '#5caeff',
};

function AxisRotationArrow({ direction }: Pick<AxisArrowProps, 'direction'>) {
  const position: [number, number, number] = direction === 'x'
    ? [AXIS_LENGTH * 0.36, 0, 0]
    : direction === 'y'
      ? [0, AXIS_LENGTH * 0.36, 0]
      : [0, 0, AXIS_LENGTH * 0.36];
  const rotation: [number, number, number] = direction === 'x'
    ? [0, Math.PI / 2, 0]
    : direction === 'y'
      ? [Math.PI / 2, 0, 0]
      : [0, 0, 0];

  return (
    <group position={position} rotation={rotation}>
      <mesh>
        <torusGeometry args={[ROTATION_RADIUS, 0.0045, 6, 24, Math.PI * 1.5]} />
        <meshBasicMaterial
          color="#f2f0e8"
          transparent
          opacity={SENSOR_BEAM_OPACITY}
          depthTest={false}
          depthWrite={false}
          toneMapped={false}
        />
      </mesh>
      <mesh position={[0, -ROTATION_RADIUS, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.013, 0.038, 8]} />
        <meshBasicMaterial
          color="#f2f0e8"
          transparent
          opacity={SENSOR_BEAM_OPACITY}
          depthTest={false}
          depthWrite={false}
          toneMapped={false}
        />
      </mesh>
    </group>
  );
}

function AxisArrow({ direction, label, active }: AxisArrowProps) {
  const color = AXIS_COLORS[label];
  const cylinderPosition: [number, number, number] = direction === 'x'
    ? [AXIS_LENGTH / 2, 0, 0]
    : direction === 'y'
      ? [0, AXIS_LENGTH / 2, 0]
      : [0, 0, AXIS_LENGTH / 2];
  const arrowPosition: [number, number, number] = direction === 'x'
    ? [AXIS_LENGTH, 0, 0]
    : direction === 'y'
      ? [0, AXIS_LENGTH, 0]
      : [0, 0, AXIS_LENGTH];
  const rotation: [number, number, number] = direction === 'x'
    ? [0, 0, -Math.PI / 2]
    : direction === 'z'
      ? [Math.PI / 2, 0, 0]
      : [0, 0, 0];

  return (
    <group>
      <mesh position={cylinderPosition} rotation={rotation}>
        <cylinderGeometry args={[0.008, 0.008, AXIS_LENGTH, 10]} />
        <meshBasicMaterial color={color} depthTest={false} toneMapped={false} />
      </mesh>
      <mesh position={arrowPosition} rotation={rotation}>
        <coneGeometry args={[0.026, 0.075, 12]} />
        <meshBasicMaterial color={color} depthTest={false} toneMapped={false} />
      </mesh>
      <Html
        position={arrowPosition}
        center
        className="inspection-axis-label"
        style={{ color, opacity: active ? 1 : 0, transition: 'opacity 280ms ease' }}
      >
        <span data-axis-direction={direction}>{label.toUpperCase()}</span>
      </Html>
      <AxisRotationArrow direction={direction} />
    </group>
  );
}

export function InspectionGuides({ activeChapter, assetAvailable }: InspectionGuidesProps) {
  const { scene, size } = useThree();
  const axisGroup = useRef<THREE.Group>(null);
  const encoderGroup = useRef<THREE.Group>(null);
  const tracked = useRef<{
    imu: THREE.Object3D | null;
    encoder: THREE.Object3D | null;
  }>({ imu: null, encoder: null });
  const scratchPosition = useRef(new THREE.Vector3());
  const reveal = useRef(0.001);
  const [renderGuides, setRenderGuides] = useState(activeChapter === 2);

  useEffect(() => {
    if (activeChapter === 2) {
      setRenderGuides(true);
      return undefined;
    }

    const hideTimer = window.setTimeout(() => {
      reveal.current = 0.001;
      setRenderGuides(false);
    }, 450);
    return () => window.clearTimeout(hideTimer);
  }, [activeChapter]);

  useEffect(() => {
    tracked.current = { imu: null, encoder: null };
  }, [assetAvailable]);

  useFrame((_, delta) => {
    if (!axisGroup.current || !encoderGroup.current) return;
    axisGroup.current.visible = true;
    encoderGroup.current.visible = true;
    reveal.current = THREE.MathUtils.damp(
      reveal.current,
      activeChapter === 2 ? 1 : 0.001,
      7,
      delta,
    );

    const responsiveScale = getResponsiveRobotScale(size.width);
    const imu = tracked.current.imu ?? scene.getObjectByName('imu');
    if (imu) {
      tracked.current.imu = imu;
      imu.getWorldPosition(scratchPosition.current);
      axisGroup.current.position.copy(scratchPosition.current);
      axisGroup.current.position.y += 0.24 * responsiveScale;
      axisGroup.current.scale.setScalar(responsiveScale * reveal.current);
    } else {
      axisGroup.current.visible = false;
    }

    const encoder = tracked.current.encoder ?? scene.getObjectByName('encoder_left');
    if (encoder) {
      tracked.current.encoder = encoder;
      encoder.getWorldPosition(scratchPosition.current);
      encoderGroup.current.position.copy(scratchPosition.current);
      encoderGroup.current.position.x += 0.35 * responsiveScale;
      encoderGroup.current.scale.setScalar(responsiveScale * reveal.current);
    } else {
      encoderGroup.current.visible = false;
    }
  });

  // Keep the tree alive briefly after chapter 03 so the scale-out can finish.
  // It is then removed because Drei Html portals ignore Three.js visibility.
  if (!renderGuides) return null;

  return (
    <group name="chapter_03_inspection_guides">
      <group ref={axisGroup} name="imu_xyz_axis">
        {/* Robot axes relabel scene X/Y/Z as Y/Z/X respectively. */}
        <AxisArrow direction="x" label="y" active={activeChapter === 2} />
        <AxisArrow direction="y" label="z" active={activeChapter === 2} />
        <AxisArrow direction="z" label="x" active={activeChapter === 2} />
      </group>

      <group
        ref={encoderGroup}
        name="encoder_rotation_guide"
        rotation={[0, Math.PI / 2, 0]}
      >
        <mesh>
          <torusGeometry args={[0.19, 0.018, 8, 40, Math.PI * 1.55]} />
          <meshBasicMaterial
            color="#ffb800"
            transparent
            opacity={0.95}
            depthTest={false}
            depthWrite={false}
            toneMapped={false}
          />
        </mesh>
        <mesh position={[0.03, -0.187, 0]} rotation={[0, 0, -Math.PI * 0.45]}>
          <coneGeometry args={[0.042, 0.11, 12]} />
          <meshBasicMaterial
            color="#ffb800"
            transparent
            opacity={0.95}
            depthTest={false}
            depthWrite={false}
            toneMapped={false}
          />
        </mesh>
      </group>
    </group>
  );
}
