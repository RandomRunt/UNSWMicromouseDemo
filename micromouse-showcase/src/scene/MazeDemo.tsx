import { Line } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';

interface MazeDemoProps {
  activeChapter: number;
  reducedMotion: boolean;
}

const wallSegments: Array<[number, number, number, number]> = [
  [-2.6, -2.6, 5.2, 0.08],
  [-2.6, 2.6, 5.2, 0.08],
  [-2.6, 0, 0.08, 5.2],
  [2.6, 0, 0.08, 5.2],
  [-1.3, -1.25, 2.55, 0.08],
  [1.3, 0, 2.55, 0.08],
  [-0.02, 1.28, 0.08, 2.55],
  [-1.28, 1.9, 0.08, 1.3],
];

const pathPoints: Array<[number, number, number]> = [
  [1.95, 0.02, 2.05],
  [0.65, 0.02, 2.05],
  [0.65, 0.02, 0.68],
  [-0.66, 0.02, 0.68],
  [-0.66, 0.02, -1.95],
  [-1.95, 0.02, -1.95],
];

export function MazeDemo({ activeChapter, reducedMotion }: MazeDemoProps) {
  const group = useRef<THREE.Group>(null);
  const marker = useRef<THREE.Mesh>(null);

  useFrame((state, delta) => {
    if (!group.current) return;
    const targetScale = activeChapter === 4 ? 1 : 0.001;
    const nextScale = THREE.MathUtils.damp(group.current.scale.x, targetScale, 6, delta);
    group.current.scale.setScalar(nextScale);

    if (marker.current && activeChapter === 4 && !reducedMotion) {
      const t = (state.clock.elapsedTime * 0.11) % 1;
      const curve = new THREE.CatmullRomCurve3(pathPoints.map((point) => new THREE.Vector3(...point)));
      marker.current.position.copy(curve.getPointAt(t));
    }
  });

  return (
    <group ref={group} position={[0, -0.19, 0]} visible={activeChapter === 4}>
      {wallSegments.map(([x, z, width, depth], index) => (
        <mesh key={index} position={[x, 0, z]} receiveShadow>
          <boxGeometry args={[width, 0.16, depth]} />
          <meshStandardMaterial color="#2a3235" roughness={0.7} metalness={0.32} />
        </mesh>
      ))}
      <Line points={pathPoints} color="#ffb800" lineWidth={2} transparent opacity={0.72} />
      <mesh ref={marker} position={pathPoints[0]}>
        <ringGeometry args={[0.12, 0.2, 24]} />
        <meshBasicMaterial color="#d8ff65" side={THREE.DoubleSide} transparent opacity={0.9} />
      </mesh>
    </group>
  );
}
