import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';

interface SensorBeamsProps {
  activeChapter: number;
  reducedMotion: boolean;
}

export function SensorBeams({ activeChapter, reducedMotion }: SensorBeamsProps) {
  const group = useRef<THREE.Group>(null);

  useFrame((state, delta) => {
    if (!group.current) return;
    const target = activeChapter === 2 ? 1 : 0.001;
    const scale = THREE.MathUtils.damp(group.current.scale.y, target, 7, delta);
    group.current.scale.setScalar(scale);
    if (!reducedMotion) {
      group.current.children.forEach((child, index) => {
        const material = (child as THREE.Mesh).material as THREE.MeshBasicMaterial;
        if (material) material.opacity = 0.28 + Math.sin(state.clock.elapsedTime * 3 + index) * 0.12;
      });
    }
  });

  return (
    <group ref={group} visible={activeChapter === 2}>
      <mesh position={[0, 0.42, -2.15]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[0.018, 0.075, 2.05, 8]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={0.38} depthWrite={false} />
      </mesh>
      <mesh position={[-1.8, 0.4, -1.18]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.018, 0.07, 1.75, 8]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={0.32} depthWrite={false} />
      </mesh>
      <mesh position={[1.8, 0.4, -1.18]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.018, 0.07, 1.75, 8]} />
        <meshBasicMaterial color="#45e6ff" transparent opacity={0.32} depthWrite={false} />
      </mesh>
      <mesh position={[0.34, 0.69, 0.08]} rotation={[Math.PI / 2, 0, 0]}>
        <torusGeometry args={[0.42, 0.018, 8, 48, Math.PI * 1.5]} />
        <meshBasicMaterial color="#9edcff" transparent opacity={0.68} />
      </mesh>
    </group>
  );
}
