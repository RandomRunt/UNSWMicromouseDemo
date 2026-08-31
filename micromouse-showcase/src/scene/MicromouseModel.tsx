import { useGLTF } from '@react-three/drei';
import { ThreeEvent, useFrame } from '@react-three/fiber';
import { ReactNode, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { componentById, componentDefinitions } from '../config/components';
import type { ComponentId } from '../types/showcase';
import { getExplodeAmount } from './motion';

interface MicromouseProps {
  activeChapter: number;
  selected: ComponentId | null;
  onSelect: (id: ComponentId) => void;
  reducedMotion: boolean;
}

interface PartProps {
  id: ComponentId;
  activeChapter: number;
  selected: ComponentId | null;
  onSelect: (id: ComponentId) => void;
  position?: [number, number, number];
  rotation?: [number, number, number];
  children: ReactNode;
}

function Part({
  id,
  activeChapter,
  selected,
  onSelect,
  position = [0, 0, 0],
  rotation = [0, 0, 0],
  children,
}: PartProps) {
  const group = useRef<THREE.Group>(null);
  const definition = componentById[id];
  const base = useMemo(() => new THREE.Vector3(...position), [position]);
  const target = useMemo(() => new THREE.Vector3(), []);

  useFrame((_, delta) => {
    if (!group.current) return;
    const amount = getExplodeAmount(activeChapter);
    target.set(
      base.x + definition.explodeOffset[0] * amount,
      base.y + definition.explodeOffset[1] * amount,
      base.z + definition.explodeOffset[2] * amount,
    );
    group.current.position.lerp(target, 1 - Math.exp(-delta * 5));
    const targetScale = selected === id ? 1.08 : 1;
    const nextScale = THREE.MathUtils.damp(group.current.scale.x, targetScale, 8, delta);
    group.current.scale.setScalar(nextScale);
  });

  const select = (event: ThreeEvent<MouseEvent>) => {
    event.stopPropagation();
    onSelect(id);
  };

  return (
    <group
      ref={group}
      name={definition.meshName}
      position={position}
      rotation={rotation}
      onClick={select}
      onPointerOver={(event) => {
        event.stopPropagation();
        document.body.style.cursor = 'pointer';
      }}
      onPointerOut={() => {
        document.body.style.cursor = '';
      }}
    >
      {children}
    </group>
  );
}

function CircuitMaterial({ color = '#1b5b43', selected = false }: { color?: string; selected?: boolean }) {
  return (
    <meshStandardMaterial
      color={color}
      roughness={0.54}
      metalness={0.2}
      emissive={selected ? '#ffb800' : '#07130f'}
      emissiveIntensity={selected ? 0.9 : 0.2}
    />
  );
}

export function ProceduralMicromouse({ activeChapter, selected, onSelect, reducedMotion }: MicromouseProps) {
  const root = useRef<THREE.Group>(null);

  useFrame((state, delta) => {
    if (!root.current) return;
    const idle = activeChapter === 0 ? 0.18 : 0;
    const desiredRotation = reducedMotion ? -0.22 : -0.25 + Math.sin(state.clock.elapsedTime * 0.35) * idle;
    root.current.rotation.y = THREE.MathUtils.damp(root.current.rotation.y, desiredRotation, 2.5, delta);
    root.current.position.y = reducedMotion ? 0 : Math.sin(state.clock.elapsedTime * 0.7) * 0.015;
  });

  return (
    <group ref={root} name="mouse_root" position={[0, 0, 0]}>
      <Part id="chassis" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0, 0.1, 0.12]}>
        <mesh castShadow receiveShadow>
          <boxGeometry args={[2.05, 0.16, 2.35]} />
          <meshStandardMaterial color="#161c1f" roughness={0.32} metalness={0.72} />
        </mesh>
        {[-0.74, 0.74].flatMap((x) => [-0.86, 0.86].map((z) => (
          <mesh key={`${x}-${z}`} position={[x, 0.22, z]} castShadow>
            <cylinderGeometry args={[0.065, 0.065, 0.17, 12]} />
            <meshStandardMaterial color="#ffb800" metalness={0.76} roughness={0.28} />
          </mesh>
        )))}
      </Part>

      <Part id="bottom_pcb" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0, 0.32, 0.1]}>
        <mesh castShadow>
          <boxGeometry args={[1.72, 0.08, 2.02]} />
          <CircuitMaterial color="#397d4a" selected={selected === 'bottom_pcb'} />
        </mesh>
      </Part>

      <Part id="top_pcb" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0, 0.74, 0.05]}>
        <mesh castShadow>
          <boxGeometry args={[1.78, 0.06, 1.88]} />
          <CircuitMaterial color="#438d51" selected={selected === 'top_pcb'} />
        </mesh>
      </Part>

      <Part id="microcontroller" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[-0.28, 0.88, -0.2]}>
        <mesh castShadow>
          <boxGeometry args={[1.12, 0.1, 1.05]} />
          <CircuitMaterial selected={selected === 'microcontroller'} />
        </mesh>
        <mesh position={[0, 0.095, 0]} castShadow>
          <boxGeometry args={[0.42, 0.09, 0.38]} />
          <meshStandardMaterial color="#121719" metalness={0.25} roughness={0.55} />
        </mesh>
        {[-0.38, -0.19, 0, 0.19, 0.38].map((x) => (
          <mesh key={x} position={[x, 0.08, -0.42]}>
            <boxGeometry args={[0.035, 0.06, 0.13]} />
            <meshStandardMaterial color="#d5b779" metalness={0.85} roughness={0.2} />
          </mesh>
        ))}
      </Part>

      <Part id="motor_driver" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0.42, 0.87, 0.42]}>
        <mesh castShadow>
          <boxGeometry args={[0.82, 0.09, 0.48]} />
          <CircuitMaterial color="#2f383b" selected={selected === 'motor_driver'} />
        </mesh>
        {[-0.24, 0.24].map((x) => (
          <mesh key={x} position={[x, 0.08, 0]} castShadow>
            <boxGeometry args={[0.18, 0.11, 0.23]} />
            <meshStandardMaterial color="#0a0c0d" roughness={0.4} />
          </mesh>
        ))}
      </Part>

      <Part id="battery" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0, 0.32, -0.56]}>
        <mesh castShadow>
          <boxGeometry args={[0.9, 0.28, 0.62]} />
          <meshStandardMaterial
            color="#22282a"
            roughness={0.48}
            metalness={0.2}
            emissive={selected === 'battery' ? '#d8ff65' : '#000000'}
            emissiveIntensity={0.45}
          />
        </mesh>
        <mesh position={[0, 0, -0.316]}>
          <planeGeometry args={[0.54, 0.12]} />
          <meshBasicMaterial color="#d8ff65" />
        </mesh>
      </Part>

      <Part id="power_switch" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0.63, 0.55, 0.72]}>
        <mesh castShadow>
          <boxGeometry args={[0.28, 0.2, 0.34]} />
          <meshStandardMaterial
            color="#b7352d"
            roughness={0.42}
            emissive={selected === 'power_switch' ? '#ffb800' : '#000000'}
            emissiveIntensity={0.5}
          />
        </mesh>
        <mesh position={[0, 0.15, 0]} rotation={[0, 0, -0.35]} castShadow>
          <boxGeometry args={[0.09, 0.22, 0.09]} />
          <meshStandardMaterial color="#d8d9d5" metalness={0.75} roughness={0.25} />
        </mesh>
      </Part>

      <Part id="imu" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0.34, 0.66, 0.08]}>
        <mesh castShadow>
          <boxGeometry args={[0.28, 0.07, 0.28]} />
          <CircuitMaterial color="#165a68" selected={selected === 'imu'} />
        </mesh>
        <mesh position={[0, 0.06, 0]}>
          <boxGeometry args={[0.12, 0.05, 0.12]} />
          <meshStandardMaterial color="#121719" />
        </mesh>
      </Part>

      <Part id="tof_front" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0, 0.42, -1.12]}>
        <mesh castShadow>
          <boxGeometry args={[0.36, 0.25, 0.18]} />
          <meshStandardMaterial color="#20282b" metalness={0.55} roughness={0.34} />
        </mesh>
        <mesh position={[0, 0, -0.1]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.065, 0.065, 0.035, 20]} />
          <meshBasicMaterial color="#45e6ff" />
        </mesh>
      </Part>

      <Part id="tof_left" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[-0.93, 0.4, -0.72]} rotation={[0, -Math.PI / 2, 0]}>
        <mesh castShadow>
          <boxGeometry args={[0.2, 0.23, 0.32]} />
          <meshStandardMaterial color="#20282b" metalness={0.55} roughness={0.34} />
        </mesh>
      </Part>

      <Part id="tof_right" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0.93, 0.4, -0.72]} rotation={[0, Math.PI / 2, 0]}>
        <mesh castShadow>
          <boxGeometry args={[0.2, 0.23, 0.32]} />
          <meshStandardMaterial color="#20282b" metalness={0.55} roughness={0.34} />
        </mesh>
      </Part>

      <Part id="oled_display" activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0, 0.91, 0.58]} rotation={[-0.12, 0, 0]}>
        <mesh castShadow>
          <boxGeometry args={[0.72, 0.08, 0.42]} />
          <CircuitMaterial color="#173f57" selected={selected === 'oled_display'} />
        </mesh>
        <mesh position={[0, 0.046, 0]} rotation={[-Math.PI / 2, 0, 0]}>
          <planeGeometry args={[0.56, 0.27]} />
          <meshBasicMaterial color="#70b7ff" />
        </mesh>
      </Part>

      {(['left', 'right'] as const).map((side) => {
        const x = side === 'left' ? -0.82 : 0.82;
        const motorId: ComponentId = side === 'left' ? 'motor_left' : 'motor_right';
        const wheelId: ComponentId = side === 'left' ? 'wheel_left' : 'wheel_right';
        const encoderId: ComponentId = side === 'left' ? 'encoder_left' : 'encoder_right';
        return (
          <group key={side}>
            <Part id={motorId} activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[x, 0.24, 0.28]} rotation={[0, 0, Math.PI / 2]}>
              <mesh castShadow>
                <cylinderGeometry args={[0.21, 0.21, 0.48, 20]} />
                <meshStandardMaterial color="#9ca3a5" metalness={0.82} roughness={0.3} />
              </mesh>
            </Part>
            <Part id={wheelId} activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[side === 'left' ? -1.08 : 1.08, 0.2, 0.3]} rotation={[0, 0, Math.PI / 2]}>
              <mesh castShadow>
                <cylinderGeometry args={[0.47, 0.47, 0.24, 30]} />
                <meshStandardMaterial color="#090b0c" roughness={0.88} metalness={0.08} />
              </mesh>
              <mesh position={[0, side === 'left' ? -0.125 : 0.125, 0]}>
                <cylinderGeometry args={[0.19, 0.19, 0.255, 20]} />
                <meshStandardMaterial color="#ffb800" metalness={0.7} roughness={0.25} />
              </mesh>
            </Part>
            <Part id={encoderId} activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[side === 'left' ? -0.78 : 0.78, 0.31, 0.31]} rotation={[0, 0, Math.PI / 2]}>
              <mesh>
                <cylinderGeometry args={[0.11, 0.11, 0.08, 16]} />
                <meshStandardMaterial color="#d8ff65" emissive="#607522" emissiveIntensity={0.35} />
              </mesh>
            </Part>
          </group>
        );
      })}

      {(['front', 'rear'] as const).map((end) => {
        const id: ComponentId = end === 'front' ? 'ball_caster_front' : 'ball_caster_rear';
        const z = end === 'front' ? -0.83 : 1.05;
        return (
          <Part key={end} id={id} activeChapter={activeChapter} selected={selected} onSelect={onSelect} position={[0, -0.06, z]}>
            <mesh castShadow>
              <sphereGeometry args={[0.2, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2]} />
              <meshStandardMaterial color="#c9cccf" metalness={0.9} roughness={0.2} />
            </mesh>
          </Part>
        );
      })}
    </group>
  );
}

export function GLBMicromouse({ activeChapter, selected, onSelect, reducedMotion }: MicromouseProps) {
  const gltf = useGLTF('/models/micromouse.glb');
  const root = useRef<THREE.Group>(null);
  const { scene, startPositions, namedComponents } = useMemo(() => {
    const clone = gltf.scene.clone(true);
    const positions = new Map<string, THREE.Vector3>();
    const named = new Map<string, THREE.Object3D>();
    clone.traverse((object) => {
      positions.set(object.uuid, object.position.clone());
      if (componentDefinitions.some((component) => component.meshName === object.name)) {
        named.set(object.name, object);
      }
    });
    return { scene: clone, startPositions: positions, namedComponents: named };
  }, [gltf.scene]);

  useFrame((state, delta) => {
    const amount = getExplodeAmount(activeChapter);
    componentDefinitions.forEach((component) => {
      const object = namedComponents.get(component.meshName);
      const start = object ? startPositions.get(object.uuid) : undefined;
      if (!object || !start) return;
      const target = start.clone().addScaledVector(new THREE.Vector3(...component.explodeOffset), amount);
      object.position.lerp(target, 1 - Math.exp(-delta * 5));
    });
    if (root.current && activeChapter === 0 && !reducedMotion) {
      root.current.rotation.y = -0.25 + Math.sin(state.clock.elapsedTime * 0.35) * 0.18;
    }
  });

  const selectObject = (event: ThreeEvent<MouseEvent>) => {
    let object: THREE.Object3D | null = event.object;
    while (object) {
      const component = componentDefinitions.find((definition) => definition.meshName === object?.name);
      if (component) {
        event.stopPropagation();
        onSelect(component.id);
        return;
      }
      object = object.parent;
    }
  };

  return (
    <group ref={root} onClick={selectObject} scale={1}>
      <primitive object={scene} />
      {selected && <pointLight position={[0, 1.2, 0]} intensity={3} color={componentById[selected].accent} distance={3} />}
    </group>
  );
}
