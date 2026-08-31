import { useGLTF } from '@react-three/drei';
import { ThreeEvent, useFrame } from '@react-three/fiber';
import { ReactNode, useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { componentById, componentDefinitions } from '../config/components';
import type { ComponentId } from '../types/showcase';
import { getExplodeAmount } from './motion';

// Main GLB presentation controls. I tune these first whenever I replace or
// re-export the Blender model. Scale is uniform, so one value preserves shape.
const GLB_SCALE = 19; // Increase/decrease this to make the complete robot larger/smaller.

const GLB_POSITION: [number, number, number] = [
  0.88,  // X: increase to move the robot right in the current opening view.
  1.03,  // Y: increase to move up; decrease to move down towards the grid.
  -0.84, // Z: moves the robot forwards/backwards through the scene.
];

// Rotation uses radians in [X, Y, Z] order. Math.PI is a 180-degree turn.
// I currently leave this at zero because the ToF sensors should face the viewer.
const GLB_ROTATION: [number, number, number] = [
  0,
  0,
  0,
];

// The imported robot intentionally faces +Z toward the opening camera. The
// original component offsets use -Z as forward, so I turn only the explosion
// directions around. If I rotate the Blender export by 180 degrees later, I
// should remove this correction instead of manually reversing every offset.
const GLB_EXPLODE_DIRECTION = new THREE.Quaternion().setFromEuler(
  new THREE.Euler(0, Math.PI, 0),
);

const COMPONENT_MESH_NAMES = new Set(
  componentDefinitions.map((component) => component.meshName),
);

interface OriginalMaterialState {
  emissive: THREE.Color;
  emissiveIntensity: number;
}

function visitComponentMeshes(
  root: THREE.Object3D,
  visitor: (mesh: THREE.Mesh) => void,
  isRoot = true,
) {
  if (!isRoot && COMPONENT_MESH_NAMES.has(root.name)) return;
  if (root instanceof THREE.Mesh) visitor(root);
  root.children.forEach((child) => visitComponentMeshes(child, visitor, false));
}

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
  const explodeScratch = useMemo(() => ({
    parentToShowcase: new THREE.Matrix4(),
    showcaseToParent: new THREE.Matrix4(),
    localOrigin: new THREE.Vector3(),
    localOffset: new THREE.Vector3(),
    target: new THREE.Vector3(),
  }), []);
  const { scene, startPositions, namedComponents, originalMaterialStates } = useMemo(() => {
    const clone = gltf.scene.clone(true);
    const positions = new Map<string, THREE.Vector3>();
    const named = new Map<string, THREE.Object3D>();
    const materialStates = new Map<THREE.MeshStandardMaterial, OriginalMaterialState>();
    clone.traverse((object) => {
      positions.set(object.uuid, object.position.clone());
      if (componentDefinitions.some((component) => component.meshName === object.name)) {
        named.set(object.name, object);
      }
      if (object instanceof THREE.Mesh) {
        const sourceMaterials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        const clonedMaterials = sourceMaterials.map((sourceMaterial) => {
          const material = sourceMaterial.clone();
          if (material instanceof THREE.MeshStandardMaterial) {
            materialStates.set(material, {
              emissive: material.emissive.clone(),
              emissiveIntensity: material.emissiveIntensity,
            });
          }
          return material;
        });
        object.material = Array.isArray(object.material)
          ? clonedMaterials
          : clonedMaterials[0];
      }
    });
    return {
      scene: clone,
      startPositions: positions,
      namedComponents: named,
      originalMaterialStates: materialStates,
    };
  }, [gltf.scene]);

  useEffect(() => {
    const restoreMaterials = () => {
      originalMaterialStates.forEach((original, material) => {
        material.emissive.copy(original.emissive);
        material.emissiveIntensity = original.emissiveIntensity;
        material.needsUpdate = true;
      });
    };

    restoreMaterials();
    if (!selected) return restoreMaterials;

    const selectedObject = namedComponents.get(componentById[selected].meshName);
    if (!selectedObject) return restoreMaterials;

    const accent = new THREE.Color(componentById[selected].accent);
    visitComponentMeshes(selectedObject, (mesh) => {
      const materials = Array.isArray(mesh.material)
        ? mesh.material
        : [mesh.material];
      materials.forEach((material) => {
        if (!(material instanceof THREE.MeshStandardMaterial)) return;
        material.emissive.copy(accent);
        material.emissiveIntensity = Math.max(material.emissiveIntensity, 0.42);
        material.needsUpdate = true;
      });
    });

    return restoreMaterials;
  }, [namedComponents, originalMaterialStates, selected]);

  useFrame((state, delta) => {
    if (!root.current) return;

    // Opening idle turn: 0.25 is the base angle, 0.18 is the sweep, and 0.35
    // controls how quickly it moves. Smaller values make the motion subtler.
    const idleRotation = activeChapter === 0 && !reducedMotion
      ? -0.25 + Math.sin(state.clock.elapsedTime * 0.35) * 0.18
      : 0;
    // The damping value controls how quickly the robot settles into its angle.
    root.current.rotation.y = THREE.MathUtils.damp(root.current.rotation.y, idleRotation, 3, delta);
    root.current.updateWorldMatrix(true, true);

    const amount = getExplodeAmount(activeChapter);
    componentDefinitions.forEach((component) => {
      const object = namedComponents.get(component.meshName);
      const start = object ? startPositions.get(object.uuid) : undefined;
      if (!object || !start || !object.parent) return;

      // Component offsets are authored in showcase space. Convert them into the
      // imported component parent's local space so FBX/GLB scale and axis
      // corrections do not shrink or rotate the exploded motion.
      explodeScratch.parentToShowcase
        .copy(root.current!.matrixWorld)
        .invert()
        .multiply(object.parent.matrixWorld);
      explodeScratch.showcaseToParent.copy(explodeScratch.parentToShowcase).invert();
      explodeScratch.localOrigin
        .set(0, 0, 0)
        .applyMatrix4(explodeScratch.showcaseToParent);
      explodeScratch.localOffset
        .set(...component.explodeOffset)
        .applyQuaternion(GLB_EXPLODE_DIRECTION)
        .applyMatrix4(explodeScratch.showcaseToParent)
        .sub(explodeScratch.localOrigin);
      explodeScratch.target
        .copy(start)
        .addScaledVector(explodeScratch.localOffset, amount);
      // Increase 5 for a faster teardown; decrease it for a slower, softer one.
      object.position.lerp(explodeScratch.target, 1 - Math.exp(-delta * 5));
    });
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
    <group ref={root} name="showcase_model_root" onClick={selectObject}>
      <group scale={GLB_SCALE} position={GLB_POSITION} rotation={GLB_ROTATION}>
        <primitive object={scene} />
      </group>
    </group>
  );
}
