import { useFrame, useThree } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';
import { componentById } from '../config/components';
import type { ComponentId, ComponentScreenAnchorRef } from '../types/showcase';

interface ComponentAnchorTrackerProps {
  selected: ComponentId | null;
  anchorRef: ComponentScreenAnchorRef;
}

function belongsToScene(object: THREE.Object3D, scene: THREE.Scene) {
  let root: THREE.Object3D = object;
  while (root.parent) root = root.parent;
  return root === scene;
}

export function ComponentAnchorTracker({ selected, anchorRef }: ComponentAnchorTrackerProps) {
  const scene = useThree((state) => state.scene);
  const bounds = useRef(new THREE.Box3());
  const worldCentre = useRef(new THREE.Vector3());
  const projected = useRef(new THREE.Vector3());
  const target = useRef<THREE.Object3D | null>(null);
  const targetName = useRef<string | null>(null);

  useFrame(({ camera, gl }) => {
    if (!selected) {
      anchorRef.current = null;
      target.current = null;
      targetName.current = null;
      return;
    }

    const meshName = componentById[selected].meshName;
    if (
      !target.current
      || targetName.current !== meshName
      || !belongsToScene(target.current, scene)
    ) {
      target.current = scene.getObjectByName(meshName) ?? null;
      targetName.current = meshName;
    }

    if (!target.current) {
      anchorRef.current = null;
      return;
    }

    target.current.updateWorldMatrix(true, true);
    bounds.current.setFromObject(target.current).getCenter(worldCentre.current);
    projected.current.copy(worldCentre.current).project(camera);

    const canvasBounds = gl.domElement.getBoundingClientRect();
    anchorRef.current = {
      x: canvasBounds.left + (projected.current.x * 0.5 + 0.5) * canvasBounds.width,
      y: canvasBounds.top + (-projected.current.y * 0.5 + 0.5) * canvasBounds.height,
      visible: projected.current.z >= -1 && projected.current.z <= 1,
    };
  });

  return null;
}
