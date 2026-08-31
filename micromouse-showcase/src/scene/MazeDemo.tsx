import { Line } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';

interface MazeDemoProps {
  activeChapter: number;
  reducedMotion: boolean;
}

type Cell = readonly [column: number, row: number];
type WallSegment = readonly [x: number, z: number, width: number, depth: number];

// Main maze tuning controls. More cells creates a denser maze, while a smaller
// cell size moves the walls closer together without changing their thickness.
const MAZE_SIZE = 7;
const CELL_SIZE = 0.62;
const WALL_THICKNESS = 0.075;
const WALL_HEIGHT = 0.42;
const MAZE_Y = -0.62; // Keeps the wall tops below the imported Micromouse.
const FLOOR_Y = -WALL_HEIGHT / 2;
const ROUTE_Y = FLOOR_Y + 0.035;
const MARKER_SIZE = 0.17;
const MARKER_Y = FLOOR_Y + MARKER_SIZE / 2 + 0.025;
const MAZE_SEED = 2409; // Change this integer to generate a different valid maze.

const CARDINAL_DIRECTIONS: ReadonlyArray<Cell> = [
  [1, 0],
  [-1, 0],
  [0, 1],
  [0, -1],
];

function cellId([column, row]: Cell) {
  return row * MAZE_SIZE + column;
}

function passageKey(a: Cell, b: Cell) {
  const first = cellId(a);
  const second = cellId(b);
  return first < second ? `${first}-${second}` : `${second}-${first}`;
}

function createSeededRandom(seed: number) {
  let value = seed >>> 0;
  return () => {
    value = (value * 1664525 + 1013904223) >>> 0;
    return value / 0x100000000;
  };
}

function buildPassages() {
  const random = createSeededRandom(MAZE_SEED);
  const start: Cell = [0, MAZE_SIZE - 1];
  const stack: Cell[] = [start];
  const visited = new Set<number>([cellId(start)]);
  const passages = new Set<string>();

  while (stack.length > 0) {
    const current = stack[stack.length - 1];
    const available = CARDINAL_DIRECTIONS
      .map(([columnOffset, rowOffset]) => [
        current[0] + columnOffset,
        current[1] + rowOffset,
      ] as Cell)
      .filter(([column, row]) => (
        column >= 0
        && column < MAZE_SIZE
        && row >= 0
        && row < MAZE_SIZE
        && !visited.has(cellId([column, row]))
      ));

    if (available.length === 0) {
      stack.pop();
      continue;
    }

    const next = available[Math.floor(random() * available.length)];
    passages.add(passageKey(current, next));
    visited.add(cellId(next));
    stack.push(next);
  }

  return passages;
}

function buildWalls(passages: Set<string>): WallSegment[] {
  const totalSize = MAZE_SIZE * CELL_SIZE;
  const halfSize = totalSize / 2;
  const walls: WallSegment[] = [
    [0, -halfSize, totalSize + WALL_THICKNESS, WALL_THICKNESS],
    [0, halfSize, totalSize + WALL_THICKNESS, WALL_THICKNESS],
    [-halfSize, 0, WALL_THICKNESS, totalSize + WALL_THICKNESS],
    [halfSize, 0, WALL_THICKNESS, totalSize + WALL_THICKNESS],
  ];

  for (let row = 0; row < MAZE_SIZE; row += 1) {
    for (let column = 0; column < MAZE_SIZE; column += 1) {
      const current: Cell = [column, row];
      const cellX = (column + 0.5) * CELL_SIZE - halfSize;
      const cellZ = (row + 0.5) * CELL_SIZE - halfSize;

      if (column < MAZE_SIZE - 1) {
        const right: Cell = [column + 1, row];
        if (!passages.has(passageKey(current, right))) {
          walls.push([
            (column + 1) * CELL_SIZE - halfSize,
            cellZ,
            WALL_THICKNESS,
            CELL_SIZE + WALL_THICKNESS,
          ]);
        }
      }

      if (row < MAZE_SIZE - 1) {
        const below: Cell = [column, row + 1];
        if (!passages.has(passageKey(current, below))) {
          walls.push([
            cellX,
            (row + 1) * CELL_SIZE - halfSize,
            CELL_SIZE + WALL_THICKNESS,
            WALL_THICKNESS,
          ]);
        }
      }
    }
  }

  return walls;
}

function solveMaze(passages: Set<string>, start: Cell, goal: Cell): Cell[] {
  const queue: Cell[] = [start];
  const previous = new Map<number, number>([[cellId(start), -1]]);

  while (queue.length > 0) {
    const current = queue.shift()!;
    if (cellId(current) === cellId(goal)) break;

    CARDINAL_DIRECTIONS.forEach(([columnOffset, rowOffset]) => {
      const next: Cell = [current[0] + columnOffset, current[1] + rowOffset];
      const [column, row] = next;
      if (
        column < 0
        || column >= MAZE_SIZE
        || row < 0
        || row >= MAZE_SIZE
        || previous.has(cellId(next))
        || !passages.has(passageKey(current, next))
      ) return;

      previous.set(cellId(next), cellId(current));
      queue.push(next);
    });
  }

  const route: Cell[] = [];
  let currentId = cellId(goal);
  while (currentId !== -1) {
    route.unshift([currentId % MAZE_SIZE, Math.floor(currentId / MAZE_SIZE)]);
    currentId = previous.get(currentId) ?? -1;
  }
  return route;
}

function cellToPoint([column, row]: Cell) {
  const halfSize = MAZE_SIZE * CELL_SIZE / 2;
  return new THREE.Vector3(
    (column + 0.5) * CELL_SIZE - halfSize,
    ROUTE_Y,
    (row + 0.5) * CELL_SIZE - halfSize,
  );
}

const passages = buildPassages();
const wallSegments = buildWalls(passages);
const routeCells = solveMaze(passages, [0, MAZE_SIZE - 1], [MAZE_SIZE - 1, 0]);
const pathPoints = routeCells.map(cellToPoint);
const routeCurve = new THREE.CurvePath<THREE.Vector3>();

for (let index = 0; index < pathPoints.length - 1; index += 1) {
  // Straight line segments keep every turn inside the carved maze corridors.
  routeCurve.add(new THREE.LineCurve3(pathPoints[index], pathPoints[index + 1]));
}

export function MazeDemo({ activeChapter, reducedMotion }: MazeDemoProps) {
  const group = useRef<THREE.Group>(null);
  const marker = useRef<THREE.Mesh>(null);

  useFrame((state, delta) => {
    if (!group.current) return;
    const targetScale = activeChapter === 4 ? 1 : 0.001;
    const nextScale = THREE.MathUtils.damp(group.current.scale.x, targetScale, 6, delta);
    group.current.scale.setScalar(nextScale);

    if (marker.current && activeChapter === 4 && !reducedMotion) {
      // 0.08 controls how quickly the cube completes the solved route.
      const t = (state.clock.elapsedTime * 0.08) % 1;
      marker.current.position.copy(routeCurve.getPointAt(t));
      marker.current.position.y = MARKER_Y;
      marker.current.rotation.y += delta * 1.6;
    }
  });

  const totalSize = MAZE_SIZE * CELL_SIZE;

  return (
    <group ref={group} position={[0, MAZE_Y, 0]} visible={activeChapter === 4}>
      <mesh position={[0, FLOOR_Y - 0.025, 0]} receiveShadow>
        <boxGeometry args={[totalSize + 0.12, 0.05, totalSize + 0.12]} />
        <meshStandardMaterial color="#101517" roughness={0.92} metalness={0.08} />
      </mesh>

      {wallSegments.map(([x, z, width, depth], index) => (
        <mesh key={`${x}-${z}-${index}`} position={[x, 0, z]} castShadow receiveShadow>
          <boxGeometry args={[width, WALL_HEIGHT, depth]} />
          <meshStandardMaterial color="#1b2225" roughness={0.72} metalness={0.28} />
        </mesh>
      ))}

      <Line
        points={pathPoints}
        color="#ffb800"
        lineWidth={3}
        transparent
        opacity={0.9}
      />

      <mesh ref={marker} position={[pathPoints[0].x, MARKER_Y, pathPoints[0].z]} castShadow>
        <boxGeometry args={[MARKER_SIZE, MARKER_SIZE, MARKER_SIZE]} />
        <meshStandardMaterial
          color="#d8ff65"
          emissive="#ffb800"
          emissiveIntensity={0.55}
          roughness={0.38}
        />
      </mesh>
    </group>
  );
}
