import { Canvas, useThree } from '@react-three/fiber';
import { Suspense, type ReactNode } from 'react';
import type {
  ComponentId,
  ComponentScreenAnchorRef,
  ModelAvailability,
} from '../types/showcase';
import { CameraRig } from '../scene/CameraRig';
import { ComponentAnchorTracker } from '../scene/ComponentAnchorTracker';
import { Lighting } from '../scene/Lighting';
import { InspectionGuides } from '../scene/InspectionGuides';
import { MazeDemo } from '../scene/MazeDemo';
import { GLBMicromouse, ProceduralMicromouse } from '../scene/MicromouseModel';
import { SceneEnvironment } from '../scene/SceneEnvironment';
import { SensorBeams } from '../scene/SensorBeams';
import { shouldSpinWheels } from '../scene/motion';
import { getResponsiveRobotScale } from '../config/responsive';
import { SceneLoader } from './LoadingScreen';
import type { ThemeMode } from '../App';

interface ShowcaseCanvasProps {
  progress: number;
  activeChapter: number;
  explorationEnabled: boolean;
  explorationExploded: boolean;
  easterEggSpinActive: boolean;
  selected: ComponentId | null;
  onSelect: (id: ComponentId) => void;
  onClearSelection: () => void;
  onInspectionInput: () => void;
  onOrbitInteraction: () => void;
  inspectionActive: boolean;
  reducedMotion: boolean;
  modelAvailability: ModelAvailability;
  theme: ThemeMode;
  componentAnchorRef: ComponentScreenAnchorRef;
  viewResetKey: number;
}

function ResponsiveRobotScale({ children }: { children: ReactNode }) {
  const canvasWidth = useThree((state) => state.size.width);
  const scale = getResponsiveRobotScale(canvasWidth);

  return <group scale={scale}>{children}</group>;
}

export function ShowcaseCanvas({
  progress,
  activeChapter,
  explorationEnabled,
  explorationExploded,
  easterEggSpinActive,
  selected,
  onSelect,
  onClearSelection,
  onInspectionInput,
  onOrbitInteraction,
  inspectionActive,
  reducedMotion,
  modelAvailability,
  theme,
  componentAnchorRef,
  viewResetKey,
}: ShowcaseCanvasProps) {
  // Chapter 06 stays assembled by default. Its button temporarily gives only
  // the robot model chapter 02's full teardown amount; the camera, maze and
  // sensor effects still receive the real active chapter.
  const modelChapter = activeChapter === 5 && explorationExploded ? 1 : activeChapter;
  const assetAvailable = modelAvailability === 'available';
  const modelSource = modelAvailability === 'checking'
    ? 'loading'
    : modelAvailability === 'available'
      ? 'digital-twin'
      : 'procedural-fallback';

  return (
    <div
      className="canvas-shell"
      aria-hidden="true"
      data-testid="showcase-canvas"
      data-exploration-enabled={explorationEnabled}
      data-model-source={modelSource}
      data-inspection-guides={activeChapter === 2}
      data-wheel-motion={shouldSpinWheels(activeChapter, reducedMotion)}
    >
      <Canvas
        shadows
        dpr={[1, 1.65]}
        camera={{ position: [4.5, 2.8, 5.4], fov: 34, near: 0.1, far: 80 }}
        gl={{ antialias: true, alpha: false, powerPreference: 'high-performance' }}
        onPointerMissed={onClearSelection}
      >
        <color attach="background" args={[theme === 'dark' ? '#4a5154' : '#e6e7e3']} />
        <Lighting theme={theme} />
        <SceneEnvironment theme={theme} />
        <CameraRig
          progress={progress}
          explorationEnabled={explorationEnabled}
          onInspectionInput={onInspectionInput}
          onOrbitInteraction={onOrbitInteraction}
          reducedMotion={reducedMotion}
          viewResetKey={viewResetKey}
        />
        <Suspense fallback={<SceneLoader />}>
          {modelAvailability === 'checking' ? (
            <SceneLoader />
          ) : (
            <>
              <ResponsiveRobotScale>
                {assetAvailable ? (
                  <GLBMicromouse
                    activeChapter={modelChapter}
                    selected={selected}
                    onSelect={onSelect}
                    reducedMotion={reducedMotion}
                    easterEggSpinActive={easterEggSpinActive}
                  />
                ) : (
                  <ProceduralMicromouse
                    activeChapter={modelChapter}
                    selected={selected}
                    onSelect={onSelect}
                    reducedMotion={reducedMotion}
                    easterEggSpinActive={easterEggSpinActive}
                  />
                )}
              </ResponsiveRobotScale>
              <SensorBeams
                activeChapter={activeChapter}
                reducedMotion={reducedMotion}
                assetAvailable={assetAvailable}
              />
              <InspectionGuides
                activeChapter={activeChapter}
                assetAvailable={assetAvailable}
              />
              <MazeDemo
                progress={progress}
                activeChapter={activeChapter}
                reducedMotion={reducedMotion}
              />
              <ComponentAnchorTracker selected={selected} anchorRef={componentAnchorRef} />
            </>
          )}
        </Suspense>
      </Canvas>
      <div
        className={`canvas-vignette${inspectionActive ? ' canvas-vignette--inspection-active' : ''}`}
      />
    </div>
  );
}
