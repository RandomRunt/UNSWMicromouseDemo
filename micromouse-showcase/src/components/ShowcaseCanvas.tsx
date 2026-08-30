import { Canvas } from '@react-three/fiber';
import { Suspense } from 'react';
import type { ComponentId, ComponentScreenAnchorRef } from '../types/showcase';
import { CameraRig } from '../scene/CameraRig';
import { ComponentAnchorTracker } from '../scene/ComponentAnchorTracker';
import { Lighting } from '../scene/Lighting';
import { MazeDemo } from '../scene/MazeDemo';
import { GLBMicromouse, ProceduralMicromouse } from '../scene/MicromouseModel';
import { SceneEnvironment } from '../scene/SceneEnvironment';
import { SensorBeams } from '../scene/SensorBeams';
import { SceneLoader } from './LoadingScreen';
import type { ThemeMode } from '../App';

interface ShowcaseCanvasProps {
  progress: number;
  activeChapter: number;
  explorationEnabled: boolean;
  selected: ComponentId | null;
  onSelect: (id: ComponentId) => void;
  onClearSelection: () => void;
  onInspectionInput: () => void;
  inspectionActive: boolean;
  reducedMotion: boolean;
  assetAvailable: boolean;
  theme: ThemeMode;
  componentAnchorRef: ComponentScreenAnchorRef;
}

export function ShowcaseCanvas({
  progress,
  activeChapter,
  explorationEnabled,
  selected,
  onSelect,
  onClearSelection,
  onInspectionInput,
  inspectionActive,
  reducedMotion,
  assetAvailable,
  theme,
  componentAnchorRef,
}: ShowcaseCanvasProps) {
  return (
    <div
      className="canvas-shell"
      aria-hidden="true"
      data-testid="showcase-canvas"
      data-exploration-enabled={explorationEnabled}
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
          reducedMotion={reducedMotion}
        />
        <Suspense fallback={<SceneLoader />}>
          {assetAvailable ? (
            <GLBMicromouse
              activeChapter={activeChapter}
              selected={selected}
              onSelect={onSelect}
              reducedMotion={reducedMotion}
            />
          ) : (
            <ProceduralMicromouse
              activeChapter={activeChapter}
              selected={selected}
              onSelect={onSelect}
              reducedMotion={reducedMotion}
            />
          )}
          <SensorBeams activeChapter={activeChapter} reducedMotion={reducedMotion} />
          <MazeDemo activeChapter={activeChapter} reducedMotion={reducedMotion} />
          <ComponentAnchorTracker selected={selected} anchorRef={componentAnchorRef} />
        </Suspense>
      </Canvas>
      <div
        className={`canvas-vignette${inspectionActive ? ' canvas-vignette--inspection-active' : ''}`}
      />
      <div className="canvas-reticle" aria-hidden="true">
        <span />
        <i />
      </div>
    </div>
  );
}
