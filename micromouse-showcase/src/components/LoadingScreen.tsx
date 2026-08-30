import { Html, useProgress } from '@react-three/drei';

export function SceneLoader() {
  const { progress } = useProgress();

  return (
    <Html center>
      <div className="scene-loader" role="status" aria-live="polite">
        <span className="scene-loader__label">CALIBRATING MODEL</span>
        <span className="scene-loader__value">{Math.round(progress)}%</span>
      </div>
    </Html>
  );
}
