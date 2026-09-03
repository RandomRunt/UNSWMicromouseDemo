import { Html } from '@react-three/drei';

export function SceneLoader() {
  return (
    <Html center>
      <div className="scene-loader" role="status" aria-live="polite" data-testid="scene-loader">
        <span className="scene-loader__label">LOADING MODEL...</span>
        <span className="scene-loader__value">PLEASE WAIT</span>
      </div>
    </Html>
  );
}
