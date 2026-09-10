import { Html } from '@react-three/drei';
import { useEffect, useState } from 'react';
import { useReducedMotion } from '../story/useReducedMotion';

const LOADING_DOT_INTERVAL_MS = 500;

export function SceneLoader() {
  const reducedMotion = useReducedMotion();
  const [dotCount, setDotCount] = useState(1);

  useEffect(() => {
    if (reducedMotion) {
      setDotCount(3);
      return;
    }

    setDotCount(1);
    const interval = window.setInterval(() => {
      setDotCount((current) => current % 3 + 1);
    }, LOADING_DOT_INTERVAL_MS);

    return () => window.clearInterval(interval);
  }, [reducedMotion]);

  return (
    <Html center>
      <div
        className="scene-loader"
        role="status"
        aria-label="Loading model"
        data-testid="scene-loader"
      >
        <span className="scene-loader__label" aria-hidden="true">
          LOADING MODEL<span
            className="scene-loader__dots"
            data-dot-count={dotCount}
          >{'.'.repeat(dotCount)}</span>
        </span>
      </div>
    </Html>
  );
}
