import { Grid, Stars } from '@react-three/drei';
import type { ThemeMode } from '../App';

export function SceneEnvironment({ theme }: { theme: ThemeMode }) {
  const isLight = theme === 'light';

  return (
    <>
      <fog attach="fog" args={[isLight ? '#e6e7e3' : '#4a5154', 8, 18]} />
      <Grid
        position={[0, -0.12, 0]}
        args={[18, 18]}
        cellSize={0.5}
        cellThickness={0.35}
        cellColor={isLight ? '#aeb5b5' : '#626b6e'}
        sectionSize={2}
        sectionThickness={0.7}
        sectionColor={isLight ? '#727e80' : '#899397'}
        fadeDistance={10}
        fadeStrength={1.5}
        infiniteGrid
      />
      {!isLight && <Stars radius={22} depth={12} count={500} factor={1.2} saturation={0} fade speed={0.15} />}
    </>
  );
}
