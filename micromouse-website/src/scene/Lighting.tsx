import type { ThemeMode } from '../App';

export function Lighting({ theme }: { theme: ThemeMode }) {
  const isLight = theme === 'light';

  return (
    <>
      <ambientLight intensity={isLight ? 1.15 : 0.82} color={isLight ? '#e7eef0' : '#c6d3d8'} />
      <directionalLight position={[4, 7, 4]} intensity={isLight ? 2.8 : 2.4} color="#fff8e8" castShadow />
      <directionalLight position={[-4, 2, -3]} intensity={isLight ? 1.1 : 1.7} color="#45e6ff" />
      <pointLight position={[0, 1, -3]} intensity={isLight ? 6 : 9} distance={6} color="#ff9f1c" />
    </>
  );
}
