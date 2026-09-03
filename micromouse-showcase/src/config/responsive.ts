export const MOBILE_VIEW_BREAKPOINT = 640;
export const MOBILE_ROBOT_SCALE = 0.68;
export const MOBILE_MAZE_SCALE = 0.4;

export function getResponsiveRobotScale(canvasWidth: number) {
  return canvasWidth <= MOBILE_VIEW_BREAKPOINT ? MOBILE_ROBOT_SCALE : 1;
}

export function getResponsiveMazeScale(canvasWidth: number) {
  return canvasWidth <= MOBILE_VIEW_BREAKPOINT ? MOBILE_MAZE_SCALE : 1;
}
