export type ChapterId =
  | 'meet'
  | 'inside'
  | 'sense'
  | 'think'
  | 'move'
  | 'explore';

export type ComponentId =
  | 'chassis'
  | 'controller'
  | 'motor_driver'
  | 'battery'
  | 'imu'
  | 'lidar_left'
  | 'lidar_front'
  | 'lidar_right'
  | 'motor_left'
  | 'motor_right'
  | 'wheel_left'
  | 'wheel_right'
  | 'encoder_left'
  | 'encoder_right';

export interface StoryChapter {
  id: ChapterId;
  number: string;
  eyebrow: string;
  title: string;
  body: string;
  metric: string;
  metricLabel: string;
}

export interface ComponentDefinition {
  id: ComponentId;
  meshName: string;
  title: string;
  shortLabel: string;
  description: string;
  accent: string;
  explodeOffset: [number, number, number];
}

export interface ComponentScreenAnchor {
  x: number;
  y: number;
  visible: boolean;
}

export interface ComponentScreenAnchorRef {
  current: ComponentScreenAnchor | null;
}
