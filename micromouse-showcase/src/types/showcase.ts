export type ChapterId =
  | 'meet'
  | 'inside'
  | 'sense'
  | 'think'
  | 'move'
  | 'explore';

export type ComponentId =
  | 'chassis'
  | 'bottom_pcb'
  | 'top_pcb'
  | 'battery'
  | 'power_switch'
  | 'microcontroller'
  | 'imu'
  | 'tof_left'
  | 'tof_front'
  | 'tof_right'
  | 'oled_display'
  | 'motor_driver'
  | 'motor_left'
  | 'motor_right'
  | 'encoder_left'
  | 'encoder_right'
  | 'wheel_left'
  | 'wheel_right'
  | 'ball_caster_front'
  | 'ball_caster_rear';

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
