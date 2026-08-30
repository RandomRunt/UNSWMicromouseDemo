import { useEffect } from 'react';

const IDLE_TIMEOUT_MS = 90_000;

export function useKioskReset(onReset: () => void) {
  useEffect(() => {
    let timeout = window.setTimeout(onReset, IDLE_TIMEOUT_MS);

    const arm = () => {
      window.clearTimeout(timeout);
      timeout = window.setTimeout(onReset, IDLE_TIMEOUT_MS);
    };

    const events: Array<keyof WindowEventMap> = ['pointerdown', 'pointermove', 'keydown', 'scroll'];
    events.forEach((eventName) => window.addEventListener(eventName, arm, { passive: true }));

    return () => {
      window.clearTimeout(timeout);
      events.forEach((eventName) => window.removeEventListener(eventName, arm));
    };
  }, [onReset]);
}
