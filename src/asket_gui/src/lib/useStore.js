import { useSyncExternalStore } from 'react';

export function useStore(connection) {
  return useSyncExternalStore(connection.subscribeToStore, connection.getSnapshot);
}
