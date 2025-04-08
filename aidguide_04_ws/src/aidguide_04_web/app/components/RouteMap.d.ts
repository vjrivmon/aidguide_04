import { FC } from 'react';

interface RouteMapProps {
  waypoints: string[];
  routeName: string;
  pgmMapUrl?: string;
  pgmYamlUrl?: string;
}

declare const RouteMap: FC<RouteMapProps>;

export default RouteMap; 