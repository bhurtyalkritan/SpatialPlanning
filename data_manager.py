# data_manager.py

import json
import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union
from typing import Dict, Any, List, Tuple


def load_no_fly_zones(buffer_dist=0.001) -> shapely.geometry.MultiPolygon:
    """
    Loads data/no_fly_zones.geojson, buffers polygons to enforce
    safe distance. Returns a union of all polygons.
    """
    with open("data/no_fly_zones.geojson", "r") as f:
        data = json.load(f)

    polygons = []
    for feature in data["features"]:
        coords = feature["geometry"]["coordinates"][0]  # outer ring
        # Flip to (lat, lon)
        poly_coords = [(lat, lon) for lon, lat in coords]
        p = Polygon(poly_coords).buffer(buffer_dist)
        polygons.append(p)
    return unary_union(polygons)


def load_buildings() -> Dict[str, Any]:
    """
    Returns building footprints as a GeoJSON dict.
    Each feature can have 'HEIGHT' property for 3D representation.
    """
    with open("data/buildings.geojson", "r") as f:
        data = json.load(f)
    return data


def load_landing_zones() -> Dict[str, Any]:
    """
    Returns landing zones as a GeoJSON FeatureCollection of points.
    """
    with open("data/landing_zones.geojson", "r") as f:
        data = json.load(f)
    return data


def load_elevations() -> List[Tuple[float, float, float]]:
    """
    Returns a list of (lat, lon, elev).
    For real usage, you'd parse a DEM or multiple spot points.
    """
    with open("data/topography_spot_elevations.geojson", "r") as f:
        data = json.load(f)

    spots = []
    for feat in data["features"]:
        lon, lat, elev = feat["geometry"]["coordinates"]
        spots.append((lat, lon, elev))
    return spots


def get_mock_weather_data() -> Dict[str, Any]:
    """
    Simulate a weather front as a simple polygon GeoJSON
    that can be displayed on the map.
    """
    # Just a sample polygon near DC
    weather_geojson = {
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature",
            "properties": {"description": "Rain area"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [[
                    [-77.05, 38.85],
                    [-77.02, 38.85],
                    [-77.02, 38.88],
                    [-77.05, 38.88],
                    [-77.05, 38.85]
                ]]
            }
        }]
    }
    return weather_geojson


def get_mock_air_traffic(num_agents=2) -> List[Dict]:
    """
    Simulate other drones in the air (multi-agent).
    Return a list of agent dicts with positions, alt, etc.
    Could treat them as moving no-fly zones or dynamic obstacles.
    """
    # Hard-code some sample positions in DC area
    traffic = []
    lat0, lon0 = 38.90, -77.02
    for i in range(num_agents):
        lat_r = lat0 + (0.01 * i)
        lon_r = lon0 - (0.01 * i)
        traffic.append({
            "id": i + 1,
            "location": (lat_r, lon_r),
            "alt": 80,  # example altitude
        })
    return traffic
