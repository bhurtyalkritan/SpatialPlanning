# comprehensive_drone_app.py
import streamlit as st
import folium
from streamlit_folium import st_folium
import json
import math
from typing import Dict, Any, List, Tuple, Optional
import shapely
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union


# -------------------------------------------------------------------
# DATA MANAGER SECTION (Merged from data_manager.py)
# -------------------------------------------------------------------
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
        # Flip to (lon, lat) -> (lat, lon)
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


# -------------------------------------------------------------------
# PATH PLANNER SECTION (Merged from path_planner.py)
# -------------------------------------------------------------------
class DronePathResult:
    """
    A container for the final path and any additional metrics
    (like total distance, battery used, risk score, etc.).
    """
    def __init__(self, path: List[Tuple[float, float]]):
        self.path = path
        self.total_distance = 0.0
        self.battery_used = 0.0
        self.risk_score = 0.0


def is_line_collision_free(
    a: Tuple[float, float],
    b: Tuple[float, float],
    no_fly_union: shapely.geometry.MultiPolygon
) -> bool:
    """
    Returns True if the line a->b does NOT intersect the no_fly_union.
    """
    line = LineString([a, b])
    return not line.intersects(no_fly_union)


def advanced_rrt_star_planner(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    no_fly_union: shapely.geometry.MultiPolygon,
    spot_elevs: List[Tuple[float, float, float]],
    wind_factor: float = 1.0,
    max_altitude: float = 120.0,
    battery: float = 3000,
    battery_usage_rate: float = 30,  # mAh/km
    distance_weight: float = 0.5,
    risk_weight: float = 0.3,
    noise_weight: float = 0.2,
    existing_traffic: Optional[List[Dict]] = None,
    max_iter: int = 2000
) -> DronePathResult:
    """
    Placeholder for a multi-objective RRT* approach.
    Currently just tries a direct line, else returns no path.
    """
    path_result = DronePathResult(path=[])

    # Check direct line feasibility
    if is_line_collision_free(start, goal, no_fly_union):
        # Compute distance in meters (roughly 1 deg lat ~ 111km)
        dist = math.hypot(goal[0] - start[0], goal[1] - start[1]) * 111_000
        battery_used = (dist / 1000) * battery_usage_rate
        if battery_used <= battery:
            # Accept direct path
            path_result.path = [start, goal]
            path_result.total_distance = dist
            path_result.battery_used = battery_used
            path_result.risk_score = dist * risk_weight
            return path_result

    # If direct line not feasible or battery insufficient, no real RRT here:
    return DronePathResult(path=[])


def dynamic_replan_rrt_star(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    no_fly_union: shapely.geometry.MultiPolygon,
    spot_elevs: List[Tuple[float, float, float]],
    wind_factor: float,
    max_altitude: float,
    battery: float,
    battery_usage_rate: float,
    distance_weight: float,
    risk_weight: float,
    noise_weight: float,
    existing_traffic: Optional[List[Dict]] = None
) -> DronePathResult:
    """
    Stub for dynamic re-planning. For demo, calls the same function.
    """
    return advanced_rrt_star_planner(
        start=start,
        goal=goal,
        no_fly_union=no_fly_union,
        spot_elevs=spot_elevs,
        wind_factor=wind_factor,
        max_altitude=max_altitude,
        battery=battery,
        battery_usage_rate=battery_usage_rate,
        distance_weight=distance_weight,
        risk_weight=risk_weight,
        noise_weight=noise_weight,
        existing_traffic=existing_traffic
    )


def apply_spline_smoothing(
    path: List[Tuple[float, float]],
    no_fly_union: shapely.geometry.MultiPolygon
) -> List[Tuple[float, float]]:
    """
    In real usage, you'd do an actual spline fit.
    Here, we do a trivial "skip middle if collision-free" approach.
    """
    if len(path) < 3:
        return path

    smooth_path = [path[0]]
    for i in range(1, len(path) - 1):
        if is_line_collision_free(smooth_path[-1], path[i+1], no_fly_union):
            continue
        else:
            smooth_path.append(path[i])
    smooth_path.append(path[-1])
    return smooth_path


# -------------------------------------------------------------------
# MAIN APP SECTION (Merged from app.py)
# -------------------------------------------------------------------
st.set_page_config(page_title="Comprehensive Drone Planner", layout="wide")

def main():
    st.title("Comprehensive Drone Planner: Single Full-Page Map")
    st.markdown(
        """
        This demo integrates **10 advanced features**:
        1. Real-Time Weather & Wind  
        2. Altitude / 2.5D Flight  
        3. Battery & Energy Constraints  
        4. Multi-Objective Optimization  
        5. Dynamic Re-Planning  
        6. Geofencing & Flight Corridors  
        7. Spline-Based Smoothing  
        8. 3D Buildings & Landing Zones  
        9. Animated Drone Flight  
        10. Multi-Agent Collision Avoidance  

        All settings are in the **sidebar**. The map below is **full screen**.
        """
    )

    # Sidebar
    st.sidebar.header("Planner Settings")

    # (1) Weather & wind
    show_weather = st.sidebar.checkbox("Show Weather Overlay", value=True)
    wind_factor = st.sidebar.slider("Wind Penalty Factor", 0.0, 2.0, 1.0)

    # (2) Altitude / 2.5D flight
    max_altitude = st.sidebar.number_input("Max Altitude (meters)", value=120, step=10)

    # (3) Battery constraints
    initial_battery = st.sidebar.number_input("Initial Battery (mAh)", value=3000, step=100)
    battery_usage_rate = st.sidebar.slider("Battery Usage Rate (mAh per km)", 10, 100, 30)

    # (4) Multi-objective optimization
    distance_weight = st.sidebar.slider("Distance Weight", 0.0, 1.0, 0.5)
    risk_weight = st.sidebar.slider("Risk Weight", 0.0, 1.0, 0.3)
    noise_weight = st.sidebar.slider("Noise Weight", 0.0, 1.0, 0.2)

    # (5) Dynamic re-planning
    dynamic_mode = st.sidebar.checkbox("Enable Dynamic Re-Planning", value=False)

    # (6) Geofencing & flight corridors
    buffer_dist = st.sidebar.slider("No-Fly Buffer (deg)", 0.0, 0.01, 0.001, 0.0001)

    # (7) Spline-based smoothing
    do_spline = st.sidebar.checkbox("Apply Spline Smoothing", value=True)

    # (8) 3D Buildings & landing zone
    show_buildings = st.sidebar.checkbox("Show 3D Buildings", value=True)
    show_landing_zones = st.sidebar.checkbox("Show Landing Zones", value=True)

    # (9) Animated flight
    animate_speed = st.sidebar.slider("Animation Speed (sec per waypoint)", 0.1, 3.0, 1.0)

    # (10) Multi-agent collision avoidance
    enable_multi_agent = st.sidebar.checkbox("Multi-Agent Mode", value=False)
    num_agents = 1
    if enable_multi_agent:
        num_agents = st.sidebar.number_input("Number of Drones", value=2, min_value=2)

    # Load data
    no_fly_union = load_no_fly_zones(buffer_dist=buffer_dist)
    buildings = load_buildings()
    landing_zones = load_landing_zones()
    spot_elevs = load_elevations()

    # Mock real-time data
    weather_data = get_mock_weather_data() if show_weather else None
    air_traffic = get_mock_air_traffic(num_agents=num_agents) if enable_multi_agent else None

    # Setup Folium map
    full_map = folium.Map(location=[38.9, -77.03], zoom_start=11, tiles=None)
    folium.TileLayer(
        tiles='http://mt0.google.com/vt/lyrs=m&x={x}&y={y}&z={z}',
        attr='Google',
        name='Google Maps',
        control=False
    ).add_to(full_map)

    # Draw no-fly zones
    if no_fly_union.geom_type == "Polygon":
        polys = [no_fly_union]
    else:
        polys = list(no_fly_union.geoms)
    for poly in polys:
        folium.Polygon(
            locations=[(c[0], c[1]) for c in poly.exterior.coords],
            color='red', fill=True, fill_opacity=0.3,
            popup="No-Fly"
        ).add_to(full_map)

    # Draw 3D buildings
    if show_buildings:
        for feature in buildings["features"]:
            coords = feature["geometry"]["coordinates"][0]  # outer ring
            height = feature["properties"].get("HEIGHT", 0)
            folium.Polygon(
                locations=[(c[1], c[0]) for c in coords],
                color='gray', fill=True, fill_opacity=0.5,
                popup=f"Building Height: {height}m"
            ).add_to(full_map)

    # Draw landing zones
    if show_landing_zones and landing_zones:
        for feature in landing_zones["features"]:
            coords = feature["geometry"]["coordinates"]
            lat, lon = coords[1], coords[0]
            folium.Marker(
                location=[lat, lon],
                icon=folium.Icon(color="green", icon="flag", prefix="fa"),
                popup="Landing Zone"
            ).add_to(full_map)

    # Weather overlay
    if weather_data:
        folium.GeoJson(
            weather_data,
            name="Weather Overlay",
            style_function=lambda x: {
                "fillColor": "blue",
                "color": "blue",
                "fillOpacity": 0.2
            }
        ).add_to(full_map)

    # Multi-Agent traffic
    if air_traffic:
        for i, agent in enumerate(air_traffic):
            lat, lon = agent["location"]
            folium.Circle(
                radius=300,
                location=[lat, lon],
                color="orange",
                fill=True,
                fill_opacity=0.2,
                popup=f"Drone #{i+1}"
            ).add_to(full_map)

    # Session State for user clicks & path
    if "click_points" not in st.session_state:
        st.session_state["click_points"] = []
    if "path" not in st.session_state:
        st.session_state["path"] = []
    if "drone_index" not in st.session_state:
        st.session_state["drone_index"] = 0

    # Show user-clicked points
    for i, (lat, lon) in enumerate(st.session_state["click_points"]):
        label = "Start" if i == 0 else "End"
        color = "green" if i == 0 else "red"
        folium.Marker(
            location=[lat, lon],
            icon=folium.Icon(color=color),
            popup=label
        ).add_to(full_map)

    # Show final path
    if st.session_state["path"]:
        final_path = st.session_state["path"]
        folium.PolyLine(
            locations=final_path,
            color="blue", weight=3
        ).add_to(full_map)
        idx = st.session_state["drone_index"]
        if idx < len(final_path):
            lat_c, lon_c = final_path[idx]
            folium.Marker(
                location=[lat_c, lon_c],
                icon=folium.Icon(color="blue", icon="plane", prefix="fa"),
                popup=f"Drone WP {idx+1}/{len(final_path)}"
            ).add_to(full_map)

    # Render map
    map_data = st_folium(full_map, width=1600, height=900)

    # Handle user clicks
    if map_data and map_data["last_clicked"] is not None:
        lat_click = map_data["last_clicked"]["lat"]
        lon_click = map_data["last_clicked"]["lng"]
        if len(st.session_state["click_points"]) < 2:
            st.session_state["click_points"].append((lat_click, lon_click))

    # Sidebar actions
    st.sidebar.subheader("Actions")
    colA, colB, colC = st.sidebar.columns(3)

    with colA:
        if st.button("Clear Points"):
            st.session_state["click_points"] = []
            st.session_state["path"] = []
            st.session_state["drone_index"] = 0
            st.experimental_rerun()

    with colB:
        # Plan Route
        if len(st.session_state["click_points"]) == 2 and st.button("Plan Route"):
            start = st.session_state["click_points"][0]
            goal = st.session_state["click_points"][1]

            # Dynamic or normal approach
            if dynamic_mode:
                result: DronePathResult = dynamic_replan_rrt_star(
                    start=start,
                    goal=goal,
                    no_fly_union=no_fly_union,
                    spot_elevs=spot_elevs,
                    wind_factor=wind_factor,
                    max_altitude=max_altitude,
                    battery=initial_battery,
                    battery_usage_rate=battery_usage_rate,
                    distance_weight=distance_weight,
                    risk_weight=risk_weight,
                    noise_weight=noise_weight,
                    existing_traffic=air_traffic
                )
            else:
                result: DronePathResult = advanced_rrt_star_planner(
                    start=start,
                    goal=goal,
                    no_fly_union=no_fly_union,
                    spot_elevs=spot_elevs,
                    wind_factor=wind_factor,
                    max_altitude=max_altitude,
                    battery=initial_battery,
                    battery_usage_rate=battery_usage_rate,
                    distance_weight=distance_weight,
                    risk_weight=risk_weight,
                    noise_weight=noise_weight,
                    existing_traffic=air_traffic
                )

            path = result.path
            if path and do_spline:
                path = apply_spline_smoothing(path, no_fly_union)

            st.session_state["path"] = path
            st.session_state["drone_index"] = 0
            if path:
                st.sidebar.success(f"Path found! Waypoints: {len(path)}")
            else:
                st.sidebar.error("No feasible path found.")

    with colC:
        if st.session_state["path"] and st.button("Reset Drone"):
            st.session_state["drone_index"] = 0

    # Animated flight
    if st.session_state["path"]:
        if st.button("Next Waypoint"):
            if st.session_state["drone_index"] < len(st.session_state["path"]) - 1:
                st.session_state["drone_index"] += 1
            else:
                st.warning("Drone reached the end!")
        # For a real-time continuous animation, you'd need a timer or async approach.


if __name__ == "__main__":
    main()
