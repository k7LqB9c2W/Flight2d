import pygame
import math
import httpx
import asyncio
import io
import threading
import queue
import collections
import time

# --- Constants ---
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
WHITE, BLACK, RED, GREEN = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0)
SR71_COLOR = (20, 20, 20)
INPUT_BOX_COLOR = (220, 220, 220)
INPUT_TEXT_COLOR = (10, 10, 10)
PLACEHOLDER_COLOR = (50, 50, 50)

# --- Map Tiling Configuration ---
TILE_SIZE = 256
TILE_URL = "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

# --- Performance Tuning ---
CONCURRENT_DOWNLOADS = 300
MAX_CACHE_SIZE_MB = 500
TILE_REQUEST_INTERVAL = 0.1

# --- Plane Physics ---
NORMAL_MAX_SPEED_MPH = 588
SR71_MAX_SPEED_MPH = 2193
THROTTLE_INCREMENT_MPH = 100
TURN_SPEED_DEGREES = 60

# --- Geographic Conversion Constants ---
MILES_PER_DEGREE_LATITUDE = 69.0
EARTH_RADIUS_MILES = 3958.8

# --- Game State Management ---
STATE_MANUAL_FLIGHT = "manual_flight"
STATE_ENTERING_TEXT = "entering_text"
STATE_AUTOPILOT_ON = "autopilot_on"
STATE_CHANGING_ZOOM = "changing_zoom"

# --- Preset Destinations ---
PRESET_CITIES = {
    "New York":    (40.7128, -74.0060),
    "London":      (51.5074, -0.1278),
    "Tokyo":       (35.6895, 139.6917),
    "Paris":       (48.8566, 2.3522),
    "Sydney":      ( -33.8688, 151.2093),
    "Cairo":       (30.0444, 31.2357),
    "Rio de Janeiro": (-22.9068, -43.1729),
    "Moscow":      (55.7558, 37.6173),
    "Beijing":     (39.9042, 116.4074),
    "Los Angeles": (34.0522, -118.2437)
}

# --- Helper Functions ---
def lonlat_to_world_pixels(lon, lat, zoom):
    scale = 2**zoom * TILE_SIZE
    x = (lon + 180.0) / 360.0 * scale
    lat_rad = math.radians(lat)
    y = (1 - math.log(math.tan(lat_rad) + 1/math.cos(lat_rad)) / math.pi) / 2 * scale
    return x, y

def get_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLon = lon2 - lon1
    x = math.sin(dLon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    initial_bearing = math.atan2(x, y)
    return (math.degrees(initial_bearing) + 360) % 360

def get_great_circle_waypoints(lat1, lon1, lat2, lon2, step_miles=20):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dLat = lat2 - lat1
    dLon = lon2 - lon1
    a = math.sin(dLat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    total_distance = EARTH_RADIUS_MILES * c
    if total_distance < step_miles: return []
    num_steps = int(total_distance / step_miles)
    waypoints = []
    for i in range(1, num_steps):
        f = i / num_steps
        A = math.sin((1-f)*c) / math.sin(c)
        B = math.sin(f*c) / math.sin(c)
        x = A * math.cos(lat1) * math.cos(lon1) + B * math.cos(lat2) * math.cos(lon2)
        y = A * math.cos(lat1) * math.sin(lon1) + B * math.cos(lat2) * math.sin(lon2)
        z = A * math.sin(lat1) + B * math.sin(lat2)
        lat_i = math.atan2(z, math.sqrt(x**2 + y**2))
        lon_i = math.atan2(y, x)
        waypoints.append((math.degrees(lat_i), math.degrees(lon_i)))
    return waypoints

# NEW: Helper function to calculate distance for the HUD
def get_distance_miles(lat1, lon1, lat2, lon2):
    """Calculate the great-circle distance in miles between two points."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_MILES * c

## --- ASYNCIO DOWNLOADER SYSTEM ---
async def tile_downloader_worker(session, q_in, q_out, status, status_lock, loop):
    while True:
        try:
            priority, tile_key = await q_in.get()
            if tile_key is None:
                q_in.task_done()
                break
            
            z, x, y = tile_key
            url = TILE_URL.format(z=z, x=x, y=y)
            response = await session.get(url, timeout=10)
            response.raise_for_status()
            loop.call_soon_threadsafe(q_out.put, (tile_key, response.content, priority))
        except Exception as e:
            # print(f"!!! Download FAILED for tile {tile_key}: {e}") # Optional: uncomment for verbose error logging
            if priority == 1:
                with status_lock:
                    if 'completed' in status: status['completed'] += 1
        finally:
            q_in.task_done()

async def asyncio_downloader_manager(q_in, q_out, status, status_lock, shutdown_event):
    loop = asyncio.get_running_loop()
    limits = httpx.Limits(max_connections=CONCURRENT_DOWNLOADS, max_keepalive_connections=20)
    headers = { 'User-Agent': 'MyFlightSim/1.0 (compatible; Mozilla/5.0)', 'Referer': 'https://www.arcgis.com/apps/mapviewer/index.html' }
    async with httpx.AsyncClient(http2=True, limits=limits, headers=headers) as session:
        tasks = [asyncio.create_task(tile_downloader_worker(session, q_in, q_out, status, status_lock, loop)) for _ in range(CONCURRENT_DOWNLOADS)]
        await shutdown_event.wait()
        for task in tasks: task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

def run_asyncio_loop(loop_ready_event, q_in, q_out, status, status_lock, shutdown_event):
    global asyncio_loop
    asyncio_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(asyncio_loop)
    loop_ready_event.set()
    asyncio_loop.run_until_complete(asyncio_downloader_manager(q_in, q_out, status, status_lock, shutdown_event))
    asyncio_loop.close()
    print("[ASYNC_RUNNER] Downloader thread has shut down cleanly.")

## --- Autopilot and Progress Reporting Threads ---
def autopilot_precacher(q, cancel_event, status, status_lock, start_lat, start_lon, end_lat, end_lon, z):
    print("Calculating autopilot route for pre-caching...")
    waypoints = get_great_circle_waypoints(start_lat, start_lon, end_lat, end_lon)
    if not waypoints: print("Destination is too close, no pre-caching needed."); return
    tiles_to_cache = set()
    corridor_width = 4
    for lat, lon in waypoints:
        if cancel_event.is_set(): print("Autopilot pre-caching planning cancelled."); return
        world_px, world_py = lonlat_to_world_pixels(lon, lat, z)
        center_tx, center_ty = int(world_px // TILE_SIZE), int(world_py // TILE_SIZE)
        for tx in range(center_tx - corridor_width, center_tx + corridor_width + 1):
            for ty in range(center_ty - corridor_width, center_ty + corridor_width + 1):
                tiles_to_cache.add((z, tx, ty))
    with status_lock:
        status['total'] = len(tiles_to_cache); status['completed'] = 0; status['active'] = True
    print(f"Route calculated. Queuing {status['total']} tiles for download.")
    for tile_key in tiles_to_cache:
        if cancel_event.is_set(): print("Autopilot pre-caching queuing cancelled."); return
        asyncio_loop.call_soon_threadsafe(q.put_nowait, (1, tile_key))

def progress_reporter(cancel_event, status, status_lock):
    while not cancel_event.is_set():
        time.sleep(2)
        with status_lock:
            if not status.get('active'): continue
            total, completed = status.get('total', 0), status.get('completed', 0)
            if total > 0:
                percent = (completed / total) * 100
                print(f"Pre-cache progress: {completed} / {total} tiles ({percent:.1f}%) downloaded.")
                if completed >= total:
                    print("Autopilot pre-caching download complete."); status['active'] = False; break

# --- Game Setup ---
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Real World Flight Simulator with Autopilot")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 20)
font_large = pygame.font.SysFont("consolas", 24, bold=True)
font_zoom = pygame.font.SysFont("consolas", 80, bold=True)
player_screen_pos = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
player_lon, player_lat = -118.2437, 34.0522
player_angle, current_speed_mph = 0, 0.0
sr71_mode, time_multiplier = False, 1.0
game_state = STATE_MANUAL_FLIGHT
autopilot_off_flash_timer = 0.0
zoom_level, selected_zoom_level = 8, 8
destination_text, destination_name = "", ""
dest_lat, dest_lon = None, None
input_error_msg, input_error_timer = "", 0.0
precaching_thread, reporter_thread, precaching_cancel_event = None, None, None
precaching_status, status_lock = {}, threading.Lock()
tile_cache = collections.OrderedDict()
cache_lock = threading.Lock()
current_cache_size_bytes = [0]
download_queue = asyncio.PriorityQueue()
processing_queue = queue.Queue()
pending_downloads = set()
placeholder_tile = pygame.Surface((TILE_SIZE, TILE_SIZE)); placeholder_tile.fill(PLACEHOLDER_COLOR)
time_since_last_tile_check = 0.0
asyncio_loop = None
loop_ready = threading.Event()
shutdown_event = asyncio.Event()

downloader_thread = threading.Thread(
    target=run_asyncio_loop,
    args=(loop_ready, download_queue, processing_queue, precaching_status, status_lock, shutdown_event),
    daemon=True
)
downloader_thread.start()
print("[MAIN] Waiting for asyncio loop to be ready...")
loop_ready.wait()
print("[MAIN] Asyncio loop is ready. Starting game loop.")

running = True
start_time = pygame.time.get_ticks()

while running:
    dt = clock.tick(60) / 1000.0
    effective_dt = dt * time_multiplier

    # --- Smarter Caching Fix: Part 1 ---
    # Calculate which tiles are currently visible on screen. This set will be used
    # to protect visible tiles from being evicted from the cache.
    player_world_px, player_world_py = lonlat_to_world_pixels(player_lon, player_lat, zoom_level)
    top_left_world_px, top_left_world_py = player_world_px - SCREEN_WIDTH / 2, player_world_py - SCREEN_HEIGHT / 2
    draw_tx_min, draw_ty_min = int(top_left_world_px // TILE_SIZE), int(top_left_world_py // TILE_SIZE)
    draw_tx_max, draw_ty_max = int((top_left_world_px + SCREEN_WIDTH) // TILE_SIZE), int((top_left_world_py + SCREEN_HEIGHT) // TILE_SIZE)
    visible_tiles = set()
    for tx in range(draw_tx_min, draw_tx_max + 2):
        for ty in range(draw_ty_min, draw_ty_max + 2):
            visible_tiles.add((zoom_level, tx, ty))

# --- CORRECTED (SMARTER) CODE ---
    while not processing_queue.empty():
        try:
            tile_key, image_bytes, priority = processing_queue.get_nowait()
            tile_surface = pygame.image.load(io.BytesIO(image_bytes)).convert()
            tile_size_bytes = tile_surface.get_bytesize()

            with cache_lock:
                # Do not process tiles for a different zoom level
                if tile_key[0] != zoom_level: continue
                
                # Add the new tile to the cache
                tile_cache[tile_key] = (tile_surface, tile_size_bytes)
                current_cache_size_bytes[0] += tile_size_bytes
                
                # --- Smarter Caching Eviction Logic ---
                # Evict tiles only if the cache is full.
                while current_cache_size_bytes[0] > MAX_CACHE_SIZE_MB * 1024 * 1024:
                    # Peek at the oldest key without removing it
                    oldest_key = next(iter(tile_cache))
                    
                    # If the oldest tile is currently visible on screen, we MUST NOT evict it.
                    # Instead, mark it as recently used and check the next oldest one.
                    if oldest_key in visible_tiles:
                        tile_cache.move_to_end(oldest_key)
                        # This prevents an infinite loop if all cached items are visible
                        # by ensuring we eventually find a non-visible one to evict.
                        continue 
                    
                    # If the oldest tile is NOT visible, it's safe to evict.
                    _key, (evicted_surface, evicted_size) = tile_cache.popitem(last=False)
                    current_cache_size_bytes[0] -= evicted_size

            if priority == 1: # This is a pre-cached tile
                with status_lock:
                    if 'completed' in precaching_status: precaching_status['completed'] += 1
        except queue.Empty:
            break
        except Exception as e:
            key_for_print = "unknown";
            if 'tile_key' in locals(): key_for_print = tile_key
            print(f"!!! CRITICAL: Error processing tile {key_for_print} in main thread: {e}")

    # --- Event Handling ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if game_state == STATE_CHANGING_ZOOM:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP: selected_zoom_level = min(23, selected_zoom_level + 1)
                elif event.key == pygame.K_DOWN: selected_zoom_level = max(0, selected_zoom_level - 1)
                elif event.key == pygame.K_RETURN:
                    if zoom_level != selected_zoom_level:
                        zoom_level = selected_zoom_level;
                        with cache_lock: tile_cache.clear(); pending_downloads.clear(); current_cache_size_bytes[0] = 0
                    game_state = STATE_MANUAL_FLIGHT
                elif event.key == pygame.K_ESCAPE: selected_zoom_level = zoom_level; game_state = STATE_MANUAL_FLIGHT
        elif game_state == STATE_ENTERING_TEXT:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    if destination_text:
                        try:
                            with httpx.Client(headers={'User-Agent': 'MyFlightSim/1.0'}) as client:
                                url = f"https://nominatim.openstreetmap.org/search?q={destination_text}&format=json&limit=1"
                                response = client.get(url, timeout=5).json()
                            if response:
                                dest_lat, dest_lon = float(response[0]['lat']), float(response[0]['lon'])
                                destination_name = response[0]['display_name'].split(',')[0]
                                game_state = STATE_AUTOPILOT_ON
                            else: input_error_msg, input_error_timer = "City not found!", 2.0
                        except Exception as e: input_error_msg, input_error_timer = f"Network Error: {e}", 2.0
                elif event.key == pygame.K_BACKSPACE: destination_text = destination_text[:-1]
                elif event.key == pygame.K_ESCAPE: game_state = STATE_MANUAL_FLIGHT; destination_text = ""
                elif pygame.K_F1 <= event.key <= pygame.K_F10:
                    city_index = event.key - pygame.K_F1
                    city_list = list(PRESET_CITIES.keys())
                    if city_index < len(city_list):
                        destination_name = city_list[city_index]
                        dest_lat, dest_lon = PRESET_CITIES[destination_name]
                        game_state = STATE_AUTOPILOT_ON
                else: destination_text += event.unicode
        else:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_z: game_state = STATE_CHANGING_ZOOM; selected_zoom_level = zoom_level
                elif event.key == pygame.K_1: sr71_mode = not sr71_mode
                elif event.key == pygame.K_2: time_multiplier = 2.0
                elif event.key == pygame.K_3: time_multiplier = 5.0
                elif event.key == pygame.K_4: time_multiplier = 10.0
                elif event.key == pygame.K_5 and game_state == STATE_MANUAL_FLIGHT:
                    game_state = STATE_ENTERING_TEXT; destination_text = ""; input_error_msg = ""
                elif event.key == pygame.K_BACKSPACE:
                    if game_state == STATE_AUTOPILOT_ON:
                        game_state = STATE_MANUAL_FLIGHT; autopilot_off_flash_timer = 1.5
                        if precaching_thread and precaching_thread.is_alive(): precaching_cancel_event.set()
                        print("Cancelled pre-caching.")
                    else: time_multiplier = 1.0

    if game_state == STATE_AUTOPILOT_ON and not precaching_thread:
        precaching_cancel_event = threading.Event();
        with status_lock: precaching_status.clear()
        precaching_thread = threading.Thread(target=autopilot_precacher, args=(download_queue, precaching_cancel_event, precaching_status, status_lock, player_lat, player_lon, dest_lat, dest_lon, zoom_level), daemon=True)
        reporter_thread = threading.Thread(target=progress_reporter, args=(precaching_cancel_event, precaching_status, status_lock), daemon=True)
        precaching_thread.start(); reporter_thread.start()
    if game_state != STATE_AUTOPILOT_ON and precaching_thread:
        if precaching_cancel_event and not precaching_cancel_event.is_set(): precaching_cancel_event.set()
        precaching_thread, reporter_thread, precaching_cancel_event = None, None, None

    # --- Game Logic (Physics) ---
    if game_state in [STATE_MANUAL_FLIGHT, STATE_AUTOPILOT_ON]:
        current_max_speed = SR71_MAX_SPEED_MPH if sr71_mode else NORMAL_MAX_SPEED_MPH
        plane_name = "SR-71 Blackbird" if sr71_mode else "Boeing 737"
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]: current_speed_mph += THROTTLE_INCREMENT_MPH * effective_dt
        if keys[pygame.K_DOWN]: current_speed_mph -= THROTTLE_INCREMENT_MPH * effective_dt
        current_speed_mph = max(0, min(current_speed_mph, current_max_speed))
        if game_state == STATE_AUTOPILOT_ON:
            target_angle = get_bearing(player_lat, player_lon, dest_lat, dest_lon)
            angle_diff = (target_angle - player_angle + 180) % 360 - 180
            if abs(angle_diff) > 1: player_angle += (1 if angle_diff > 0 else -1) * TURN_SPEED_DEGREES * effective_dt
        else:
            if keys[pygame.K_a]: player_angle -= TURN_SPEED_DEGREES * effective_dt
            if keys[pygame.K_d]: player_angle += TURN_SPEED_DEGREES * effective_dt
        player_angle %= 360
        miles_moved = (current_speed_mph / 3600.0) * effective_dt
        rad_angle = math.radians(player_angle)
        d_lat = (math.cos(rad_angle) * miles_moved) / MILES_PER_DEGREE_LATITUDE
        miles_per_degree_lon = MILES_PER_DEGREE_LATITUDE * math.cos(math.radians(player_lat))
        d_lon = (math.sin(rad_angle) * miles_moved) / miles_per_degree_lon if miles_per_degree_lon > 0 else 0
        player_lat, player_lon = player_lat + d_lat, player_lon + d_lon

    # --- Drawing ---
    screen.fill(BLACK)
    if game_state in [STATE_MANUAL_FLIGHT, STATE_AUTOPILOT_ON, STATE_CHANGING_ZOOM]:
        dynamic_buffer = 2 + int(current_speed_mph / 1500)
        time_since_last_tile_check += dt
        if time_since_last_tile_check > TILE_REQUEST_INTERVAL:
            time_since_last_tile_check = 0.0
            req_tx_min, req_ty_min = int(top_left_world_px // TILE_SIZE) - dynamic_buffer, int(top_left_world_py // TILE_SIZE) - dynamic_buffer
            req_tx_max, req_ty_max = int((top_left_world_px + SCREEN_WIDTH) // TILE_SIZE) + dynamic_buffer, int((top_left_world_py + SCREEN_HEIGHT) // TILE_SIZE) + dynamic_buffer
            with cache_lock: completed_downloads = pending_downloads.intersection(tile_cache.keys())
            pending_downloads.difference_update(completed_downloads)
            for tx in range(req_tx_min, req_tx_max + 1):
                for ty in range(req_ty_min, req_ty_max + 1):
                    tile_key = (zoom_level, tx, ty)
                    with cache_lock: is_in_cache = tile_key in tile_cache
                    if not is_in_cache and tile_key not in pending_downloads:
                        pending_downloads.add(tile_key)
                        asyncio_loop.call_soon_threadsafe(download_queue.put_nowait, (0, tile_key))
        
        with cache_lock:
            for tile_key in visible_tiles:
                surface_to_draw = placeholder_tile
                if tile_key in tile_cache:
                    surface_to_draw = tile_cache[tile_key][0]
                    tile_cache.move_to_end(tile_key)
                tx, ty = tile_key[1], tile_key[2]
                screen.blit(surface_to_draw, (tx * TILE_SIZE - top_left_world_px, ty * TILE_SIZE - top_left_world_py))
        
        plane_points = [(player_screen_pos[0], player_screen_pos[1] - 15), (player_screen_pos[0] - 10, player_screen_pos[1] + 10), (player_screen_pos[0] + 10, player_screen_pos[1] + 10)]
        rad_draw_angle = math.radians(player_angle)
        rotated_points = []
        for x, y in plane_points:
            x_r, y_r = x - player_screen_pos[0], y - player_screen_pos[1]
            new_x = x_r * math.cos(rad_draw_angle) - y_r * math.sin(rad_draw_angle) + player_screen_pos[0]
            new_y = x_r * math.sin(rad_draw_angle) + y_r * math.cos(rad_draw_angle) + player_screen_pos[1]
            rotated_points.append((new_x, new_y))
        pygame.draw.polygon(screen, RED if not sr71_mode else SR71_COLOR, rotated_points)
    
    # --- UI Panels ---
    if game_state == STATE_ENTERING_TEXT:
        s = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA); s.fill((0,0,0,180)); screen.blit(s, (0,0))
        box_w, box_h = 800, 400; box_x, box_y = (SCREEN_WIDTH - box_w) / 2, (SCREEN_HEIGHT - box_h) / 2
        pygame.draw.rect(screen, INPUT_BOX_COLOR, (box_x, box_y, box_w, box_h), border_radius=10); pygame.draw.rect(screen, BLACK, (box_x, box_y, box_w, box_h), 3, border_radius=10)
        title_surf = font_large.render("Set Autopilot Destination", True, BLACK); screen.blit(title_surf, (box_x + 20, box_y + 20))
        input_field_rect = pygame.Rect(box_x + 20, box_y + 70, box_w - 40, 40); pygame.draw.rect(screen, WHITE, input_field_rect); pygame.draw.rect(screen, BLACK, input_field_rect, 2)
        screen.blit(font.render(destination_text, True, INPUT_TEXT_COLOR), (input_field_rect.x + 5, input_field_rect.y + 5))
        instr_y = box_y + 130; screen.blit(font.render("Type a city and press Enter, or select a preset:", True, BLACK), (box_x + 20, instr_y))
        for i, city in enumerate(list(PRESET_CITIES.keys())):
            col, row = i // 5, i % 5; screen.blit(font.render(f"F{i+1}: {city}", True, BLACK), (box_x + 40 + col * 350, instr_y + 40 + row * 30))
        if input_error_timer > 0: screen.blit(font.render(input_error_msg, True, RED), (box_x + 20, box_y + box_h - 40)); input_error_timer -= dt
    elif game_state == STATE_CHANGING_ZOOM:
        s = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA); s.fill((0,0,0,180)); screen.blit(s, (0,0))
        box_w, box_h = 600, 300; box_x, box_y = (SCREEN_WIDTH - box_w) / 2, (SCREEN_HEIGHT - box_h) / 2
        pygame.draw.rect(screen, INPUT_BOX_COLOR, (box_x, box_y, box_w, box_h), border_radius=10); pygame.draw.rect(screen, BLACK, (box_x, box_y, box_w, box_h), 3, border_radius=10)
        screen.blit(font_large.render("Set Zoom Level", True, BLACK), (box_x + 20, box_y + 20))
        zoom_text_surf = font_zoom.render(str(selected_zoom_level), True, INPUT_TEXT_COLOR); screen.blit(zoom_text_surf, zoom_text_surf.get_rect(center=(SCREEN_WIDTH / 2, box_y + 120)))
        instr1_surf = font.render("Use UP/DOWN arrows to change", True, BLACK); screen.blit(instr1_surf, instr1_surf.get_rect(center=(SCREEN_WIDTH / 2, box_y + box_h - 60)))
        instr2_surf = font.render("ENTER to confirm, ESC to cancel", True, BLACK); screen.blit(instr2_surf, instr2_surf.get_rect(center=(SCREEN_WIDTH / 2, box_y + box_h - 35)))

    # --- HUD ---
    if game_state in [STATE_MANUAL_FLIGHT, STATE_AUTOPILOT_ON]:
        elapsed_seconds = (pygame.time.get_ticks() - start_time) / 1000; h, rem = divmod(elapsed_seconds, 3600); m, s = divmod(rem, 60)
        y_pos = 10
        def draw_hud_line(text, y): screen.blit(font.render(text, True, WHITE), (10, y)); return y + 25
        y_pos = draw_hud_line(f"Mode: {plane_name}", y_pos); y_pos = draw_hud_line(f"Speed: {current_speed_mph:.0f} MPH", y_pos); y_pos = draw_hud_line(f"Lat: {player_lat:.4f}", y_pos); y_pos = draw_hud_line(f"Lon: {player_lon:.4f}", y_pos); y_pos = draw_hud_line(f"Time: {int(h):02}:{int(m):02}:{int(s):02}", y_pos); y_pos = draw_hud_line(f"Time Warp: {time_multiplier:.0f}x", y_pos); y_pos = draw_hud_line(f"Zoom: {zoom_level}", y_pos); y_pos = draw_hud_line(f"Cache: {current_cache_size_bytes[0] / (1024*1024):.1f}/{MAX_CACHE_SIZE_MB} MB", y_pos)
        if game_state == STATE_AUTOPILOT_ON:
            screen.blit(font.render(f"AUTOPILOT ON -> {destination_name}", True, GREEN), (10, y_pos))
            # NEW: Autopilot distance and ETA HUD
            dist_left = get_distance_miles(player_lat, player_lon, dest_lat, dest_lon)
            eta_str = "--:--:--"
            if current_speed_mph > 1:
                eta_hours = dist_left / current_speed_mph
                eta_h, rem = divmod(eta_hours * 3600, 3600); eta_m, eta_s = divmod(rem, 60)
                eta_str = f"{int(eta_h):02}:{int(eta_m):02}:{int(eta_s):02}"
            
            y_pos_right = 10
            def draw_hud_line_right(text, y):
                surf = font.render(text, True, GREEN)
                screen.blit(surf, surf.get_rect(topright=(SCREEN_WIDTH - 10, y)))
                return y + 25
            y_pos_right = draw_hud_line_right(f"Dist: {dist_left:.1f} mi", y_pos_right)
            y_pos_right = draw_hud_line_right(f"ETA: {eta_str}", y_pos_right)

        if autopilot_off_flash_timer > 0:
            if int(autopilot_off_flash_timer * 4) % 2 == 1: flash_surf = font_large.render("AUTOPILOT OFF", True, RED); screen.blit(flash_surf, flash_surf.get_rect(center=(SCREEN_WIDTH/2, SCREEN_HEIGHT/2)))
            autopilot_off_flash_timer -= dt

    pygame.display.flip()

# --- Graceful Shutdown ---
print("[MAIN] Exiting game loop. Shutting down...")
if precaching_cancel_event: precaching_cancel_event.set()
if precaching_thread and precaching_thread.is_alive(): precaching_thread.join(timeout=1.0)
if reporter_thread and reporter_thread.is_alive(): reporter_thread.join(timeout=1.0)
print("[MAIN] Sending shutdown signal to downloader...")
asyncio_loop.call_soon_threadsafe(shutdown_event.set)
print("[MAIN] Sending sentinel values to unblock workers...")
for _ in range(CONCURRENT_DOWNLOADS):
    asyncio_loop.call_soon_threadsafe(download_queue.put_nowait, (0, None))
print("[MAIN] Waiting for downloader thread to join...")
downloader_thread.join()
print("[MAIN] Downloader thread joined. Quitting Pygame.")
pygame.quit()
