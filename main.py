import pygame
import math
import requests
import io
import threading ## NEW ##
import queue     ## NEW ##

# --- Constants ---
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
WHITE, BLACK, RED = (255, 255, 255), (0, 0, 0), (255, 0, 0)
SR71_COLOR = (20, 20, 20)
INPUT_BOX_COLOR = (220, 220, 220)
INPUT_TEXT_COLOR = (10, 10, 10)
PLACEHOLDER_COLOR = (50, 50, 50) ## NEW ##

# --- Map Tiling Configuration ---
TILE_SIZE = 256
ZOOM_LEVEL = 8
TILE_URL = "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
PREDICTIVE_BUFFER = 2 ## NEW: How many extra tiles to load around the screen ##

# --- Plane Physics ---
NORMAL_MAX_SPEED_MPH = 588
SR71_MAX_SPEED_MPH = 2193
THROTTLE_INCREMENT_MPH = 100
TURN_SPEED_DEGREES = 60

# --- Geographic Conversion Constants ---
MILES_PER_DEGREE_LATITUDE = 69.0

# --- Game State Management ---
STATE_MANUAL_FLIGHT = "manual_flight"
STATE_ENTERING_TEXT = "entering_text"
STATE_AUTOPILOT_ON = "autopilot_on"

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

## NEW ## --- Tile Downloader Thread Function ---
def tile_downloader(q, cache):
    """A worker function to be run in a separate thread."""
    while True:
        tile_key = q.get()
        if tile_key is None: # Sentinel value to signal shutdown
            break
        
        z, x, y = tile_key
        url = TILE_URL.format(z=z, x=x, y=y)
        try:
            headers = {'User-Agent': 'MyFlightSim/1.0'}
            response = requests.get(url, headers=headers, timeout=10)
            response.raise_for_status()
            tile_surface = pygame.image.load(io.BytesIO(response.content)).convert()
            cache[tile_key] = tile_surface
        except Exception as e:
            # Optionally, you could put a "failed" tile in the cache
            # to prevent re-requesting it constantly. For now, we just print.
            print(f"Could not load tile {tile_key}: {e}")
        
        q.task_done()

# --- Game Setup ---
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Real World Flight Simulator with Autopilot")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 20)
font_large = pygame.font.SysFont("consolas", 24, bold=True)

# --- Player (Plane) Setup ---
player_screen_pos = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
player_lon, player_lat = -118.2437, 34.0522
player_angle = 0
current_speed_mph = 250.0

# --- Mode and State Variables ---
sr71_mode = False
time_multiplier = 1.0
game_state = STATE_MANUAL_FLIGHT
autopilot_off_flash_timer = 0.0

# --- Autopilot & Text Input Variables ---
destination_text = ""
destination_name = ""
dest_lat, dest_lon = None, None
input_error_msg = ""
input_error_timer = 0.0

## MODIFIED ## --- Tiling System Setup ---
tile_cache = {}
download_queue = queue.Queue()
pending_downloads = set()
placeholder_tile = pygame.Surface((TILE_SIZE, TILE_SIZE))
placeholder_tile.fill(PLACEHOLDER_COLOR)

# Start the downloader thread
downloader_thread = threading.Thread(target=tile_downloader, args=(download_queue, tile_cache), daemon=True)
downloader_thread.start()

# --- Main Game Loop ---
running = True
start_time = pygame.time.get_ticks()

while running:
    dt = clock.tick(60) / 1000.0
    effective_dt = dt * time_multiplier

    # --- Event Handling ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        if game_state == STATE_ENTERING_TEXT:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    if destination_text:
                        try:
                            url = f"https://nominatim.openstreetmap.org/search?q={destination_text}&format=json&limit=1"
                            headers = {'User-Agent': 'MyFlightSim/1.0'}
                            response = requests.get(url, headers=headers, timeout=5).json()
                            if response:
                                dest_lat = float(response[0]['lat'])
                                dest_lon = float(response[0]['lon'])
                                destination_name = response[0]['display_name'].split(',')[0]
                                game_state = STATE_AUTOPILOT_ON
                            else:
                                input_error_msg = "City not found!"
                                input_error_timer = 2.0
                        except Exception as e:
                            input_error_msg = f"Network Error: {e}"
                            input_error_timer = 2.0
                elif event.key == pygame.K_BACKSPACE:
                    destination_text = destination_text[:-1]
                elif event.key == pygame.K_ESCAPE:
                    game_state = STATE_MANUAL_FLIGHT
                    destination_text = ""
                elif pygame.K_F1 <= event.key <= pygame.K_F10:
                    city_index = event.key - pygame.K_F1
                    city_list = list(PRESET_CITIES.keys())
                    if city_index < len(city_list):
                        destination_name = city_list[city_index]
                        dest_lat, dest_lon = PRESET_CITIES[destination_name]
                        game_state = STATE_AUTOPILOT_ON
                else:
                    destination_text += event.unicode
        else:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1: sr71_mode = not sr71_mode
                elif event.key == pygame.K_2: time_multiplier = 2.0
                elif event.key == pygame.K_3: time_multiplier = 5.0
                elif event.key == pygame.K_4: time_multiplier = 10.0
                elif event.key == pygame.K_5 and game_state == STATE_MANUAL_FLIGHT:
                    game_state = STATE_ENTERING_TEXT
                    destination_text = ""
                    input_error_msg = ""
                elif event.key == pygame.K_BACKSPACE:
                    if game_state == STATE_AUTOPILOT_ON:
                        game_state = STATE_MANUAL_FLIGHT
                        autopilot_off_flash_timer = 1.5
                    else:
                        time_multiplier = 1.0

    # --- Game Logic ---
    if game_state != STATE_ENTERING_TEXT:
        current_max_speed = SR71_MAX_SPEED_MPH if sr71_mode else NORMAL_MAX_SPEED_MPH
        plane_color = SR71_COLOR if sr71_mode else RED
        plane_name = "SR-71 Blackbird" if sr71_mode else "Boeing 737"

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]: current_speed_mph += THROTTLE_INCREMENT_MPH * effective_dt
        if keys[pygame.K_DOWN]: current_speed_mph -= THROTTLE_INCREMENT_MPH * effective_dt
        current_speed_mph = max(0, min(current_speed_mph, current_max_speed))

        if game_state == STATE_AUTOPILOT_ON:
            target_angle = get_bearing(player_lat, player_lon, dest_lat, dest_lon)
            angle_diff = (target_angle - player_angle + 180) % 360 - 180
            if abs(angle_diff) > 1:
                turn_direction = 1 if angle_diff > 0 else -1
                player_angle += turn_direction * TURN_SPEED_DEGREES * effective_dt
                player_angle %= 360
        else:
            if keys[pygame.K_a]: player_angle -= TURN_SPEED_DEGREES * effective_dt
            if keys[pygame.K_d]: player_angle += TURN_SPEED_DEGREES * effective_dt
            player_angle %= 360

        # Physics Update
        miles_per_second = current_speed_mph / 3600.0
        miles_moved = miles_per_second * effective_dt
        rad_angle_for_physics = math.radians(player_angle)
        
        d_miles_y = math.cos(rad_angle_for_physics) * miles_moved
        d_miles_x = math.sin(rad_angle_for_physics) * miles_moved
        d_lat = d_miles_y / MILES_PER_DEGREE_LATITUDE
        miles_per_degree_lon = MILES_PER_DEGREE_LATITUDE * math.cos(math.radians(player_lat))
        d_lon = d_miles_x / miles_per_degree_lon if miles_per_degree_lon > 0 else 0
        player_lat += d_lat
        player_lon += d_lon

    # --- Drawing ---
    if game_state != STATE_ENTERING_TEXT:
        player_world_px, player_world_py = lonlat_to_world_pixels(player_lon, player_lat, ZOOM_LEVEL)
        top_left_world_px = player_world_px - SCREEN_WIDTH / 2
        top_left_world_py = player_world_py - SCREEN_HEIGHT / 2
        
        ## MODIFIED ## --- Predictive Tile Calculation ---
        tx_min = int(top_left_world_px // TILE_SIZE) - PREDICTIVE_BUFFER
        ty_min = int(top_left_world_py // TILE_SIZE) - PREDICTIVE_BUFFER
        tx_max = int((top_left_world_px + SCREEN_WIDTH) // TILE_SIZE) + PREDICTIVE_BUFFER
        ty_max = int((top_left_world_py + SCREEN_HEIGHT) // TILE_SIZE) + PREDICTIVE_BUFFER

        screen.fill(BLACK)
        
        # Clean up the pending_downloads set
        # A tile is no longer pending if it has appeared in the cache
        completed_downloads = pending_downloads.intersection(tile_cache.keys())
        pending_downloads.difference_update(completed_downloads)

        for tx in range(tx_min, tx_max + 1):
            for ty in range(ty_min, ty_max + 1):
                tile_key = (ZOOM_LEVEL, tx, ty)
                
                ## MODIFIED ## --- Non-Blocking Tile Logic ---
                if tile_key in tile_cache:
                    # If tile is in cache, draw it
                    surface_to_draw = tile_cache[tile_key]
                else:
                    # If not in cache, draw a placeholder
                    surface_to_draw = placeholder_tile
                    # And if not already requested, add it to the download queue
                    if tile_key not in pending_downloads:
                        pending_downloads.add(tile_key)
                        download_queue.put(tile_key)

                screen_px = tx * TILE_SIZE - top_left_world_px
                screen_py = ty * TILE_SIZE - top_left_world_py
                screen.blit(surface_to_draw, (screen_px, screen_py))

        # Draw Plane
        plane_points = [(player_screen_pos[0], player_screen_pos[1] - 15), (player_screen_pos[0] - 10, player_screen_pos[1] + 10), (player_screen_pos[0] + 10, player_screen_pos[1] + 10)]
        center_x, center_y = player_screen_pos
        rotated_points = []
        rad_angle_for_drawing = math.radians(player_angle)
        for x, y in plane_points:
            x, y = x - center_x, y - center_y
            new_x = x * math.cos(rad_angle_for_drawing) - y * math.sin(rad_angle_for_drawing)
            new_y = x * math.sin(rad_angle_for_drawing) + y * math.cos(rad_angle_for_drawing)
            rotated_points.append((new_x + center_x, new_y + center_y))
        pygame.draw.polygon(screen, plane_color, rotated_points)
    
    if game_state == STATE_ENTERING_TEXT:
        s = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        s.fill((0,0,0,180))
        screen.blit(s, (0,0))
        
        box_w, box_h = 800, 400
        box_x, box_y = (SCREEN_WIDTH - box_w) / 2, (SCREEN_HEIGHT - box_h) / 2
        pygame.draw.rect(screen, INPUT_BOX_COLOR, (box_x, box_y, box_w, box_h), border_radius=10)
        pygame.draw.rect(screen, BLACK, (box_x, box_y, box_w, box_h), 3, border_radius=10)

        title_surf = font_large.render("Set Autopilot Destination", True, BLACK)
        screen.blit(title_surf, (box_x + 20, box_y + 20))

        input_field_rect = pygame.Rect(box_x + 20, box_y + 70, box_w - 40, 40)
        pygame.draw.rect(screen, WHITE, input_field_rect)
        pygame.draw.rect(screen, BLACK, input_field_rect, 2)
        text_surf = font.render(destination_text, True, INPUT_TEXT_COLOR)
        screen.blit(text_surf, (input_field_rect.x + 5, input_field_rect.y + 5))

        instr_y = box_y + 130
        instr_surf = font.render("Type a city and press Enter, or select a preset:", True, BLACK)
        screen.blit(instr_surf, (box_x + 20, instr_y))
        
        for i, city in enumerate(list(PRESET_CITIES.keys())):
            preset_surf = font.render(f"F{i+1}: {city}", True, BLACK)
            col = i // 5
            row = i % 5
            screen.blit(preset_surf, (box_x + 40 + col * 350, instr_y + 40 + row * 30))

        if input_error_timer > 0:
            error_surf = font.render(input_error_msg, True, RED)
            screen.blit(error_surf, (box_x + 20, box_y + box_h - 40))
            input_error_timer -= dt

    else:
        elapsed_seconds = (pygame.time.get_ticks() - start_time) / 1000
        hours, minutes, seconds = int(elapsed_seconds // 3600), int((elapsed_seconds % 3600) // 60), int(elapsed_seconds % 60)
        
        hud_mode = font.render(f"Mode: {plane_name}", True, WHITE)
        hud_speed = font.render(f"Speed: {current_speed_mph:.0f} MPH", True, WHITE)
        hud_lat = font.render(f"Lat: {player_lat:.4f}", True, WHITE)
        hud_lon = font.render(f"Lon: {player_lon:.4f}", True, WHITE)
        hud_time = font.render(f"Time: {hours:02}:{minutes:02}:{seconds:02}", True, WHITE)
        hud_warp = font.render(f"Time Warp: {time_multiplier:.0f}x", True, WHITE)
        
        screen.blit(hud_mode, (10, 10))
        screen.blit(hud_speed, (10, 35))
        screen.blit(hud_lat, (10, 60))
        screen.blit(hud_lon, (10, 85))
        screen.blit(hud_time, (10, 110))
        screen.blit(hud_warp, (10, 135))

        if game_state == STATE_AUTOPILOT_ON:
            ap_surf = font.render(f"AUTOPILOT ON -> {destination_name}", True, (0, 255, 0))
            screen.blit(ap_surf, (10, 160))

        if autopilot_off_flash_timer > 0:
            if int(autopilot_off_flash_timer * 4) % 2 == 1:
                flash_surf = font_large.render("AUTOPILOT OFF", True, RED)
                text_rect = flash_surf.get_rect(center=(SCREEN_WIDTH/2, SCREEN_HEIGHT/2))
                screen.blit(flash_surf, text_rect)
            autopilot_off_flash_timer -= dt

    pygame.display.flip()

## NEW ## --- Graceful Shutdown ---
# Signal the downloader thread to exit and wait for it
download_queue.put(None)
downloader_thread.join()

pygame.quit()
