import numpy as np
from PIL import Image
import random
import yaml

#---------------------------------------------------------------------
# Этот скрипт строит рандомный алгоритм через DFS, генерирует для него 
# .pgm и .yaml файлы для Nav2
# .sdf файл лабиринта для Gazebo
#---------------------------------------------------------------------



MAZE_PGM_FILE_FOLDER = 'config/'
MAZE_PGM_FILENAME    = 'random_maze.pgm'
MAZE_PGM_FILE_PATH   = MAZE_PGM_FILE_FOLDER + MAZE_PGM_FILENAME

MAZE_SDF_FILE_PATH = 'world/random_maze/model.sdf'

# MAZE_YAML_FILE_NAME = MAZE_PGM_FILE_NAME.replace('.pgm', '.yaml')  -  так кодом задается имя yaml файла

CELL_SIZE          = 1      # 1 x 1 метр
CELL_TO_WALL_RATIO = 3      # коридор в 3 раза шире стены

PGM_MAP_SCALE  = 10         # 1 метр = 10 пикселей

WALL_HEIGHT    = 2.5        # 2.5 метра в симуляции Gazebo

def generate_maze(maze_width, maze_height):
    
    # заполняем лабиринт целиком стенами

    blocks_count_x = 2 * maze_width  + 1
    blocks_count_y = 2 * maze_height + 1

    maze = np.ones((blocks_count_y, blocks_count_x), dtype = np.uint8)  # 1 — стена
    
    # Стартовая клетка
    def carve_pass(x, y):
        maze[y][x] = 0
        
        dirs = [(2, 0), (-2, 0), (0, 2), (0, -2)]   # "блок" стены имеет такой же размер, как комнаты, так что проходим через 2 блока
        random.shuffle(dirs)

        for dx, dy in dirs:
            new_x = x + dx
            new_y = y + dy
        
            if 1 <= new_x < blocks_count_x - 1 and 1 <= new_y < blocks_count_y - 1:
                if maze[new_y][new_x] == 1:
                    maze[y + dy // 2][x + dx // 2] = 0

                    carve_pass(new_x, new_y)

    # начать вырезать проход с угла
    carve_pass(blocks_count_y - 2, 1)

    maze[blocks_count_y - 1, 0] = 0     # вход на старте

    # пробиваем несколько выходов (по краям), чтобы выходов из лабиринта было несколько
    for _ in range(2):
        side = random.choice(['top', 'bottom', 'left', 'right'])
        if side == 'top':
            x = random.randrange(1, blocks_count_x - 1, 2)
            maze[0][x] = 0
        elif side == 'bottom':
            x = random.randrange(1, blocks_count_x - 1, 2)
            maze[blocks_count_y - 1][x] = 0
        elif side == 'left':
            y = random.randrange(1, blocks_count_y - 1, 2)
            maze[y][0] = 0
        elif side == 'right':
            y = random.randrange(1, blocks_count_y - 1, 2)
            maze[y][blocks_count_x - 1] = 0

    return maze

#---------------------------------------------------------------------

# делает стенки заданной толщины

def upscale_maze(maze, scale):
    height, width = maze.shape
    new_height = height * scale
    new_width  = width  * scale

    maze_upscaled = np.ones((new_height, new_width), dtype=np.uint8)  # всё стены

    for i in range(height):
        for j in range(width):
            if maze[i][j] == 0:
                start_y = i * scale
                start_x = j * scale

                # print(f"start_x = {start_x}, start_y = {start_y}\n")

                maze_upscaled[start_y:start_y + scale, start_x:start_x + scale] = 0

    return maze_upscaled

#---------------------------------------------------------------------

# основываюсь на карту клеточного лабиринта, меняем увеличенный чтобы построить лабиринт с тонкими стенками

def thin_walls(maze, scaled_maze, scale):
    height, width = maze.shape

    # утоньчение по вертикали

    for row_num in range(height - 2):
        can_be_thin = True

        for col_num in range(width - 2):
            if maze[row_num][col_num] == 0 and maze[row_num][col_num + 1] == 0:     # это не просто арка, а коридор
                can_be_thin = False
                break

        if can_be_thin:
            scaled_maze_row_num = row_num * scale
            
            for col_num in range(width - 1):
                scaled_maze_col_num = col_num * scale

                if maze[row_num][col_num] == 1 and maze[row_num + 1][col_num] == 1:     # если это стена и под ней стена, то по вертикали не сужаем
                    continue

                scaled_maze[scaled_maze_row_num + 1 : scaled_maze_row_num + scale, scaled_maze_col_num : scaled_maze_col_num + scale] = 0

    last_scaled_row_num = (height - 1) * scale
    last_scaled_col_num = (width - 1)  * scale

    scaled_maze[last_scaled_row_num + 1 : last_scaled_row_num + scale, 0 : last_scaled_col_num + scale] = 0


    # утоньчение по вертикали

    for col_num in range(width - 2):
        can_be_thin = True

        for row_num in range(height - 2):
            if maze[row_num][col_num] == 0 and maze[row_num + 1][col_num] == 0:     # это не просто арка, а коридор
                can_be_thin = False
                break

        if can_be_thin:
            scaled_maze_col_num = col_num * scale
            
            for row_num in range(height - 1):
                scaled_maze_row_num = row_num * scale

                if maze[row_num][col_num] == 1 and maze[row_num][col_num + 1] == 1:     # если это стена и под ней стена, то по вертикали не сужаем
                    continue

                scaled_maze[scaled_maze_row_num : scaled_maze_row_num + scale, scaled_maze_col_num + 1 : scaled_maze_col_num + scale] = 0

    last_scaled_row_num = (height - 1) * scale
    last_scaled_col_num = (width - 1)  * scale

    scaled_maze[0 : last_scaled_row_num + scale, last_scaled_col_num + 1 : last_scaled_col_num + scale] = 0
#---------------------------------------------------------------------

def save_maze_as_pgm(maze, cell_pixel_size, pgm_file_path=MAZE_PGM_FILE_PATH):      # один блок - это cell_pixel_size*cell_pixel_size пикселей
    pixels_arr = 255 * (1 - maze)                                                   # стены = 0 (черный), проходы = 255 (белый)
    pixels_arr = np.kron(pixels_arr, np.ones((cell_pixel_size, cell_pixel_size)))   # масштаб

    img_pgm = Image.fromarray(pixels_arr.astype(np.uint8))
    img_pgm.save(pgm_file_path)

#---------------------------------------------------------------------

def generate_yaml(pgm_file_folder, pgm_filename, resolution, origin=[0.0, 0.0, 0.0]):
    yaml_data = {
        'image': pgm_filename,
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    with open((pgm_file_folder + pgm_filename).replace('.pgm', '.yaml'), 'w') as yaml_file:
        yaml.dump(yaml_data, yaml_file, default_flow_style=None)


#---------------------------------------------------------------------

def generate_sdf(maze, sdf_filename,
                           cell_size,
                           wall_height=WALL_HEIGHT):
    maze_hight, maze_width = maze.shape
    wall_id = 0
    cells_links = []

    
    for j in range(maze_hight):
        for i in range(maze_width):
            if maze[j][i] == 1:
                cell_coord_x = i * cell_size
                cell_coord_y = (maze_hight - j - 1) * cell_size
                cell_coord_z = 0
                name = f"Wall_{wall_id}"

                block = f"""
                <link name='{name}'>
                <collision name='{name}_Collision'>
                    <geometry>
                    <box>
                        <size>{cell_size} {cell_size} {wall_height}</size>
                    </box>
                    </geometry>
                    <pose>0 0 {wall_height / 2} 0 0 0</pose>
                </collision>
                <visual name='{name}_Visual'>
                    <pose>0 0 {wall_height / 2} 0 0 0</pose>
                    <geometry>
                    <box>
                        <size>{cell_size} {cell_size} {wall_height}</size>
                    </box>
                    </geometry>
                    <material>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Wood</name>
                    </script>
                    <ambient>1 1 1 1</ambient>
                    </material>
                    <meta>
                    <layer>0</layer>
                    </meta>
                </visual>
                <pose>{cell_coord_x + cell_size / 2} {cell_coord_y + cell_size / 2} {cell_coord_z} 0 0 0</pose>
                </link>
                """

                cells_links.append(block)
                wall_id += 1

    sdf_text = f"""
    <?xml version='1.0'?>
    <sdf version='1.7'>
    <model name='maze'>
        <static>true</static>
        {''.join(cells_links)}
    </model>
    </sdf>
    """
    with open(sdf_filename, 'w') as sdf_file:
        sdf_file.write(sdf_text)

#---------------------------------------------------------------------



maze = generate_maze(5, 5)
for row in maze:
    print("".join("##" if cell == 1 else "  " for cell in row))

print("\n\n")

maze_increase_scale = CELL_TO_WALL_RATIO

maze_upscaled = upscale_maze(maze, maze_increase_scale)

for row in maze_upscaled:
    print("".join("##" if cell == 1 else "  " for cell in row))
print("\n\n")


thin_walls(maze, maze_upscaled, CELL_TO_WALL_RATIO)
for row in maze_upscaled:
    print("".join("##" if cell == 1 else "  " for cell in row))

generate_sdf(maze_upscaled, MAZE_SDF_FILE_PATH, CELL_SIZE / CELL_TO_WALL_RATIO)


cell_pixel_size = int(CELL_SIZE / maze_increase_scale * PGM_MAP_SCALE)

print(f"cell_p_s = {cell_pixel_size}\n")

save_maze_as_pgm(maze_upscaled, cell_pixel_size)
generate_yaml(MAZE_PGM_FILE_FOLDER, MAZE_PGM_FILENAME, CELL_SIZE / maze_increase_scale / cell_pixel_size)