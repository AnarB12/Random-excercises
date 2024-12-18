import pygame
import sys
import random
import os

# Initialize pygame (pip install pygame)
pygame.init()
pygame.mixer.init()

# ---------------- Configuration Constants ----------------
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
GRID_SIZE = 20
GRID_WIDTH = SCREEN_WIDTH // GRID_SIZE
GRID_HEIGHT = SCREEN_HEIGHT // GRID_SIZE

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (34, 139, 34)
RED = (200, 0, 0)
YELLOW = (255, 215, 0)
GRAY = (100, 100, 100)
BLUE = (0, 0, 200)

# Fonts
FONT_PATH = pygame.font.get_default_font()
FONT_SMALL = pygame.font.SysFont(FONT_PATH, 20)
FONT_MEDIUM = pygame.font.SysFont(FONT_PATH, 30)
FONT_LARGE = pygame.font.SysFont(FONT_PATH, 50)

# Game states
STATE_MAIN_MENU = "main_menu"
STATE_GAME = "game"
STATE_PAUSE = "pause"
STATE_GAME_OVER = "game_over"
STATE_SETTINGS = "settings"
STATE_HIGHSCORES = "highscores"

# Difficulty settings (speed and obstacle count)
DIFFICULTIES = {
    "Easy": {"speed": 7,  "obstacles": 0},
    "Medium": {"speed": 10, "obstacles": 5},
    "Hard": {"speed": 15, "obstacles": 10},
}

# Global Assets (sounds, music)
EAT_SOUND = None
GAME_OVER_SOUND = None
CLICK_SOUND = None

# High score file
HIGHSCORE_FILE = "highscore.txt"


class Snake:
    def __init__(self):
        self.reset()

    def reset(self):
        self.positions = [(GRID_WIDTH // 2, GRID_HEIGHT // 2)]
        self.direction = random.choice([(1,0), (-1,0), (0,1), (0,-1)])
        self.length = 1

    def head_position(self):
        return self.positions[0]

    def move(self):
        # current head
        head_x, head_y = self.head_position()
        dx, dy = self.direction
        new_head = (head_x + dx, head_y + dy)
        self.positions.insert(0, new_head)
        while len(self.positions) > self.length:
            self.positions.pop()

    def turn(self, direction):
        # Avoid reversing directly
        if (direction[0] * -1, direction[1] * -1) == self.direction:
            return
        self.direction = direction

    def collide_with_self(self):
        return len(self.positions) != len(set(self.positions))

    def draw(self, surface):
        for i, pos in enumerate(self.positions):
            x = pos[0]*GRID_SIZE
            y = pos[1]*GRID_SIZE
            color = GREEN if i == 0 else (0, 180, 0)
            pygame.draw.rect(surface, color, (x, y, GRID_SIZE, GRID_SIZE))


class Food:
    def __init__(self, snake_positions, obstacles):
        self.position = self.random_position(snake_positions, obstacles)
        self.special = False
        self.timer = 0  # For special food longevity

    def random_position(self, snake_positions, obstacles):
        available = [(x, y) for x in range(GRID_WIDTH) for y in range(GRID_HEIGHT)
                     if (x, y) not in snake_positions and (x,y) not in obstacles]
        return random.choice(available)

    def spawn_special(self, snake_positions, obstacles):
        self.special = True
        self.timer = 300  # frames till vanish
        self.position = self.random_position(snake_positions, obstacles)

    def draw(self, surface):
        x = self.position[0]*GRID_SIZE
        y = self.position[1]*GRID_SIZE
        color = YELLOW if self.special else RED
        pygame.draw.rect(surface, color, (x, y, GRID_SIZE, GRID_SIZE))

    def update(self):
        if self.special:
            self.timer -= 1
            if self.timer <= 0:
                self.special = False


class Game:
    def __init__(self):
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Snake Game XOXO.")
        self.clock = pygame.time.Clock()

        # Load sounds (if available)
        self.load_sounds()

        # Initial Game variables
        self.state = STATE_MAIN_MENU
        self.snake = Snake()
        self.current_difficulty = "Easy"
        self.speed = DIFFICULTIES[self.current_difficulty]["speed"]
        self.obstacle_count = DIFFICULTIES[self.current_difficulty]["obstacles"]
        self.obstacles = []
        self.food = None
        self.score = 0
        self.highscore = self.load_highscore()
        self.theme = "Classic"  # Could be used to switch color schemes

        self.background_music = None
        self.play_background_music()

    def load_sounds(self):
        if pygame.mixer.get_init():
            try:
                EAT_SOUND = pygame.mixer.Sound("eat.wav")
            except:
                EAT_SOUND = None
            try:
                GAME_OVER_SOUND = pygame.mixer.Sound("gameover.wav")
            except:
                GAME_OVER_SOUND = None
            try:
                CLICK_SOUND = pygame.mixer.Sound("click.wav")
            except:
                CLICK_SOUND = None
        else:
            EAT_SOUND = None
            GAME_OVER_SOUND = None
            CLICK_SOUND = None
        

        globals()['EAT_SOUND'] = EAT_SOUND
        globals()['GAME_OVER_SOUND'] = GAME_OVER_SOUND
        globals()['CLICK_SOUND'] = CLICK_SOUND

        # Another way if actual sound files:
        # EAT_SOUND = pygame.mixer.Sound("eat.wav")
        # GAME_OVER_SOUND = pygame.mixer.Sound("gameover.wav")
        # CLICK_SOUND = pygame.mixer.Sound("click.wav")

    def play_sound(self, sound):
        if sound is not None:
            sound.play()

    def play_background_music(self):
        # Load a music file if have one
        # pygame.mixer.music.load("background.mp3")
        # pygame.mixer.music.play(-1)  # loop forever
        pass

    def load_highscore(self):
        if not os.path.exists(HIGHSCORE_FILE):
            with open(HIGHSCORE_FILE, 'w') as f:
                f.write("0")
            return 0
        with open(HIGHSCORE_FILE, 'r') as f:
            try:
                return int(f.read().strip())
            except:
                return 0

    def save_highscore(self):
        with open(HIGHSCORE_FILE, 'w') as f:
            f.write(str(self.highscore))

    def reset_game(self):
        self.snake.reset()
        self.score = 0
        self.obstacles = self.generate_obstacles(self.obstacle_count)
        self.food = Food(self.snake.positions, self.obstacles)
        self.state = STATE_GAME
        self.speed = DIFFICULTIES[self.current_difficulty]["speed"]

    def generate_obstacles(self, count):
        obstacles = set()
        while len(obstacles) < count:
            x = random.randint(0, GRID_WIDTH-1)
            y = random.randint(0, GRID_HEIGHT-1)
            # Make sure not to spawn on snake start pos or center area
            if (x, y) not in self.snake.positions and (x,y) != (GRID_WIDTH//2, GRID_HEIGHT//2):
                obstacles.add((x,y))
        return obstacles

    def run(self):
        while True:
            if self.state == STATE_MAIN_MENU:
                self.handle_main_menu()
            elif self.state == STATE_SETTINGS:
                self.handle_settings_menu()
            elif self.state == STATE_HIGHSCORES:
                self.handle_highscores_screen()
            elif self.state == STATE_GAME:
                self.handle_gameplay()
            elif self.state == STATE_PAUSE:
                self.handle_pause_menu()
            elif self.state == STATE_GAME_OVER:
                self.handle_game_over_screen()
            else:
                break

    def handle_main_menu(self):
        self.screen.fill(BLACK)
        title_surf = FONT_LARGE.render("SNAKE GAME", True, WHITE)
        start_surf = FONT_MEDIUM.render("1. Play Game", True, WHITE)
        settings_surf = FONT_MEDIUM.render("2. Settings", True, WHITE)
        highscores_surf = FONT_MEDIUM.render("3. High Scores", True, WHITE)
        quit_surf = FONT_MEDIUM.render("4. Quit", True, WHITE)

        self.screen.blit(title_surf, (SCREEN_WIDTH//2 - title_surf.get_width()//2, 100))
        self.screen.blit(start_surf, (SCREEN_WIDTH//2 - start_surf.get_width()//2, 200))
        self.screen.blit(settings_surf, (SCREEN_WIDTH//2 - settings_surf.get_width()//2, 240))
        self.screen.blit(highscores_surf, (SCREEN_WIDTH//2 - highscores_surf.get_width()//2, 280))
        self.screen.blit(quit_surf, (SCREEN_WIDTH//2 - quit_surf.get_width()//2, 320))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    self.reset_game()
                elif event.key == pygame.K_2:
                    self.state = STATE_SETTINGS
                elif event.key == pygame.K_3:
                    self.state = STATE_HIGHSCORES
                elif event.key == pygame.K_4:
                    pygame.quit()
                    sys.exit()

    def handle_settings_menu(self):
        self.screen.fill(BLACK)
        title_surf = FONT_LARGE.render("SETTINGS", True, WHITE)
        self.screen.blit(title_surf, (SCREEN_WIDTH//2 - title_surf.get_width()//2, 80))

        # Difficulty selection
        difficulty_text = FONT_MEDIUM.render("Difficulty: " + self.current_difficulty, True, WHITE)
        self.screen.blit(difficulty_text, (SCREEN_WIDTH//2 - difficulty_text.get_width()//2, 200))

        instructions = FONT_SMALL.render("Press Left/Right to change difficulty. Press ESC to return.", True, WHITE)
        self.screen.blit(instructions, (SCREEN_WIDTH//2 - instructions.get_width()//2, 280))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    # Cycle difficulty backwards
                    keys = list(DIFFICULTIES.keys())
                    idx = keys.index(self.current_difficulty)
                    self.current_difficulty = keys[idx-1]
                elif event.key == pygame.K_RIGHT:
                    # Cycle difficulty forwards
                    keys = list(DIFFICULTIES.keys())
                    idx = keys.index(self.current_difficulty)
                    self.current_difficulty = keys[(idx+1)%len(keys)]
                elif event.key == pygame.K_ESCAPE:
                    self.state = STATE_MAIN_MENU

    def handle_highscores_screen(self):
        self.screen.fill(BLACK)
        title_surf = FONT_LARGE.render("HIGH SCORES", True, WHITE)
        score_surf = FONT_MEDIUM.render(f"Current High Score: {self.highscore}", True, WHITE)
        instructions = FONT_SMALL.render("Press ESC to return.", True, WHITE)

        self.screen.blit(title_surf, (SCREEN_WIDTH//2 - title_surf.get_width()//2, 100))
        self.screen.blit(score_surf, (SCREEN_WIDTH//2 - score_surf.get_width()//2, 200))
        self.screen.blit(instructions, (SCREEN_WIDTH//2 - instructions.get_width()//2, 280))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.state = STATE_MAIN_MENU

    def handle_gameplay(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key in [pygame.K_UP, pygame.K_w]:
                    self.snake.turn((0, -1))
                elif event.key in [pygame.K_DOWN, pygame.K_s]:
                    self.snake.turn((0, 1))
                elif event.key in [pygame.K_LEFT, pygame.K_a]:
                    self.snake.turn((-1, 0))
                elif event.key in [pygame.K_RIGHT, pygame.K_d]:
                    self.snake.turn((1, 0))
                elif event.key == pygame.K_p or event.key == pygame.K_ESCAPE:
                    self.state = STATE_PAUSE

        self.snake.move()

        # Check Collisions
        head_x, head_y = self.snake.head_position()

        # Wrap around or end game if going off-screen?
        # Let's do game over if off-screen:
        if head_x < 0 or head_x >= GRID_WIDTH or head_y < 0 or head_y >= GRID_HEIGHT:
            self.game_over()
            return

        # Self collision
        if self.snake.collide_with_self():
            self.game_over()
            return

        # Obstacle collision
        if (head_x, head_y) in self.obstacles:
            self.game_over()
            return

        # Food collision
        if (head_x, head_y) == self.food.position:
            self.play_sound(EAT_SOUND)
            if self.food.special:
                self.score += 5
            else:
                self.score += 1
            self.snake.length += 1
            self.food = Food(self.snake.positions, self.obstacles)

            # Chance to spawn special food occasionally
            if random.random() < 0.1:  # 10% chance
                self.food.spawn_special(self.snake.positions, self.obstacles)

        # Update special food timer
        self.food.update()

        # Draw everything
        self.screen.fill(BLACK)
        self.draw_obstacles()
        self.snake.draw(self.screen)
        self.food.draw(self.screen)
        self.draw_score()

        pygame.display.flip()
        self.clock.tick(self.speed)

    def handle_pause_menu(self):
        self.screen.fill(BLACK)
        pause_surf = FONT_LARGE.render("PAUSED", True, WHITE)
        resume_surf = FONT_MEDIUM.render("Press P/ESC to Resume", True, WHITE)
        quit_surf = FONT_MEDIUM.render("Press Q to Quit to Main Menu", True, WHITE)

        self.screen.blit(pause_surf, (SCREEN_WIDTH//2 - pause_surf.get_width()//2, 150))
        self.screen.blit(resume_surf, (SCREEN_WIDTH//2 - resume_surf.get_width()//2, 220))
        self.screen.blit(quit_surf, (SCREEN_WIDTH//2 - quit_surf.get_width()//2, 260))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p or event.key == pygame.K_ESCAPE:
                    self.state = STATE_GAME
                elif event.key == pygame.K_q:
                    self.state = STATE_MAIN_MENU

    def handle_game_over_screen(self):
        self.screen.fill(BLACK)
        game_over_surf = FONT_LARGE.render("GAME OVER", True, RED)
        score_surf = FONT_MEDIUM.render(f"Score: {self.score}", True, WHITE)
        highscore_surf = FONT_MEDIUM.render(f"High Score: {self.highscore}", True, WHITE)
        instructions = FONT_SMALL.render("Press R to Restart, M for Main Menu", True, WHITE)

        self.screen.blit(game_over_surf, (SCREEN_WIDTH//2 - game_over_surf.get_width()//2, 120))
        self.screen.blit(score_surf, (SCREEN_WIDTH//2 - score_surf.get_width()//2, 200))
        self.screen.blit(highscore_surf, (SCREEN_WIDTH//2 - highscore_surf.get_width()//2, 240))
        self.screen.blit(instructions, (SCREEN_WIDTH//2 - instructions.get_width()//2, 300))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    self.reset_game()
                elif event.key == pygame.K_m:
                    self.state = STATE_MAIN_MENU

    def game_over(self):
        self.play_sound(GAME_OVER_SOUND)
        if self.score > self.highscore:
            self.highscore = self.score
            self.save_highscore()
        self.state = STATE_GAME_OVER

    def draw_score(self):
        score_text = f"Score: {self.score}  High: {self.highscore}"
        surf = FONT_SMALL.render(score_text, True, WHITE)
        self.screen.blit(surf, (5, 5))

    def draw_obstacles(self):
        for obs in self.obstacles:
            x = obs[0]*GRID_SIZE
            y = obs[1]*GRID_SIZE
            pygame.draw.rect(self.screen, GRAY, (x, y, GRID_SIZE, GRID_SIZE))


if __name__ == "__main__":
    game = Game()
    game.run()


