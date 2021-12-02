
import pygame
import random
from pygame.locals import *  # eg    K_UP, K_DOWN,K_LEFT,K_RIGHT,K_ESCAPE,KEYDOWN,QUIT
#from P6_classes import *

from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_ESCAPE,
    KEYDOWN,
    QUIT,
)

# Define constants for the screen width and height
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600
GAMESPEED = 13
PLAYERSPEED = 2*GAMESPEED/3
LEVEL = 1

# having trouble with classes in a separate file

class Player(pygame.sprite.Sprite):
    def __init__(self):
        super(Player, self).__init__()
        Player_width=24
        Player_height=70
        # self.surf = pygame.Surface((Player_width, Player_height))
        # self.surf.fill((255, 255, 255))
        # self.rect = self.surf.get_rect()
        self.surf = pygame.image.load("car_icon_3.png").convert()
        self.surf.set_colorkey((0, 0, 0), RLEACCEL)
        self.rect = self.surf.get_rect()
        self.rect.bottom=DISPLAY_HEIGHT
        self.rect.left=DISPLAY_WIDTH/2-Player_width/2

    # def __init__(self):
    #     super(Player, self).__init__()
    #     self.surf = pygame.image.load("icon.png").convert()
    #     self.surf.set_colorkey((255, 255, 255), RLEACCEL)
    #     self.rect = self.surf.get_rect()

    # Move the sprite based on keypresses
    def update(self, pressed_keys):
        if pressed_keys[K_UP]:
            self.rect.move_ip(0, -PLAYERSPEED)
        if pressed_keys[K_DOWN]:
            self.rect.move_ip(0, PLAYERSPEED)
        if pressed_keys[K_LEFT]:
            self.rect.move_ip(-PLAYERSPEED, 0)
        if pressed_keys[K_RIGHT]:
            self.rect.move_ip(PLAYERSPEED, 0)

        # Keep player on the screen
        if self.rect.left < 0:
            self.rect.left = 0
        elif self.rect.right > DISPLAY_WIDTH:
            self.rect.right = DISPLAY_WIDTH
        if self.rect.top <= 0:
            self.rect.top = 0
        elif self.rect.bottom >= DISPLAY_HEIGHT:
            self.rect.bottom = DISPLAY_HEIGHT

class Obstacle(pygame.sprite.Sprite):
    def __init__(self):
        super(Obstacle, self).__init__()
        self.surf = pygame.image.load("spider.png").convert()
        self.surf.set_colorkey((0,0,0), RLEACCEL)
        # self.surf = pygame.Surface((20, 10))
        # self.surf.fill((255, 255, 255))
        self.rect = self.surf.get_rect(
            center=(
                random.randint(0, DISPLAY_WIDTH),
                random.randint(-50, -1),  # Have obstacles come on screen at random times
            )
        )
        self.speed = GAMESPEED

    # Move the sprite based on speed
    # Remove it when it passes the left edge of the screen
    def update(self):
        self.rect.move_ip(0, self.speed)
        if self.rect.top > DISPLAY_HEIGHT:
            self.kill()

class Obstacle2(pygame.sprite.Sprite):
    def __init__(self, side):
        super(Obstacle2, self).__init__()
        self.surf = pygame.image.load("spider_mirror.png").convert_alpha()
        self.surf.set_colorkey((0,0,0), RLEACCEL)
        self.side = side
        self.move=random.randint(-5,5)
        self.rect = self.surf.get_rect(
            center=(
                random.randint(0, DISPLAY_WIDTH),
                random.randint(-50, -1),  # Have obstacles come on screen at random times
            )
        )
        self.speed = GAMESPEED

    # Move the sprite based on speed
    # Remove it when it passes the left edge of the screen
    def update(self):
        if self.side < 15:
            self.surf=pygame.image.load("spider.png").convert_alpha()
            self.side = self.side + 1
        elif self.side < 30:
            self.surf=pygame.image.load("spider_mirror.png").convert_alpha()
            self.side=self.side+1
        else:
            self.side=0
        self.rect.move_ip(self.move, self.speed/1.5+self.move/2)
        if self.rect.top > DISPLAY_HEIGHT:
            self.kill()


class Tree1(pygame.sprite.Sprite):
    def __init__(self, side, width):
        super(Tree1, self).__init__()
        self.surf = pygame.image.load("Tree1sm.png").convert()  #  ***** add image *****
        self.surf.set_colorkey((0, 0, 0), RLEACCEL)
        # The starting position is randomly generated
        if side > 0:
            self.rect = self.surf.get_rect(
                center=(
                    random.randint(0, width),
                    random.randint(-width, -20 ),
                    )
                )
        else:
            self.rect = self.surf.get_rect(
                center=(
                    random.randint(DISPLAY_WIDTH-width, DISPLAY_WIDTH),
                    random.randint(-60, -20 ),
                    )
                )

        self.speed = GAMESPEED

    # Move the sprite based on speed
    # Remove it when it passes the left edge of the screen
    def update(self):
        self.rect.move_ip(0, self.speed)
        if self.rect.top > DISPLAY_HEIGHT:
            self.kill()

class Tree2(pygame.sprite.Sprite):
    def __init__(self):
        super(Tree2, self).__init__()
        self.surf = pygame.image.load("Tree1sm.png").convert()  # ***** add image *****
        self.surf.set_colorkey((0, 0, 0), RLEACCEL)
        # The starting position is randomly generated

        self.rect = self.surf.get_rect(
            center=(
                random.randint(0, DISPLAY_WIDTH),
                random.randint(-60, -1),
            )
        )

        self.speed = GAMESPEED

    # Move the sprite based on speed
    # Remove it when it passes the left edge of the screen
    def update(self):
        self.rect.move_ip(0, self.speed)
        if self.rect.top > DISPLAY_HEIGHT:
            self.kill()


def main():

    global LEVEL

    # Initialize pygame
    pygame.init()

    # Setup the clock for a decent framerate
    clock = pygame.time.Clock()

    # Set screen sizeset
    screen = pygame.display.set_mode((DISPLAY_WIDTH, DISPLAY_HEIGHT))

    # Create events.
    AddObstacle = pygame.USEREVENT + 1        # ensure individual index for event
    AddTree1 = pygame.USEREVENT + 2
    AddObstacle2 = pygame.USEREVENT + 3
    AddTree2 = pygame.USEREVENT + 4

    if LEVEL < 500:
        pygame.time.set_timer(AddObstacle, 800)  # interval for adding obstacles in ms
        # pygame.time.set_timer(AddObstacle2, 2000)
        pygame.time.set_timer(AddTree1, 20)
        pygame.time.set_timer(AddTree2, 4000)
        width=60


    # Create our 'player'
    player = Player()

    # Create groups to hold enemy sprites, and every sprite
    # - enemies is used for collision detection and position updates
    # - all_sprites is used for rendering
    obstacles = pygame.sprite.Group()
    scenery = pygame.sprite.Group()
    moving_scenery = pygame.sprite.Group()     # for screen scrolling
    all_sprites = pygame.sprite.Group()
    all_sprites.add(player)

    # Variable to keep our main loop running
    running = True
    side=1
    # Our main loop
    while running:
        clock.tick(30)


        # cycle events and take actions:
        for event in pygame.event.get():
            # Key pressed?
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:  # ending game if escape pressed
                    running = False

            # If window close button clicked:
            elif event.type == QUIT:
                running = False

            # If time to add obstacle:
            elif event.type == AddObstacle:
                # Create the new enemy, and add it to our sprite groups
                new_obstacle = Obstacle()
                obstacles.add(new_obstacle)
                all_sprites.add(new_obstacle)

            elif event.type == AddObstacle2:
                # Create the new enemy, and add it to our sprite groups
                new_obstacle = Obstacle2(1)
                obstacles.add(new_obstacle)
                all_sprites.add(new_obstacle)

                # If time to add tree:
            elif event.type == AddTree1:
                # Create the new enemy, and add it to our sprite groups
                new_tree = Tree1(side, width)
                moving_scenery.add(new_tree)
                all_sprites.add(new_tree)
                side=side*(-1)

            elif event.type == AddTree2:
                # Create the new enemy, and add it to our sprite groups
                new_tree = Tree2()
                moving_scenery.add(new_tree)
                all_sprites.add(new_tree)

        # Get the set of keys pressed and check for user input
        pressed_keys = pygame.key.get_pressed()
        player.update(pressed_keys)

        # Update the position of our enemies
        obstacles.update()
        moving_scenery.update()

        # Fill the screen with black
        screen.fill((131, 105, 83))

        # Draw all our sprites
        for entity in all_sprites:
            screen.blit(entity.surf, entity.rect)

        # Check if player has collided with any obstacles
        if pygame.sprite.spritecollideany(player, obstacles):
            # If so, remove the player and stop the loop
            print("You are spider lunch!!")
            player.kill()
            running = False

        # Leveling
        LEVEL=LEVEL+1

        if LEVEL == 500:
            pygame.time.set_timer(AddTree1, 20)
            pygame.time.set_timer(AddObstacle, 800)  # interval for adding obstacles in ms
            pygame.time.set_timer(AddObstacle2, 2000)
            pygame.time.set_timer(AddTree2, 3000)
            width=100

        if LEVEL == 1000:
            pygame.time.set_timer(AddTree1, 15)
            pygame.time.set_timer(AddObstacle, 400000)  # interval for adding obstacles in ms
            pygame.time.set_timer(AddObstacle2, 1000)
            pygame.time.set_timer(AddTree2, 2000)
            width=120

        if LEVEL == 1500:
            pygame.time.set_timer(AddTree1, 10)
            pygame.time.set_timer(AddObstacle, 400000)  # interval for adding obstacles in ms
            pygame.time.set_timer(AddObstacle2, 700)
            pygame.time.set_timer(AddTree2, 1000)
            width = 140

        if LEVEL == 2000:
            pygame.time.set_timer(AddTree1, 3)
            pygame.time.set_timer(AddObstacle, 2000)  # interval for adding obstacles in ms
            pygame.time.set_timer(AddObstacle2, 500)
            pygame.time.set_timer(AddTree2, 500)
            width = 160

        # Flip everything to the display
        pygame.display.flip()


if  __name__ == "__main__":
    main()

