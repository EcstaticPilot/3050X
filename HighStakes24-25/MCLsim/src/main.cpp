#include <SDL2/SDL.h>
#include <iostream>
#include "monteCarlo.h"
// Constants for the screen size
const int SCREEN_WIDTH = 1000;  // Window width
const int SCREEN_HEIGHT = 1000; // Window height

// Scale factor to convert inches to pixels (adjust as needed)
extern const double SCALE_FACTOR;
const double SCALE_FACTOR = 5.0; // 1 inch = 5 pixels

void drawRobot(SDL_Renderer *renderer, SDL_Texture *texture, int x, int y, int size, double angle)
{
    SDL_Point center = {size / 2, size / 2};
    SDL_Rect robot = {x - size / 2, y - size / 2, size, size};
    
    SDL_RenderCopyEx(renderer, texture, NULL, &robot, angle, &center, SDL_FLIP_NONE);
}

void drawLadder(SDL_Renderer *renderer, int x, int y)
{
    SDL_Point points[] = {
        {x, y + static_cast<int>(SCALE_FACTOR)}, {x - static_cast<int>(SCALE_FACTOR), y}, {x - static_cast<int>(SCALE_FACTOR), y}, {x, y - static_cast<int>(SCALE_FACTOR)}, {x, y - static_cast<int>(SCALE_FACTOR)}, {x + static_cast<int>(SCALE_FACTOR), y}, {x + static_cast<int>(SCALE_FACTOR), y}, {x, y + static_cast<int>(SCALE_FACTOR)}};

    SDL_RenderDrawLines(renderer, points, 8);
}

void drawGrid(SDL_Renderer *renderer, int x, int y, int boxSize, int gridSize)
{
    int gridSizePixels = static_cast<int>(gridSize * SCALE_FACTOR);
    int centerX = x + boxSize / 2;
    int centerY = y + boxSize / 2;

    // Draw vertical lines
    for (int i = centerX; i <= x + boxSize; i += gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, i, y, i, y + boxSize);
    }
    for (int i = centerX; i >= x; i -= gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, i, y, i, y + boxSize);
    }

    // Draw horizontal lines
    for (int i = centerY; i <= y + boxSize; i += gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, x, i, x + boxSize, i);
    }
    for (int i = centerY; i >= y; i -= gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, x, i, x + boxSize, i);
    }
}

int main(int argc, char *argv[])
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("MCL sim",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window)
    {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer)
    {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    // robot
    SDL_Texture *robotTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 24 * SCALE_FACTOR, 24 * SCALE_FACTOR);
    if (!robotTexture)
    {
        std::cerr << "SDL_CreateTexture Error: " << SDL_GetError() << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Set the robot texture to a solid color (e.g., red)
    SDL_SetTextureBlendMode(robotTexture, SDL_BLENDMODE_BLEND);
    
    SDL_SetRenderTarget(renderer, robotTexture);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 55);
    SDL_RenderClear(renderer);
    SDL_Rect rect = {0, 0, 24 * SCALE_FACTOR, 24 * SCALE_FACTOR};
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderDrawRect(renderer, &rect); // Draw the outline of the rectangle
    SDL_SetRenderTarget(renderer, NULL);

    // Box dimensions (scaled to pixels)
    int boxSize = static_cast<int>(140.75 * SCALE_FACTOR);
    int x = (SCREEN_WIDTH - boxSize) / 2; // Center the box
    int y = (SCREEN_HEIGHT - boxSize) / 2;

    // Main loop
    bool running = true;
    SDL_Event event;
    // Clear the screen with a white background
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    // Set the draw color to black
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

    // Draw the hollow box
    SDL_Rect box = {x, y, boxSize, boxSize};
    SDL_RenderDrawRect(renderer, &box);
    drawLadder(renderer, 24 * SCALE_FACTOR, 0);
    // Present the renderer
    SDL_RenderPresent(renderer);
    Robot = pose(0, 0, 0);
    bool moveForward = false, moveBackward = false, turnLeft = false, turnRight = false;

    const int FRAME_RATE = 30;
    const int FRAME_DELAY = 1000 / FRAME_RATE;
    std::cout << "Starting MCL" << std::endl;
    MonteCarlo::init(pose(0, 0, 0), 1000);
    std::cout << "MCL started" << std::endl;
    
    //SDL_Delay(1000);
    while (running)
    {
        Uint32 frameStart = SDL_GetTicks();
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                switch (event.key.keysym.sym)
                {
                case SDLK_w:
                    moveForward = true;
                    break;
                case SDLK_s:
                    moveBackward = true;
                    break;
                case SDLK_a:
                    turnLeft = true;
                    break;
                case SDLK_d:
                    turnRight = true;
                    break;
                }
            }
            else if (event.type == SDL_KEYUP)
            {
                switch (event.key.keysym.sym)
                {
                case SDLK_w:
                    moveForward = false;
                    break;
                case SDLK_s:
                    moveBackward = false;
                    break;
                case SDLK_a:
                    turnLeft = false;
                    break;
                case SDLK_d:
                    turnRight = false;
                    break;
                }
            }
        }

        // Clear the screen with a white background
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);

        if (moveForward)
        {
            Robot.x += (0.35 * SCALE_FACTOR * sin(Robot.theta * M_PI / 180.0));
            Robot.y += (0.35 * SCALE_FACTOR * cos(Robot.theta * M_PI / 180.0));
        }
        if (moveBackward)
        {
            Robot.x -= (0.35 * SCALE_FACTOR * sin(Robot.theta * M_PI / 180.0));
            Robot.y -= (0.35 * SCALE_FACTOR * cos(Robot.theta * M_PI / 180.0));
        }
        if (turnLeft)
        {
            Robot.theta -= 3;
        }
        if (turnRight)
        {
            Robot.theta += 3;
        }

        // Set the draw color to black
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

        // Draw the hollow box
        SDL_Rect box = {x, y, boxSize, boxSize};
        SDL_RenderDrawRect(renderer, &box);
        drawGrid(renderer, x, y, boxSize, 24);

        drawLadder(renderer, (x + boxSize / 2) + 24 * SCALE_FACTOR, y + boxSize / 2);
        drawLadder(renderer, (x + boxSize / 2), y + boxSize / 2 + 24 * SCALE_FACTOR);
        drawLadder(renderer, (x + boxSize / 2) - 24 * SCALE_FACTOR, y + boxSize / 2);
        drawLadder(renderer, (x + boxSize / 2), y + boxSize / 2 - 24 * SCALE_FACTOR);
        // draw robot
        drawRobot(
            renderer,
            robotTexture,
            (x + boxSize / 2) + Robot.x * SCALE_FACTOR,
            (y + boxSize / 2) + -Robot.y * SCALE_FACTOR,
            16 * SCALE_FACTOR,
            Robot.theta);
        MonteCarlo::renderDistanceSensors(renderer);
        MonteCarlo::update();

        MonteCarlo::normalizeSamples();  
        MonteCarlo::renderParticles(renderer);
       
        MonteCarlo::resample();
        
        //MonteCarlo::getPose().print();
        // Present the renderer
        SDL_RenderPresent(renderer);
        Uint32 frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < FRAME_DELAY)
        {
            SDL_Delay(FRAME_DELAY - frameTime);
        }
    }

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
