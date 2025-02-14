#define RAYGUI_IMPLEMENTATION

#include <iostream>

#include "raylib.h"

#include "raymath.h"
#include "simulation/planet.h"
#include "simulation/simulation_governor.h"
#include "data-structures/oct_tree.h"
#include "raygui.h"
#include <bits/stdc++.h>


const Color color_map[18] = {
    LIGHTGRAY,GRAY,DARKGRAY,BROWN,BLUE,PURPLE,ORANGE,RED,PINK,GRAY,DARKGRAY,MAROON,GOLD,LIME,DARKGREEN,DARKBLUE,
    DARKPURPLE
};
constexpr float dif = 0.00f;
constexpr int BODY_SIZE = 1.5f;
constexpr float FORCE_MULTIPLIER = 2.5f;

/**
 *
 * @param octree_t root octree
 * @param depth depth to which the octree debug should be drawn
 * @param whl width height length
 * @param draw_level level to which the octree debug should be drawn
 * @param draw_center_of_mass whether to draw center of mass
 */
void draw_structure_iterative(const dstruct::OctTree &octree_t, int depth, float whl, int draw_level,
                              bool draw_center_of_mass) {
    std::stack<std::tuple<const dstruct::OctTree *, int, float> > stack;
    stack.push(std::make_tuple(&octree_t, depth, whl));

    while (!stack.empty()) {
        auto [node, current_depth, current_whl] = stack.top();
        stack.pop();

        if (current_depth >= draw_level) {
            DrawCubeWires(node->position, current_whl - dif, current_whl - dif, current_whl - dif,
                          color_map[current_depth]);
            if (draw_center_of_mass) {
                DrawSphereWires(node->center_of_mass, 1.0f, 5, 5,GREEN);
            }
            //DrawCubeWires(node->bodies_[0]->position, BODY_SIZE, BODY_SIZE, BODY_SIZE, BLACK);
            //DrawSphere(node->center_of_mass, 0.5f, RED);
        }

        if (!node->children.empty()) {
            for (const auto &child: node->children) {
                stack.push(std::make_tuple(child, current_depth + 1, current_whl / 2));
            }
        }
    }
}

/**
 * Draws the octree structure
 * @param octree_t root octree
 * @param depth depth to which the octree debug should be drawn
 * @param whl width height length
 * @param draw_level level to which the octree debug should be drawn
 */
void draw_structure(const dstruct::OctTree &octree_t, int depth, float whl, int draw_level) {
    //auto start = std::chrono::high_resolution_clock::now();

    if (depth >= draw_level) {
        DrawCubeWires(octree_t.position, whl - dif, whl - dif, whl - dif, color_map[depth]);
    }
    //DrawCubeWires(octree_t.position, whl-dif, whl-dif, whl-dif,color_map[depth]);
    if (octree_t.children.empty()) {
        return;
    }
    for (const auto &child: octree_t.children) {
        //if()
        draw_structure(*child, depth + 1, whl / 2, draw_level);
    }
}

int main() {
    const int screenWidth = 1200;
    const int screenHeight = 720;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE); // Window configuration flags
    InitWindow(screenWidth, screenHeight, "simulation test");
    SetTraceLogLevel(LOG_ERROR);

    //Camera settings
    Camera3D camera = {0};
    camera.position = (Vector3){50.0f, 50.0f, 50.0f}; // Camera position
    camera.target = (Vector3){0.0f, 0.0f, 0.0f}; // Camera looking at point
    camera.up = (Vector3){0.0f, 1.0f, 0.0f}; // Camera up vector (rotation towards target)
    camera.fovy = 70.0f; // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;
    SetTargetFPS(60);
    //movement and such
    float speed = 0.05f;
    float movement_speed = 0.2f;
    int draw_level = 50;
    //interface
    bool is_gui_mode = false;
    SetExitKey(KEY_NULL);
    srand(static_cast<unsigned>(time(0)));
    HideCursor();
    std::vector<std::shared_ptr<sim::RigidBody> > bodies = {};
    std::mutex mtx;
    sim::SimulationGovernor governor = sim::SimulationGovernor(3.0f, 100000);
    governor.oct_tree = new dstruct::OctTree(Vector3{0, 0, 0}, 60);
    std::vector<sim::RigidBody *> bodies_{};
    float great_val{};
    bool is_cluster = true;
    bool draw_center_of_mass = false;
    bool draw_force = false;
    bool draw_planets = true;
    bool draw_grid=true;
    auto start = std::chrono::high_resolution_clock::now();


    //INTERESTING ONES
    /*{20, 20, -20},
            {20, 20, 20},
            {-20, 20, 20},
            {-20, 20, -20}*/


    if (is_cluster) {
        srand(time(nullptr));
        float great_x{}, great_y{}, great_z{};
        std::vector<Vector3> cluster_centers = {
            {-10, 10, -20},
            {20, 20, 20},
            {-20, 20, 20},
            {-20, -20, -20}
        };
        for (int i = 0; i < 10000; ++i) {
            // Select a random cluster center
            Vector3 center = cluster_centers[std::rand() % cluster_centers.size()];
            // Generate spherical coordinates
            float radius = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 5));
            float theta = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (2 * PI)));
            float phi = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / PI));
            // Convert spherical coordinates to Cartesian coordinates
            float x = center.x + radius * sin(phi) * cos(theta);
            float y = center.y + radius * sin(phi) * sin(theta);
            float z = center.z + radius * cos(phi);

            if (abs(x) > great_val) {
                great_val = abs(x);
            }
            if (abs(y) > great_val) {
                great_val = abs(y);
            }
            if (abs(z) > great_val) {
                great_val = abs(z);
            }
            if (abs(x) > great_x) great_x = abs(x);
            if (abs(y) > great_y) great_y = abs(y);
            if (abs(z) > great_z) great_z = abs(z);
            sim::RigidBody *body = new Planet(Vector3{x, y, z}, 100000.0f, Vector3{0.0001f, 0, 0}, 1, BLACK, "Earth",
                                              1);
            governor.oct_tree->add_body(body);
            governor.rigid_bodies.push_back(body);
        }
    } else {
        srand(time(nullptr));
        for (int i = 0; i < 10000; ++i) {
            float x, y, z;
            x = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            y = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            z = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            if (abs(x) > great_val) {
                great_val = abs(x);
            }
            if (abs(y) > great_val) {
                great_val = abs(y);
            }
            if (abs(z) > great_val) {
                great_val = abs(z);
            }
            sim::RigidBody *body = new Planet(Vector3{x, y, z}, 10000, Vector3{0.000, 0, 0}, 1, BLACK, "Earth", 1);
            governor.oct_tree->add_body(body);
            governor.rigid_bodies.push_back(body);
        }
        sim::RigidBody *body = new Planet(Vector3{0, 0, 0.1}, 100000000, Vector3{0.000, 0, 0}, 1, YELLOW, "SUN", 1);
        governor.oct_tree->add_body(body);
        governor.rigid_bodies.push_back(body);
        sim::RigidBody *body2 = new Planet(Vector3{30, 0, 0}, 1000, Vector3{0.000, 0.005, 0}, 1, BLACK, "Earth", 1);
        governor.oct_tree->add_body(body2);
        governor.rigid_bodies.push_back(body2);
        sim::RigidBody *body3 = new Planet(Vector3{-30, 0, 0}, 1000, Vector3{0.000, 0.010, 0.001}, 1, BLUE, "Earth", 1);
        governor.oct_tree->add_body(body3);
        governor.rigid_bodies.push_back(body3);
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::nanoseconds>(stop - start);
    // To get the value of duration use the count()
    // member function on the duration object
    std::cout << "generating octree took on average:" << duration.count() << " ns and " << duration_cast<
        std::chrono::milliseconds>(stop - start).count() << "ms" << std::endl;
    bool is_finished = false;
    auto thread_ = std::thread([&]() {
        constexpr int MAX = 10;
        for (int i = 0; i < MAX; ++i) {
            float x, y, z;
            x = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            y = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            z = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            if (abs(x) > great_val) {
                great_val = abs(x);
            }
            if (abs(y) > great_val) {
                great_val = abs(y);
            }
            if (abs(z) > great_val) {
                great_val = abs(z);
            }
            governor.oct_tree->size = 60;
        }
        is_finished = true;
    });
    std::cout << sizeof(governor.oct_tree);

    //--------------------------------------------------------------------------------------

    std::cout << GetMonitorRefreshRate(0) << std::endl;
    Vector3 camera_position = {0, 0, 0};

    governor.start_simulation(mtx);
    auto frame_start = std::chrono::high_resolution_clock::now();
    auto frame_end = std::chrono::high_resolution_clock::now();

    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        frame_start = std::chrono::high_resolution_clock::now();
        //GUI state control
        if (!is_gui_mode) {
            SetWindowFocused();
        }
        if (!IsWindowFocused) {
            is_gui_mode = true;
        }
        if (IsKeyPressed(KEY_LEFT_ALT)) {
            if (is_gui_mode) {
                HideCursor();
            } else {
                ShowCursor();
            }
            is_gui_mode = !is_gui_mode;
        }


        Vector3 camera_change = {0, 0, 0};
        if (IsKeyDown(KEY_W)) {
            camera_change.z = 1;
        }
        if (IsKeyDown(KEY_S)) {
            camera_change.z = 1;
        }
        if (IsKeyDown(KEY_A)) {
            camera_change.x = 1;
        }
        if (IsKeyDown(KEY_D)) {
            camera_change.x = 1;
        }
        if (IsKeyDown(KEY_SPACE)) {
            camera_change.y = 1;
        }
        IsKeyDown(KEY_W);

        //HideCursor();

        Vector3 camera_move = {0, 0, 0};
        if (!is_gui_mode) {
            camera_move = {
                GetMouseDelta().x * speed, // Rotation: yaw
                GetMouseDelta().y * speed, // Rotation: pitch
                0.0f // Rotation: roll
            };
            SetMousePosition(GetRenderWidth() / 2, GetRenderHeight() / 2);
        }
        ClearBackground(WHITE);
        Vector3Scale(camera_change, speed);
        camera_position = Vector3Add(camera_position, camera_change);
        UpdateCameraPro(&camera,
                        (Vector3){
                            (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) * movement_speed - // Move forward-backward
                            (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) * movement_speed,
                            (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) * movement_speed - // Move right-left
                            (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) * movement_speed,
                            -(IsKeyDown(KEY_C)) * movement_speed + (IsKeyDown(KEY_SPACE)) * movement_speed
                            // Move up-down
                        }, camera_move,
                        GetMouseWheelMove() * 2.0f);

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();


        BeginMode3D(camera);

        ///
        ///
        /// DRAWING/RENDERING
        ///
        ///
        if (is_finished) {
            std::lock_guard<std::mutex> lock(mtx); {
                if (draw_planets){
                    for (const auto &rigid_body: governor.rigid_bodies) {
                        try {
                            auto *we = dynamic_cast<Planet *>(rigid_body);
                            if (draw_force) {
                                Vector3 normalized_force = Vector3Normalize(we->force);
                                DrawLine3D(we->position, normalized_force * FORCE_MULTIPLIER + we->position,RED);
                            }
                            DrawCubeWires(we->position, .5f, .5f, .5f,BLACK);
                        } catch (error_t) {
                            std::cout << "Error" << std::endl;
                        }
                    }
                }
            }
            if (draw_grid) {
                DrawGrid(500, 1.0);
            }
            draw_structure_iterative(*governor.oct_tree, 0, governor.oct_tree->size * 2, draw_level,
                                     draw_center_of_mass);
            auto stop = std::chrono::high_resolution_clock::now();
        }
        EndMode3D();

        Color color = {255, 0, 0, 255};
        {
            GuiSetStyle(TEXTBOX, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
            GuiPanel((Rectangle){10, 10, 250, static_cast<float>(GetRenderHeight()) - 100}, "Panel 1");
            GuiCheckBox((Rectangle){25, 60, 15, 15}, "Is running", &governor.is_running);

            GuiGroupBox((Rectangle){25, 90, 125, 130}, "Simulation controls");
            //GuiLock();
            GuiSetState(STATE_NORMAL);
            if (GuiButton((Rectangle){30, 100, 115, 30}, "Pause simulation")) { governor.pause_simulation(); }
            GuiSetState(STATE_NORMAL);
            if (GuiButton((Rectangle){30, 140, 115, 30}, "Start simulation")) { governor.start_simulation(mtx); }
            GuiSetState(STATE_NORMAL);
            if (GuiButton((Rectangle){30, 180, 115, 30}, "Save simulation")) { governor.save_simulation(mtx); }
            //GuiSetState(STATE_NORMAL); if (GuiButton((Rectangle){ 30, 190, 115, 30 }, "Save simulation")) {governor.load_simulation();}
            //GuiUnlock();
            //DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);
            GuiGroupBox((Rectangle){25, 230, 200, 130}, "Camera controls");
            GuiLabel((Rectangle){30, 270, 150, 20}, "Camera sensitivity:");
            GuiSlider((Rectangle){50, 290, 80, 20}, "", TextFormat("%2.2f", speed), &speed, 0.0001, 1);
            GuiLabel((Rectangle){30, 310, 150, 20}, "Movement speed:");
            GuiSlider((Rectangle){50, 330, 80, 20}, "", TextFormat("%2.2f", movement_speed), &movement_speed, 0.001, 4);
            GuiGroupBox((Rectangle){25, 370, 200, 150}, "Debug controls");
            GuiLabel((Rectangle){30, 380, 150, 20}, "Node render");
            float a = draw_level;
            GuiSlider((Rectangle){50, 410, 80, 20}, "", TextFormat("", draw_level), &a, 0, 50);
            GuiCheckBox((Rectangle){50, 440, 15, 15}, "Draw center of mass", &draw_center_of_mass);
            GuiCheckBox((Rectangle){50, 460, 15, 15}, "Draw forces", &draw_force);
            GuiCheckBox((Rectangle){50, 480, 15, 15}, "Draw planets", &draw_planets);
            GuiCheckBox((Rectangle){50, 500, 15, 15}, "Draw grid", &draw_grid);
            draw_level = a;
            //TODO here
            EndDrawing();
            frame_end = std::chrono::high_resolution_clock::now();
        }

        //----------------------------------------------------------------------------------
    }
    thread_.join();
    governor.pause_simulation();
    //--------------------------------------------------------------------------------------
    CloseWindow(); // Close window and OpenGL context
    //--------------------------------------------------------------------------------------
    return 0;
}
