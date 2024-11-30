#define RAYGUI_IMPLEMENTATION

#include <iostream>

#include "raylib.h"

#include <raymath.h>
#include "simulation/planet.h"
#include "simulation/simulation_governor.h"
#include "data-structures/oct_tree.h"
#include "raygui.h"
#include <bits/stdc++.h>



const Color color_map[18] = {LIGHTGRAY,GRAY,DARKGRAY,BROWN,BLUE,PURPLE,ORANGE,RED,PINK,GRAY,DARKGRAY,MAROON,GOLD,LIME,DARKGREEN,DARKBLUE,DARKPURPLE};
constexpr  float dif=0.00f;

//convert this to use heap memory
void draw_structure_iterative(const dstruct::OctTree &octree_t, int depth, float whl, int draw_level) {
    std::stack<std::tuple<const dstruct::OctTree*, int, float>> stack;
    stack.push(std::make_tuple(&octree_t, depth, whl));

    while (!stack.empty()) {
        auto [node, current_depth, current_whl] = stack.top();
        stack.pop();

        if (current_depth >= draw_level) {
            DrawCubeWires(node->position, current_whl - dif, current_whl - dif, current_whl - dif, color_map[current_depth]);
        }

        if (!node->children.empty()) {
            for (const auto& child : node->children) {
                stack.push(std::make_tuple(child.get(), current_depth + 1, current_whl / 2));
            }
        }
    }
}

void draw_structure(const dstruct::OctTree &octree_t, int depth,float whl,int draw_level) {
    //auto start = std::chrono::high_resolution_clock::now();

    if(depth>=draw_level) {
        DrawCubeWires(octree_t.position, whl-dif, whl-dif, whl-dif,color_map[depth]);
    }
    //DrawCubeWires(octree_t.position, whl-dif, whl-dif, whl-dif,color_map[depth]);
    if(octree_t.children.empty()) {
        return;
    }
    for (const auto& child: octree_t.children) {
        //if()
        draw_structure(*child,depth+1,whl/2,draw_level);
    }
    //auto stop = std::chrono::high_resolution_clock::now();
    //auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
    //std::cout<<"Drawing octree took:"<<duration.count() << std::endl;
}

int main()
{
    const int screenWidth = 1200;
    const int screenHeight = 720;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);    // Window configuration flags
    InitWindow(screenWidth, screenHeight, "simulation test");
    SetTraceLogLevel(LOG_ERROR);

    //Camera settings
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 50.0f, 50.0f, 50.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 70.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;    SetTargetFPS(60);
    //movement and such
    float speed=0.05f;
    float movement_speed=0.2f;
    int draw_level=15;
    //interface
    bool is_gui_mode=false;
    SetExitKey(KEY_NULL);
    srand (static_cast <unsigned> (time(0)));
    //std::cout<<static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30<<std::endl;
    HideCursor();
    std::vector<std::shared_ptr<sim::RigidBody>> bodies = {};
    std::mutex mtx;
    sim::SimulationGovernor governor = sim::SimulationGovernor(0.01f,1000);
    governor.oct_tree = std::make_shared<dstruct::OctTree>(Vector3{0,0,0},1);

//        std::cout<<"Simulation not loaded"<<std::numeric_limits<size_t>::max()<<std::endl;
        /*Planet planet = new Planet(Vector3{3,0,0},10000,Vector3{0.00001,0,0},1,BLACK,"Earth",1);
        Planet planet2 = Planet(Vector3{-3,0,0},10,Vector3{-0.00001,0,0},1,RED,"Moon",2);*/

        //bodies.push_back(std::make_shared<Planet>(Vector3{-3, 0, 0}, 1000, Vector3{-0.00004, 0, 0},1,RED, "Moon", 2));
    float great_val{};
    bool is_cluster=true;
    if(is_cluster) {
        srand(time(nullptr));
        float great_x{}, great_y{}, great_z{};
        std::vector<Vector3> cluster_centers = {
            {-20, -20, -20},
            {20, 20, 20}
        };

        for (int i = 0; i < 100000; ++i) {
            // Select a random cluster center
            Vector3 center = cluster_centers[std::rand() % cluster_centers.size()];

            // Generate spherical coordinates
            float radius = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 10));
            float theta = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (2 * PI)));
            float phi = static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / PI));

            // Convert spherical coordinates to Cartesian coordinates
            float x = center.x + radius * sin(phi) * cos(theta);
            float y = center.y + radius * sin(phi) * sin(theta);
            float z = center.z + radius * cos(phi);

            if(abs(x) > great_val) {
                great_val = abs(x);
            }
            if(abs(y) > great_val) {
                great_val = abs(y);
            }
            if(abs(z) > great_val) {
                great_val = abs(z);
            }
            if (abs(x) > great_x) great_x = abs(x);
            if (abs(y) > great_y) great_y = abs(y);
            if (abs(z) > great_z) great_z = abs(z);

            governor.oct_tree->bodies_.push_back(std::make_shared<Planet>(Vector3{x, y, z}, 10000, Vector3{0.00016, 0, 0}, 1, BLACK, "Earth", 1));
        }
    }
    else {
        srand(time(nullptr));
        for (int i = 0; i < 100000; ++i){
            float x,y,z;
            x=static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            y=static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            z=static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            if(abs(x)>great_val) {
                great_val=abs(x);
            }
            if(abs(y)>great_val) {
                great_val=abs(y);
            }
            if(abs(z)>great_val) {
                great_val=abs(z);
            }
            governor.oct_tree->bodies_.push_back(std::make_shared<Planet>(Vector3{x,y,z}, 10000, Vector3{0.00016, 0, 0}, 1,BLACK, "Earth", 1));
            ////sleep for 1ms
        }
    }



    bool is_finished=false;
    auto thread_ = std::thread([&]() {
        constexpr int MAX=100;
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < MAX; ++i) {
            float x,y,z;
            x=static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            y=static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            z=static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / 60)) - 30;
            if(abs(x)>great_val) {
                great_val=abs(x);
            }
            if(abs(y)>great_val) {
                great_val=abs(y);
            }
            if(abs(z)>great_val) {
                great_val=abs(z);
            }
            governor.oct_tree->bodies_.push_back(std::make_shared<Planet>(Vector3{x,y,z}, 10000, Vector3{0.00016, 0, 0}, 1,BLACK, "Earth", 1));
            governor.oct_tree->size=great_val;
            dstruct::OctTree::GenerateDasOctree(governor.oct_tree);
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(4000));;
        //governor.oct_tree->generateOctree(great_x,great_y,great_z);
        //dstruct::OctTree::GenerateDasOctree(great_x,great_y,great_z,governor.oct_tree);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = duration_cast<std::chrono::nanoseconds>(stop - start);
        // To get the value of duration use the count()
        // member function on the duration object
        std::cout << "generating octree took on average:"<< duration.count()/MAX <<" ns and "<<duration_cast<std::chrono::milliseconds>(stop - start).count()/MAX<< "ms" << std::endl;
        is_finished=true;
    });
    std::cout<<sizeof(governor.oct_tree);


        //governor.setBodies(bodies);
        //governor.loadSimulation();
    try {
        //governor.startSimulation(mtx);
    }
    catch (std::exception e) {
        std::cout<<e.what()<<std::endl;
    }
    //sim::RigidBody* yes = &planet;
    //Planet* z= dynamic_cast<Planet*>(yes);

    //--------------------------------------------------------------------------------------
    std::cout<<GetMonitorRefreshRate(0)<<std::endl;
    Vector3 camera_position = {0,0,0};
    //float speed=0.1f;
    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        std::cout<<"IS IN GUI MODE: "<<is_gui_mode<<std::endl;
        //GUI state control
        if(!is_gui_mode) {
            SetWindowFocused();
        }
        if(!IsWindowFocused) {
            is_gui_mode=true;
        }
        if(IsKeyPressed(KEY_LEFT_ALT) ) {
            if(is_gui_mode) {
                HideCursor();
            }else {
                ShowCursor();
            }
            is_gui_mode =!is_gui_mode;
        }


        Vector3 camera_change={0,0,0};
        if(IsKeyDown(KEY_W)){
            camera_change.z=1;
        }
        if(IsKeyDown(KEY_S)){
            camera_change.z=1;
        }
        if(IsKeyDown(KEY_A)){
            camera_change.x=1;
        }
        if(IsKeyDown(KEY_D)){
            camera_change.x=1;
        }
        if(IsKeyDown(KEY_SPACE)){
            camera_change.y=1;
        }
        IsKeyDown(KEY_W);

        //HideCursor();

        Vector3 camera_move={0,0,0};
        if(!is_gui_mode) {
            camera_move={
                GetMouseDelta().x*speed,                            // Rotation: yaw
                GetMouseDelta().y*speed,                            // Rotation: pitch
                0.0f                                                // Rotation: roll
                };
            SetMousePosition(GetRenderWidth()/2,GetRenderHeight()/2);
        }
        ClearBackground(RAYWHITE);
        Vector3Scale(camera_change,speed);
        camera_position=Vector3Add(camera_position,camera_change);
        UpdateCameraPro(&camera,
            (Vector3){
                (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))*movement_speed -      // Move forward-backward
                (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))*movement_speed,
                (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT))*movement_speed -   // Move right-left
                (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))*movement_speed,
                -(IsKeyDown(KEY_C))*movement_speed +(IsKeyDown(KEY_SPACE))*movement_speed                                                  // Move up-down
            },camera_move,
            GetMouseWheelMove()*2.0f);

        // Update
        //----------------------------------------------------------------------------------
        // TODO: Update your variables here
        //----------------------------------------------------------------------------------
        Mesh mesh = GenMeshPlane(2, 2, 1, 1);
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        /*auto bodiz= governor.getBodies();
        //auto a= dynamic_cast<Planet*>(bodiz[0]);*/




        BeginMode3D(camera);
        //DrawCubeWires((Vector3){0,0,0},60.0,60.0,60.0,YELLOW);
        Vector3 position = { 0.0f, 0.0f, 0.0f };
        //DrawModel(LoadModelFromMesh(GenMeshCube(5.0f,5.0f,5.0f)), position, 1.0f, WHITE);

        if(is_finished)
            {
            {
                auto start = std::chrono::high_resolution_clock::now();
                std::lock_guard<std::mutex> lock(mtx);
                for (const auto& rigid_body: governor.oct_tree->bodies_) {
                    try {
                        auto* we =dynamic_cast<Planet*>(rigid_body.get());
                        //DrawModel(LoadModelFromMesh(GenMeshSphere(.1f,3,3)), we->position, 1.0f, we->color);
                        DrawCubeWires(we->position, 0.01f, 0.01f, 0.01f, we->color);
                        //std::cout<<"model drawn"<<we->name<<std::endl;
                    }catch (error_t) {
                        std::cout<<"Error"<<std::endl;
                    }
                }
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
                //std::cout<<"Drawing took:"<<duration.count() << std::endl;
            }


            //DrawGrid(500, 1.0);
            //float greatest_val=std::max(great_x,std::max(great_y,great_z));
            auto start = std::chrono::high_resolution_clock::now();
            draw_structure_iterative(*governor.oct_tree,0,60,draw_level);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
            //std::cout<<"Drawing octree took:"<<duration.count() << std::endl;
        }
        EndMode3D();

        Color color = {255, 0, 0, 255};
        //GuiLoadStyle("styles/light.rgs");
        //GuiLoadStyleDark();


        GuiSetStyle(TEXTBOX, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
        GuiPanel((Rectangle){ 10, 10, 250, static_cast<float>(GetRenderHeight())-100 }, "Panel 1");
        GuiCheckBox((Rectangle){ 25, 60, 15, 15 }, "Is running", &governor.is_running);

        GuiGroupBox((Rectangle){ 25,90 , 125, 130 }, "Simulation controls");
        //GuiLock();
        GuiSetState(STATE_NORMAL); if (GuiButton((Rectangle){ 30, 100, 115, 30 }, "Pause simulation")) {governor.pauseSimulation();}
        GuiSetState(STATE_NORMAL); if (GuiButton((Rectangle){ 30, 140, 115, 30 }, "Start simulation")) {governor.startSimulation(mtx);}
        GuiSetState(STATE_NORMAL); if (GuiButton((Rectangle){ 30, 180, 115, 30 }, "Save simulation")) {governor.saveSimulation(mtx);}
        //GuiUnlock();
        //DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);
        GuiGroupBox((Rectangle){25,250,200,100},"Camera controls");
        GuiLabel((Rectangle){ 30, 270, 150, 20 }, "Camera sensitivity:");
        GuiSlider((Rectangle){ 50, 290, 80, 20 }, "", TextFormat("%2.2f", speed), &speed, 0.0001, 1);
        GuiLabel((Rectangle){ 30, 310, 150, 20 }, "Movement speed:");
        GuiSlider((Rectangle){ 50, 330, 80, 20 }, "", TextFormat("%2.2f", movement_speed), &movement_speed, 0.001, 4);
        GuiGroupBox((Rectangle){25,340,200,40},"Debug controls");
        GuiLabel((Rectangle){ 30, 350, 150, 20 }, "Node render");
        float a=draw_level;
        GuiSlider((Rectangle){ 50, 370, 80, 20 }, "", TextFormat("", draw_level), &a, 0, 15);
        draw_level=a;
        //TODO here
        EndDrawing();

        //----------------------------------------------------------------------------------
    }
    thread_.join();
    governor.pauseSimulation();
    //governor.saveSimulation();
    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------
    return 0;
}