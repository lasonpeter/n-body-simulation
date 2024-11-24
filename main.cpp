#define RAYGUI_IMPLEMENTATION

#include <iostream>

#include "raylib.h"

#include <raymath.h>
#include "simulation/planet.h"
#include "simulation/simulation_governor.h"
#include "raygui.h"
int main()
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);    // Window configuration flags
    InitWindow(screenWidth, screenHeight, "simulation test");

    //Camera settings
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 16.0f, 16.0f, 16.0f }; // Camera position
    //camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;    SetTargetFPS(60);
    //movement and such
    float speed=0.05f;
    float movement_speed=0.1f;
    //interface
    bool is_gui_mode=false;
    SetExitKey(KEY_NULL);
    HideCursor();
    std::vector<std::shared_ptr<sim::RigidBody>> bodies = {};
    std::mutex mtx;
    sim::SimulationGovernor governor = sim::SimulationGovernor(0.01f,1000);

        std::cout<<"Simulation not loaded"<<std::endl;
        /*Planet planet = new Planet(Vector3{3,0,0},10000,Vector3{0.00001,0,0},1,BLACK,"Earth",1);
        Planet planet2 = Planet(Vector3{-3,0,0},10,Vector3{-0.00001,0,0},1,RED,"Moon",2);*/

        bodies.push_back(std::make_shared<Planet>(Vector3{-3, 0, 0}, 1000, Vector3{-0.00004, 0, 0}, 1,RED, "Moon", 2));
        bodies.push_back(std::make_shared<Planet>(Vector3{3, 0, 0}, 10000, Vector3{0.00016, 0, 0}, 1,BLACK, "Earth", 1));
        bodies.push_back(std::make_shared<Planet>(Vector3{3, 0, 9}, 1000000, Vector3{-0.00001, 0, 0}, 1,YELLOW, "Sun", 2));
        bodies.push_back(std::make_shared<Planet>(Vector3{3, 2, 0}, 10000, Vector3{0.00002, 0.000005, 0}, 1,GREEN, "Yoo", 1));
        bodies.push_back(std::make_shared<Planet>(Vector3{3, -6, 0}, 1000, Vector3{-0.00001, 0, 0}, 1,VIOLET, "Eve", 2));
        bodies.push_back(std::make_shared<Planet>(Vector3{3, 0, 5}, 10000, Vector3{0.00032, 0, 0.00032}, 1,ORANGE, "Mars", 1));
        bodies.push_back(std::make_shared<Planet>(Vector3{-3, -3, 0}, 1000, Vector3{-0.00001, 0, 0}, 1,BLUE, "Neptune", 2));
        bodies.push_back(std::make_shared<Planet>(Vector3{1, -1, 5}, 10000, Vector3{0.00002, 0, 0}, 1,GRAY, "Pluto", 1));
        governor.setBodies(bodies);
        //governor.loadSimulation();
    try {
        governor.startSimulation(mtx);
    }
    catch (std::exception e) {
        std::cout<<e.what()<<std::endl;
    }
    //sim::RigidBody* yes = &planet;
    //Planet* z= dynamic_cast<Planet*>(yes);
    std::cout<<"WEEEEEEEe"<<std::endl;

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
        std::cout<<"WEEEEEEEe"<<std::endl;
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
        std::cout<<"WEEEEEEEe"<<std::endl;
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
        std::cout<<"WEEEEEEEe"<<std::endl;

        std::cout<<"WEEEEEEEe"<<std::endl;

        BeginMode3D(camera);
        Vector3 position = { 0.0f, 0.0f, 0.0f };
        //DrawModel(LoadModelFromMesh(GenMeshCube(5.0f,5.0f,5.0f)), position, 1.0f, WHITE);

        {
            std::lock_guard<std::mutex> lock(mtx);
            for (auto rigid_body: governor.bodies_) {
                try {
                    Planet* we =dynamic_cast<Planet*>(rigid_body.get());
                    DrawModel(LoadModelFromMesh(GenMeshSphere(1.0f,10.0f,10.0f)), we->position, 1.0f, we->color);
                    std::cout<<"model drawn"<<we->name<<std::endl;
                }catch (error_t) {
                    std::cout<<"Error"<<std::endl;
                }
            }
        }
        DrawGrid(500, 1.0);

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

        EndDrawing();

        //----------------------------------------------------------------------------------
    }
    governor.pauseSimulation();
    //governor.saveSimulation();
    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}